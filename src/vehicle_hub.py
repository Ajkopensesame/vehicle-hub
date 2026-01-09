#!/usr/bin/env python3
"""
Pajero Vehicle Hub (BBB)
- Reads newline-delimited JSON frames from Arduino Uno (Serial)
- Builds a stable "vehicle_state" object (always complete schema)
- Serves it over WebSocket to multiple clients (e.g., BeagleY + debug tools)

Design goals:
- Never crash on missing/partial data
- Never emit partial vehicle_state (prevents KeyError in clients)
- Keep running even if serial drops out
- Legacy-friendly websockets API (Debian Bookworm)
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Dict, Optional

import serial  # pyserial
import websockets


# ----------------------------
# Logging
# ----------------------------
LOG_LEVEL = os.getenv("VEHICLE_HUB_LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=LOG_LEVEL,
    format="[%(levelname)s] %(message)s",
)
log = logging.getLogger("vehicle-hub")


# ----------------------------
# Runtime config
# ----------------------------
WS_HOST = os.getenv("VEHICLE_HUB_WS_HOST", "0.0.0.0")
WS_PORT = int(os.getenv("VEHICLE_HUB_WS_PORT", "8765"))

SERIAL_BAUD = int(os.getenv("VEHICLE_HUB_SERIAL_BAUD", "115200"))
SERIAL_TIMEOUT_SEC = float(os.getenv("VEHICLE_HUB_SERIAL_TIMEOUT_SEC", "0.5"))

# How often we broadcast vehicle_state (Hz)
BROADCAST_HZ = float(os.getenv("VEHICLE_HUB_BROADCAST_HZ", "10"))
BROADCAST_INTERVAL = 1.0 / max(BROADCAST_HZ, 1.0)

# How long without valid UNO frames until we mark stale
STALE_AFTER_MS = int(os.getenv("VEHICLE_HUB_STALE_AFTER_MS", "750"))

# Prefer stable by-id path if present
DEFAULT_BY_ID_GLOB = "/dev/serial/by-id"
DEFAULT_FALLBACK_PORT = "/dev/ttyACM0"
SERIAL_PORT = os.getenv("VEHICLE_HUB_SERIAL_PORT", "").strip()


def _pick_serial_port() -> str:
    if SERIAL_PORT:
        return SERIAL_PORT

    try:
        if os.path.isdir(DEFAULT_BY_ID_GLOB):
            entries = sorted(os.listdir(DEFAULT_BY_ID_GLOB))
            for name in entries:
                # Your Arduino shows up like: usb-Arduino__www.arduino.cc__0043_...-if00
                if "Arduino" in name or "arduino" in name:
                    path = os.path.join(DEFAULT_BY_ID_GLOB, name)
                    return path
            # if no Arduino match, still pick first entry (better than ttyACM0 in some cases)
            if entries:
                return os.path.join(DEFAULT_BY_ID_GLOB, entries[0])
    except Exception:
        pass

    return DEFAULT_FALLBACK_PORT


# ----------------------------
# Time helper (monotonic ms)
# ----------------------------
START_TS = time.monotonic()


def now_ms() -> int:
    return int((time.monotonic() - START_TS) * 1000)


# ----------------------------
# VehicleState Builder (always complete schema)
# ----------------------------
class VehicleStateBuilder:
    def __init__(self) -> None:
        self.seq: int = 0
        self.last_rx_ms: Optional[int] = None
        self.last_frame: Dict[str, Any] = {}
        self.analog_state = defaultdict(lambda: {"smooth": None})

    def update_from_uno(self, frame: dict) -> None:
        """Call on every valid UNO vehicle_inputs frame."""
        self.last_rx_ms = now_ms()
        if isinstance(frame.get("seq"), int):
            self.seq = frame["seq"]
        self.last_frame = frame

    def _smooth(self, key: str, raw: float, alpha: float = 0.15) -> float:
        state = self.analog_state[key]
        if state["smooth"] is None:
            state["smooth"] = raw
        else:
            state["smooth"] = (alpha * raw) + ((1.0 - alpha) * state["smooth"])
        return float(state["smooth"])

    def build(self) -> Dict[str, Any]:
        ts = now_ms()

        stale = (
            self.last_rx_ms is None
            or (ts - self.last_rx_ms) > STALE_AFTER_MS
        )

        frame = self.last_frame or {}
        inputs = frame.get("inputs") or {}
        analog = frame.get("analog") or {}

        def d(pin: str) -> bool:
            # Always return a bool, even if missing
            return bool(inputs.get(pin, False))

        def a(name: str) -> Dict[str, Any]:
            raw = analog.get(name, None)
            if raw is None:
                return {"raw": None, "smooth": None, "norm": None}
            try:
                raw_i = int(raw)
            except Exception:
                return {"raw": None, "smooth": None, "norm": None}

            smooth = self._smooth(name, raw_i)
            return {
                "raw": raw_i,
                "smooth": round(smooth, 2),
                "norm": round(raw_i / 1023.0, 4),
            }

        # IMPORTANT: Always include full schema (no partials)
        return {
            "type": "vehicle_state",
            "ts_ms": ts,
            "source": "bbb_vehicle_hub",
            "seq": self.seq,
            "uptime_ms": int(frame.get("uptime_ms", 0) or 0),
            "heartbeat": int(frame.get("heartbeat", 0) or 0),

            "indicators": {
                "left": d("D2"),
                "right": d("D3"),
                "left_flashing": d("D2"),
                "right_flashing": d("D3"),
                "high_beam": d("D4"),
            },

            "warnings": {
                "brake": d("D5"),
                "oil": d("D6"),
                "charge": d("D7"),
                "door": d("D8"),
            },

            "spares": {
                "spare_1": d("D9"),
            },

            "analog": {
                "fuel_sender_raw": a("A0"),
                "coolant_sender_raw": a("A1"),
                "aux_analog_2": a("A2"),
                "aux_analog_3": a("A3"),
                "aux_analog_4": a("A4"),
                "aux_analog_5": a("A5"),
            },

            "_health": {
                "stale": stale,
                "last_rx_ms": self.last_rx_ms,
            },
        }


# ----------------------------
# Serial reader -> asyncio Queue
# ----------------------------
async def serial_reader_task(q: asyncio.Queue, builder: VehicleStateBuilder) -> None:
    port = _pick_serial_port()
    log.info(f"Opening serial {port} @ {SERIAL_BAUD}")

    backoff = 0.25
    while True:
        try:
            with serial.Serial(port, SERIAL_BAUD, timeout=SERIAL_TIMEOUT_SEC) as ser:
                log.info(f"Serial open OK ({port} @ {SERIAL_BAUD})")
                backoff = 0.25

                while True:
                    line = ser.readline()
                    if not line:
                        # timeout; just continue
                        await asyncio.sleep(0)
                        continue

                    # tolerate junk bytes
                    try:
                        s = line.decode("utf-8", errors="replace").strip()
                    except Exception:
                        continue

                    if not s:
                        continue

                    # parse JSON
                    try:
                        obj = json.loads(s)
                    except Exception:
                        # ignore malformed lines
                        continue

                    # We only update builder on vehicle_inputs frames
                    if obj.get("type") == "vehicle_inputs":
                        builder.update_from_uno(obj)

                    # queue raw frames too (optional: can be useful for debugging)
                    try:
                        q.put_nowait(obj)
                    except asyncio.QueueFull:
                        # drop oldest behavior (simple)
                        try:
                            _ = q.get_nowait()
                        except Exception:
                            pass
                        try:
                            q.put_nowait(obj)
                        except Exception:
                            pass

        except Exception as e:
            log.warning(f"Serial error: {e} (retrying)")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, 5.0)


# ----------------------------
# WebSocket server
# ----------------------------
@dataclass
class HubState:
    builder: VehicleStateBuilder
    latest_state: Dict[str, Any]


async def ws_handler(ws, path, hub: HubState) -> None:
    # Send hello immediately
    await ws.send(json.dumps({
        "type": "hello",
        "service": "vehicle_hub",
        "source": "bbb_vehicle_hub",
        "ts_ms": now_ms(),
    }))

    # Then continuously push latest vehicle_state at the broadcast rate
    try:
        while True:
            await ws.send(json.dumps(hub.latest_state))
            await asyncio.sleep(BROADCAST_INTERVAL)
    except Exception:
        # Client disconnected or network issue
        return


async def main() -> None:
    # Builder + shared hub state
    builder = VehicleStateBuilder()
    hub = HubState(builder=builder, latest_state=builder.build())

    # Serial queue (kept mainly for debugging / future expansion)
    q: asyncio.Queue = asyncio.Queue(maxsize=200)

    # Start serial reader
    asyncio.create_task(serial_reader_task(q, builder))

    # Background task updates latest_state (always complete schema)
    async def state_pump() -> None:
        while True:
            hub.latest_state = builder.build()
            await asyncio.sleep(BROADCAST_INTERVAL)

    asyncio.create_task(state_pump())

    # Start WebSocket server
    server = await websockets.serve(
        lambda ws, path: ws_handler(ws, path, hub),
        WS_HOST,
        WS_PORT,
    )

    log.info(f"server listening on {WS_HOST}:{WS_PORT}")
    log.info(f"WebSocket listening on ws://{WS_HOST}:{WS_PORT}")

    # Run forever
    await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
