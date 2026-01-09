from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from .mapping import PinMap, normalize_analog
from .stabilizers import HoldLatch, FlashDetector, EMA, OutlierClamp


@dataclass
class VehicleStateTransformer:
    pinmap: PinMap

    # Stabilisers per digital signal
    _dig_stab: dict[str, HoldLatch] = None
    _flash: dict[str, FlashDetector] = None

    # Analog stabilisers
    _analog_clamp: dict[str, OutlierClamp] = None
    _analog_ema: dict[str, EMA] = None

    # Stale detection
    stale_timeout_ms: int = 1500
    _last_frame_ms: int = 0

    def __post_init__(self) -> None:
        self._dig_stab = {}
        self._flash = {}

        # sensible defaults:
        # - 30ms stable requirement
        # - indicators often have brief chattering edges; hold ON for 80ms
        for d_pin, name in self.pinmap.digital.items():
            hold = 80 if name in ("left_indicator", "right_indicator") else 0
            self._dig_stab[name] = HoldLatch(min_stable_ms=30, hold_on_ms=hold)

            if name in ("left_indicator", "right_indicator"):
                self._flash[name] = FlashDetector(window_ms=1200, min_toggles=2)

        self._analog_clamp = {}
        self._analog_ema = {}
        for a_pin, amap in self.pinmap.analog.items():
            self._analog_clamp[amap.name] = OutlierClamp(max_step=120.0)
            self._analog_ema[amap.name] = EMA(alpha=0.25)

    def transform(self, frame: dict[str, Any]) -> dict[str, Any]:
        """
        Input: Arduino JSON frame {type:"vehicle_inputs", seq, uptime_ms, inputs:{D2..}, analog:{A0..}}
        Output: vehicle_state with stabilized digital + smoothed analog + metadata.
        """
        now_ms = int(time.time() * 1000)
        self._last_frame_ms = now_ms

        raw_inputs: dict[str, bool] = dict(frame.get("inputs", {}) or {})
        raw_analog: dict[str, Any] = dict(frame.get("analog", {}) or {})

        # DIGITAL -> named, stabilized
        named_digital: dict[str, bool] = {}
        flashing: dict[str, bool] = {}

        for d_pin, name in self.pinmap.digital.items():
            raw_val = bool(raw_inputs.get(d_pin, False))
            stable = self._dig_stab[name].update(raw_val, now_ms)
            named_digital[name] = stable

            if name in self._flash:
                flashing[name] = self._flash[name].update(stable, now_ms)

        # ANALOG -> clamp + ema + normalize
        analog_out: dict[str, Any] = {}
        for a_pin, amap in self.pinmap.analog.items():
            raw = raw_analog.get(a_pin, None)
            if raw is None:
                continue

            try:
                raw_f = float(raw)
            except Exception:
                continue

            clamped = self._analog_clamp[amap.name].update(raw_f)
            smooth = self._analog_ema[amap.name].update(clamped)
            norm = normalize_analog(smooth, amap)

            analog_out[amap.name] = {
                "raw": int(raw_f),
                "smooth": float(round(smooth, 2)),
                "norm": float(round(norm, 4))
            }

        # Build a “vehicle_state” contract that BeagleY can consume directly
        state: dict[str, Any] = {
            "type": "vehicle_state",
            "ts_ms": now_ms,
            "source": "bbb_vehicle_hub",
            "seq": int(frame.get("seq", 0)),
            "uptime_ms": int(frame.get("uptime_ms", 0)),
            "heartbeat": int(frame.get("heartbeat", 0)),

            "indicators": {
                "left": named_digital.get("left_indicator", False),
                "right": named_digital.get("right_indicator", False),
                "left_flashing": flashing.get("left_indicator", False),
                "right_flashing": flashing.get("right_indicator", False),
                "high_beam": named_digital.get("high_beam", False),
            },

            "warnings": {
                "brake": named_digital.get("brake_warning", False),
                "oil": named_digital.get("oil_pressure", False),
                "charge": named_digital.get("charge_lamp", False),
                "door": named_digital.get("door_ajar", False),
            },

            "spares": {
                "spare_1": named_digital.get("spare_1", False),
            },

            "analog": analog_out,
            "_health": {
                "stale": False
            }
        }

        return state

    def health_state(self) -> dict[str, Any]:
        """
        Emits a minimal health state if serial stalls (so BeagleY can show 'No Data').
        """
        now_ms = int(time.time() * 1000)
        stale = (now_ms - self._last_frame_ms) > self.stale_timeout_ms if self._last_frame_ms else True

        return {
            "type": "vehicle_state",
            "ts_ms": now_ms,
            "source": "bbb_vehicle_hub",
            "_health": {
                "stale": bool(stale)
            }
        }
