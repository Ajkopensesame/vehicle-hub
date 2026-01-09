import time
from collections import defaultdict

START_TS = time.monotonic()

def now_ms():
    return int((time.monotonic() - START_TS) * 1000)


class VehicleStateBuilder:
    def __init__(self):
        self.seq = 0
        self.last_rx_ms = None
        self.last_frame = {}

        # EMA smoothing state per analog channel
        self.analog_state = defaultdict(lambda: {"smooth": None})

    def update_from_uno(self, frame: dict):
        """Call this on every valid UNO frame"""
        self.last_rx_ms = now_ms()
        self.seq = frame.get("seq", self.seq)
        self.last_frame = frame

    def _smooth(self, key, raw, alpha=0.15):
        state = self.analog_state[key]
        if state["smooth"] is None:
            state["smooth"] = raw
        else:
            state["smooth"] = (alpha * raw) + ((1 - alpha) * state["smooth"])
        return state["smooth"]

    def build(self):
        ts = now_ms()

        stale = (
            self.last_rx_ms is None or
            (ts - self.last_rx_ms) > 750
        )

        frame = self.last_frame

        inputs = frame.get("inputs", {})
        analog = frame.get("analog", {})

        def d(pin):
            return bool(inputs.get(pin, False))

        def a(name):
            raw = analog.get(name)
            if raw is None:
                return {"raw": None, "smooth": None, "norm": None}
            smooth = self._smooth(name, raw)
            return {
                "raw": raw,
                "smooth": round(smooth, 2),
                "norm": round(raw / 1023.0, 4),
            }

        return {
            "type": "vehicle_state",
            "ts_ms": ts,
            "source": "bbb_vehicle_hub",
            "seq": self.seq,
            "uptime_ms": frame.get("uptime_ms", 0),
            "heartbeat": frame.get("heartbeat", 0),

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
                "stale": stale
            }
        }
