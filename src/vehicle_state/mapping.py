from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class AnalogMap:
    name: str
    min: int = 0
    max: int = 1023
    invert: bool = False


@dataclass(frozen=True)
class PinMap:
    digital: dict[str, str]         # e.g., "D2" -> "left_indicator"
    analog: dict[str, AnalogMap]    # e.g., "A0" -> AnalogMap(...)


def load_pinmap(path: str) -> PinMap:
    p = Path(path)
    data = json.loads(p.read_text())

    digital = dict(data.get("digital", {}))
    analog_raw = dict(data.get("analog", {}))
    analog: dict[str, AnalogMap] = {}

    for k, v in analog_raw.items():
        analog[k] = AnalogMap(
            name=str(v.get("name", k)),
            min=int(v.get("min", 0)),
            max=int(v.get("max", 1023)),
            invert=bool(v.get("invert", False)),
        )

    return PinMap(digital=digital, analog=analog)


def normalize_analog(raw: float, amap: AnalogMap) -> float:
    """
    Normalizes raw ADC value to 0..1 based on min/max.
    Keeps within bounds. Supports invert.
    """
    lo = float(amap.min)
    hi = float(amap.max)
    if hi <= lo:
        return 0.0

    x = float(raw)
    if x < lo:
        x = lo
    if x > hi:
        x = hi

    n = (x - lo) / (hi - lo)
    if amap.invert:
        n = 1.0 - n
    return max(0.0, min(1.0, n))
