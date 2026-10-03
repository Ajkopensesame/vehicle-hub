"""Sensor conversion for the vehicle hub (Phase 4).

Turns raw UNO readings into the engineering values on the wire:
fuelPct (0..100), coolantC (deg C), speedKph, rpm.

Calibration lives in a JSON file (default ``src/config/sensor_calibration.json``,
override with ``VEHICLE_HUB_CALIBRATION``). Until a table has points, that
signal stays at the stub value ``0.0`` -- the hub never invents numbers.

Wire format is unchanged: only the *values* of existing keys change.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger("vehicle-hub.sensors")

# ADC counts at/below FAULT_LOW or at/above FAULT_HIGH mean an open/shorted sender.
DEFAULT_FAULT_LOW = 5
DEFAULT_FAULT_HIGH = 1018


def piecewise_linear(x: float, points: list[tuple[float, float]]) -> float:
    """Interpolate x over (input, output) points; clamp to the end values."""
    pts = sorted(points)
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            if x1 == x0:
                return y1
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


@dataclass
class TableMap:
    """ADC counts -> engineering value via a calibration table."""
    name: str
    points: list[tuple[float, float]] = field(default_factory=list)
    out_min: float = float("-inf")
    out_max: float = float("inf")
    fault_low: int = DEFAULT_FAULT_LOW
    fault_high: int = DEFAULT_FAULT_HIGH
    _faulted: bool = False

    @property
    def calibrated(self) -> bool:
        return len(self.points) >= 2

    def convert(self, raw_smooth: Optional[float]) -> float:
        """Return the value, or 0.0 if uncalibrated / no reading / sender fault."""
        if raw_smooth is None or not self.calibrated:
            return 0.0
        if raw_smooth <= self.fault_low or raw_smooth >= self.fault_high:
            if not self._faulted:
                log.warning("%s sender fault (raw=%.0f: open or shorted); reporting 0.0", self.name, raw_smooth)
                self._faulted = True
            return 0.0
        self._faulted = False
        v = piecewise_linear(float(raw_smooth), self.points)
        return max(self.out_min, min(self.out_max, v))


@dataclass
class PulseMap:
    """Optional: UNO-supplied pulse frequency (Hz) -> unit, via a fixed scale."""
    name: str
    hz_key: str
    units_per_hz: float = 0.0   # 0 => disabled
    max_value: float = float("inf")

    def convert(self, frame: dict[str, Any]) -> float:
        if self.units_per_hz <= 0:
            return 0.0
        try:
            hz = float(frame.get(self.hz_key))
        except (TypeError, ValueError):
            return 0.0
        if hz < 0:
            return 0.0
        return min(hz * self.units_per_hz, self.max_value)


@dataclass
class SensorMapper:
    fuel: TableMap
    coolant: TableMap
    speed: PulseMap
    rpm: PulseMap
    fuel_low_pct: float = 10.0

    def compute(self, frame: dict[str, Any], smooth: dict[str, Optional[float]]) -> dict[str, Any]:
        """smooth: smoothed raw ADC by analog pin name (e.g. 'A0')."""
        fuel = self.fuel.convert(smooth.get("A0"))
        return {
            "speedKph": round(self.speed.convert(frame), 1),
            "rpm": round(self.rpm.convert(frame), 0),
            "fuelPct": round(fuel, 1),
            "coolantC": round(self.coolant.convert(smooth.get("A1")), 1),
            "fuel_low": bool(self.fuel.calibrated and fuel <= self.fuel_low_pct
                             and smooth.get("A0") is not None
                             and self.fuel.fault_low < smooth["A0"] < self.fuel.fault_high),
        }


def _table(name: str, cfg: dict[str, Any], out_min: float, out_max: float) -> TableMap:
    pts = [(float(p[0]), float(p[1])) for p in (cfg.get("points") or [])]
    return TableMap(
        name=name,
        points=pts,
        out_min=out_min,
        out_max=out_max,
        fault_low=int(cfg.get("fault_low", DEFAULT_FAULT_LOW)),
        fault_high=int(cfg.get("fault_high", DEFAULT_FAULT_HIGH)),
    )


def mapper_from_config(data: dict[str, Any]) -> SensorMapper:
    fuel_cfg = data.get("fuel") or {}
    sp = data.get("speed") or {}
    rp = data.get("rpm") or {}
    return SensorMapper(
        fuel=_table("fuelPct", fuel_cfg, 0.0, 100.0),
        coolant=_table("coolantC", data.get("coolant") or {}, -40.0, 150.0),
        speed=PulseMap("speedKph", str(sp.get("hz_key", "speed_hz")),
                       float(sp.get("kph_per_hz", 0.0)), float(sp.get("max", 300.0))),
        rpm=PulseMap("rpm", str(rp.get("hz_key", "rpm_hz")),
                     float(rp.get("rpm_per_hz", 0.0)), float(rp.get("max", 9000.0))),
        fuel_low_pct=float(fuel_cfg.get("low_pct", 10.0)),
    )


def load_mapper(path: Optional[str]) -> SensorMapper:
    """Load calibration; any problem falls back to all-stubs (never raises)."""
    if path:
        try:
            data = json.loads(Path(path).read_text())
            m = mapper_from_config(data)
            log.info("sensor calibration %s: fuel=%s coolant=%s speed=%s rpm=%s", path,
                     "on" if m.fuel.calibrated else "stub", "on" if m.coolant.calibrated else "stub",
                     "on" if m.speed.units_per_hz > 0 else "stub", "on" if m.rpm.units_per_hz > 0 else "stub")
            return m
        except FileNotFoundError:
            log.warning("sensor calibration file not found: %s (all stubs)", path)
        except Exception as exc:  # bad json / values: never take the hub down
            log.error("sensor calibration %s unusable (%s); all stubs", path, exc)
    return mapper_from_config({})
