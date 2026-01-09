from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class HoldLatch:
    """
    Digital stabilizer that:
    - requires a state to remain stable for `min_stable_ms` before accepting
    - optionally holds ON for `hold_on_ms` to ignore brief dropouts
    """
    min_stable_ms: int = 30
    hold_on_ms: int = 0

    _raw_last: bool = False
    _raw_last_change_ms: int = 0
    _stable: bool = False
    _stable_since_ms: int = 0
    _hold_until_ms: int = 0

    def update(self, raw: bool, now_ms: int) -> bool:
        if raw != self._raw_last:
            self._raw_last = raw
            self._raw_last_change_ms = now_ms

        # If raw has been steady long enough, accept as new stable
        if (now_ms - self._raw_last_change_ms) >= self.min_stable_ms:
            if self._stable != raw:
                self._stable = raw
                self._stable_since_ms = now_ms
                if self._stable and self.hold_on_ms > 0:
                    self._hold_until_ms = now_ms + self.hold_on_ms

        # Hold ON to ignore brief dropout
        if self.hold_on_ms > 0 and self._stable is False and now_ms < self._hold_until_ms:
            return True

        return self._stable


@dataclass
class FlashDetector:
    """
    Detects 'flashing' for indicators based on recent toggles.
    If toggles >= `min_toggles` within `window_ms`, it's considered flashing.
    """
    window_ms: int = 1200
    min_toggles: int = 2

    _last: bool = False
    _toggle_times_ms: list[int] = None

    def __post_init__(self) -> None:
        if self._toggle_times_ms is None:
            self._toggle_times_ms = []

    def update(self, value: bool, now_ms: int) -> bool:
        if value != self._last:
            self._last = value
            self._toggle_times_ms.append(now_ms)

        cutoff = now_ms - self.window_ms
        # keep only toggles within window
        self._toggle_times_ms = [t for t in self._toggle_times_ms if t >= cutoff]
        return len(self._toggle_times_ms) >= self.min_toggles


@dataclass
class EMA:
    """
    Exponential moving average for analog smoothing.
    alpha in (0..1]: higher = follows input more closely.
    """
    alpha: float = 0.25
    _has: bool = False
    _v: float = 0.0

    def update(self, x: float) -> float:
        if not self._has:
            self._has = True
            self._v = float(x)
            return self._v
        self._v = self.alpha * float(x) + (1.0 - self.alpha) * self._v
        return self._v


@dataclass
class OutlierClamp:
    """
    Prevent sudden unrealistic jumps from dominating:
    clamps per-sample delta to +/- max_step.
    """
    max_step: float = 80.0
    _has: bool = False
    _last: float = 0.0

    def update(self, x: float) -> float:
        x = float(x)
        if not self._has:
            self._has = True
            self._last = x
            return x
        delta = x - self._last
        if delta > self.max_step:
            x = self._last + self.max_step
        elif delta < -self.max_step:
            x = self._last - self.max_step
        self._last = x
        return x
