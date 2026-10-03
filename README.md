# vehicle-hub

BeagleBone Black WebSocket hub: reads Arduino `vehicle_inputs` over serial and broadcasts always-complete `vehicle_state` JSON to cluster clients (e.g. beagley-cluster).

## Quick start

```bash
cd src
python3 vehicle_hub.py
```

Dependencies: `pyserial`, `websockets` (Debian Bookworm-friendly).

Override bind / serial / rates via env — see [PROTOCOL.md](PROTOCOL.md).

## Protocol

Phase 1 locked flat camelCase contract is documented in [PROTOCOL.md](PROTOCOL.md).

Engineering fields (`speedKph`, `rpm`, `fuelPct`, `coolantC`) and drivetrain (`gear`, `overdrive`) are always present; values may be stubs until sensors/conversion exist.

## Sensor calibration (Phase 4)

`fuelPct` and `coolantC` are converted from UNO analog A0/A1 using piecewise-linear tables in `src/config/sensor_calibration.json` (`points` are `[raw_adc_counts, value]`, at least 2). Empty tables leave the value at `0.0`.

1. Run `python3 scripts/sample_raw.py ws://<hub>:8765 10` at known fuel levels / coolant temperatures and note the mean raw counts.
2. Put the pairs in the JSON file and restart the hub (hub restart only; the cluster service is not involved).

`speedKph` / `rpm` can come from UNO-supplied top-level `speed_hz` / `rpm_hz` pulse frequencies, scaled by `speed.kph_per_hz` / `rpm.rpm_per_hz` (0 = disabled). The current UNO firmware does not send these yet.

Rollback: set `VEHICLE_HUB_CALIBRATION=/nonexistent.json` (everything returns to the Phase 1 stubs) or revert the merge commit.

Tests: `python3 -m unittest discover -s tests` (needs `websockets` and `pyserial` installed).
