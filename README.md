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
