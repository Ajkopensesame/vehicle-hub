# vehicle-hub

> **LEGACY.** The live BBB hub is `tools/bbb_hub/vehicle_hub_prod.py` in the
> [beagley-cluster](https://github.com/Ajkopensesame/beagley-cluster) repo, not this repository. The current
> contract is `tools/schema/vehicle_state_v1.md` there, and the live hub address is `ws://10.24.0.7:8765`
> (the `192.168.0.7` address below is from the old hotspot-era network). This file is kept for history
> and for the Phase 1 flat camelCase contract this repo implements.

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
