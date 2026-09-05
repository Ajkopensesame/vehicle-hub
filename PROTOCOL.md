# vehicle_state WebSocket protocol (Phase 1)

Producer: `vehicle-hub` on BBB. Consumer: `beagley-cluster` (`VehicleStateClient`).

## Transport

- WebSocket JSON text frames
- Default listen: `0.0.0.0:8765` (`VEHICLE_HUB_WS_HOST` / `VEHICLE_HUB_WS_PORT`)
- Client URL (cluster): `VEHICLE_HUB_WS_URL` (fallback `ws://192.168.0.7:8765` only if unset)

## Message types

### `hello` (once on connect)

```json
{ "type": "hello", "service": "vehicle_hub", "source": "bbb_vehicle_hub", "ts_ms": 0 }
```

Clients ignore non-`vehicle_state` messages.

### `vehicle_state` (broadcast ~10 Hz)

Always-complete flat camelCase frame. Example:

```json
{
  "type": "vehicle_state",
  "ts_ms": 0,
  "source": "bbb_vehicle_hub",
  "seq": 0,
  "speedKph": 0.0,
  "rpm": 0.0,
  "fuelPct": 0.0,
  "coolantC": 0.0,
  "gear": "P",
  "overdrive": false,
  "indicators": { "left": false, "right": false, "high_beam": false },
  "warnings": {
    "brake": false, "oil": false, "charge": false, "door": false,
    "check": false, "at": false, "fuel_low": false
  },
  "_health": { "stale": true }
}
```

### Units

| Field | Unit |
|-------|------|
| `speedKph` | km/h |
| `rpm` | RPM |
| `fuelPct` | 0–100 % |
| `coolantC` | °C |
| `gear` | `P` \| `R` \| `N` \| `D` \| `2` \| `1` |

Hub owns fuel%/coolant°C conversion (stubs `0.0` until mapped from analog senders).

### Good-frame rule (cluster)

A frame refreshes link health (`lastGoodRx`) only if it includes `type`, `_health.stale`, core indicators/warnings, **and** `speedKph` / `rpm` / `fuelPct` / `coolantC`. Those four keys must always be present (values may be `0`).

### Rates / stale

| Knob | Default | Env |
|------|---------|-----|
| Broadcast rate | 10 Hz | `VEHICLE_HUB_BROADCAST_HZ` |
| Hub UNO stale | 750 ms without `vehicle_inputs` → `_health.stale=true` | `VEHICLE_HUB_STALE_AFTER_MS` |
| Client link stale | >1000 ms without a good frame | (cluster) |

Optional passthrough (not required QML props): `uptime_ms`, `heartbeat`, `spares`, `analog`, flashing flags, `_health.last_rx_ms`.

## Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `VEHICLE_HUB_WS_HOST` | `0.0.0.0` | Bind address |
| `VEHICLE_HUB_WS_PORT` | `8765` | Bind port |
| `VEHICLE_HUB_SERIAL_PORT` | auto (`by-id` Arduino or `/dev/ttyACM0`) | UNO serial |
| `VEHICLE_HUB_SERIAL_BAUD` | `115200` | Baud |
| `VEHICLE_HUB_SERIAL_TIMEOUT_SEC` | `0.5` | Read timeout |
| `VEHICLE_HUB_BROADCAST_HZ` | `10` | WS push rate |
| `VEHICLE_HUB_STALE_AFTER_MS` | `750` | UNO stale |
| `VEHICLE_HUB_LOG_LEVEL` | `INFO` | Logging |

Do not rely on a hardcoded bench IP as the only configuration path.
