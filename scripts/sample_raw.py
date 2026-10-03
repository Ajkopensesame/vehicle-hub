#!/usr/bin/env python3
"""Print live raw/smoothed A0 (fuel) and A1 (coolant) from the hub for calibration.

Usage: python3 scripts/sample_raw.py [ws://10.24.0.7:8765] [seconds]
Read-only: connects as a normal WebSocket client. Write down 'smooth' at each
known fuel level / coolant temperature and put pairs in config/sensor_calibration.json
as [raw_counts, value].
"""
import asyncio, json, sys, statistics
import websockets

async def main(url: str, seconds: float) -> None:
    fuel, cool = [], []
    async with websockets.connect(url) as ws:
        end = asyncio.get_event_loop().time() + seconds
        while asyncio.get_event_loop().time() < end:
            st = json.loads(await ws.recv())
            if st.get("type") != "vehicle_state":
                continue
            an = st.get("analog", {})
            f = (an.get("fuel_sender_raw") or {}).get("raw")
            c = (an.get("coolant_sender_raw") or {}).get("raw")
            if f is not None: fuel.append(f)
            if c is not None: cool.append(c)
            print(f"A0 fuel raw={f} smooth={(an.get('fuel_sender_raw') or {}).get('smooth')}  "
                  f"A1 coolant raw={c} smooth={(an.get('coolant_sender_raw') or {}).get('smooth')}  "
                  f"stale={st['_health']['stale']}")
    for n, v in (("A0 fuel", fuel), ("A1 coolant", cool)):
        if v:
            print(f"{n}: n={len(v)} mean={statistics.mean(v):.1f} min={min(v)} max={max(v)}")

if __name__ == "__main__":
    asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "ws://10.24.0.7:8765",
                     float(sys.argv[2]) if len(sys.argv) > 2 else 10.0))
