import json, os, sys, tempfile, unittest

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
sys.path.insert(0, SRC)

CAL = {
    "fuel": {"points": [[100, 0], [500, 50], [900, 100]], "low_pct": 10},
    "coolant": {"points": [[900, 20], [500, 60], [200, 120]]},
    "speed": {"hz_key": "speed_hz", "kph_per_hz": 0.5, "max": 300},
    "rpm": {"hz_key": "rpm_hz", "rpm_per_hz": 30.0, "max": 9000},
}
GAUGE_KEYS = {"speedKph", "rpm", "fuelPct", "coolantC"}


def make_builder(cal):
    if cal is None:
        os.environ["VEHICLE_HUB_CALIBRATION"] = "/nonexistent.json"
    else:
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(cal, f); f.close()
        os.environ["VEHICLE_HUB_CALIBRATION"] = f.name
    sys.modules.pop("vehicle_hub", None)
    import vehicle_hub
    return vehicle_hub.VehicleStateBuilder()


def frame(a0=None, a1=None, **extra):
    analog = {}
    if a0 is not None: analog["A0"] = a0
    if a1 is not None: analog["A1"] = a1
    f = {"type": "vehicle_inputs", "seq": 1, "inputs": {}, "analog": analog}
    f.update(extra)
    return f


class SensorTests(unittest.TestCase):
    def test_uncalibrated_stays_stub_and_schema_complete(self):
        b = make_builder(None)
        b.update_from_uno(frame(500, 500))
        s = b.build()
        self.assertTrue(GAUGE_KEYS <= set(s))
        for k in GAUGE_KEYS:
            self.assertEqual(s[k], 0.0)

    def test_fuel_and_coolant_interpolate(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(500, 500))
        s = b.build()
        self.assertAlmostEqual(s["fuelPct"], 50.0, places=1)
        self.assertAlmostEqual(s["coolantC"], 60.0, places=1)
        self.assertFalse(s["warnings"]["fuel_low"])

    def test_clamped_and_fuel_low(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(120, 300))
        s = b.build()
        self.assertAlmostEqual(s["fuelPct"], 2.5, places=1)
        self.assertTrue(s["warnings"]["fuel_low"])
        self.assertTrue(0 <= s["fuelPct"] <= 100)

    def test_sender_fault_reports_zero(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(1023, 2))
        s = b.build()
        self.assertEqual(s["fuelPct"], 0.0)
        self.assertEqual(s["coolantC"], 0.0)
        self.assertFalse(s["warnings"]["fuel_low"])

    def test_pulse_speed_rpm(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(500, 500, speed_hz=100.0, rpm_hz=50.0))
        s = b.build()
        self.assertEqual(s["speedKph"], 50.0)
        self.assertEqual(s["rpm"], 1500.0)

    def test_missing_pulse_fields_are_zero(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(500, 500))
        s = b.build()
        self.assertEqual(s["speedKph"], 0.0)
        self.assertEqual(s["rpm"], 0.0)

    def test_stale_zeroes_engineering_values(self):
        import vehicle_hub
        b = make_builder(CAL)
        b.update_from_uno(frame(500, 500, speed_hz=100.0))
        b.last_rx_ms -= vehicle_hub.STALE_AFTER_MS + 500
        s = b.build()
        self.assertTrue(s["_health"]["stale"])
        for k in GAUGE_KEYS:
            self.assertEqual(s[k], 0.0)

    def test_bad_calibration_file_falls_back(self):
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        f.write("{not json"); f.close()
        os.environ["VEHICLE_HUB_CALIBRATION"] = f.name
        sys.modules.pop("vehicle_hub", None)
        import vehicle_hub
        b = vehicle_hub.VehicleStateBuilder()
        b.update_from_uno(frame(500, 500))
        self.assertEqual(b.build()["fuelPct"], 0.0)

    def test_wire_keys_unchanged(self):
        b = make_builder(CAL)
        b.update_from_uno(frame(500, 500))
        s = b.build()
        for k in ("type", "ts_ms", "source", "seq", "gear", "overdrive", "indicators", "warnings", "_health", "analog"):
            self.assertIn(k, s)
        for k in ("check", "at", "fuel_low", "brake", "oil", "charge", "door"):
            self.assertIn(k, s["warnings"])


if __name__ == "__main__":
    unittest.main()
