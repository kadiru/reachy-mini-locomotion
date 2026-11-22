import unittest
from experiments.meter_compute import meter_window


class ComputeMeterTests(unittest.TestCase):
    def test_integrates_time_weighted_watts_with_clipped_boundaries(self):
        measured = meter_window([(0.,10.),(2.,14.),(4.,18.),(7.,24.)],1.,6.)
        self.assertEqual(measured['mean_w'],17.)
        self.assertAlmostEqual(measured['kwh'],85/3_600_000)
        self.assertEqual(measured['peak_w'],22.)

    def test_rejects_missing_boundaries_or_sensor_dropout(self):
        with self.assertRaises(ValueError):
            meter_window([(0.,5.),(20.,8.)],9.,10.)
        for samples in ([(1.,5.),(3.,9.)],[(0.,5.),(9.,10.)]):
            with self.assertRaises(ValueError):
                meter_window(samples,0.,7.)

    def test_zero_orbit_defaults_do_not_change_a_policy_identity(self):
        import json
        from pathlib import Path
        from experiments.compare_refit import effective_digest
        root = Path(__file__).resolve().parents[1]
        original = json.loads((root/'configs/continuous-spin.json').read_text())
        expanded = dict(original, orbit=dict(original['orbit'], roll_bias_deg=0., y_bias_mm=0.))
        self.assertEqual(effective_digest('spin', original), effective_digest('spin', expanded))
        expanded['orbit']['period_frames'] += 1
        self.assertNotEqual(effective_digest('spin', original), effective_digest('spin', expanded))
