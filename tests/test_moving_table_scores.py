import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from qualify_moving_table import bounds, edge_pass
from moving_table import footprint_displacement


class MovingTableScores(unittest.TestCase):
    def test_fixed_contact_rotating_with_table_has_zero_footprint_travel(self):
        rows = []
        for t, degrees in [(2.5, 0.), (3., 0.), (4., 28.), (5., 0.)]:
            angle = math.radians(degrees)
            rows.append(dict(time_s=t, table_deg=degrees,
                             contact_world_m=[0., .05*math.cos(angle), .05*math.sin(angle)]))
        self.assertAlmostEqual(footprint_displacement(rows, 3.)['max_footprint_travel_mm'], 0.)
        rows[-1]['contact_world_m'][1] += .012
        self.assertAlmostEqual(footprint_displacement(rows, 3.)['final_footprint_travel_mm'], 12.)

    def test_higher_success_does_not_hide_first_failed_condition(self):
        self.assertEqual(bounds({10.: True, 6.: False, 5.: True, 2.: True}), (5., 6.))
        self.assertEqual(bounds({2.: True, 8.: True}), (8., None))

    def test_edge_hold_requires_valid_table_motion_and_uninterrupted_support(self):
        row = dict(job=dict(cruise=True, flat=False, rate=32.), tracking_error_deg=.01,
                   peak_table_rate=32.02, start_ok=True, continuous_edge=True, final_edge=True,
                   termination='completed', joint_margin=.02, shell=None, stable_endpoints=False)
        self.assertTrue(edge_pass(row))
        for change in (dict(continuous_edge=False), dict(joint_margin=-.001),
                       dict(shell={'time_s': 4.8}), dict(peak_table_rate=28.)):
            self.assertFalse(edge_pass(dict(row, **change)))
        flat = dict(row, job=dict(row['job'], flat=True), peak_table_rate=0.)
        self.assertTrue(edge_pass(flat))
