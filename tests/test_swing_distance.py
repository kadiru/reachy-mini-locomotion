import unittest

from reachy_locomotion.swing_distance import cumulative_strokes, stroke_count


class SwingCounterTests(unittest.TestCase):
    def test_small_reversals_do_not_split_a_stroke(self):
        self.assertEqual(list(cumulative_strokes([0, 11, 18, 13, 22, 12, 3, -8, 3])),
                         [0, 1, 1, 1, 1, 2, 2, 2, 3])

    def test_rebound_at_rest_counts_and_sign_is_symmetric(self):
        angles = [0, -18, 10, -7, 5, 0, 0]
        self.assertEqual(stroke_count(angles), 4)
        self.assertEqual(stroke_count(-x for x in angles), 4)
        self.assertEqual(stroke_count([0, 2, 0, -2]), 0)
        self.assertEqual(stroke_count([]), 0)
