import unittest

from reachy_locomotion.target_pivot import TargetPivot, pivot_metrics


class TargetPivotTest(unittest.TestCase):
    def test_requested_direction_and_overshoot(self):
        bank = [dict(id=str(d), turn_deg=d, responses=[dict(turn=d, edge=d, drift=0, roll=10, delay=1)])
                for d in (-15, -4, 4, 15)]
        p = TargetPivot(bank, 25, None)
        self.assertEqual(p.choose(16)['turn_deg'], 15)
        self.assertEqual(p.choose(-5)['turn_deg'], -4)

    def test_wrong_way_landing_changes_next_motion(self):
        def option(name, turns, rolls):
            return dict(id=name, responses=[dict(turn=t, edge=t, drift=0, roll=r, delay=1)
                                            for t,r in zip(turns, rolls)])
        bank = [option('probe', [10, -4], [20, 3]), option('alternative', [-2, 9], [16, 15])]
        p = TargetPivot(bank, 20, None)
        p.strokes.append(dict(option='probe', measured_turn_deg=-4, peak_sensed_roll_deg=3))
        p.measured_turns['probe'] = -4
        self.assertEqual(p.choose(24)['id'], 'alternative')

    def test_zero_turn_holds_neutral(self):
        p = TargetPivot([], 0, None, noise=0)
        pose, stage, _ = p.target(0, 0)
        self.assertTrue(p.finished)
        self.assertEqual(len(p.strokes), 0)
        self.assertEqual(stage, 'heading hold')
        self.assertEqual(pose[2, 3], .177)

    def test_edge_yaw_excludes_rim_changes_and_unwinds(self):
        trace = [dict(time_s=.02*i, foot_xyz_m=[0.,0.,0.], roll_deg=roll, edge_support=edge,
                      torso_yaw_deg=yaw, foot_yaw_deg=yaw)
                 for i,(roll,edge,yaw) in enumerate([(10,True,0),(10,True,8),
                                                      (-10,True,14),(-10,True,10),(0,False,15)])]
        r = dict(trace=trace, final_torso_yaw_deg=15, final_foot_yaw_deg=15,
                 edge_support_seconds=.08, peak_side_height_difference_mm=11,
                 edge_contact_slip_distance_mm=1., measured_waist_angle_range_deg=[0,0],
                 commanded_waist_position_range_rad=[0,0], settled=True, fell=False,
                 termination='completed', completed=True)
        m = pivot_metrics(r,15)
        self.assertEqual(m['edge_deg_by_rim'], dict(left=-4, right=8))
        self.assertEqual(m['signed_edge_deg'], 4)
        self.assertFalse(m['passed'])


if __name__ == '__main__':
    unittest.main()
