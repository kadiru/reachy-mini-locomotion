import unittest
import numpy as np
from reachy_locomotion.rim_roll import measures


class RimRollTests(unittest.TestCase):
    def ring(self):
        t = np.arange(0., 5.01, .02)
        rows=[]
        for s in t:
            v=np.array([np.cos(2*np.pi*s),np.sin(2*np.pi*s)])
            rows.append(dict(time_s=s,contact_foot_xy_m=(.056*v).tolist(),foot_xyz_m=(.021*v).tolist(),
                             contact_world_m=(.055*v).tolist(),tilt_deg=16.,normal_n=10.,contact_slip_m_s=0.))
        return rows

    def test_both_frames_measure_different_circles(self):
        r=measures(self.ring(),.056,1,0.)
        self.assertGreaterEqual(r['laps'],4)
        self.assertAlmostEqual(r['signed_rim_turns'],5.)
        self.assertAlmostEqual(r['center_diameter_mm'],42.)
        self.assertAlmostEqual(r['contact_diameter_mm'],110.)

    def test_unloaded_gaps_and_straight_drift_do_not_count_as_circles(self):
        rows=self.ring()
        for i,row in enumerate(rows):
            row['foot_xyz_m']=[i*.001,0]
        self.assertEqual(measures(rows,.056,1,0.)['center_diameter_mm'],0.)
        for i in range(0,len(rows),20): rows[i]['normal_n']=0.
        r=measures(rows,.056,1,0.)
        self.assertEqual(r['laps'],0)
        self.assertLess(r['longest_rim_turns'],1.)

    def test_floor_course_closes_a_slow_loop_but_rejects_a_one_way_arc(self):
        from reachy_locomotion.rim_roll import ground_course
        t=np.arange(0.,44.02,.02)
        data=[dict(time_s=s,foot_xyz_m=[.10*np.cos(2*np.pi*s/32)+.002*np.sin(12*s),
                                        .10*np.sin(2*np.pi*s/32),0.]) for s in t]
        loop=ground_course(data)['best_loop']
        self.assertIsNotNone(loop)
        self.assertAlmostEqual(loop['diameter_mm'],200.,delta=2.)
        self.assertIsNone(ground_course(data[:550])['best_loop'])

    def test_wrong_direction_is_not_a_positive_rim_lap(self):
        r=measures(self.ring(),.056,-1,0.)
        self.assertLess(r['signed_rim_turns'],0.)
        self.assertEqual(r['laps'],0)
