import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from reachy_locomotion.ball_disturbance import RollingBall,add_balls,BallDisturbances
from reachy_locomotion.balance_curriculum import impact_metrics


class RollingBallTests(unittest.TestCase):
    def test_invalid_disturbances_are_rejected(self):
        self.assertEqual(RollingBall(speed_m_s=12).speed_m_s, 12)
        for args in [dict(mass_kg=-1),dict(speed_m_s=12.01),dict(from_side=0),dict(launch_s=float('nan'))]:
            with self.assertRaises(ValueError):RollingBall(**args)

    def test_real_rolling_contact_moves_body_and_rebounds_from_both_sides(self):
        for side in [-1,1]:
            root=ET.fromstring('<mujoco><option timestep="0.00025"/><worldbody><geom name="floor" type="plane" size="2 2 .1"/><body name="robot" pos="0 0 .06"><freejoint/><geom type="box" size=".06 .06 .06" mass="1"/></body></worldbody></mujoco>')
            settings=[dict(launch_s=1,from_side=side,speed_m_s=.7)]
            add_balls(root.find('worldbody'),settings)
            m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);base=m.body('robot').id
            mujoco.mj_forward(m,d);balls=BallDisturbances(m,base,settings);before=d.qpos[:7].copy()
            balls.release(d,.98);self.assertFalse(balls.report()[0]['released'])
            balls.release(d,1);np.testing.assert_array_equal(d.qpos[:7],before)
            event=balls.events[0];q=event['qadr'];v=event['vadr'];self.assertAlmostEqual(d.qpos[q+2],.0301)
            np.testing.assert_allclose(d.qvel[v+3:v+6],np.cross([0,0,1],d.qvel[v:v+3])/.03)
            for _ in range(2800):
                mujoco.mj_step(m,d);mujoco.mj_forward(m,d);balls.observe(d)
            result=balls.report()[0]
            self.assertTrue(result['contacted_robot']);self.assertTrue(result['rebounded'])
            self.assertAlmostEqual(result['incoming_speed_m_s'], .7, delta=.04)
            self.assertGreater(result['normal_impulse_ns'],.001)
            self.assertGreater(-side*d.qpos[1],.00001)
            old=d.qpos.copy();balls.release(d,2);np.testing.assert_array_equal(d.qpos,old)

    def test_a_miss_or_unbalanced_impact_cannot_count_as_recovery(self):
        rows=[dict(time_s=i*.02,roll_deg=32.1,roll_rate_deg_s=0,pitch_deg=0,edge_support=True) for i in range(301)]
        event=dict(index=0,first_robot_contact_s=2.,normal_impulse_ns=.02,contacted_robot=True,rebounded=True)
        result=dict(trace=rows,completed=True,ball_disturbances=[event])
        self.assertTrue(impact_metrics(result,'roll',1,32.1)['all_impacts_pass'])
        event['rebounded']=False;self.assertFalse(impact_metrics(result,'roll',1,32.1)['all_impacts_pass'])
        event['rebounded']=True
        for x in rows:
            if x['time_s']<2:x['roll_deg']=0
        self.assertFalse(impact_metrics(result,'roll',1,32.1)['all_impacts_pass'])


if __name__=='__main__':unittest.main()
