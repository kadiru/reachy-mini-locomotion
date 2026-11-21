import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from reachy_locomotion.ball_disturbance import RollingBall,add_balls,BallDisturbances,ball_config
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

    def test_airborne_and_floor_first_contacts_are_distinguishable(self):
        for side, height, vz, airborne in [(1,.15,.25,True),(-1,.15,.25,True),(1,.045,-.6,False)]:
            root = ET.fromstring('<mujoco><option timestep=".00025"/><worldbody><geom name="floor" type="plane" size="2 2 .1"/><body name="robot" pos="0 0 .1"><freejoint/><geom type="box" size=".06 .06 .1" mass="1"/></body></worldbody></mujoco>')
            settings = [dict(launch_s=1, from_side=side, distance_m=.2, speed_m_s=1.2,
                             launch_height_m=height, vertical_speed_m_s=vz)]
            add_balls(root.find('worldbody'),settings)
            model = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
            data = mujoco.MjData(model)
            data.time = 1.
            mujoco.mj_forward(model,data)
            balls = BallDisturbances(model,model.body('robot').id,settings)
            robot_pose = data.qpos[:7].copy()
            balls.release(data,data.time)
            e = balls.events[0]
            self.assertAlmostEqual(data.qpos[e['qadr']+2],height)
            np.testing.assert_array_equal(data.qvel[e['vadr']+3:e['vadr']+6],0.)
            np.testing.assert_array_equal(data.qpos[:7],robot_pose)
            for _ in range(2400):
                mujoco.mj_step(model,data)
                mujoco.mj_forward(model,data)
                balls.observe(data)
            event = balls.report()[0]
            self.assertTrue(event['contacted_robot'])
            self.assertEqual(event['first_robot_body'],'robot')
            self.assertEqual(event['first_robot_contact_s'] < event['first_floor_contact_s'],airborne)
            self.assertAlmostEqual(event['incoming_3d_speed_m_s'],np.linalg.norm(event['incoming_velocity_m_s']))
            if airborne:
                self.assertAlmostEqual(event['incoming_speed_m_s'],1.2,places=5)
                self.assertGreater(event['first_contact_height_m'],.09)
                self.assertGreater(event['incoming_3d_speed_m_s'],event['incoming_speed_m_s'])

    def test_airborne_release_requires_a_valid_height(self):
        for setting in [dict(vertical_speed_m_s=.3),dict(launch_height_m=.02),dict(launch_height_m=float('nan'))]:
            with self.assertRaises(ValueError):
                ball_config(setting)

    def test_ten_simultaneous_overhead_drops_start_from_rest(self):
        world = ET.fromstring('<mujoco><option timestep=".00025"/><worldbody><geom name="floor" type="plane" size="2 2 .1"/><body name="robot" pos="0 0 .10"><freejoint/><geom size=".18 .14 .10" type="box" mass="2"/><site name="head"/></body></worldbody></mujoco>')
        settings = [dict(drop_height_m=.45,offset_x_m=(i%5-2)*.034,offset_y_m=(i//5-.5)*.034,launch_s=1)
                    for i in range(10)]
        with self.assertRaises(ValueError):
            add_balls(world.find('worldbody'),settings+[settings[0]])
        add_balls(world.find('worldbody'),settings)
        model=mujoco.MjModel.from_xml_string(ET.tostring(world,encoding='unicode'))
        data=mujoco.MjData(model)
        manager=BallDisturbances(model,model.body('robot').id,settings)
        for _ in range(80):
            mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
        data.time=1
        manager.release(data,1)
        positions=[]
        for e in manager.events:
            np.testing.assert_array_equal(data.qvel[e['vadr']:e['vadr']+6],np.zeros(6))
            positions.append(e['release_position_m'])
            self.assertEqual(positions[-1][2],.45)
        for i,pos in enumerate(positions):
            for other in positions[:i]:
                self.assertGreater(np.linalg.norm(np.subtract(pos,other)),.03)
        for _ in range(1250):
            mujoco.mj_step(model,data)
            mujoco.mj_forward(model,data)
            manager.observe(data)
        for e in manager.report():
            self.assertEqual(e['release_s'],1)
            self.assertEqual(e['first_robot_body'],'robot')
            self.assertIsNone(e['first_floor_contact_s'])
            self.assertLess(e['incoming_velocity_m_s'][2],-1.9)
            self.assertLess(e['incoming_speed_m_s'],.00001)


if __name__=='__main__':unittest.main()
