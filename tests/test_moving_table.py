import math
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from reachy_locomotion.tilt_table import Excursion, add_table


class TableMotionTests(unittest.TestCase):
    def test_speed_experiment_cruises_at_the_named_rate_in_each_direction(self):
        path=Excursion(5.,32.,cycles=2,cruise=True)
        for origin,sign in ((path.start,1),(path.start+path.leg+path.dwell,-1)):
            for fraction in (.25,.5,.75):
                t=origin+fraction*path.leg
                position,velocity,acceleration=path.sample(t)
                self.assertGreater(position,0)
                self.assertLess(position,5)
                self.assertAlmostEqual(velocity,sign*32)
                self.assertEqual(acceleration,0)
                numerical=(path.sample(t+1e-6)[0]-path.sample(t-1e-6)[0])/2e-6
                self.assertAlmostEqual(velocity,numerical,places=6)
        self.assertAlmostEqual(path.sample(path.start+path.leg)[0],5.)
        self.assertAlmostEqual(path.sample(path.end)[0],0.)

    def test_both_legs_reach_the_specified_angle_and_peak_rate(self):
        for angle in (-10.,10.):
            path=Excursion(angle,8.,cycles=2)
            for i in range(path.cycles):
                start=path.start+i*2*(path.leg+path.dwell)
                for fraction,position,rate in ((0,0,0),(.5,angle/2,math.copysign(8,angle)),(1,angle,0)):
                    x,v,_=path.sample(start+fraction*path.leg)
                    self.assertAlmostEqual(x,position)
                    self.assertAlmostEqual(v,rate)
                x,v,_=path.sample(start+1.5*path.leg+path.dwell)
                self.assertAlmostEqual(x,angle/2)
                self.assertAlmostEqual(v,-math.copysign(8,angle))
            self.assertEqual(path.sample(path.end+.1),(0.,0.,0.))

    def test_hinge_motion_has_actual_contact_point_velocity(self):
        root=ET.fromstring('<mujoco><option timestep=".000125" integrator="implicitfast"/>'
                           '<worldbody><geom type="plane" name="floor" size="1 1 .02"/></worldbody></mujoco>')
        add_table(root.find('worldbody'))
        model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        data=mujoco.MjData(model)
        path=Excursion(-8.,16.,start=0.)
        for _ in range(round(.5*path.leg/model.opt.timestep)):
            path.drive(data,0)
            mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
        expected=path.sample(data.time)
        self.assertAlmostEqual(math.degrees(data.qpos[0]),expected[0],delta=.02)
        self.assertAlmostEqual(math.degrees(data.qvel[0]),expected[1],delta=.05)
        body=model.body('tilt_table').id
        point=data.xpos[body]+data.xmat[body].reshape(3,3)@np.array([0.,.05,0.])
        jac=np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jac,None,point,body)
        np.testing.assert_allclose(jac@data.qvel,np.cross([data.qvel[0],0.,0.],point),atol=1e-12)
        self.assertLess((jac@data.qvel)[2],-.01)
