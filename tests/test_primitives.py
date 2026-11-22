import importlib.util
import json
import unittest
from pathlib import Path

from reachy_locomotion.primitives import Primitive,PrimitiveController,classify_trial


class PrimitivePolicyTests(unittest.TestCase):
    def test_delayed_trigger_and_no_waist_command(self):
        p=Primitive(catch_at_deg=10);c=PrimitiveController(p)
        c.target(.7,0);c.target(.72,12)
        self.assertIsNone(c.catch_trigger)
        pose,stage,_=c.target(.74,12)
        self.assertEqual(c.catch_trigger,.74)
        self.assertEqual(stage,'catch')
        self.assertEqual(pose.shape,(4,4))

    def test_guard_stop_is_not_a_fall_or_recovery(self):
        tail=[dict(tilt_deg=0,base_angular_speed_rad_s=0,base_speed_m_s=0,normal_n=10)]
        r=classify_trial('actual_joint_limit_exceeded',tail,20)
        self.assertFalse(r['fell']);self.assertFalse(r['recovered']);self.assertFalse(r['model_valid'])
        self.assertTrue(classify_trial('nonfoot_floor_contact',tail,50)['fell'])
        self.assertTrue(classify_trial(None,tail,20)['recovered'])
        self.assertFalse(classify_trial(None,tail,2)['recovered'])


ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'experiments/upstream-reachy-mini'

@unittest.skipUnless(importlib.util.find_spec('mujoco') and (UPSTREAM/'.git').exists(),'Needs simulation assets')
class PrimitivePhysicsTests(unittest.TestCase):
    def test_moving_curriculum_starts_with_zero_velocity_at_the_supporting_rim(self):
        import mujoco
        import numpy as np
        from reachy_locomotion.primitives import PrimitiveSimulator
        config=json.loads((ROOT/'configs/motors-speed-trained.json').read_text());config['timestep_s']=.00025
        sim=PrimitiveSimulator(UPSTREAM,config);m=sim.model
        for axis,angle,rate in [('roll',-31,-10),('pitch',31,10)]:
            r=sim.run_primitive(Primitive(),seconds=1,record=True,tilt_limit_deg=35,
                                diagnostic_initial_angular_rate_deg_s=rate,**{f'diagnostic_initial_{axis}_deg':angle})
            self.assertFalse(r['natural_upright_start']);self.assertEqual(r['diagnostic_initial_angular_rate_deg_s'],rate)
            d=mujoco.MjData(m);d.qpos[:]=r['trajectory'][0]['qpos'];d.qvel[:]=r['trajectory'][0]['qvel'];mujoco.mj_forward(m,d)
            rim=np.zeros(3);rim[1 if axis=='roll' else 0]=config['foot_radius_m']*np.sign(angle)*(-1 if axis=='roll' else 1)
            point=d.xpos[sim.base]+d.xmat[sim.base].reshape(3,3)@rim;jac=np.zeros((3,m.nv))
            mujoco.mj_jac(m,d,jac,None,point,sim.base)
            np.testing.assert_allclose(jac@d.qvel,0,atol=1e-10)

    def test_cad_release_pivots_about_loaded_vertex_on_both_edges(self):
        import mujoco
        import numpy as np
        from reachy_locomotion.primitives import PrimitiveSimulator
        from reachy_locomotion.shell_contact import place_on_floor
        config = json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        config.update(timestep_s=.00025, cad_shell_floor=True)
        sim = PrimitiveSimulator(UPSTREAM, config)
        model = sim.model
        root_z = model.joint('floating_foot').qposadr[0]+2
        for side in (-1, 1):
            result = sim.run_primitive(Primitive(), seconds=1, record=True, waist_mode='hold_neutral',
                                       tilt_limit_deg=None, observe_landing=True,
                                       diagnostic_initial_roll_deg=side*32.,
                                       diagnostic_initial_angular_rate_deg_s=side*2.)
            data = mujoco.MjData(model)
            state = result['trajectory'][0]
            data.qpos[:] = state['qpos']
            data.qvel[:] = state['qvel']
            mujoco.mj_forward(model, data)
            contact = place_on_floor(model, data, root_z)
            self.assertAlmostEqual(data.qpos[root_z], state['qpos'][root_z], places=9)
            self.assertAlmostEqual(contact[2], 0., places=10)
            jac = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jac, None, contact, sim.base)
            np.testing.assert_allclose(jac@data.qvel, 0., atol=1e-10)

    def test_40_degree_reset_continues_until_contact(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        config = json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        config['timestep_s'] = .00025
        sim = PrimitiveSimulator(UPSTREAM, config)
        for backstop in (80, None):
            with self.subTest(backstop=backstop):
                recording = dict(record=True, collision_window=(.003,.017)) if backstop is None else {}
                r = sim.run_primitive(Primitive(preload_roll_deg=0, roll_deg=0), seconds=1,
                                      waist_mode='hold_neutral', diagnostic_initial_roll_deg=40,
                                      tilt_limit_deg=backstop, **recording)
                self.assertEqual(r['first_35deg_time_s'], 0)
                self.assertEqual(r['first_40deg_time_s'], 0)
                self.assertEqual(r['termination'], 'nonfoot_floor_contact')
                self.assertGreater(r['peak_tilt_deg'], 60)
                if recording:
                    times = [frame['time_s'] for frame in r['collision_frames']]
                    self.assertGreaterEqual(len(times), 7)
                    self.assertTrue(all(.003-1e-9 <= t <= .017+1e-9 for t in times))
                    self.assertLessEqual(max(b-a for a,b in zip(times,times[1:])), .00200001)

    def test_ball_floor_contact_does_not_disable_robot_floor_guard(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        config=json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        config['timestep_s']=.00025;config['disturbance_balls']=[dict(launch_s=2)]
        sim=PrimitiveSimulator(UPSTREAM,config)
        result=sim.run_primitive(Primitive(preload_roll_deg=0,roll_deg=0),seconds=1.2,waist_mode='hold_neutral',tilt_limit_deg=35)
        self.assertTrue(result['completed'])
        self.assertFalse(result['ball_disturbances'][0]['released'])
        self.assertFalse(result['ball_disturbances'][0]['contacted_robot'])
        self.assertEqual(result['tilt_limit_deg'],35)
        self.assertFalse(result['direct_robot_forces_applied'])
        self.assertAlmostEqual(sim.model.body_subtreemass[sim.base],config['motor_model']['total_mass_kg'],places=10)

    def test_pitch_curriculum_reset_is_labeled_and_loads_the_requested_edge(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        config=json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        config['timestep_s']=.00025
        sim=PrimitiveSimulator(UPSTREAM,config)
        for side in [-1,1]:
            result=sim.run_primitive(Primitive(),seconds=1,waist_mode='hold_neutral',
                                     tilt_limit_deg=35,diagnostic_initial_pitch_deg=side*32.1)
            self.assertFalse(result['natural_upright_start'])
            self.assertEqual(result['diagnostic_initial_pitch_deg'],side*32.1)
            self.assertIsNone(result['diagnostic_initial_roll_deg'])
            row=result['trace'][0]
            self.assertAlmostEqual(row['pitch_deg'],side*32.1)
            self.assertAlmostEqual(row['roll_deg'],0)
            loaded=1 if side==1 else 0
            self.assertGreater(row['foot_front_back_normal_forces_n_back_front'][loaded],.5)
            self.assertLess(row['foot_front_back_normal_forces_n_back_front'][1-loaded],.1)
            self.assertGreater(row['front_back_height_difference_mm'],59)
            self.assertFalse(result['external_forces_applied'])
        with self.assertRaises(ValueError):
            sim.run_primitive(Primitive(),diagnostic_initial_roll_deg=30,diagnostic_initial_pitch_deg=30)

    def test_finish_settling_preserves_every_physical_frame(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        class FinishedController(PrimitiveController):
            finished=True
        s=PrimitiveSimulator(UPSTREAM,json.loads((ROOT/'configs/motors-speed-trained.json').read_text()))
        p=Primitive(preload_roll_deg=-15,roll_deg=15)
        full=s.run_primitive(p,seconds=1.2,record=True,controller=FinishedController(p))
        short=s.run_primitive(p,seconds=1.2,record=True,controller=FinishedController(p),finish_settle_s=.8)
        self.assertTrue(short['stopped_after_controller_finish'])
        self.assertEqual(short['max_duration_seconds'],1.2)
        self.assertAlmostEqual(short['elapsed_seconds'],.8)
        self.assertEqual(short['trajectory'],full['trajectory'][:len(short['trajectory'])])
        with self.assertRaises(ValueError):s.run_primitive(p,finish_settle_s=.4)

    def test_disabled_waist_has_zero_torque_and_recording_does_not_change_physics(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        s=PrimitiveSimulator(UPSTREAM,json.loads((ROOT/'configs/motors-speed-trained.json').read_text()))
        p=Primitive(preload_roll_deg=-15,roll_deg=15,yaw_deg=15)
        a=s.run_primitive(p,seconds=1.2)
        b=s.run_primitive(p,seconds=1.2,record=True)
        self.assertEqual(a,{k:v for k,v in b.items() if k!='trajectory'})
        self.assertEqual(a['waist_peak_torque_nm'],0)
        self.assertEqual(a['waist_absolute_motor_work_j'],0)
        self.assertEqual(a['commanded_waist_position_range_rad'],[0.,0.])
        self.assertFalse(a['external_forces_applied'])
        self.assertTrue(a['model_valid'])
        self.assertTrue(all(x['waist_torque_nm']==0 for x in b['trajectory']))
        json.dumps(b, allow_nan=False)

    def test_fixed_waist_target_is_not_claimed_unpowered(self):
        from reachy_locomotion.primitives import PrimitiveSimulator
        s=PrimitiveSimulator(UPSTREAM,json.loads((ROOT/'configs/motors-speed-trained.json').read_text()))
        r=s.run_primitive(Primitive(preload_roll_deg=-15,roll_deg=15,yaw_deg=30),seconds=1.2,waist_mode='hold_neutral')
        self.assertEqual(r['commanded_waist_position_range_rad'],[0.,0.])
        self.assertGreater(r['waist_peak_torque_nm'],0)
        self.assertEqual(r['waist_mode'],'hold_neutral')

    def test_active_waist_moves_and_audit_observes_every_substep(self):
        import numpy as np
        from reachy_locomotion.geometry import rotation
        from reachy_locomotion.primitives import PrimitiveSimulator
        config = json.loads((ROOT/'configs/motors-speed-trained.json').read_text())
        sim = PrimitiveSimulator(UPSTREAM, config)

        class Controller:
            waist_target_rad = 0.
            catch_trigger = yaw_trigger = None

            def observe_waist(self, position):
                self.encoder = position

            def target(self, t, roll):
                self.waist_target_rad = .16 * min(t, 1.)
                pose = np.eye(4)
                pose[2, 3] = .177
                pose[:3, :3] = rotation((0., 0., self.waist_target_rad))
                return pose, 'sweep', roll

        times = []
        result = sim.run_primitive(Primitive(), seconds=1.2, waist_mode='position',
                                   controller=Controller(), step_audit=lambda _, d: times.append(d.time))
        self.assertTrue(result['completed'])
        self.assertEqual(result['commanded_waist_position_range_rad'], [0., .16])
        self.assertGreater(result['measured_waist_angle_range_deg'][1], 7.)
        self.assertEqual(len(times), round(1.2 / sim.dt))
