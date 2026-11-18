import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "experiments/upstream-reachy-mini"
AVAILABLE = (importlib.util.find_spec("mujoco") is not None
             and importlib.util.find_spec("reachy_mini_rust_kinematics") is not None
             and (UPSTREAM / ".git").exists())


@unittest.skipUnless(AVAILABLE, "Install sim extras and fetch pinned model")
class SwayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from reachy_locomotion.sway import SwaySimulator
        cls.sim = SwaySimulator(UPSTREAM, json.loads((ROOT / "configs/sway-cad-proxy.json").read_text()))

    def test_static_ik_matches_measured_foot_relative_head(self):
        import mujoco
        from reachy_locomotion.contact_probe import relative_head_pose
        from reachy_locomotion.geometry import rotation, angle
        sim = self.sim
        state = mujoco.MjData(sim.model)
        state.qpos[:] = sim.initial_qpos
        state.ctrl[:] = sim.initial_ctrl
        pose = np.eye(4)
        pose[:3, :3] = rotation(np.radians([10, 5, 10]))
        pose[:3, 3] = [.005, -.003, .177]
        state.ctrl[sim.act] = sim.checked_ik(pose)
        for _ in range(1500):
            mujoco.mj_step(sim.model, state)
        mujoco.mj_forward(sim.model, state)
        measured = relative_head_pose(sim.model, state)
        self.assertLess(np.linalg.norm(measured[:3,3]-pose[:3,3]), .001)
        self.assertLess(angle(measured, pose), np.radians(1))
        # Optical forward is -Z in MuJoCo's camera frame, here approximately +X.
        camera = state.cam_xmat[sim.model.camera("eye_camera").id].reshape(3,3)
        self.assertGreater(-camera[0,2], .9)

    def test_unreachable_ik_is_rejected_not_clamped(self):
        pose = np.eye(4)
        pose[2,3] = 1
        with self.assertRaises(ValueError):
            self.sim.checked_ik(pose)

    def test_stationary_baseline_has_no_locomotion(self):
        from reachy_locomotion.sway import Gait
        result = self.sim.run(Gait(roll_deg=0,yaw_deg=0),cycles=4)
        self.assertTrue(result["valid"])
        self.assertLess(np.linalg.norm([result["forward_m"],result["lateral_m"]]), 1e-5)
        self.assertFalse(result["external_forces_applied"])

    def test_sway_unloads_both_edges_with_closed_chain_and_no_push(self):
        from reachy_locomotion.sway import Gait
        gait = Gait(**json.loads((ROOT / "configs/sway-candidate.json").read_text()))
        result = self.sim.run(gait,cycles=4,record=True)
        self.assertTrue(result["valid"], result["failure_reason"])
        self.assertGreater(result["left_unloaded_fraction"], .1)
        self.assertGreater(result["right_unloaded_fraction"], .1)
        self.assertLess(result["max_closure_error_m"], .001)
        self.assertFalse(result["external_forces_applied"])
        self.assertAlmostEqual(result["measured_seconds"], 4/result["actual_frequency_hz"])
        self.assertEqual(len(result["cycle_boundaries"]), 5)
        self.assertEqual(result["cycle_boundaries"][0]["foot_displacement_m"], [0.,0.,0.])

    def test_invalid_gait_does_not_run(self):
        from reachy_locomotion.sway import Gait
        for kwargs in [dict(frequency_hz=0),dict(roll_deg=float("nan")),dict(roll_deg=50),dict(x_mm=30)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Gait(**kwargs)
