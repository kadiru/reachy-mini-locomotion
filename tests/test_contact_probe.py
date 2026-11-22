import copy
import importlib.util
import json
import unittest
from pathlib import Path

from reachy_locomotion.contact_probe import validate_config, scene_xml, relative_head_pose, run_probe

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "experiments/upstream-reachy-mini"
CONFIG = json.loads((ROOT / "configs/contact-proxy.json").read_text())


class ContactConfigTests(unittest.TestCase):
    def test_invalid_physics_parameters_rejected(self):
        for key, value in (("foot_radius_m", -1), ("motor_force_limit_nm", float("nan")),
                           ("timestep_s", .02), ("friction_slide_torsion_roll", [1, -1, 0]),
                           ("friction_slide_torsion_roll", [1, 0]), ("evidence", "physical")):
            config = copy.deepcopy(CONFIG)
            config[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_config(config)


@unittest.skipUnless(importlib.util.find_spec("mujoco") and (UPSTREAM / ".git").exists(),
                     "Install sim extra and fetch pinned upstream assets for physics tests")
class ContactPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        cls.xml, _ = scene_xml(UPSTREAM, CONFIG)
        cls.model = mujoco.MjModel.from_xml_string(cls.xml)

    def test_free_base_preserves_mass_and_closed_chain(self):
        import mujoco
        stock = mujoco.MjModel.from_xml_path(str(UPSTREAM / "src/reachy_mini/descriptions/reachy_mini/mjcf/scenes/empty.xml"))
        self.assertEqual(self.model.nq, stock.nq + 7)
        self.assertEqual(self.model.nv, stock.nv + 6)
        self.assertEqual(self.model.nu, stock.nu)
        self.assertEqual(self.model.neq, stock.neq)
        self.assertAlmostEqual(sum(self.model.body_mass), sum(stock.body_mass))
        free = self.model.joint("floating_foot").id
        dof = self.model.jnt_dofadr[free]
        self.assertTrue((self.model.dof_frictionloss[dof:dof+6] == 0).all())
        self.assertTrue((self.model.dof_damping[dof:dof+6] == 0).all())

    def test_cad_floor_uses_external_parts_and_places_a_side_release_without_penetration(self):
        import mujoco
        import numpy as np
        from reachy_locomotion.shell_contact import place_on_floor
        model = mujoco.MjModel.from_xml_string(scene_xml(UPSTREAM, dict(CONFIG,cad_shell_floor=True))[0])
        self.assertAlmostEqual(sum(model.body_mass), sum(self.model.body_mass))
        floor = model.geom("floor").id
        foot = model.geom("foot_contact_proxy").id
        self.assertEqual(model.geom_type[foot], mujoco.mjtGeom.mjGEOM_MESH)
        self.assertEqual(model.geom_conaffinity[floor], 8)
        shells = np.flatnonzero(model.geom_contype == 8)
        meshes = [model.mesh(model.geom_dataid[g]).name for g in shells]
        self.assertEqual(meshes.count("antenna"), 2)
        self.assertIn("head_front_3dprint",meshes)
        self.assertIn("body_down_3dprint",meshes)
        d = mujoco.MjData(model)
        adr = model.jnt_qposadr[model.joint("floating_foot").id]
        d.qpos[adr+3:adr+7] = [np.cos(np.pi/4),np.sin(np.pi/4),0.,0.]
        mujoco.mj_forward(model,d)
        place_on_floor(model,d,adr+2)
        mujoco.mj_forward(model,d)
        gaps = [mujoco.mj_geomDistance(model,d,floor,g,1.,None) for g in shells]
        self.assertAlmostEqual(min(gaps),0.,places=6)

    def test_foot_frame_is_invariant_to_world_translation_and_yaw(self):
        import mujoco
        import numpy as np
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        before = relative_head_pose(self.model, data)
        addr = self.model.jnt_qposadr[self.model.joint("floating_foot").id]
        data.qpos[addr:addr+3] += [1, -2, .3]
        data.qpos[addr+3:addr+7] = [np.cos(.4), 0, 0, np.sin(.4)]
        mujoco.mj_forward(self.model, data)
        np.testing.assert_allclose(relative_head_pose(self.model, data), before, atol=1e-12)

    def test_hold_and_push_have_support_and_small_closure_error(self):
        import numpy as np
        report = run_probe(UPSTREAM, CONFIG, seconds=.6)
        hold, push = report["trials"]
        self.assertLess(np.linalg.norm(hold["foot_displacement_world_m"][:2]), 1e-5)
        self.assertGreater(push["foot_displacement_world_m"][1], 1e-5)
        for trial in report["trials"]:
            self.assertGreater(trial["foot_floor_contact_fraction"], .9)
            self.assertLess(trial["max_loop_closure_error_m"], 1e-4)
            self.assertEqual(trial["warnings"], {})
        self.assertFalse(report["gait_demonstrated"])
        self.assertFalse(report["hardware_ready"])
