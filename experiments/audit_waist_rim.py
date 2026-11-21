"""Evaluation-only contact audit at the MuJoCo integration timestep."""
import math

import mujoco
import numpy as np


class RimAudit:
    def __init__(self, controller):
        self.controller = controller
        self.steps = self.loaded_steps = 0
        self.first_loss = None
        self.gap = self.longest_gap = self.peak_angle = 0.
        self.wrench = np.empty(6)

    def __call__(self, sim, data):
        if self.controller.start is None:
            return
        side = self.controller.side
        rot = data.xmat[sim.base].reshape(3, 3)
        foot = data.xpos[sim.base]
        desired_force = other_force = xsum = ysum = 0.
        for index, contact in enumerate(data.contact):
            if contact.efc_address < 0 or {contact.geom1, contact.geom2} != {sim.foot, sim.floor}:
                continue
            mujoco.mj_contactForce(sim.model, data, index, self.wrench)
            force = abs(self.wrench[0])
            x, y, _ = rot.T @ (contact.pos - foot)
            if side * y < 0:
                desired_force += force
            else:
                other_force += force
            xsum += x * force
            ysum += y * force
        angle = abs(math.degrees(math.atan2(xsum, -side * ysum)))
        self.peak_angle = max(self.peak_angle, angle)
        lift_mm = 2000 * sim.config['foot_radius_m'] * side * rot[2, 1]
        loaded = desired_force > .5 and other_force < .1 and lift_mm >= 10 and angle <= 15
        self.steps += 1
        if loaded:
            self.loaded_steps += 1
            self.gap = 0.
        else:
            if self.first_loss is None:
                self.first_loss = float(data.time)
            self.gap += sim.dt
            self.longest_gap = max(self.longest_gap, self.gap)

    def report(self):
        return dict(samples=self.steps, loaded_samples=self.loaded_steps, first_loss_s=self.first_loss,
                    longest_loss_s=self.longest_gap, peak_contact_azimuth_error_deg=self.peak_angle)
