"""Externally driven roll hinge for free-body balance tests in MuJoCo."""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass


INERTIA = 20.
STIFFNESS = 50_000.
DAMPING = 2_000.


def add_table(world):
    floor = world.find("geom[@name='floor']")
    world.remove(floor)
    table = ET.SubElement(world, 'body', name='tilt_table')
    ET.SubElement(table, 'inertial', pos='0 0 0', mass='60', diaginertia='20 20 20')
    ET.SubElement(table, 'joint', name='table_roll', type='hinge', axis='1 0 0',
                  limited='false', stiffness=str(STIFFNESS), damping=str(DAMPING))
    floor.attrib.update(type='box', size='2 2 .02', pos='0 0 -.02')
    table.append(floor)


@dataclass(frozen=True)
class Excursion:
    """A signed out-and-back bank. Rate is peak speed for either leg profile."""
    angle: float
    rate: float
    start: float = 3.
    dwell: float = 1.
    recovery: float = 2.
    cycles: int = 1
    cruise: bool = False

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.angle, self.rate, self.start, self.dwell, self.recovery)):
            raise ValueError('Motion parameters must be finite')
        if self.rate <= 0 or min(self.start, self.dwell, self.recovery) < 0 or abs(self.angle) >= 90:
            raise ValueError('Invalid moving-table excursion')
        if type(self.cycles) is not int or self.cycles < 1:
            raise ValueError('Expected a positive number of cycles')

    @property
    def leg(self):
        if self.cruise:
            return abs(self.angle) / (.8 * self.rate)
        return math.pi * abs(self.angle) / (2 * self.rate)

    @property
    def end(self):
        return self.start + self.cycles * (2*self.leg + 2*self.dwell) - self.dwell

    def sample(self, t):
        elapsed = t - self.start
        if elapsed < 0 or not self.angle or t >= self.end:
            return 0., 0., 0.
        leg = self.leg
        elapsed %= 2*leg + 2*self.dwell
        if elapsed >= 2*leg + self.dwell:
            return 0., 0., 0.
        if leg <= elapsed < leg + self.dwell:
            return self.angle, 0., 0.
        returning = elapsed >= leg + self.dwell
        t = elapsed - leg - self.dwell if returning else elapsed
        sign = -1 if returning else 1

        if self.cruise:
            ease = .2 * leg
            rate = math.copysign(self.rate, self.angle)
            if t < ease:
                acc = rate / ease
                pos, vel = .5*acc*t*t, acc*t
            elif t <= leg - ease:
                pos, vel, acc = rate*(t - .5*ease), rate, 0.
            else:
                remaining = leg - t
                acc = -rate / ease
                pos, vel = self.angle + .5*acc*remaining**2, -acc*remaining
            return (self.angle if returning else 0.) + sign*pos, sign*vel, sign*acc

        phase = t * math.pi / leg
        return (self.angle * (1 - sign*math.cos(phase))/2,
                sign*self.angle * (math.pi/leg)*math.sin(phase)/2,
                sign*self.angle * (math.pi/leg)**2*math.cos(phase)/2)

    def drive(self, data, dof):
        pos, vel, acc = self.sample(data.time)
        # The hinge's passive spring and damper supply the measured-state terms.
        data.qfrc_applied[dof] = math.radians(STIFFNESS*pos + DAMPING*vel + INERTIA*acc)
