"""A ceiling-hung bob on a unilateral cable, released from rest."""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def vector(values):
    return ' '.join(map(str, values))


def add_pendulum(root, settings):
    world = root.find('worldbody')
    length, mass, radius = (settings[k] for k in ('length_m','mass_kg','radius_m'))
    anchor = np.asarray(settings['anchor_xyz_m'], dtype=float)
    angle = math.radians(settings['release_deg'])
    side = settings['side']
    if not (side in (-1,1) and 0<=angle<math.pi/2 and length>0 and radius>0 and mass>0):
        raise ValueError('Invalid pendulum dimensions, side or release angle')
    if anchor.shape!=(3,) or not np.isfinite(anchor).all() or anchor[2]-length<=radius:
        raise ValueError('The bob must clear the floor at rest')
    start = anchor + [0,side*length*math.sin(angle),-length*math.cos(angle)]
    ET.SubElement(world,'site',name='ceiling_anchor',pos=vector(anchor),size='.009',rgba='.7 .8 .85 1')
    ET.SubElement(world,'geom',type='box',pos=vector(anchor+[0,0,.014]),size='.07 .08 .010',
                  contype='0',conaffinity='0',rgba='.5 .65 .7 1')
    bob = ET.SubElement(world,'body',name='pendulum_bob',pos=vector(start))
    ET.SubElement(bob,'freejoint',name='pendulum_free')
    ET.SubElement(bob,'geom',name='pendulum_ball',type='sphere',size=str(radius),mass=str(mass),
                  contype='1',conaffinity='1',priority='1',condim='3',friction='.3 .001 .00002',
                  solref='-25000 -35',rgba='1 .32 .04 1')
    ET.SubElement(bob,'site',name='bob_center',pos='0 0 0',size='.001')
    tendons=root.find('tendon')
    if tendons is None:
        tendons=ET.SubElement(root,'tendon')
    cable=ET.SubElement(tendons,'spatial',name='pendulum_string',limited='true',range=f'0 {length}',
                        width='.0017',rgba='.95 .85 .6 1',solreflimit='.002 1')
    ET.SubElement(cable,'site',site='ceiling_anchor')
    ET.SubElement(cable,'site',site='bob_center')
    constraints=root.find('equality')
    if constraints is None:
        constraints=ET.SubElement(root,'equality')
    ET.SubElement(constraints,'connect',name='pendulum_latch',body1='pendulum_bob',anchor='0 0 0')


class SwingContacts:
    """Read real solver contacts and the incoming COM velocity one integration step earlier."""
    def __init__(self,model,release_s):
        self.bob = model.geom('pendulum_ball').id
        self.vel = model.jnt_dofadr[model.joint('pendulum_free').id]
        self.cable = model.tendon('pendulum_string').id
        self.release_s = release_s
        self.prev_velocity=np.zeros(3)
        self.wrench=np.zeros(6)
        self.hits=[]
        self.last_contact=-1.
        self.lengths=[float('inf'),0.]
        self.floor_s=None

    def __call__(self,sim,data):
        if data.time < self.release_s:
            return
        length=float(data.ten_length[self.cable])
        self.lengths=[min(length,self.lengths[0]),max(length,self.lengths[1])]
        for i,c in enumerate(data.contact):
            pair=(c.geom1,c.geom2)
            if self.bob not in pair or c.efc_address<0:
                continue
            other=pair[1] if pair[0]==self.bob else pair[0]
            mujoco.mj_contactForce(sim.model,data,i,self.wrench)
            if self.wrench[0]<=.001:
                continue
            if other==sim.floor:
                if self.floor_s is None:
                    self.floor_s=float(data.time)
                continue
            body=sim.model.geom_bodyid[other]
            if sim.model.body_rootid[body]!=sim.base:
                continue
            if data.time-self.last_contact>.015:
                self.hits.append(dict(time_s=float(data.time),body=sim.model.body(body).name,
                                      speed_m_s=float(np.linalg.norm(self.prev_velocity)),
                                      velocity_m_s=self.prev_velocity.tolist(),impulse_ns=0.))
            self.hits[-1]['impulse_ns']+=float(self.wrench[0])*sim.dt
            self.hits[-1]['last_s']=float(data.time)
            self.last_contact=float(data.time)
        self.prev_velocity=data.qvel[self.vel:self.vel+3].copy()
