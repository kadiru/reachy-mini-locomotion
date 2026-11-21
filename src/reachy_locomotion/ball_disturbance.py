"""Physical rolling-ball disturbances, separate from the robot controller.

Each ball is initialized once at its scheduled release. Subsequently all
motion and reaction forces come from MuJoCo gravity/contact integration.
Robot state is never reset, pushed or exposed to the balancing policy here.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict,dataclass

import mujoco
import numpy as np


@dataclass
class RollingBall:
    launch_s: float = 6.
    from_side: int = 1  # +Y left, -Y right in the base heading at release
    radius_m: float = .03
    mass_kg: float = .04
    speed_m_s: float = .8
    distance_m: float = .30
    offset_x_m: float = 0.
    stiffness: float = 25000.
    damping: float = 35.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):raise ValueError('Nonfinite ball configuration')
        if self.from_side not in (-1,1):raise ValueError('Ball side must be -1 or 1')
        if not 1<=self.launch_s<=590 or not .015<=self.radius_m<=.06 or not .005<=self.mass_kg<=.2:
            raise ValueError('Invalid ball time, radius or mass')
        if not .1<=self.speed_m_s<=3 or not .2<=self.distance_m<=1 or abs(self.offset_x_m)>.06:
            raise ValueError('Invalid ball launch')
        if not 5000<=self.stiffness<=50000 or not 10<=self.damping<=200:raise ValueError('Invalid ball contact law')


def add_balls(world, settings):
    if not isinstance(settings,list) or len(settings)>8:raise ValueError('At most eight rolling balls')
    for i,setting in enumerate(settings):
        ball=RollingBall(**setting)
        body=ET.SubElement(world,'body',name=f'disturbance_ball_{i}',pos=f'{3+i*.2} 3 {ball.radius_m}')
        ET.SubElement(body,'freejoint',name=f'disturbance_ball_joint_{i}')
        ET.SubElement(body,'geom',name=f'disturbance_ball_geom_{i}',type='sphere',size=str(ball.radius_m),
                      mass=str(ball.mass_kg),contype='1',conaffinity='1',condim='6',priority='1',
                      friction='.25 .001 .00002',solref=f'{-ball.stiffness} {-ball.damping}',
                      rgba='1 .35 .08 1' if ball.from_side==1 else '.1 .6 1 1')


class BallDisturbances:
    def __init__(self,model,base,settings):
        self.model=model;self.base=base;self.balls=[RollingBall(**x) for x in settings];self.events=[]
        self.geom_ids={model.geom(f'disturbance_ball_geom_{i}').id:i for i in range(len(self.balls))}
        for i,ball in enumerate(self.balls):
            j=model.joint(f'disturbance_ball_joint_{i}').id
            self.events.append(dict(index=i,parameters=asdict(ball),qadr=int(model.jnt_qposadr[j]),vadr=int(model.jnt_dofadr[j]),
                                    released=False,first_robot_contact_s=None,last_robot_contact_s=None,
                                    normal_impulse_ns=0.,peak_normal_force_n=0.,robot_bodies=[],
                                    rebound_speed_m_s=0.,incoming_direction=None))

    def release(self,data,t):
        changed=False
        for ball,event in zip(self.balls,self.events):
            if event['released'] or t+1e-9<ball.launch_s:continue
            yaw=math.atan2(data.xmat[self.base][3],data.xmat[self.base][0])
            forward=np.array([math.cos(yaw),math.sin(yaw),0.]);left=np.array([-math.sin(yaw),math.cos(yaw),0.])
            direction=-ball.from_side*left
            position=data.xpos[self.base]+ball.from_side*ball.distance_m*left+ball.offset_x_m*forward
            position[2]=ball.radius_m+.0001
            q=event['qadr'];v=event['vadr'];data.qpos[q:q+3]=position;data.qpos[q+3:q+7]=[1,0,0,0]
            velocity=ball.speed_m_s*direction
            data.qvel[v:v+3]=velocity;data.qvel[v+3:v+6]=np.cross([0,0,1],velocity)/ball.radius_m
            event.update(released=True,release_s=t,release_position_m=position.tolist(),incoming_direction=direction.tolist())
            changed=True
        if changed:mujoco.mj_forward(self.model,data)

    def observe(self,data):
        m=self.model;touching=set()
        for ci,c in enumerate(data.contact):
            a,b=int(c.geom1),int(c.geom2)
            if a in self.geom_ids:idx,other=self.geom_ids[a],b
            elif b in self.geom_ids:idx,other=self.geom_ids[b],a
            else:continue
            if m.body_rootid[m.geom_bodyid[other]]!=self.base or c.efc_address<0:continue
            event=self.events[idx];wrench=np.zeros(6);mujoco.mj_contactForce(m,data,ci,wrench);force=abs(float(wrench[0]))
            if force<.001:continue
            touching.add(idx)
            if event['first_robot_contact_s'] is None:event['first_robot_contact_s']=float(data.time)
            event['last_robot_contact_s']=float(data.time);event['normal_impulse_ns']+=force*m.opt.timestep
            event['peak_normal_force_n']=max(event['peak_normal_force_n'],force)
            body=m.body(int(m.geom_bodyid[other])).name
            if body not in event['robot_bodies']:event['robot_bodies'].append(body)
        for event in self.events:
            if event['first_robot_contact_s'] is not None and event['index'] not in touching and data.time-event['last_robot_contact_s']>=.005:
                v=event['vadr'];speed=-float(np.dot(data.qvel[v:v+3],event['incoming_direction']))
                event['rebound_speed_m_s']=max(event['rebound_speed_m_s'],speed)

    def report(self):
        return [dict(**{k:v for k,v in e.items() if k not in ('qadr','vadr')},
                     contacted_robot=e['normal_impulse_ns']>.0001,
                     rebounded=e['rebound_speed_m_s']>.02) for e in self.events]
