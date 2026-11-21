"""Physical rolling and airborne ball disturbances for the balance simulator.

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
        if not .1<=self.speed_m_s<=12 or not .2<=self.distance_m<=1 or abs(self.offset_x_m)>.06:
            raise ValueError('Invalid ball launch')
        if not 5000<=self.stiffness<=50000 or not 10<=self.damping<=200:raise ValueError('Invalid ball contact law')


@dataclass
class ThrownBall(RollingBall):
    """Height is the centre height above the floor. Speed is its horizontal component."""
    launch_height_m: float = .16
    vertical_speed_m_s: float = 0.

    def __post_init__(self):
        super().__post_init__()
        if not self.radius_m < self.launch_height_m <= .6 or abs(self.vertical_speed_m_s) > 4:
            raise ValueError('Invalid airborne ball height or vertical velocity')


@dataclass
class DroppedBall:
    """Vertical free fall above the head, with horizontal offsets in the base yaw frame."""
    drop_height_m: float
    offset_x_m: float = 0.
    offset_y_m: float = 0.
    launch_s: float = 5.
    radius_m: float = .015
    mass_kg: float = .010
    stiffness: float = 25000.
    damping: float = 35.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Nonfinite drop configuration')
        if not (.35<=self.drop_height_m<=1 and abs(self.offset_x_m)<=.15 and abs(self.offset_y_m)<=.15
                and 1<=self.launch_s<=590 and .015<=self.radius_m<=.06 and .005<=self.mass_kg<=.2
                and 5000<=self.stiffness<=50000 and 10<=self.damping<=200):
            raise ValueError('Invalid overhead drop')


def ball_config(settings):
    if 'drop_height_m' in settings:
        return DroppedBall(**settings)
    if 'launch_height_m' in settings:
        return ThrownBall(**settings)
    if 'vertical_speed_m_s' in settings:
        raise ValueError('An airborne release requires a launch height')
    return RollingBall(**settings)


def add_balls(world, settings):
    if not isinstance(settings,list) or len(settings)>10:raise ValueError('At most ten disturbance balls')
    for i,setting in enumerate(settings):
        ball=ball_config(setting)
        body=ET.SubElement(world,'body',name=f'disturbance_ball_{i}',pos=f'{3+i*.2} 3 {ball.radius_m}')
        if isinstance(ball,DroppedBall):
            body.set('pos',f'{3+i*.2} 3 1')
            body.set('gravcomp','1')
        ET.SubElement(body,'freejoint',name=f'disturbance_ball_joint_{i}')
        ET.SubElement(body,'geom',name=f'disturbance_ball_geom_{i}',type='sphere',size=str(ball.radius_m),
                      mass=str(ball.mass_kg),contype='1',conaffinity='1',condim='6',priority='1',
                      friction='.25 .001 .00002',solref=f'{-ball.stiffness} {-ball.damping}',
                      rgba='1 .35 .08 1' if getattr(ball,'from_side',1)==1 else '.1 .6 1 1')


class BallDisturbances:
    def __init__(self,model,base,settings):
        self.model=model;self.base=base;self.balls=[ball_config(x) for x in settings];self.events=[]
        self.precontact_velocity = [None] * len(self.balls)
        self.floor = model.geom('floor').id
        self.geom_ids={model.geom(f'disturbance_ball_geom_{i}').id:i for i in range(len(self.balls))}
        for i,ball in enumerate(self.balls):
            j=model.joint(f'disturbance_ball_joint_{i}').id
            if isinstance(ball,DroppedBall):
                model.body_gravcomp[model.jnt_bodyid[j]]=1.
                head = model.site_bodyid[model.site('head').id]
                colliders = np.flatnonzero((model.geom_bodyid==head) & (model.geom_contype!=0))
                if len(colliders)!=1:
                    raise ValueError('Expected one solid head shell for overhead drops')
                self.head_shell = int(colliders[0])
            self.events.append(dict(index=i,parameters=asdict(ball),qadr=int(model.jnt_qposadr[j]),vadr=int(model.jnt_dofadr[j]),
                                    released=False,first_robot_contact_s=None,last_robot_contact_s=None,
                                    normal_impulse_ns=0.,peak_normal_force_n=0.,robot_bodies=[],
                                    incoming_speed_m_s=None, incoming_velocity_m_s=None, incoming_3d_speed_m_s=None,
                                    first_floor_contact_s=None, first_contact_height_m=None, first_robot_body=None,
                                    rebound_speed_m_s=0.,incoming_direction=None))

    def release(self,data,t):
        changed=False
        for ball,event in zip(self.balls,self.events):
            if event['released'] or t+1e-9<ball.launch_s:continue
            yaw=math.atan2(data.xmat[self.base][3],data.xmat[self.base][0])
            forward=np.array([math.cos(yaw),math.sin(yaw),0.]);left=np.array([-math.sin(yaw),math.cos(yaw),0.])
            if isinstance(ball,DroppedBall):
                position = data.geom_xpos[self.head_shell]+ball.offset_x_m*forward+ball.offset_y_m*left
                position[2] = ball.drop_height_m
                direction=np.array([0.,0.,-1.])
                velocity=np.zeros(3)
                spin=np.zeros(3)
                self.model.body_gravcomp[self.model.body(f"disturbance_ball_{event['index']}").id]=0.
            else:
                direction=-ball.from_side*left
                position=data.xpos[self.base]+ball.from_side*ball.distance_m*left+ball.offset_x_m*forward
                thrown = isinstance(ball, ThrownBall)
                position[2] = ball.launch_height_m if thrown else ball.radius_m+.0001
                velocity=ball.speed_m_s*direction
                if thrown:
                    velocity[2] = ball.vertical_speed_m_s
                spin=0. if thrown else np.cross([0,0,1],velocity)/ball.radius_m
            q=event['qadr'];v=event['vadr'];data.qpos[q:q+3]=position;data.qpos[q+3:q+7]=[1,0,0,0]
            data.qvel[v:v+3]=velocity
            data.qvel[v+3:v+6]=spin
            event.update(released=True,release_s=t,release_position_m=position.tolist(),incoming_direction=direction.tolist())
            self.precontact_velocity[event["index"]] = velocity.copy()
            changed=True
        if changed:mujoco.mj_forward(self.model,data)

    def observe(self,data):
        m=self.model;touching=set()
        for ci,c in enumerate(data.contact):
            a,b=int(c.geom1),int(c.geom2)
            if a in self.geom_ids:idx,other=self.geom_ids[a],b
            elif b in self.geom_ids:idx,other=self.geom_ids[b],a
            else:continue
            event=self.events[idx]
            if not event['released'] or c.efc_address<0:continue
            if other == self.floor:
                if event['first_floor_contact_s'] is None:
                    wrench=np.zeros(6)
                    mujoco.mj_contactForce(m,data,ci,wrench)
                    if abs(wrench[0]) >= .001:
                        event['first_floor_contact_s']=float(data.time)
                continue
            if m.body_rootid[m.geom_bodyid[other]]!=self.base:continue
            wrench=np.zeros(6);mujoco.mj_contactForce(m,data,ci,wrench);force=abs(float(wrench[0]))
            if force<.001:continue
            touching.add(idx)
            if event['first_robot_contact_s'] is None:
                event['first_robot_contact_s'] = float(data.time)
                velocity = self.precontact_velocity[idx]
                # Preserve the historical horizontal speed. Airborne reports also use the 3-D speed.
                event['incoming_speed_m_s'] = math.hypot(*velocity[:2])
                event['incoming_velocity_m_s'] = velocity.tolist()
                event['incoming_3d_speed_m_s'] = float(np.linalg.norm(velocity))
                event['first_contact_height_m'] = float(c.pos[2])
                event['first_robot_body'] = m.body(int(m.geom_bodyid[other])).name
            event['last_robot_contact_s']=float(data.time);event['normal_impulse_ns']+=force*m.opt.timestep
            event['peak_normal_force_n']=max(event['peak_normal_force_n'],force)
            body=m.body(int(m.geom_bodyid[other])).name
            if body not in event['robot_bodies']:event['robot_bodies'].append(body)
        for event in self.events:
            if event['released'] and event['first_robot_contact_s'] is None:
                v = event['vadr']
                self.precontact_velocity[event['index']] = data.qvel[v:v+3].copy()
            if event['first_robot_contact_s'] is not None and event['index'] not in touching and data.time-event['last_robot_contact_s']>=.005:
                v=event['vadr'];speed=-float(np.dot(data.qvel[v:v+3],event['incoming_direction']))
                event['rebound_speed_m_s']=max(event['rebound_speed_m_s'],speed)

    def report(self):
        return [dict(**{k:v for k,v in e.items() if k not in ('qadr','vadr')},
                     contacted_robot=e['normal_impulse_ns']>.0001,
                     rebounded=e['rebound_speed_m_s']>.02) for e in self.events]
