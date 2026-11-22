"""Release a stationary Reachy on its rim or side; audit the actual floor contacts."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

from reachy_locomotion.edge_balance import balance_metrics
from reachy_locomotion.edge_linear import LinearEdgeController
from reachy_locomotion.geometry import rotation
from reachy_locomotion.primitives import Primitive, PrimitiveSimulator, blend
from train_deep_lift import LiftCircle, POLICY, read, write
from reachy_locomotion.rim_roll import measures

WORK = Path('experiments/generated/tilted-release')
UPSTREAM = 'experiments/upstream-reachy-mini'
MOTORS = 'configs/motors-speed-trained.json'
EDGES = 'configs/balance-recovery-experimental.json'
DRIVE_S = 2.


@lru_cache(maxsize=4)
def plant(cad, dt, battery, sliding_friction=None):
    config = read(MOTORS)
    config.update(cad_shell_floor=cad, timestep_s=dt)
    if sliding_friction is not None:
        config['friction_slide_torsion_roll'][0] = sliding_friction
    if battery:
        config['motor_model'].update(voltage_v=4.95, pid_scale=.99)
    sim = PrimitiveSimulator(UPSTREAM, config)
    return sim, sim.initial_targets.copy()


class HeadMotion:
    catch_trigger = yaw_trigger = None

    def __init__(self, job):
        self.job = job
        self.pitch = 0.
        self.previous_up = np.array([0.,float(job.get('side',1))])

    def observe_attitude(self,t,pitch):
        self.pitch = pitch

    def target(self, t, roll):
        side = self.job['side']
        bias, lateral = self.job.get('setup', (0., 0.))
        # Bias is a prepared, fixed side-rest head pose. Drive starts two seconds later.
        elapsed = max(0., t-self.job.get('drive_s',DRIVE_S))
        phase = min(elapsed*self.job.get('hz',1.),self.job.get('cycles',3))*2*np.pi
        wave = np.sin(phase)
        loop = (1-np.cos(phase))/2
        drive = self.job['mode']
        angles = [side*bias, 0., 0.]
        x = 0.
        if drive == 'yaw':
            angles[2] = 22.*wave
        elif drive == 'brake':
            u = (phase/(2*np.pi)) % 1.
            angles[2] = 25.*(blend(u/.18) if u < .18 else 1-blend((u-.18)/.82))
            angles[1] = 12.*wave
        elif drive == 'pitch':
            angles[1] = 18.*wave
            x = .010*wave
        elif drive == 'loop':
            angles[1], angles[2] = 15.*wave, 24.*loop
            x = .009*wave
        scale = self.job.get('amplitude',1.)
        angles[1] *= scale
        angles[2] *= scale
        x *= scale
        pose = np.eye(4)
        pose[:3,:3] = rotation(np.radians(angles))
        pose[:3,3] = [x, side*lateral/1000, .177]
        if self.job.get('follow_up'):
            # Delayed gravity direction projected into the body's cross section.
            phi,theta = np.radians([roll,self.pitch])
            up = self.previous_up
            estimated = np.array([-np.sin(theta),np.sin(phi)*np.cos(theta)])
            self.previous_up = estimated / max(.1,np.linalg.norm(estimated))
            u = blend((t-self.job.get('drive_s',DRIVE_S))/.4)
            up = (1-u)*np.array([0.,side])+u*up
            tangent = side*np.array([up[1],-up[0]])
            shift = (lateral/1000)*up+x*tangent
            normal = -bias*up + angles[1]*tangent
            pose[:3,3] = [*shift,.177]
            pose[:3,:3] = rotation(np.radians([-normal[1],normal[0],angles[2]]))
        return pose, drive, roll


class FloorAudit:
    """Any head touch includes either penetration or a positive constraint force."""
    def __init__(self, sim, start, record=False):
        self.samples = [] if record else None
        self.steps = 0
        self.start = start
        m = sim.model
        self.floor = sim.floor
        self.kind = {}
        for i in range(m.ngeom):
            mesh = m.geom_dataid[i]
            name = m.mesh(mesh).name if mesh >= 0 else ''
            self.kind[i] = ('foot' if i == sim.foot else 'torso' if name.startswith('body_') else 'head')
        self.torso = m.body('body_down_3dprint').id
        self.first = {}
        self.first_drive = {}
        self.secs = dict(foot=0.,torso=0.,head=0.)
        self.spin = self.max_spin = self.min_spin = self.slip = 0.
        self.end_speed = 0.
        self.wrench = np.empty(6)
        self.vel = np.empty(6)
        self.jac = np.empty((3,m.nv))
        self.max_pre_speed = 0.

    def __call__(self, sim, d):
        m = sim.model
        active = set()
        torso_force = 0.
        slip_weighted = 0.
        for i,c in enumerate(d.contact):
            if self.floor not in (c.geom1,c.geom2) or c.efc_address < 0:
                continue
            g = c.geom2 if c.geom1 == self.floor else c.geom1
            mujoco.mj_contactForce(m,d,i,self.wrench)
            force = max(0., self.wrench[0])
            if c.dist > 0 and force == 0:
                continue
            kind = self.kind[g]
            mesh = m.geom_dataid[g]
            desc = dict(time_s=float(d.time), surface=m.mesh(mesh).name if mesh >= 0 else m.geom(g).name)
            self.first.setdefault(kind, desc)
            if d.time > self.start:
                self.first_drive.setdefault(kind, desc)
            active.add(kind)
            if kind == 'torso' and force > 0:
                torso_force += force
                if d.time >= self.start:
                    mujoco.mj_jac(m,d,self.jac,None,c.pos,m.geom_bodyid[g])
                    velocity = np.einsum('ij,j->i',self.jac[:2],d.qvel)
                    slip_weighted += force*np.linalg.norm(velocity)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,self.torso,self.vel,1)
        if self.start-.2 <= d.time <= self.start:
            self.max_pre_speed = max(self.max_pre_speed, float(np.linalg.norm(self.vel[:3])))
        self.steps += 1
        if self.samples is not None and self.steps % sim.substeps == 0:
            self.samples.append(dict(t=float(d.time),support=sorted(active),spin=self.spin))
        if d.time < self.start:
            return
        for kind in active:
            self.secs[kind] += sim.dt
        self.end_speed = float(np.linalg.norm(self.vel[:3]))
        self.spin += float(np.degrees(self.vel[2])*sim.dt)
        self.min_spin = min(self.min_spin,self.spin)
        self.max_spin = max(self.max_spin,self.spin)
        if torso_force > .5:
            self.slip += slip_weighted/torso_force*sim.dt


class StopHead:
    def __init__(self, controller, stop_s):
        self.controller = controller
        self.stop_s = stop_s
        self.last = None

    def __getattr__(self, name):
        return getattr(self.controller, name)

    def target(self,t,roll):
        if t < self.stop_s or self.last is None:
            self.last = self.controller.target(t,roll)
        return self.last


def balance_controller(sim, side, job):
    source = job.get('regulator_side', side)
    p = read(EDGES)['edges']['left' if source < 0 else 'right']
    p['feedback']['sensor_seed'] = job.get('seed',p['feedback']['sensor_seed'])
    if job.get('cad_model'):
        p['linear'] = read(WORK/'cad-model.json')[str(side)]
    c = LinearEdgeController(p['recipe'],p['feedback'],side,sim.checked_ik,p['linear'])
    c.hold_from_start = True
    c.configure_sensors(p.get('sensors'))
    c.configure_phase_tuning(p.get('phase_tuning'))
    c.control_gain = p.get('linear_gain_scale',1.)
    c.command_rate_limit = p.get('linear_command_rate_limit',0.)
    if 'gain' in job:
        c.control_gain = c.capture_gain = job['gain']
        c.command_rate_limit = job['rate_limit']
    if 'pitch_bias' in job:
        c.xeq[1] += job['pitch_bias']
        c.K[[1,2],:] *= job.get('pitch_gain',1.)
    return c


def run(job):
    experiment = job['experiment']
    deep = experiment == 'deep'
    sim, neutral = plant(job.get('cad',True),job.get('dt',.000125),deep)
    sim.initial_targets = neutral.copy()
    side = job.get('side',1)
    if deep:
        p = read(POLICY)
        ctl = LiftCircle(sim,'edge_battery',p['lift'],p['orbit'])
        if 'freeze_s' in job:
            ctl = StopHead(ctl,job['freeze_s'])
    elif job['mode'] == 'feedback':
        ctl = balance_controller(sim,side,job)
    else:
        ctl = HeadMotion(job)
        sim.initial_targets[sim.act] = sim.checked_ik(ctl.target(0,0)[0])
    drive_s = job.get('drive_s', DRIVE_S)
    record = bool(job.get('record'))
    audit = FloorAudit(sim, drive_s, record) if experiment == 'side' or record else None
    r = sim.run_primitive(Primitive(),seconds=job.get('seconds',6.),controller=ctl,waist_mode='hold_neutral',
            tilt_limit_deg=None,observe_landing=True,record=record,step_audit=audit,record_rim_contact=True,
            diagnostic_initial_roll_deg=None if deep else side*job['angle'])
    trace = r['trace']
    result = {k:r[k] for k in ('termination','elapsed_seconds','first_body_floor_contact','first_joint_travel_warning',
            'min_joint_margin_rad','max_closure_error_m','commanded_waist_position_range_rad')}
    result.update(job={k:v for k,v in job.items() if k!='record'}, final_tilt_deg=trace[-1]['tilt_deg'],
                  settled_flat=r['settled'], peak_tilt_deg=r['peak_tilt_deg'])
    if experiment == 'side':
        start = trace[min(round(drive_s/.02),len(trace)-1)]
        delta = np.asarray(trace[-1]['foot_xyz_m'])-start['foot_xyz_m']
        result.update(touch=audit.first,drive_touch=audit.first_drive,contact_duration_after_start_s=audit.secs,
                      peak_angular_speed_before_drive_rad_s=audit.max_pre_speed,
                      end_angular_speed_rad_s=audit.end_speed,net_axis_rotation_deg=audit.spin,range_axis_rotation_deg=[audit.min_spin,audit.max_spin],
                      foot_travel_after_start_mm=float(1000*np.linalg.norm(delta[:2])),
                      integrated_torso_slip_mm=float(1000*audit.slip))
    elif deep:
        result.update(capture_s=ctl.start,roll=measures(trace,sim.config['foot_radius_m'],1,
                      start_s=ctl.start if ctl.start is not None else 3.4))
    else:
        b = balance_metrics(r,side,ctl.f.target_roll_deg if job['mode']=='feedback' else 32.1)
        result.update(balance=b,edge_pass=bool(b['final_five_seconds_pass'] and not r['fell'] and r['min_joint_margin_rad']>=0))
    if record:
        path = WORK / (job['record']+'.json')
        path.parent.mkdir(exist_ok=True,parents=True)
        with path.open('w') as out:
            data = {k:r[k] for k in ('elapsed_seconds','config','terminal_state','trajectory')}
            data.update(metrics=result,contacts=audit.samples)
            json.dump(data,out,allow_nan=False)
    print(experiment,side,job.get('angle'),job.get('mode'),job.get('setup'),result.get('edge_pass'),
          round(result['final_tilt_deg'],1),list(result.get('touch',{})),flush=True)
    return result


def cases(group):
    if group=='tilt':
        return [dict(experiment='tilt',side=s,angle=a,mode=m,seconds=7.)
                for a in (24,28,30,31,32,33,34,36,40) for s in (-1,1) for m in ('feedback','hold')]
    if group=='side':
        return [dict(experiment='side',side=s,angle=90,mode=m,setup=b,seconds=6.)
                for b in ((0,0),(0,18),(-20,18),(20,18)) for s in (-1,1) for m in ('hold','yaw','pitch','loop')]
    if group=='fit':
        return [dict(experiment='tilt',side=s,angle=32,mode='feedback',seconds=7.,gain=g,rate_limit=rate)
                for g,rate in ((.8,120),(1.,120),(1.2,0),(1.,240),(.85,240),(1.4,240)) for s in (-1,1)]
    if group=='local':
        return [dict(experiment='tilt',side=s,angle=a,mode='feedback',seconds=7.,gain=g,rate_limit=rate,cad_model=True)
                for a in (31.5,32.,32.5) for g,rate in ((.85,160),(1.,240),(1.15,0)) for s in (-1,1)]
    if group=='follow_side':
        return [dict(experiment='side',side=s,angle=90,mode=m,amplitude=amp,setup=(-20,18),
                     seconds=15.,drive_s=8.,cycles=4,follow_up=True)
                for m,amp in (('hold',1.),('loop',.65),('loop',.85),('loop',1.),('brake',.85)) for s in (-1,1)]
    if group=='side_rest':
        return [dict(experiment='side',side=s,angle=90,mode=m,amplitude=amp,setup=(-20,18),
                     seconds=15.,drive_s=8.,cycles=4)
                for m,amp in (('hold',1.),('loop',.5),('loop',.75),('loop',1.),('brake',.5)) for s in (-1,1)]
    if group=='facet':
        return [dict(experiment='tilt',side=s,angle=32,mode='feedback',seconds=7.,gain=.9,
                     rate_limit=120,pitch_bias=bias,pitch_gain=pg)
                for bias,pg in ((-2.,.7),(-1.,.7),(0.,.65),(0.,.8),(1.,.7),(2.,.7),(-1.5,1.),(1.5,1.))
                for s in (-1,1)]
    if group=='gentle':
        return [dict(experiment='side',side=s,angle=90,mode=m,amplitude=amp,setup=(-20,18),seconds=6.)
                for m in ('brake','pitch','loop') for amp in (.2,.5) for s in (-1,1)]
    if group=='deep':
        return [dict(experiment='deep',cad=c,seconds=32.) for c in (False,True)] + [dict(experiment='deep',cad=False,seconds=16.,freeze_s=10.)]
    raise ValueError(group)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('group', choices=['tilt','side','deep','fit','gentle','side_rest','follow_side','local','facet','jobs'])
    ap.add_argument('--jobs',type=Path)
    ap.add_argument('--workers',type=int,default=5)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    jobs=read(a.jobs) if a.group=='jobs' else cases(a.group)
    with ProcessPoolExecutor(a.workers) as pool:
        results=list(pool.map(run,jobs))
    write(a.out,results)
