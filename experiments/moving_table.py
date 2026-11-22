"""Measure a frozen edge regulator on a hinge that moves after a level-ground hold."""
import argparse
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from contextlib import nullcontext
from functools import lru_cache
from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np
import mujoco

from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.tilt_table import Excursion
from tilted_release import MOTORS, UPSTREAM, read, write
from train_release_catch import POLICY, make_controller

WORK = Path('experiments/generated/moving-table')
REPORT = Path('experiments/reports/moving-table.json')
SEEDS = (4915, 27017, 37003)
SLOW = 2.


@lru_cache(maxsize=2)
def plant(dt, base_imu=False):
    cfg = read(MOTORS)
    cfg.update(cad_shell_floor=True, tilt_table=True, timestep_s=dt)
    if base_imu:
        cfg.update(base_imu=True)
    return PrimitiveSimulator(UPSTREAM, cfg)


def case(side, angle, rate=SLOW, seed=4915, cycles=1, flat=False, dt=.000125):
    return dict(side=side, angle=float(angle), rate=float(rate), seed=seed, cycles=cycles, flat=flat, dt=dt)


def support(row, side):
    cop=row['contact_foot_xy_m']
    load=row['side_load_n']
    index=0 if side==1 else 1
    return bool(row['edge_support'] and cop is not None and -side*cop[1]>.040
                and load[index]>.5 and load[1-index]<.1)


def quiet(row, side):
    return (support(row,side) and abs(row['roll_rate_deg_s'])<=8
            and abs(row['pitch_rate_deg_s'])<=8 and row['base_speed_m_s']<.03)


def stable(rows, side):
    if not rows or not all(support(r,side) for r in rows):
        return False
    xyz=np.asarray([r['foot_xyz_m'] for r in rows])
    return bool(np.ptp([r['roll_deg'] for r in rows])<=3 and np.ptp([r['pitch_deg'] for r in rows])<=3
                and np.ptp([r['foot_yaw_deg'] for r in rows])<=5
                and np.max(np.linalg.norm(xyz-xyz[0],axis=1))<=.010)


class ContactAudit:
    """Sample the measured platform motion and the load on each side at 1 kHz."""
    def __init__(self, sim, side, start, recording=False):
        self.sim,self.side,self.start=sim,side,start
        joint=sim.model.joint('table_roll').id
        self.q=sim.model.jnt_qposadr[joint]
        self.v=sim.model.jnt_dofadr[joint]
        self.stride=round(.001/sim.dt)
        if not math.isclose(self.stride*sim.dt, .001, rel_tol=0., abs_tol=1e-12):
            raise ValueError("The contact audit needs an integer number of physics steps per millisecond")
        self.ticks=self.gap=self.longest=0
        self.onset=self.loss=None
        self.peak_rate=self.peak_angle=0.
        self.wrench=np.zeros(6)
        self.recording=recording
        self.frames=[]

    def __call__(self, sim, d):
        self.ticks+=1
        if self.ticks%self.stride:
            return
        angle=math.degrees(d.qpos[self.q])
        rate=math.degrees(d.qvel[self.v])
        if self.recording and self.ticks%(5*self.stride)==0:
            self.frames.append(np.r_[d.time,angle,rate,d.qpos])
        if d.time<self.start-1e-8:
            return
        self.peak_rate=max(self.peak_rate,abs(rate))
        self.peak_angle=max(self.peak_angle,abs(angle))
        axis=d.xmat[sim.base].reshape(3,3)[:,1]
        normal=d.geom_xmat[sim.floor].reshape(3,3)[:,2]
        directed_height=2*sim.config['foot_radius_m']*self.side*float(normal@axis)
        loaded=other=moment=0.
        if directed_height>=.010:
            for i,c in enumerate(d.contact):
                if {c.geom1,c.geom2}!={sim.foot,sim.floor} or c.efc_address<0:
                    continue
                mujoco.mj_contactForce(sim.model,d,i,self.wrench)
                f=abs(self.wrench[0])
                offset=-self.side*float(axis@(c.pos-d.xpos[sim.base]))
                if offset>0:
                    loaded+=f
                else:
                    other+=f
                moment+=f*offset
        if loaded>.5 and other<.1 and moment>.040*(loaded+other):
            self.gap=0
            return
        if not self.gap:
            self.onset=dict(time_s=float(d.time),table_deg=angle,table_rate_deg_s=rate)
        self.gap+=1
        self.longest=max(self.longest,self.gap)
        if self.gap==20 and self.loss is None:
            self.loss=self.onset


def footprint_displacement(rows, start):
    """Maximum and final distance of the force centroid from its original deck location."""
    contacts = [row for row in rows if row['contact_world_m'] is not None]
    times = np.array([row['time_s'] for row in contacts])
    baseline = (times >= start-.5) & (times <= start)
    if not baseline.any() or not (times >= start).any():
        return dict(max_footprint_travel_mm=None, final_footprint_travel_mm=None)
    points = np.array([row['contact_world_m'] for row in contacts])
    bank = np.radians([row['table_deg'] for row in contacts])
    uv = np.column_stack((points[:, 0], np.cos(bank)*points[:, 1] + np.sin(bank)*points[:, 2]))
    origin = np.median(uv[baseline], axis=0)
    distance_mm = np.linalg.norm(uv[times >= start] - origin, axis=1)*1000
    return dict(max_footprint_travel_mm=float(distance_mm.max()), final_footprint_travel_mm=float(distance_mm[-1]))


def simulate(job, recording=False, wrap_controller=None, measure_footprint=False, imu_observer=None):
    sim=plant(job['dt'], imu_observer is not None)
    side=job['side']
    ctl=make_controller(sim,side,job['seed'],read(POLICY)['sides'][str(side)])
    if wrap_controller is not None:
        ctl=wrap_controller(ctl)
    path=Excursion(job['angle'],job['rate'],cycles=job['cycles'],cruise=job.get('cruise',False))
    duration=math.ceil((path.end+path.recovery)*50)/50
    audit=ContactAudit(sim,side,path.start,recording)
    run=sim.run_primitive(Primitive(),seconds=duration,controller=ctl,waist_mode='hold_neutral',
                          diagnostic_initial_roll_deg=side*job.get('release',32.2),tilt_limit_deg=None,observe_landing=True,
                          record=recording,record_rim_contact=True,step_audit=audit,
                          table_motion=None if job['flat'] else path, imu_observer=imu_observer)
    rows=run['trace']
    pre=[r for r in rows if path.start-1<=r['time_s']<=path.start]
    post=[r for r in rows if r['time_s']>=path.start]
    tail=[r for r in rows if r['time_s']>=duration-1]
    dwells=[]
    for i in range(path.cycles):
        t=path.start+i*(2*path.leg+2*path.dwell)+path.leg+path.dwell
        dwells.append([r for r in rows if t-.5<=r['time_s']<=t])
        if i<path.cycles-1:
            level=t+path.leg+path.dwell
            dwells.append([r for r in rows if level-.5<=r['time_s']<=level])
    shell=run['first_body_floor_contact']
    start_ok=len(pre)==51 and stable(pre,side)
    held=bool(post) and audit.loss is None
    stopped=all(stable(window,side) for window in [*dwells,tail])
    quiet_windows=all(all(quiet(r,side) for r in window) for window in [pre,*dwells,tail])
    target=[0. if job['flat'] else path.sample(r['time_s'])[0] for r in rows]
    error=max(abs(v-r['table_deg']) for v,r in zip(target,rows))
    safety=run['completed'] and shell is None and run['min_joint_margin_rad']>=0
    passed=bool(start_ok and held and stopped and safety and error<.1)
    out=dict(job=job,passed=passed,start_ok=start_ok,continuous_edge=held,stable_endpoints=stopped,
             quiet_threshold_pass=bool(passed and quiet_windows),
             final_edge=support(rows[-1],side),flat_return=run['settled'],shell=shell,
             first_loss=audit.loss,max_support_gap_ms=audit.longest,
             peak_table_deg=audit.peak_angle,peak_table_rate=audit.peak_rate,
             tracking_error_deg=error,final_table_deg=rows[-1]['table_deg'],final_body_deg=rows[-1]['roll_deg'],
             joint_margin=run['min_joint_margin_rad'],termination=run['termination'],duration=duration,
             elapsed=run['elapsed_seconds'],waist=run['commanded_waist_position_range_rad'],
             pre_quiet_fraction=sum(quiet(r,side) for r in pre)/max(1,len(pre)))
    if measure_footprint:
        return dict(result=out, **footprint_displacement(rows,path.start))
    if not recording:
        return out
    track=[dict(t=r['time_s'],bank=r['table_deg'],speed=r['table_rate_deg_s'],roll=r['roll_deg'],
                head=math.degrees(p['head_rpy_rad'][0]),lift=r['side_height_difference_mm'],
                cop=r['contact_foot_xy_m'],edge=support(r,side),qpos=p['qpos']) for r,p in zip(rows,run['trajectory'])]
    return out,track,sim.config,np.asarray(audit.frames)


def key(job):
    return json.dumps(dict(job,angle=job['angle'] or 0.),sort_keys=True)


@lru_cache(maxsize=1)
def trial_cache():
    digest = hashlib.sha256()
    digest.update(f'{np.__version__} {mujoco.__version__}'.encode())
    sources = [Path(__file__), Path('experiments/train_release_catch.py'), Path('experiments/tilted_release.py'),
               *sorted(Path('src/reachy_locomotion').glob('*.py')), *sorted(Path('configs').glob('*.json'))]
    for path in sources:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return WORK / ('balance-' + digest.hexdigest()[:20])


def sweep(jobs,workers,pool=None):
    directory = trial_cache()
    directory.mkdir(parents=True, exist_ok=True)
    unique_jobs = {key(job): job for job in jobs}
    paths = {identifier: directory / (hashlib.sha256(identifier.encode()).hexdigest()+'.json')
             for identifier in unique_jobs}
    records = {identifier: read(path) for identifier, path in paths.items() if path.exists()}
    missing = [identifier for identifier in unique_jobs if identifier not in records]
    with (nullcontext(pool) if pool is not None else ProcessPoolExecutor(workers)) as executor:
        for identifier, result in zip(missing, executor.map(simulate,(unique_jobs[i] for i in missing))):
            records[identifier] = result
            with NamedTemporaryFile(dir=directory,mode='w',delete=False) as out:
                json.dump(result,out,allow_nan=False)
            Path(out.name).replace(paths[identifier])
    return [records[key(job)] for job in jobs]


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--angles',type=float,nargs='+',default=[0., 5., -5., 10., -10.])
    p.add_argument('--sides',type=int,nargs='+',default=[-1,1])
    p.add_argument('--rates',type=float,nargs='+',default=[SLOW])
    p.add_argument('--seeds',type=int,nargs='+',default=[4915])
    p.add_argument('--cycles',type=int,default=1)
    p.add_argument('--workers',type=int,default=4)
    a=p.parse_args()
    rows=sweep([case(s,b,r,n,a.cycles) for b in a.angles for s in a.sides for r in a.rates for n in a.seeds],a.workers)
    for r in rows:
        print(r['job'],r['passed'],r['start_ok'],round(r['pre_quiet_fraction'],2),r['first_loss'],r['shell'],round(r['final_body_deg'],2),r['stable_endpoints'])
