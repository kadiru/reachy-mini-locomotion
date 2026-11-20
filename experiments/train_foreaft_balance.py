"""Identify, train and evaluate front/back balance with unchanged physical limits."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
from reachy_locomotion.edge_balance import EdgeFeedback
from reachy_locomotion.foreaft_balance import ForeAftController,foreaft_metrics
from reachy_locomotion.primitives import PrimitiveSimulator

CACHE={}
def read(p):return json.loads(Path(p).read_text())


def evaluate(job):
    config=copy.deepcopy(job.get('config') or read('configs/motors-speed-trained.json'));config['timestep_s']=job.get('dt',.00025)
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];p=job['policy'];side=job['side'];reset=job.get('diagnostic_initial_pitch_deg')
    ctrl=ForeAftController(p['recipe'],p['feedback'],side,sim.checked_ik,p.get('linear'),job.get('excitation_seed'))
    c=ctrl.child;c.hold_from_start=reset is not None;c.stop_when_flat=job.get('early_stop_flat',False)
    c.control_gain=p.get('linear_gain_scale',1.);c.tracking_error_limit=p.get('linear_tracking_error_limit',0.);c.command_rate_limit=p.get('linear_command_rate_limit',0.)
    r=sim.run_primitive(p['recipe'],seconds=job.get('seconds',10),waist_mode='hold_neutral',controller=ctrl,
                        tilt_limit_deg=35,record=bool(job.get('record')),diagnostic_initial_pitch_deg=reset,
                        finish_settle_s=.6 if c.stop_when_flat else None)
    metrics=foreaft_metrics(r,side,c.f.target_roll_deg)
    r.update(foreaft_balance=dict(policy=p,side=side,edge='front' if side==1 else 'back',metrics=metrics),workspace_projected_frames=c.projected_frames,
             attitude_sensor_assumption='Simulated base attitude and ideal head kinematics; both delayed; physical estimator uncalibrated')
    extra={}
    if job.get('excitation_seed') is not None:
        extra['trace']=[{k:x[k] for k in ['time_s','edge_state','edge_action','edge_observation']} for x in r['trace']]
    if job.get('record'):
        out=Path(job['record']);out.parent.mkdir(parents=True,exist_ok=True)
        if out.exists():raise FileExistsError(out)
        out.write_text(json.dumps(r,allow_nan=False)+'\n')
    r.pop('trajectory',None);r.pop('trace');r.pop('terminal_state')
    score=metrics['soft_hold_s']+2*metrics['longest_strict_hold_s']-.02*(not r['completed'])
    return dict(id=job['id'],side=side,score=score,policy=p,result=r,metrics=metrics,**extra)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--jobs')
    p.add_argument('--identify',action='store_true');p.add_argument('--side',type=int,choices=[-1,1],default=1)
    p.add_argument('--count',type=int,default=64);p.add_argument('--timestep',type=float,default=.00025);p.add_argument('--workers',type=int,default=6)
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    paths=[Path(__file__),*[Path('src/reachy_locomotion')/x for x in ['foreaft_balance.py','edge_balance.py','edge_linear.py','primitives.py','motors.py']]]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(a),candidates=[],
                source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths})
    if a.identify:
        policy=dict(recipe=read('configs/primitive-catch-experimental.json'),feedback=asdict(EdgeFeedback(target_roll_deg=32.1,sensor_delay_frames=0)))
        jobs=[]
        for i in range(a.count):
            seed=2026112200+i;angle=a.side*(32.1+float(np.random.default_rng(seed).uniform(-.8,.8)))
            jobs.append(dict(id=f'id-{i}',policy=policy,side=a.side,dt=a.timestep,seconds=2,excitation_seed=seed,diagnostic_initial_pitch_deg=angle))
    elif a.jobs:jobs=read(a.jobs)
    else:p.error('Use --identify or --jobs')
    def save():
        out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(report,allow_nan=False)+'\n');tmp.replace(out)
    save()
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            if not a.identify:print(json.dumps(dict(id=c['id'],termination=c['result']['termination'],metrics=c['metrics'])),flush=True)
    if a.identify:report['rows']=[dict(id=c['id'],trace=c.pop('trace')) for c in report['candidates']]
    report['completed']=True;save();print('completed',len(jobs),'trials',flush=True)


if __name__=='__main__':main()
