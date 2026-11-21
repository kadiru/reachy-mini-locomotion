"""Seeded direct policy search for natural launch and sustained edge balance."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from reachy_locomotion.edge_balance import EdgeFeedback,EdgeBalanceController,balance_metrics
from reachy_locomotion.primitives import Primitive,PrimitiveSimulator

CACHE={}
def read(path):return json.loads(Path(path).read_text())


def evaluate(job):
    config=copy.deepcopy(job.get('config') or read('configs/motors-speed-trained.json'));config['timestep_s']=job.get('dt',.0005)
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];policy=job['policy'];f=EdgeFeedback(**policy['feedback']);g=Primitive(**policy['recipe'])
    if 'linear' in policy:
        from reachy_locomotion.edge_linear import LinearEdgeController
        ctrl=LinearEdgeController(g,f,job.get('side',1),sim.checked_ik,linear=policy['linear'])
        ctrl.control_gain=policy.get('linear_gain_scale',1.)
        ctrl.tracking_error_limit=policy.get('linear_tracking_error_limit',0.)
        ctrl.command_rate_limit=policy.get('linear_command_rate_limit',0.)
        ctrl.configure_observer(policy.get('observer'))
        ctrl.configure_sensors(policy.get('sensors'))
        ctrl.configure_phase_tuning(policy.get('phase_tuning'))
        ctrl.velocity_command_rate_gain=policy.get('velocity_command_rate_gain',0.)
    else:ctrl=EdgeBalanceController(g,f,job.get('side',1),sim.checked_ik)
    ctrl.stop_when_flat=job.get('early_stop_flat',False)
    reset=job.get('diagnostic_initial_roll_deg');ctrl.hold_from_start=reset is not None
    r=sim.run_primitive(g,seconds=job.get('seconds',8),waist_mode='hold_neutral',controller=ctrl,tilt_limit_deg=None,record=bool(job.get('record',False)),finish_settle_s=.6 if ctrl.stop_when_flat else None,diagnostic_initial_roll_deg=reset)
    metrics=balance_metrics(r,job.get('side',1),f.target_roll_deg)
    r.update(edge_balance=dict(policy=policy,side=job.get('side',1),metrics=metrics),workspace_projected_frames=ctrl.projected_frames)
    score=metrics['soft_hold_s']+2*metrics['longest_strict_hold_s']-.02*(not r['completed'])
    if job.get('record'):
        Path(job['record']).write_text(json.dumps(r,allow_nan=False)+'\n')
    r.pop('trajectory',None);r.pop('trace');r.pop('terminal_state')
    return dict(id=job['id'],side=job.get('side',1),score=score,policy=policy,result=r,metrics=metrics)


PARAMS={
    'trigger_deg':(12,30,26,4), 'prediction_s':(0,.15,.04,.04), 'capture_blend_s':(.02,.25,.07,.05),
    'roll_bias_deg':(-30,40,15,18),'y_bias_mm':(-25,25,-8,12),
    'roll_kp':(-15,15,-3,5),'roll_kd_s':(-2,2,.3,.6),
    'y_kp_mm_deg':(-10,10,1,4),'y_kd_mm_s_deg':(-1.5,1.5,-.1,.5),
    'pitch_bias_deg':(-15,15,0,4),'x_bias_mm':(-15,15,-2,4),
    'pitch_kp':(-8,8,0,3),'pitch_kd_s':(-1,1,0,.3),
    'x_kp_mm_deg':(-8,8,0,3),'x_kd_mm_s_deg':(-1,1,0,.3),
}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--input')
    p.add_argument('--jobs');p.add_argument('--generations',type=int,default=8);p.add_argument('--population',type=int,default=40)
    p.add_argument('--workers',type=int,default=6);p.add_argument('--seconds',type=float,default=8);p.add_argument('--timestep',type=float,default=.0005)
    p.add_argument('--seed',type=int,default=2026112011);p.add_argument('--side',type=int,choices=[-1,1],default=1);p.add_argument('--sigma',type=float,default=1)
    p.add_argument('--diagnostic-reset',action='store_true',help='Artificial near-edge initialization; not launch evidence')
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve previous evidence')
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(a),candidates=[],
                source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),Path('src/reachy_locomotion/edge_balance.py'),Path('src/reachy_locomotion/primitives.py')]})
    def save():out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        if a.jobs:
            for c in pool.map(evaluate,read(a.jobs)):
                report['candidates'].append(c);save();print(json.dumps(dict(id=c['id'],score=c['score'],metrics=c['metrics'],termination=c['result']['termination'])),flush=True)
        else:
            policy=read(a.input) if a.input else dict(recipe=read('configs/primitive-fall-experimental.json'),feedback=asdict(EdgeFeedback()))
            keys=list(PARAMS);lo=np.array([PARAMS[k][0] for k in keys]);hi=np.array([PARAMS[k][1] for k in keys])
            mu=np.array([policy['feedback'].get(k,PARAMS[k][2]) if a.input else PARAMS[k][2] for k in keys])
            sigma=np.array([PARAMS[k][3]*a.sigma for k in keys]);rng=np.random.default_rng(a.seed);best=None
            for generation in range(a.generations):
                xs=np.clip(rng.normal(mu,sigma,(a.population,len(keys))),lo,hi);xs[0]=mu
                if best is not None:xs[1]=np.array([best['policy']['feedback'][k] for k in keys])
                jobs=[]
                for i,x in enumerate(xs):
                    pol=copy.deepcopy(policy);pol['feedback'].update({k:float(v) for k,v in zip(keys,x)})
                    jobs.append(dict(id=f'g{generation}-{i}',policy=pol,side=a.side,dt=a.timestep,seconds=a.seconds,early_stop_flat=True))
                    if a.diagnostic_reset:jobs[-1]['diagnostic_initial_roll_deg']=a.side*32.1
                cs=[]
                for c in pool.map(evaluate,jobs):cs.append(c);report['candidates'].append(c);save()
                cs.sort(key=lambda c:c['score'],reverse=True);elite=cs[:max(4,a.population//5)]
                if best is None or cs[0]['score']>best['score']:best=cs[0]
                vals=np.array([[c['policy']['feedback'][k] for k in keys] for c in elite])
                mu=.2*mu+.8*vals.mean(axis=0);sigma=np.maximum(.2*sigma+.8*vals.std(axis=0),.012*(hi-lo))
                report['best']=best;save();print(json.dumps(dict(generation=generation,best=best['id'],score=best['score'],metrics=best['metrics'],termination=best['result']['termination'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
