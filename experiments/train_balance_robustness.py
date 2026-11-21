"""Four-edge capture/return curriculum with optional physical rolling balls."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from reachy_locomotion.balance_curriculum import BalanceReturn,edge_ok,impact_metrics
from reachy_locomotion.edge_balance import balance_metrics
from reachy_locomotion.edge_linear import LinearEdgeController
from reachy_locomotion.foreaft_balance import ForeAftController,foreaft_metrics
from reachy_locomotion.primitives import PrimitiveSimulator

CACHE={}


def read(path):return json.loads(Path(path).read_text())


def evaluate(job):
    config=copy.deepcopy(job.get('config') or read('configs/motors-speed-trained.json'))
    config['timestep_s']=job.get('dt',.00025)
    if 'balls' in job:config['disturbance_balls']=job['balls']
    key=json.dumps(config,sort_keys=True)
    if key not in CACHE:CACHE[key]=PrimitiveSimulator('experiments/upstream-reachy-mini',config)
    sim=CACHE[key];policy=job['policy'];edge=job['edge'];side=-1 if edge in ('left','back') else 1
    axis='pitch' if edge in ('front','back') else 'roll'
    args=(policy['recipe'],policy['feedback'],side,sim.checked_ik,policy['linear'])
    ctrl=ForeAftController(*args) if axis=='pitch' else LinearEdgeController(*args)
    child=ctrl.child if axis=='pitch' else ctrl
    child.control_gain=policy.get('linear_gain_scale',1.)
    child.tracking_error_limit=policy.get('linear_tracking_error_limit',0.)
    child.command_rate_limit=policy.get('linear_command_rate_limit',0.)
    child.velocity_command_rate_gain=policy.get('velocity_command_rate_gain',0.)
    if not 0<=child.velocity_command_rate_gain<=200:raise ValueError('Invalid adaptive command profile')
    child.configure_observer(policy.get('observer'))
    child.configure_sensors(policy.get('sensors'))
    child.configure_phase_tuning(policy.get('phase_tuning'))
    reset=job.get('initial_angle_deg');child.hold_from_start=reset is not None
    if 'return' in policy:
        returning=BalanceReturn(child,**policy['return'])
        if axis=='pitch':ctrl.child=returning
        else:ctrl=returning
    initial={f'diagnostic_initial_{axis}_deg':None if reset is None else side*reset,
             'diagnostic_initial_angular_rate_deg_s':side*job.get('outward_rate_deg_s',0.)}
    r=sim.run_primitive(policy['recipe'],seconds=job.get('seconds',12),waist_mode='hold_neutral',controller=ctrl,
                        tilt_limit_deg=35,record=bool(job.get('record')),**initial)
    metrics=(foreaft_metrics if axis=='pitch' else balance_metrics)(r,side,child.f.target_roll_deg)
    impacts=impact_metrics(r,axis,side,child.f.target_roll_deg)
    return_started=next((x.get('return_started_s',x.get('virtual_return_started_s')) for x in reversed(r['trace'])
                         if x.get('return_started_s',x.get('virtual_return_started_s')) is not None),None)
    hold_return=bool(r['completed'] and r['settled'] and metrics['longest_strict_hold_s']>=4.9 and return_started is not None)
    full_sequence=hold_return and r['natural_upright_start']
    safe_flat=bool(r['completed'] and r['settled'] and impacts['events'] and all(x['contacted_robot'] and x['rebounded'] and x['balanced_before_impact'] for x in impacts['events']))
    impact_trial=bool(impacts['all_impacts_pass'] and (metrics['final_five_seconds_pass'] or hold_return))
    r['foreaft_balance' if axis=='pitch' else 'edge_balance']=dict(policy=policy,side=side,metrics=metrics)
    r['balance_curriculum']=dict(axis=axis,edge=edge,impacts=impacts,return_started_s=return_started,hold_return_pass=hold_return,full_sequence_pass=full_sequence,safe_flat_after_impacts=safe_flat,impact_trial_pass=impact_trial)
    r['workspace_projected_frames']=child.projected_frames
    r['attitude_sensor_assumption']='Delayed noisy attitude; optional noisy simulated body gyro; ideal head pose; optional causal model observer; no contact/ball input to policy; sensors uncalibrated'
    score=metrics['soft_hold_s']+2*metrics['longest_strict_hold_s']+10*hold_return+5*impacts['all_impacts_pass']+5*safe_flat-.05*(not r['completed'])
    if job.get('record'):
        path=Path(job['record']);path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('x') as f:json.dump(r,f,allow_nan=False)
    r.pop('trajectory',None);r.pop('trace');r.pop('terminal_state')
    return dict(id=job['id'],edge=edge,score=score,policy=policy,metrics=metrics,impacts=impacts,result=r)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--jobs',required=True);p.add_argument('--output',required=True);p.add_argument('--workers',type=int,default=6)
    a=p.parse_args();out=Path(a.output)
    if out.exists():p.error('Preserve earlier evidence')
    paths=[Path(__file__),*[Path('src/reachy_locomotion')/x for x in ['balance_curriculum.py','ball_disturbance.py','edge_observer.py','edge_linear.py','edge_balance.py','foreaft_balance.py','primitives.py','contact_probe.py','motors.py']]]
    jobs=read(a.jobs);report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(a),jobs=jobs,candidates=[],source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths})
    def save():
        out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(report,allow_nan=False)+'\n');tmp.replace(out)
    save()
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save()
            print(json.dumps(dict(id=c['id'],score=c['score'],hold=c['metrics']['longest_strict_hold_s'],termination=c['result']['termination'],sequence=c['result']['balance_curriculum']['full_sequence_pass'],impacts=c['impacts'])),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
