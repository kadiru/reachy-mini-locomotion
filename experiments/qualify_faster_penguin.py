"""Freeze a sensor-age-aware gait and qualify all seen models plus a fresh split."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from train_faster_penguin import selected_policy,make_job
from train_faster_balance import cases,read
from qualify_policy_bank import evaluate

if __name__=='__main__':
    source=read('experiments/reports/motor-training-faster-balance-fallback-launch-2026-11-19.json');assert source['completed']
    p=selected_policy();p['delayed_sensor_fallback']=source['best']['primitive'];p.update(target_step_turn_deg=12,max_repeats=4)
    p['sequencing']=dict(min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0.)
    policy=Path('configs/penguin-faster-trained.json')
    out=Path('experiments/reports/motor-training-faster-balance-qualified-2026-11-19.json')
    if out.exists() or policy.exists():raise FileExistsError('Preserve frozen evidence')
    policy.write_text(json.dumps(p,indent=2)+'\n')
    jobs=[make_job(j,p,seconds=24) for j in cases()]
    rng=np.random.default_rng(2026111991);nominal=cases()[0]
    for i in range(8):
        model=copy.deepcopy(nominal);model['id']=f'unseen-{i}'
        model['config']['friction_slide_torsion_roll'][0]=float(rng.uniform(.55,.9))
        model['config']['motor_model'].update(voltage_v=float(rng.uniform(4.7,5.5)),pid_scale=float(rng.uniform(.9,1.1)),
                 delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
        for side in [-1,1]:
            j=make_job(model,p,seconds=24);j['side']=side;j['heading_seed']=int(rng.integers(0,100000))
            sensor=dict(sensor_delay_frames=int(rng.integers(1,3)),sensor_noise_deg=float(rng.uniform(.05,.15)),sensor_seed=int(rng.integers(0,100000)))
            for family in ['primary','fallback','delayed_sensor_fallback']:j['policy'][family]['feedback'].update(sensor)
            jobs.append(j)
    for side in [-1,1]:
        j=make_job(nominal,p,seconds=36,cycles=4);j['side']=side;j['id']='nominal-four-pairs';jobs.append(j)
    j=make_job(nominal,p,seconds=36,cycles=4,dt=.000125);j['side']=1;j['id']='nominal-four-pairs-0.125ms';jobs.append(j)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,frozen_policy=p,seed=2026111991,
                prior_fresh_cases_are_now_development=True,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(case=c['id'],side=c['side'],passed=c['passed'],termination=c['result']['termination'],
                complete=c['penguin']['completed'],speed=c['penguin']['speed_mm_s'],forward=c['penguin']['forward_mm'],heading=c['penguin']['max_completed_cycle_heading_deg'])),flush=True)
    report['completed']=True;save()
