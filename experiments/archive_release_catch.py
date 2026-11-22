"""Archive every release test, the frozen controller, metered searches and exact film inputs."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
from pathlib import Path

import numpy as np

from train_release_catch import WORK, OLD, POLICY, RESERVED, job, batch, read, write

REPORT=Path('experiments/reports/motor-training-cad-release-catch.json')


def chapters():
    panels=[['Original right','original-right'],['Trained right','trained-right'],['Still target on right','still-right']]
    timeline=[dict(title='Right edge at 32.2°  /  released from rest',start=0.,stop=8.,speed=1.,panels=panels),
              dict(title='First 0.5 s  /  same original regulator mirrored left',start=0.,stop=.5,speed=.125,
                   panels=[['Original right','original-right'],['Mirrored left','same-left'],['Trained right','trained-right']])]
    for i,(angle,rate) in enumerate(RESERVED):
        state='outward' if rate>0 else 'inward' if rate<0 else 'at rest'
        label=f'{abs(rate):.1f}°/s {state}' if rate else state
        timeline.append(dict(title=f'Fresh release {i+1} of 10  /  {angle:g}°  /  {label}',
            panels=[['Original right',f'prior-right-{i}'],['Trained right',f'fresh-1-{i}'],['Trained left',f'fresh--1-{i}']],
            start=0.,stop=8.,speed=2.))
    return timeline


def early(clip):
    data=read(WORK/(clip+'.json'))
    s=data['result']['side']
    samples=[]
    for i in (0,1,2,5,12,25):
        row=data['trace'][i]
        head=data['trajectory'][i]
        samples.append(dict(time_s=row['time_s'],outward_roll_deg=s*row['roll_deg'],
                            outward_roll_rate_deg_s=s*row['roll_rate_deg_s'],
                            head_roll_deg=float(np.degrees(head['head_rpy_rad'][0])),
                            foot_contact_mm=None if row['contact_foot_xy_m'] is None else
                            [v*1000 for v in row['contact_foot_xy_m']]))
    return samples


def archive():
    qual=read(WORK/'qualification.json')
    digest=hashlib.sha256(POLICY.read_bytes()).hexdigest()
    if digest!=qual['policy_sha256']:
        raise ValueError('Frozen policy changed after evaluation')
    timeline=chapters()
    all_clips=sorted({p[1] for chapter in timeline for p in chapter['panels']})
    results={k:read(WORK/(k+'.json'))['result'] for k in all_clips}
    poses=[read(WORK/(k+'.json'))['trajectory'][0] for _,k in timeline[0]['panels']]
    for a in poses[1:]:
        for key in ('qpos','qvel'):
            np.testing.assert_array_equal(a[key], poses[0][key])
    measured=['screen','curriculum','impulse','hold']
    compute=[read(WORK/(f+'-meter.json')) for f in measured]
    costs=[]
    stages=[['screen'],['identification','plane_screen','varied','finalists'],['expand','select'],['neutral-trim','filters','validation']]
    # Store training conditions/outcomes once per policy without repeating full system matrices.
    searches={}
    for i,stage in enumerate(stages):
        times=[]
        for name in stage:
            data=read(WORK/(name+'.json'))
            if name in ('identification','neutral-trim'):
                searches[name]=data
                times.extend(v['elapsed'] for v in data['rollouts'])
            else:
                searches[name]=[]
                for v in data:
                    times.extend(r['elapsed'] for r in v['results'])
                    searches[name].append(dict(side=v.get('side',1),policy={k:t for k,t in v['policy'].items() if k!='model'},
                        results=[{k:r[k] for k in ('angle','outward_rate','seed','dt','seconds','elapsed','strict','edge',
                                                    'outcome','safe_flat','joint')} for r in v['results']]))
        m=compute[i]
        if m['status']!='complete' or m['returncode']!=0:
            raise ValueError('Incomplete training receipt')
        costs.append(dict(stage=measured[i],meter_index=i,rollouts=len(times),simulated_rollout_s=sum(times),
                          workers=6,cpu=m['hardware']['cpu'],wall_s=m['wall_s'],cpu_s=m['cpu_s'],gross_wh=m['gross']['kwh']*1000))
    write(REPORT,dict(evidence='uncalibrated MuJoCo simulation',sha256={str(POLICY):digest},qualification=qual,
                      neutral_verification=read(WORK/'neutral-verification.json'),
                      supported_model=read(WORK/'supported-model.json'),ablation=read(WORK/'ablation.json'),early={k:early(k) for k in ('original-right','same-left','trained-right','still-right')},
                      film=dict(chapters=timeline,results=results), training=searches,compute=compute,
                      training_cost=costs,failed_start=read(WORK/'id-meter.json')))
    write(WORK/'film.json',timeline)


def replay(workers, render=True):
    from film_release_catch import film
    archive=read(REPORT)
    if archive['sha256'][str(POLICY)]!=hashlib.sha256(POLICY.read_bytes()).hexdigest():
        raise ValueError('Controller differs from archived policy')
    cfg=read(POLICY)['sides']
    jobs=[]
    specs=archive['film']['results']
    for k,r in specs.items():
        s=r['side']
        p=(None if k=='still-right' else dict(regulator_side=1) if k=='same-left' else
           OLD['right'] if k.startswith(('original','prior-right')) else cfg[str(s)])
        jobs.append(job(s,r['angle'],r['outward_rate'],p,r['seconds'],r['seed'],r['dt'],k if render else None))
    with ProcessPoolExecutor(workers) as pool:
        rows=batch(pool,jobs)
    for k,r in zip(specs,rows):
        if r!=specs[k]:
            raise ValueError(f'Replay differs from archived simulation for {k}')
    if render:
        write(WORK/'qualification.json',archive['qualification'])
        film(archive['film']['chapters'],WORK/'comparison-replay.mp4')
    print(f'Verified {len(specs)} recorded runs against the archive.')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('task',choices=['archive','replay','verify'])
    ap.add_argument('--workers',type=int,default=6)
    a=ap.parse_args()
    if a.task=='archive':
        archive()
    else:
        replay(a.workers,render=a.task=='replay')
