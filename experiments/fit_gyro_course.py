"""Fit a broad floor circle while retaining continuous winding on the foot rim."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path
import numpy as np

from train_gyro_peg import WORK, POLICY, REPORT, read,write,trial
from fit_gyro_feedback import rank

WIDE=Path('configs/gyro-peg-wide-experimental.json')


def batch(jobs,workers):
    with ProcessPoolExecutor(workers) as pool:return list(pool.map(partial(trial,course=True),jobs))


def prospect(r):
    if not r['safe']:return -1000.
    a=r['floor_course']['arc']
    arc_score=0.
    if a and a['radial_cv']<.32 and a['coherence']>.7 and abs(a['sweep_deg'])>25:
        arc_score=(min(250.,a['diameter_mm'])/8 + min(130.,abs(a['sweep_deg']))/7
                   - 18*a['radial_cv']-max(0.,a['period_s']-145)/5)
    return rank(r)+arc_score


def choice(r):
    loop=r['floor_course']['best_loop']
    return (r['large_floor_pass'],r['safe'],r['passed'],bool(loop),
            min(500.,loop['diameter_mm']) if loop else 0.,prospect(r))


def fit(workers):
    rng=np.random.default_rng(413713)
    selected=read(POLICY)
    parents=[(selected['orbit'],selected['foot_winding_for_positive_mirror'])]
    for r in sorted(read(WORK/'phase-refine.json')+read(WORK/'phase-search.json'),key=rank,reverse=True):
        if all(r['params']!=p for p,_ in parents):parents.append((r['params'],r['direction']))
        if len(parents)==8:break
    jobs=[]
    for i in range(48):
        p,s=parents[i%len(parents)]
        p=dict(p)
        if i>=len(parents):
            p['pitch_bias_deg']=float(rng.uniform(-min(11,40-abs(p['pitch_deg'])),min(11,40-abs(p['pitch_deg']))))
            p['y_bias_mm']=float(rng.uniform(-min(10,25-abs(p['xy_mm'])),min(10,25-abs(p['xy_mm']))))
            p['yaw_deg']=float(np.clip(p['yaw_deg']+rng.normal(0,9),-50,50))
        jobs.append((p,'nominal',1,20.,s))
    short=batch(jobs,workers)
    write(WORK/'course-screen.json',short)
    nominees=sorted(short,key=prospect,reverse=True)[:5]
    endurance=batch([(r['params'],'nominal',1,120.,r['direction']) for r in nominees],workers)
    write(WORK/'course-endurance.json',endurance)
    winner=max(endurance,key=choice)
    write(WIDE,dict(orbit=winner['params'],foot_winding_for_positive_mirror=winner['direction'],waist_target_rad=0.,command_hz=50))


def qualify(workers):
    p=read(WIDE);cases=('nominal','peg_fine','peg_grip','peg_slick','peg_mass')
    jobs=[(p['orbit'],c,m,120.,m*p['foot_winding_for_positive_mirror']) for c in cases for m in (1,-1)]
    rows=batch(jobs,workers)
    write(WORK/'course-checks.json',rows)
    # Retain the actual featured trace for a real-time excerpt and a floor track.
    featured=max(rows[:2],key=choice)
    replay=trial((p['orbit'],'nominal',featured['mirror'],120.,featured['direction'],WORK/'course-best.json'),course=True)
    if featured!=replay:raise ValueError('Recorded ground course differs from its qualification')
    print([(r['condition'],r['mirror'],r['large_floor_pass']) for r in rows])


def replay_best():
    archive=read(REPORT)
    r=max((v for v in archive['wide_tests'] if v['condition'] in archive['held_out_conditions']),key=choice)
    policy=read(WIDE)
    replay=trial((policy['orbit'],r['condition'],r['mirror'],r['seconds'],r['direction'],WORK/'wide-best.json'),course=True)
    if replay!=r:raise ValueError('Replay differs from the archived held-out test')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=('fit','qualify','replay'))
    p.add_argument('--workers',type=int,default=8)
    a=p.parse_args()
    if a.phase=='replay':replay_best()
    else:{'fit':fit,'qualify':qualify}[a.phase](a.workers)
