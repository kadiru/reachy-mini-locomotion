"""Fit upright-start head orbits to roll around the circumference of the foot."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from reachy_locomotion.continuous_spin import Orbit, SpinController
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.rim_roll import TiltPhase, ground_course, measures
from train_rim_efficiency import MODELS, simulator
from train_target_pivot import read, write

WORK = Path('experiments/generated/gyro-peg')
POLICY = Path('configs/gyro-peg-experimental.json')
REPORT = Path('experiments/reports/motor-training-gyro-peg.json')
# Held out of fit and finalist selection. Last condition changes only integration rate.
MODELS.update(peg_slick=(.567,4.84,.955,1,'torso',.0004,1,.16,794),
              peg_grip=(.827,5.24,1.04,1,'torso',.0004,1,.15,795),
              peg_mass=(.763,5.07,1.013,2,'uniform',.0004,1,.17,796),
              peg_fine=(.70,5.,1.,1,'torso',.00025,1,.12,797))
CASES = ('peg_slick','peg_grip','peg_mass','peg_fine')


def controller_for(sim,params,condition,mirror):
    orbit = {k:v for k,v in params.items() if k!='tilt_feedback'}
    controller = SpinController(Orbit(**orbit),sim.checked_ik,sim.command_limits,mirror)
    if 'tilt_feedback' in params:
        delay,noise,seed=MODELS[condition][-3:]
        controller=TiltPhase(controller,params['tilt_feedback'],delay,noise,seed)
    return controller


def trial(args, course=False, step_audit=None):
    params, condition, mirror, seconds, sign, *record = args
    sim = simulator(condition)
    controller = controller_for(sim,params,condition,mirror)
    run = sim.run_primitive(Primitive(),seconds=seconds,controller=controller,waist_mode='hold_neutral',
                            record=bool(record),record_rim_contact=True,tilt_limit_deg=None,step_audit=step_audit)
    m = measures(run['trace'],run['config']['foot_radius_m'],sign)
    safe = run['completed'] and run['min_joint_margin_rad']>=0
    # Foot-frame winding and world-frame circle size are separate outcomes.
    peg = (safe and m['longest_rim_turns']>=3 and m['loaded_fraction']>=.8
           and m['median_lean_deg']>=12 and m['coherent_fraction']>=.9 and m['slip_per_rim_arc']<=.25)
    wide = peg and m['center_diameter_mm']>=1000*run['config']['foot_radius_m']
    result = dict(params=params,condition=condition,mirror=mirror,seconds=seconds,elapsed_s=run['elapsed_seconds'],
                  termination=run['termination'],fell=run['fell'],joint_margin_rad=run['min_joint_margin_rad'],
                  waist_command_rad=run['commanded_waist_position_range_rad'],base_yaw_deg=run['final_foot_yaw_deg'],
                  safe=bool(safe), passed=bool(peg), wide_pass=bool(wide),**m)
    if course:
        result['floor_course']=ground_course(run['trace'],2000*run['config']['foot_radius_m'])
        loop=result['floor_course']['best_loop']
        if loop:
            section=[r for r in run['trace'] if loop['start_s']<=r['time_s']<=loop['end_s']]
            loop['rim']=measures(section,run['config']['foot_radius_m'],sign,0.)
        result['large_floor_pass']=bool(peg and loop and loop['diameter_mm']>=2*1000*run['config']['foot_radius_m']
                                        and loop['rim']['loaded_fraction']>=.8 and loop['rim']['median_lean_deg']>=12)
    if record:
        write(record[0],dict(config=run['config'],trace=run['trace'],trajectory=[x['qpos'] for x in run['trajectory']],metrics=result))
    return result


def score(r):
    if not r['safe']:
        return -1000.+r['elapsed_s']
    return (min(3.,r['longest_rim_turns'])*4 + r['loaded_fraction']*10 + r['coherent_fraction']*5
            + min(28.,r['median_lean_deg']) + .50*min(70.,r['center_diameter_mm'])
            - 12*min(1.,r['slip_per_rim_arc']))


def batch(jobs, workers):
    with ProcessPoolExecutor(workers) as pool:
        return list(pool.map(trial,jobs))


def screen(workers):
    rng=np.random.default_rng(325715)
    p0=read('configs/continuous-rim-spin.json')['orbit']
    params=[p0,read('configs/continuous-spin.json')['orbit']]
    for i in range(160):
        period=int(rng.integers(10,32))
        p=asdict(Orbit(period,float(rng.uniform(19,40)),float(rng.uniform(19,40)),
                       float(rng.uniform(-122,-56)),float(rng.uniform(10,25)),float(rng.uniform(-180,180)),
                       float(rng.uniform(-30,30)),float(rng.uniform(-180,180)),float(rng.uniform(-14,2)),
                       xy_sense=(-1 if i%2 else 1), vertical_mm=float(rng.uniform(0,9)),
                       vertical_phase_deg=float(rng.uniform(-180,180))))
        params.append(p)
    rows=batch([(p,'nominal',1,7.,None) for p in params],workers)
    write(WORK/'screen.json',rows)
    return rows


def refine(rows,workers):
    rng=np.random.default_rng(18723)
    parents=sorted(rows,key=score,reverse=True)[:12]
    jobs=[]
    for i in range(100):
        p=dict(parents[i%len(parents)]['params'])
        if i>=len(parents):
            p['period_frames']=int(np.clip(p['period_frames']+rng.integers(-3,4),10,50))
            for k,sd,lo,hi in [('roll_deg',3,8,40),('pitch_deg',4,8,40),('pitch_phase_deg',11,-160,-22),
                               ('xy_mm',2.7,0,25),('xy_phase_deg',14,-240,240),('yaw_deg',5,-45,45),
                               ('yaw_phase_deg',18,-260,260),('z_mm',2,-15,4)]:
                p[k]=float(np.clip(p[k]+rng.normal(0,sd),lo,hi))
        jobs.append((p,'nominal',1,9.,None))
    result=batch(jobs,workers)
    write(WORK/'refine.json',result)
    return result


def fit(workers):
    rows=screen(workers)
    refined=refine(rows,workers)
    parents=[]
    for r in sorted(rows+refined,key=score,reverse=True):
        if not any(r['params']==x['params'] for x in parents):
            parents.append(r)
        if len(parents)==6:break
    jobs=[(r['params'],c,m,15.,r['direction']*m) for r in parents for c in ('nominal','dev') for m in (1,-1)]
    finalist=batch(jobs,workers)
    write(WORK/'finalists.json',finalist)
    def rank(i):
        group=finalist[i*4:i*4+4]
        return (sum(x['wide_pass'] for x in group),sum(x['passed'] for x in group),
                sum(x['safe'] for x in group),min(map(score,group)),np.mean(list(map(score,group))))
    win=parents[max(range(len(parents)),key=rank)]
    write(POLICY,dict(orbit=asdict(Orbit(**win['params'])),foot_winding_for_positive_mirror=win['direction'],
                      waist_target_rad=0.,command_hz=50))
    print(win,flush=True)


def qualify(workers):
    p=read(POLICY)
    jobs=[(p['orbit'],c,m,24.,p['foot_winding_for_positive_mirror']*m)
          for c in ('nominal',*CASES) for m in (1,-1)]
    rows=batch(jobs,workers)
    baseline=[]
    for name in ('continuous-rim-spin','continuous-spin'):
        orbit=read('configs/'+name+'.json')['orbit']
        baseline.extend(dict(policy=name,**r) for r in batch([(orbit,'nominal',m,24.,None) for m in (1,-1)],workers))
    if 'tilt_feedback' in p['orbit']:
        timing_off=dict(p['orbit'],tilt_feedback=dict(p['orbit']['tilt_feedback'],lock_gain=0.))
        baseline.extend(dict(policy='selected orbit with phase locking off', **r) for r in batch(
            [(timing_off,'nominal',m,24.,p['foot_winding_for_positive_mirror']*m) for m in (1,-1)],workers))
    # The held-out tests are reported after freezing, without influencing selection.
    write(WORK/'peg-checks.json',dict(policy=p, conditions={c:MODELS[c] for c in ('nominal','dev',*CASES)},
                      baseline=baseline,nominal=rows[:2],held_out=rows[2:]))
    demo=max(rows[:2],key=score)
    replay=trial((p['orbit'],'nominal',demo['mirror'],24.,demo['direction'],WORK/'best.json'))
    if demo!=replay:raise ValueError('Recorded measurements differ from qualification')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=('fit','qualify'))
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args()
    {'fit':fit,'qualify':qualify}[args.phase](args.workers)
