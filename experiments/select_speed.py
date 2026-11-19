"""Select speed-first candidates on long/fine development runs, then freeze.

Fresh model variations are evaluated only after selection, never used to choose
the winner. Whole-cycle speed uses the fixed heading at startup, not path length.
"""
import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.sway import Gait, SwaySimulator
from train_speed import PROFILES, evaluate, metrics


def candidates(path, count):
    report = json.loads(Path(path).read_text())
    raw = report['initial'] + [r for h in report['history'] for r in h['candidates']]
    unique = {json.dumps(r['gait'], sort_keys=True): r for r in raw if r['metrics']['feasible']}
    ranked = sorted(unique.values(), key=lambda r:r['metrics']['speed_mm_s'], reverse=True)
    selected = ranked[:count]
    fast = [r for r in ranked if r['metrics']['speed_mm_s'] >= .75*ranked[0]['metrics']['speed_mm_s']]
    selected += sorted(fast, key=lambda r:r['metrics']['heading_deg'])[:2]
    strokes = [r for r in ranked if r['gait'].get('yaw_stroke_fraction') or r['gait'].get('roll_stroke_fraction')]
    selected += strokes[:2]
    return list({json.dumps(r['gait'],sort_keys=True):r for r in selected}.values())


def stress_job(payload):
    label, recipe, config, seconds = payload
    g = Gait(**recipe['gait']); period = round(50/g.frequency_hz)/50
    sim = SwaySimulator('experiments/upstream-reachy-mini', config)
    r = sim.run(g, cycles=math.ceil(seconds/period-1e-10), warmup_cycles=3,
                feedback=recipe['feedback'], sensor=json.loads(Path('configs/heading-sensor-assumed.json').read_text()) if recipe['feedback'] else None)
    return dict(case=label, result=r, metrics=metrics(r))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--existing', required=True); p.add_argument('--sharp', required=True)
    p.add_argument('--output', required=True); args=p.parse_args(); path=Path(args.output)
    if path.exists():p.error('Choose a new report')
    pool_candidates = candidates(args.existing, 6) + candidates(args.sharp, 8)
    feedback = json.loads(Path('configs/heading-trained.json').read_text())
    feedback.update(heading_deadband_deg=10., startup_ramp_cycles=1.)
    controllers = {'open':None, 'deadband10':feedback}
    recipes=[]
    for i,c in enumerate(pool_candidates):
        for control, fb in controllers.items():
            recipes.append(dict(id=f'{i}-{control}', gait=c['gait'], profile=c['profile'], feedback=fb))
    recipes.append(dict(id='previous_fast',gait=json.loads(Path('configs/sway-progress-fast.json').read_text()),profile='existing',feedback=None))
    recipes.append(dict(id='previous_heading',gait=json.loads(Path('configs/sway-heading-trained.json').read_text()),profile='existing',feedback=json.loads(Path('configs/heading-trained.json').read_text())))
    receipt=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                 objective='fastest sustained fixed-axis forward speed with simulation no-fall validity and <=20 deg heading including startup',
                 fine_timestep_agreement_relative_limit=.15, development=[], fine=[], endurance=[],
                 fresh_stress_seed=2026111913, stress=[])
    def save():path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
    def job(recipe, seconds, dt):return recipe['gait'],recipe['profile'],seconds,dt,recipe['feedback']
    with ProcessPoolExecutor(max_workers=4) as pool:
        jobs=[(r,dt) for r in recipes for dt in [.002,.001]]
        grouped={}
        for (recipe,dt),r in zip(jobs,pool.map(evaluate,[job(rec,20.,dt) for rec,dt in jobs])):
            entry=dict(recipe=recipe,dt=dt,evaluation=r);receipt['development'].append(entry)
            grouped.setdefault(recipe['id'],[]).append(entry);save()
            print(json.dumps(dict(stage='20s',id=recipe['id'],profile=recipe['profile'],dt=dt,**r['metrics'])),flush=True)
        eligible=[rows for rows in grouped.values() if all(x['evaluation']['metrics']['feasible'] for x in rows)]
        eligible.sort(key=lambda rows:min(x['evaluation']['metrics']['speed_mm_s'] for x in rows),reverse=True)
        shortlisted=[rows[0]['recipe'] for rows in eligible[:8]]
        if not shortlisted:raise RuntimeError('No candidate meets the long-run constraint')
        for recipe,r in zip(shortlisted,pool.map(evaluate,[job(rec,20.,.0005) for rec in shortlisted])):
            at1=next(x['evaluation'] for x in grouped[recipe['id']] if x['dt']==.001)
            speed=r['metrics']['speed_mm_s'];difference=abs(speed-at1['metrics']['speed_mm_s'])/max(abs(speed),abs(at1['metrics']['speed_mm_s']),1e-9)
            entry=dict(recipe=recipe,evaluation=r,dt=.0005,relative_difference_from_1ms=difference,
                       accepted=r['metrics']['feasible'] and difference<=.15,
                       fine_speed_min_mm_s=min(speed,at1['metrics']['speed_mm_s']))
            receipt['fine'].append(entry);save()
            print(json.dumps(dict(stage='fine',id=recipe['id'],agreement=difference,**r['metrics'])),flush=True)
        fine=sorted([r for r in receipt['fine'] if r['accepted']],key=lambda r:r['fine_speed_min_mm_s'],reverse=True)
        if not fine:raise RuntimeError('No candidate passes fine-timestep verification')
        for entry,r in zip(fine[:4],pool.map(evaluate,[job(e['recipe'],40.,.001) for e in fine[:4]])):
            receipt['endurance'].append(dict(recipe=entry['recipe'],evaluation=r,
                                             accepted=r['metrics']['feasible'],
                                             selection_speed_mm_s=min(entry['fine_speed_min_mm_s'],r['metrics']['speed_mm_s'])))
            save();print(json.dumps(dict(stage='40s',id=entry['recipe']['id'],**r['metrics'])),flush=True)
        survivors=[r for r in receipt['endurance'] if r['accepted']]
        if not survivors:raise RuntimeError('No candidate meets the 40-second heading/no-fall requirement')
        best=max(survivors,key=lambda r:r['selection_speed_mm_s']);recipe=best['recipe']
        receipt['frozen_recipe']=recipe;receipt['best_development']=best;save()
        Path('configs/sway-speed-trained.json').write_text(json.dumps(recipe['gait'],indent=2)+'\n')
        config=json.loads(Path('configs/motors-datasheet.json').read_text());config['motor_model'].update(PROFILES[recipe['profile']])
        config['timestep_s']=.001
        config['notes'] += ' Speed-first software target profile: '+recipe['profile']+'; physical torque/mass/contact bounds unchanged. This is not an identified firmware profile.'
        Path('configs/motors-speed-trained.json').write_text(json.dumps(config,indent=2)+'\n')
        Path('configs/heading-speed-trained.json').write_text(json.dumps(recipe['feedback'],indent=2)+'\n')
        rng=np.random.default_rng(receipt['fresh_stress_seed']);jobs=[]
        for i in range(8):
            c=copy.deepcopy(config);c['foot_radius_m']=float(rng.uniform(.054,.058))
            c['friction_slide_torsion_roll'][0]=float(rng.uniform(.5,.95))
            c['motor_model'].update(voltage_v=float(rng.uniform(4.7,6)),pid_scale=float(rng.uniform(.85,1.15)),
                                    delay_frames=int(rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2],
                                    passive_friction_nm=float(rng.uniform(.0007,.002)),joint_friction_nm=float(rng.uniform(.007,.015)),
                                    joint_armature_kg_m2=float(rng.uniform(.00007,.00015)))
            for dt in [.001,.0005]:
                c=copy.deepcopy(c);c['timestep_s']=dt;jobs.append((f'fresh-{i}-{dt}',recipe,c,20.))
        for r in pool.map(stress_job,jobs):
            receipt['stress'].append(r);save()
            print(json.dumps(dict(stage='fresh',case=r['case'],**r['metrics'])),flush=True)
    receipt['completed']=True;save()


if __name__=='__main__':main()
