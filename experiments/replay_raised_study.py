"""Reproduce the raised-start trials in the ball-speed and high-tilt study."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path

from probe_high_tilt import run
from train_balance_robustness import evaluate


def replay(task):
    group, row, path, policy = task
    if group == 'edge':
        job = dict(id=path.stem, policy=policy, edge=row['edge'], seconds=10, dt=.000125, tilt_limit_deg=80,
                   initial_angle_deg=row['angle'], outward_rate_deg_s=row['rate'], record=str(path))
        if path.exists():
            result = json.loads(path.read_text())
            final_hold = result['edge_balance']['metrics']['final_five_seconds_pass']
        else:
            candidate = evaluate(job)
            result = candidate['result']
            final_hold = candidate['metrics']['final_five_seconds_pass']
        observed = dict(termination=result['termination'], end_s=result['elapsed_seconds'],
                        peak_tilt_deg=result['peak_tilt_deg'], settled_flat=result['settled'], final_hold=final_hold)
    else:
        if not path.exists():
            run(dict(row['case'],tilt_limit_deg=80,record=str(path)))
        result = json.loads(path.read_text())
        observed = {k: result[k] for k in ('peak_tilt_deg','settled','termination','elapsed_seconds','completed',
                                            'min_joint_margin_rad','max_closure_error_m','waist_peak_torque_nm')}
        observed['final_roll_deg'] = math.degrees(result['terminal_state']['foot_rpy_rad'][0])
        if 'trigger_s' in row:
            observed['trigger_s'] = None if row['case'].get('hold_setup') else result['catch_trigger_s']
    differences = []
    for key,value in observed.items():
        expected = row[key]
        if isinstance(expected,float) and isinstance(value,float):
            if not math.isclose(expected,value,rel_tol=1e-9,abs_tol=1e-10):
                differences.append(key)
        elif expected != value:
            differences.append(key)
    if differences:
        raise ValueError(f'Raised-start replay differs for {path.stem}: {differences}')
    return path.stem


def panels(study, folder):
    policy = json.loads(Path(study['policy']).read_text())['edges']
    tasks, groups = [], {}
    for i, row in enumerate(study['raised_with_edge_policy']):
        ident = f'edge-{i:03d}'
        path = folder/(ident+'.json')
        tasks.append(('edge',row,path,policy[row['edge']]))
        groups.setdefault('Frozen edge controller',[]).append(
            (ident,row['edge'],row['angle'],row['rate'],row['termination'],row['peak_tilt_deg'],'original edge feedback',.000125))
    for row in study['catch_probes']:
        case, ident = row['case'], row['id']
        path = folder/(ident+'.json')
        tasks.append((row['batch'],row,path,None))
        mode = 'held pose' if case.get('hold_setup') else f"trigger {case['trigger_at']:g}°" if 'trigger_at' in case else 'early catch'
        groups.setdefault(row['batch'],[]).append(
            (ident,'right' if case['side']==1 else 'left',case['angle'],case['rate'],row['termination'],
             row['peak_tilt_deg'],mode,case['dt']))
    chapters = []
    for label, rows in groups.items():
        for offset in range(0,len(rows),4):
            items = []
            for ident,edge,angle,rate,termination,peak,mode,dt in rows[offset:offset+4]:
                outcome = {'nonfoot_floor_contact':'BODY HIT FLOOR','completed':'SETTLED FLAT',
                           'actual_joint_limit_exceeded':'JOINT LIMIT'}.get(termination,'IK TARGET REJECTED')
                items.append(dict(label=f'{edge.title()} edge   |   {angle:g}°   |   {rate:+g}°/s outward',
                    detail=f'{mode}   ·   peak {peak:.2f}°   ·   {dt*1000:g} ms',
                    outcome=f'{ident}   |   {outcome}',color='#83e2ac' if termination=='completed' else '#ff947c',
                    record=str(folder/(ident+'.json'))))
            chapters.append(dict(title=f'{label}   |   trials {offset+1}–{min(offset+4,len(rows))} of {len(rows)}',
                subtitle='Artificial raised starts   |   Angle and outward angular rate are initial conditions   |   Waist target 0°',
                panels=items+[None]*(4-len(items))))
    return tasks, chapters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    args.work.mkdir(parents=True,exist_ok=True)
    tasks, chapters = panels(json.loads(args.report.read_text()),args.work)
    (args.work/'raised-film.json').write_text(json.dumps(chapters,indent=2))
    with ProcessPoolExecutor(args.workers) as pool:
        for i, _ in enumerate(pool.map(replay,tasks),1):
            if i % 50 == 0:
                print(f'Verified {i}/{len(tasks)}',flush=True)
    print(f'Verified all {len(tasks)} raised-start trials in {len(chapters)} chapters')


if __name__ == '__main__':
    main()
