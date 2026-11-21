"""Summarize single-ball emergency returns and prepare their comparison film."""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import json
from pathlib import Path

COLORS = {'settled flat':'#83e2ac', 'edge recovery':'#83e2ac', 'floor contact':'#ff947c',
          'joint stop':'#ffd184', 'IK stop':'#ffd184', 'unqualified before hit':'#bdadf1',
          'completed':'#75cde2', '80° backstop':'#ffd184', 'control held edge':'#83e2ac',
          'control triggered':'#ffd184','control failed':'#ff947c','flat without rebound':'#75cde2'}


def result_row(job, recording):
    policy = job['policy']['return']
    run = json.loads(recording.read_text())
    event = run['ball_disturbances'][0]
    evaluation = run['balance_curriculum']
    impact = evaluation['impacts']['events'][0]
    contact = event['first_robot_contact_s']
    trigger = evaluation['return_started_s']
    decision = next((r for r in run['trace'] if r['time_s']==trigger),None)
    if decision is not None and not decision['emergency_return']:
        raise ValueError(job['id']+' used a timed return')
    eligible = bool(impact['balanced_before_impact'] and event['contacted_robot']
                    and (trigger is None or trigger >= contact))
    diagnostics = run['motor_diagnostics']
    fraction = max(t/cap for t,cap in zip(run['peak_motor_torque_nm'],diagnostics['torque_caps_nm']))
    motor = run['config']['motor_model']
    if (run['commanded_waist_position_range_rad'] != [0.,0.] or fraction>1+1e-9
            or diagnostics['max_filtered_command_speed_rad_s']>motor['target_speed_rad_s']+1e-9
            or diagnostics['max_filtered_command_acceleration_rad_s2']>motor['target_acceleration_rad_s2']+1e-9):
        raise ValueError('Motor envelope differs for '+job['id'])
    reason = run['termination']
    if event['parameters']['launch_s']>job['seconds']:
        if trigger is not None:
            outcome = 'control triggered'
        elif run['edge_balance']['metrics']['final_five_seconds_pass']:
            outcome = 'control held edge'
        else:
            outcome = 'control failed'
    elif not eligible:
        outcome = 'unqualified before hit'
    elif evaluation['safe_flat_after_impacts']:
        outcome = 'settled flat'
    elif run['settled'] and not event['rebounded']:
        outcome = 'flat without rebound'
    elif impact['pass_impact'] and run['edge_balance']['metrics']['final_five_seconds_pass']:
        outcome = 'edge recovery'
    elif reason == 'nonfoot_floor_contact':
        outcome = 'floor contact'
    elif reason == 'actual_joint_limit_exceeded':
        outcome = 'joint stop'
    elif reason == 'tilt_reached_80_deg':
        outcome = '80° backstop'
    elif reason.startswith('invalid_head_target'):
        outcome = 'IK stop'
    elif run['completed']:
        outcome = 'completed'
    else:
        raise ValueError(reason)
    return dict(id=job['id'],edge=job['edge'],side=event['parameters']['from_side'],
                release_m_s=event['parameters']['speed_m_s'],impact_m_s=event['incoming_speed_m_s'],
                launch_s=event['parameters']['launch_s'],seed=job['policy']['feedback']['sensor_seed'],
                dt=job['dt'],seconds=job['seconds'],tilt_limit_deg=run['tilt_limit_deg'],return_policy=policy,contact_s=contact,
                eligible=eligible,rebounded=event['rebounded'],settled=run['settled'],trigger_s=trigger,
                robot_bodies=event['robot_bodies'],normal_impulse_ns=event['normal_impulse_ns'],
                peak_normal_force_n=event['peak_normal_force_n'],rebound_speed_m_s=event['rebound_speed_m_s'],
                sensed_roll_at_trigger_deg=decision['sensed_roll_deg'] if decision else None,
                actual_roll_at_trigger_deg=decision['roll_deg'] if decision else None,
                observed_roll_rate_at_trigger_deg_s=decision['sensed_roll_rate_deg_s'] if decision else None,
                peak_tilt_deg=run['peak_tilt_deg'],first_40_s=run['first_40deg_time_s'],
                termination=reason,end_s=run['elapsed_seconds'],outcome=outcome,
                max_torque_fraction_of_cap=fraction,min_joint_margin_rad=run['min_joint_margin_rad'])


def panel(row,record):
    policy = row['return_policy']
    angle,rate=policy['emergency_roll_deg'],policy.get('emergency_rate_deg_s',0)
    if angle or rate:
        trigger = f"{angle:g}°" if angle else f"{rate:g}°/s"
        label = f"At {trigger}   |   head roll {policy['roll_deg']:+g}°   ·   y {policy['y_mm']:+g} mm"
    else:
        label = 'Keep holding the edge'
    return dict(label=label,
                outcome=row['outcome'].upper(),color=COLORS[row['outcome']],record=str(record),
                trigger=row['trigger_s'],detail=f"Peak {row['peak_tilt_deg']:.2f}°   ·   trial ended at {row['end_s']:.2f} s")


def make_chapters(by_condition):
    chapters=[]
    for (edge,side,speed,release,seed,dt,cutoff),group in by_condition.items():
        limit = 'contact only' if cutoff is None else f'{cutoff:g}° backstop'
        for i in range(0,len(group),4):
            items=group[i:i+4]
            title = (f"{edge.title()} edge   |   unstruck control" if release>14 else
                     f"{edge.title()} edge   |   ball from {'left' if side==1 else 'right'}   |   {speed:g} m/s release")
            chapters.append(dict(title=title,
                subtitle=f'Natural lift   |   Emergency return after a stable hold   |   Release {release:g} s   |   Seed {seed}   |   {dt*1000:g} ms   |   {limit}',
                panels=items+[None]*(4-len(items))))
    return chapters


def replay(task):
    from train_balance_robustness import evaluate
    row, folder, bank = task
    path = folder/(row['id']+'.json')
    policy = deepcopy(bank['edges'][row['edge']])
    policy['feedback']['sensor_seed'] = row['seed']
    policy['return'] = row['return_policy']
    ball = dict(bank['rolling_ball'], from_side=row['side'], speed_m_s=row['release_m_s'], launch_s=row['launch_s'])
    job = dict(id=row['id'],edge=row['edge'],policy=policy,balls=[ball],seconds=row['seconds'],
               dt=row['dt'],tilt_limit_deg=row['tilt_limit_deg'],record=str(path))
    if not path.exists():
        evaluate(job)
    if result_row(job,path) != row:
        raise ValueError('Replayed return differs from report: '+row['id'])
    return row, path


def add_panel(groups,row,path):
    key=(row['edge'],row['side'],row['release_m_s'],row['launch_s'],row['seed'],row['dt'],row['tilt_limit_deg'])
    groups[key].append(panel(row,path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batches',nargs='*',type=Path)
    parser.add_argument('--report',required=True,type=Path)
    parser.add_argument('--manifest',required=True,type=Path)
    parser.add_argument('--replay',type=Path,metavar='WORK_DIR',help='Recreate trajectories from the published report')
    parser.add_argument('--workers',type=int,default=4)
    args = parser.parse_args()
    if args.replay:
        archive=json.loads(args.report.read_text())
        args.replay.mkdir(parents=True,exist_ok=True)
        bank=json.loads(Path(archive['policy']).read_text())
        groups=defaultdict(list)
        with ProcessPoolExecutor(args.workers) as pool:
            for row,path in pool.map(replay,((r,args.replay,bank) for r in archive['results'])):
                add_panel(groups,row,path)
        args.manifest.write_text(json.dumps(make_chapters(groups),indent=2))
        print('Verified',len(archive['results']),'return trials')
        return
    if not args.batches:
        parser.error('Supply result batches or --replay')
    rows, by_condition, hashes = [],defaultdict(list),{}
    for path in args.batches:
        batch=json.loads(path.read_text())
        if not batch['completed'] or len(batch['candidates'])!=len(batch['jobs']):
            raise ValueError(str(path)+' incomplete')
        hashes[path.stem]=batch['source_sha256']
        for job,original in zip(batch['jobs'],batch['candidates']):
            record=Path(job['record'])
            row=result_row(job,record)
            if row['termination']!=original['result']['termination'] or row['peak_tilt_deg']!=original['result']['peak_tilt_deg']:
                raise ValueError('Recorded outcome differs for '+job['id'])
            rows.append(row)
            add_panel(by_condition,row,record)
    if len({r['id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate trial identifiers')
    chapters=make_chapters(by_condition)
    with args.report.open('x') as f:
        json.dump(dict(evidence='uncalibrated_simulation', hardware_ready=False,source_sha256=hashes,
                       policy='configs/balance-recovery-experimental.json',ball='8 cm diameter, 10 g',
                       results=rows),f,indent=2,allow_nan=False)
    args.manifest.write_text(json.dumps(chapters,indent=2))
    print(len(rows),dict(Counter(r['outcome'] for r in rows)))
    for edge in ('left','right'):
        for side in (1,-1):
            good=[r for r in rows if (r['edge'],r['side'])==(edge,side) and r['outcome']=='settled flat']
            print(edge,side,max((r['impact_m_s'],r['id']) for r in good) if good else None)


if __name__=='__main__':
    main()
