"""Collect the hit-speed study and build its chart and film manifests."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import hashlib
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

BALL_BATCHES = ('first','grid','upper','refine','checks','boundary')
CATCH_BATCHES = ('catch','posture','preload','catch-confirm','outward','outward-confirm','right-confirm','after40','after40-confirm')
SLOTS = [('left',1),('left',-1),('right',1),('right',-1)]
COLORS = {'edge recovery':'#83e2ac','floor contact':'#ff947c','joint stop':'#ffd184',
          'completed':'#75cde2','launch failure':'#bdadf1','unbalanced at impact':'#bdadf1'}


def load(path):
    return json.loads(Path(path).read_text())


def hit_row(batch, job, candidate):
    r = candidate['result']
    event = r['ball_disturbances'][0]
    assessment = candidate['impacts']['events'][0]
    p = event['parameters']
    outcome = 'completed'
    if not event['contacted_robot']:
        outcome = 'launch failure'
    elif not assessment['balanced_before_impact']:
        outcome = 'unbalanced at impact'
    elif r['balance_curriculum']['impact_trial_pass']:
        outcome = 'edge recovery'
    elif r['termination'] == 'actual_joint_limit_exceeded':
        outcome = 'joint stop'
    elif r['termination'] == 'nonfoot_floor_contact':
        outcome = 'floor contact'
    elif not r['completed']:
        raise ValueError(r['termination'])
    return dict(batch=batch,id=job['id'],edge=job['edge'],side=p['from_side'],release_m_s=p['speed_m_s'],
                launch_s=p['launch_s'],seed=job['policy']['feedback']['sensor_seed'],dt=job['dt'],
                impact_m_s=event['incoming_speed_m_s'],contact_s=event['first_robot_contact_s'],
                impulse_ns=event['normal_impulse_ns'],balanced_before=assessment['balanced_before_impact'],
                rebound=event['rebounded'],recovery_s=assessment['recovery_s'],outcome=outcome,
                termination=r['termination'],completed=r['completed'],settled_flat=r['settled'],
                final_hold=candidate['metrics']['final_five_seconds_pass'],end_s=r['elapsed_seconds'],
                peak_tilt_deg=r['peak_tilt_deg'],first_40_s=r['first_40deg_time_s'],
                min_joint_margin_rad=r['min_joint_margin_rad'])


def hit_panel(row, folder):
    speed = row['impact_m_s']
    timing = row['end_s']-row['contact_s'] if row['contact_s'] is not None else None
    details = {
        'edge recovery': f"Stable again after {row['recovery_s'] or 0:.2f} s",
        'completed': 'Completed; missed a strict edge-hold criterion',
        'joint stop': 'Joint travel limit stopped the trial',
        'floor contact': 'Body touched the floor',
        'launch failure': 'Trial stopped before ball contact',
        'unbalanced at impact': 'No qualifying hold immediately before contact'}
    detail = details[row['outcome']]
    if timing is not None and row['outcome'] in ('floor contact','joint stop'):
        detail += f' {timing:.2f} s after hit'
    incoming = 'left' if row['side']==1 else 'right'
    label = f"{row['edge'].title()} edge  ·  ball from {incoming}"
    speed_text = f'   |   hit {speed:.3f} m/s' if speed is not None else ''
    return dict(label=label,outcome=row['outcome'].upper()+speed_text,detail=detail,color=COLORS[row['outcome']],
                record=str(folder/'recordings'/(row['id']+'.json')))


def write_manifest(hits, folder):
    chapters = defaultdict(dict)
    for row in hits:
        key = (row['dt'], row['seed'], row['launch_s'], row['release_m_s'])
        slot = (row['edge'], row['side'])
        if slot in chapters[key]:
            raise ValueError(f'Duplicate chapter panel: {key}, {slot}')
        chapters[key][slot] = hit_panel(row, folder)
    manifest=[]
    for (dt,seed,release,speed),group in sorted(chapters.items(),key=lambda kv:(-kv[0][0], kv[0][1:])):
        changes = []
        if dt != .000125:
            changes.append(f'{dt*1000:g} ms physics')
        if seed != 2026112101:
            changes.append('new seed')
        if release != 5:
            changes.append(f'{release:g} s release')
        condition = ', '.join(changes) or 'nominal'
        manifest.append(dict(title=f'{speed:g} m/s release  ·  {condition}',
            subtitle=f'10 g, 8 cm ball   |   Physics {dt*1000:g} ms   |   Release {release:g} s   |   Seed {seed}   |   Waist target 0°',
            panels=[group.get(s) for s in SLOTS]))
    (folder/'hit-film.json').write_text(json.dumps(manifest,indent=2))
    return manifest


def replay_one(task):
    from train_balance_robustness import evaluate
    from copy import deepcopy
    row, folder, bank = task
    path = folder/'recordings'/(row['id']+'.json')
    policy = deepcopy(bank['edges'][row['edge']])
    policy['feedback']['sensor_seed'] = row['seed']
    ball = dict(bank['rolling_ball'], from_side=row['side'], launch_s=row['launch_s'], speed_m_s=row['release_m_s'])
    job = dict(id=row['id'], edge=row['edge'], dt=row['dt'], seconds=14, tilt_limit_deg=80, policy=policy, balls=[ball])
    if path.exists():
        result = load(path)
        candidate = dict(result=result, metrics=result['edge_balance']['metrics'],
                         impacts=result['balance_curriculum']['impacts'])
    else:
        candidate = evaluate(dict(job, record=str(path)))
    if hit_row(row['batch'], job, candidate) != row:
        raise ValueError('Replay differs from report: '+row['id'])
    return row['id']


def write_state_details(hits, folder, destination, physics):
    from reachy_locomotion.primitives import PrimitiveSimulator
    import numpy as np
    sim = PrimitiveSimulator('experiments/upstream-reachy-mini', load(physics))
    fields = ['id','edge','side','impact_m_s','contact_s','outcome','last_precontact_sample_s',
              'precontact_roll_deg','precontact_roll_rate_deg_s','precontact_pitch_deg',
              'first_40_s','first_sample_after_40_s','roll_rate_after_40_deg_s',
              'contacted_links','peak_normal_force_n','last_contact_s','rebound_speed_m_s',
              'limit_joint','limit_end','terminal_joint_angle_rad','joint_limit_rad']
    with destination.open('x',newline='') as out:
        writer = csv.DictWriter(out,fieldnames=fields,lineterminator='\n')
        writer.writeheader()
        for row in hits:
            run = load(folder/'recordings'/(row['id']+'.json'))
            info = {k:row[k] for k in fields if k in row}
            event = run['ball_disturbances'][0]
            info.update(contacted_links=','.join(event['robot_bodies']),
                        peak_normal_force_n=event['peak_normal_force_n'],last_contact_s=event['last_robot_contact_s'],
                        rebound_speed_m_s=event['rebound_speed_m_s'])
            trace = run['trace']
            if row['contact_s'] is not None:
                sample = next(x for x in reversed(trace) if x['time_s']<=row['contact_s'])
                info.update(last_precontact_sample_s=sample['time_s'],precontact_roll_deg=sample['roll_deg'],
                            precontact_roll_rate_deg_s=sample['roll_rate_deg_s'],precontact_pitch_deg=sample['pitch_deg'])
            if row['first_40_s'] is not None:
                sample = next((x for x in trace if x['time_s']>=row['first_40_s']),None)
                if sample is not None:
                    info.update(first_sample_after_40_s=sample['time_s'],roll_rate_after_40_deg_s=sample['roll_rate_deg_s'])
            if row['termination']=='actual_joint_limit_exceeded':
                q = np.array(run['terminal_state']['qpos'])[sim.joint_qpos]
                margins = np.column_stack((q-sim.command_limits[:,0], sim.command_limits[:,1]-q))
                joint,limit = np.unravel_index(np.argmin(margins),margins.shape)
                if margins[joint,limit]>=-.005:
                    raise ValueError('No terminal travel violation for '+row['id'])
                info.update(limit_joint=sim.ik.names[joint],limit_end=('lower','upper')[limit],
                            terminal_joint_angle_rad=q[joint],joint_limit_rad=sim.command_limits[joint,limit])
            writer.writerow(info)


def matched_catches(folder):
    pairs = []
    for label, indexes in [('left', (0, 2)), ('right', (24, 26))]:
        for i in indexes:
            active = load(folder/f'after40-pair-{i}.json')
            held = load(folder/f'after40-pair-{i+1}.json')
            start = active['trajectory'][0]
            assert all(start[k] == held['trajectory'][0][k] for k in ('qpos','qvel'))
            assert active['settled'] and held['termination'] == 'nonfoot_floor_contact'
            trigger = active['catch_trigger_s']
            sample = next(r for r in active['trace'] if r['time_s'] == trigger)
            assert abs(sample['sensed_roll_deg']) >= 40 and active['first_40deg_time_s'] < trigger
            pairs.append(dict(edge=label, active=f'after40-confirm-{i:03d}', held=f'after40-confirm-{i+1:03d}',
                              dt=active['config']['timestep_s'], first_40_s=active['first_40deg_time_s'],
                              observed_roll_at_command_deg=sample['sensed_roll_deg'],
                              actual_roll_at_command_deg=sample['roll_deg'], command_s=trigger,
                              recovered_peak_deg=active['peak_tilt_deg'], held_floor_peak_deg=held['peak_tilt_deg']))
    return pairs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work',type=Path,default=Path('experiments/generated/hit-envelope'))
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--replay',action='store_true',help='Recreate and verify ball trajectories from a published report')
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--state-details',type=Path,help='Write per-impact attitude and offending joint from verified replays')
    args=parser.parse_args()
    if args.state_details and not args.replay:
        parser.error('--state-details requires --replay')
    if args.replay:
        study = load(args.report)
        hits = study['hits']
        bank = load(study['policy'])
        args.work.mkdir(exist_ok=True,parents=True)
        with ProcessPoolExecutor(args.workers) as pool:
            verified = list(pool.map(replay_one, ((r,args.work,bank) for r in hits)))
        write_manifest(hits,args.work)
        if args.state_details:
            write_state_details(hits,args.work,args.state_details,study['physics'])
        print(f'Verified {len(verified)} trajectories')
        return
    hits, raised = [],[]
    for batch in BALL_BATCHES:
        report=load(args.work/(batch+'.json'))
        assert report['completed'] and len(report['jobs'])==len(report['candidates'])
        for job,c in zip(report['jobs'], report['candidates']):
            assert c['id']==job['id'] and c['result']['tilt_limit_deg']==80
            if not job.get('balls'):
                raised.append(dict(edge=job['edge'],angle=job['initial_angle_deg'],rate=job['outward_rate_deg_s'],
                                   termination=c['result']['termination'],end_s=c['result']['elapsed_seconds'],
                                   peak_tilt_deg=c['result']['peak_tilt_deg'],settled_flat=c['result']['settled'],
                                   final_hold=c['metrics']['final_five_seconds_pass']))
                continue
            row=hit_row(batch,job,c)
            hits.append(row)
    catches=[]
    for batch in CATCH_BATCHES:
        cases=[json.loads(line) for line in (args.work/(batch+'.jsonl')).read_text().splitlines()]
        jobs=load(args.work/(batch+'-jobs.json'))
        assert len(cases)==len(jobs) and all(r['case']==j for r,j in zip(cases,jobs))
        for index,r in enumerate(cases):
            r['case'].pop('record',None)
            if r['case'].get('hold_setup'):
                r['trigger_s'] = None
            catches.append(dict(id=f'{batch}-{index:03d}',batch=batch,**r))
    assert len(set(r['id'] for r in hits))==len(hits)
    args.report.parent.mkdir(exist_ok=True,parents=True)
    with args.report.open('x') as out:
        json.dump(dict(evidence='uncalibrated_simulation',hardware_ready=False,
            source_sha256=dict(report['source_sha256'], **{str(path):hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in map(Path,('experiments/probe_high_tilt.py','configs/motors-speed-trained.json','configs/balance-recovery-experimental.json'))}),
            policy='configs/balance-recovery-experimental.json',physics='configs/motors-speed-trained.json',
            ball=dict(diameter_m=.08,mass_kg=.01,distance_m=.3),
            observation='Incoming horizontal ball speed in the last physics sample preceding first robot contact. Edge recovery requires a stable pre-hit hold, rebound, recovery within 3 s and a strict final 5 s.',
            matched_after_measured_40=matched_catches(args.work),
            hits=hits,raised_with_edge_policy=raised,catch_probes=catches),out,indent=2,allow_nan=False)
    with args.report.with_suffix('.csv').open('x', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=hits[0].keys(),lineterminator='\n')
        writer.writeheader()
        writer.writerows(hits)
    manifest = write_manifest(hits, args.work)
    print(len(hits),'ball trials in',len(manifest),'chapters;',len(catches),'catch probes')
    for slot in SLOTS:
        rows=[r for r in hits if (r['edge'],r['side'])==slot and r['balanced_before']]
        for tag,rr in [('nominal',[r for r in rows if r['dt']==.000125 and r['launch_s']==5 and r['seed']==2026112101]),('all',rows)]:
            passes=[r['impact_m_s'] for r in rr if r['outcome']=='edge recovery']
            contacts=[r['impact_m_s'] for r in rr if r['outcome']=='floor contact']
            survival=[r['impact_m_s'] for r in rr if r['completed']]
            print(slot,tag,'max pass',max(passes,default=None),'max complete',max(survival,default=None),
                  'first floor',min(contacts,default=None),dict(Counter(r['outcome'] for r in rr)))


if __name__=='__main__':
    main()
