"""Write a searchable CSV and film manifests from a side-impact study."""
import argparse
import csv
from itertools import groupby
import json
from pathlib import Path

import numpy as np
from reachy_locomotion.primitives import PrimitiveSimulator
from side_ball_study import EDGES, MODES, MOTOR_CONFIG
from train_balance_robustness import read

COLORS = {'edge recovery':'#83e2ac', 'control held':'#83e2ac', 'toppled':'#ff947c', 'joint stop':'#ffd184'}


def recovered_edge(row):
    return (row.get('balanced_before',False) and row['termination']=='completed' and
            row['recovery_s'] is not None and row['final_hold'])


def panel(row, states, start, stop=None, dense=False, mode_label=False):
    event = row.get('event', {})
    hit = event.get('first_robot_contact_s')
    speed = event.get('incoming_3d_speed_m_s')
    label = 'Unstruck control' if row['source'] is None else f"Ball from {row['source']}"
    if mode_label:
        label = row['mode'].title()
    label = f"{row['edge'].title()} edge  ·  {label}"
    outcome = row['outcome'].upper()
    if row['outcome']=='incomplete recovery' and recovered_edge(row):
        outcome = 'EDGE HELD, NO REBOUND'
    if speed is not None:
        outcome += f"   |   {speed:.2f} m/s at contact"
    detail = 'No ball released'
    if row['source'] is not None:
        detail = 'No recorded robot contact'
    if hit is not None:
        kind = 'Airborne' if row['airborne_hit'] else 'Ground first'
        detail = f"{kind}  ·  {event['first_contact_height_m']*100:.1f} cm above floor"
    item = dict(label=label, outcome=outcome, detail=detail, color=COLORS.get(row['outcome'],'#bdadf1'),
                record=str(states/(row['id']+'.json')), start_s=start)
    if stop is not None:
        item.update(stop_s=stop, dense=dense)
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--csv',type=Path,required=True)
    args = parser.parse_args()
    rows = read(args.report)['rows']
    states = args.work/'states'
    sim = PrimitiveSimulator('experiments/upstream-reachy-mini', read(MOTOR_CONFIG))
    flat_rows=[]
    for r in rows:
        e=r.get('event',{})
        flat={k:v for k,v in r.items() if k!='event'}
        flat['regained_and_kept_edge'] = recovered_edge(r) if e else None
        if r['termination']=='actual_joint_limit_exceeded':
            terminal = read(states/(r['id']+'.json'))['terminal_state']
            angles = np.asarray(terminal['qpos'])[sim.joint_qpos]
            margins = np.column_stack([angles-sim.command_limits[:,0], sim.command_limits[:,1]-angles])
            joint, end = np.unravel_index(margins.argmin(),margins.shape)
            if margins[joint,end] >= -.005:
                raise ValueError('No terminal travel violation for '+r['id'])
            flat['limiting_joint'] = sim.ik.names[joint]
            flat['limit_end'] = ('lower','upper')[end]
        for key in ('incoming_speed_m_s','incoming_3d_speed_m_s','first_floor_contact_s','first_robot_contact_s',
                    'first_contact_height_m','first_robot_body','rebounded','normal_impulse_ns'):
            flat[key]=e.get(key)
        flat_rows.append(flat)
    with args.csv.open('x',newline='') as f:
        writer=csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for row in flat_rows for k in row)),lineterminator='\n')
        writer.writeheader()
        writer.writerows(flat_rows)
    chapters=[]
    def group(r):
        return r['repeat'],r['speed'],r['mode']
    for (repeat,speed,mode),items in groupby(sorted(rows,key=group),group):
        items=list(items)
        release=6 if repeat else 5
        title = 'Unstruck controls' if mode=='control' else f"{mode.title()}  ·  {speed:g} m/s horizontal release"
        subtitle = 'Second sensor seed and 6 s release' if repeat else 'Nominal sensor seed and 5 s release'
        if mode=='control':
            subtitle=f"Unstruck, sensor seed {items[0]['seed']}"
        chapters.append(dict(title=title, subtitle=subtitle+'   |   5 V motor model   |   neutral waist target',
                             panels=[panel(r,states,0 if mode=='control' else release-.6) for r in items]))
    # Show the same speed, edge and incoming side for all three launches.
    # Select a comparison with an airborne edge recovery and a contrasting outcome when available.
    triples=[]
    for speed in sorted({r['speed'] for r in rows if r['source']}):
        for edge in EDGES:
            for source in EDGES:
                match={r['mode']:r for r in rows if not r['repeat'] and r['speed']==speed and
                       r['edge']==edge and r['source']==source}
                if not set(MODES)<=match.keys():
                    continue
                trio=[match[mode] for mode in MODES]
                airborne=all(r['airborne_hit'] for r in trio[1:])
                if not airborne:
                    continue
                diverse=len({r['outcome'] for r in trio})
                key=(any(r['outcome']=='edge recovery' for r in trio[1:]),diverse,
                     any(r['outcome']=='toppled' for r in trio), speed)
                triples.append((key,trio))
    if not triples:
        raise ValueError('No matched rolling and airborne collision comparison')
    trio=max(triples,key=lambda t:t[0])[1]
    r=trio[0]
    caption=f"{r['edge'].title()} edge  ·  from {r['source']}  ·  {r['speed']:g} m/s horizontal release  ·  10 g ball"
    highlights=[
        dict(title='One rolling ball and two airborne releases',subtitle=caption,playback=.2,
             panels=[panel(x,states,5.,5.64,True,True) for x in trio]),
        dict(title='Balance after the impact',subtitle=caption,playback=1.,
             panels=[panel(x,states,4.8,mode_label=True) for x in trio])]
    for filename,data in [('chapters',chapters),('highlights',highlights)]:
        (args.work/(filename+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print([r['id'] for r in trio])


if __name__=='__main__':
    main()
