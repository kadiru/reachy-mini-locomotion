"""Drop simultaneous, non-overlapping spheres onto a naturally balancing Reachy in MuJoCo."""
import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from reachy_locomotion.balance_curriculum import edge_ok
from train_balance_robustness import read, evaluate

POLICY = 'configs/balance-recovery-experimental.json'
# Nested, mirrored horizontal layout. Centre first, then opposite pairs.
STEP = .032
H = math.sqrt(3)/2
SLOTS = [(0,0),(0,1),(0,-1),(H,.5),(-H,-.5),(H,-.5),(-H,.5),
         (2*H,0),(-2*H,0),(H,1.5)]


def study_case(task):
    folder,n,edge,repeat,mass,height,release,compact = task
    tag = f"{edge}-{n:02d}-{'repeat' if repeat else 'nominal'}"
    source = read(POLICY)
    p = copy.deepcopy(source['edges'][edge])
    if repeat:
        p['feedback']['sensor_seed']=2026112231
    mirror = 1 if edge=='left' else -1
    slots = SLOTS[:7]+[(H,1.5),(-H,-1.5),(-H,1.5)] if compact else SLOTS
    balls = [dict(drop_height_m=height,offset_x_m=x*STEP,offset_y_m=mirror*y*STEP,
                  launch_s=release if i<n else 590.,radius_m=.015,mass_kg=mass)
             for i,(x,y) in enumerate(slots)]
    path = folder/'states'/(tag+'.json')
    evaluate(dict(id=tag,edge=edge,policy=p,balls=balls,dt=.000125,seconds=14,record=str(path),
                  tilt_limit_deg=None,observe_landing=True,collision_window=[release,release+.8]))
    run=read(path)
    assert run['observe_landing'] and run['tilt_limit_deg'] is None and run['commanded_waist_position_range_rad']==[0.,0.]
    contacts = [e for e in run['ball_disturbances'] if e['released']]
    direct = [e for e in contacts if e['contacted_robot'] and e['first_robot_contact_s'] is not None and
              (e['first_floor_contact_s'] is None or e['first_robot_contact_s']<e['first_floor_contact_s'])]
    if len(contacts)!=n or any(abs(e['release_s']-release)>1e-8 for e in contacts):
        raise ValueError(f'{tag}: episode did not reach simultaneous release')
    # Use a common pre-drop interval even for the unstruck controls.
    trace=run['trace']
    sign=run['edge_balance']['side']
    target=p['feedback']['target_roll_deg']
    pre=[r for r in trace if release-1<=r['time_s']<=release]
    qualified=len(pre)==51 and all(edge_ok(r,'roll',sign,target) for r in pre)
    end=run['edge_balance']['metrics']['final_five_seconds_pass']
    floor=run['first_body_floor_contact']
    if not qualified:
        outcome='unqualified start'
    elif floor:
        outcome='floor contact'
    elif not run['completed']:
        outcome=run['termination']
    elif end:
        outcome='held edge'
    elif run['settled']:
        outcome='settled flat'
    else:
        outcome='lost edge'
    hit_times=[e['first_robot_contact_s'] for e in direct]
    return dict(id=tag,count=n,edge=edge,repeat=repeat,release_s=release,seed=p['feedback']['sensor_seed'],
                prequalified=qualified,direct_hits=len(direct),first_hit_s=min(hit_times,default=None),
                last_first_hit_s=max(hit_times,default=None),outcome=outcome,floor_contact=floor,
                final_hold=end,settled_flat=run['settled'],end_s=run['elapsed_seconds'],
                final_tilt_deg=trace[-1]['tilt_deg'],peak_tilt_deg=run['peak_tilt_deg'],
                first_joint_warning=run['first_joint_travel_warning'],min_joint_margin_rad=run['min_joint_margin_rad'],
                max_closure_error_m=run['max_closure_error_m'],termination=run['termination'],
                first_35_s=run['first_35deg_time_s'],first_40_s=run['first_40deg_time_s'],balls=contacts)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--report',type=Path,required=True)
    ap.add_argument('--mass',type=float,default=.010)
    ap.add_argument('--height',type=float,default=.48)
    ap.add_argument('--nominal-release',type=float,default=5.)
    ap.add_argument('--repeat-release',type=float,default=6.)
    ap.add_argument('--workers',type=int,default=6)
    ap.add_argument('--counts',nargs='+',type=int,choices=range(11),default=range(11))
    ap.add_argument('--repeat',action='store_true',help='Also run second observation seed and release time')
    ap.add_argument('--compact',action='store_true',help='Place balls 8–10 across the wider axis of the head')
    a=ap.parse_args()
    if a.work.exists() or a.report.exists():
        ap.error('Choose fresh output paths')
    a.work.mkdir(parents=True)
    tasks=[(a.work,n,edge,repeat,a.mass,a.height,a.repeat_release if repeat else a.nominal_release,a.compact)
           for repeat in ([False,True] if a.repeat else [False])
           for n in a.counts for edge in ('left','right')]
    rows=[]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for r in pool.map(study_case,tasks):
            rows.append(r)
            print(r['id'],r['direct_hits'],r['outcome'],round(r['peak_tilt_deg'],1),r['termination'],flush=True)
    a.report.write_text(json.dumps(dict(evidence='uncalibrated MuJoCo simulation',policy=POLICY,
        ball_mass_kg=a.mass,ball_radius_m=.015,release_height_from_floor_m=a.height,timestep_s=.000125,
        compact_outer_slots=a.compact,
        rows=rows),indent=2,allow_nan=False)+'\n')
