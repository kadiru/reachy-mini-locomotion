"""Replay an edge hold, controlled return or side-ball impact in simulation."""
import argparse
import copy
import json
from pathlib import Path

from train_balance_robustness import evaluate,read


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('policy');p.add_argument('--edge',choices=['left','right','front','back'],required=True)
    p.add_argument('--mode',choices=['hold','cycle','impacts','single-hit'],default='hold')
    p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=32)
    p.add_argument('--timestep',type=float,default=.000125);p.add_argument('--record',action='store_true')
    p.add_argument('--raised-start-deg',type=float,help='Explicit artificial initial angle; never a natural launch')
    p.add_argument('--outward-rate-deg-s',type=float,default=0)
    p.add_argument('--capture-policy',action='store_true');p.add_argument('--config',default='configs/motors-speed-trained.json')
    p.add_argument('--sensor-seed',type=int,help='Override the frozen observation noise seed')
    p.add_argument('--ball-from',choices=['left','right'],default='left',help='Incoming side in single-hit mode')
    p.add_argument('--ball-speed',type=float,default=.3,help='Horizontal single-hit release speed in m/s')
    p.add_argument('--launch-height',type=float,help='Ball centre height above floor in metres for an airborne release')
    p.add_argument('--vertical-speed',type=float,help='Upward launch velocity in m/s for an airborne release')
    p.add_argument('--release-at',type=float,default=5.,help='Single-hit release time in seconds')
    p.add_argument('--emergency-return',type=float,nargs=3,metavar=('TRIGGER_DEG','HEAD_ROLL_DEG','HEAD_Y_MM'),
                   help='Single-hit 0.2 s head return after a stable hold and measured outward tilt')
    p.add_argument('--emergency-rate',type=float,default=0,help='Optional absolute measured roll-rate trigger in degrees/s')
    p.add_argument('--collision-window',type=float,nargs=2,metavar=('START_S','END_S'),
                   help='Record high-rate physical poses in a short window for slow-motion film')
    limits=p.add_mutually_exclusive_group()
    limits.add_argument('--tilt-backstop',type=int,choices=[35,80],help='Reproduce an earlier angle-based stopping rule')
    limits.add_argument('--contact-only',action='store_true',help='Use the default physical termination rules')
    a=p.parse_args();out=Path(a.output)
    if a.vertical_speed is not None and a.launch_height is None:p.error('--vertical-speed requires --launch-height')
    if a.launch_height is not None and a.mode!='single-hit':p.error('--launch-height requires --mode single-hit')
    if a.emergency_rate and not a.emergency_return:p.error('--emergency-rate requires --emergency-return')
    if a.emergency_return and a.mode!='single-hit':
        p.error('--emergency-return requires --mode single-hit')
    if out.exists():p.error('Preserve earlier evidence')
    source=read(a.policy);bank=source['capture_policies'] if a.capture_policy else source['edges']
    if a.edge not in bank:p.error('No frozen policy for this edge/mode')
    policy=copy.deepcopy(bank[a.edge]);job=dict(id='replay-'+a.edge,edge=a.edge,policy=policy,seconds=a.seconds,dt=a.timestep,config=read(a.config))
    job['tilt_limit_deg']=a.tilt_backstop
    if a.sensor_seed is not None:policy['feedback']['sensor_seed']=a.sensor_seed
    if a.raised_start_deg is not None:job['initial_angle_deg']=a.raised_start_deg
    job['outward_rate_deg_s']=a.outward_rate_deg_s
    if a.mode in ('cycle','impacts'):policy['return']=copy.deepcopy(source['controlled_return'])
    if a.mode=='impacts':
        if a.edge not in ('left','right'):p.error('The frozen rolling-ball curriculum is for left/right balance')
        policy['return']['hold_s']=12
        job['balls']=[dict(source['rolling_ball'],launch_s=8,from_side=1),dict(source['rolling_ball'],launch_s=12,from_side=-1)]
    if a.mode=='single-hit':
        if a.edge not in ('left','right'):p.error('Single-hit mode requires a side edge')
        job['balls'] = [dict(source['rolling_ball'], launch_s=a.release_at,
                             from_side=1 if a.ball_from=='left' else -1, speed_m_s=a.ball_speed)]
        if a.launch_height is not None:
            job['balls'][0].update(launch_height_m=a.launch_height,vertical_speed_m_s=a.vertical_speed or 0.)
    if a.emergency_return:
        threshold, head_roll, head_y = a.emergency_return
        policy['return'] = dict(hold_s=30, roll_deg=head_roll, y_mm=head_y, duration_s=.2,
                                neutral_at_deg=25, emergency_roll_deg=threshold, emergency_rate_deg_s=a.emergency_rate)
    if a.collision_window:job['collision_window']=a.collision_window
    if a.record:job['record']=str(out)
    result=evaluate(job)
    if not a.record:
        out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,allow_nan=False)+'\n')
    print(json.dumps(dict(termination=result['result']['termination'],metrics=result['metrics'],curriculum=result['result']['balance_curriculum']),indent=2))


if __name__=='__main__':main()
