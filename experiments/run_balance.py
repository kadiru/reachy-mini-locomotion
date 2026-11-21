"""Replay a frozen balance policy in bounded simulation, optionally recording."""
import argparse
import json
from pathlib import Path
from dataclasses import asdict

from reachy_locomotion.balance import BalanceController,mirrored
from reachy_locomotion.primitives import PrimitiveSimulator
from finalize_primitives import record_terminal

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('policy');p.add_argument('--output',required=True)
    p.add_argument('--side',type=int,choices=[-1,1],default=1);p.add_argument('--timestep',type=float,default=.00025)
    p.add_argument('--seconds',type=float,default=5);p.add_argument('--record',action='store_true')
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve previous recording')
    policy=json.loads(Path(args.policy).read_text());config=json.loads(Path('configs/motors-speed-trained.json').read_text());config['timestep_s']=args.timestep
    sim=PrimitiveSimulator('experiments/upstream-reachy-mini',config);g=mirrored(policy['recipe'],args.side)
    controller=BalanceController(g,policy['feedback'],checked_ik=sim.checked_ik)
    r=sim.run_primitive(g,seconds=args.seconds,waist_mode='hold_neutral',controller=controller,tilt_limit_deg=None,record=args.record)
    if args.record:record_terminal(sim,r)
    r.update(balance_feedback=asdict(controller.f),roll_sensor_delay_s=.02*controller.f.sensor_delay_frames,
             selected_yaw_target_deg=controller.yaw_amplitude,side=args.side)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(r,allow_nan=False)+'\n')
    print(json.dumps({k:r[k] for k in ['termination','recovered','peak_roll_deg','peak_side_height_difference_mm','final_foot_yaw_deg']}))

if __name__=='__main__':main()
