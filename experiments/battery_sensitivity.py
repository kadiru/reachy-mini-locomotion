"""Battery-operation follow-up: isolate voltage within published motor data.

The user confirmed battery power for the reference video. Actual bus voltage,
regulation and sag remain unknown; no extrapolation above the 6 V data endpoint.
"""
import argparse
import copy
import json
from pathlib import Path

from reachy_locomotion.sway import SwaySimulator,Gait
from summarize_motor_training import qualified


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output)
    if out.exists():p.error('Choose a new output path')
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    gait=Gait(**json.loads(Path('configs/sway-trained.json').read_text()))
    rows=[]
    for v in [5.,5.5,6.]:
        for dt in [.002,.001]:
            c=copy.deepcopy(base);c['timestep_s']=dt;c['motor_model']['voltage_v']=v
            r=SwaySimulator('experiments/upstream-reachy-mini',c).run(gait,cycles=24)
            rows.append(dict(qualified=qualified(r),result=r))
            print(v,dt,r['forward_m'],r['lateral_m'],r['heading_change_rad'],qualified(r),flush=True)
    out.write_text(json.dumps({'evidence':'uncalibrated_simulation','hardware_ready':False,
        'reference_power':'battery_user_confirmed','actual_motor_bus_voltage':'unknown',
        'voltage_is_constant_within_each_rollout':True,'results':rows},indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
