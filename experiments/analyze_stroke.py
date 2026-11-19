"""Summarize recorded contact and braking timing, without a hardware causal claim."""
import argparse
import json
from pathlib import Path

import numpy as np


def analyze(r):
    rows=[x for x in r['trajectory'] if x['measuring']]
    yaw=np.unwrap([x['head_rpy_rad'][2] for x in rows])
    rate=np.gradient(yaw,.02);acc=np.gradient(rate,.02)
    slowing=(rate*acc<0)&(np.abs(rate)>.3)
    forces=np.array([x['foot_side_normal_forces_n_right_left'] for x in rows])
    edge=((forces[:,0]<.1)&(forces[:,1]>.1))|((forces[:,1]<.1)&(forces[:,0]>.1))
    torque=np.array([x['motor_torque_nm'][1:7] for x in rows])
    speed=np.array([x['motor_velocity_rad_s'][1:7] for x in rows])
    power=(torque*speed).sum(axis=1)
    h=np.array([x['head_subtree_angular_momentum_world'] for x in rows])
    bins=[]
    phase=np.array([x['phase_rad'] for x in rows])
    for i in range(16):
        mask=(phase>=i*2*np.pi/16)&(phase<(i+1)*2*np.pi/16)
        if mask.any():bins.append(dict(phase_center_deg=(i+.5)*360/16,
            head_yaw_rate_rad_s=float(rate[mask].mean()),head_momentum_z_kg_m2_s=float(h[mask,2].mean()),
            right_left_normal_n=forces[mask].mean(axis=0).tolist(),
            slowing_fraction=float(slowing[mask].mean()),edge_fraction=float(edge[mask].mean()),
            signed_head_motor_power_w=float(power[mask].mean())))
    return {'evidence':'uncalibrated_simulation','interpretation':'kinematic slowing plus contact correlation, not proof of physical causation',
        'head_momentum_reference':'head-site body subtree about its own COM, world axes; not total angular momentum about the foot',
        'yaw_slowing_fraction':float(slowing.mean()),'edge_support_fraction_at_50hz':float(edge.mean()),
        'fraction_of_slowing_samples_on_one_edge':float(edge[slowing].mean()) if slowing.any() else 0.,
        'negative_net_head_motor_power_fraction':float((power<0).mean()),
        'head_momentum_z_range_kg_m2_s':[float(h[:,2].min()),float(h[:,2].max())],
        'phase_bins':bins}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report');p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output)
    if out.exists():p.error('Choose new output path')
    out.write_text(json.dumps(analyze(json.loads(Path(a.report).read_text())),indent=2,allow_nan=False)+'\n')
    print(out)


if __name__=='__main__':main()
