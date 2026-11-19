"""Independent wider/slower compound-swing basins with fixed motor limits."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_faster import decode, evaluate, read, vector


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    args=p.parse_args(); out=Path(args.output)
    if out.exists(): p.error('Choose a new report path')
    base=read('configs/sway-speed-trained.json')
    fb=read('configs/heading-speed-trained.json')
    rng=np.random.default_rng(2026111922); gaits=[]
    for frequency in [3.125,3.5714285714,4.1666666667,4.5454545455,5.]:
        for yaw in [42,50,56]:
            for roll in [30,38]:
                for sharp in [.7,2.5]:
                    gaits.append(dict(base,frequency_hz=frequency,yaw_deg=yaw,roll_deg=roll,
                                      yaw_sharpness=sharp,yaw_phase_deg=float(rng.uniform(105,135)),
                                      x_mm=float(rng.uniform(-12,0)),pitch_bias_deg=float(rng.uniform(-2,6))))
    for _ in range(48):
        gaits.append(dict(base,frequency_hz=float(rng.uniform(2.8,5)),yaw_deg=float(rng.uniform(40,58)),
                          roll_deg=float(rng.uniform(28,39)),yaw_phase_deg=float(rng.uniform(90,145)),
                          x_mm=float(rng.uniform(-15,7)),z_mm=float(rng.uniform(-6,3)),
                          pitch_bias_deg=float(rng.uniform(-5,8)),roll_sharpness=float(rng.uniform(.5,3)),
                          yaw_stroke_fraction=float(rng.uniform(.5,1)),
                          stroke_accel_fraction=float(rng.uniform(.15,.4)),stroke_brake_fraction=.025,
                          phase_skew=float(rng.uniform(-.3,.4))))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,
                seed=2026111922,physical_and_software_motor_limits_unchanged=True,candidates=[],
                source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                               [Path(__file__),Path('experiments/train_faster.py'),Path('configs/motors-speed-trained.json')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        jobs=[(f'broad-{i}',decode(vector(g)),fb,8.,.001) for i,g in enumerate(gaits)]
        for r in pool.map(evaluate,jobs):
            report['candidates'].append(r);out.write_text(json.dumps(report,allow_nan=False)+'\n')
            print(json.dumps(dict(id=r['id'],**r['metrics'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report,allow_nan=False)+'\n')


if __name__=='__main__': main()
