"""Tune measured-yaw-excursion adaptation while retaining the original fast gait."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
from reachy_locomotion.heading import HeadingFeedback,HeadingSensor
from select_progress import stress
from train_heading import evaluate,brief


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seed-report');a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    base=json.loads(Path('configs/motors-datasheet.json').read_text());configs=[base]+stress(base)
    gait=json.loads(Path('configs/sway-progress-fast.json').read_text())
    f=asdict(HeadingFeedback(kp=.2,ki=.05,kd=.005,filter_tau_s=.12,max_correction_deg=2.5,
                            correction_rate_deg_s=10,yaw_per_roll=-.25,startup_ramp_cycles=2,
                            yaw_excursion_target_deg=4,min_yaw_scale=.5,initial_yaw_scale=.65))
    sensor=asdict(HeadingSensor());candidates=[]
    for target in [2.5,4.]:
        for kp in [.08,.2,.4]:
            for ki in [.01,.05]:candidates.append((gait,dict(f,kp=kp,ki=ki,yaw_excursion_target_deg=target)))
    if a.seed_report:
        best=json.loads(Path(a.seed_report).read_text())['best']
        candidates += [(best['gait'],dict(f,kp=kp,ki=.03,yaw_excursion_target_deg=t)) for kp in [.1,.3] for t in [2.5,4.]]
    candidates += [(gait,dict(f,kp=kp,ki=.03,yaw_excursion_target_deg=t,
                              yaw_amplitude_response=.15,yaw_phase_slowdown_deg=70.,roll_slowdown_deg=10.))
                   for kp in [.08,.2,.4] for t in [2.5,4.]]
    r=dict(evidence='uncalibrated_simulation',hardware_ready=False,development_seed=591191,
           configs=configs,sensor=sensor,history=[],source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
           [Path(__file__),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/heading.py')]})
    with ProcessPoolExecutor(max_workers=4) as pool:
        for i,o in enumerate(pool.map(evaluate,[(g,f,configs,8.,sensor) for g,f in candidates])):
            r['history'].append(o);r['best']=max(r['history'],key=lambda o:o['score'])
            path.write_text(json.dumps(r,allow_nan=False)+'\n');print(i,json.dumps(brief(o)),flush=True)
    r['complete']=True;r['history'].sort(key=lambda o:o['score'],reverse=True)
    path.write_text(json.dumps(r,allow_nan=False)+'\n')
if __name__=='__main__':main()
