"""Joint CEM of a fast sway and stride-held heading correction, local CPU only."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
import numpy as np
from reachy_locomotion.heading import HeadingFeedback,HeadingSensor
from select_progress import stress
from train_heading import evaluate,brief

GK=['frequency_hz','roll_deg','yaw_deg','yaw_phase_deg','pitch_bias_deg','x_mm','y_mm','y_phase_deg','roll_bias_deg']
FK=['kp','ki','kd','max_correction_deg','correction_rate_deg_s']
LOW=np.array([1.7,31,20,80,6,-16,10,-155,-2, .005,0,0,.5,.2])
HIGH=np.array([2.1,39,35,145,12,-8,18,-110,1, .15,.05,.08,3,5])

def main():
    global FK,LOW,HIGH
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--generations',type=int,default=10)
    p.add_argument('--population',type=int,default=24);p.add_argument('--seed-report');p.add_argument('--fast-feedback',action='store_true')
    a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    base=json.loads(Path('configs/motors-datasheet.json').read_text());configs=[base]+stress(base)
    g=json.loads(Path('configs/sway-progress-fast.json').read_text())
    f=asdict(HeadingFeedback(kp=.04,ki=.008,kd=.02,filter_tau_s=.02,max_correction_deg=1.5,
                            correction_rate_deg_s=2,cycle_average=True,cycle_update=True,startup_ramp_cycles=2))
    if a.fast_feedback:
        f=asdict(HeadingFeedback(**json.loads(Path('experiments/reports/motor-training-heading-cem-2026-11-19.json').read_text())['best']['feedback']))
        f['startup_ramp_cycles']=2.
        FK=FK+['phase_skew_per_correction']
        LOW=np.array([1.75,35,22,85,7,-15,11,-150,-1.8, .06,0,0,1.,3.,-.08])
        HIGH=np.array([2.05,39,32,115,12,-9,19,-115,.5, .5,.15,.03,4.,20.,.08])
    def vector(o):return np.array([o['gait'][k] for k in GK]+[o['feedback'][k] for k in FK])
    def decode(row):
        row=np.clip(row,LOW,HIGH);gg=dict(g,**dict(zip(GK,row[:len(GK)].tolist())))
        gg['roll_deg']=min(gg['roll_deg'],40-abs(gg['roll_bias_deg']))
        return gg,dict(f,**dict(zip(FK,row[len(GK):].tolist())))
    seeds=[dict(gait=g,feedback=f),dict(gait=dict(g,roll_deg=36,yaw_phase_deg=103,y_phase_deg=-140),feedback=f),
           dict(gait=json.loads(Path('configs/sway-progress-conservative.json').read_text()),feedback=f)]
    if a.seed_report:
        old=json.loads(Path(a.seed_report).read_text());seeds=old['history'][:6]+seeds
        g=seeds[0]['gait'];f=seeds[0]['feedback']
    mean=vector(seeds[0]);std=.13*(HIGH-LOW);rng=np.random.default_rng(2026111906)
    receipt=dict(evidence='uncalibrated_simulation',hardware_ready=False,development_seed=591191,
                 search_seed=2026111906,configs=configs,sensor=asdict(HeadingSensor()),history=[],args=vars(a),
                 source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path('src/reachy_locomotion/heading.py'),Path('src/reachy_locomotion/sway.py')]})
    best=None
    with ProcessPoolExecutor(max_workers=4) as pool:
        for gen in range(a.generations):
            rows=np.clip(rng.normal(mean,std,size=(a.population,len(LOW))),LOW,HIGH)
            for i,o in enumerate(seeds[:3]):rows[i]=vector(o)
            if best:rows[0]=vector(best)
            pairs=[decode(row) for row in rows]
            out=list(pool.map(evaluate,[(gg,ff,configs,6.5,receipt['sensor']) for gg,ff in pairs]))
            out.sort(key=lambda o:o['score'],reverse=True);best=out[0];seeds=out[:3]
            elite=np.array([vector(o) for o in out[:max(4,a.population//6)]])
            mean=.2*mean+.8*elite.mean(axis=0);std=np.maximum(.3*std+.7*elite.std(axis=0),.02*(HIGH-LOW))
            receipt['history'].extend(out);receipt['best']=best
            path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
            print(gen,json.dumps(brief(best)),flush=True)
    receipt['history'].sort(key=lambda o:o['score'],reverse=True);receipt['best']=receipt['history'][0];receipt['complete']=True
    path.write_text(json.dumps(receipt,allow_nan=False)+'\n')
if __name__=='__main__':main()
