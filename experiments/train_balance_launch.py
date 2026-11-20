"""CEM search of the launch waveform on both weak models and both sides."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_balance import read,evaluate
from train_balance_pivot import signed_pivot

KEYS=['preload_roll_deg','roll_deg','preload_y_mm','y_mm','pump_hz','pump_y_phase_deg','preload_s','pitch_deg','x_mm','z_mm']
LOW=np.array([-40,22,8,-25,1.8,-60,.35,-12,0,-10.])
HIGH=np.array([-20,40,25,-6,4.,90,.85,6,22,5.])

def score(cs):
    if not all(c['result']['recovered'] for c in cs):
        return -1000+100*sum(c['result']['recovered'] for c in cs)
    turns=[-c['side']*c['result']['edge_supported_foot_yaw_deg'] for c in cs]
    final=[-c['side']*c['result']['final_foot_yaw_deg'] for c in cs]
    drift=max(c['pivot_metrics']['center_drift_mm'] for c in cs)
    slip=max(c['result']['edge_contact_slip_distance_mm'] for c in cs)
    return 500*sum(signed_pivot(c) for c in cs)+10*min(turns)+2*min(final)-.5*drift-max(0,slip-15)*10

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--generations',type=int,default=4);p.add_argument('--population',type=int,default=24)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    seed=next(c for c in read('experiments/reports/motor-training-balance-lateral-stress-2026-11-19.json')['candidates'] if c['id']=='lateral-4')
    mean=np.array([seed['recipe'][k] for k in KEYS]);std=.15*(HIGH-LOW);rng=np.random.default_rng(2026111970)
    best=dict(recipe=seed['recipe'],feedback=seed['feedback'],score=-1e9)
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),seed=2026111970,
                development_model_ids=[3,4],history=[],candidates=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for gen in range(args.generations):
            rows=np.clip(rng.normal(mean,std,size=(args.population,len(KEYS))),LOW,HIGH)
            rows[0]=np.array([best['recipe'][k] for k in KEYS]);jobs=[]
            for i,row in enumerate(rows):
                g=dict(seed['recipe'],**dict(zip(KEYS,map(float,row))))
                for side,model in itertools.product([-1,1],[3,4]):jobs.append((f'g{gen}-{i}',g,seed['feedback'],side,model,.0005,6.))
            cs=[]
            for c in pool.map(evaluate,jobs):
                cs.append(c)
            grouped=[cs[i:i+4] for i in range(0,len(cs),4)]
            ranked=sorted(grouped,key=score,reverse=True)
            elite=np.array([[c[0]['recipe'][k] for k in KEYS] for c in ranked[:5]])
            mean=.3*mean+.7*elite.mean(axis=0);std=np.maximum(.3*std+.7*elite.std(axis=0),.025*(HIGH-LOW))
            if score(ranked[0])>best['score']:
                c=ranked[0][0];best=dict(recipe=c['recipe'],feedback=c['feedback'],score=score(ranked[0]),cases=ranked[0])
            report['history'].append(dict(generation=gen,candidates=cs));report['best']=best
            report['candidates']=[c for group in ranked[:6] for c in group];save()
            print(json.dumps(dict(generation=gen,best_score=best['score'],best_recipe=best['recipe'],
                  cases=[dict(side=c['side'],model=c['model_id'],recovered=c['result']['recovered'],passed=signed_pivot(c),edge=c['result']['edge_supported_foot_yaw_deg'],yaw=c['result']['final_foot_yaw_deg']) for c in best.get('cases',[])])),flush=True)
            if best['score']>1950:break
    report['completed']=True;save()

if __name__=='__main__':main()
