"""CEM refinement of useful forward progress on all conditions needing fallback."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from train_faster_balance import cases,evaluate,read
from train_faster_penguin import selected_policy

KEYS=['preload_roll_deg','preload_y_mm','y_mm','pump_hz','pitch_deg','x_mm','z_mm','preload_s']
LOW=np.array([-40,16,-20,2.3,-12,5,-12,.4]);HIGH=np.array([-26,25,-6,3.4,2,23,0,.85])

def score(cs):
    good=sum(c['result']['recovered'] for c in cs)
    forward=[c['result']['planar_displacement_m'][0]*1000 for c in cs]
    turns=[c['signed_turn_deg'] for c in cs]
    return (good, min(forward)+.5*min(turns)-.2*max(0,max(c['result']['peak_tilt_deg'] for c in cs)-28))

def main():
    seed=selected_policy()['fallback'];selected=[j for j in cases() if (j['id'],j['side']) in
        [('development-4',1),('fresh-7',-1),('fresh-5',-1),('fresh-5',1)]]
    rng=np.random.default_rng(2026111990);mean=np.array([seed['recipe'][k] for k in KEYS]);std=.1*(HIGH-LOW)
    out=Path('experiments/reports/motor-training-faster-balance-fallback-launch-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,seed=2026111990,history=[],best=None)
    best=seed;best_score=(-1,-1e10)
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for gen in range(3):
            rows=np.clip(rng.normal(mean,std,(16,len(KEYS))),LOW,HIGH);rows[0]=[best['recipe'][k] for k in KEYS];jobs=[]
            for i,row in enumerate(rows):
                p=copy.deepcopy(seed);p['recipe'].update(dict(zip(KEYS,map(float,row))))
                for original in selected:
                    config=copy.deepcopy(original['config']);config['timestep_s']=.00025
                    sensors={k:v for k,v in original['policy']['fallback']['feedback'].items() if k.startswith('sensor_')}
                    jobs.append(dict(id=original['id'],variant=f'g{gen}-{i}',side=original['side'],config=config,primitive=p,sensors=sensors))
            checks=[];generation=dict(index=gen,jobs=jobs,checks=checks);report['history'].append(generation)
            for c in pool.map(evaluate,jobs):
                checks.append(c);save()
                if len(checks)%4==0:
                    cs=checks[-4:];print(json.dumps(dict(generation=gen,variant=cs[0]['variant'],score=score(cs),forward=[round(x['result']['planar_displacement_m'][0]*1000,1) for x in cs],recovered=[x['result']['recovered'] for x in cs])),flush=True)
            groups=[checks[i:i+4] for i in range(0,len(checks),4)];ranked=sorted(groups,key=score,reverse=True)
            if score(ranked[0])>best_score:
                best=ranked[0][0]['primitive'];best_score=score(ranked[0]);report['best']=dict(primitive=best,cases=ranked[0],score=best_score)
            elite=np.array([[cs[0]['primitive']['recipe'][k] for k in KEYS] for cs in ranked[:4]])
            mean=.3*mean+.7*elite.mean(axis=0);std=np.maximum(.3*std+.7*elite.std(axis=0),.025*(HIGH-LOW));save()
    report['completed']=True;save()

if __name__=='__main__':main()
