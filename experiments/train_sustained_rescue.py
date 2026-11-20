"""Seeded cross-entropy training of the weak-condition rescue launch."""
import copy,json
from pathlib import Path
import numpy as np
from train_sustained_penguin import batch,development,read
from train_faster_penguin import make_job

FIELDS=[('preload_roll_deg',-40,-22,4),('preload_y_mm',17,25,2),('preload_s',.4,.8,.08),
        ('pump_hz',2.2,3.3,.22),('y_mm',-22,-5,3),('pitch_deg',-12,4,3),
        ('x_mm',4,23,4),('z_mm',-15,0,2.5),('pump_y_phase_deg',5,65,12)]

def score(checks):
    def one(c):
        r=c['result'];p=c['penguin']
        physical=r['completed'] and r['settled']
        last=p['steps'][-1] if p['steps'] else None
        goal_error=abs(last['heading_deg']-last['target_heading_deg']) if last and not last['turn_goal_met'] else 0
        return (1000 if physical else -2000)+(700 if p['completed'] else 0)+(1000 if c['passed'] else 0)+p['forward_mm']-4*goal_error
    values=[one(c) for c in checks]
    return min(values)+.05*sum(values)

if __name__=='__main__':
    rng=np.random.default_rng(2026112003)
    base=read('experiments/generated/sustained-penguin/cadence-candidate.json')
    bank=read('experiments/reports/motor-training-sustained-weak-turn-2026-11-20.json')
    warm=next(j['policy']['delayed_sensor_fallback']['recipe'] for j in bank['jobs'] if j['variant']=='bank-g1-1')
    base['delayed_sensor_fallback']['recipe']=copy.deepcopy(warm)
    mean=np.array([warm[k] for k,*_ in FIELDS]);std=np.array([s for k,a,b,s in FIELDS])
    lo=np.array([a for k,a,b,s in FIELDS]);hi=np.array([b for k,a,b,s in FIELDS])
    anchors=[j for j in development() if (j['id'],j['side']) in [('unseen-7',-1),('fresh-5',1)]]
    history=[];best=None
    for generation in range(3):
        vectors=[mean]+[np.clip(rng.normal(mean,std),lo,hi) for _ in range(23)]
        jobs=[];policies=[]
        for i,v in enumerate(vectors):
            p=copy.deepcopy(base)
            p['delayed_sensor_fallback']['recipe'].update({f[0]:float(x) for f,x in zip(FIELDS,v)})
            policies.append(p)
            for source in anchors:
                j=make_job(source,p,seconds=26);j['finish_settle_s']=1.;j['variant']=f'g{generation}-{i}';jobs.append(j)
        report=batch(jobs,f'rescue-cem-g{generation}',4,dict(seed=2026112003,search_fields=FIELDS))
        grouped=[report['checks'][i*len(anchors):(i+1)*len(anchors)] for i in range(len(vectors))]
        order=sorted(range(len(vectors)),key=lambda i:score(grouped[i]),reverse=True)
        history.append(dict(generation=generation,ranking=[dict(index=i,score=score(grouped[i]),passes=sum(c['passed'] for c in grouped[i])) for i in order]))
        best=dict(policy=policies[order[0]],checks=grouped[order[0]],score=score(grouped[order[0]]))
        target=Path('experiments/generated/sustained-penguin/rescue-cem-best.json')
        target.write_text(json.dumps(dict(history=history,best=best),indent=2)+'\n')
        print('GENERATION BEST',generation,order[0],best['score'],sum(c['passed'] for c in best['checks']),flush=True)
        if all(c['passed'] for c in best['checks']):break
        elite=np.array([vectors[i] for i in order[:6]])
        mean=elite.mean(axis=0);std=np.maximum(elite.std(axis=0),np.array([s for k,a,b,s in FIELDS])*.3)
