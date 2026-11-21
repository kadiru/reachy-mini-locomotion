"""Retain a low-cadence lift and tune yaw about the loaded edge."""
import argparse
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from train_cadence import read,save,rollout
from refine_cadence import quality,speed_score,validate_frozen


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--forward-lift',action='store_true',help='Start from a forward-moving stroke close to edge support')
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    maps=read(args.source/'candidates.json')
    source=read(args.source/'history.json')['search-0']
    seed=max((r for r in source if r['completed'] and r['cadence_ok'] and r['settled']
              and (not args.forward_lift or r['passed'])),key=lambda r:r['rim_lift_mm'])
    base=maps[seed['name']]
    training_seed=2026112115 if args.forward_lift else 2026112113
    rng=np.random.default_rng(training_seed)
    model=read('configs/motors-speed-trained.json')
    policies={};history={}
    def jobs(names,seconds,dt,sides):
        return [(n,policies[n],s,seconds,dict(model,timestep_s=dt),None) for n in names for s in sides]
    def batch(ex,label,jobs):
        rows=list(ex.map(rollout,jobs))
        history[label]=rows;save(args.out/'history.json',history)
        b=max(rows,key=quality)
        print(label,len(rows),b['name'],round(b['distance_mm'],1),round(b['edge_s'],2),b['passed'],flush=True)
        return rows
    def order(n,rows):
        rr=[r for r in rows if r['name']==n]
        return (sum(quality(r)[0] for r in rr),sum(r['passed'] for r in rr),min(speed_score(r) for r in rr))
    with ProcessPoolExecutor(max_workers=4) as ex:
        for yaw in ([base['yaw']-5,base['yaw'],base['yaw']+5] if args.forward_lift else [-36,-24,-12,0,12,24,36]):
            for delay in ([base['yaw_delay']-.025,base['yaw_delay'],base['yaw_delay']+.04] if args.forward_lift else [.02,.12,.22,.32]):
                for scale in ([.9,1,1.1,1.2] if args.forward_lift else [.80,1.0]):
                    n=f'grid-{yaw}-{delay}-{scale}'
                    policies[n]=dict(base,yaw=yaw,roll=min(40,base['roll']*scale),y=float(np.clip(base['y']*scale,-25,25)),yaw_delay=max(0,delay))
        rows=batch(ex,'lift-grid',jobs(policies,8 if args.forward_lift else 4,.001,[1]))
        for g in range(3):
            ranked=sorted(rows,key=quality,reverse=True)
            lifted=sorted((r for r in rows if r['completed'] and r['settled'] and r['distance_mm']>0
                           and (not args.forward_lift or r['passed'])),
                          key=lambda r:r['rim_lift_mm'],reverse=True)
            parents=list(dict.fromkeys(r['name'] for r in (lifted[:5]+ranked[:3] if args.forward_lift else ranked[:5]+lifted[:4])))
            new=[]
            for i in range(32):
                parent=policies[parents[i%len(parents)]]
                p=dict(parent)
                for k,sigma,lo,hi in [('yaw',6,-46,46),('y',2,-25,25),('roll',2,18,40),
                        ('yaw_delay',.055,0,.38),('y_delay',.035,0,.20),('roll_delay',.025,0,.18),
                        ('yaw_s',.025,.045,.28),('y_s',.020,.045,.23),('roll_s',.020,.045,.24),
                        ('x_arc',3,-15,15),('z_arc',3,-16,16),('x',2.5,-16,18),('pitch',2,-10,14),
                        ('brake',.035,.025,.3)]:
                    p[k]=float(np.clip(rng.normal(parent[k],sigma*(.85**g)),lo,hi))
                p['swings_per_block']=int(rng.choice([6,8,10])) if i%6==0 else parent['swings_per_block']
                n=f'lift-{g}-{i}';policies[n]=p;new.append(n)
            rows=batch(ex,f'search-{g}',jobs(new+parents,8,.001,[-1 if g%2 else 1]))
        save(args.out/'candidates.json',policies)
        top=[r['name'] for r in sorted(rows,key=quality,reverse=True)[:7]]
        checks=batch(ex,'development',jobs(top,16,.0005,[-1,1]))
        final=sorted(top,key=lambda n:order(n,checks),reverse=True)[:3]
        rr=batch(ex,'nominal',jobs(final,24,.00025,[-1,1]))
        name=max(final,key=lambda n:order(n,rr));p=policies[name]
        save(args.out/'policy.json',dict(p,selected_candidate=name,training_seed=training_seed,evidence='uncalibrated_simulation',hardware_ready=False))
        print('frozen',name,flush=True)
        validate_frozen(ex, batch, model, name, p, training_seed+1, args.out)


if __name__=='__main__':
    main()
