"""Search short, high-amplitude strokes at a low repetition rate with fixed-time scoring."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from train_cadence import read,save,rollout,BOUNDS
import copy


def speed_score(row):
    # Penalize more strokes at the same forward velocity without rewarding standing still.
    return row['mm_per_active_s'] / np.sqrt(row['cadence_hz'])


def quality(row):
    return (row['passed'] and len(row['edge_brake_strokes'])>=2,row['passed'],speed_score(row))


def validate_frozen(ex, batch, cfg, winner, p, seed, out):
    policies = {winner: p}
    def jobs(names, sides, seconds, dt):
        return [(n, policies[n], side, seconds, dict(cfg,timestep_s=dt), None)
                for n in names for side in sides]
    old=read('configs/head-swing-distance-trained.json')
    policies.update({'soft-brake':dict(p,brake=.45),'long-stroke':dict(p,roll_s=.44,y_s=.44,yaw_s=.44,brake=.45),
                     'no-yaw':dict(p,yaw=0.),'old-retimed':dict(old,gait=dict(old['gait'],frequency_hz=.5))})
    batch(ex,'controls',jobs(['soft-brake','long-stroke','no-yaw','old-retimed'],[-1,1],24,.00025))
    tests=[];conditions=[]
    fresh_rng=np.random.default_rng(seed)
    for i in range(5):
        c=copy.deepcopy(cfg);c['timestep_s']=.0005
        c['friction_slide_torsion_roll'][0]=float(fresh_rng.uniform(.55,.9))
        c['motor_model'].update(voltage_v=float(fresh_rng.uniform(4.7,5.5)),pid_scale=float(fresh_rng.uniform(.9,1.1)),
            delay_frames=int(fresh_rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
        conditions.append(c)
        tests.extend((f'heldout-{i}',p,s,16,c,None) for s in (-1,1))
    save(out/'conditions.json',conditions)
    batch(ex,'heldout',tests)
    batch(ex,'finer',jobs([winner],[-1,1],24,.000125))
    batch(ex,'long',jobs([winner],[-1,1],40,.00025))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--reverse-seeds', type=Path, help='Use first-generation backward edge pivots from this prior search')
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    seed = 2026112111 if args.reverse_seeds else 2026112109
    rng=np.random.default_rng(seed)
    cfg=read('configs/motors-speed-trained.json')
    policies={};history={}
    bounds=dict(BOUNDS,roll_s=(.04,.25),yaw_s=(.04,.32),y_s=(.04,.25),x_arc=(-10,10),z_arc=(-15,12))
    def fresh():
        p={k:float(rng.uniform(*b)) for k,b in bounds.items()}
        p.update(roll=float(rng.uniform(27,39)),y=float(rng.choice([-1,1])*rng.uniform(16,24)),
                 yaw=float(rng.choice([-1,1])*rng.uniform(10,40)),roll_delay=.08,y_delay=float(rng.uniform(.04,.13)),
                 roll_s=float(rng.uniform(.055,.15)),y_s=float(rng.uniform(.05,.14)),
                 yaw_delay=float(rng.uniform(.03,.22)),yaw_s=float(rng.uniform(.06,.17)),brake=float(rng.uniform(.025,.12)),
                 swings_per_block=8)
        return p
    def mutate(parent):
        p={k:float(np.clip(rng.normal(parent[k],(hi-lo)*.10),lo,hi)) for k,(lo,hi) in bounds.items()}
        p['swings_per_block']=parent['swings_per_block'] if rng.random()<.80 else int(rng.choice([6,8,10]))
        return p
    def jobs(names,sides,t,dt):
        return [(n,policies[n],s,t,dict(cfg,timestep_s=dt),None) for n in names for s in sides]
    def batch(ex,label,jobs):
        rows=list(ex.map(rollout,jobs))
        for r in rows:r['cadence_weighted_speed']=float(speed_score(r))
        history[label]=rows
        save(args.out/'history.json',history)
        b=max(rows,key=quality)
        print(label,len(rows),sum(r['passed'] for r in rows),'best',b['name'],round(b['distance_mm'],1),round(b['peak_tilt_deg'],1),
              'max tilt',round(max(r['peak_tilt_deg'] for r in rows),1),flush=True)
        return rows
    def group(name,rows):
        checks=[r for r in rows if r['name']==name]
        return (sum(quality(r)[0] for r in checks),sum(r['passed'] for r in checks),min(speed_score(r) for r in checks))
    pool=[]
    with ProcessPoolExecutor(max_workers=4) as ex:
        for g in range(4):
            new=[]
            if g==0 and args.reverse_seeds:
                previous=read(args.reverse_seeds/'candidates.json')
                backward=[r for r in read(args.reverse_seeds/'history.json')['search-0']
                          if r['completed'] and r['settled'] and r['distance_mm']<0 and r['cadence_ok']]
                for j,r in enumerate(sorted(backward,key=lambda r:r['rim_lift_mm'],reverse=True)[:3]):
                    for mask in range(8):
                        p=dict(previous[r['name']])
                        for flag,keys in [(1,['yaw']),(2,['x','x_arc']),(4,['pitch'])]:
                            if mask&flag:
                                for k in keys:p[k]*=-1
                        n=f'redirect-{j}-{mask}'
                        new.append(n);policies[n]=p
            for i in range(36):
                n=f'impulse-{g}-{i}'
                policies[n]=mutate(policies[pool[i%len(pool)]]) if pool and i%6 else fresh()
                new.append(n)
            rows=batch(ex,f'search-{g}',jobs(new+pool,[1 if g%2==0 else -1],8,.001))
            by_quality=sorted(rows,key=quality,reverse=True)
            # Carry an edge-lift line alongside the highest-scoring feasible motions.
            lifted=sorted((r for r in rows if r['completed'] and r['settled'] and r['distance_mm']>0),
                          key=lambda r:r['rim_lift_mm'],reverse=True)
            pool=list(dict.fromkeys([r['name'] for r in by_quality[:6]+lifted[:3]]))
            save(args.out/'candidates.json',policies)
        rows=batch(ex,'development',jobs(pool,[-1,1],16,.0005))
        finalists=sorted(pool,key=lambda n:group(n,rows),reverse=True)[:4]
        checks=batch(ex,'nominal',jobs(finalists,[-1,1],24,.00025))
        winner=max(finalists,key=lambda n:group(n,checks));p=policies[winner]
        save(args.out/'policy.json',dict(p,selected_candidate=winner,training_seed=seed,
             evidence='uncalibrated_simulation',hardware_ready=False))
        print('frozen',winner,flush=True)
        validate_frozen(ex, batch, cfg, winner, p, seed+1, args.out)


if __name__=='__main__':
    main()
