"""Learn the timing of the head orbit around the observed foot inclination."""
import argparse
import numpy as np
from train_gyro_peg import WORK,POLICY,batch,score,read,write


def rank(row):
    if not row['safe']: return -1000.+row['elapsed_s']
    # Reward the weakest portion of each lap, preventing a large peak tilt
    # from compensating for repeatedly collapsing onto the flat foot.
    return (score(row)+1.7*min(25.,row['low_lean_deg'])
            + min(6.,row['longest_rim_turns']) + 8*row['loaded_fraction'])


def train(workers):
    rng=np.random.default_rng(65691)
    initial=read(POLICY)
    write(WORK/'open-loop.json',initial)
    trials=read(WORK/'refine.json')+read(WORK/'screen.json')
    seed=[]
    for r in sorted(trials,key=rank,reverse=True):
        if all(r['params']!=x['params'] for x in seed): seed.append(r)
        if len(seed)==8:break
    jobs=[]
    for i in range(144):
        r=seed[i%len(seed)]
        f=dict(lead_deg=float(rng.uniform(-180,180)),gain_per_deg=float(rng.uniform(-4,4)),
               target_lean_deg=20.,sense=r['direction'] if i%11 else -r['direction'],
               lock_gain=float(rng.uniform(.06,.60)))
        jobs.append((dict(r['params'],tilt_feedback=f),'nominal',1,8.,None))
    trained=batch(jobs,workers)
    write(WORK/'phase-search.json',trained)
    parents=sorted(trained,key=rank,reverse=True)[:8]
    jobs=[]
    for i in range(96):
        p=dict(parents[i%len(parents)]['params'])
        f=dict(p['tilt_feedback'])
        if i>=len(parents):
            f['lead_deg']+=float(rng.normal(0,16.))
            f['gain_per_deg']+=float(rng.normal(0,1.8))
            f['lock_gain']=float(np.clip(f['lock_gain']+rng.normal(0,.07),.025,.9))
            p['period_frames']=int(np.clip(p['period_frames']+rng.integers(-2,3),10,48))
            p['pitch_phase_deg']+=float(rng.normal(0,8.))
            p['xy_phase_deg']+=float(rng.normal(0,10.))
            p['xy_mm']=float(np.clip(p['xy_mm']+rng.normal(0,2),1,25))
        p['tilt_feedback']=f
        jobs.append((p,'nominal',1,9.,None))
    tuned=batch(jobs,workers)
    write(WORK/'phase-refine.json',tuned)
    candidates=[(initial['orbit'],initial['foot_winding_for_positive_mirror'])]
    for r in sorted(trained+tuned,key=rank,reverse=True):
        if all(r['params']!=p for p,_ in candidates):candidates.append((r['params'],r['direction']))
        if len(candidates)==7:break
    checks=batch([(p,c,m,18.,s*m) for p,s in candidates for c in ('nominal','dev') for m in (1,-1)],workers)
    write(WORK/'phase-finalists.json',checks)
    def key(i):
        g=checks[4*i:4*i+4]
        return (sum(r['safe'] for r in g),sum(r['wide_pass'] for r in g),sum(r['passed'] for r in g),
                min(map(rank,g)),float(np.mean(list(map(rank,g)))))
    n=max(range(len(candidates)),key=key)
    p,s=candidates[n]
    write(POLICY,dict(orbit=p,foot_winding_for_positive_mirror=s,waist_target_rad=0.,command_hz=50))
    print('Selected finalist',n,key(n),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=8)
    train(parser.parse_args().workers)
