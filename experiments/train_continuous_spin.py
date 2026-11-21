"""Search for continuous, head-driven turns with the simulated battery motor model."""
import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from reachy_locomotion.continuous_spin import Orbit, SpinController, spin_metrics
from reachy_locomotion.primitives import Primitive
from train_rim_efficiency import MODELS, simulator
from train_target_pivot import read, write

WORK = Path('experiments/generated/continuous-spin')
POLICY = Path('configs/continuous-spin.json')
RIM_POLICY = Path('configs/continuous-rim-spin.json')
REPORT = Path('experiments/reports/motor-training-continuous-spin.json')
RESERVED = {
    'reserve_nominal': (.685,4.96,.98,1,'torso',.0004,1,.16,713),
    'reserve_slick': (.565,4.82,.95,1,'torso',.0004,2,.19,714),
    'reserve_grip': (.824,5.25,1.045,1,'torso',.0004,1,.14,715),
    'reserve_mass': (.775,5.08,1.015,2,'uniform',.0004,1,.17,716),
}
MODELS.update(RESERVED)


def trial(job):
    params,c,duration,mirror,direction,path,*hold=job
    sim=simulator(c)
    delay,noise,seed=MODELS[c][-3:]
    ctl=SpinController(Orbit(**params),sim.checked_ik,sim.command_limits,mirror,direction=direction,
                       delay=delay,noise=noise,seed=seed,hold_at=hold[0] if hold else None)
    result=sim.run_primitive(Primitive(),seconds=duration,controller=ctl,record=bool(path),
                             waist_mode='hold_neutral',tilt_limit_deg=None)
    m=spin_metrics(result,duration,direction)
    if hold:
        rows=result['trace']
        start=min(rows,key=lambda r:abs(r['time_s']-hold[0]))
        later=[r for r in rows if r['time_s']>=hold[0]+.5]
        m['yaw_after_hold_deg']=direction*(rows[-1]['foot_yaw_deg']-start['foot_yaw_deg'])
        m['yaw_after_first_half_second_deg']=direction*(rows[-1]['foot_yaw_deg']-later[0]['foot_yaw_deg']) if later else None
    if path:
        head=np.degrees([row['head_rpy_rad'] for row in result['trajectory']])
        measured=dict(head_min_rpy_deg=head.min(axis=0).tolist(),head_max_rpy_deg=head.max(axis=0).tolist(),
                      waist_range_deg=result['measured_waist_angle_range_deg'],
                      peak_motor_torques_nm=result['peak_motor_torque_nm'])
        write(path,dict(title='Continuous head-driven spin',measured=measured,config=result['config'],trace=result['trace'],
                        trajectory=[dict(qpos=r['qpos']) for r in result['trajectory']],metrics=dict(m,condition=c,mode='continuous_spin')))
    return dict(params=params,condition=c,mirror=mirror,**m)


def batch(jobs, workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(trial,jobs))


def score(r):
    if r['termination']!='completed' or r['joint_margin_rad']<0:
        return -10000.+r['elapsed_s']
    speed=r['rpm']*6
    return (min(speed,r['last_half_deg_s']) + .25*speed + 8*r['forward_window_fraction'] + 15*r['forward_frame_fraction']
            - .25*max(0.,r['max_excursion_mm']-25.) - .1*r['reverse_deg']/r['elapsed_s'])


def parameters(rng):
    pitch=rng.uniform(10,33)
    return asdict(Orbit(int(rng.integers(12,35)),rng.uniform(10,38),pitch,rng.uniform(-130,-50),
                        rng.uniform(2,25),rng.uniform(-180,180),rng.uniform(-42,42),
                        rng.uniform(-180,180),rng.uniform(-12,2),rng.uniform(-5,5),int(rng.integers(1,3))))


def screen(workers):
    rng=np.random.default_rng(21411)
    params=[parameters(rng) for _ in range(96)]
    rows=batch([(p,'nominal',6.,1,1,None) for p in params],workers)
    # Chirality is learned on training data. Later selection and qualification
    # use the resulting fixed signed command, including on unseen surfaces.
    write(WORK/'screen.json',rows)
    for r in sorted(rows,key=lambda r:-abs(r['rpm']) if r['termination']=='completed' and r['joint_margin_rad']>=0 else 100)[:5]:
        print(round(r['rpm'],2),round(r['max_excursion_mm'],1),round(r['peak_tilt_deg'],1))


def evolve(workers):
    rng=np.random.default_rng(55231)
    screen_rows=read(WORK/'screen.json')
    safe=[r for r in screen_rows if r['termination']=='completed' and r['joint_margin_rad']>=0]
    def initial(r):
        sign=np.sign(r['rpm'])
        return min(abs(r['rpm'])*6,sign*r['last_half_deg_s'])-.3*max(0.,r['max_excursion_mm']-25.)
    parents=sorted(safe,key=initial,reverse=True)[:10]
    jobs=[]
    for i in range(100):
        r=parents[i%len(parents)]
        p=dict(r['params'])
        if i>=len(parents):
            p['period_frames']=int(np.clip(p['period_frames']+rng.integers(-3,4),10,52))
            for k,sd,lo,hi in [('roll_deg',3,3,40),('pitch_deg',3,3,35),('pitch_phase_deg',12,-180,0),
                               ('xy_mm',2.,0,25),('xy_phase_deg',17,-240,240),('yaw_deg',5,-55,55),
                               ('yaw_phase_deg',15,-240,240),('z_mm',2,-18,7)]:
                p[k]=float(np.clip(p[k]+rng.normal(0,sd),lo,hi))
        jobs.append((p,'nominal',8.,1,int(np.sign(r['rpm']) or 1),None))
    rows=batch(jobs,workers)
    write(WORK/'evolve.json',rows)
    for r in sorted(rows,key=score,reverse=True)[:4]:
        print(round(r['rpm'],2),round(r['forward_frame_fraction'],2),round(r['max_excursion_mm'],1),flush=True)


def compare(workers):
    rows=sorted(read(WORK/'evolve.json'),key=score,reverse=True)
    pool=rows[:8]
    cases=[('nominal',1),('slick',1),('inertia',1),('nominal',-1)]
    jobs=[(r['params'],c,15.,mirror,r['direction']*mirror,None) for r in pool for c,mirror in cases]
    results=batch(jobs,workers)
    groups=[results[i:i+len(cases)] for i in range(0,len(results),len(cases))]
    write(WORK/'compare.json',groups)
    for group in groups:
        print([(r['condition'],round(r['rpm'],1),round(r['forward_frame_fraction'],2),round(r['max_excursion_mm']),r['termination']) for r in group],flush=True)


def drive(workers):
    rng=np.random.default_rng(73152)
    ranked=sorted(read(WORK/'evolve.json'),key=score,reverse=True)
    elite=ranked[:12]
    jobs=[]
    for i in range(120):
        parent=elite[i%len(elite)]
        p=dict(parent['params'])
        p['period_frames']=int(np.clip(p['period_frames']+rng.integers(-2,3),10,45))
        p['vertical_mm']=float(rng.uniform(1,min(18,25-abs(p['z_mm']))))
        p['vertical_phase_deg']=float(rng.uniform(-180,180))
        p['yaw_stroke_fraction']=float(rng.uniform(.2,.85)) if i%3 else 0.
        p['yaw_phase_deg']+=float(rng.uniform(-20,20))
        p['yaw_deg']=float(np.clip(p['yaw_deg']+rng.normal(0,4),-55,55))
        jobs.append((p,'nominal',7.,1,parent['direction'],None))
    rows=batch(jobs,workers)
    write(WORK/'drive.json',rows)
    for r in sorted(rows,key=score,reverse=True)[:3]:
        print(round(r['rpm'],2),round(r['forward_frame_fraction'],2),round(r['max_excursion_mm'],1),flush=True)


def rim_search(workers):
    rng=np.random.default_rng(97321)
    jobs=[]
    for i in range(112):
        amp=float(rng.uniform(14,36))
        p=asdict(Orbit(period_frames=int(rng.integers(10,22)),roll_deg=amp,
                       pitch_deg=float(np.clip(amp*rng.uniform(.8,1.20),6,38)),
                       pitch_phase_deg=float(rng.uniform(-110,-70)),xy_mm=float(rng.uniform(5,25)),
                       xy_phase_deg=float(rng.uniform(-180,180)),xy_sense=-1 if i%5 else 1,
                       yaw_deg=float(rng.uniform(-28,28)),yaw_phase_deg=float(rng.uniform(-180,180)),
                       yaw_harmonic=2 if i%2 else 1,z_mm=float(rng.uniform(-12,2)),
                       vertical_mm=float(rng.uniform(0,10)),vertical_phase_deg=float(rng.uniform(-180,180))))
        jobs.append((p,'nominal',7.,1,1,None))
    write(WORK/'rim-search.json',batch(jobs,workers))


def cross(workers):
    candidates=[]
    for stage in ('drive','rim-search','evolve'):
        group=[r for r in read(WORK/f'{stage}.json') if r['joint_margin_rad']>=0 and r['termination']=='completed']
        for rank in (
            lambda r: score(r),
            lambda r: score(r)+23*r['edge_fraction']-.35*r['max_excursion_mm'],
            lambda r: 40*r['forward_frame_fraction']+r['rpm']*5-.5*r['max_excursion_mm'],
        ):
            for r in sorted(group,key=rank,reverse=True)[:3]:
                if r not in candidates:
                    candidates.append(r)
    cases=[('nominal',1),('slick',1),('inertia',1),('grip',-1),('nominal',-1)]
    jobs=[(r['params'],c,12.,m,m*r['direction'],None) for r in candidates for c,m in cases]
    rs=batch(jobs,workers)
    groups=[rs[i:i+len(cases)] for i in range(0,len(rs),len(cases))]
    write(WORK/'cross.json',groups)
    for group in sorted(groups,key=group_score,reverse=True)[:4]:
        print([(round(r['rpm'],1),round(r['max_excursion_mm']),round(r['forward_frame_fraction'],2),round(r['edge_fraction'],2)) for r in group],flush=True)


def group_score(group):
    def steady(r):
        return (r['termination']=='completed' and r['joint_margin_rad']>=0 and r['rpm']>1.5
                and r['max_excursion_mm']<=56 and r['forward_window_fraction']>=.98 and r['forward_frame_fraction']>=.80)
    return (sum(steady(r) for r in group),
            sum(r['termination']=='completed' and r['joint_margin_rad']>=0 for r in group),
            float(np.mean([score(r)+10*r['edge_fraction'] for r in group])))


def feedback(workers):
    groups=sorted(read(WORK/'cross.json'),key=group_score,reverse=True)
    # Include one high-rim-contact parent even if shallow spin is initially steadier.
    eligible=[g for g in groups if all(r['termination']=='completed' and r['joint_margin_rad']>=0 for r in g)]
    parents=groups[:3]
    if eligible:
        edge=max(eligible,key=lambda g:np.mean([r['edge_fraction'] for r in g]))
        if edge not in parents:
            parents.append(edge)
    jobs=[]
    for g in parents:
        parent=g[0]
        for gain in (0.,-.18,.12,.28):
            p=dict(parent['params'],rate_gain=gain,reference_deg_s=max(7.,parent['rpm']*6))
            for c,m in [('nominal',1),('slick',1),('inertia',1),('grip',-1),('nominal',-1)]:
                jobs.append((p,c,16.,m,parent['direction']*m,None))
    rs=batch(jobs,workers)
    fitted=sorted([rs[i:i+5] for i in range(0,len(rs),5)],key=group_score,reverse=True)
    write(WORK/'feedback.json',fitted)
    for g in fitted[:3]:
        print(g[0]['params']['rate_gain'],group_score(g),flush=True)


def refine(workers):
    rng=np.random.default_rng(61034)
    groups=read(WORK/'cross.json')
    parents=sorted(groups,key=group_score,reverse=True)[:3]
    safe=[g for g in groups if g[0]['termination']=='completed' and g[0]['max_excursion_mm']<56]
    parents += sorted(safe,key=lambda g:g[0]['rpm']+3*g[0]['edge_fraction'],reverse=True)[:3]
    jobs=[]
    for i in range(120):
        r=parents[i%len(parents)][0]
        p=asdict(Orbit(**r['params']))
        p['period_frames']=int(np.clip(p['period_frames']+rng.integers(-2,3),10,35))
        p['pitch_bias_deg']=float(np.clip(p['pitch_bias_deg']+rng.normal(0,2),-8,8))
        for k,sd,lo,hi in [('pitch_deg',2,3,40-abs(p['pitch_bias_deg'])),('roll_deg',2.5,3,40),
                           ('yaw_deg',4,-55,55),('xy_mm',1.7,0,25),('pitch_phase_deg',7,-135,-45),
                           ('xy_phase_deg',10,-250,250),('yaw_phase_deg',12,-250,250),
                           ('vertical_mm',2,0,25-abs(p['z_mm']))]:
            p[k]=float(np.clip(p[k]+rng.normal(0,sd),lo,hi))
        jobs.append((p,'nominal',9.,1,r['direction'],None))
    rows=batch(jobs,workers)
    good=[r for r in rows if r['termination']=='completed' and r['joint_margin_rad']>=0]
    top=[]
    for key in (score,lambda r:score(r)+23*r['edge_fraction']-.4*r['max_excursion_mm']):
        top.extend(r for r in sorted(good,key=key,reverse=True)[:6] if r not in top)
    cases=[('nominal',1),('slick',1),('inertia',1),('grip',-1),('nominal',-1)]
    extended=batch([(r['params'],c,16.,m,m*r['direction'],None) for r in top for c,m in cases],workers)
    write(WORK/'refine.json',dict(search=rows,groups=[extended[i:i+5] for i in range(0,len(extended),5)]))


def long_probe(workers):
    best=max(read(WORK/'cross.json'),key=group_score)[0]
    jobs=[(best['params'],c,60.,m,m*best['direction'],None)
          for c,m in [('nominal',1),('slick',1),('nominal',-1)]]
    rs=batch(jobs,workers)
    write(WORK/'long-probe.json',rs)
    for r in rs:
        print(r['condition'],r['mirror'],r['passed'],round(r['base_deg']/360,2),round(r['max_excursion_mm'],1),flush=True)


def lean(workers):
    rng=np.random.default_rng(20419)
    groups=read(WORK/'cross.json')
    ranked=[g[0] for g in groups if g[0]['termination']=='completed' and g[0]['joint_margin_rad']>=0]
    parents=sorted(ranked,key=lambda r:score(r)+35*r['edge_fraction'],reverse=True)[:6]
    jobs=[]
    for i in range(72):
        parent=parents[i%len(parents)]
        p=asdict(Orbit(**parent['params']))
        p['roll_bias_deg']=float(rng.uniform(-13,13))
        p['roll_deg']=min(40-abs(p['roll_bias_deg']),p['roll_deg']*float(rng.uniform(.88,1.2)))
        p['y_bias_mm']=float(rng.uniform(-11,11))
        p['xy_mm']=min(p['xy_mm'],25.-abs(p['y_bias_mm']))
        p['pitch_phase_deg']+=float(rng.uniform(-12,12))
        jobs.append((p,'nominal',8.,1,parent['direction'],None))
    rows=batch(jobs,workers)
    rank=lambda r:score(r)+30.*r['edge_fraction']-.25*r['max_excursion_mm']
    top=sorted(rows,key=rank,reverse=True)[:6]
    cases=[('nominal',1),('slick',1),('inertia',1),('grip',-1),('nominal',-1)]
    runs=batch([(r['params'],c,16.,m,m*r['direction'],None) for r in top for c,m in cases],workers)
    write(WORK/'lean.json',dict(search=rows,groups=[runs[i:i+5] for i in range(0,len(runs),5)]))


def save_policy(path, row):
    motor_file = Path('configs/motors-speed-trained.json')
    write(path, dict(orbit=row['params'], positive_mirror_direction=row['direction'] * row['mirror'],
                     command_hz=50, waist_target_rad=0.,
                     motor_config_sha256=hashlib.sha256(motor_file.read_bytes()).hexdigest()))


def freeze(workers):
    candidates=read(WORK/'feedback.json')+read(WORK/'refine.json')['groups']+read(WORK/'lean.json')['groups']
    ordered=sorted(candidates,key=group_score,reverse=True)
    finalists=ordered[:4]
    strong=[g for g in ordered if group_score(g)[0]>=3]
    if strong:
        lifted=max(strong,key=lambda g:np.mean([r['edge_fraction'] for r in g]))
        if lifted not in finalists:
            finalists.append(lifted)
    contexts=[('nominal',1),('nominal',-1),('slick',1),('grip',-1),('inertia',1)]
    jobs=[(g[0]['params'],c,60.,m,m*g[0]['direction'],None) for g in finalists for c,m in contexts]
    rs=batch(jobs,workers)
    groups=[rs[i:i+len(contexts)] for i in range(0,len(rs),len(contexts))]
    def endurance(group):
        return (sum(r['passed'] for r in group),*group_score(group))
    winner=max(groups,key=endurance)
    write(WORK/'endurance.json',groups)
    save_policy(POLICY, winner[0])
    for r in winner:
        print(r['condition'],r['direction'],r['passed'],round(r['rpm'],2),round(r['max_excursion_mm'],1),flush=True)


def qualify(workers):
    policy=read(POLICY)
    p,sign=policy['orbit'],policy['positive_mirror_direction']
    contexts=('nominal','slick','grip','inertia',*RESERVED)
    jobs=[(p,c,60.,m,sign*m,None) for c in contexts for m in (1,-1)]
    rs=batch(jobs,workers)
    # A zero-amplitude control and a fixed head-target coast test isolate
    # spontaneous drift and the spin remaining when the orbit is interrupted.
    neutral=asdict(Orbit(20,0.,0.,-90.,0.,0.,0.,0.,0.))
    controls=batch([(neutral,'nominal',10.,1,1,None),
                    (p,'nominal',15.,1,sign,None,12.),(p,'slick',15.,1,sign,None,12.)],workers)
    search_counts={}
    for stage in ('screen','evolve','compare','drive','rim-search','cross','feedback','refine','lean','endurance','long-probe'):
        data=read(WORK/f'{stage}.json')
        if isinstance(data,dict):
            data=data['search']+sum(data['groups'],[])
        elif data and isinstance(data[0],list):
            data=sum(data,[])
        search_counts[stage]=dict(trials=len(data),falls=sum(x['fell'] for x in data),
                                  joint_stops=sum(x['termination']=='actual_joint_limit_exceeded' for x in data))
    write(REPORT,dict(policy=policy,policy_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest(),
                      conditions={c:MODELS[c] for c in contexts},simulation_only=True,
                      training=search_counts,matched=rs[:8],held_out=rs[8:],controls=controls))
    for section in (rs[:8],rs[8:]):
        print('passed',sum(r['passed'] for r in section),'of',len(section),flush=True)
        for r in section:
            print(r['condition'],r['direction'],round(r['rpm'],2),round(r['max_excursion_mm'],1),
                  r['passed'],r['termination'],flush=True)


def rim_demo(workers):
    """Select a visibly tilted spin using nominal training data alone."""
    pool=[]
    for label in ('screen','rim-search','lean','refine','feedback','drive','evolve','cross'):
        rows=read(WORK/f'{label}.json')
        if isinstance(rows,dict):
            rows=rows['search']+sum(rows['groups'],[])
        if isinstance(rows[0],list):
            rows=sum(rows,[])
        pool.extend(r for r in rows if r['condition']=='nominal' and r['mirror']==1
                    and r['termination']=='completed' and r['joint_margin_rad']>=0
                    and r['forward_frame_fraction']>.785 and r['forward_window_fraction']>.97
                    and r['max_excursion_mm']<=56 and r['rpm']>=2)
    candidates=[]
    for r in sorted(pool,key=lambda r:r['edge_fraction'],reverse=True):
        if all(r['params']!=prior['params'] for prior in candidates):
            candidates.append(r)
        if len(candidates)==8:
            break
    jobs=[(r['params'],'nominal',60.,m,r['direction']*m,None) for r in candidates for m in (1,-1)]
    results=batch(jobs,workers)
    groups=[results[i:i+2] for i in range(0,len(results),2)]
    def rank(group):
        return (sum(r['passed'] for r in group),
                min(r['direction']*r['rim_base_deg']/360 for r in group))
    selected=max(groups,key=rank)
    first=selected[0]
    save_policy(RIM_POLICY, first)
    checks=batch([(first['params'],'reserve_nominal',60.,m,first['direction']*m,None) for m in (1,-1)],workers)
    write(WORK/'rim-demo.json',dict(candidates=results,selected=selected,independent=checks))
    for r in selected+checks:
        print(r['condition'],r['direction'],r['passed'],round(r['rpm'],2),
              round(r['edge_fraction'],2),round(r['rim_base_deg']),flush=True)


def rim_record(workers):
    result=read(WORK/'rim-demo.json')
    report=read(REPORT)
    report['rim_variant']=result
    report['featured_measurements']=read(WORK/'best.json')['measured']
    good=[r for r in result['independent'] if r['passed']]
    if good:
        best=max(good,key=lambda r:r['direction']*r['rim_base_deg'])
        result['featured'] = best
        result['featured_measurements'] = replay_rim(best)
    write(REPORT,report)


def replay_rim(row):
    policy = read(RIM_POLICY)
    if row['params'] != policy['orbit'] or row['direction'] != policy['positive_mirror_direction'] * row['mirror']:
        raise ValueError('Frozen rim policy and requested record differ')
    path = WORK / 'rim-best.json'
    replay = trial((policy['orbit'], row['condition'], row['seconds'], row['mirror'], row['direction'], path))
    if replay != row:
        raise ValueError('Recorded rim spin differs from qualification')
    recorded = read(path)
    recorded['title'] = 'Turning on a raised rim'
    write(path, recorded)
    return recorded['measured']


def record_rim(workers):
    replay_rim(read(REPORT)['rim_variant']['featured'])



def sustain(workers):
    report=read(REPORT)
    policy=read(POLICY)
    if hashlib.sha256(POLICY.read_bytes()).hexdigest()!=report['policy_sha256']:
        raise ValueError('Frozen policy and qualification differ')
    sign=policy['positive_mirror_direction']
    report['three_minute_runs']=batch([(policy['orbit'],'nominal',180.,m,sign*m,None) for m in (1,-1)],workers)
    write(REPORT,report)
    for r in report['three_minute_runs']:
        print(r['direction'],r['base_deg']/360,r['max_excursion_mm'],r['passed'],flush=True)


def record(workers):
    report=read(REPORT)
    if hashlib.sha256(POLICY.read_bytes()).hexdigest()!=report['policy_sha256']:
        raise ValueError('Frozen policy and qualification differ')
    # Prefer an independently tested multi-revolution spin and rank by sustained speed.
    good=[r for r in report['held_out'] if r['passed']]
    if not good:
        good=[r for r in report['matched'] if r['passed']]
    best=max(good,key=lambda r:(r['rpm'],r['forward_frame_fraction']))
    r=trial((best['params'],best['condition'],best['seconds'],best['mirror'],best['direction'],WORK/'best.json'))
    if r!=best:
        raise ValueError('Recorded spin differs from qualification')
    write(WORK/'featured.json',best)
    print(best['condition'],best['direction'],best['rpm'],flush=True)


if __name__ == '__main__':
    stages = (screen, evolve, compare, drive, rim_search, cross, feedback, refine, lean,
              long_probe, freeze, qualify, sustain, record, rim_demo, rim_record, record_rim)
    commands = {stage.__name__: stage for stage in stages}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=commands)
    parser.add_argument('--workers', type=int, default=7)
    args = parser.parse_args()
    commands[args.stage](args.workers)
