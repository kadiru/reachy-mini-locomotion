"""Search head strokes for loaded-rim torso rotation and qualify connected turns."""
import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.swing_distance import cumulative_strokes, stroke_count
from reachy_locomotion.target_pivot import TargetPivot, edge_turns, pivot_metrics
from train_target_pivot import CONDITIONS, BANK, read, write


WORK = Path('experiments/generated/rim-efficiency')
POLICY = Path('configs/efficient-rim-pivot.json')
REPORT = Path('experiments/reports/motor-training-rim-efficiency.json')
MODELS = dict(CONDITIONS,
              validation_slick=(.565,4.83,.948,1,'torso',.0004,2,.19,819),
              validation_delay=(.785,5.07,1.015,2,'uniform',.0004,1,.17,820),
              validation_middle=(.695,4.99,.99,1,'torso',.0004,1,.14,821))
CACHE = {}


def simulator(condition):
    if condition not in CACHE:
        # Each MuJoCo model owns the robot meshes. Keep worker memory bounded.
        if len(CACHE)>=2:
            CACHE.pop(next(iter(CACHE)))
        friction, volts, gain, delay, mass, dt, *_ = MODELS[condition]
        config = read('configs/motors-speed-trained.json')
        config['friction_slide_torsion_roll'][0] = friction
        config['timestep_s'] = dt
        config['motor_model'].update(voltage_v=volts, pid_scale=gain, delay_frames=delay, mass_distribution=mass)
        CACHE[condition] = PrimitiveSimulator('experiments/upstream-reachy-mini', config)
    return CACHE[condition]


def sample_candidates():
    parent = read('configs/penguin-sustained-trained.json')['primary']
    rng = np.random.default_rng(32619)
    candidates = []
    for i in range(112):
        recipe = dict(parent['recipe'])
        feedback = dict(parent['feedback'])
        recipe.update(preload_roll_deg=float(rng.uniform(-40,-20)), preload_y_mm=float(rng.uniform(6,25)),
                      preload_s=float(rng.uniform(.32,.85)), roll_deg=float(rng.uniform(29,40)),
                      y_mm=float(rng.uniform(-25,-5)), x_mm=float(rng.uniform(-5,23)),
                      pitch_deg=float(rng.uniform(-16,10)), pump_hz=float(rng.uniform(2.0,3.35)),
                      pump_y_phase_deg=float(rng.uniform(-40,85)), pump_cycles=0 if i%4==0 else 8,
                      swing_s=float(rng.uniform(.055,.16)), yaw_deg=int(rng.choice([-60,-45,35,50,60])),
                      yaw_s=float(rng.uniform(.045,.15)), yaw_at_roll_deg=float(rng.uniform(2,14)),
                      yaw_hold_s=float(rng.uniform(.12,.50)), yaw_return_s=float(rng.uniform(.25,.65)),
                      catch_hold_s=float(rng.uniform(.08,.34)))
        feedback.update(trigger_deg=float(rng.uniform(6,20)), prediction_s=float(rng.uniform(.02,.12)),
                        yaw_before_catch=bool(i%3), yaw_direction=int(rng.choice([-1,0,1])),
                        yaw_prediction_s=float(rng.uniform(.01,.12)), yaw_delay_after_catch_s=0.,
                        workspace_joint_margin_rad=float(rng.uniform(.075,.15)), sensor_latency_compensation=1.)
        candidates.append(dict(id=f'rim-{i}',recipe=recipe,feedback=feedback,side=1))
    return candidates


def windup_candidates():
    rng = np.random.default_rng(51489)
    parent = read('configs/penguin-sustained-trained.json')['primary']
    candidates = []
    for i in range(104):
        recipe = dict(parent['recipe'], yaw_deg=int(rng.choice([-60,-45,45,60])),
                      x_mm=float(rng.uniform(-15,20)), pitch_deg=float(rng.uniform(-10,12)),
                      pump_hz=float(rng.uniform(2.1,3.3)), preload_s=float(rng.uniform(.40,.8)),
                      pump_cycles=0 if i%5==0 else 8, yaw_s=float(rng.uniform(.06,.19)),
                      yaw_at_roll_deg=float(rng.uniform(2,11)), yaw_hold_s=float(rng.uniform(.15,.45)),
                      yaw_return_s=float(rng.uniform(.35,.8)))
        feedback = dict(parent['feedback'], yaw_windup_deg=float(rng.uniform(25,60)),
                        yaw_before_catch=True, yaw_direction=int(rng.choice([-1,0,1])),
                        yaw_prediction_s=float(rng.uniform(.01,.1)), trigger_deg=float(rng.uniform(7,18)),
                        workspace_joint_margin_rad=float(rng.uniform(.08,.15)))
        candidates.append(dict(id=f'wind-{i}',side=1,recipe=recipe,feedback=feedback))
    return candidates


def surge_candidates():
    rng=np.random.default_rng(63783)
    seeds=read(BANK)['options']
    seeds=[o for o in seeds if o['id'] in ('primary-10-+1','search-4-+1','fallback-55-+1','search-0-+1')]
    options=[]
    for i in range(108):
        o=seeds[i%len(seeds)]
        sign=int(rng.choice([-1,1]))
        g=dict(o['recipe'], x_mm=sign*float(rng.uniform(7,24)), yaw_deg=int(rng.choice([-55,-25,25,55])),
               yaw_s=float(rng.uniform(.06,.17)), yaw_hold_s=float(rng.uniform(.14,.38)),
               yaw_at_roll_deg=float(rng.uniform(2,12)),yaw_return_s=float(rng.uniform(.35,.75)))
        f=dict(o['feedback'],edge_surge_mm=-sign*float(rng.uniform(22,48)),yaw_before_catch=True,
               yaw_direction=int(rng.choice([-1,1])),yaw_prediction_s=float(rng.uniform(.03,.16)),
               trigger_deg=float(rng.uniform(7,17)),workspace_joint_margin_rad=float(rng.uniform(.075,.14)))
        options.append(dict(id=f'surge-{i}',side=1,recipe=g,feedback=f))
    return options


def phased_candidates():
    old = read('experiments/reports/motor-training-balance-pivot-stress-2026-11-19.json')['candidates']
    old = next(c for c in old if c['id']=='pivot-17-yaw-12')
    primary = read('configs/penguin-sustained-trained.json')['primary']
    rng=np.random.default_rng(8132)
    options=[]
    for i in range(128):
        parent = primary if i%2 else old
        g = dict(parent['recipe'])
        g.update(pump_hz=float(rng.uniform(2.25,3.2)),pitch_deg=float(rng.uniform(-13,6)),
                 x_mm=float(rng.uniform(4,23)),yaw_deg=int(rng.choice([-50,-25,0,25,50])))
        f = dict(parent['feedback'], workspace_projection=True,return_x_after_catch_s=.6,
                 return_x_duration_s=.4,workspace_joint_margin_rad=float(rng.uniform(.03,.11)),
                 pump_yaw_deg=float(rng.uniform(12,52)),pump_yaw_phase_deg=float(rng.uniform(-180,180)),
                 trigger_deg=float(rng.uniform(6,19)), prediction_s=float(rng.uniform(.025,.09)))
        options.append(dict(id=f'phase-{i}',side=1,recipe=g,feedback=f))
    return options


class Sequence(TargetPivot):
    def __init__(self,bank,goal,ik,**args):
        super().__init__(bank,goal,ik,single=True,**args)
        self.single=len(bank)==1

    def choose(self, error):
        return self.bank[min(len(self.strokes),len(self.bank)-1)]


def measures(result):
    t = result['trace']
    edge = sum(edge_turns(t,'torso_yaw_deg').values())
    base_edge = sum(edge_turns(t,'foot_yaw_deg').values())
    angles = np.degrees([r['head_rpy_rad'] for r in result['trajectory']])
    roll, yaw = (stroke_count(angles[:,i]) for i in (0,2))
    return dict(torso_edge_deg=edge, base_edge_deg=base_edge, measured_roll_swings=roll, measured_yaw_swings=yaw,
                measured_swings=max(roll,yaw))


def run(job, *, policy_class=None):
    bank, goal, condition, mode, path, *override = job
    sim = simulator(condition)
    delay, noise, seed = MODELS[condition][-3:]
    if mode == 'sequence':
        controller = Sequence(bank,goal,sim.checked_ik,seed=seed,delay=delay,noise=noise,
                              max_strokes=len(bank))
    elif mode == 'baseline':
        controller = TargetPivot(bank,goal,sim.checked_ik,seed=seed,delay=delay,noise=noise,
                                 **read(BANK)['control'])
    else:
        from reachy_locomotion.efficient_pivot import EfficientPivot
        controller_type = policy_class or EfficientPivot
        controller = controller_type(bank,goal,sim.checked_ik,seed=seed,delay=delay,noise=noise,
                                    settings=override[0] if override else None)
    result = sim.run_primitive(Primitive(),seconds=4.4*controller.max_strokes+1.,controller=controller,
                               waist_mode='hold_neutral',record=True,tilt_limit_deg=None,finish_settle_s=.8)
    m = pivot_metrics(result,goal)
    m.update(measures(result), condition=condition, strokes=len(controller.strokes),
             sequence=controller.strokes, joint_margin_rad=result['min_joint_margin_rad'])
    if result['termination']=='actual_joint_limit_exceeded':
        q=np.asarray(result['terminal_state']['qpos'])[sim.joint_qpos]
        margin=np.minimum(q-sim.command_limits[:,0],sim.command_limits[:,1]-q)
        actuator=sim.act[int(np.argmin(margin))]
        m['guard_joint']=sim.model.joint(int(sim.model.actuator_trnid[actuator,0])).name
    m['mean_slip_mm']=m['slip_mm']/max(1,m['strokes'])
    m['passed'] = bool(m['passed'] and m['mean_slip_mm']<=15 and
                       np.sign(goal)*m['torso_edge_deg']>=.4*abs(goal))
    if mode == 'sequence' and len(bank)>1:
        m['segments'] = []
        for item in controller.strokes:
            rows = [r for r in result['trace'] if item['start_s']-1e-8 <= r['time_s']
                    <= item.get('end_s',m['elapsed_s'])+1e-8]
            traj = [r for r in result['trajectory'] if rows[0]['time_s']<=r['time_s']<=rows[-1]['time_s']]
            seg = measures(dict(trace=rows,trajectory=traj))
            seg['turn'] = rows[-1]['torso_yaw_deg'] - rows[0]['torso_yaw_deg']
            delta = 1000*(np.array(rows[-1]['foot_xyz_m'][:2])-rows[0]['foot_xyz_m'][:2])
            a = np.radians(rows[0]['foot_yaw_deg'])
            seg['dx'] = float(np.cos(a)*delta[0]+np.sin(a)*delta[1])
            seg['dy'] = float(-np.sin(a)*delta[0]+np.cos(a)*delta[1])
            seg['drift'] = float(np.linalg.norm(delta))
            seg['roll'] = item['peak_sensed_roll_deg']
            m['segments'].append(seg)
    if path:
        trajectory = [{k:r[k] for k in ('time_s','qpos','foot_rpy_rad')} for r in result['trajectory']]
        trace = [{k:r[k] for k in ('time_s','foot_xyz_m','foot_yaw_deg','torso_yaw_deg','roll_deg',
                                    'edge_support','side_height_difference_mm')} for r in result['trace']]
        angles=np.degrees([r['head_rpy_rad'] for r in result['trajectory']])
        counts=np.maximum(list(cumulative_strokes(angles[:,0])),list(cumulative_strokes(angles[:,2])))
        net=0.
        for j,row in enumerate(trace):
            if j and row['edge_support'] and trace[j-1]['edge_support'] and row['roll_deg']*trace[j-1]['roll_deg']>0:
                net+=row['torso_yaw_deg']-trace[j-1]['torso_yaw_deg']
            row.update(head_swings=int(counts[j]),torso_edge_deg=net,
                       stroke_number=sum(s['start_s']<=row['time_s'] for s in controller.strokes))
        write(path,dict(config=result['config'],trajectory=trajectory,trace=trace,metrics=m,
                        elapsed_seconds=result['elapsed_seconds'],terminal_state=result['terminal_state']))
    return m


def evaluate(jobs,workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(run,jobs))


FIT_FLOORS = ('nominal','slick','inertia','fresh_low','grip','fresh_mass')


def fitness(row):
    if not row['settled'] or row['mean_slip_mm']>15 or row['joint_margin_rad'] < .004:
        return -100.
    sign = np.sign(row['torso_deg'])
    total = abs(row['torso_deg'])
    edge = min(sign*row['signed_edge_deg'], sign*row['torso_edge_deg'])
    return (min(total,edge) + .35*total + 1.6*min(total,edge)/max(1,row['measured_swings'])
            - .25*row['drift_mm'] - .75*max(0.,row['drift_mm']-.60*total))


def evolve(workers):
    rng = np.random.default_rng(97211)
    seeds = read(BANK)
    previous = read(WORK/'search.json')
    sampled = sample_candidates()
    retained = {}
    for i, cond in enumerate(('slick','nominal','inertia')):
        rank = sorted(zip(sampled,previous[i::3]),key=lambda v:fitness(v[1]),reverse=True)
        profiles = [o for o in seeds['options'] if o['side']==1]
        j = seeds['fit_conditions'].index({'nominal':'fit','inertia':'mass_fit','slick':'slick'}[cond])
        def old_score(o):
            r=o['responses'][j]
            return min(abs(r['turn']),np.sign(r['turn'])*r['edge'])-.22*r['drift']
        parents = sorted(profiles,key=old_score,reverse=True)[:4]+[o for o,_ in rank[:3]]
        retained[cond] = [dict(id='seed-'+o['id'], side=1, recipe=o['recipe'],feedback=o['feedback']) for o in parents]
    archive=[]
    for gen in range(4):
        pool=[]
        for cond,parents in retained.items():
            for j in range(34):
                parent=parents[j%len(parents)]
                g=dict(parent['recipe'])
                f=dict(parent['feedback'])
                if j>=len(parents):
                    scale = 1. if gen==0 else .72
                    def vary(d,key,sigma,lo,hi):
                        d[key]=float(np.clip(d[key]+scale*rng.normal(0,sigma),lo,hi))
                    for key,sd,lo,hi in [('preload_s',.065,.22,1.),('pump_hz',.16,1.65,3.8),
                                          ('pump_y_phase_deg',12,-90,120),('x_mm',4,-22,24),
                                          ('pitch_deg',3,-25,25),('preload_roll_deg',3,-40,-15),
                                          ('yaw_at_roll_deg',2,1,22),('yaw_hold_s',.06,.06,.7)]:
                        vary(g,key,sd,lo,hi)
                    vary(f,'prediction_s',.02,.01,.2)
                    vary(f,'trigger_deg',2.4,3,25)
                    vary(f,'workspace_joint_margin_rad',.018,.055,.15)
                    if rng.random()<.55:
                        f['state_dependent_yaw']=True
                        f['yaw_windup_deg']=float(np.clip(f.get('yaw_windup_deg',0)+rng.normal(4,14),0,55))
                o=dict(id=f'evo-{cond}-{gen}-{j}',side=1,recipe=g,feedback=f)
                pool.append((o,cond))
        rows=evaluate([([o],170,c,'sequence',None) for o,c in pool],workers)
        for (o,c),r in zip(pool,rows):
            archive.append(dict(option=o,condition=c,result=r))
        for cond in retained:
            ranked=sorted([a for a in archive if a['condition']==cond],key=lambda a:fitness(a['result']),reverse=True)
            parents=[]
            for a in ranked:
                o=a['option']
                if not any(o['recipe']==p['recipe'] and o['feedback']==p['feedback'] for p in parents):
                    parents.append(o)
                if len(parents)==6:
                    break
            retained[cond]=parents
            print(gen,cond,round(fitness(ranked[0]['result']),2),flush=True)
        write(WORK/'evolution.json',archive)


def local_search(workers):
    archive=read(WORK/'evolution.json')
    rng=np.random.default_rng(211124)
    new=[]
    for generation in range(4):
        tasks=[]
        for c in ('slick','nominal'):
            ranked=sorted([a for a in archive+new if a['condition']==c],key=lambda a:fitness(a['result']),reverse=True)
            parents=[]
            for a in ranked:
                o=a['option']
                if not any(o['recipe']==x['recipe'] and o['feedback']==x['feedback'] for x in parents):
                    parents.append(o)
                if len(parents)==4:break
            for j in range(36):
                o=parents[j%len(parents)]
                g=dict(o['recipe']); f=dict(o['feedback'])
                fields=[(g,'pump_hz',.16,1.8,3.65),(g,'preload_s',.055,.25,.95),
                        (g,'pitch_deg',5.,-24,22),(g,'x_mm',5.5,-23,25),
                        (g,'yaw_deg',18.,-60,60),(g,'catch_s',.036,.04,.19),
                        (g,'catch_hold_s',.06,.05,.55),(g,'catch_y_mm',4.5,-25,10),
                        (f,'trigger_deg',3.5,3,25),(f,'roll_kp',.65,-3.5,3.5),
                        (f,'roll_kd_s',.07,-.4,.4),(f,'yaw_prediction_s',.035,0,.2),
                        (f,'workspace_joint_margin_rad',.03,.02,.15)]
                for k in rng.choice(len(fields),size=2+j%3,replace=False):
                    d,key,sd,lo,hi=fields[k]
                    d[key]=float(np.clip(d[key]+rng.normal(0,sd),lo,hi))
                v=dict(id=f'local-{c}-{generation}-{j}',side=1,recipe=g,feedback=f)
                tasks.append((v,c))
        rows=evaluate([([o],170,c,'sequence',None) for o,c in tasks],workers)
        new.extend(dict(option=o,condition=c,result=r) for (o,c),r in zip(tasks,rows))
        write(WORK/'local.json',new)
        print(generation,[round(max(fitness(a['result']) for a in new if a['condition']==c),2)
                          for c in ('slick','nominal')],flush=True)


def train_slick_pair(workers):
    rng=np.random.default_rng(88321)
    rows=read(WORK/'surge.json')[::3]
    parents=[o for o,r in sorted(zip(surge_candidates(),rows),key=lambda a:fitness(a[1]),reverse=True)[:5]]
    saved=[]
    for gen in range(2):
        population=[]
        for j in range(28):
            p=parents[j%len(parents)]
            g=dict(p['recipe']); f=dict(p['feedback'])
            if j>=len(parents):
                g['preload_s']=float(np.clip(g['preload_s']+rng.normal(0,.026),.25,.95))
                g['pump_hz']=float(np.clip(g['pump_hz']+rng.normal(0,.09),1.8,3.6))
                g['pitch_deg']=float(np.clip(g['pitch_deg']+rng.normal(0,2),-20,20))
                f['edge_surge_mm']*=float(rng.uniform(.62,1.03))
                f['trigger_deg']=float(np.clip(f['trigger_deg']+rng.normal(-.6,1.6),4,21))
                f['yaw_prediction_s']=float(np.clip(f['yaw_prediction_s']+rng.normal(0,.018),0,.2))
            population.append(dict(id=f'pair-{gen}-{j}',side=1,recipe=g,feedback=f))
        jobs=[([o],170,c,'sequence',None) for o in population for c in ('slick','fresh_low')]
        results=evaluate(jobs,workers)
        for j,o in enumerate(population):
            pair=results[2*j:2*j+2]
            scores=[fitness(r) for r in pair]
            compatible=pair[0]['torso_deg']*pair[1]['torso_deg']>0
            score=min(scores)+.3*max(scores) if compatible else -100.
            saved.append(dict(option=o,results=pair,score=score))
        parents=[a['option'] for a in sorted(saved,key=lambda x:x['score'],reverse=True)[:5]]
        write(WORK/'robust.json',saved)


def finalists():
    options = sample_candidates() + windup_candidates()
    rows = read(WORK/'search.json') + read(WORK/'windup.json')
    selected = {}
    for i, floor in enumerate(('slick','nominal','inertia')):
        viable = [(o,r) for o,r in zip(options,rows[i::3]) if r['settled'] and r['mean_slip_mm']<14
                  and r['joint_margin_rad']>.007 and r['max_excursion_mm']<43]
        for direction in (-1,1):
            correct = [(o,r) for o,r in viable if direction*r['torso_deg']>2 and direction*r['torso_edge_deg']>1]
            for desired,count in ((40,3),(8,1),(90,1)):
                def reward(pair):
                    _,r = pair
                    turn = direction*r['torso_deg']
                    edge = min(direction*r['torso_edge_deg'],direction*r['signed_edge_deg'])
                    if desired==90:
                        return (min(turn,edge) - .12*r['drift_mm'])/max(1,r['measured_swings'])
                    return (min(desired,turn) - .6*max(0,turn-desired)
                            + .9*min(edge,turn) - .3*r['drift_mm'] - .5*r['measured_swings'])
                for o,r in sorted(correct,key=reward,reverse=True)[:count]:
                    selected[o['id']] = o
    for a in sorted(read(WORK/'robust.json'),key=lambda x:x['score'],reverse=True)[:5]:
        o=a['option']
        selected[o['id']]=o
    phase=phased_candidates()
    rows=read(WORK/'phase.json')
    for i in range(3):
        for o,r in sorted(zip(phase,rows[i::3]),key=lambda pair:fitness(pair[1]),reverse=True)[:4]:
            selected[o['id']]=o
    surge=surge_candidates()
    rows=read(WORK/'surge.json')
    for i in range(3):
        for o,r in sorted(zip(surge,rows[i::3]),key=lambda pair:fitness(pair[1]),reverse=True)[:3]:
            selected[o['id']]=o
    evolution = read(WORK/'evolution.json')+read(WORK/'local.json')
    for c in ('slick','nominal','inertia'):
        unique=[]
        for a in sorted([x for x in evolution if x['condition']==c],key=lambda a:fitness(a['result']),reverse=True):
            o=a['option']
            if not any(o['recipe']==v['recipe'] and o['feedback']==v['feedback'] for v in unique):
                unique.append(o)
                selected[o['id']]=o
            if len(unique)==4:
                break
    seeds = read(BANK)['options']
    for name in ('search-4-+1','fallback-55-+1','primary-10-+1'):
        o = next(o for o in seeds if o['id']==name)
        selected[name] = {k:o[k] for k in ('id','side','recipe','feedback')}
    return [dict(o,id=f"{o['id']}/{side:+d}",side=side) for o in selected.values() for side in (1,-1)]


def fit_repeats(workers):
    opts = finalists()
    jobs = [([o,o],170,c,'sequence',None) for o in opts for c in FIT_FLOORS]
    print(f'Fitting {len(opts)} signed actions on {len(FIT_FLOORS)} conditions',flush=True)
    rows = evaluate(jobs,workers)
    write(WORK/'repeats.json',rows)
    write(POLICY,dict(fit_conditions=list(FIT_FLOORS),options=build_bank(opts,rows)))


def build_bank(opts,rows):
    actions=[]
    for i,o in enumerate(opts):
        responses = []
        for c,r in zip(FIT_FLOORS,rows[i*len(FIT_FLOORS):(i+1)*len(FIT_FLOORS)]):
            valid = r['settled'] and r['termination']=='completed' and len(r['segments'])==2
            responses.append(dict(delay=MODELS[c][6], valid=valid, first_completed='end_s' in r['sequence'][0], segments=r['segments'],
                                  margin=r['joint_margin_rad'], slip=r['mean_slip_mm']))
        if any(q['valid'] for q in responses):
            actions.append(dict(o,responses=responses))
    return actions


def train_openings(workers):
    policy=read(POLICY)
    bank=policy['options']
    jobs=[]
    groups=[]
    for delay,floors in ((1,('nominal','inertia')),(2,('slick','fresh_low'))):
        for sign in (-1,1):
            key=f'{delay}/{sign}'
            indices=[j for j,c in enumerate(FIT_FLOORS) if MODELS[c][6]==delay]
            safe=[o for o in bank if all(o['responses'][j]['first_completed'] for j in indices)]
            picks=[None]
            for c in floors:
                j=FIT_FLOORS.index(c)
                def score(o):
                    r=o['responses'][j]['segments'][0]
                    edge=min(sign*r['base_edge_deg'],sign*r['torso_edge_deg'])
                    return min(sign*r['turn'],edge)+.35*sign*r['turn']-.30*r['drift']-.35*r['measured_swings']
                for o in sorted(safe,key=score,reverse=True)[:2]:
                    if o['id'] not in picks:picks.append(o['id'])
            groups.append((key,picks,floors))
            for name in picks:
                alternate=[dict(o,opening_for=[key]) if o['id']==name else o for o in bank]
                jobs.extend((alternate,90*sign,c,'efficient',None) for c in floors)
    rows=evaluate(jobs,workers)
    trials=[];start=0
    for key,picks,floors in groups:
        group=[]
        for name in picks:
            rs=rows[start:start+len(floors)];start+=len(floors)
            score=(sum(r['passed'] for r in rs), sum(r['settled'] for r in rs),
                   -sum(r['strokes'] if r['passed'] else 10 for r in rs),
                   -sum(r['measured_swings'] if r['passed'] else 120 for r in rs),
                   -sum(r['max_tail_error_deg'] for r in rs))
            group.append(dict(key=key,action=name,score=score,results=rs))
        winner=max(group,key=lambda a:a['score'])
        if winner['action']:
            next(o for o in bank if o['id']==winner['action']).setdefault('opening_for',[]).append(key)
        trials.extend(group)
        print(key,winner['action'],winner['score'],flush=True)
    write(WORK/'openings.json',trials)
    write(POLICY,policy)


def fit_small_turns(workers):
    bank=read(POLICY)['options']
    for o in bank:o.pop('trim_for', None)
    jobs=[];groups=[]
    for delay,floors in ((1,('nominal','fine','inertia','fresh_mass','grip','fresh_high')),(2,('slick','fresh_low'))):
        indices=[j for j,c in enumerate(FIT_FLOORS) if MODELS[c][6]==delay]
        safe=[o for o in bank if all(o['responses'][j]['first_completed'] for j in indices)]
        for sign in (-1,1):
            key=f'{delay}/{sign}'
            def score(o,j):
                s=o['responses'][j]['segments'][0]
                return (abs(sign*17-s['turn'])+1.5*max(0.,8.-min(sign*s['torso_edge_deg'],sign*s['base_edge_deg']))
                        +.15*s['measured_swings']+.08*s['drift'])
            choices=[None]
            ranked=sorted(safe,key=lambda o:sum(score(o,j) for j in indices))
            for o in ranked[:2]+[min(safe,key=lambda o:score(o,j)) for j in indices]:
                if o['id'] not in choices:choices.append(o['id'])
            groups.append((key,floors,choices))
            for name in choices:
                alt=[dict(o,trim_for=[key]) if o['id']==name else o for o in bank]
                jobs.extend((alt,sign*17,c,'efficient',None) for c in floors)
    rows=evaluate(jobs,workers)
    trials=[];n=0
    for key,floors,choices in groups:
        for name in choices:
            rs=rows[n:n+len(floors)];n+=len(floors)
            score=(sum(r['passed'] for r in rs),sum(r['settled'] for r in rs),
                   -sum(r['strokes'] for r in rs),-sum(r['max_tail_error_deg'] for r in rs))
            trials.append(dict(key=key,opening=name,score=score,results=rs))
    write(WORK/'small-turns.json',trials)


def train_probe_plans(workers):
    bank=read(POLICY)['options']
    floors=('slick','fresh_low')
    indices=[FIT_FLOORS.index(c) for c in floors]
    safe=[o for o in bank if all(o['responses'][j]['first_completed'] for j in indices)]
    def distinguish(o):
        a,b=[o['responses'][j]['segments'][0] for j in indices]
        return (abs(a['turn']-b['turn'])/2.8+abs(a['roll']-b['roll'])/2.5
                -.08*(a['drift']+b['drift']))
    probe=max(safe,key=distinguish)['id']
    jobs=[];entries=[]
    for sign in (-1,1):
        for c,i in zip(floors,indices):
            def benefit(o):
                seg=o['responses'][i]['segments'][-1]
                gain=sign*seg['turn']
                edge=min(sign*seg['torso_edge_deg'],sign*seg['base_edge_deg'])
                return gain+.6*min(gain,edge)-max(0.,.5*gain-edge)-.3*seg['drift']-.25*seg['measured_swings']
            options=sorted((o for o in bank if o['responses'][i]['valid']),key=benefit,reverse=True)
            top=options[0]
            first=top['responses'][i]['segments'][-1]
            def opposing(o):
                seg=o['responses'][i]['segments'][-1]
                return benefit(o)-.65*np.hypot(first['dx']+seg['dx'],first['dy']+seg['dy'])
            counter=max(options,key=opposing)
            names=[o['id'] for o in options[:3]]+[counter['id']]
            plans=[[n] for n in dict.fromkeys(names)]
            plans.extend([[names[0],names[-1]],[names[-1],names[0]],[names[0],names[1]]])
            for plan in plans:
                settings=dict(opening=probe,plans={str(i):plan},temperature=.6,travel_cost=1.,travel_start=38.)
                jobs.append((bank,90*sign,c,'efficient',None,settings))
                entries.append(dict(sign=sign,profile=i,settings=settings))
    results=evaluate(jobs,workers)
    for entry,row in zip(entries,results):entry['result']=row
    combined=[]
    for sign in (-1,1):
        plans={}
        for i in indices:
            choices=[a for a in entries if a['profile']==i and a['sign']==sign]
            def score(a):
                r=a['result']
                return (r['passed'],r['settled'],-r['strokes'] if r['passed'] else -r['max_tail_error_deg'],
                        -r['measured_swings'],-r['max_excursion_mm'])
            plans[str(i)]=max(choices,key=score)['settings']['plans'][str(i)]
        s=dict(opening=probe,plans=plans,temperature=.6,travel_cost=1.,travel_start=38.)
        rows=evaluate([(bank,90*sign,c,'efficient',None,s) for c in floors],workers)
        distance_error=sum(max(0.,r['max_excursion_mm']-56.)+max(0.,r['max_tail_error_deg']-5.)+
                           max(0.,36.-sign*r['signed_edge_deg']) for r in rows)
        score=(sum(r['passed'] for r in rows),sum(r['settled'] for r in rows),-distance_error,
               -sum(r['strokes'] for r in rows),-sum(r['measured_swings'] for r in rows))
        combined.append(dict(sign=sign,settings=s,score=score,results=rows))
        print('probe',sign,score,flush=True)
    write(WORK/'probe-plans.json',dict(trials=entries,combined=combined))


def repair_finish(workers):
    bank=read(POLICY)['options']
    options={o['id']:o for o in bank}
    prior=read(WORK/'slick-feedback.json')+read(WORK/'probe-plans.json')['combined']
    results=[];accepted=[]
    for sign in (-1,1):
        best=max((a for a in prior if a['sign']==sign),key=lambda a:a['score'])
        chosen=dict(best['settings'])
        for r in best['results']:
            if r['passed'] or r['termination']!='completed':continue
            seq=r['sequence']; index=FIT_FLOORS.index(r['condition'])
            variants=[]
            for pos in (len(seq)-1,len(seq)-2):
                if pos<1:continue
                last=seq[pos-1]; current=seq[pos]
                err=current['residual_deg']
                failed=options[current['option']]['responses'][index]['segments'][-1]
                choices=[o for o in bank if o['responses'][index]['valid'] and o['id']!=current['option']]
                def match(o):
                    seg=o['responses'][index]['segments'][-1]
                    return (abs(err-seg['turn'])+.7*max(0.,.5*abs(err)-sign*seg['base_edge_deg'])
                            +.45*np.hypot(seg['dx']+failed['dx'],seg['dy']+failed['dy']))
                for o in sorted(choices,key=match)[:8]:
                    rule=dict(previous=last['option'],turn=last['measured_turn_deg'],roll=last['peak_sensed_roll_deg'],
                              error=err,action=o['id'])
                    variants.append(dict(chosen,late_choices=[rule]))
            tests=evaluate([(bank,90*sign,r['condition'],'efficient',None,s) for s in variants],workers)
            results.extend(tests)
            passed=[(a,s) for a,s in zip(tests,variants) if a['passed']]
            if passed:
                _,chosen=min(passed,key=lambda pair:(pair[0]['strokes'],pair[0]['measured_swings'],pair[0]['max_excursion_mm']))
        rs=evaluate([(bank,90*sign,c,'efficient',None,chosen) for c in ('slick','fresh_low')],workers)
        distance_error=sum(max(0.,r['max_excursion_mm']-56.)+max(0.,r['max_tail_error_deg']-5.)+
                           max(0.,36.-sign*r['signed_edge_deg']) for r in rs)
        score=(sum(r['passed'] for r in rs),sum(r['settled'] for r in rs),-distance_error,
               -sum(r['strokes'] for r in rs),-sum(r['measured_swings'] for r in rs))
        accepted.append(dict(sign=sign,settings=chosen,score=score,results=rs))
        print('finish',sign,score,flush=True)
    write(WORK/'finish-repair.json',dict(trials=results,combined=accepted))


def freeze():
    policy=read(POLICY);bank=policy['options']
    small=read(WORK/'small-turns.json')
    slick=read(WORK/'slick-feedback.json')+read(WORK/'probe-plans.json')['combined']
    slick+=read(WORK/'finish-repair.json')['combined']
    for o in bank:
        o.pop('selection',None);o.pop('trim_for',None)
    for key in sorted({t['key'] for t in small}):
        winner=max((t for t in small if t['key']==key),key=lambda a:a['score'])
        if winner['opening']:
            next(o for o in bank if o['id']==winner['opening']).setdefault('trim_for',[]).append(key)
    bank[0]['selection']={}
    for sign in (-1,1):
        winner=max((t for t in slick if t['sign']==sign),key=lambda a:a['score'])
        bank[0]['selection'][f'2/{sign}']=winner['settings']
    write(POLICY,policy)


def fit_slick_feedback(workers):
    policy=read(POLICY)
    bank=policy['options']
    floors=('slick','fresh_low')
    indices=[FIT_FLOORS.index(c) for c in floors]
    valid=[o for o in bank if all(o['responses'][j]['first_completed'] for j in indices)]
    trials=[]
    for sign in (-1,1):
        def information(o):
            a,b=[o['responses'][j]['segments'][0] for j in indices]
            return (abs(a['turn']-b['turn'])/2.8+abs(a['roll']-b['roll'])/2.5
                    +.14*sign*(a['turn']+b['turn'])-.08*(a['drift']+b['drift']))
        candidates=[None]
        for o in sorted(valid,key=information,reverse=True):
            if not any(p and p['recipe']==o['recipe'] and p['side']==o['side'] for p in candidates):
                candidates.append(o)
            if len(candidates)==4:break
        settings=[{}]
        for o in candidates:
            for t,weight,radius in ((.4,1.4,35.),(.6,.5,42.)):
                settings.append(dict(opening=o['id'] if o else None, temperature=t,travel_cost=weight,travel_start=radius))
        jobs=[(bank,sign*90,c,'efficient',None,s) for s in settings for c in floors]
        rows=evaluate(jobs,workers)
        group=[]
        for i,s in enumerate(settings):
            pair=rows[2*i:2*i+2]
            distance_error=sum(max(0.,r['max_excursion_mm']-56.)+
                               max(0.,r['max_tail_error_deg']-5.)+
                               max(0.,36.-sign*r['signed_edge_deg']) for r in pair)
            score=(sum(r['passed'] for r in pair),sum(r['settled'] for r in pair),-distance_error,
                   -sum(r['strokes'] for r in pair),-sum(r['measured_swings'] for r in pair))
            group.append(dict(sign=sign,settings=s,score=score,results=pair))
        winner=max(group,key=lambda a:a['score'])
        trials.extend(group)
        print('slick',sign,winner['settings'],winner['score'],flush=True)
    write(WORK/'slick-feedback.json',trials)


def train_transitions(workers):
    policy=read(POLICY)
    options=policy['options']
    by_id={o['id']:o for o in options}
    previous=read('experiments/reports/motor-training-target-edge-pivot.json')['results']
    # Start with the original failures and recheck the existing successful
    # turns too. Reserve the three new validation conditions for the final run.
    cases=[(r['target_deg'],r['condition']) for r in sorted(previous,key=lambda r:r['passed'])]
    cases.extend((g,c) for c in ('slick','inertia','fresh_low','fresh_mass') for g in (-90,90))
    history=[]
    for iteration in range(4):
        rows=evaluate([(options,g,c,'efficient',None) for g,c in cases],workers)
        history.extend(rows)
        failed=[]
        for r in rows:
            seq=r['sequence']
            if r['termination'] == 'completed' or not seq:
                continue
            prev = seq[-2] if len(seq)>1 else None
            rule=dict(delay=MODELS[r['condition']][6],previous=prev['option'] if prev else None,
                      turn=prev['measured_turn_deg'] if prev else 0.,roll=prev['peak_sensed_roll_deg'] if prev else 0.)
            rules=by_id[seq[-1]['option']].setdefault('blocked_after',[])
            if rule not in rules:rules.append(rule)
            failed.append((r['target_deg'],r['condition']))
        print('transition pass',iteration,'physical stops',len(failed),'of',len(cases),flush=True)
        if not failed:
            break
        cases=failed
    write(WORK/'transitions.json',history)
    write(POLICY,policy)


def trials(quick):
    if quick:
        return [(g,c) for c in ('nominal','slick','inertia','fresh_low') for g in (-90,90,-30,30)]
    old=read('experiments/reports/motor-training-target-edge-pivot.json')['results']
    cases=[(r['target_deg'],r['condition']) for r in old]
    cases.extend((g,c) for c in ('slick','inertia','fresh_low','fresh_mass','grip') for g in (-90,90))
    cases.extend((g,c) for c in ('validation_slick','validation_delay','validation_middle') for g in (-90,-45,-24,24,45,90))
    return cases


def qualify(workers,quick):
    opts=read(POLICY)['options']
    cases=trials(quick)
    jobs=[(opts,g,c,'efficient',None) for g,c in cases]
    results=evaluate(jobs,workers)
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,
                policy_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest(),
                waist_command_rad=0.,angle_cutoff_deg=None,conditions=MODELS,results=results)
    if not quick:
        robust=read(WORK/'robust.json')
        report['paired_slick_training']=dict(episodes=2*len(robust),
                                             best_pair=max(robust,key=lambda a:a['score']))
        report['opening_training']=read(WORK/'openings.json')
        if (WORK/'small-turns.json').exists() and (WORK/'slick-feedback.json').exists():
            report['small_turn_training']=read(WORK/'small-turns.json')
            report['slick_feedback_training']=read(WORK/'slick-feedback.json')
        if (WORK/'probe-plans.json').exists():report['probe_training']=read(WORK/'probe-plans.json')
        if (WORK/'finish-repair.json').exists():report['finish_training']=read(WORK/'finish-repair.json')
        report['transition_training']=read(WORK/'transitions.json')
        report['paired_baseline']=read(WORK/'baseline.json')+read(WORK/'paired.json')
        report['training']={}
        for stage in ('search','windup','surge','phase','evolution','local','repeats'):
            data=read(WORK/(stage+'.json'))
            measurements=[r.get('result',r) for r in data]
            report['training'][stage]=dict(episodes=len(data),settled=sum(r['settled'] for r in measurements),
                joint_stops=sum(r['termination']=='actual_joint_limit_exceeded' for r in measurements),
                best_per_condition={c:max((r for r in measurements if r['condition']==c),key=fitness)
                                    for c in sorted({r['condition'] for r in measurements})})
    write(WORK/'quick.json' if quick else REPORT,report)
    for r in results:
        print(r['condition'],r['target_deg'],r['strokes'],r['measured_swings'],round(r['torso_deg'],1),
              round(r['signed_edge_deg'],1),round(r['torso_edge_deg'],1),round(r['max_excursion_mm'],1),
              r['passed'],r['termination'])


def record_best(workers):
    report=read(REPORT)
    if hashlib.sha256(POLICY.read_bytes()).hexdigest()!=report['policy_sha256']:
        raise ValueError('Qualification and current policy differ')
    pool=[r for r in report['results'] if r['passed'] and abs(r['target_deg'])==90 and r['joint_margin_rad']>=0.]
    def rank(r):
        return (r['strokes'],r['measured_swings'],-abs(r['torso_edge_deg'])/max(1,r['measured_swings']),
                r['max_tail_error_deg'])
    clips=[]
    for group in (pool,[r for r in pool if MODELS[r['condition']][0]<=.60],
                  [r for r in pool if r['target_deg']<0 and MODELS[r['condition']][3]==2]):
        if group:
            r=min(group,key=rank)
            if r not in clips:clips.append(r)
    bank=read(POLICY)['options']
    paths=[WORK/f"featured-{i}.json" for i in range(len(clips))]
    replay=evaluate([(bank,r['target_deg'],r['condition'],'efficient',p) for r,p in zip(clips,paths)],workers)
    for reference,actual in zip(clips,replay):
        if actual!=reference:
            raise ValueError('Recorded replay differs from the frozen qualification')
    write(WORK/'featured.json',[dict(record=str(p),result=r) for p,r in zip(paths,clips)])
    print([str(p) for p in paths])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=('baseline','search','windup','phase','surge','evolve','local',
                                   'robust','fit','openings','transitions','slick-feedback','probe-plans','finish-repair','small-turns','freeze','paired','quick','qualify','replay','best'))
    p.add_argument('--workers',type=int,default=7)
    p.add_argument('--condition',choices=MODELS,default='nominal')
    p.add_argument('--goal',type=float,default=90)
    p.add_argument('--out',type=Path)
    args=p.parse_args()
    if args.mode=='best':
        record_best(args.workers)
    if args.mode in ('quick','qualify'):
        qualify(args.workers,args.mode=='quick')
    if args.mode=='replay':
        if not args.out:
            p.error('Supply --out to save the replay')
        print(run((read(POLICY)['options'],args.goal,args.condition,'efficient',args.out)))
    if args.mode in ('search','windup','surge','phase'):
        opts={'search':sample_candidates,'windup':windup_candidates,'surge':surge_candidates,'phase':phased_candidates}[args.mode]()
        jobs=[([o],170,cond,'sequence',None) for o in opts for cond in ('slick','nominal','inertia')]
        write(WORK/(args.mode+'.json'),evaluate(jobs,args.workers))
    if args.mode=='robust':
        train_slick_pair(args.workers)
    if args.mode=='local':
        local_search(args.workers)
    if args.mode=='evolve':
        evolve(args.workers)
    if args.mode=='fit':
        fit_repeats(args.workers)
    if args.mode=='openings':
        train_openings(args.workers)
    if args.mode=='finish-repair':
        repair_finish(args.workers)
    if args.mode=='probe-plans':
        train_probe_plans(args.workers)
    if args.mode=='small-turns':
        fit_small_turns(args.workers)
    if args.mode=='freeze':
        freeze()
    if args.mode=='slick-feedback':
        fit_slick_feedback(args.workers)
    if args.mode=='transitions':
        train_transitions(args.workers)
    if args.mode=='paired':
        bank=read(BANK)['options']
        jobs=[(bank,g,c,'baseline',None) for c in ('validation_slick','validation_delay','validation_middle','fresh_low','fresh_mass')
              for g in (-90,90)]
        write(WORK/'paired.json',evaluate(jobs,args.workers))
    if args.mode=='baseline':
        bank=read(BANK)['options']
        cases=[(g,c) for c in ('slick','nominal','inertia') for g in (-90,90)]
        cases.extend([(-30,'inertia'),(30,'slick')])
        write(WORK/'baseline.json', evaluate([(bank,g,c,'baseline',None) for g,c in cases],args.workers))


if __name__=='__main__':
    main()
