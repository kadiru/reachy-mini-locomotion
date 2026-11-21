"""Fit short, feedback-guided turns from continuous MuJoCo rollouts."""
import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import numpy as np

import train_rim_efficiency as rim
from train_target_pivot import read, write
from reachy_locomotion.landing_pivot import LandingPivot

WORK = Path('experiments/generated/slick-quarter')
CONFIG = Path('configs/slick-quarter-pivot.json')
REPORT = Path('experiments/reports/motor-training-slick-quarter.json')
SLICK = ('slick', 'validation_slick', 'fresh_low')
SHORT = ('validation_middle', 'unseen_inertia', 'unseen_grip', 'validation_delay')
FRESH = {
    'slick_seed_2': (*rim.MODELS['slick'][:-1], 2110),
    'slick_seed_3': (*rim.MODELS['slick'][:-1], 2111),
    'reserve_a': (.552, 4.81, .943, 1, 'torso', .0005, 2, .18, 1101),
    'reserve_b': (.578, 4.85, .95, 1, 'torso', .0004, 2, .20, 1102),
    'reserve_c': (.69, 4.98, .984, 1, 'torso', .0005, 1, .18, 1103),
    'reserve_d': (.772, 5.08, 1.018, 2, 'uniform', .0005, 1, .16, 1104),
}
FINAL_CHECK = {
    'check_slick_a': (.554,4.815,.944,1,'torso',.0005,2,.19,3125),
    'check_slick_b': (.573,4.84,.95,1,'torso',.0004,2,.17,3126),
    'check_middle': (.702,5.01,.998,1,'torso',.0004,1,.16,3127),
    'check_seed': (*rim.MODELS['slick'][:-1],3128),
}
rim.MODELS.update(FINAL_CHECK)
rim.MODELS.update(FRESH)
for extra_seed in (734,735):
    rim.MODELS[f'reserve_a_{extra_seed}'] = (*FRESH['reserve_a'][:-1],extra_seed)
for training_floor in (*SLICK, 'validation_middle'):
    for seed in (724,725):
        rim.MODELS[f'{training_floor}_{seed}'] = (*rim.MODELS[training_floor][:-1],seed)


BASE_HASH = hashlib.sha256(rim.POLICY.read_bytes()).hexdigest()
OPTIONS = read(rim.POLICY)['options']
ACTIONS = {o['id']:o for o in OPTIONS}


def variants():
    rng = np.random.default_rng(99511)
    for i in range(54):
        parent = ACTIONS[('surge-60/-1','local-slick-3-28/+1','pair-1-18/+1')[i%3]]
        recipe = dict(parent['recipe'])
        feedback = dict(parent['feedback'])
        recipe['roll_deg'] *= float(rng.uniform(.75,1.))
        recipe['pump_hz'] *= float(rng.uniform(.90,1.07))
        recipe['x_mm'] += float(rng.uniform(-6,6))
        recipe['pitch_deg'] += float(rng.uniform(-5,5))
        recipe['preload_s'] += float(rng.uniform(-.055,.055))
        feedback['trigger_deg'] *= float(rng.uniform(.60,1.22))
        feedback['prediction_s'] *= float(rng.uniform(.85,1.75))
        if 'edge_surge_mm' in feedback:
            feedback['edge_surge_mm'] *= float(rng.uniform(.70,1.03))
        yield dict(id=f'slick-stroke-{i}',side=parent['side'],recipe=recipe,feedback=feedback)


ACTIONS.update({o['id']:o for o in variants()})
for parent in ('surge-60/-1','local-slick-3-28/+1'):
    for threshold in (16,20,24,28):
        for predict in (.015,.045,.085):
            for blend in (.06,.16):
                base = ACTIONS[parent]
                name = f"catch-{parent.split('/')[0]}-{threshold}-{predict}-{blend}"
                ACTIONS[name] = dict(id=name,side=base['side'],recipe=base['recipe'],
                                     feedback=dict(base['feedback'],emergency_roll_deg=threshold,
                                                   emergency_prediction_s=predict,emergency_blend_s=blend))



def power_actions():
    rng = np.random.default_rng(79151)
    for i in range(80):
        base = ACTIONS[('surge-60/-1','local-slick-3-28/+1')[i%2]]
        g = dict(base['recipe'])
        f = dict(base['feedback'])
        g['pump_hz'] += float(rng.normal(0,.06))
        g['preload_s'] += float(rng.normal(0,.018))
        g['pitch_deg'] += float(rng.normal(0,1.5))
        g['x_mm'] += float(rng.normal(0,1.8))
        g['catch_hold_s'] *= float(rng.uniform(.82,1.2))
        f['trigger_deg'] *= float(rng.uniform(.87,1.13))
        g['yaw_deg'] *= float(rng.uniform(.82,1.22))
        yield dict(id=f'power-{i}',side=base['side'],recipe=g,feedback=f)


ACTIONS.update({o['id']:o for o in power_actions()})


for index in (45,43,16,31,37,32,18,53):
    right = ACTIONS[f'slick-stroke-{index}']
    name = f'mirror-{index}'
    ACTIONS[name] = dict(right,id=name,side=-right['side'])


for power_name in ('power-15','power-31'):
    for fraction in (.08,.18,.32,.48):
        start = ACTIONS['local-slick-3-28/+1']
        power = ACTIONS[power_name]
        name = f'finish-{power_name}-{fraction}'
        mixed = dict(id=name,side=start['side'])
        for field in ('recipe','feedback'):
            mixed[field]=dict(start[field])
            for key,value in start[field].items():
                if isinstance(value,float) and key in power[field]:
                    mixed[field][key]=value+fraction*(power[field][key]-value)
        ACTIONS[name]=mixed


def signed_actions(goal):
    side = 1 if goal < 0 else -1
    return [f'{name}/{s:+d}' for name,s in (
        ('surge-60', -side), ('local-slick-3-28',side), ('local-slick-3-4',side),
        ('local-slick-3-16',side), ('pair-1-18',side), ('phase-107',side))]


def rollout(task):
    condition, goal, names = task
    if names:
        return rim.run(([ACTIONS[n] for n in names], goal, condition, 'sequence', None))
    return rim.run((OPTIONS, goal, condition, 'efficient', None))


def batch(stage, tasks, workers):
    tasks = list(dict.fromkeys((c,g,tuple(ids)) for c,g,ids in tasks))
    file = WORK / f'{stage}.json'
    records = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for (c,g,ids), row in zip(tasks,pool.map(rollout,tasks)):
            records.append(dict(condition=c,goal=g,actions=ids,result=row))
            if len(records)%32==0:
                print(stage,len(records),'/',len(tasks),flush=True)
    write(file,records)
    print(stage,len(records),'trials;',sum(t['result']['passed'] for t in records),'qualified',flush=True)
    return records


def search(workers):
    jobs = []
    for goal in (-90,90):
        a,b,c,d,e,f = signed_actions(goal)
        # Start with the two highest-turn strokes that move the centre in opposite directions.
        sequences = [(a,b,b,b),(a,b,a,b),(a,a,b,b),(b,b,b,b),(b,a,b,b),
                     (a,b,b,a),(a,c,b,b),(a,b,d,b),(b,b,a,b),(a,a,a,a)]
        for condition in SLICK:
            jobs.extend((condition,goal,s) for s in sequences)
    bank = sorted(OPTIONS, key=lambda o: abs(o['responses'][0]['segments'][0]['turn']+24)
                  if o['responses'][0]['segments'] else 1000)
    for o in bank[:14]:
        jobs.append(('validation_middle',-24,[o['id']]))
    batch('search', jobs,workers)


def probe(workers):
    jobs = []
    for goal in (-90,90):
        a,b,c,d,e,f = signed_actions(goal)
        starts = [e, 'phase-35/-1', 'surge-71/+1'] if goal<0 else ['pair-1-8/-1']
        for start in starts:
            for condition in SLICK:
                for suffix in [(b,b,b),(a,b,b),(b,a,b),(b,b,a),(a,a,b)]:
                    jobs.append((condition,goal,(start,*suffix)))
    start = 'evo-inertia-3-12/+1'
    options = sorted(OPTIONS, key=lambda o: abs(o['responses'][0]['segments'][-1]['turn']+19)
                     if o['responses'][0]['valid'] else 1000)
    for o in options[:18]:
        jobs.append(('validation_middle',-24,(start,o['id'])))
    batch('probe',jobs,workers)


def sample(workers):
    # Measure both settled heading and rim rotation on the formerly held-out slick floor.
    jobs = [('validation_slick', 90*o['side'], [o['id']]) for o in OPTIONS]
    jobs.extend((c,-90,[n]) for n in ACTIONS if n.startswith('slick-stroke') for c in SLICK)
    batch('sample',jobs,workers)


def left(workers):
    batch('left',[(c,90,['pair-1-8/-1',n]) for n in ACTIONS if n.startswith('mirror-') for c in SLICK],workers)


def power(workers):
    batch('power',[('slick',-90,['slick-stroke-45',n]) for n in ACTIONS if n.startswith('power-')],workers)


def catches(workers):
    batch('catches',[(c,-90,[n]) for n in ACTIONS if n.startswith('catch-') for c in SLICK],workers)


def training_records(*stages):
    return [t for name in stages for t in read(WORK / f'{name}.json')]


def performance(row):
    return (row['passed'] and row['joint_margin_rad'] >= 0, row['settled'],
            -row['strokes'], row['max_tail_error_deg']<=1.5, -row['measured_swings'], np.sign(row['target_deg'])*row['torso_edge_deg'],
            -row['max_tail_error_deg'])


def measured_actions():
    # First and repeated stroke models guide search. Final scores always come
    # from connected trajectories with the original dynamics and stopping rules.
    estimates = {}
    for name in SLICK:
        estimates[name] = {}
        if name in rim.FIT_FLOORS:
            idx = rim.FIT_FLOORS.index(name)
            for o in OPTIONS:
                r = o['responses'][idx]
                if r['valid'] and r['margin'] >= 0:
                    a,b = r['segments'][0],r['segments'][-1]
                    estimates[name][o['id']] = (a['turn'],b['turn'],b['base_edge_deg'],b['drift'],b['measured_swings'])
    for t in training_records('sample','catches'):
        r = t['result']
        if r['settled'] and r['joint_margin_rad'] >= 0:
            estimates[t['condition']][t['actions'][0]] = (r['torso_deg'],r['torso_deg'],r['signed_edge_deg'],r['drift_mm'],r['measured_swings'])
    for t in training_records('power','left'):
        r=t['result']
        if r['settled'] and r['joint_margin_rad']>=0:
            b=r['segments'][-1]
            estimates[t['condition']][t['actions'][-1]]=(b['turn'],b['turn'],b['base_edge_deg'],b['drift'],b['measured_swings'])
    return estimates


def opening_pool(est):
    shared = set.intersection(*(set(est[c]) for c in SLICK))
    def strength(n):
        gains = [-est[c][n][0] for c in SLICK]
        return gains[0]+.5*min(gains)-.22*est['slick'][n][-1]
    ranked = sorted((n for n in shared if all(est[c][n][0]<-3 for c in SLICK)),
                    key=lambda n:(-strength(n),n))
    chosen = []
    for n in ranked:
        if any(max(abs(est[c][n][0]-est[c][p][0]) for c in SLICK)<2. for p in chosen):
            continue
        chosen.append(n)
        if len(chosen)==3:
            break
    return chosen


def left_starts(workers):
    jobs=[]
    for c in SLICK:
        for first in ('mirror-43','mirror-16'):
            for second in ('mirror-43','mirror-16','mirror-53','phase-107/-1','local-slick-3-4/-1'):
                for trim in ('phase-107/-1','mirror-53'):
                    # The final trim is repeated until feedback reaches the
                    # deadband or the nine-stroke trial budget is exhausted.
                    jobs.append((c,90,[first, second, first, second, first, trim, trim, trim, trim]))
    batch('left_starts',jobs,workers)


def four(workers):
    records = training_records('power')
    safe = [t for t in records if t['result']['settled'] and t['result']['joint_margin_rad']>=0]
    ranked = sorted(safe,key=lambda t:t['result']['segments'][-1]['turn'])
    names = [t['actions'][-1] for t in ranked[:5]]
    finish = names[:3]+['surge-60/-1','local-slick-3-28/+1','phase-107/+1','pair-0-12/+1']
    paths = [('slick',-90,('slick-stroke-45',first,second,last))
             for first in names[:2] for second in names[:3] for last in finish]
    batch('four',paths,workers)


def left_quick(workers):
    a,b,c,d,_,f = signed_actions(90)
    jobs=[]
    for first in ('mirror-16','mirror-43'):
        for middle in ((b,b,b),(a,b,b),(c,b,b),(b,a,b),(b,c,b),(b,b,a)):
            for last in (b,a,c,f):
                jobs.append(('slick',90,(first,*middle,last)))
    batch('left_quick',jobs,workers)


NOISE_FLOORS = ('slick', 'slick_724', 'slick_725')


def stable_third(workers):
    doubles = training_records('power')
    candidates = sorted((t for t in doubles if t['result']['settled'] and t['result']['joint_margin_rad']>.01),
                        key=lambda t:t['result']['segments'][-1]['turn'])
    actions = [t['actions'][-1] for t in candidates[:16]]
    actions += signed_actions(-90)[:5]+['slick-stroke-45','catch-surge-60-28-0.085-0.06']
    jobs = [(c,-90,('slick-stroke-45','power-15',a)) for a in actions for c in NOISE_FLOORS]
    batch('stable_third',jobs,workers)


def stable_four(workers):
    triples = training_records('stable_third')
    groups = {}
    for t in triples:
        groups.setdefault(tuple(t['actions']),[]).append(t['result'])
    eligible = [(p,rs) for p,rs in groups.items() if all(r['settled'] and r['joint_margin_rad']>=0 for r in rs)]
    def reach(pair):
        return max(abs(-90-r['torso_deg']) for r in pair[1])+.5*np.ptp([r['torso_deg'] for r in pair[1]])
    model = measured_actions()['slick']
    jobs=[]
    for prefix,rows in sorted(eligible,key=reach)[:5]:
        remainder=np.mean([-90-r['torso_deg'] for r in rows])
        candidates=sorted(model,key=lambda n:(abs(remainder-model[n][1])+.12*model[n][-1],n))
        actions=[]
        for n in [prefix[-1], 'power-15', 'local-slick-3-28/+1', *candidates]:
            if n not in actions and not any(abs(model[n][1]-model[k][1])<.1 for k in actions):
                actions.append(n)
            if len(actions)>=8:
                break
        for n in actions:
            jobs.extend((c,-90,(*prefix,n)) for c in NOISE_FLOORS)
    batch('stable_four',jobs,workers)


def precise_four(workers):
    data = training_records('stable_four')
    per_path = {}
    for t in data:
        per_path.setdefault(tuple(t['actions']),[]).append(t['result'])
    prefixes = sorted({p[:3] for p,rs in per_path.items() if all(runnable(r) for r in rs)})
    jobs=[(c,-90,(*prefix,n)) for prefix in prefixes for n in ACTIONS if n.startswith('finish-power')
          for c in NOISE_FLOORS]
    batch('precise_four',jobs,workers)


def stable_trim(workers):
    rows = training_records('stable_four')
    full = {}
    for t in rows:
        full.setdefault(tuple(t['actions']), {})[t['condition']] = t['result']
    m = measured_actions()['slick']
    jobs=[]
    safe = [(p,rs) for p,rs in full.items() if runnable(rs['slick']) and
            all(r['settled'] and r['joint_margin_rad']>=0 for r in rs.values())]
    if not any(all(runnable(r) for r in rs.values()) for _,rs in safe):
        def rank(item):
            rs=item[1].values()
            return (-sum(runnable(r) for r in rs),max(r['max_tail_error_deg'] for r in rs))
        for prefix,rs in sorted(safe,key=rank)[:7]:
            for c,r in rs.items():
                remaining = -90.-r['base_deg']
                if runnable(r) or abs(remaining)<=4.1:
                    continue
                options = sorted(m,key=lambda n:(abs(remaining-m[n][1])+.12*m[n][-1],n))
                jobs.extend((c,-90,(*prefix,n)) for n in options[:4])
    batch('stable_trim',jobs,workers)


def shared_noise_paths():
    records = training_records('stable_four','precise_four','stable_trim')
    by_prefix = {}
    for t in records:
        row=t['result']
        prefix=tuple(t['actions'][:4])
        by_prefix.setdefault(prefix,{}).setdefault(t['condition'],[]).append(row)
    ranked=[]
    for prefix,per_floor in by_prefix.items():
        if set(per_floor)!=set(NOISE_FLOORS):
            continue
        chosen = {c:max((r for r in rows if runnable(r)),key=performance)
                  for c,rows in per_floor.items() if any(runnable(r) for r in rows)}
        if len(chosen)==len(NOISE_FLOORS):
            rows = list(chosen.values())
            rank = (max(r['strokes'] for r in rows),sum(r['strokes'] for r in rows),
                    max(r['max_tail_error_deg'] for r in rows)>2,
                    sum(r['measured_swings'] for r in rows),max(r['max_tail_error_deg'] for r in rows))
            ranked.append((rank,prefix,chosen))
    return min(ranked,key=lambda v:(v[0],v[1]))[2] if ranked else {}


def finish_four(workers):
    models = measured_actions()['slick']
    prefixes = {}
    for entry in training_records('four'):
        row=entry['result']
        if len(row['sequence'])==4 and row['settled'] and row['joint_margin_rad']>=0:
            last=row['sequence'][3]
            prefixes[tuple(entry['actions'][:3])]=last['residual_deg']
    jobs=[]
    for prefix,error in prefixes.items():
        ranked = sorted(models, key=lambda n:(abs(error-models[n][1])+.14*models[n][-1],n))
        # Nearby predictions may be identical projections of the same motion.
        # Diversify the finishes by their measured angular response.
        turns=[]
        for n in ranked:
            turn=models[n][1]
            if abs(error-turn)>10 or any(abs(turn-v)<.22 for v in turns):
                continue
            turns.append(turn)
            jobs.append(('slick',-90,(*prefix,n)))
            if len(turns)>=7:
                break
    batch('finish_four',jobs,workers)


def projected(workers):
    est = measured_actions()
    right = opening_pool(est)
    print('strong shared openings',right,flush=True)
    jobs = []
    rng = np.random.default_rng(28561)
    for c in SLICK:
        for sign,starts in ((-1,right),(1,['pair-1-8/-1'])):
            m = est[c]
            for first in starts:
                if first not in m:
                    continue
                def merit(n):
                    _,gain,edge,drift,count = m[n]
                    return sign*gain + .7*min(sign*gain,sign*edge) - .45*drift - .26*count
                options = sorted((n for n in m if sign*m[n][1]>3),key=lambda n:(-merit(n),n))
                top = options[:8]
                # Fit a small bank of short paths with complementary high-gain strokes.
                # Include repeated actions and a lower-gain trim to close the heading.
                paths = set()
                for j in range(360):
                    path = [first]
                    remaining = 90-sign*m[first][0]
                    while remaining>4 and len(path)<9:
                        if remaining < 24:
                            pool = sorted(options,key=lambda n:abs(remaining-sign*m[n][1])-.16*merit(n))[:3]
                        else:
                            pool = top[:4] if j%2 else top
                        action = pool[int(rng.integers(len(pool)))] if j else pool[0]
                        path.append(action)
                        remaining -= sign*m[action][1]
                    paths.add(tuple(path))
                def priority(path):
                    pred = m[first][0]+sum(m[n][1] for n in path[1:])
                    edge = sum(sign*m[n][2] for n in path)
                    return (3.0*abs(90-sign*pred)+2*max(0,40-edge)+len(path)*5+
                            .14*sum(m[n][-1] for n in path)+.1*sum(m[n][3] for n in path))
                seen = set()
                for path in sorted(paths,key=lambda x:(priority(x),x)):
                    # Diversify the crucial second stroke instead of testing many
                    # near-identical finishes after the same unqualified launch.
                    prefix = path[:3]
                    if prefix in seen:
                        continue
                    jobs.append((c,sign*90,path))
                    seen.add(prefix)
                    if len(seen)>=11:
                        break
    batch('projected',jobs,workers)


def runnable(row):
    if not row['passed'] or row['joint_margin_rad'] < 0:
        return False
    end = row['sequence'][-1]
    return row['strokes']==9 or abs(end['residual_deg']-end['measured_turn_deg'])<=4.1


def complete(workers, goal):
    estimates = measured_actions()
    rows = training_records('projected','probe','search','four','finish_four')
    if goal>0:
        rows += training_records('left_starts')
    starts_right = opening_pool(estimates)
    four_stroke_starts = {t['actions'][0] for t in rows
                          if t['condition']=='slick' and t['goal']==-90 and
                          t['result']['strokes']<=4 and runnable(t['result'])}
    starts_right = [n for n in starts_right if n in four_stroke_starts] or starts_right
    jobs = []
    starts = starts_right if goal<0 else ['pair-1-8/-1','mirror-43','mirror-16']
    for c in SLICK:
        m = estimates[c]
        for first in starts:
            group = [t['result'] for t in rows if t['condition']==c and t['goal']==goal and
                     t['actions'][0]==first]
            passed = [r for r in group if runnable(r)]
            budget = min([r['strokes'] for r in passed]+[10])
            if passed and (budget<=4 or (c,goal)!=('slick',-90)):
                continue
            # Replay from startup for every suffix substitution. Never reset
            # pose, motor delay buffers, noise stream, or contact state mid-turn.
            prefixes = {}
            for r in group:
                seq = r['sequence']
                for j in range(1,min(len(seq)+1,budget,9)):
                    last=seq[j-1]
                    if 'measured_turn_deg' not in last:
                        continue
                    residual=last['residual_deg']-last['measured_turn_deg']
                    if abs(residual)<=4.1 or abs(residual)>45:
                        continue
                    names=tuple(x['option'] for x in seq[:j])
                    prefixes[names]=residual
            options=[]
            for prefix,remaining in prefixes.items():
                dirs=np.sign(remaining)
                for n,v in m.items():
                    gain=v[1]
                    if dirs*gain<1:
                        continue
                    delta=abs(remaining-gain)
                    if delta>12:
                        continue
                    reward=delta + 1.2*len(prefix) + .16*v[3] + .12*v[4] - .20*dirs*v[2]
                    if len(prefix)==3 and c=='slick' and goal==-90:
                        reward-=5
                    options.append((reward,prefix,n))
            unique=set()
            for score,prefix,n in sorted(options):
                if (prefix,n) in unique:
                    continue
                unique.add((prefix,n))
                jobs.append((c,goal,(*prefix,n)))
                if len(unique)>=18:
                    break
    batch('complete_right' if goal<0 else 'complete_left',jobs,workers)


def complete_right(workers):
    complete(workers,-90)


def complete_left(workers):
    complete(workers,90)


def guide_trial(task):
    config,goal,c,path = task
    if config['base_sha256'] != BASE_HASH:
        raise ValueError('Frozen stroke bank differs from the trained base policy')
    model = partial(LandingPivot, guidance=config)
    return rim.run((OPTIONS,goal,c,'efficient',path),policy_class=model)


def guide_tests(config,cases,workers):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(guide_trial,[(config,g,c,None) for g,c in cases]))


def paths_from(rows,goal,delay,opening):
    items = []
    for row in rows:
        seq = row['sequence']
        if seq[0]['option'] != opening or not row['passed'] or row['joint_margin_rad']<0:
            continue
        items.append([dict(action=s['option'],error=s['residual_deg'],turn=s['measured_turn_deg'],
                           roll=s['peak_sensed_roll_deg']) for s in seq])
    return dict(delay=delay,target_deg=goal,target_tolerance_deg=3.,opening=opening,paths=items)


def config_for(groups):
    ids = {s['action'] for g in groups for path in g['paths'] for s in path}
    ids.update(g['opening'] for g in groups)
    extras = [ACTIONS[n] for n in sorted(ids) if n not in {o['id'] for o in OPTIONS}]
    return dict(base_policy=str(rim.POLICY),base_sha256=BASE_HASH,
                extra_actions=extras,groups=groups)


def practice(workers, left_only):
    records = training_records('search','probe','projected','four','finish_four','complete_right')
    if left_only:
        records += training_records('left_starts','complete_left','left_quick')
    rows = [t['result'] for t in records] + read(rim.REPORT)['results']
    jobs=[]
    for c in (*SLICK,'validation_middle'):
        for g in ((-24,) if c=='validation_middle' else (-90,90)):
            if (g>0)!=left_only:
                continue
            subset = [r for r in rows if r['target_deg']==g and r['condition']==c and runnable(r)]
            choices = sorted(subset,key=performance,reverse=True)
            by_first = {}
            for r in choices:
                seq=tuple(s['option'] for s in r['sequence'])
                bucket=by_first.setdefault(seq[0],[])
                if len(bucket)>=2 or seq in bucket:
                    continue
                bucket.append(seq)
                for seed in (724,725):
                    jobs.append((f'{c}_{seed}',g,seq))
    batch('practice_left' if left_only else 'practice_right',jobs,workers)


def practice_right(workers):
    practice(workers,False)


def practice_left(workers):
    practice(workers,True)


def left_repeats(workers):
    entries=training_records('left_quick','left_starts','complete_left','projected')
    jobs=[]
    for c in ('slick','validation_slick'):
        for start in ('mirror-16','mirror-43'):
            seen=set()
            good=sorted((t for t in entries if t['condition']==c and t['goal']==90 and
                         t['actions'][0]==start and runnable(t['result'])),
                        key=lambda t:performance(t['result']),reverse=True)
            for t in good:
                seq=tuple(s['option'] for s in t['result']['sequence'])
                if seq in seen:
                    continue
                seen.add(seq)
                jobs.extend((f'{c}_{seed}',90,seq) for seed in (724,725))
                if len(seen)>=8:
                    break
    batch('left_repeats',jobs,workers)


def shared_left(rows,opening):
    chosen={}
    for floor in SLICK:
        floors={floor,f'{floor}_724',f'{floor}_725'}
        by_sequence={}
        for r in rows:
            if r['target_deg']!=90 or r['condition'] not in floors or not runnable(r):
                continue
            seq=tuple(s['option'] for s in r['sequence'])
            if seq[0]==opening:
                by_sequence.setdefault(seq,{})[r['condition']]=r
        full=[rs for rs in by_sequence.values() if set(rs)==floors]
        if full:
            best=min(full,key=lambda rs:(max(r['strokes'] for r in rs.values()),
                                         max(r['max_tail_error_deg'] for r in rs.values()),
                                         sum(r['measured_swings'] for r in rs.values())))
            chosen.update(best)
    return chosen


def fit(workers):
    records = training_records('search','probe','projected','complete_right','complete_left','four','finish_four','stable_four','precise_four','stable_trim','left_starts','left_quick','practice_right','practice_left','left_repeats')
    current = read(rim.REPORT)['results']
    allrows = [t['result'] for t in records] + current
    groups = []
    diagnostics = []
    for goal, delay, floors, openings in [
            (-90,2,SLICK,opening_pool(measured_actions())),
            (90,2,SLICK,['pair-1-8/-1','mirror-43','mirror-16']),
            (-24,1,SHORT,['evo-inertia-3-12/+1'])]:
        candidates=[]
        for opening in openings:
            chosen=[]
            repeated = [f'{c}_{seed}' for c in floors for seed in (724,725) if f'{c}_{seed}' in rim.MODELS]
            shared = shared_noise_paths() if goal==-90 and opening=='slick-stroke-45' else {}
            if goal==90:
                shared = shared_left(allrows,opening)
            for c in (*floors,*repeated):
                if c in shared:
                    chosen.append(shared[c])
                    continue
                eligible=[r for r in allrows if r['target_deg']==goal and r['condition']==c and
                          runnable(r) and r['sequence'][0]['option']==opening]
                if eligible:
                    chosen.append(max(eligible,key=performance))
            group = paths_from(chosen,goal,delay,opening)
            conf = config_for([group])
            tests = guide_tests(conf,[(goal,c) for c in (*floors,*repeated)],workers)
            matched = tests[:len(floors)]
            score = (sum(r['passed'] and r['joint_margin_rad']>=0 for r in matched),
                     -sum(r['fell'] or r['termination']!='completed' for r in tests),
                     -tests[0]['strokes'], sum(runnable(r) for r in tests), -sum(r['strokes'] for r in tests),
                     -sum(r['measured_swings'] for r in tests))
            candidates.append((score,group))
            diagnostics.append(dict(goal=goal,opening=opening,score=score,results=tests))
            print('fit',goal,opening,score,[(r['condition'],r['passed'],r['strokes'],round(r['torso_deg'],1)) for r in tests],flush=True)
        groups.append(max(candidates,key=lambda x:x[0])[1])
    write(CONFIG, config_for(groups))
    write(WORK/'fit-diagnostics.json',diagnostics)


def repair_search(workers):
    jobs=[]
    left_start='mirror-43'
    for repeat in ('mirror-43','mirror-16','mirror-53','mirror-32','phase-107/-1'):
        for alternate in ('mirror-43','mirror-16','mirror-53','phase-107/-1','local-slick-3-4/-1'):
            # Test a lower-gain second action before admitting another large turn.
            seq=(left_start,repeat,alternate,repeat,alternate,repeat,alternate,repeat,'phase-107/-1')
            jobs.append(('reserve_a',90,seq))
    for c,start,idx,error in [('reserve_a','phase-107/+1',1,-14.),('reserve_c','evo-inertia-3-12/+1',0,-21.)]:
        choices=[]
        for o in OPTIONS:
            resp=o['responses'][idx]
            if not resp['valid'] or resp['margin']<0:
                continue
            seg=resp['segments'][-1]
            cost=abs(error-seg['turn'])+.22*seg['base_edge_deg']+.10*seg['drift']
            choices.append((cost,o['id']))
        for n in dict.fromkeys([n for _,n in sorted(choices)[:24]]+['surge-77/-1','slick-stroke-45','slick-stroke-16']):
            jobs.append((c,-24,(start,n)))
    batch('repair_search',jobs,workers)


def repair_radius(workers):
    from copy import deepcopy
    conf=read(WORK/'first-policy.json')
    tasks=[]
    for radius in (.10,.14,.17,.20,.25,.40,.75,1.):
        cfg=deepcopy(conf)
        next(g for g in cfg['groups'] if g['target_deg']==90)['match_radius']=radius
        tasks.append((cfg,90,'reserve_a',None))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows=list(pool.map(guide_trial,tasks))
    write(WORK/'repair_radius.json',rows)
    print([(r['passed'],r['termination'],round(r['torso_deg'],1)) for r in rows],flush=True)


def final_cases():
    return [(g,c) for c in FINAL_CHECK for g in ((-90,) if c=='check_seed' else (-90,-24,90))]


def repair_finish(workers):
    rows=training_records('repair_search')
    m=measured_actions()['validation_slick']
    jobs=[]
    for c,g in [('reserve_a',90),('reserve_a',-24),('reserve_c',-24)]:
        subset=[t for t in rows if t['condition']==c and t['goal']==g]
        if any(runnable(t['result']) for t in subset):
            continue
        choices=[]
        for t in subset:
            r=t['result']
            for j,stroke in enumerate(r['sequence']):
                if 'measured_turn_deg' not in stroke or j>=8:
                    continue
                error=stroke['residual_deg']-stroke['measured_turn_deg']
                if not 4.1<abs(error)<30:
                    continue
                prefix=tuple(k['option'] for k in r['sequence'][:j+1])
                for n,seg in m.items():
                    shortfall=abs(error-seg[1])
                    if shortfall<8:
                        choices.append((shortfall+.12*seg[-1]+.2*len(prefix),prefix,n))
        tried=set()
        for _,prefix,n in sorted(choices):
            path=(*prefix,n)
            if path not in tried:
                tried.add(path)
                jobs.append((c,g,path))
            if len(tried)>=24:
                break
    batch('repair_finish',jobs,workers)


def repair_left(workers):
    initial=read(WORK/'first-qualification.json')
    old=next(r for r in initial['old_held_out'] if r['condition']=='reserve_a' and r['target_deg']==90)
    legacy=[s['option'] for s in old['sequence']]
    legacy[0]='mirror-43'
    trials=[t['result'] for t in training_records('repair_search') if t['goal']==90]
    def deficit(r):
        return (r['max_tail_error_deg']+max(0.,r['max_excursion_mm']-56.)
                +max(0.,36.-r['signed_edge_deg'])+max(0.,36.-r['torso_edge_deg']))
    least=sorted((r for r in trials if r['settled']),key=deficit)[:3]
    paths=[legacy]
    for r in least:
        base=[s['option'] for s in r['sequence']]
        for last in ('pair-1-8/-1','phase-35/+1','mirror-53','mirror-43','phase-68/+1','mirror-32','mirror-31'):
            paths.append(base[:-1]+[last])
        paths.extend((base[:-2]+legacy[-2:],base[:-3]+legacy[-3:]))
    batch('repair_left',[('reserve_a',90,p) for p in paths],workers)


def repair_replicas(workers):
    rows=[t['result'] for t in training_records('repair_search','repair_finish','repair_left')]
    options=sorted((r for r in rows if r['target_deg']==90 and runnable(r)),key=performance,reverse=True)
    seen=[]
    for r in options:
        seq=tuple(s['option'] for s in r['sequence'])
        if seq not in seen:
            seen.append(seq)
        if len(seen)>=6:
            break
    batch('repair_replicas',[(f'reserve_a_{k}',90,p) for p in seen for k in (734,735)],workers)


def repair_fit(workers):
    conf=read(WORK/'first-policy.json')
    prior=read(WORK/'first-qualification.json')
    development=[t['result'] for t in training_records('repair_search','repair_finish','repair_left')]
    replica_trials=training_records('repair_replicas')
    replica_rows=[t['result'] for t in replica_trials]
    # Right quarter-turn guidance remains frozen. Add physically qualified
    # continuations for the newly observed weak and ambiguous landings.
    for c,g,delay,opening in [('reserve_a',90,2,'mirror-43'),
                               ('reserve_a',-24,2,'phase-107/+1'),
                               ('reserve_c',-24,1,'evo-inertia-3-12/+1')]:
        choices=[r for r in development if r['condition']==c and r['target_deg']==g and runnable(r)]
        if not choices:
            continue
        if g==90:
            def replication_score(r):
                seq=[s['option'] for s in r['sequence']]
                replicas=[trial['result'] for trial in replica_trials if trial['actions']==seq]
                return (sum(runnable(a) for a in replicas),-sum(a['fell'] for a in replicas),performance(r))
            best=max(choices,key=replication_score)
        else:
            best=max(choices,key=performance)
        group=next((v for v in conf['groups'] if v['target_deg']==g and v['delay']==delay),None)
        if group is None:
            compatible=[r for r in prior['matched'] if r['target_deg']==g and
                        rim.MODELS[r['condition']][-3]==delay and r['sequence'][0]['option']==opening and runnable(r)]
            group=paths_from(compatible,g,delay,opening)
            conf['groups'].append(group)
        extra = [best]
        if g==90:
            for seed in (734,735):
                valid=[r for r in replica_rows if r['condition']==f'reserve_a_{seed}' and runnable(r)]
                if valid:
                    extra.append(max(valid,key=performance))
        group['paths']+=paths_from(extra,g,delay,opening)['paths']
    for group in conf['groups']:
        g=group['target_deg']
        if g==-90:
            continue
        retained=[r for r in prior['held_out'] if r['target_deg']==g and runnable(r) and
                  rim.MODELS[r['condition']][-3]==group['delay'] and r['sequence'][0]['option']==group['opening']]
        group['paths']+=paths_from(retained,g,group['delay'],group['opening'])['paths']
    conf=config_for(conf['groups'])
    # Test narrower observation support against previous landings, the new
    # fall, and unseen-to-the-initial-fit noise replicas on the same new floor.
    cfgs=[]
    from copy import deepcopy
    scenarios=[(90,c) for c in (*SLICK,*(f'{c}_{seed}' for c in SLICK for seed in (724,725)),
                                 'reserve_a','reserve_a_734','reserve_a_735','reserve_b')]
    scenarios+=[(-24,c) for c in ('reserve_a','reserve_c','unseen_low','validation_middle')]
    for radius in (1.,.35,.18):
        candidate=deepcopy(conf)
        next(g for g in candidate['groups'] if g['target_deg']==90)['match_radius']=radius
        rs=guide_tests(candidate,scenarios,workers)
        matched=[r for r in rs if r['condition'] in (*SLICK,'reserve_a','reserve_b','reserve_c','validation_middle','unseen_low')]
        score=(sum(runnable(r) for r in matched),-sum(r['fell'] or r['termination']!='completed' for r in rs),
               sum(runnable(r) for r in rs),-sum(r['strokes'] for r in rs))
        print('repair radius',radius,score,[(r['condition'],r['target_deg'],r['termination']) for r in rs if not r['passed']],flush=True)
        cfgs.append((score,candidate,rs))
    _,winner,_=max(cfgs,key=lambda x:x[0])
    write(CONFIG,winner)
    write(WORK/'repair-fit.json',[dict(score=s,results=rs,radius=next(g for g in cfg['groups'] if g['target_deg']==90)['match_radius'])
                                 for s,cfg,rs in cfgs])


def repair_qualify(workers):
    config=read(CONFIG)
    previous=read(WORK/'first-qualification.json')
    original,exposed,fresh=rim.trials(False),reserved_cases(),final_cases()
    rows=guide_tests(config,[*original,*exposed,*fresh],workers)
    n,m=len(original),len(exposed)
    write(REPORT,dict(evidence=previous['evidence'],policy_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
                      base_sha256=BASE_HASH,motor_model_sha256=previous['motor_model_sha256'],
                      training=training_summary(),repair_search={stage:len(training_records(stage))
                      for stage in ('repair_search','repair_finish','repair_left','repair_replicas')},fit_results=previous['fit_results'],
                      initial_policy_sha256=previous['policy_sha256'],first_held_out=previous['held_out'],
                      original_held_out_baseline=previous['old_held_out'],repair_fit=read(WORK/'repair-fit.json'),
                      conditions=rim.MODELS,matched=rows[:n],exposed_retest=rows[n:n+m],held_out=rows[n+m:],
                      old_held_out=[t['result'] for t in training_records('paired_final')]))
    for split in (rows[:n],rows[n:n+m],rows[n+m:]):
        print(sum(r['passed'] for r in split),'/',len(split),'falls',sum(r['fell'] for r in split),
              'joint stops',sum(r['termination']=='actual_joint_limit_exceeded' for r in split),flush=True)
        print([(r['condition'],r['target_deg'],r['termination']) for r in split if not r['passed']],flush=True)


def paired_final(workers):
    batch('paired_final',[(c,g,()) for g,c in final_cases()],workers)


def reserved_cases():
    return [(g,c) for c in FRESH for g in ((-90,) if c.startswith('slick_seed') else (-90,-24,90))]


def paired(workers):
    batch('paired',[(c,g,()) for g,c in reserved_cases()],workers)


def training_summary():
    summary = {}
    for stage in ('search','probe','sample','catches','power','left','left_starts','left_quick',
                  'four','finish_four','stable_third','stable_four','precise_four','stable_trim','projected','complete_right','complete_left','practice_right','practice_left','left_repeats'):
        rows = [t['result'] for t in training_records(stage)]
        summary[stage] = dict(episodes=len(rows),settled=sum(r['settled'] for r in rows),
                              falls=sum(r['fell'] for r in rows),
                              joint_stops=sum(r['termination']=='actual_joint_limit_exceeded' for r in rows))
    return summary


def qualification(workers):
    config = read(CONFIG)
    cases = rim.trials(False)
    fresh = reserved_cases()
    rows = guide_tests(config,cases+fresh,workers)
    write(REPORT,dict(evidence='uncalibrated MuJoCo simulation',policy_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
                      base_sha256=config['base_sha256'],motor_model_sha256=hashlib.sha256(Path('configs/motors-speed-trained.json').read_bytes()).hexdigest(),
                      training=training_summary(),fit_results=read(WORK/'fit-diagnostics.json'),conditions=rim.MODELS,matched=rows[:len(cases)],
                      held_out=rows[len(cases):],old_held_out=[t['result'] for t in training_records('paired')]))
    (WORK/'first-qualification.json').write_bytes(REPORT.read_bytes())
    (WORK/'first-policy.json').write_bytes(CONFIG.read_bytes())
    for block in [rows[:len(cases)], rows[len(cases):]]:
        print('passed',sum(r['passed'] for r in block),'of',len(block),'falls',sum(r['fell'] for r in block),
              'joint stops',sum(r['termination']=='actual_joint_limit_exceeded' for r in block),flush=True)
        for r in block:
            if not r['passed']:
                print(r['condition'],r['target_deg'],r['termination'],round(r['max_tail_error_deg'],1),
                      round(r['max_excursion_mm'],1),round(r['signed_edge_deg'],1),flush=True)


def film(workers):
    config,report = read(CONFIG),read(REPORT)
    if hashlib.sha256(CONFIG.read_bytes()).hexdigest()!=report['policy_sha256']:
        raise ValueError('Policy changed since qualification')
    original = next(r for r in report['matched'] if r['target_deg']==-90 and r['condition']=='slick')
    if not original['passed'] or original['joint_margin_rad']<0:
        raise ValueError('Matched slick-floor right turn failed')
    replicas = report.get('exposed_retest',[]) + report['held_out']
    pool = [original] + [r for r in replicas if (r['condition'].startswith('slick_seed') or r['condition']=='check_seed')
                         and r['target_deg']==-90 and r['passed'] and r['joint_margin_rad']>=0]
    best = min(pool,key=lambda r:(r['strokes'],r['max_tail_error_deg'],r['measured_swings']))
    clips = [best] if best==original else [best,original]
    for i,r in enumerate(clips):
        path=WORK/f'film-{i}.json'
        replay = guide_trial((config,-90,r['condition'],path))
        if replay!=r:
            raise ValueError('Film trajectory differs from qualification')
        capture=read(path)
        capture['title']='Matched slick-floor turn' if r==original else 'Slick floor with a new noise seed'
        write(path,capture)
        print(path,r['condition'],r['torso_deg'],flush=True)
    write(WORK/'film-index.json',[dict(record=str(WORK/f'film-{i}.json'),result=r) for i,r in enumerate(clips)])



def baseline(workers):
    batch('baseline', [(c,g,()) for c,g in [('slick',-90),('validation_slick',-90),
                                             ('validation_slick',90),('validation_middle',-24)]],workers)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    stages = {stage.__name__:stage for stage in (
        baseline,search,probe,sample,catches,power,left,left_starts,left_quick,
        four,finish_four,stable_third,stable_four,precise_four,stable_trim,projected,
        complete_right,complete_left,practice_right,practice_left,left_repeats,fit,paired,qualification,film,repair_search,repair_radius,repair_finish,repair_left,repair_replicas,repair_fit,paired_final,repair_qualify,
    )}
    p.add_argument('stage',choices=[*stages,'replay'])
    p.add_argument('--workers',type=int,default=6)
    p.add_argument('--condition', choices=sorted(rim.MODELS), default='slick')
    p.add_argument('--goal', type=float, default=-90.)
    p.add_argument('--out', type=Path, default=WORK/'replay.json')
    args=p.parse_args()
    if args.stage=='replay':
        r=guide_trial((read(CONFIG),args.goal,args.condition,args.out))
        print(r['condition'],r['target_deg'],r['torso_deg'],r['strokes'],r['passed'],args.out)
    else:
        stages[args.stage](args.workers)
