"""Train bounded compound head strokes on projected millimetres per swing."""
import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from reachy_locomotion.heading import HeadingController, HeadingFeedback, HeadingSensor
from reachy_locomotion.primitives import Primitive, PrimitiveSimulator
from reachy_locomotion.sway import Gait
from reachy_locomotion.swing_distance import stroke_count
from train_rocking import decode, read, vector
from finalize_primitives import record_terminal


class SwingSequence:
    """Whole left/right pairs followed by a physical return to neutral."""
    catch_trigger = yaw_trigger = None

    def __init__(self, policy, side, pairs, sensor):
        self.gait = Gait(**dict(policy['gait'], mirror=side))
        self.period = round(50 / self.gait.frequency_hz)
        self.stop_frame = pairs * self.period
        self.feedback = HeadingController(HeadingFeedback(**policy['feedback']),
                                         HeadingSensor(**sensor), 0., self.period)
        self.last_pose_gait = self.gait

    def observe_heading(self, t, degrees):
        self.feedback.update(math.radians(degrees))

    def target(self, t, roll):
        frame = round(t * 50)
        phase = 2 * math.pi * min(frame, self.stop_frame) / self.period
        if frame <= self.stop_frame:
            self.last_pose_gait = self.feedback.steer(self.gait)
            ramp = min(1., frame / (self.period * self.feedback.config.startup_ramp_cycles))
            stage = 'compound sway'
        else:
            ramp = max(0., 1 - (frame - self.stop_frame) / 20)
            stage = 'return to neutral' if ramp else 'settle'
        return self.last_pose_gait.pose(phase, ramp*ramp*(3-2*ramp)), stage, roll


SIMULATORS = {}


def trial(job):
    name, policy, config, side, pairs, sensor, recording = job
    key = json.dumps(config, sort_keys=True)
    if key not in SIMULATORS:
        SIMULATORS[key] = PrimitiveSimulator('experiments/upstream-reachy-mini', config)
    sim = SIMULATORS[key]
    ctl = SwingSequence(policy, side, pairs, sensor)
    r = sim.run_primitive(Primitive(), seconds=(ctl.stop_frame + 80)/50,
                          waist_mode='hold_neutral', record=True, controller=ctl, tilt_limit_deg=None)
    rows = r['trajectory']
    yaw0 = rows[0]['foot_rpy_rad'][2]
    dx, dy = r['planar_displacement_m']
    distance = 1000 * (dx*math.cos(yaw0) + dy*math.sin(yaw0))
    lateral = 1000 * (-dx*math.sin(yaw0) + dy*math.cos(yaw0))
    head = np.degrees([row['head_rpy_rad'] for row in rows])
    counts = {}
    for threshold in (6, 10, 15):
        roll = stroke_count(head[:,0], threshold)
        yaw = stroke_count(head[:,2], threshold)
        counts[str(threshold)] = dict(roll=roll, yaw=yaw, charged=max(2*pairs, roll, yaw))
    headings = [math.degrees(row['heading_from_startup_rad']) for row in rows]
    ends = list(range(ctl.period, min(len(rows), ctl.stop_frame+1), ctl.period))
    pair_headings = [headings[i] for i in ends]
    turn_spans = [max(headings[i-ctl.period:i+1])-min(headings[i-ctl.period:i+1]) for i in ends]
    end_error = max(map(abs, pair_headings), default=0.)
    mm_per_swing = distance/counts['10']['charged']
    passed = (r['completed'] and r['settled'] and len(ends)==pairs and distance>0
              and end_error<=20 and abs(headings[-1])<=20)
    summary = dict(name=name, side=side, pairs=pairs, completed_pairs=len(ends), dt=config['timestep_s'],
                   passed=passed, physically_completed=r['completed'], settled=r['settled'], fell=r['fell'],
                   termination=r['termination'], distance_mm=distance, lateral_mm=lateral,
                   mm_per_swing=mm_per_swing, counts=counts, mean_pair_mm=distance/pairs,
                   pair_end_headings_deg=pair_headings, max_pair_end_deg=end_error,
                   mean_intrapair_yaw_span_deg=float(np.mean(turn_spans)) if turn_spans else 0.,
                   final_heading_deg=headings[-1], peak_tilt_deg=r['peak_tilt_deg'],
                   edge_support_s=r['edge_support_seconds'], rim_height_difference_mm=r['peak_side_height_difference_mm'],
                   head_peak_torque_nm=max(r['peak_motor_torque_nm'][sim.model.actuator('stewart_'+str(i)).id] for i in range(1,7)),
                   min_joint_margin_rad=r['min_joint_margin_rad'], waist_target_rad=r['commanded_waist_position_range_rad'])
    if recording:
        record_terminal(sim, r)
        r.update(swing_distance=summary, swing_policy=policy)
        Path(recording).write_text(json.dumps(r, allow_nan=False))
    return summary


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def ranking(row):
    return (row['passed'], row['physically_completed'] and row['settled'],
            row['mm_per_swing']-.6*max(0, row['max_pair_end_deg']-20))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--refine-from', type=Path, help='Seed a longer pair-budget search with a prior trained policy')
    p.add_argument('--replay', type=Path, help='Run a saved gait and feedback configuration')
    p.add_argument('--pairs', type=int, default=24)
    p.add_argument('--side', type=int, choices=(-1,1), default=1)
    p.add_argument('--dt', type=float, choices=(.001,.0005,.00025,.000125), default=.00025)
    args=p.parse_args()
    if not 1 <= args.pairs <= 100:
        p.error('Use a pair budget from 1 through 100')
    args.output.mkdir(parents=True, exist_ok=False)
    nominal=read('configs/motors-speed-trained.json')
    sensor=read('configs/heading-sensor-assumed.json')
    if args.replay:
        policy=read(args.replay)
        result=trial((policy.get('selected_candidate','replay'),policy,dict(nominal,timestep_s=args.dt),
                      args.side,args.pairs,sensor,str(args.output/'walk.json')))
        save(args.output/'summary.json',result)
        print(json.dumps(result))
        return
    anchors={name:dict(gait=read('configs/sway-'+tag+'.json'),feedback=read('configs/heading-'+tag+'.json'))
             for name,tag in [('rocking','rocking-experimental'),('rapid','faster-trained')]}
    refining = args.refine_from is not None
    training_seed = 2026112103 if refining else 2026112101
    rng=np.random.default_rng(training_seed)
    all_policies=dict(anchors)
    history=[]
    def job(name,pairs,dt,side=1,config=None,record=None,sensing=None):
        return (name,all_policies[name],dict(config or nominal,timestep_s=dt),side,pairs,sensing or sensor,record)
    def batch(pool,jobs,label):
        rows=list(pool.map(trial,jobs))
        history.append(dict(stage=label,trials=rows))
        save(args.output/'history.json',history)
        best=max(rows,key=ranking)
        print(label,len(rows),sum(r['passed'] for r in rows),'passes; best',best['name'],round(best['mm_per_swing'],3),flush=True)
        return rows
    def shortlist(rows,count):
        return [r['name'] for r in sorted(rows,key=ranking,reverse=True)[:count]]
    # Mutation scales in the same physical units/order as the existing rocking search.
    spread=np.array([.40,3,5,9,2,2.4,3.8,12,1.2,.4,.35,.6,.3,.06,1.4,12,.09,.06,.04,.035,.45,10])
    parents=list(anchors)
    if refining:
        first = read(args.refine_from)
        all_policies['previous'] = {key:first[key] for key in ('gait', 'feedback')}
        parents.append('previous')
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(5 if refining else 4):
            new=[]
            for n in range(36 if refining else 24):
                base=all_policies[parents[n%len(parents)]]
                radius = (2.0 if n%4==0 else .7) if refining else 1.
                values=vector(base['gait'])+rng.normal(size=len(spread))*spread*(.80**gen)*radius
                gait=decode(values)
                name=f'g{gen}-{n:02}'
                feedback=dict(base['feedback'])
                if refining:
                    feedback['kp']=float(np.clip(feedback['kp']*math.exp(rng.normal(0,.3)),.03,.5))
                    feedback['yaw_per_roll']=float(np.clip(feedback['yaw_per_roll']+rng.normal(0,.4),-4,2))
                    feedback['startup_ramp_cycles']=float(np.clip(feedback['startup_ramp_cycles']+rng.normal(0,.35),1,3))
                all_policies[name]=dict(gait=gait,feedback=feedback)
                new.append(name)
            screen_pairs = 16 if refining else 8
            screen_side = -1 if refining and gen%2 else 1
            rows=batch(pool,[job(name,screen_pairs,.0005 if refining else .001,screen_side)
                             for name in parents+new],f'generation-{gen}')
            parents=shortlist(rows,5)
            save(args.output/'candidates.json',all_policies)
        # Select on sustained nominal performance on both starting sides and two integration steps.
        candidates=list(dict.fromkeys(parents+list(anchors)))
        rows=batch(pool,[job(n,24,dt,s) for n in candidates for dt in (.0005,.00025) for s in (-1,1)],'selection')
        if refining:
            rows = rows + batch(pool,[job(n,64,.0005,s) for n in candidates for s in (-1,1)],'selection-long')
        def group_key(n):
            checks=[r for r in rows if r['name']==n]
            return sum(r['passed'] for r in checks),min(r['mm_per_swing'] for r in checks)
        chosen=max([n for n in candidates if n not in anchors],key=group_key)
        policy=dict(all_policies[chosen], selected_candidate=chosen, evidence='uncalibrated_simulation',
                    hardware_ready=False,training_seed=training_seed,controller='SwingSequence',waist_mode='hold_neutral')
        save(args.output/'policy.json',policy)
        print('Frozen:',chosen,group_key(chosen),flush=True)
        # No further selection uses these conditions or the longer run.
        holdout=np.random.default_rng(training_seed+1)
        jobs=[]
        stress=[]
        for i in range(6):
            config=copy.deepcopy(nominal)
            config['friction_slide_torsion_roll'][0]=float(holdout.uniform(.55,.9))
            config['motor_model'].update(voltage_v=float(holdout.uniform(4.7,5.5)),pid_scale=float(holdout.uniform(.9,1.1)),
                delay_frames=int(holdout.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
            sensing=dict(sensor,seed=int(holdout.integers(100000)),delay_frames=int(holdout.integers(1,4)))
            stress.append(dict(config=config,sensor=sensing))
            for side in (-1,1):
                for n in (chosen,'rocking','rapid'):
                    jobs.append(job(n,24,.0005,side,config,sensing=sensing))
        results=batch(pool,jobs,'fresh')
        for i,row in enumerate(results):row['condition']=i//6
        save(args.output/'stress-conditions.json',stress)
        batch(pool,[job(n,80,.00025,s) for n in (chosen,'rocking','rapid') for s in (-1,1)],'long')
        batch(pool,[job(chosen,24,.00025,1,record=str(args.output/'walk.json'))],'video')
    save(args.output/'history.json',history)
    save(args.output/'candidates.json',all_policies)


if __name__=='__main__':
    main()
