"""Train isolated, fast head strokes under a low active-cadence budget in MuJoCo."""
import argparse
import copy
import json
import math
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from reachy_locomotion.geometry import rotation
from reachy_locomotion.primitives import Primitive, PrimitiveSimulator, blend, stroke
from reachy_locomotion.swing_distance import cumulative_strokes
from train_swing_distance import SwingSequence, read, save

TICK = .02
BLOCK = 8.
COST_MM = 3.0
CACHE = {}


class SpacedSwing:
    """Each directional stroke has independently timed roll, lateral and yaw sweeps."""
    record_swing_dynamics = True
    catch_trigger = yaw_trigger = None

    def __init__(self, policy, side, seconds):
        self.policy = policy
        self.side = side
        self.seconds = seconds
        self.count = round(policy['swings_per_block'] * seconds / BLOCK)
        self.interval = seconds / self.count

    def yaw_axis(self, index, local, start, end, roll):
        p = self.policy
        return start + (end-start)*stroke((local-p['yaw_delay'])/p['yaw_s'], p['brake'])

    def target(self, t, roll):
        p = self.policy
        if t >= self.seconds:
            return neutral_pose(), 'settle', roll
        i = min(int(t / self.interval), self.count-1)
        local = t - i*self.interval
        end = self.side * (-1 if i % 2 else 1)
        start = -end if i else 0
        end = end if i < self.count-1 else 0
        def axis(delay, duration):
            return start + (end-start)*stroke((local-delay)/duration, p['brake'])
        head_roll = p['roll']*axis(p['roll_delay'], p['roll_s'])
        head_yaw = p['yaw']*self.yaw_axis(i, local, start, end, roll)
        head_y = p['y']*axis(p['y_delay'], p['y_s'])
        sweep = stroke((local-p['roll_delay'])/p['roll_s'],p['brake'])
        arc = math.sin(math.pi*sweep) * (abs(end-start)/2)
        envelope = blend(min(t, self.seconds-t)/.4)
        pose = neutral_pose()
        pose[:3,:3] = rotation(np.radians([head_roll, p['pitch']*envelope, head_yaw]))
        pose[:3,3] += np.array([p['x']*envelope+p.get('x_arc',0)*arc, head_y,
                                p['z']*envelope+p.get('z_arc',0)*arc])/1000
        finish = max(p['roll_delay']+p['roll_s'],p['yaw_delay']+p['yaw_s'],p['y_delay']+p['y_s'])
        return pose, ('stroke' if local<finish else 'endpoint hold'), roll


class EdgeTriggeredSwing(SpacedSwing):
    """Time the yaw stroke from delayed base roll and a single 50 Hz gyro sample."""

    def __init__(self, policy, side, seconds):
        super().__init__(policy, side, seconds)
        self.gate = policy['edge_gate']
        self.samples = deque([(0., 0.)] * (1+self.gate['delay_ticks']),
                             maxlen=1+self.gate['delay_ticks'])
        self.index = -1
        self.level = self.origin = self.rate = 0.
        self.launch = None
        self.stopped = False
        self.launches = []
        self.stops = []
        self.head_yaw_deg = 0.
        self.head_samples = deque([0.] * (1+self.gate['delay_ticks']),
                                  maxlen=1+self.gate['delay_ticks'])
        self.steering = policy.get('pair_steering')
        if self.steering:
            self.heading_history = deque([0.]*4,maxlen=4)
            self.heading_rng = np.random.default_rng(self.steering.get('seed',19031))
        self.pair_heading = 0.
        self.yaw_delay = policy['yaw_delay']

    def observe_heading(self, t, degrees):
        if self.steering:
            self.heading_history.append(degrees+self.heading_rng.normal(0,.25))

    def observe_head(self, t, degrees):
        self.head_samples.append(degrees)
        self.head_yaw_deg = self.head_samples[0]

    def observe_angular_velocity(self, t, velocity):
        self.rate = math.degrees(velocity[0])

    def yaw_axis(self, index, local, start, end, roll):
        gate = self.gate
        self.samples.append((roll, self.rate))
        angle, rate = self.samples[0]
        predicted = abs(angle + gate['lead_s']*rate)
        if index != self.index:
            self.index, self.origin = index, self.level
            self.launch, self.stopped = None, False
            if self.steering:
                if index % 2 == 0:
                    self.pair_heading = self.heading_history[0]
                direction = self.side * (-1 if index%2 else 1)
                limit = self.steering.get('max_yaw_bias',.16)
                bias = np.clip(self.steering['yaw_bias_per_deg']*self.pair_heading,-limit,limit)
                self.yaw_end = end + bias if end else 0.
                shift = self.steering['delay_s_per_deg']*self.pair_heading*direction
                self.yaw_delay = self.policy['yaw_delay'] + float(np.clip(shift,-.06,.06))
                self.yaw_delay = max(0.,self.yaw_delay)
        if self.steering:
            end = self.yaw_end
        if self.launch is None and local >= self.yaw_delay:
            if predicted >= gate['launch_deg']:
                self.launches.append(index+1)
                self.launch = local
            elif local >= self.yaw_delay + gate['max_wait_s']:
                self.launch = local
        if self.launch is None or self.stopped:
            return self.level
        phase = (local-self.launch)/self.policy['yaw_s']
        self.level = self.origin + (end-self.origin)*stroke(phase, self.policy['brake'])
        # Complete the final return to neutral even if the base rocks.
        if index < self.count-1 and gate['min_fraction'] <= phase < 1 and predicted >= gate['stop_deg']:
            self.stopped = True
            self.stops.append(index+1)
        # The physical head can still be moving after its reference reaches the endpoint.
        # Pull the reference toward delayed encoder yaw once the base starts rocking.
        if (gate.get('catch_deg') and not self.stopped and index < self.count-1
                and phase >= 1 and local <= .48 and predicted >= gate['catch_deg']):
            pending = self.level*self.policy['yaw'] - self.head_yaw_deg
            sweep_sign = (end-self.origin)*self.policy['yaw']
            if pending*sweep_sign > 0 and abs(pending) >= 2:
                correction = np.clip(pending, -gate['catch_range_deg'], gate['catch_range_deg'])
                self.level -= float(correction)/self.policy['yaw']
                self.stopped = True
                self.stops.append(index+1)
        return self.level


def neutral_pose():
    pose = np.eye(4)
    pose[2,3] = .177
    return pose


def count_events(values, band):
    counts = np.asarray(list(cumulative_strokes(values, band)))
    return np.flatnonzero(np.diff(counts, prepend=0))


def cadence_measurement(axes, seconds, count):
    events = [count_events(x, band) for x, band in zip(axes,(10,10,5))]
    # Count any settling rebound against the active clock. Axis counts overlap for a compound swing.
    n = max(count, *(len(e) for e in events))
    min_gap = min((int(np.min(np.diff(e)))*TICK for e in events if len(e)>1), default=seconds)
    window = 100   # half-open 2-second windows, sampled every 20 ms
    peak_2s = max((int(np.max(np.searchsorted(e,e+window)-np.arange(len(e)))) for e in events if len(e)),default=0)
    return n, [len(e) for e in events], min_gap, peak_2s


def assess(r, seconds, count):
    rows, trace = r['trajectory'], r['trace']
    head = np.degrees([q['head_rpy_rad'] for q in rows])
    axes = (head[:,0], head[:,2], [q['head_y_mm'] for q in rows])
    n, observed, min_gap, peak_2s = cadence_measurement(axes,seconds,count)
    heading = np.asarray([row['foot_yaw_deg'] for row in trace])
    time = np.asarray([row['time_s'] for row in trace])
    pairs = np.rint(np.arange(1,count//2+1)*2*seconds/count/TICK).astype(int)
    ends = pairs[pairs<len(trace)]
    max_heading = max((abs(heading[i]) for i in ends),default=0.)
    max_pair_change = max(abs(np.diff(np.r_[0., heading[ends]])), default=0.)
    yaw0 = rows[0]['foot_rpy_rad'][2]
    forward = np.array([math.cos(yaw0),math.sin(yaw0)])
    distance = 1000*float(forward@r['planar_displacement_m'])
    half_idx = min(len(trace)-1,round(seconds/2/TICK))
    mid_delta = np.subtract(trace[half_idx]['foot_xyz_m'][:2],trace[0]['foot_xyz_m'][:2])
    late_distance = distance-1000*float(forward@mid_delta)
    edge = np.asarray([q['edge_support'] for q in trace])
    rate = np.gradient(head[:,2],TICK) if len(rows)>1 else np.zeros(len(rows))
    speed = abs(rate)
    slowing = (speed>20) & (np.gradient(speed,TICK)<-100) if len(rows)>1 else np.zeros(len(rows),bool)
    spin = np.array([q['head_spin_z_kg_m2_s'] for q in rows])
    # Head yaw deceleration on a loaded rim, counted once per directional stroke.
    brake_events = []
    brake_sides = {key: [] for key in ('left','right')}
    roll = np.array([q['roll_deg'] for q in trace])
    for i in range(count):
        zone = (time>=i*seconds/count) & (time<(i+1)*seconds/count)
        indices = np.flatnonzero(zone & edge & slowing)
        if len(indices):
            brake_events.append(i+1)
            for side, direction in [('left', -1), ('right', 1)]:
                if np.any(direction*roll[indices] > 0):
                    brake_sides[side].append(i+1)
    rates_ok = n/seconds <= 1.30 and min_gap>.5 and peak_2s<4
    course_ok = len(ends)==count//2 and max_heading<=20 and abs(heading[-1])<=20
    passed = bool(r['completed'] and r['settled'] and rates_ok and course_ok and distance>0 and late_distance>0)
    return dict(passed=passed,cadence_ok=bool(rates_ok),course_ok=bool(course_ok),termination=r['termination'],
                completed=r['completed'],settled=r['settled'],fell=r['fell'],active_seconds=seconds,
                planned_swings=count,counted_swings=n,observed_strokes=observed,
                cadence_hz=n/seconds,min_observed_gap_s=min_gap,peak_axis_strokes_in_2s=peak_2s,
                distance_mm=distance,late_distance_mm=late_distance,mm_per_swing=distance/n,
                mm_per_active_s=distance/seconds,utility=(distance-COST_MM*n)/seconds,
                max_pair_end_deg=float(max_heading),max_pair_change_deg=float(max_pair_change),final_heading_deg=float(heading[-1]),
                edge_s=r['edge_support_seconds'],rim_lift_mm=r['peak_side_height_difference_mm'],
                peak_tilt_deg=r['peak_tilt_deg'],edge_brake_strokes=brake_events,
                edge_brake_by_roll_sign=brake_sides,
                peak_head_yaw_speed_deg_s=float(max(speed)),peak_head_spin_z_kg_m2_s=float(max(abs(spin))),
                head_negative_work_j=r['head_braking_work_j'],waist_target_rad=r['commanded_waist_position_range_rad'],
                min_joint_margin_rad=r['min_joint_margin_rad'])


def rollout(job):
    name, policy, side, seconds, cfg, recording, *review_failures = job
    key = json.dumps(cfg,sort_keys=True)
    if key not in CACHE:
        CACHE[key] = PrimitiveSimulator('experiments/upstream-reachy-mini',cfg)
    if 'gait' in policy:
        pairs = round(seconds*policy['gait']['frequency_hz'])
        ctl = SwingSequence(policy,side,pairs,read('configs/heading-sensor-assumed.json'))
        ctl.record_swing_dynamics = True
        count = pairs*2
    else:
        controller_type = EdgeTriggeredSwing if 'edge_gate' in policy else SpacedSwing
        ctl = controller_type(policy,side,seconds)
        count = ctl.count
    r = CACHE[key].run_primitive(Primitive(),seconds=seconds+1.6,waist_mode='hold_neutral',
                                 record=True,controller=ctl,tilt_limit_deg=None)
    summary = dict(assess(r,seconds,count),name=name,side=side,dt=cfg['timestep_s'])
    if isinstance(ctl, EdgeTriggeredSwing):
        summary.update(sensor_launch_strokes=ctl.launches, sensor_stop_strokes=ctl.stops)
    # Only attribute motion to the rim when both endpoints have strict support.
    trace = r['trace']
    active = [q for q in trace if q['time_s'] <= seconds]
    aligned = [(a,b) for a,b in zip(active,active[1:]) if a['edge_support'] and b['edge_support']
               and a['roll_deg']*b['roll_deg'] > 0]
    yaw0 = r['trajectory'][0]['foot_rpy_rad'][2]
    forward = np.array([math.cos(yaw0),math.sin(yaw0)])
    summary['edge_forward_mm'] = sum(1000*float(forward @ np.subtract(b['foot_xyz_m'][:2],a['foot_xyz_m'][:2])) for a,b in aligned)
    summary['edge_absolute_pivot_deg'] = sum(abs(b['foot_yaw_deg']-a['foot_yaw_deg']) for a,b in aligned)
    if recording:
        if not r['completed'] and review_failures != [True]:
            raise ValueError('Incomplete demonstration; enable failure review to retain this run')
        r.update(cadence_policy=policy,cadence_result=summary)
        Path(recording).write_text(json.dumps(r,allow_nan=False))
    return summary


# Limits here bound requested poses and stroke times. Motor constraints are applied downstream by MuJoCo.
BOUNDS = dict(roll=(17,39),yaw=(-48,48),y=(-24,24),pitch=(-10,14),x=(-15,18),z=(-9,3),
              roll_s=(.08,.36),yaw_s=(.08,.38),y_s=(.08,.38),roll_delay=(0,.20),
              yaw_delay=(0,.32),y_delay=(0,.20),brake=(.025,.30))


def sample(rng, parent=None, scale=.12):
    p={}
    for key,(low,high) in BOUNDS.items():
        v=rng.uniform(low,high) if parent is None else rng.normal(parent[key],scale*(high-low))
        p[key]=float(np.clip(v,low,high))
    p['swings_per_block']=int(rng.choice([6,8,10])) if parent is None or rng.random()<.23 else parent['swings_per_block']
    return p


def rank(r):
    return (r['passed'] and len(r['edge_brake_strokes'])>=2, r['passed'],
            r['completed'] and r['settled'] and r['cadence_ok'],r['utility'])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--replay',type=Path)
    ap.add_argument('--seconds',type=int,choices=[8,16,24,40],default=24)
    ap.add_argument('--side',type=int,choices=[1,-1],default=1)
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    model=read('configs/motors-speed-trained.json')
    if args.replay:
        cfg=dict(model,timestep_s=.00025)
        s=rollout(('replay',read(args.replay),args.side,args.seconds,cfg,str(args.out/'walk.json')))
        save(args.out/'summary.json',s)
        return
    rng=np.random.default_rng(2026112107)
    history={}
    policies={}
    def jobs(names,sides,seconds,dt):
        return [(n,policies[n],s,seconds,dict(model,timestep_s=dt),None) for n in names for s in sides]
    def batch(executor,label,items):
        rows=list(executor.map(rollout,items))
        history[label]=rows
        save(args.out/'history.json',history)
        best=max(rows,key=rank)
        print(label,len(rows),sum(r['passed'] for r in rows),'best',best['name'],round(best['distance_mm'],1),best['counted_swings'],flush=True)
        return rows
    with ProcessPoolExecutor(max_workers=4) as ex:
        parents=[]
        for g in range(4):
            new=[]
            for i in range(28):
                name=f'g{g}-{i}'
                parent=policies[parents[i%len(parents)]] if parents and i%7 else None
                policies[name]=sample(rng,parent,.16*.78**g)
                new.append(name)
            rows=batch(ex,f'generation-{g}',jobs(new+parents,[1 if g%2==0 else -1],8,.001))
            parents=[r['name'] for r in sorted(rows,key=rank,reverse=True)[:6]]
            save(args.out/'candidates.json',policies)
        rows=batch(ex,'selection',jobs(parents,[-1,1],16,.0005))
        def group_rank(name,rows):
            rs=[r for r in rows if r['name']==name]
            return (sum(rank(r)[0] for r in rs),sum(r['passed'] for r in rs),min(r['utility'] for r in rs))
        finalists=sorted(parents,key=lambda n:group_rank(n,rows),reverse=True)[:3]
        fine=batch(ex,'confirmation',jobs(finalists,[-1,1],24,.00025))
        winner=max(finalists,key=lambda n:group_rank(n,fine))
        policy=policies[winner]
        save(args.out/'policy.json',dict(policy,selected_candidate=winner,training_seed=2026112107,
                                         evidence='uncalibrated_simulation',hardware_ready=False))
        print('frozen',winner,flush=True)
        comparisons={'soft-brake':dict(policy,brake=.45),
                     'longer-stroke':dict(policy,roll_s=.44,yaw_s=.44,y_s=.44,brake=.45),
                     'no-yaw':dict(policy,yaw=0.)}
        old=read('configs/head-swing-distance-trained.json')
        comparisons['old-retimed']=dict(old,gait=dict(old['gait'],frequency_hz=.5))
        policies.update(comparisons)
        batch(ex,'controls',jobs(comparisons,[-1,1],24,.00025))
        stress_rng=np.random.default_rng(2026112108)
        tests=[];conditions=[]
        for i in range(5):
            c=copy.deepcopy(model);c['timestep_s']=.0005
            c['friction_slide_torsion_roll'][0]=float(stress_rng.uniform(.55,.9))
            c['motor_model'].update(voltage_v=float(stress_rng.uniform(4.7,5.5)),pid_scale=float(stress_rng.uniform(.9,1.1)),
                                    delay_frames=int(stress_rng.integers(0,3)),mass_distribution=['torso','uniform'][i%2])
            conditions.append(c)
            tests.extend((f'heldout-{i}',policy,s,16,c,None) for s in (-1,1))
        save(args.out/'conditions.json',conditions)
        batch(ex,'heldout',tests)
        batch(ex,'long',jobs([winner],[-1,1],40,.00025))
        batch(ex,'timestep',jobs([winner],[-1,1],24,.000125))


if __name__=='__main__':
    main()
