"""Learn a capture from upright and search for large closed foot-center circles at 31 degrees."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np

from fit_traveling_edge import TravelingEdge, MODELS, simulator
from reachy_locomotion.geometry import euler, rotation
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.rim_roll import ground_course, measures
from train_target_pivot import read, write

WORK = Path('experiments/generated/deep-lift')
POLICY = Path('configs/deep-lift-circle.json')
TEMPLATE = Path('configs/deep-lift-reference.json')
ATTITUDE = Path('configs/deep-circle-attitude.json')
FAILED = ('edge_floor', 'edge_battery', 'edge_noise', 'edge_fine')
FRESH = ('lift_floor', 'lift_battery', 'lift_noise', 'lift_fine')
MODELS.update(lift_floor=(.68,5.,1.,1,'torso',.000125,1,0.,0),
              lift_battery=(.7,4.97,.995,1,'torso',.000125,1,0.,0),
              lift_noise=(.7,5.,1.,1,'torso',.000125,1,.03,5151),
              lift_fine=(.7,5.,1.,1,'torso',.02/180,1,0.,0),
              lift_noise_8715=(*MODELS['edge_noise'][:-1],8715),
              lift_noise_9715=(*MODELS['edge_noise'][:-1],9715),
              lift_mixed_noise=(.69,4.99,.998,1,'torso',.000125,1,.03,319107))


def choose_launch(prefix, library):
    observed=np.asarray(prefix).reshape(-1,2)
    profiles=np.asarray([entry['prefix'] for entry in library]).reshape(len(library),-1,2)
    clean=profiles[[not entry.get('noise_example',False) for entry in library]]
    def variability(profile):
        delta=profile-clean
        nearest=delta[np.argmin(np.mean(delta**2,axis=(1,2)))]
        return np.sqrt(np.mean(np.diff(nearest,n=2,axis=0)**2)/6.)
    noise=variability(observed)
    difference=np.sqrt(np.mean((profiles-observed)**2,axis=(1,2)))
    distance=difference+5.*np.abs([variability(profile)-noise for profile in profiles])
    return library[int(np.argmin(distance))]['params']


class LiftCircle(TravelingEdge):
    gyro_interval_average = True

    def __init__(self, sim, condition, lift, orbit):
        super().__init__(sim,-1,orbit.get('period',14.),1,orbit.get('gain',1.),condition,
                         sector_mix=orbit.get('mix',.55))
        self.lift = lift
        self.orbit = orbit
        self.prefix = []
        self.selected = None
        self.yaw = 0.
        self.lean_shift = 0.
        self.filtered = None
        self.replay_target = None
        self.correction_blend = 0.
        self.reference_frames = self.residual_frames = 0
        self.imu_rng = np.random.default_rng(MODELS[condition][-1]+701)
        self.noise = MODELS[condition][-2]

    def observe_angular_velocity(self,t,velocity):
        self.gyro = np.asarray(velocity)

    def observe_head_state(self,t,pose):
        if self.yaw and self.orbit.get('head_axis',False):
            pose=pose.copy()
            pose[:3,:3]=pose[:3,:3]@rotation((0.,0.,-self.yaw))
        super().observe_head_state(t,pose)

    def filter_attitude(self,roll,tau):
        measured=np.array([roll,self.pitch])+self.imu_rng.normal(0,self.noise,2)
        if self.filtered is None:
            self.filtered=measured
        else:
            phi,theta=np.radians(self.filtered)
            p,q,r=self.gyro+self.imu_rng.normal(0,np.radians(.1),3)
            rates=np.degrees([p+np.sin(phi)*np.tan(theta)*q+np.cos(phi)*np.tan(theta)*r,
                              np.cos(phi)*q-np.sin(phi)*r])
            predicted=self.filtered+.02*rates
            self.filtered=predicted+(1-np.exp(-.02/tau))*(measured-predicted)
        return self.filtered

    def reference_angle(self,elapsed):
        phase=super().reference_angle(elapsed)
        return phase + self.orbit.get('skew',0.)*np.sin(phase)

    def mix_models(self):
        super().mix_models()
        if self.lean_shift:
            self.child.xeq[0]+=self.lean_shift

    def target(self, t, roll):
        # This prefix precedes the first change to the launch motion. Use only
        # delayed, noisy attitude observations already available to the regulator.
        if 0.9 <= t <= 1.18:
            self.prefix.extend(self.child.sensed.tolist())
        if t >= 1.18 and self.selected is None:
            if 'library' in self.lift:
                self.selected = choose_launch(self.prefix,self.lift['library'])
            else:
                self.selected = self.lift
            self.child.f = replace(self.child.f, **self.selected.get('feedback', {}))
            self.child.p = replace(self.child.p, **self.selected.get('recipe', {}))
            self.child.launch.p = self.child.p  # left launch, unit mirror
            self.child.capture_gain = self.selected.get('capture_gain', self.gain0)
            self.child.hold_transition_s = .7
            self.child.tracking_error_limit = self.selected.get('tracking', 0.)
            if 'imu_tau_s' in self.selected:
                self.child.f=replace(self.child.f,sensor_noise_deg=0.)
        if self.selected and 'prior_halfwidth_deg' in self.selected:
            prior=attitude_prior()
            phase_time=t if self.start is None else t-self.start+prior['capture_s']
            index=round(phase_time/.02)
            if index >= len(prior['rpy']):
                period=round(prior['period_s']/.02)
                index=len(prior['rpy'])-period+(index-len(prior['rpy']))%period
            expected=np.asarray(prior['rpy'][index])
            measured=np.array([roll,self.pitch])+self.imu_rng.normal(0,self.noise,2)
            error=measured-expected
            attitude=measured-np.clip(error,-self.selected['prior_halfwidth_deg'],self.selected['prior_halfwidth_deg'])
            roll,self.pitch=attitude
            if 'residual_gain' in self.selected:
                self.replay_target=np.asarray(prior['head_pose'][index])
                excess=max(0.,float(np.max(np.abs(error)))-self.selected['prior_halfwidth_deg'])
                self.correction_blend=min(1.,self.selected['residual_gain']*excess)
        elif self.selected and 'imu_tau_s' in self.selected:
            roll,self.pitch=self.filter_attitude(roll,self.selected['imu_tau_s'])
        if self.selected and self.selected.get('positive_catch') and self.child.launch.catch_trigger is None:
            # On a weaker bus the first, opposite wind-up can cross the old
            # absolute threshold early. Arm braking on the positive excursion.
            armed = self.child.sensed[0]>0 and t>=self.selected.get('arm_s',0.)
            threshold = self.selected['positive_catch'] if armed else 0.
            self.child.p = replace(self.child.p, catch_at_deg=threshold)
            self.child.launch.p = self.child.p
        if self.start is not None:
            self.child.tracking_error_limit = 0.
            ramp = min(1., (t-self.start)/2.)
            wave=self.orbit.get('yaw_wave',0.)*np.sin(self.angle)
            stroke=2*np.pi*(t-self.start)*self.orbit.get('shake_hz',1.)
            shake=self.orbit.get('shake_deg',0.)*(np.sin(stroke)+self.orbit.get('harmonic',0.)*np.sin(2*stroke))
            self.lean_shift=ramp*self.orbit.get('nutation',0.)*np.sin(stroke+np.radians(self.orbit.get('nutation_phase',0.)))
            if self.orbit.get('lean_trim',0.):
                sensed=np.radians(self.child.sensed)
                lean=np.degrees(np.arccos(np.clip(np.cos(sensed).prod(),-1,1)))
                self.lean_shift+=ramp*self.orbit['lean_trim']*np.clip(31.-lean,-2.,2.)
            self.yaw = ramp*np.radians(self.orbit.get('yaw',0.)+wave+shake)
        pose,stage,sensed=super().target(t,roll)
        if self.selected and t<self.selected.get('reference_until_s',0.):
            frame=round(t/.02)
            ref=reference()
            poses=ref['poses'] if isinstance(ref,dict) else ref
            pose=np.asarray(poses[frame])
            if 'reference_gain' in self.selected:
                delta=self.child.state-np.asarray(ref['states'][frame])
                correction=-self.selected['reference_gain']*(self.child.K@delta)
                correction=np.clip(correction,[-18,-18,-9,-9],[18,18,9,9])
                angles=euler(pose)
                for scale in (1.,.5,.25,.125,0.):
                    move=scale*correction
                    candidate=pose.copy()
                    candidate[:3,:3]=rotation(angles+np.radians([self.side*move[0],move[1],0.]))
                    candidate[:2,3]+=np.array([move[2],self.side*move[3]])/1000
                    if np.any(np.abs(candidate[:2,3])>.0250001):
                        continue
                    try:
                        self.sim.checked_ik(candidate)
                    except ValueError:
                        continue
                    pose=candidate
                    break
                pitch_roll=np.degrees(euler(pose))
                self.child.command_history[-1]=np.array([self.side*pitch_roll[0],pitch_roll[1],
                                                         1000*pose[0,3],self.side*1000*pose[1,3]])
        if self.replay_target is not None:
            # Imitate the recorded motor target inside the IMU noise band.
            # Outside it, mix in the live edge regulator and check physical IK.
            ref=self.replay_target
            blend=self.correction_blend
            self.reference_frames+=1
            self.residual_frames+=int(blend>0)
            if blend==0.:
                pose=ref
            elif blend<1.:
                candidate=ref.copy()
                candidate[:3,:3]=rotation((1-blend)*euler(ref)+blend*euler(pose))
                candidate[:3,3]=(1-blend)*ref[:3,3]+blend*pose[:3,3]
                try:
                    self.sim.checked_ik(candidate)
                    pose=candidate
                except ValueError:
                    pass
        return pose,stage,sensed

    def physical(self, pose):
        out = super().physical(pose)
        if self.yaw:
            out = out.copy()
            yaw=rotation((0.,0.,self.yaw))
            if self.orbit.get('head_axis',False):
                out[:3,:3]=out[:3,:3]@yaw
            else:
                out[:3,:3]=yaw@out[:3,:3]
        return out


def evaluate(job):
    name, lift, orbit, seconds, *record = job
    sim = simulator(name)
    ctl = LiftCircle(sim,name,lift,orbit)
    run = sim.run_primitive(Primitive(),seconds=seconds,controller=ctl,waist_mode='hold_neutral',
                            tilt_limit_deg=None,record_rim_contact=True,record=bool(record))
    rows = run['trace']
    safe = run['completed'] and run['min_joint_margin_rad'] >= 0
    active = [r for r in rows if ctl.start is not None and r['time_s'] >= ctl.start]
    held_after_capture = (len(active)>=26 and all(28<=r['tilt_deg']<=35 and r['normal_n']>.5
                          and r['contact_foot_xy_m'] is not None
                          and np.linalg.norm(r['contact_foot_xy_m'])>=.85*sim.config['foot_radius_m']
                          for r in active[:26]))
    result = dict(condition=name,params=ctl.selected,orbit=orbit,prefix=ctl.prefix,
                  seconds=seconds,elapsed_s=run['elapsed_seconds'],safe=safe,capture_s=ctl.start,
                  launch_pass=bool(held_after_capture),termination=run['termination'],
                  peak_lean_deg=run['peak_tilt_deg'],joint_margin_rad=run['min_joint_margin_rad'],
                  waist_rad=run['commanded_waist_position_range_rad'],fell=run['fell'],
                  max_signed_roll=max(-r['roll_deg'] for r in rows),
                  end_lean_deg=rows[-1]['tilt_deg'])
    result['brake_s'] = ctl.child.launch.catch_trigger
    result['catch_s'] = ctl.child.catch_trigger
    if ctl.reference_frames:
        result['reference_frames']=ctl.reference_frames
        result['residual_feedback_frames']=ctl.residual_frames
    near = np.array([[r['roll_deg']+32.,r['roll_rate_deg_s'],r['pitch_deg']] for r in rows])
    result['near_hold_s'] = float(.02*np.exp(-.5*((near/[4.,28.,8.])**2).sum(axis=1)).sum())
    if len(active)>2:
        result['rim'] = measures(active,sim.config['foot_radius_m'],1,0.)
    if len(active)>200 and seconds>10:
        times=np.array([r['time_s'] for r in active])
        lean=np.array([r['tilt_deg'] for r in active])
        cop=np.array([r['contact_foot_xy_m'] or [0.,0.] for r in active])
        loaded=(np.linalg.norm(cop,axis=1)>=.85*sim.config['foot_radius_m']) & (lean>=5.)
        loaded &= np.array([r['normal_n']>.5 for r in active])
        azimuth=np.arctan2(cop[:,1],cop[:,0])
        continuous=loaded[1:] & loaded[:-1] & (np.abs(np.angle(np.exp(1j*np.diff(azimuth))))<np.pi/2)
        def lean_window(loop):
            first=np.searchsorted(times,loop['start_s'])
            last=np.searchsorted(times,loop['end_s'],side='right')
            window=lean[first:last]
            return (30<=np.median(window)<=32 and np.mean((window>=29)&(window<=33))>=.95
                    and window.min()>=28 and window.max()<=34 and continuous[first:last-1].mean()>=.95)
        geometric=ground_course(active)['best_loop']
        if geometric and not lean_window(geometric):
            result['widest_geometric_mm']=geometric['diameter_mm']
            loop=ground_course(active,accept_loop=lean_window)['best_loop'] or geometric
        else:
            loop=geometric
        if loop:
            angles = np.array([r['tilt_deg'] for r in active if loop['start_s']<=r['time_s']<=loop['end_s']])
            rim = measures([r for r in active if loop['start_s']<=r['time_s']<=loop['end_s']],sim.config['foot_radius_m'],1,0.)
            loop.update(min_lean=float(angles.min()),max_lean=float(angles.max()),median_lean=float(np.median(angles)),
                        fraction_29_33=float(np.mean((angles>=29)&(angles<=33))),loaded=rim['loaded_fraction'])
            centers=1000*np.array([r['foot_xyz_m'][:2] for r in active if loop['start_s']<=r['time_s']<=loop['end_s']])
            radii=np.linalg.norm(centers-np.array(loop['center_xy_mm']),axis=1)
            loop['raw_mean_diameter_mm']=float(2*radii.mean())
            loop['raw_radius_cv']=float(radii.std()/radii.mean())
            loop['raw_endpoint_gap_mm']=float(np.linalg.norm(centers[-1]-centers[0]))
            section=[r for r in active if loop['start_s']<=r['time_s']<=loop['end_s']]
            roll,pitch,yaw=np.radians([[r[k] for r in section] for k in ('roll_deg','pitch_deg','foot_yaw_deg')])
            # ZYX Euler velocity projected onto the fixed floor normal.
            normal_spin=np.diff(yaw)-np.diff(roll)*np.sin(.5*(pitch[:-1]+pitch[1:]))
            loop['net_floor_normal_spin_deg']=float(np.degrees(normal_spin.sum()))
            theta=np.radians(loop['median_lean'])
            loop['ideal_no_twist_diameter_mm']=float(2000*sim.config['foot_radius_m']*(1/np.cos(theta)-np.cos(theta)))
            result['loop']=loop
    loop=result.get('loop',{})
    result['circle_pass']=bool(safe and loop and 30<=loop['median_lean']<=32 and loop['fraction_29_33']>=.95
                               and loop['min_lean']>=28 and loop['max_lean']<=34 and loop['loaded']>=.95)
    if record:
        write(record[0],dict(trace=rows,trajectory=[r['qpos'] for r in run['trajectory']],metrics=result,config=run['config']))
    return result


def batch(jobs, n):
    with ProcessPoolExecutor(n) as pool:
        return list(pool.map(evaluate,jobs))


@lru_cache(maxsize=1)
def reference():
    return read(TEMPLATE)


@lru_cache(maxsize=1)
def attitude_prior():
    return read(ATTITUDE)


def build_reference():
    sim=simulator('edge_nominal')
    controller=TravelingEdge(sim,-1,14.,1,1.,'edge_nominal',.55)
    poses=[]
    states=[]
    target=controller.target
    def record_target(t,roll):
        result=target(t,roll)
        poses.append(result[0].tolist())
        states.append(controller.child.state.tolist())
        return result
    controller.target=record_target
    sim.run_primitive(Primitive(),seconds=4.,controller=controller,waist_mode='hold_neutral',tilt_limit_deg=None)
    if controller.start is None:
        raise ValueError('Reference launch missed capture')
    write(TEMPLATE,dict(poses=poses,states=states))


def launch_score(r):
    return (r['launch_pass'] and r['safe'],r['launch_pass'],r['capture_s'] is not None,r['safe'] and r['end_lean_deg']>25,
            -abs(r['end_lean_deg']-31),r['elapsed_s'])


def fit(n):
    params=[{}]
    for catch in (8.5,9.5,10.4,11.,12.):
        params.append(dict(recipe=dict(catch_at_deg=catch)))
    for change in (.135,.145,.165,.175):
        params.append(dict(recipe=dict(catch_s=change)))
    for trigger in (22.,24.,26.,29.,30.):
        params.append(dict(feedback=dict(trigger_deg=trigger)))
    for limit in (9.,16.,24.,32.):
        params.append(dict(tracking=limit))
    rows=[]
    for name in FAILED:
        group=batch([(name,p,{},5.2) for p in params],n)
        rows.extend(group)
        write(WORK/'lift-screen.json',rows)
        print(name,sum(r['launch_pass'] for r in group),'/',len(group),flush=True)
    nominal=evaluate(('edge_nominal',{}, {},5.2))
    write(WORK/'baseline.json',nominal)
    freeze(rows,nominal)


def freeze(rows, nominal):
    # Each reference is sampled before the policy selects its correction.
    group=[nominal]+[max([r for r in rows if r['condition']==name],key=launch_score) for name in FAILED]
    write(WORK/'lift.json',dict(library=[dict(prefix=r['prefix'],params=r['params'],
                                             noise_example=r['condition']=='edge_noise') for r in group]))
    print([(r['condition'],r['launch_pass'],r['params']) for r in group],flush=True)


def repair(n):
    rows = read(WORK/'lift-screen.json')
    additional = []
    for name in FAILED:
        if any(r['launch_pass'] for r in rows if r['condition']==name):
            continue
        if name in ('edge_floor','edge_battery'):
            params = [dict(positive_catch=threshold,recipe=dict(catch_s=duration))
                      for threshold in (7.,10.,13.,16.,19.) for duration in (.085,.115,.15)]
            params += [dict(positive_catch=11.,recipe=dict(catch_s=s),feedback=dict(trigger_deg=t))
                       for s in (.09,.125) for t in (22.,25.,27.)]
            params += [dict(recipe=dict(catch_s=s)) for s in (.075,.095,.105,.115,.125)]
        else:
            params = [dict(tracking=l,capture_gain=g,feedback=dict(trigger_deg=t))
                      for l in (12.,24.,0.) for g in (.8,1.4) for t in (25.,27.,29.)]
            params += [dict(recipe=dict(catch_s=s),capture_gain=g)
                       for s in (.145,.151,.158,.165) for g in (1.,1.3)]
        group=batch([(name,p,{},5.4) for p in params],n)
        additional.extend(group)
        write(WORK/'lift-repair.json',additional)
        print(name,sum(r['launch_pass'] for r in group),'/',len(group),flush=True)
    freeze(rows+additional,read(WORK/'baseline.json'))


def tune(n):
    build_reference()
    old=read(WORK/'lift-screen.json')+read(WORK/'lift-repair.json')
    additions=[]
    for name in FAILED:
        if any(r['launch_pass'] for r in old if r['condition']==name):
            continue
        if name=='edge_noise':
            params=[dict(reference_until_s=stop,imu_tau_s=tau)
                    for stop in (2.6,2.9,3.18,3.34) for tau in (.02,.06,.15,.6)]
            params.extend(dict(reference_until_s=stop) for stop in (2.6,2.9,3.18,3.34))
        elif name=='edge_battery':
            params=[dict(positive_catch=threshold,arm_s=arm,recipe=dict(catch_s=.13))
                    for arm in (1.65,1.9,2.15,2.4,2.65,3.,3.4) for threshold in (8.,14.,20.)]
            params.extend(dict(positive_catch=threshold,recipe=dict(catch_s=.1,roll_deg=40.,
                               y_mm=-24.,pump_hz=hz)) for hz in (2.3,2.55,2.8,3.) for threshold in (9.,14.))
        else:
            params=[dict(recipe=dict(catch_s=s),feedback=dict(trigger_deg=t,capture_blend_s=b))
                    for s in (.098,.104,.11,.118) for t in (26.5,28.1,29.) for b in (.09,.135)]
        group=batch([(name,p,{},5.6) for p in params],n)
        additions.extend(group)
        write(WORK/'lift-tune.json',additions)
        print(name,sum(r['launch_pass'] for r in group),'/',len(group),flush=True)
    freeze(old+additions,read(WORK/'baseline.json'))


def orbit_score(r):
    return (r['circle_pass'],r.get('loop',{}).get('diameter_mm',0.) if r['circle_pass'] else 0.,r['safe'])


def refine_capture(n):
    build_reference()
    old=sum((read(WORK/f'lift-{stage}.json') for stage in ('screen','repair','tune')),[])
    rng=np.random.default_rng(38191)
    policy=read('configs/edge-balance-experimental.json')['sides']['left']
    added=[]
    for name in FAILED:
        known=[r for r in old if r['condition']==name]
        if any(r['safe'] and r['launch_pass'] for r in known):
            continue
        tracking=[dict(reference_until_s=stop,reference_gain=g)
                  for stop in (2.24,2.72,3.12) for g in (.04,.12,.28,.55,.95)]
        if name=='edge_battery':
            tracking.extend(dict(positive_catch=9.,recipe=dict(catch_s=.10,roll_deg=40.,y_mm=-24.,
                            pump_hz=2.55,catch_roll_deg=-38.,catch_hold_s=hold),
                            feedback=dict(trigger_deg=trigger))
                            for hold in (.055,.115,.22,.33,.45) for trigger in (17.,21.,25.))
            tracking.extend(dict(recipe=dict(catch_hold_s=hold,catch_roll_deg=-39.,catch_s=.09),
                                 feedback=dict(trigger_deg=trigger))
                            for hold in (.07,.14,.30,.45) for trigger in (15.,20.,24.))
        if name=='edge_noise':
            for p in tracking:
                p['imu_tau_s']=.1
        controls=batch([(name,p,{},5.8) for p in tracking],n)
        added.extend(controls)
        write(WORK/'lift-refined.json',added)
        for gen in range(3):
            rows=known+[r for r in added if r['condition']==name]
            if any(r['launch_pass'] and r['safe'] for r in rows):
                break
            def score(r):
                return r.get('near_hold_s',0.)+.002*min(30.,r['max_signed_roll'])
            parents=sorted((r for r in rows if 'reference_until_s' not in r['params']),
                           key=score,reverse=True)[:5]
            settings=[]
            for j in range(32):
                p=deepcopy(parents[j%5]['params'])
                scale=.72**gen
                for key,sd,low,high in [('catch_s',.012,.045,.3),('catch_hold_s',.032,.09,.4),
                                        ('pitch_deg',.8,-9,3),('x_mm',1.1,2.,17.)]:
                    recipe=p.setdefault('recipe',{})
                    recipe[key]=float(np.clip(rng.normal(recipe.get(key,policy['recipe'][key]),sd*scale),low,high))
                f=p.setdefault('feedback',{})
                for key,sd,low,high in [('trigger_deg',1.1,21.,31.),('launch_return_x_after_s',.044,.01,.38),
                                        ('capture_blend_s',.019,.04,.22)]:
                    f[key]=float(np.clip(rng.normal(f.get(key,policy['feedback'][key]),sd*scale),low,high))
                if j%4==0:
                    f['prebalance_pitch_at_deg']=float(rng.uniform(13.,24.))
                    f['prebalance_pitch_gain']=float(rng.uniform(.35,.95))
                settings.append(p)
            group=batch([(name,p,{},5.8) for p in settings],n)
            added.extend(group)
            write(WORK/'lift-refined.json',added)
            print(name,gen,sum(r['launch_pass'] and r['safe'] for r in group),max(score(r) for r in group),flush=True)
    freeze(old+added,read(WORK/'baseline.json'))


def course(n):
    lift=read(WORK/'lift.json')
    orbits=[{},dict(period=10),dict(period=18),dict(period=22),dict(period=12,mix=.8)]
    orbits.extend(dict(yaw=y) for y in (-24,-12,12,24))
    orbits.extend(dict(yaw_wave=y) for y in (-20,20))
    orbits.extend(dict(period=16,skew=k) for k in (-.45,.45))
    orbits.extend(dict(shake_deg=a,shake_hz=f) for a in (6.,15.) for f in (.55,1.,1.7))
    orbits.extend(dict(period=16.,shake_deg=sign*10.,shake_hz=f,harmonic=.35,
                       nutation=1.,nutation_phase=phase)
                  for sign in (-1,1) for f in (.55,1.1) for phase in (-90,90))
    rows=batch([('edge_nominal',lift,o,32.) for o in orbits],n)
    write(WORK/'circle-search.json',rows)
    best=max(rows,key=orbit_score)
    write(POLICY,dict(lift=lift,orbit=best['orbit'],model='configs/motors-speed-trained.json',waist_target_rad=0.))
    print('best',best['orbit'],best.get('loop'),flush=True)


def qualify(n):
    p=read(POLICY)
    rows=batch([(name,p['lift'],p['orbit'],36.) for name in ('edge_nominal',*FAILED,*FRESH)],n)
    write(WORK/'checks.json',rows)
    if p['orbit']:
        reference_rows=batch([(name,p['lift'],{},36.) for name in FAILED],n)
    else:
        reference_rows=[r for r in rows if r['condition'] in FAILED]
    write(WORK/'matched-reference.json',reference_rows)
    matched=max((r for r in rows if r['condition'] in FAILED),key=orbit_score)
    long=batch([(name,p['lift'],p['orbit'],75.) for name in ('edge_nominal',matched['condition'])],2)
    write(WORK/'long.json',long)
    development=max(read(WORK/'circle-finalists.json'),key=orbit_score)
    best=max(rows+long+[development]+reference_rows,key=orbit_score)
    replay=evaluate((best['condition'],p['lift'],best['orbit'],best['seconds'],WORK/'best.json'))
    if replay!=best:
        raise ValueError('Replay differs from qualification')
    print([(r['condition'],r['launch_pass'],r['circle_pass']) for r in rows],flush=True)


def refine_circle(n):
    p=read(POLICY)
    orbits=[dict(period=t,lean_trim=k) for t in (17.,20.,24.) for k in (.015,.045,.09)]
    orbits.extend(dict(period=t,head_axis=True,yaw=y,lean_trim=.045)
                  for t in (16.,20.) for y in (-4.,-1.5,1.5,4.))
    orbits.extend(dict(period=16.,head_axis=True,shake_deg=y,shake_hz=f,harmonic=.35,
                       lean_trim=.035) for y in (-4.,-1.5,1.5,4.) for f in (.45,1.2))
    rows=batch([('edge_nominal',p['lift'],o,35.) for o in orbits],n)
    write(WORK/'circle-refined.json',rows)
    best=max(read(WORK/'circle-search.json')+rows,key=orbit_score)
    write(POLICY,dict(p,orbit=best['orbit']))
    print(best['orbit'],best.get('loop'),flush=True)


def select_circle(n):
    policy=read(POLICY)
    previous=read(WORK/'circle-search.json')+read(WORK/'circle-refined.json')
    orbits=[r['orbit'] for r in previous if r['safe'] and r.get('loop')]
    orbits.extend(dict(period=t,lean_trim=k) for t in (15.,18.,22.) for k in (-.008,-.02,-.04))
    orbits.extend(dict(period=t,head_axis=True,yaw=y,lean_trim=-.015)
                  for t in (14.,17.) for y in (-4.,-1.8,1.8,4.))
    rows=batch([('edge_nominal',policy['lift'],o,38.) for o in orbits],n)
    write(WORK/'circle-finalists.json',rows)
    best=max(rows,key=orbit_score)
    if not best['circle_pass']:
        raise ValueError('No qualified replay among the finalists')
    write(POLICY,dict(policy,orbit=best['orbit']))
    print(best['orbit'],best['loop'],flush=True)


def replay():
    report=read('experiments/reports/motor-training-deep-lift.json')
    p=read(POLICY)
    best=report['featured']
    result=evaluate((best['condition'],p['lift'],best['orbit'],best['seconds'],WORK/'best.json'))
    if result!=best:
        raise ValueError('Frozen center-circle replay differs from the archived trial')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=('fit','repair','tune','refine','course','refine-circle','select-circle','qualify','replay'))
    parser.add_argument('--workers',type=int,default=7)
    args=parser.parse_args()
    if args.phase=='replay':
        replay()
    else:
        {'fit':fit,'repair':repair,'tune':tune,'refine':refine_capture,'course':course,
         'refine-circle':refine_circle,'select-circle':select_circle,'qualify':qualify}[args.phase](args.workers)
