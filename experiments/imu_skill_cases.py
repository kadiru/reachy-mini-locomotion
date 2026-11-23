"""Task-specific measures for the frozen IMU comparison."""
from functools import lru_cache
import math

from imu_frequency_study import cfg, metric

GROUPS = {
    'walking': ('shuffle', 'large rocking', 'sustained steps', 'distance per swing', 'teleop cadence',
                'edge braking', 'encoder edge walking', 'alternating edge walking'),
    'turning': ('target pivot','efficient pivot','slick quarter turn','fast spin','rim spin','waist sweep',
                'gyro peg','wide gyro peg'),
    'terrain': ('deep lift and circle','pendulum','slope launch','overhead drop','40 degree catch',
                'left side regulator','right side regulator','side shell rocking')
}


@lru_cache(maxsize=1)
def sway():
    from reachy_locomotion.sway import SwaySimulator
    return SwaySimulator('experiments/upstream-reachy-mini', cfg('motors-speed-trained'))


def walk(name, rep):
    from reachy_locomotion.sway import Gait
    side = (1, -1)[rep]
    if name in ('shuffle', 'large rocking'):
        from reachy_locomotion.rocking import rocking_metrics
        variant = 'faster-trained' if name == 'shuffle' else 'rocking-experimental'
        sensor = cfg('heading-sensor-assumed')
        sensor['seed'] += rep
        gait = Gait(**cfg('sway-' + variant))
        cycles = 24 if name == 'shuffle' else math.ceil(20 / (round(50/gait.frequency_hz)/50) - 1e-10)
        row = sway().run(gait, cycles=cycles, warmup_cycles=3, feedback=cfg('heading-'+variant), sensor=sensor)
        m = rocking_metrics(row)
        passed = m['course_ok'] if name == 'shuffle' else m['course_ok'] and m['style_ok']
        return metric(passed, 1000*row['measurement_displacement_in_startup_frame_m'][0]/max(.02,row['measured_seconds']),
                      'mm/s',heading=m['cycle_heading_deg'])
    if name in ('sustained steps','first penguin','faster penguin','feedback penguin'):
        from train_sustained_penguin import development, make_job, run
        config = {'sustained steps':'penguin-sustained-trained','first penguin':'penguin-feedback-experimental',
                  'faster penguin':'penguin-faster-trained','feedback penguin':'penguin-faster-feedback-trained'}[name]
        job = make_job(development()[0], cfg(config), seconds=26, cycles=1)
        job.update(side=side,finish_settle_s=1.)
        row = run(job)
        return metric(row['passed'],row['penguin']['forward_mm'],'mm')
    if name == 'distance per swing':
        from train_swing_distance import trial
        row = trial((name,cfg('head-swing-distance-trained'),cfg('motors-speed-trained'),side,12,cfg('heading-sensor-assumed'),None))
        return metric(row['passed'], row['mm_per_swing'], 'mm/swing')
    from train_cadence import rollout
    policy = {'teleop cadence':'teleop-cadence-trained','edge braking':'cadence-fast-edge-experimental',
              'encoder edge walking':'cadence-encoder-edge-experimental', 'alternating original':'cadence-alternating-experimental',
              'alternating edge walking':'cadence-alternating-repair-experimental'}[name]
    row = rollout((name,cfg(policy),side,12.,cfg('motors-speed-trained'),None))
    return metric(row['passed'],row['mm_per_active_s'],'mm/s')


def turn(name, rep):
    side = (1, -1)[rep]
    if name == 'target pivot':
        from train_target_pivot import run
        row = run((cfg('target-edge-pivot')['options'], 30*side, 'nominal',False,None))
        return metric(row['passed'],side * row['torso_deg'],'signed turn deg')
    if name in ('efficient pivot','slick quarter turn'):
        if name == 'efficient pivot':
            from train_rim_efficiency import run
            row = run((cfg('efficient-rim-pivot')['options'],90*side,'nominal','efficient',None))
        else:
            from train_slick_quarter_turn import guide_trial
            row = guide_trial((cfg('slick-quarter-pivot'),90*side,'slick',None))
        return metric(row['passed'],side * row['torso_deg'],'signed turn deg',strokes=row['strokes'])
    if name == 'waist sweep':
        from train_waist_balance import trial, job
        p = cfg('waist-edge-sweep')
        r = trial(job(side,p['amplitude'],p['period'],gain_change=p['gain_change'],com_offset_mm=p['com_offset_mm'],audit=True))
        return metric(r['passed'],r['same_edge_s'],'loaded s')
    if name in ('gyro peg','wide gyro peg'):
        from train_gyro_peg import trial
        p = cfg('gyro-peg-wide-experimental' if name.startswith('wide') else 'gyro-peg-experimental')
        duration = 120. if name.startswith('wide') else 24.
        row = trial((p['orbit'],'nominal',side,duration,side*p['foot_winding_for_positive_mirror']),course=name.startswith('wide'))
        return metric(row['large_floor_pass'] if name.startswith('wide') else row['passed'],row['longest_rim_turns'],'rim turns')
    from train_continuous_spin import trial
    p = cfg('continuous-spin' if name == 'fast spin' else 'continuous-rim-spin')
    r = trial((p['orbit'],'nominal',20.,side,side*p['positive_mirror_direction'],None))
    # Shortened screen scales the original two-revolution/minute displacement floor.
    passed = (r['termination']=='completed' and r['joint_margin_rad']>=0 and r['rpm']>=2.
              and r['last_half_deg_s']>0 and r['max_excursion_mm']<=56.
              and r['forward_window_fraction']>=.98 and r['forward_frame_fraction']>=.8)
    if name == 'rim spin':
        passed = passed and r['edge_fraction']>=.30
    return metric(passed,r['rpm'],'rpm',loaded=r['edge_fraction'])


def terrain(name, rep):
    side = (1,-1)[rep]
    if name in ('left side regulator','right side regulator'):
        from train_edge_balance import evaluate
        edge = name.split()[0]
        p = cfg('edge-balance-experimental')['sides'][edge]
        if rep:
            p['feedback']['sensor_seed'] = 91304
        r = evaluate(dict(id=name,policy=p,side=1 if edge=='right' else -1,dt=.000125,seconds=10.))
        return metric(r['metrics']['final_five_seconds_pass'],r['metrics']['longest_strict_hold_s'],'hold s')
    if name == 'slope launch':
        from slope_balance import run, spec
        row = run(spec(9.,1,cfg('slope-balance')['sides']['1'], seconds=8.,seed=55001+rep))
        return metric(row['strict'],row['soft_s'],'hold score s')
    if name == 'pendulum':
        from test_deep_pendulum import swing
        row = swing((30.,side,cfg('deep-lift-circle'),False))
        return metric(row['recovered'],row['final_lean_deg'],'lean deg',hits=len(row['hits']))
    if name == 'deep lift and circle':
        from train_deep_lift import evaluate
        p = cfg('deep-lift-circle')
        row = evaluate((('edge_nominal','edge_battery')[rep],p['lift'],p['orbit'],36.))
        rim = row.get('rim',{})
        turns = rim.get('longest_rim_turns',0.)
        return metric(row['safe'] and row['launch_pass'] and turns>=1.,turns,'rim turns')
    if name == 'side shell rocking':
        from tilted_release import run
        row = run(dict(experiment='side',side=side,angle=90,mode='loop',amplitude=.5,setup=(-20,18),
                       seconds=15.,drive_s=8.,cycles=4))
        low, high = row['range_axis_rotation_deg']
        return metric(row['termination']=='completed' and 'head' not in row['touch'] and high-low>=30.,
                      high-low,'rock arc deg',head_touch='head' in row['touch'])
    if name == '40 degree catch':
        from probe_high_tilt import run
        case = dict(initial_head=[-40,25,-12],first=[35,-5],second=[-30,20],rise=.065,switch=.3,
                    return_time=.18,neutral_at=26,side=(-1,1)[rep],angle=(39.95,39.99)[rep],
                    rate=(3.,2.)[rep],dt=.000125,trigger_at=40.)
        r = run(case)
        return metric(r['completed'] and r['settled'] and r['trigger_s'] is not None,
                      r['peak_tilt_deg'],'peak deg',trigger_s=r['trigger_s'],termination=r['termination'])
    from train_balance_robustness import evaluate
    edge = ('left','right')[rep]
    p = cfg('balance-recovery-experimental')['edges'][edge]
    ball = dict(drop_height_m=.5,offset_x_m=0.,offset_y_m=0.,launch_s=5.,radius_m=.015,mass_kg=.010)
    row = evaluate(dict(id=name,edge=edge,policy=p,dt=.000125,seconds=12.,tilt_limit_deg=None,
                        observe_landing=True,balls=[ball]))
    hit = row['result']['ball_disturbances'][0]['contacted_robot']
    return metric(hit and row['metrics']['final_five_seconds_pass'],
                  row['metrics']['longest_strict_hold_s'],'hold s',hit=bool(hit))


OPEN_LOOP = ('first sway candidate','motor limited open gait','open progress fast','open progress conservative')


HISTORICAL = ('heading gait','speed gait','first penguin','faster penguin','feedback penguin',
              'alternating original','initial deep gyro','bilateral catch','fall primitive','catch primitive','pivot primitive')


def historical(name, rep):
    if name in ('first penguin','faster penguin','feedback penguin','alternating original'):
        return walk(name,rep)
    if name in ('heading gait','speed gait'):
        from reachy_locomotion.sway import SwaySimulator, Gait
        from reachy_locomotion.rocking import rocking_metrics
        old = name == 'heading gait'
        motor = cfg('motors-datasheet' if old else 'motors-speed-trained')
        # Use the archived fine timestep for every arm so 1 kHz is representable.
        motor['timestep_s'] = .001
        gait = Gait(**cfg('sway-heading-trained' if old else 'sway-speed-trained'))
        sensor = cfg('heading-sensor-assumed')
        sensor['seed'] += rep
        sim = SwaySimulator('experiments/upstream-reachy-mini',motor)
        row = sim.run(gait,cycles=38,warmup_cycles=3,sensor=sensor,
                      feedback=cfg('heading-trained' if old else 'heading-speed-trained'))
        return metric(rocking_metrics(row)['course_ok'],1000*row['measurement_displacement_in_startup_frame_m'][0]
                      / max(.02,row['measured_seconds']),'mm/s')
    if name == 'initial deep gyro':
        from fit_traveling_edge import trial
        p = cfg('gyro-peg-deep-experimental')
        job = (p['side'],p['period'],p['direction'],p['gain'],('edge_nominal','edge_battery')[rep],p['seconds'])
        row = trial(job,sector_mix=p['sector_mix'])
        return metric(row['passed'],row['metrics']['longest_rim_turns'],'rim turns')
    if name == 'bilateral catch':
        from train_balance import evaluate
        p = cfg('balance-recovery-trained')
        r = evaluate((name,p['recipe'],p['feedback'],(1,-1)[rep],0,.00025,8.))['result']
        return metric(r['recovered'],r['peak_roll_deg'],'peak roll deg')
    from train_primitives import evaluate
    from qualify_primitives import pivot_metrics
    kind = name.split()[0]
    r = evaluate((name,cfg('primitive-'+kind+'-experimental'),('off','hold_neutral')[rep],.00025,8.))['result']
    passed = {'fall':r['fell'], 'catch':r['recovered'] and r['catch_trigger_s'] is not None,
              'pivot':pivot_metrics(r)['passed']}[kind]
    return metric(passed,r['peak_roll_deg'],'peak roll deg')


def motion_case(name, repeat):
    if name in OPEN_LOOP:
        from reachy_locomotion.sway import SwaySimulator, Gait
        from reachy_locomotion.rocking import rocking_metrics
        idx = OPEN_LOOP.index(name)
        motor = cfg('sway-cad-proxy' if idx == 0 else 'motors-datasheet')
        motor['timestep_s'] = (.001,.0005)[repeat]
        gait = cfg(('sway-candidate','sway-trained','sway-progress-fast','sway-progress-conservative')[idx])
        sim = SwaySimulator('experiments/upstream-reachy-mini',motor)
        row = sim.run(Gait(**gait),cycles=24,warmup_cycles=3)
        return metric(rocking_metrics(row)['course_ok'],1000*row['measurement_displacement_in_startup_frame_m'][0]
                      / max(.02,row['measured_seconds']),'mm/s')
    if name in HISTORICAL:
        return historical(name,repeat)
    if name in GROUPS['walking']:
        return walk(name,repeat)
    if name in GROUPS['turning']:
        return turn(name,repeat)
    return terrain(name,repeat)
