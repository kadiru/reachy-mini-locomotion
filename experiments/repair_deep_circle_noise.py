"""Learn a bounded attitude correction for the noisy handoff into sustained rim rolling."""
import argparse
from copy import deepcopy

from reachy_locomotion.primitives import Primitive
from train_deep_lift import (ATTITUDE, POLICY, WORK, LiftCircle,
                             batch, evaluate, orbit_score, read, simulator, write)

def fit(workers):
    policy=read(POLICY)
    write(WORK/'pre-noise-policy.json',policy)
    sim=simulator('edge_nominal')
    ctl=LiftCircle(sim,'edge_nominal',policy['lift'],policy['orbit'])
    real=sim.run_primitive(Primitive(),seconds=44.,waist_mode='hold_neutral',controller=ctl,tilt_limit_deg=None)
    if not real['completed'] or ctl.start is None:
        raise ValueError('Nominal attitude reference did not complete')
    write(ATTITUDE,dict(period_s=ctl.period,capture_s=ctl.start,
                        rpy=[[r['roll_deg'],r['pitch_deg']] for r in real['trace']]))
    def with_width(width):
        copy=deepcopy(policy['lift'])
        for entry in copy['library']:
            if entry['noise_example']:
                entry['params']['prior_halfwidth_deg']=width
        return copy
    widths=(.025,.05,.075,.10,.15,.20,.30,.45)
    short=batch([('edge_noise',with_width(w),policy['orbit'],11.) for w in widths],workers)
    finalists=[w for w,r in zip(widths,short) if r['safe'] and r['launch_pass']]
    if not finalists:
        finalists=[widths[max(range(len(short)),key=lambda i:short[i]['elapsed_s'])]]
    long=batch([('edge_noise',with_width(w),policy['orbit'],36.) for w in finalists],workers)
    good=[w for w,r in zip(finalists,long) if r['circle_pass'] and r['launch_pass']]
    write(WORK/'noise-roll-fit.json',short+long)
    if good:
        policy['lift']=with_width(min(good))
        policy['attitude_prior']=str(ATTITUDE)
        write(POLICY,policy)
    print('passing widths',good,flush=True)


def fit_trajectory(workers):
    policy=read(WORK/'pre-noise-policy.json')
    sim=simulator('edge_nominal')
    ctl=LiftCircle(sim,'edge_nominal',policy['lift'],policy['orbit'])
    poses=[]
    target=ctl.target
    def record(t,roll):
        result=target(t,roll)
        poses.append(result[0].tolist())
        return result
    ctl.target=record
    real=sim.run_primitive(Primitive(),seconds=44.,waist_mode='hold_neutral',controller=ctl,tilt_limit_deg=None)
    prior=read(ATTITUDE)
    if prior['rpy']!=[[r['roll_deg'],r['pitch_deg']] for r in real['trace']]:
        raise ValueError('Head targets and attitude reference differ in replay')
    write(ATTITUDE,dict(prior,head_pose=poses))
    def settings(width,gain):
        lift=deepcopy(policy['lift'])
        for entry in lift['library']:
            if entry['noise_example']:
                entry['params'].update(prior_halfwidth_deg=width,residual_gain=gain)
        return lift
    pairs=[(w,g) for w in (.045,.08,.14,.22) for g in (.35,1.)]
    short=batch([('edge_noise',settings(w,g),policy['orbit'],11.) for w,g in pairs],workers)
    finalists=[pair for pair,r in zip(pairs,short) if r['safe'] and r['launch_pass']][:4]
    checks=batch([('edge_noise',settings(w,g),policy['orbit'],36.) for w,g in finalists],workers) if finalists else []
    write(WORK/'noise-trajectory-fit.json',short+checks)
    winners=[pair for pair,r in zip(finalists,checks) if r['circle_pass'] and r['launch_pass']]
    if winners:
        policy['lift']=settings(*min(winners))
        policy['attitude_prior']=str(ATTITUDE)
        write(POLICY,policy)
    print('qualifying feedback settings',winners,flush=True)


def verify(workers):
    policy=read(POLICY)
    baseline_path=WORK/'pre-noise-checks.json'
    if baseline_path.exists():
        old=read(baseline_path)
    else:
        old=read(WORK/'checks.json')
        write(baseline_path,old)
    # The other acquisition branches and the orbit are exactly identical.
    changed=[r['condition'] for r in old if 'imu_tau_s' in r['params']]
    names=changed+['lift_noise_8715','lift_noise_9715','lift_mixed_noise']
    checks=batch([(c,policy['lift'],policy['orbit'],36.) for c in names],workers)
    write(WORK/'extra-noise-checks.json',checks[len(changed):])
    revised={r['condition']:r for r in checks[:len(changed)]}
    write(WORK/'checks.json',[revised.get(r['condition'],r) for r in old])
    old_best=read(WORK/'best.json')['metrics']
    eligible=[r for r in checks if r['circle_pass']]
    if eligible and orbit_score(max(eligible,key=orbit_score))>orbit_score(old_best):
        best=max(eligible,key=orbit_score)
        replay=evaluate((best['condition'],policy['lift'],best['orbit'],best['seconds'],WORK/'best.json'))
        if replay!=best:
            raise ValueError('Noisy replay differs from its archived qualification')
    print([(r['condition'],r['launch_pass'],r['circle_pass']) for r in checks],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=('fit','trajectory','verify'))
    parser.add_argument('--workers',type=int,default=6)
    args=parser.parse_args()
    {'fit':fit,'trajectory':fit_trajectory,'verify':verify}[args.phase](args.workers)
