"""Replay the moving-table catch using only 50 Hz IMU packets for attitude and rate."""
import argparse
import hashlib
import math
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from moving_table import POLICY, REPORT, SEEDS, case, read, simulate, write
from qualify_moving_table import edge_pass
from reachy_locomotion.geometry import euler, rotation

OUTPUT = REPORT.with_name('moving-table-imu.json')
DT = .02
GRAVITY = 9.80665


def counts(values, scale):
    return np.clip(np.rint(values * scale), -32768, 32767) / scale


def supported_quaternion(up):
    # During the supported setup the accelerometer sees only gravity.
    x, y, z = up
    roll = math.atan2(y, z)
    pitch = math.atan2(-x, math.hypot(y, z))
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation((roll,pitch,0.)).ravel())
    return quat


class IMUPackets:
    def __init__(self, mode):
        if mode not in ('reference','gyro_only','raw','handoff'):
            raise ValueError('Unknown IMU experiment mode')
        from ahrs.filters import Madgwick

        self.mode = mode
        self.filter = Madgwick()
        self.q = None
        self.trace = []

    def channels(self, model, data):
        acceleration_g = data.sensor('balance_accelerometer').data / GRAVITY
        return acceleration_g, np.degrees(data.sensor('balance_gyro').data)

    def sample(self, model, data, t):
        specific_force, angular_rate = self.channels(model,data)
        acceleration = counts(specific_force,10920.)
        gyro = np.deg2rad(counts(angular_rate,65.536))
        if self.q is None:
            mounting = data.site('balance_imu').xmat.reshape(3,3)
            static_sample = counts(mounting.T @ -model.opt.gravity / GRAVITY, 10920.)
            self.q = supported_quaternion(static_sample)
        else:
            self.q = self.filter.updateIMU(self.q, gyr=gyro, acc=acceleration, dt=DT)
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix,self.q)
        tilt = np.degrees(euler(matrix.reshape(3,3))[:2])
        self.trace.append([t, *tilt, *acceleration, *np.rad2deg(gyro)])
        if self.mode=='reference' or (self.mode=='handoff' and t<3.):
            return None,None
        return (None if self.mode=='gyro_only' else tilt),gyro


class FilteredPackets(IMUPackets):
    """Filter the physics-rate sensor stream and publish latched 100 Hz registers."""
    def __init__(self, age_ms):
        super().__init__('raw')
        if not 0 <= age_ms < 10:
            raise ValueError('Register age must be in the 10 ms update interval')
        self.age_s = age_ms * .001
        self.ticks = 0
        self.register = None

    def channels(self, model, data):
        if self.register is None:
            dt = model.opt.timestep
            self.period = round(.01 / dt)
            self.phase = (-round(self.age_s / dt)) % self.period
            self.raw_acc = data.sensor('balance_accelerometer').data
            self.raw_gyro = data.sensor('balance_gyro').data
            mounting = data.site('balance_imu').xmat.reshape(3, 3)
            self.acc = mounting.T @ -model.opt.gravity
            self.rate = np.zeros(3)
            self.register = (self.acc / GRAVITY, np.degrees(self.rate))
            self.alpha_acc = -math.expm1(-2 * math.pi * 40 * dt)
            self.alpha_gyro = -math.expm1(-2 * math.pi * 12 * dt)
        return self.register

    def advance(self):
        # Physical ranges before the bandwidth model, numerical integration at physics rate.
        self.acc += self.alpha_acc * (np.clip(self.raw_acc, -3 * GRAVITY, 3 * GRAVITY) - self.acc)
        limit = math.radians(500.)
        self.rate += self.alpha_gyro * (np.clip(self.raw_gyro, -limit, limit) - self.rate)
        self.ticks += 1
        if self.ticks % self.period == self.phase:
            self.register = (self.acc / GRAVITY, np.degrees(self.rate))


def trial(spec):
    mode, flat, seed = spec
    job = case(1,10.,seed=seed,flat=flat)
    sensor = FilteredPackets(int(mode.removeprefix('filtered_'))) if mode.startswith('filtered_') else IMUPackets(mode)
    result, track, _, _ = simulate(job, recording=True, imu_observer=sensor)
    trace = np.asarray(sensor.trace)
    t, estimated = trace[:,0], trace[:,1]
    truth = np.array([r['roll'] for r in track])
    np.testing.assert_allclose(t, [r['t'] for r in track], atol=1e-9,rtol=0)
    error = (estimated-truth+180.)%360.-180.
    end = result['shell']['time_s'] if result['shell'] else result['elapsed']
    valid = t <= end
    out = dict(mode=mode, result=result, edge_pass=edge_pass(result),
               pre_landing_roll_rmse_deg=float(np.sqrt(np.mean(error[valid]**2))),
               pre_landing_roll_peak_error_deg=float(np.abs(error[valid]).max()),
               at_deck_start=dict(estimated_deg=float(estimated[150]),true_deg=float(truth[150])) if len(t)>150 else None)
    if seed==SEEDS[0]:
        out['trace_columns']=['time_s','body_roll_deg','imu_roll_deg','deck_deg','edge_loaded']
        out['trace']=[[r['t'],r['roll'],float(a),r['bank'],r['edge']] for r,a in zip(track,estimated)]
    return out


def study(workers, filtered=False):
    original = read(REPORT)
    digest = hashlib.sha256(POLICY.read_bytes()).hexdigest()
    if digest != original['policy_sha256']:
        raise ValueError('Reference study used different policy weights')
    conditions = [('reference',False),('gyro_only',False),('raw',False),('raw',True),('handoff',False)]
    if filtered:
        conditions = [(f'filtered_{age}',flat) for age in (0,5,9) for flat in (False,True)]
    jobs = [(mode,flat,seed) for mode,flat in conditions for seed in SEEDS]
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(trial,jobs))
    for row in rows:
        if row['mode']=='reference':
            expected = next(r for r in original['trials'] if r['job']==row['result']['job'])
            if row['result']!=expected:
                raise ValueError('Instrumented baseline failed to reproduce the archived trial')
    output = OUTPUT.with_name('moving-table-imu-bandwidth.json') if filtered else OUTPUT
    write(output,dict(policy_sha256=digest, seeds=SEEDS, workers=workers, trials=rows))
    for row in rows:
        r=row['result']
        print(row['mode'],r['job']['seed'],r['job']['flat'],r['start_ok'],row['edge_pass'],r['shell'],r['final_body_deg'])


def plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from pathlib import Path

    trials = read(OUTPUT)['trials'] + read(OUTPUT.with_name('moving-table-imu-bandwidth.json'))['trials']
    fig, axes = plt.subplots(4, 1, figsize=(9,8.5), sharex=True, constrained_layout=True)
    for ax,mode,title in zip(axes, ('reference','raw','handoff','filtered_5'),
                             ('Original controller inputs', '50 Hz IMU from release',
                              'Switch to 50 Hz IMU as the deck starts','Bandwidth model, 5 ms register age')):
        row = next(r for r in trials if r['mode']==mode and not r['result']['job']['flat'] and 'trace' in r)
        t,truth,estimate,bank,edge = np.asarray(row['trace']).T
        ax.plot(t,truth,color='#075c64',lw=2,label='Actual body roll')
        ax.plot(t,estimate,color='#c05915',lw=1.3,ls='--',label='IMU estimate')
        ax.plot(t,bank,color='#7c8492',lw=1,label='Deck angle')
        ax.axvline(3,color='#666666',lw=.8,ls=':')
        shell = row['result']['shell']
        if shell:
            at = shell['time_s']
            ax.scatter([at],[np.interp(at,t,truth)],marker='x',s=60,c='#a32424',label='Shell contact',zorder=4)
        outcome = 'held' if row['edge_pass'] else 'shell landed' if shell else 'settled flat' if row['result']['flat_return'] else 'missed hold'
        ax.set_title(f'{title}  /  {outcome}',loc='left',fontsize=11)
        ax.set_ylabel('Degrees')
        ax.grid(axis='y',alpha=.22)
    axes[0].legend(loc='best',ncol=3,fontsize=8)
    axes[-1].set_xlabel('Time since supported release (s)   ·   Deck motion starts at 3 s')
    fig.suptitle('Reachy Mini on the downhill right edge',fontsize=16)
    fig.savefig(Path('media/simulation/moving-table-imu.png'),dpi=140)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workers',type=int,default=5)
    p.add_argument('--bandwidth',action='store_true',help='Run the 100 Hz register-phase and bandwidth comparison')
    p.add_argument('--plot',action='store_true',help='Plot the archived first-seed traces')
    args=p.parse_args()
    if args.plot:
        plot()
    else:
        study(args.workers,args.bandwidth)
