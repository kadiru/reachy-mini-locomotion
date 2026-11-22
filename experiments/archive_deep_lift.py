"""Archive individual lift, orbit, and hanging-ball trials with the matching power traces."""
import hashlib
from pathlib import Path
from importlib.metadata import version

from train_deep_lift import WORK, POLICY, TEMPLATE, ATTITUDE, MODELS, FAILED, FRESH, choose_launch, read, write

REPORT=Path('experiments/reports/motor-training-deep-lift.json')


def main():
    specs=[('launch screen','lift-energy','lift-screen'),('launch timing','repair-energy','lift-repair'),
           ('reference handoff','tune-energy','lift-tune'),('capture refinement','refine-energy','lift-refined'),
           ('center circles','circle-energy','circle-search'),('circle refinement','circle-refine-energy','circle-refined'),
           ('qualified center laps','circle-final-energy','circle-finalists'),
           ('noisy orbit handoff','noise-roll-energy','noise-roll-fit'),
           ('reference trajectory and residual feedback','noise-trajectory-energy','noise-trajectory-fit')]
    stages={}
    costs=[]
    for stage,meter,table in specs:
        trials=read(WORK/f'{table}.json')
        stages[stage]=trials
        receipt=read(WORK/f'{meter}.json')
        if receipt['status']!='complete' or receipt['returncode']!=0:
            raise ValueError(f'Incomplete training receipt {meter}')
        path=Path('experiments/reports/compute-meter')/f'deep-lift-{meter}.json'
        write(path,receipt)
        calibration_s=[read(WORK/'baseline.json')['elapsed_s']] if stage=='launch screen' else []
        if stage in ('reference handoff','capture refinement'):
            calibration_s=[4.]
        if stage in ('noisy orbit handoff','reference trajectory and residual feedback'):
            calibration_s=[44.]
        costs.append(dict(stage=stage,rollouts=len(trials)+len(calibration_s),
                          simulated_rollout_s=sum(r['elapsed_s'] for r in trials)+sum(calibration_s),
                          receipt=str(path),workers=int(receipt['command'][-1]),cpu=receipt['hardware']['cpu'],
                          wall_s=receipt['wall_s'],cpu_s=receipt['cpu_s'],gross_wh=1000*receipt['gross']['kwh'],
                          usd=receipt['gross_usd']))
    policy=read(POLICY)
    pendulum=read(WORK/'pendulum-checks.json')
    prior_policy=read(WORK/'pre-noise-policy.json')
    for row in pendulum:
        s=row['settings']
        clip=read(WORK/f"pendulum-{s['side']}-{s['release_deg']}.json")
        trace=clip['trace']
        # This fixture uses the zero-noise nominal model. At decision time the
        # previous regulator output contains the attitude from two frames back.
        prefix=[trace[i-2][k] for i,v in enumerate(trace) if .9<=v['time_s']<=1.18 for k in ('roll_deg','pitch_deg')]
        chosen=choose_launch(prefix,policy['lift']['library'])
        if (chosen!=choose_launch(prefix,prior_policy['lift']['library']) or chosen or
                policy['orbit']!=prior_policy['orbit'] or row!=clip['metrics']):
            raise ValueError('Recorded pendulum trial does not use the frozen nominal controller')
    report=dict(evidence='MuJoCo simulation',mujoco=version('mujoco'),hardware_verified=False,
                backend='Apple M4 CPU',training_gpu=None,waist_target_rad=0.,control_hz=50,tilt_cutoff=None,
                conditions={k:MODELS[k] for k in ('edge_nominal',*FAILED,*FRESH,'lift_noise_8715','lift_noise_9715','lift_mixed_noise')},
                repaired=list(FAILED),fresh=list(FRESH),training=stages,training_cost=costs,
                checks=read(WORK/'checks.json'),original_orbit_with_new_lift=read(WORK/'matched-reference.json'),
                before_noisy_orbit_repair=read(WORK/'pre-noise-checks.json'),
                additional_noise_seeds=read(WORK/'extra-noise-checks.json'),
                endurance=read(WORK/'long.json'),featured=read(WORK/'best.json')['metrics'],
                pendulum=pendulum,pendulum_bracket=read(WORK/'pendulum-grazing.json'),policy=policy,
                sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                        (POLICY,TEMPLATE,ATTITUDE,Path('configs/motors-speed-trained.json'),
                         Path('configs/edge-balance-experimental.json'),Path('configs/foreaft-balance-experimental.json'))})
    write(REPORT,report)


if __name__=='__main__':
    main()
