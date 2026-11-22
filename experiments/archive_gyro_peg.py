"""Publish measurement receipts and both gyro-peg qualification suites."""
import hashlib
from pathlib import Path
from importlib.metadata import version

from fit_gyro_course import WIDE
from train_gyro_peg import WORK,POLICY,REPORT,CASES,read,write


def main():
    report=read(WORK/'peg-checks.json')
    stages={'periodic orbit':['screen','refine','finalists'],
            'tilt feedback':['phase-search','phase-refine','phase-finalists'],
            'deep edge reference':['traveling-edge'],'wide floor circle':['course-screen','course-endurance'],
            'deep edge endurance':['traveling-endurance'],'four-sector blend':['sector-fit']}
    receipts=('fit-energy','feedback-energy','edge-energy','course-energy','deep-energy','sector-energy')
    costs=[]
    for (label,inputs),filename in zip(stages.items(),receipts):
        raw=read(WORK/(filename+'.json'))
        if raw['status']!='complete' or raw['returncode']!=0:
            raise ValueError('Incomplete training power trace: '+filename)
        rows=[r for s in inputs for r in read(WORK/(s+'.json'))]
        path=Path('experiments/reports/compute-meter')/('gyro-peg-'+filename+'.json')
        write(path,raw)
        costs.append(dict(stage=label,rollouts=len(rows),simulated_rollout_s=sum(r['elapsed_s'] for r in rows),
                          receipt=str(path),gpu=None,cpu=raw['hardware']['cpu'],
                          workers=int(raw['command'][-1]),wall_s=raw['wall_s'],cpu_s=raw['cpu_s'],
                          gross_wh=raw['gross']['kwh']*1000,mean_system_w=raw['gross']['mean_w'],
                          usd=raw['gross_usd'],rate_usd_kwh=raw['rate_usd_kwh']))
    report.update(training_stages={s:read(WORK/(s+'.json')) for stage in stages.values() for s in stage},
                  primary_policy=read(POLICY), wide_policy=read(WIDE), initial_policy=read(WORK/'open-loop.json'),
                  deep_single_model=read(WORK/'deep-checks.json'),
                  deep_tests=read(WORK/'sector-checks.json'), deep_policy=read(WORK/'sector-frozen.json'),
                  deep_endurance_audit=read(WORK/'deep-audit.json'),
                  wide_tests=read(WORK/'course-checks.json'), contact_audit=read(WORK/'audit.json'),training_cost=costs,
                  held_out_conditions=list(CASES),nominal_foot_radius_mm=56.,simulator_version=version('mujoco'),
                  tariff_basis='Illustrative flat rate; household bill unconfirmed.',
                  training_backend='Mac CPU MuJoCo, zero training GPU-hours', hardware_verified=False)
    deep_path=Path('configs/gyro-peg-deep-experimental.json')
    write(deep_path,dict(**report['deep_policy'],waist_target_rad=0.,command_hz=50,
                         side_regulator='configs/edge-balance-experimental.json',
                         foreaft_regulator='configs/foreaft-balance-experimental.json'))
    report['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (POLICY,WIDE,deep_path,Path('configs/motors-speed-trained.json'),Path('configs/edge-balance-experimental.json'),Path('configs/foreaft-balance-experimental.json'))}
    write(REPORT,report)


if __name__=='__main__':
    main()
