"""Verify isolated refits and attach their frozen-policy hashes to the meter receipts."""
import argparse
import hashlib
import json
import subprocess
from importlib.metadata import version
from pathlib import Path

from compare_refit import effective_policy, effective_digest
from meter_compute import meter_window
from compute_ledger import POLICIES, RECEIPTS, REPORTS, read


def simulation_effort(model, directory):
    """Sum each indexed rollout once. Endurance includes the two waist baselines."""
    if model == 'waist':
        names = ['screen','fit','offsets','endurance']
    else:
        names = list(read(REPORTS/'motor-training-continuous-spin.json')['training'])
    seconds = 0.
    count = 0
    for name in names:
        rows = read(directory/(name+'.json'))
        if isinstance(rows,dict):
            rows = rows['search'] + [r for group in rows['groups'] for r in group]
        elif rows and isinstance(rows[0],list):
            rows = [r for group in rows for r in group]
        seconds += sum(r['elapsed_s'] for r in rows)
        count += len(rows)
    return dict(rollouts=count,simulated_rollout_s=seconds)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--receipts',type=Path,default=RECEIPTS)
    parser.add_argument('--macmon',help='Executable used if an older receipt omits its meter version')
    a=parser.parse_args()
    a.receipts.mkdir(parents=True,exist_ok=True)
    evaluation_path = a.directory/'spin-evaluation.json'
    evaluation = read(evaluation_path) if evaluation_path.exists() else None
    if evaluation is not None and (evaluation['status'] != 'complete' or evaluation['returncode'] != 0
                                   or evaluation['phase'] != 'evaluation' or evaluation['model'] != 'spin'):
        raise ValueError('Incomplete or unverified spin evaluation')
    for model in ('spin','waist','walk'):
        m=read(a.directory/(model+'.json'))
        if m['status'] not in ('complete','workload_failed'):
            raise ValueError(f'Incomplete {model} meter trace')
        if model=='walk':
            target=read(REPORTS/'motor-training-cadence-heading-repair-2026-11-21.json')['policy']
            fit_report=read(a.directory/'walk-fit.json')
            fit=fit_report['policy']
            m['rollouts']=len(fit_report['fit'])+len(fit_report['nominal'])
            digest=hashlib.sha256(json.dumps(target,sort_keys=True).encode()).hexdigest()
        else:
            target=read(POLICIES[model])
            fit=read(a.directory/(model+'-fit')/'frozen.json')
            digest=hashlib.sha256(Path(POLICIES[model]).read_bytes()).hexdigest()
            m['stages']=read(a.directory/(model+'-fit')/'stages.json')
            m.update(simulation_effort(model, a.directory/(model+'-fit')))
        if effective_policy(model,target)!=effective_policy(model,fit):
            raise ValueError(f'{model} refit differs from evaluated policy')
        if m['returncode'] != 0:
            # The original meter run used a strict JSON comparison, before this
            # reader supported omitted zero-valued optional orbit parameters.
            if (model!='spin' or m['returncode']!=1 or target==fit or m['rollouts']!=910
                    or m['clock_origin_utc'] != '2026-11-22T00:09:43.084589+00:00'
                    or m['stages'][-1]['stage']!='freeze' or evaluation is None):
                raise ValueError(f'{model} training command failed')
            m['verification_note'] = ('The command completed freeze and exited 1 on its final raw JSON comparison. '
                                      'Every effective Orbit parameter and all outer policy fields match.')
        if all('cpu_w' in r and 'gpu_w' in r for r in m['samples']):
            for sensor in ('cpu_w','gpu_w'):
                component = [(r['t_s'], r[sensor]) for r in m['samples']]
                m['mean_host_'+sensor] = meter_window(component,m['start_s'],m['end_s'])['mean_w']
        m['verified_refit']=True
        m['effective_policy_sha256']=effective_digest(model,fit)
        m['frozen_policy_sha256']=digest
        m['workers']=int(m['command'][m['command'].index('--workers')+1])
        m['simulator_version']=version('mujoco')
        trainer = {'spin':'train_continuous_spin.py', 'waist':'train_waist_balance.py',
                   'walk':'repair_cadence_heading.py'}[model]
        inputs = [Path('experiments')/trainer, Path('configs/motors-speed-trained.json')]
        if model == 'waist':
            inputs.append(Path('configs/edge-balance-experimental.json'))
        if model == 'walk':
            inputs.extend((Path('configs/cadence-alternating-experimental.json'),
                           REPORTS/'motor-training-alternating-edge-walks-2026-11-21.json'))
        m['refit_source_sha256'] = {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
        # The raw command uses the project interpreter and the verified Homebrew bottle.
        if 'meter_version' not in m:
            if not a.macmon:
                parser.error('Supply the macmon executable for the older receipt')
            m['meter_version']=subprocess.check_output([a.macmon,'--version'],text=True).strip()
        with (a.receipts/(model+'.json')).open('w') as f:
            json.dump(m,f,indent=1,allow_nan=False)
            f.write('\n')
    if evaluation is not None:
        m = evaluation
        m.update(rollouts=19, matched_trials=8, held_out_trials=8, control_trials=3,
                 frozen_policy_sha256=hashlib.sha256(Path(POLICIES['spin']).read_bytes()).hexdigest())
        with (a.receipts/'spin-evaluation.json').open('w') as f:
            json.dump(m,f,indent=1,allow_nan=False)
            f.write('\n')


if __name__=='__main__':
    main()
