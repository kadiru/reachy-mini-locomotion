"""Join metered refits to the frozen policies' recorded behavior tests."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


REPORTS = Path('experiments/reports')
RECEIPTS = REPORTS / 'compute-meter'
POLICIES = {'waist':'configs/waist-edge-sweep.json', 'spin':'configs/continuous-spin.json'}


def read(path):
    return json.loads(Path(path).read_text())


def build(rate, rate_basis, receipts=RECEIPTS):
    rows = []

    def add(model, file, count, scope, suite, flags, *, receipt=None, policy=None):
        flags = list(flags)
        row = dict(model=model, indexed_fit_rollouts=count, fit_scope=scope, test_suite=suite,
                   passed=sum(flags), tested=len(flags), success_fraction=sum(flags)/len(flags),
                   outcome_report='experiments/reports/'+file, policy_sha256=None,
                   training_backend='CPU MuJoCo', training_gpu='none', metered_host_cpu=None,
                   energy_evidence='historical energy unavailable', meter_receipt=None, energy_scope=None,
                   workers=None, simulated_rollout_s=None, wall_s=None, process_tree_cpu_s=None, mean_system_w=None,
                   peak_sample_system_w=None, baseline_system_w=None, gross_system_kwh=None,
                   above_baseline_kwh=None, rate_usd_kwh=rate, rate_basis=rate_basis,
                   gross_system_cost_usd=None, above_baseline_cost_usd=None)
        if receipt:
            path = receipts/(receipt+'.json')
            m = read(path)
            if not m['verified_refit'] or m['model']!=receipt or m['phase']!='training':
                raise ValueError('Invalid training receipt: '+str(path))
            if receipt in POLICIES:
                digest = hashlib.sha256(Path(POLICIES[receipt]).read_bytes()).hexdigest()
            else:
                digest = hashlib.sha256(json.dumps(policy, sort_keys=True).encode()).hexdigest()
            if m['frozen_policy_sha256'] != digest:
                raise ValueError('Meter receipt does not match the evaluated policy')
            if m['rollouts'] != count:
                raise ValueError('Refit trial count differs from the indexed experiment')
            row.update(simulated_rollout_s=m.get('simulated_rollout_s'), energy_evidence='metered refit', meter_receipt=str(path), energy_scope=m['scope'], policy_sha256=digest,
                       workers=m['workers'], metered_host_cpu=m['hardware']['cpu'], wall_s=m['wall_s'], process_tree_cpu_s=m['cpu_s'],
                       mean_system_w=m['gross']['mean_w'], peak_sample_system_w=m['gross']['peak_w'],
                       baseline_system_w=m['baseline']['mean_w'], gross_system_kwh=m['gross']['kwh'],
                       above_baseline_kwh=m['incremental_kwh'], gross_system_cost_usd=m['gross']['kwh']*rate,
                       above_baseline_cost_usd=m['incremental_kwh']*rate)
        if scope.startswith('evaluation only'):
            row.update(training_backend='reused policy', training_gpu='not applicable')
        rows.append(row)

    file = 'motor-training-continuous-spin.json'
    r = read(REPORTS/file)
    add('Continuous spin',file,sum(x['trials'] for x in r['training'].values()),
        'primary search through freeze','8 reserved surface/direction trials',
        (x['passed'] for x in r['held_out']), receipt='spin')
    file = 'motor-training-waist-balance.json'
    r = read(REPORTS/file)
    add('Active-waist edge sweep',file,sum(len(r[k]) for k in ('screen','fit','offsets','endurance')),
        'new sweep and finalists on existing edge regulator','8 reserved starts/conditions',
        (x['passed'] for x in r['challenges']),receipt='waist')
    file = 'motor-training-cadence-heading-repair-2026-11-21.json'
    r = read(REPORTS/file)
    add('Walking heading repair',file,len(r['fit'])+len(r['nominal']),
        'steering repair and finalist selection on existing gait','10 fresh course trials',
        (x['passed'] for x in r['fresh']),receipt='walk',policy=r['policy'])
    file = 'motor-training-slick-quarter.json'
    r = read(REPORTS/file)
    add('Slick-floor quarter-turn',file,sum(v['episodes'] for v in r['training'].values())+sum(r['repair_search'].values()),
        'indexed search and repair stages; other fit/retest work excluded','10 final held-out turns',
        (x['passed'] for x in r['held_out']))
    file = 'motor-training-rim-efficiency.json'
    r = read(REPORTS/file)
    add('Efficient edge pivot',file,sum(v['episodes'] for v in r['training'].values()),
        '7 indexed search stages; additional tuning excluded','95 mixed-condition turns',
        (x['passed'] for x in r['results']))
    file = 'motor-training-edge-artifacts-2026-11-20.json'
    r = read(REPORTS/file)
    add('Side-edge balance',file,sum(s['recorded_trials'] for s in r['searches']),
        '30 indexed searches; dynamics-identification cost unmetered','14 regular side-balance checks',
        (x['metrics']['final_five_seconds_pass'] for x in r['checks']
         if x['id'].startswith('final-') and 'feedback-disabled' not in x['id']))
    file = 'motor-training-foreaft-hold-qualification-2026-11-20.json'
    r = read(REPORTS/file)
    add('Front/back balance',file,None,'prior fit unmetered','18 raised-start checks',
        (x['metrics']['final_five_seconds_pass'] for x in r['candidates']))
    file = 'motor-training-direct-recovery-2026-11-21.json'
    r = read(REPORTS/file)
    add('Side-hit recovery',file,len(r['fit']),'tracking-limit comparison on existing balance controller',
        '12 held-out impacts for the selected limit',
        (x['passed'] for x in r['heldout'] if x['limit']==r['selected_limit']))
    file = 'motor-training-overhead-summary-2026-11-20.csv'
    with (REPORTS/file).open() as f:
        add('Overhead disturbance',file,None,'evaluation only; reuses balance policy',
            '40 matched disturbed trials; held edge',
            (x['final_hold']=='True' for x in csv.DictReader(f) if x['primary']=='True' and int(x['count'])>0))
    gyro_file = 'motor-training-gyro-peg.json'
    if (REPORTS/gyro_file).exists():
        report = read(REPORTS/gyro_file)
        def gyro(model, policy, stages, suite, flags):
            costs = [c for c in report['training_cost'] if c['stage'] in stages]
            meters = [read(c['receipt']) for c in costs]
            seconds = sum(c['wall_s'] for c in costs)
            energy = sum(c['gross_wh'] for c in costs)/1000
            incremental = sum(m['incremental_kwh'] for m in meters)
            add(model, gyro_file, sum(c['rollouts'] for c in costs), ' + '.join(stages), suite, flags)
            rows[-1].update(
                policy_sha256=report['sha256'][policy], metered_host_cpu=costs[0]['cpu'],
                workers=costs[0]['workers'], wall_s=seconds,
                simulated_rollout_s=sum(c['simulated_rollout_s'] for c in costs),
                process_tree_cpu_s=sum(c['cpu_s'] for c in costs),
                meter_receipt=str(REPORTS/gyro_file), energy_evidence='metered search, cumulative stages',
                energy_scope=meters[0]['scope'], gross_system_kwh=energy,
                mean_system_w=energy*3_600_000/seconds,
                peak_sample_system_w=max(m['gross']['peak_w'] for m in meters),
                baseline_system_w=sum(m['baseline']['mean_w']*m['wall_s'] for m in meters)/seconds,
                above_baseline_kwh=incremental, gross_system_cost_usd=energy*rate,
                above_baseline_cost_usd=incremental*rate)
        core = ['periodic orbit','tilt feedback']
        gyro('Gyro-peg rim roll','configs/gyro-peg-experimental.json',core,
             '8 reserved 24-second rim-roll trials',(r['passed'] for r in report['held_out']))
        gyro('Wide gyro-peg circle','configs/gyro-peg-wide-experimental.json',core+['wide floor circle'],
             '8 reserved two-minute traveling circles',
             (r['large_floor_pass'] for r in report['wide_tests'] if r['condition'] in report['held_out_conditions']))
        gyro('Deep gyro-peg roll','configs/gyro-peg-deep-experimental.json',
             ['deep edge reference','deep edge endurance','four-sector blend'],
             '4 reserved deep-roll conditions on selected launch side',
             (r['passed'] for r in report['deep_tests']['held_out']))
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rate-usd-kwh',type=float,required=True)
    p.add_argument('--rate-basis',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not 0 <= args.rate_usd_kwh < float('inf'):
        p.error('Rate must be finite and nonnegative')
    rows=build(args.rate_usd_kwh,args.rate_basis)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


if __name__=='__main__':
    main()
