"""Summarize the frozen November 20 ball curriculum without rerunning physics."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def main():
    root = Path('experiments/generated/ball-balance')
    output = Path('experiments/reports/motor-training-ball-artifacts-2026-11-20.json')
    chart = Path('media/simulation/ball-balance-recovery-2026-11-20-chart.png')
    if output.exists() or chart.exists():
        raise SystemExit('Preserve earlier evidence')
    qualification = read('experiments/reports/motor-training-ball-qualification-2026-11-20.json')
    selected = read('configs/balance-recovery-experimental.json')
    assert qualification['completed'] and len(qualification['candidates']) == 38
    summary = dict(evidence='uncalibrated_simulation', hardware_ready=False,
                   demonstrations={}, qualification={})
    groups = {
        'impact_trials': ('qual-', 'impact_trial_pass'),
        'hold_return_trials': ('qual-cycle-', 'hold_return_pass'),
        'moving_front_capture_trials': ('qual-moving-', 'hold_return_pass'),
    }
    for name, (prefix, key) in groups.items():
        rows = [c for c in qualification['candidates'] if c['id'].startswith(prefix)
                and (name != 'impact_trials' or c['impacts']['events'])]
        summary['qualification'][name] = dict(
            passed=sum(c['result']['balance_curriculum'][key] for c in rows), total=len(rows),
            checks=[dict(id=c['id'], passed=c['result']['balance_curriculum'][key],
                         termination=c['result']['termination'],
                         natural_start=c['result']['natural_upright_start']) for c in rows])

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for col, edge in enumerate(('left', 'right')):
        run = read(root / f'bilateral-{edge}.json')
        sign = -1 if edge == 'left' else 1
        trace = run['trace']
        t = np.array([x['time_s'] for x in trace])
        roll = sign * np.array([x['roll_deg'] for x in trace])
        rate = sign * np.array([x['roll_rate_deg_s'] for x in trace])
        target = selected['edges'][edge]['feedback']['target_roll_deg']
        assert run['natural_upright_start'] and run['balance_curriculum']['full_sequence_pass']
        assert run['balance_curriculum']['impact_trial_pass']
        summary['demonstrations'][edge] = {
            k: run[k] for k in ['completed', 'fell', 'settled', 'peak_tilt_deg',
                                'waist_peak_torque_nm', 'measured_waist_angle_range_deg',
                                'min_joint_margin_rad', 'max_closure_error_m',
                                'balance_curriculum', 'ball_disturbances']}
        for ax in axes[:, col]:
            ax.grid(alpha=.2)
            ax.set_xlabel('Simulated time (s)')
            for event in run['ball_disturbances']:
                color = '#d56812' if event['parameters']['from_side'] == 1 else '#007daa'
                ax.axvline(event['first_robot_contact_s'], color=color, lw=1.6,
                           label='Ball from left' if color == '#d56812' else 'Ball from right')
        axes[0, col].plot(t, roll, color='#222222', lw=1.4, label='Measured tilt')
        axes[0, col].axhspan(target-2, target+2, color='#238b45', alpha=.12)
        axes[0, col].axvline(run['balance_curriculum']['return_started_s'], color='#777777',
                           ls='--', label='Controlled return')
        axes[0, col].set(xlim=(0, 32), ylim=(-1, 35), ylabel='Tilt toward loaded edge (deg)',
                         title=edge.upper() + ' EDGE: natural lift, two impacts, return')
        axes[0, col].legend(fontsize=8, loc='lower center')
        axes[1, col].plot(t, rate, color='#222222', lw=1.1)
        axes[1, col].axhspan(-5, 5, color='#238b45', alpha=.12)
        extent = 5 * np.ceil(1.1 * np.max(np.abs(rate[(t >= 7.5) & (t <= 16)])) / 5)
        axes[1, col].set(xlim=(7.5, 16), ylim=(-extent, extent), ylabel='Outward roll rate (deg/s)',
                         title='Impact response; green band is the rate criterion')
    fig.suptitle('Physical rolling-ball recovery — uncalibrated MuJoCo simulation\n'
                 '8 cm diameter, 10 g, 0.3 m/s release; waist target 0°; unchanged motor limits', fontsize=12)
    fig.savefig(chart, dpi=160)
    plt.close(fig)

    baseline = read(root / 'bilateral-left.json')
    replay = read(root / 'cli-left-replay.json')
    summary['cli_replay'] = dict(
        trajectory_frames=len(replay['trajectory']),
        trajectory_exact_match=replay['trajectory'] == baseline['trajectory'],
        trace_exact_match=replay['trace'] == baseline['trace'],
        curriculum_exact_match=replay['balance_curriculum'] == baseline['balance_curriculum'])
    assert all(summary['cli_replay'][k] for k in ('trajectory_exact_match', 'trace_exact_match', 'curriculum_exact_match'))
    files = [*Path('experiments/reports').glob('motor-training-ball-*.json'),
             *Path('media/simulation').glob('ball-balance-recovery-2026-11-20*'),
             Path('configs/balance-recovery-experimental.json'),
             Path('experiments/reports/repository-tests-ball-balance-final-2026-11-20.txt'),
             Path('experiments/run_balance_curriculum.py'), Path(__file__),
             root / 'bilateral-left.json', root / 'bilateral-right.json', root / 'cli-left-replay.json']
    summary['artifacts'] = {str(p): dict(bytes=p.stat().st_size,
                                       sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(files)}
    output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(qualification={k: [v['passed'], v['total']] for k, v in summary['qualification'].items()},
                         cli_replay=summary['cli_replay']), indent=2))


if __name__ == '__main__':
    main()
