"""Summarize measured edge holds and retain explicit unsuccessful checks."""
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','experiments/generated/edge-balance/mpl-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read(p):return json.loads(Path(p).read_text())
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    output=Path('experiments/reports/motor-training-edge-artifacts-2026-11-20.json')
    if output.exists():raise FileExistsError(output)
    sources=[Path('configs/edge-balance-experimental.json'),Path('configs/motors-speed-trained.json'),
             *[Path('src/reachy_locomotion')/x for x in ['edge_balance.py','edge_linear.py','primitives.py','motors.py','sway.py']],
             Path('experiments/render_sway.py'),Path(__file__)]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,source_sha256={str(p):digest(p) for p in sources},demos={},ablations={},checks=[],searches=[])
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    colors={'left':'#1976b9','right':'#e16c1e'}
    for side in ['left','right']:
        file=Path(f'experiments/generated/edge-balance/final-{side}.json');r=read(file);a=read(f'experiments/generated/edge-balance/ablation-{side}.json')
        assert r['natural_upright_start'] and r['diagnostic_initial_roll_deg'] is None and not r['external_forces_applied']
        assert r['completed'] and r['edge_balance']['metrics']['final_five_seconds_pass']
        assert r['commanded_waist_position_range_rad']==[0,0]
        assert np.all(np.array(r['peak_motor_torque_nm'])<=np.array(r['motor_diagnostics']['torque_caps_nm'])+1e-9)
        keys=['peak_roll_deg','peak_tilt_deg','peak_side_height_difference_mm','min_joint_margin_rad','max_closure_error_m',
              'waist_peak_torque_nm','waist_absolute_motor_work_j','measured_waist_angle_range_deg','head_braking_work_j','peak_motor_torque_nm','motor_diagnostics','catch_trigger_s']
        report['demos'][side]=dict(recording=str(file),sha256=digest(file),metrics=r['edge_balance']['metrics'],**{k:r[k] for k in keys})
        prefix=0
        for x,y in zip(r['trajectory'],a['trajectory']):
            if x['qpos']!=y['qpos'] or x['qvel']!=y['qvel']:break
            prefix+=1
        report['ablations'][side]=dict(termination=a['termination'],hold=a['edge_balance']['metrics'],identical_prefix_frames=prefix,identical_through_s=(prefix-1)*.02)
        times=np.array([x['time_s'] for x in r['trace']]);color=colors[side]
        axes[0].plot(times,[x['roll_deg'] for x in r['trace']],label=side.title(),color=color)
        target=32.1*(-1 if side=='left' else 1);axes[0].axhspan(target-2,target+2,color=color,alpha=.1)
        axes[1].plot(times,[x['side_height_difference_mm'] for x in r['trace']],color=color)
        axes[2].plot(times,[x['roll_rate_deg_s'] for x in r['trace']],color=color)
    axes[0].set_ylabel('Body roll (degrees)');axes[0].legend(loc='center right');axes[1].set_ylabel('Opposite-rim height\ndifference (mm)')
    axes[2].set_ylabel('Body roll rate (deg/s)');axes[2].set_xlabel('Time from flat start (seconds)')
    for ax in axes:ax.grid(alpha=.2);ax.set_xlim(0,20)
    fig.suptitle('Natural lift and sustained edge balance — separate left/right trials')
    fig.text(.5,.015,'Uncalibrated simulation • 0.125 ms physics • 50 Hz control • fixed zero waist target\nNominal success only: capture still fails sensor-noise, delay and some timestep checks.',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.06,1,.96]);chart=Path('media/simulation/edge-balance-2026-11-20-chart.png');fig.savefig(chart,dpi=160);plt.close(fig)
    for file in ['experiments/reports/motor-training-edge-final-2026-11-20.json','experiments/reports/motor-training-edge-extended-2026-11-20.json']:
        r=read(file);assert r['completed']
        for c in r['candidates']:
            report['checks'].append(dict(source=file,id=c['id'],termination=c['result']['termination'],metrics=c['metrics']))
    for file in sorted(Path('experiments/reports').glob('motor-training-edge-*.json')):
        if file==output:continue
        r=read(file)
        if 'candidates' in r:report['searches'].append(dict(path=str(file),sha256=digest(file),completed=r['completed'],recorded_trials=len(r['candidates'])))
    report['artifacts']={str(p):dict(bytes=p.stat().st_size,sha256=digest(p)) for p in [chart,Path('media/simulation/edge-balance-bilateral-2026-11-20.mp4'),Path('media/simulation/edge-balance-bilateral-2026-11-20.png')]}
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(output)


if __name__=='__main__':main()
