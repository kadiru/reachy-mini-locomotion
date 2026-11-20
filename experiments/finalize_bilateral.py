"""Audit limits, source preservation, replay and artifact provenance for this study."""
import hashlib
import json
import platform
import subprocess
from pathlib import Path
import mujoco
import numpy as np


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    out=Path('experiments/reports/motor-training-bilateral-artifact-receipt-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    recovery=read('experiments/reports/motor-training-balance-neutral-qualified-2026-11-19.json')
    qualification=read('experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json')
    replay=read('experiments/reports/motor-training-bilateral-replay-2026-11-19.json')
    demo=read('experiments/reports/motor-training-penguin-demo-2026-11-19.json')['check']
    finer=read('experiments/reports/motor-training-penguin-demo-finer-2026-11-19.json')['check']
    assert recovery['completed'] and qualification['completed'] and replay['completed']
    checks=recovery['checks']+qualification['checks']+[demo,finer]
    for c in checks:
        r=c['result'];m=r['motor_diagnostics']
        assert r['commanded_waist_position_range_rad']==[0.,0.]
        assert r['tilt_limit_deg']==35 and not r['external_forces_applied']
        assert np.all(np.array(r['peak_motor_torque_nm'])<=np.array(m['torque_caps_nm'])+1e-12)
        assert m['max_filtered_command_speed_rad_s']<=10.7+1e-10
        assert m['max_filtered_command_acceleration_rad_s2']<=360+1e-8
    protected=subprocess.check_output(['git','ls-files','configs'],text=True).splitlines()
    protected+=['src/reachy_locomotion/motors.py','src/reachy_locomotion/sway.py','src/reachy_locomotion/contact_probe.py']
    for name in protected:
        assert Path(name).read_bytes()==subprocess.check_output(['git','show','HEAD:'+name]),name
    names=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],text=True).splitlines()
    modified=subprocess.check_output(['git','diff','--name-only'],text=True).splitlines()
    artifacts=[p for p in names+modified if Path(p).is_file()]
    recordings=[f'experiments/generated/bilateral/{name}.json' for name in ['recovery-left','recovery-right','penguin-two-cycles']]
    inventory=[]
    for name in names:
        if not name.startswith('experiments/reports/motor-training-') or not name.endswith('.json'):continue
        r=read(name);rows=r.get('candidates',r.get('checks',[]))
        inventory.append(dict(path=name,completed=r.get('completed'),record_count=len(rows),
                              note='Record counts include repeated recipes and nested development/qualification; not independent samples.'))
    assert all(c['passed'] and max(c['max_absolute_errors'].values())==0 for c in replay['checks'])
    video=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=width,height,r_frame_rate,nb_frames',
                                             '-of','json','media/simulation/bilateral-penguin-2026-11-19.mp4']))
    summary={}
    for prefix in ['development','fresh','nominal']:
        for pivot in [True,False]:
            rows=[c for c in qualification['checks'] if c['id'].startswith(prefix) and c['pivot_only']==pivot]
            summary[prefix+('-pivot' if pivot else '-walking')]=dict(passed=sum(c['passed'] for c in rows),total=len(rows))
    report=dict(completed=True,evidence='uncalibrated_simulation',hardware_ready=False,python=platform.python_version(),
                mujoco=mujoco.__version__,numpy=np.__version__,base_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                upstream_commit=demo['result']['config']['upstream_commit'],unchanged_from_base=protected,
                tests=dict(count=100,result='passed',log='experiments/reports/repository-tests-bilateral-final-2026-11-19.txt'),
                physical_limit_audit=dict(trials=len(checks),waist_target_rad=0,torque_caps_respected=True,software_profile_unchanged=True,
                                          tilt_guard_deg=35,external_forces=False),
                recovery=dict(passed=sum(c['result']['recovered'] for c in recovery['checks']),total=len(recovery['checks'])),
                qualification=summary,demonstration=demo['penguin'],finer_demonstration=finer['penguin'],
                independent_replay=replay,video=video,video_chapters_s=dict(left_recovery=[0,5.04],right_recovery=[5.04,10.08],penguin=[10.08,42.12]),
                visual_review=['both recovery clips','walking head/support visibility','walking summary frame','all-outcome chart'],
                report_inventory=inventory,hashes={p:sha(p) for p in sorted(set(artifacts+recordings))},
                limitations=['Original development passes; unseen pivot and walking each pass 13/16.',
                             'Guard stops are failures; five guard records repeat some initial-condition prefixes across modes.',
                             'Emergency override remains experimental and is disabled in the selected/video policy.',
                             'Frozen job feedback settings are authoritative for sensor delay in older reports.',
                             'No hardware or shared GPU/Nimbus/EY work was touched.'])
    out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(recovery=report['recovery'],qualification=summary,protected_files=len(protected),audit_trials=len(checks))))

if __name__=='__main__':main()
