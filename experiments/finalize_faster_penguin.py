"""Audit the selected gait, failed checks, preserved physics, and saved artifacts."""
import hashlib,json,platform,subprocess
from pathlib import Path
import mujoco,numpy as np


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

if __name__=='__main__':
    out=Path('experiments/reports/motor-training-faster-balance-artifact-receipt-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    q=read('experiments/reports/motor-training-faster-balance-feedback-qualified-2026-11-19.json')
    fine=read('experiments/reports/motor-training-faster-balance-finer-failures-2026-11-19.json')
    demo=read('experiments/reports/motor-training-faster-balance-left-demo-2026-11-19.json')['check']
    finer=read('experiments/reports/motor-training-faster-balance-left-finer-2026-11-19.json')['check']
    right=read('experiments/reports/motor-training-faster-balance-feedback-demo-2026-11-19.json')['check']
    old=read('experiments/reports/motor-training-penguin-demo-2026-11-19.json')['check']
    assert q['completed'] and fine['completed'] and demo['passed'] and finer['passed'] and right['passed']
    seen=[c for c in q['checks'] if c['id'].startswith(('development-','fresh-'))]
    unseen=[c for c in q['checks'] if c['id'].startswith('unseen-')]
    assert len(seen)==26 and all(c['passed'] for c in seen)
    assert len(fine['checks'])==4 and all(c['passed'] for c in fine['checks'])
    checks=q['checks']+fine['checks']+[demo,finer,right]
    for c in checks:
        r=c['result'];m=r['motor_diagnostics']
        assert r['commanded_waist_position_range_rad']==[0.,0.] and r['tilt_limit_deg']==35
        assert not r['external_forces_applied']
        assert np.all(np.array(r['peak_motor_torque_nm'])<=np.array(m['torque_caps_nm'])+1e-12)
        assert m['max_filtered_command_speed_rad_s']<=10.7+1e-10
        assert m['max_filtered_command_acceleration_rad_s2']<=360+1e-8
        if c.get('passed'):
            assert r['completed'] and r['settled'] and r['min_joint_margin_rad']>=-.005
            assert r['max_closure_error_m']<=.001
    protected=subprocess.check_output(['git','ls-files','configs'],text=True).splitlines()+[
        'src/reachy_locomotion/motors.py','src/reachy_locomotion/sway.py','src/reachy_locomotion/contact_probe.py','src/reachy_locomotion/primitives.py']
    for name in protected:assert Path(name).read_bytes()==subprocess.check_output(['git','show','HEAD:'+name]),name
    qualified=next(c for c in q['checks'] if c['id']=='nominal-four-pairs' and c['side']==-1)
    for key in ['termination','peak_roll_deg','peak_tilt_deg','planar_displacement_m','peak_motor_torque_nm']:
        assert qualified['result'][key]==demo['result'][key],key
    names=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],text=True).splitlines()
    modified=subprocess.check_output(['git','diff','--name-only'],text=True).splitlines()
    artifacts=[p for p in names+modified if Path(p).is_file()]
    artifacts+=['experiments/generated/faster-balance/best-left-four-pairs.json','experiments/generated/faster-balance/best-feedback-four-pairs.json']
    inventory=[]
    for name in names:
        if name.startswith('experiments/reports/motor-training-faster-balance-') and name.endswith('.json'):
            r=read(name);count=len(r.get('checks',[]))+sum(len(g['checks']) for g in r.get('history',[]))
            inventory.append(dict(path=name,completed=r.get('completed'),records=count))
    video=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=width,height,r_frame_rate,nb_frames',
        '-of','json','media/simulation/faster-penguin-best-2026-11-19.mp4']))
    report=dict(completed=True,evidence='uncalibrated_simulation',hardware_ready=False,
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        python=platform.python_version(),mujoco=mujoco.__version__,numpy=np.__version__,
        selected_policy='configs/penguin-faster-feedback-trained.json',selected_start_side=-1,
        seen=dict(passed=sum(c['passed'] for c in seen),total=len(seen)),unseen=dict(passed=sum(c['passed'] for c in unseen),total=len(unseen),seed=q['seed']),
        finer_former_failures=dict(passed=4,total=4),nominal_four_pair_checks=dict(passed=4,total=4),
        open_failures=[dict(id=c['id'],side=c['side'],termination=c['result']['termination'],complete=c['penguin']['completed'],forward_mm=c['penguin']['forward_mm'],heading_deg=c['penguin']['max_completed_cycle_heading_deg']) for c in unseen if not c['passed']],
        stepping_speed_gain_percent=100*(demo['penguin']['speed_mm_s']/old['penguin']['speed_mm_s']-1),
        finer_speed_difference_percent=100*(finer['penguin']['speed_mm_s']/demo['penguin']['speed_mm_s']-1),
        selected_demo=demo['penguin'],finer_demo=finer['penguin'],right_start_demo=right['penguin'],
        physical_limit_audit=dict(trials=len(checks),constant_waist_target_rad=0,tilt_guard_deg=35,torque_caps_respected=True,
                position_and_closure_guards_respected_in_passes=True,software_profile_unchanged=True),
        unchanged_from_base=protected,independent_qualification_matches_selected_recording=True,
        other_recording_replay=read('experiments/reports/motor-training-faster-balance-recording-replay-2026-11-19.json'),
        tests=dict(count=105,passed=True,log='experiments/reports/repository-tests-faster-balance-2026-11-19.txt'),
        video=video,render_until_s=31.44,full_simulation_horizon_s=36,video_playback_rate=1,
        visual_review=['selected front/side framing','support and heading overlays','speed comparison and all-outcome plot'],
        report_inventory=inventory,hashes={p:sha(p) for p in sorted(set(artifacts))},
        notes=['All earlier failing walking conditions repaired; unseen failures remain.',
               'Qualification counts are trials, not independent robots or guarantees.',
               'Stepping speed excludes the post-finish idle tail; video removes only idle tail.',
               'Earlier age-only qualification is incomplete and superseded; all partial evidence preserved.',
               'No hardware or shared GPU/Nimbus/EY work used.'])
    out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['seen','unseen','stepping_speed_gain_percent','finer_speed_difference_percent']}))
