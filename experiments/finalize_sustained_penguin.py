"""Audit policy, physical limits, qualification evidence and final video files."""
import argparse,hashlib,json,platform,subprocess
from pathlib import Path
import mujoco,numpy

ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads((ROOT/p).read_text())
def digest(p):return hashlib.sha256((ROOT/p).read_bytes()).hexdigest()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--record',required=True)
    parser.add_argument('--video',required=True);parser.add_argument('--reports',nargs='+',required=True)
    parser.add_argument('--output',required=True);args=parser.parse_args()
    out=ROOT/args.output
    if out.exists():raise FileExistsError(out)
    groups={};failures=[];audited=0
    for path in args.reports:
        r=read(path);assert r['completed']
        for c in r['checks']:
            p=c['result'];audited+=1
            groups.setdefault(c['variant'],[]).append(c['passed'])
            assert p['waist_mode']=='hold_neutral' and p['commanded_waist_position_range_rad']==[0.,0.]
            assert p['tilt_limit_deg']==35 and not p['external_forces_applied']
            assert all(a<=b+1e-8 for a,b in zip(p['peak_motor_torque_nm'],p['motor_diagnostics']['torque_caps_nm']))
            m=p['config']['motor_model']
            assert [m[k] for k in ['target_speed_rad_s','target_acceleration_rad_s2','target_filter_hz']]==[10.7,360.,25.]
            assert m['total_mass_kg']==1.475
            if c['passed']:
                assert p['completed'] and p['settled'] and p['min_joint_margin_rad']>=-.005
                assert p['max_closure_error_m']<=.001 and p['peak_tilt_deg']<35
                assert c['penguin']['completed'] and c['penguin']['max_completed_cycle_heading_deg']<=20
            else:failures.append(dict(report=path,id=c['id'],side=c['side'],variant=c['variant'],termination=p['termination'],
                forward_mm=c['penguin']['forward_mm'],cycle_heading_deg=c['penguin']['max_completed_cycle_heading_deg']))
    tracked=subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines()
    protected=[p for p in tracked if p.startswith('configs/')]+['src/reachy_locomotion/motors.py','src/reachy_locomotion/sway.py','src/reachy_locomotion/contact_probe.py']
    for path in protected:
        old=subprocess.check_output(['git','show',f'HEAD:{path}'],cwd=ROOT)
        assert hashlib.sha256(old).hexdigest()==digest(path),path
    test_path='experiments/reports/repository-tests-sustained-final-2026-11-20.txt'
    log=(ROOT/test_path).read_text();assert 'Ran 110 tests' in log and log.rstrip().endswith('OK')
    record=read(args.record);assert record['penguin']['passed'] and len(record['penguin']['pairs'])==20
    assert record['balance_policy']==read('configs/penguin-sustained-trained.json')
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=width,height,r_frame_rate,nb_frames','-of','json',str(ROOT/args.video)],text=True))
    assert probe['streams'][0]['r_frame_rate']=='25/1'
    assert abs(float(probe['format']['duration'])-(record['penguin']['steps'][-1]['end_s']+1))<.05
    replay=read('experiments/reports/motor-training-sustained-recording-replay-2026-11-20.json')
    assert replay['completed'] and all(v==0 for v in replay['max_absolute_errors'].values())
    baseline=next(c for c in read('experiments/reports/motor-training-sustained-long-screen-2026-11-20.json')['checks']
                  if c['variant']=='previous-baseline' and c['side']==-1)
    changed=set(subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=ROOT,text=True).splitlines())
    changed.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines())
    changed.add(args.record)
    result=dict(completed=True,evidence='uncalibrated_simulation',hardware_ready=False,
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        python=platform.python_version(),mujoco=mujoco.__version__,numpy=numpy.__version__,
        selected_policy='configs/penguin-sustained-trained.json',selected_record=args.record,
        groups={k:dict(passed=sum(v),total=len(v)) for k,v in groups.items()},failures=failures,
        audited_physical_trials=audited,unchanged_files=protected,tests=110,video=probe,
        selected_demo=record['penguin'],waist_range_deg=record['measured_waist_angle_range_deg'],
        waist_peak_torque_nm=record['waist_peak_torque_nm'],
        nominal_speed_gain_percent=100*(record['penguin']['speed_mm_s']/baseline['penguin']['speed_mm_s']-1),
        independent_nominal_replay=replay,
        hashes={p:digest(p) for p in sorted(changed) if p!=args.output},
        limits=['Simulation only; motor dynamics, battery bus, sensing and contact remain uncalibrated.',
                'No thermal model or hardware endurance claim.',
                'Known cases were used for development; new held-out cases were evaluated after freezing the policy.',
                'Speed uses stepping duration; video is real time and only removes final idle frames.'])
    out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(groups=result['groups'],failures=failures,tests=110,physical_trials=audited),indent=2))
