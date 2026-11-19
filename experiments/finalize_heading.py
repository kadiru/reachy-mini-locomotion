"""Freeze a non-stalling fast candidate using development data only; fresh audit."""
import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from evaluate_heading import paired,fresh_domains
from select_progress import stress
from train_heading import evaluate,brief


def main():
    p=argparse.ArgumentParser();p.add_argument('development');p.add_argument('--output',required=True);a=p.parse_args();path=Path(a.output)
    if path.exists():p.error('Choose new receipt')
    # Deliberately never access this receipt's old heldout or sensor results.
    development=json.loads(Path(a.development).read_text())['development_20s']
    eligible=[o for o in development if all(m['feasible'] and m['speed_mm_s']>=14 for m in o['metrics'][:2])
              and all(m['speed_mm_s']>=8 for m in o['metrics'])]
    if not eligible:raise RuntimeError('No development candidate preserves speed without stalls')
    selected=max(eligible,key=lambda o:o['score']);policy=dict(gait=selected['gait'],feedback=selected['feedback'])
    base=json.loads(Path('configs/motors-datasheet.json').read_text());sensor=dict(delay_frames=3,noise_std_deg=.25,bias_deg=0.,seed=19031)
    r=dict(evidence='uncalibrated_simulation',hardware_ready=False,selection='development only: >=14mm/s nominal at both dt with nominal tests passing; >=8mm/s in every old stress case; then highest score',
           frozen_policy=policy,development_source=a.development,sensor=sensor,development_seed=591191,heldout_seed=2026111907,
           source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('experiments/train_heading.py'),
           Path('src/reachy_locomotion/heading.py'),Path('src/reachy_locomotion/sway.py'),Path('src/reachy_locomotion/motors.py')]})
    path.write_text(json.dumps(r)+'\n');path.with_name(path.stem+'-policy.json').write_text(json.dumps(policy,indent=2)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        dev=paired([base]+stress(base))
        checks=list(pool.map(evaluate,[(policy['gait'],policy['feedback'],[c],20.,sensor) for c in dev]))
        r['development_20s']=[dict(case_index=i//2,dt_s=c['timestep_s'],trial=o) for i,(c,o) in enumerate(zip(dev,checks))]
        for old,o in zip(selected['results'],checks):
            if abs(old['forward_m']-o['results'][0]['forward_m'])>1e-9:raise RuntimeError('Frozen replay differs from saved development physics')
        r['reproduces_development_exactly']=True;path.write_text(json.dumps(r,allow_nan=False)+'\n')
        baseline=json.loads(Path('configs/sway-progress-fast.json').read_text());zero=dict(policy['feedback'],kp=0.,ki=0.,kd=0.)
        tasks=[(label,g,f,c) for label,g,f in [('original_fast',baseline,None),('feedback_off_same_gait_and_startup',policy['gait'],zero)] for c in dev]
        out=list(pool.map(evaluate,[(g,f,[c],12.,sensor) for label,g,f,c in tasks]))
        r['development_comparisons_12s']=[dict(label=l,case_index=i%14//2,trial=o) for i,((l,g,f,c),o) in enumerate(zip(tasks,out))]
        # Check selected on the identical 12s horizon used by the old stress test.
        out=list(pool.map(evaluate,[(policy['gait'],policy['feedback'],[c],12.,sensor) for c in dev]))
        r['development_12s']=[dict(case_index=i//2,trial=o) for i,o in enumerate(out)]
        fresh=paired(fresh_domains(base,r['heldout_seed'],10));r['fresh_configs']=fresh
        tasks=[(l,g,f,c,dict(sensor,seed=190117+7919*(i//2))) for l,g,f in [('feedback',policy['gait'],policy['feedback']),('original_fast',baseline,None)] for i,c in enumerate(fresh)]
        out=list(pool.map(evaluate,[(g,f,[c],12.,s) for l,g,f,c,s in tasks]))
        r['heldout']=[dict(label=l,case_index=i%20//2,trial=o) for i,((l,g,f,c,s),o) in enumerate(zip(tasks,out))]
        variants=[dict(delay_frames=0,noise_std_deg=0,bias_deg=0,seed=19),dict(delay_frames=5,noise_std_deg=.5,bias_deg=1.5,seed=20),
                  dict(delay_frames=10,noise_std_deg=1.,bias_deg=-2.,seed=21)]
        tasks=[(si,i,c,s) for si,s in enumerate(variants) for i,c in enumerate(paired([base,stress(base)[0],stress(base)[3]]))]
        out=list(pool.map(evaluate,[(policy['gait'],policy['feedback'],[c],12.,s) for si,i,c,s in tasks]))
        r['sensor_stress']=[dict(sensor_index=si,case_index=i,sensor=s,trial=o) for (si,i,c,s),o in zip(tasks,out)]
    r['complete']=True;r['all_old_stress_pass']=all(x['trial']['metrics'][0]['feasible'] for x in r['development_12s'] if x['case_index']>0)
    path.write_text(json.dumps(r,allow_nan=False)+'\n');print('Completed',path,'all old stress pass:',r['all_old_stress_pass'],flush=True)
if __name__=='__main__':main()
