"""Re-evaluate primitive candidates at a finer timestep before using them."""
import argparse
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_primitives import evaluate,read,summary

def pivot_metrics(r):
    drift=1000*math.hypot(*r['planar_displacement_m']);turn=abs(r['edge_supported_foot_yaw_deg'])
    passed=r['recovered'] and r['edge_support_seconds']>=.08 and turn>=5 and abs(r['final_foot_yaw_deg'])>=5 and drift<=56 and r['edge_contact_slip_distance_mm']<=15
    return dict(passed=passed,center_drift_mm=drift,edge_turn_deg=turn,
                score=(1000 if passed else 0)+turn-.25*drift-.5*r['edge_contact_slip_distance_mm'])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['falls','catches','pivots'],required=True)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--timestep',type=float,default=.00025)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    s=read(args.input)
    if not s['completed']:p.error('Wait for complete input')
    cs=s['candidates']
    if args.mode=='falls':
        cs=[c for c in cs if c['result']['fell']];cs.sort(key=lambda c:c['result']['peak_roll_deg'],reverse=True)
    elif args.mode=='catches':
        cs=[c for c in cs if c['result']['recovered'] and c['result']['catch_trigger_s'] is not None]
        cs.sort(key=lambda c:c['result']['peak_roll_deg'],reverse=True)
    else:
        cs=[c for c in cs if c['result']['completed']]
        cs.sort(key=lambda c:pivot_metrics(c['result'])['score'],reverse=True)
    unique={}
    for c in cs:unique.setdefault(json.dumps(c['recipe'],sort_keys=True),c)
    cs=list(unique.values())[:12]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                candidates=[],checks=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('experiments/train_primitives.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    jobs=[(c['id'],c['recipe'],c['waist_mode'],args.timestep,c['result']['requested_seconds']) for c in cs]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for old,c in zip(cs,pool.map(evaluate,jobs)):
            r=c['result']
            if args.mode=='falls':accepted=r['fell'] and r['peak_roll_deg']>=max(20,.8*r['peak_tilt_deg'])
            elif args.mode=='catches':accepted=r['recovered'] and r['catch_trigger_s'] is not None
            else:accepted=pivot_metrics(r)['passed'] and pivot_metrics(old['result'])['passed']
            c.update(coarser_result=old['result'],coarser_timestep_s=old['timestep_s'],accepted=accepted)
            if args.mode=='pivots':c['pivot_metrics']=pivot_metrics(r)
            report['checks'].append(c)
            if accepted:report['candidates'].append(c)
            save();print(json.dumps(dict(accepted=accepted,**summary(c))),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
