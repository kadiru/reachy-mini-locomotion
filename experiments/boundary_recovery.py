"""Resolve late catches around the observed fall with full eight-second trials."""
import argparse
import copy
import hashlib
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_primitives import evaluate,read,summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Preserve evidence')
    s=read(args.input);base=s['best']
    if not s['completed'] or not base['result']['fell']:p.error('Require a measured falling launch')
    jobs=[(f'no-catch-{dt}',base['recipe'],'hold_neutral',dt,8.) for dt in [.001,.0005,.00025]]
    for threshold,action,duration in itertools.product([18,20,22,24,26,28,30,32,34,36,38],['neutral','preload','continue'],[.06,.16,.32]):
        g=copy.deepcopy(base['recipe']);g.update(catch_at_deg=threshold,catch_s=duration)
        if action=='preload':g.update(catch_roll_deg=g['preload_roll_deg'],catch_y_mm=g['preload_y_mm'])
        elif action=='continue':g.update(catch_roll_deg=g['roll_deg'],catch_y_mm=g['y_mm'])
        jobs.append((f'at{threshold}-{action}-{duration}',g,'hold_neutral',.001,8.))
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                baseline=base,candidates=[],source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),Path('src/reachy_locomotion/primitives.py'),Path('experiments/train_primitives.py')]})
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['candidates'].append(c);save();print(json.dumps(summary(c)),flush=True)
    report['completed']=True;save()


if __name__=='__main__':main()
