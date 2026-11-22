"""Repeat the published fits in isolated output directories, including failed candidates."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from train_target_pivot import write
from compare_refit import effective_policy

STAGES = {
    'waist': ('screen','fit','fit_offsets','baseline_long','endurance'),
    'spin': ('screen','evolve','compare','drive','rim_search','cross','feedback','refine','lean','long_probe','freeze'),
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('model',choices=STAGES)
    p.add_argument('output',type=Path)
    p.add_argument('--workers',type=int,default=8)
    p.add_argument('--qualify-spin',action='store_true',help='Run the published test suite on an isolated spin refit')
    a = p.parse_args()
    if not a.qualify_spin:
        a.output.mkdir(parents=True,exist_ok=False)
    elif a.model!='spin' or not (a.output/'frozen.json').exists() or (a.output/'qualification.json').exists():
        p.error('Qualification requires an existing spin refit and a new qualification output')
    if a.model == 'waist':
        import train_waist_balance as module
    else:
        import train_continuous_spin as module
    reference = module.POLICY
    module.POLICY = a.output/'frozen.json'
    module.WORK = a.output
    if a.qualify_spin:
        original = json.loads(module.REPORT.read_text())
        module.REPORT = a.output/'qualification.json'
        module.qualify(a.workers)
        measured = json.loads(module.REPORT.read_text())
        # The archived policy may omit dataclass fields equal to their defaults.
        def normalized(row):
            return dict(row, params=effective_policy('spin',dict(orbit=row['params']))['orbit'])

        for section in ('matched','held_out','controls'):
            if [normalized(r) for r in measured[section]] != [normalized(r) for r in original[section]]:
                raise ValueError(f'Spin evaluation differs in {section}')
        return
    stages = []
    for name in STAGES[a.model]:
        start = time.monotonic()
        started_utc = datetime.now(timezone.utc).isoformat()
        getattr(module,name)(a.workers)
        stages.append(dict(stage=name,seconds=time.monotonic()-start,started_utc=started_utc,
                           ended_utc=datetime.now(timezone.utc).isoformat()))
        write(a.output/'stages.json',stages)
    fitted = json.loads(module.POLICY.read_text())
    if effective_policy(a.model,fitted) != effective_policy(a.model,json.loads(reference.read_text())):
        raise RuntimeError('Refit differs from the published policy')
    print('Matched effective policy in', reference, flush=True)


if __name__ == '__main__':
    main()
