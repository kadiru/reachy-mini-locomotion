"""Measure head-only steering sensitivity in the existing stress domains."""
import argparse
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from select_progress import stress
from train_progress import evaluate

def main():
    p=argparse.ArgumentParser();p.add_argument('--phase',action='store_true');p.add_argument('--harmonic',action='store_true');p.add_argument('--output');a=p.parse_args()
    output=Path(a.output or ('experiments/reports/motor-training-heading-'+('harmonic-probe' if a.harmonic else ('phase-probe' if a.phase else 'probe'))+'-2026-11-19.json'))
    if output.exists():p.error('Choose a new --output path')
    base=json.loads(Path('configs/motors-datasheet.json').read_text())
    gait=json.loads(Path('configs/sway-progress-fast.json').read_text())
    variants={'original':gait}
    for k,values in [('yaw_bias_deg',[-8,8]),('roll_bias_deg',[-1.3,1.3]),('yaw_phase_deg',[73,113]),('y_phase_deg',[-149,-109])]:
        for v in values:variants[k+str(v)]=dict(gait,**{k:v})
    if a.phase:variants={str(v):dict(gait,phase_skew=v) for v in [-.15,0,.15]}
    if a.harmonic:variants={f'{v}/{phase}':dict(gait,yaw_harmonic_deg=v,yaw_harmonic_phase_deg=phase) for v in [-4,4] for phase in [0,90]}
    tasks=[(label,i,g,c) for label,g in variants.items() for i,c in enumerate([base]+stress(base))]
    with ProcessPoolExecutor(max_workers=4) as pool:
        out=list(pool.map(evaluate,[(g,[c],4.3) for _,_,g,c in tasks]))
    rows=[dict(label=l,domain=i,trial=o) for (l,i,g,c),o in zip(tasks,out)]
    output.write_text(json.dumps(rows,allow_nan=False)+'\n')
    for label in variants:
        print(label,[(r['domain'],round(r['trial']['results'][0]['forward_m_per_s']*1000,1),round(r['trial']['results'][0]['unwrapped_heading_change_rad']*57.2958,1),r['trial']['results'][0]['valid']) for r in rows if r['label']==label],flush=True)
if __name__=='__main__':main()
