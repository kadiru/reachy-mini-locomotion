"""Probe larger yaw and shorter turn/stop strokes around a rocking candidate."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from train_rocking import decode,evaluate,feedback,read,vector


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();out=Path(args.output)
    if out.exists():p.error('Choose a new report')
    source=read(args.input)
    if not source['completed']:p.error('Wait for the search to finish')
    best=source['best_short'];gaits=[]
    for frequency in [1.6666666667,2.,2.5]:
        for yaw in [30,40,50]:
            for phase in [50,75,100,125]:
                for kind in ['yaw_brake','both_brake']:
                    g=dict(best['gait'],frequency_hz=frequency,yaw_deg=yaw,yaw_phase_deg=phase,
                           yaw_stroke_fraction=.6,stroke_accel_fraction=.25,stroke_brake_fraction=.03)
                    if kind=='both_brake':g['roll_stroke_fraction']=.65
                    gaits.append(decode(vector(g)))
    for pitch in [0,4,8,12]:
        for phase in [-180,-90,0,90]:
            gaits.append(dict(best['gait'],pitch_deg=pitch,pitch_phase_deg=phase))
    jobs=[(f'step-{i}-{kind}',g,feedback(kind),8.,.001) for i,g in enumerate(gaits) for kind in ['open','endpoint']]
    report=dict(evidence='uncalibrated_simulation',hardware_ready=False,completed=False,args=vars(args),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),candidates=[])
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(evaluate,jobs):
            report['candidates'].append(r);out.write_text(json.dumps(report,allow_nan=False)+'\n')
            print(json.dumps(dict(id=r['id'],**r['metrics'])),flush=True)
    report['completed']=True;out.write_text(json.dumps(report,allow_nan=False)+'\n')


if __name__=='__main__':main()
