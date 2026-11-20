"""Fine-timestep catch-phase search on a promoted failure and nominal anchor."""
import argparse,copy,itertools,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_balance import cases,evaluate,read


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--family',choices=['primary','fallback'],default='primary')
    args=parser.parse_args();family=args.family
    seed=read('configs/penguin-feedback-experimental.json')[family];jobs=[]
    anchors=[('development-0',1),('fresh-3',-1)] if family=='primary' else [('development-4',1),('fresh-7',-1)]
    selected=[j for j in cases() if (j['id'],j['side']) in anchors]
    for i,(trigger,pred,delay) in enumerate(itertools.product([6,10,14],[.04,.08],[0,.02])):
        p=copy.deepcopy(seed);p['feedback'].update(trigger_deg=trigger,prediction_s=pred,return_x_after_catch_s=.6,return_x_duration_s=.4,yaw_delay_after_catch_s=.02+delay)
        p['recipe'].update(catch_delay_s=delay,yaw_return_s=.4)
        for original in selected:
            config=copy.deepcopy(original['config']);config['timestep_s']=.00025
            sensors={k:v for k,v in original['policy'][family]['feedback'].items() if k.startswith('sensor_')}
            jobs.append(dict(id=original['id'],variant=f'phase-{i}',side=original['side'],config=config,primitive=p,sensors=sensors))
    suffix='feedback' if family=='primary' else 'fallback-feedback'
    out=Path(f'experiments/reports/motor-training-faster-balance-{suffix}-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=2) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(variant=c['variant'],case=c['id'],side=c['side'],recovered=c['result']['recovered'],termination=c['result']['termination'],turn=c['signed_turn_deg'],forward_mm=1000*c['result']['planar_displacement_m'][0])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
