"""Use finer physics for return-timing repairs and the weak-condition fallback."""
import copy
import itertools
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_balance import cases,evaluate,read


def main():
    bank=read('configs/penguin-feedback-experimental.json');jobs=[]
    for family in ['primary','fallback']:
        timing=[(.6,.2,.4),(.6,.4,.4),(1.,.2,.4)] if family=='primary' else list(itertools.product([.2,.6,1.],[.2,.4],[.4,.7]))
        variants=[('baseline',bank[family])]
        for after,duration,yaw in timing:
            p=copy.deepcopy(bank[family]);p['feedback'].update(return_x_after_catch_s=after,return_x_duration_s=duration);p['recipe']['yaw_return_s']=yaw
            variants.append((f'x{after}-{duration}-yaw{yaw}',p))
        names=['development-0','fresh-3','fresh-7'] if family=='primary' else ['development-4','fresh-7']
        for name,p in variants:
            for original in [j for j in cases() if j['id'] in names]:
                config=copy.deepcopy(original['config']);config['timestep_s']=.00025
                sensors={k:v for k,v in original['policy'][family]['feedback'].items() if k.startswith('sensor_')}
                jobs.append(dict(id=original['id'],variant=family+'-'+name,side=original['side'],config=config,primitive=p,sensors=sensors))
    out=Path('experiments/reports/motor-training-faster-balance-fine-timing-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(variant=c['variant'],case=c['id'],side=c['side'],recovered=c['result']['recovered'],termination=c['result']['termination'],turn=c['signed_turn_deg'],forward_mm=1000*c['result']['planar_displacement_m'][0])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
