"""Blend pivot feedback into the already trained recovery gains after the stroke."""
import copy,itertools,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_balance import cases,evaluate,read


def main():
    bank=read('configs/penguin-feedback-experimental.json');jobs=[]
    for family in ['primary','fallback']:
        selected=[j for j in cases() if (j['id'],j['side']) in ([('development-0',1),('fresh-3',-1)] if family=='primary' else [('development-4',1),('fresh-7',-1)])]
        for after,duration in itertools.product([.2,.4,.6,.8],[.12,.24]):
            p=copy.deepcopy(bank[family]);p['feedback'].update(return_x_after_catch_s=.6,return_x_duration_s=.4,
                  recovery_gain_after_catch_s=after,recovery_gain_blend_s=duration)
            p['recipe']['yaw_return_s']=.4
            for original in selected:
                config=copy.deepcopy(original['config']);config['timestep_s']=.00025
                sensors={k:v for k,v in original['policy'][family]['feedback'].items() if k.startswith('sensor_')}
                jobs.append(dict(id=original['id'],variant=f'{family}-{after}-{duration}',side=original['side'],config=config,primitive=p,sensors=sensors))
    out=Path('experiments/reports/motor-training-faster-balance-transition-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=2) as pool:
        for c in pool.map(evaluate,jobs):
            report['checks'].append(c);save();print(json.dumps(dict(variant=c['variant'],case=c['id'],side=c['side'],recovered=c['result']['recovered'],termination=c['result']['termination'],turn=c['signed_turn_deg'],forward_mm=1000*c['result']['planar_displacement_m'][0])),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
