"""Compose the repaired primitives and shorten only the phase-completion pauses."""
import copy,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from train_faster_balance import cases,read
from qualify_policy_bank import evaluate


def selected_policy():
    a=read('experiments/reports/motor-training-faster-balance-primary-stress-2026-11-19.json')
    b=read('experiments/reports/motor-training-faster-balance-fallback-feedback-2026-11-19.json')
    primary=[c for c in a['checks'] if c['variant']=='phase-11']
    fallback=[c for c in b['checks'] if c['variant']=='phase-5']
    assert len(primary)==26 and all(c['result']['recovered'] for c in primary)
    assert len(fallback)==2 and all(c['result']['recovered'] for c in fallback)
    p=read('configs/penguin-feedback-experimental.json');p['primary']=primary[0]['primitive'];p['fallback']=fallback[0]['primitive']
    return p

def make_job(original,p,seconds=20,cycles=1,dt=.00025):
    j=copy.deepcopy(original);j['policy']=copy.deepcopy(p);j['pivot_only']=False;j['seconds']=seconds;j['cycles']=cycles
    j['config']['timestep_s']=dt
    for family in [n for n in ['primary','fallback','delayed_sensor_fallback'] if n in p]:
        sensors={k:v for k,v in original['policy'].get(family,original['policy']['primary'])['feedback'].items() if k.startswith('sensor_')}
        j['policy'][family]['feedback'].update(sensors)
    return j

def main():
    p=selected_policy();jobs=[]
    sequence=[dict(min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0.),
              dict(min_step_s=1.5,settle_s=.08,post_catch_s=.8,head_return_margin_s=.04),
              dict(min_step_s=2.,settle_s=.14,post_catch_s=1.2,head_return_margin_s=.2)]
    for i,s in enumerate(sequence):
        policy=dict(p,sequencing=s)
        for original in cases():
            if original['id'] not in ['development-0','development-4','fresh-3','fresh-5','fresh-7']:continue
            j=make_job(original,policy);j['variant']=f'pause-{i}';jobs.append(j)
    out=Path('experiments/reports/motor-training-faster-balance-composition-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    report=dict(completed=False,evidence='uncalibrated_simulation',hardware_ready=False,frozen_policy=p,jobs=jobs,checks=[])
    def save():out.write_text(json.dumps(report,allow_nan=False)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for j,c in zip(jobs,pool.map(evaluate,jobs)):
            c['variant']=j['variant'];report['checks'].append(c);save()
            print(json.dumps(dict(variant=c['variant'],case=c['id'],side=c['side'],passed=c['passed'],complete=c['penguin']['completed'],
                  termination=c['result']['termination'],speed=c['penguin']['speed_mm_s'],forward_mm=c['penguin']['forward_mm'],
                  heading=c['penguin']['max_completed_cycle_heading_deg'],bursts=len(c['penguin']['bursts']))),flush=True)
    report['completed']=True;save()

if __name__=='__main__':main()
