"""Fit a traveling reference which blends four previously identified edge regulators."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from fit_traveling_edge import trial,WORK,read,write
from train_gyro_peg import REPORT


def run(job):
    args,mix=job
    return trial(args,sector_mix=mix)


def fit(n):
    jobs=[((s,p,-s,1.,'edge_nominal',4.5+2.3*p),mix)
          for s in (-1,1) for p in (8.,10.,14.) for mix in (.55,1.)]
    with ProcessPoolExecutor(n) as pool:rows=list(pool.map(run,jobs))
    write(WORK/'sector-fit.json',rows)
    def score(r):
        m=r['metrics']
        return (r['passed'],m['longest_rim_turns'],r['safe'],m['median_lean_deg'])
    best=max(rows+read(WORK/'traveling-endurance.json'),key=score)
    write(WORK/'sector-frozen.json',dict(side=best['side'],period=best['period'],direction=best['direction'],
                                        gain=best['gain'],seconds=best['seconds'],sector_mix=best.get('sector_mix',0.)))
    print('Best',score(best),best['termination'],flush=True)


def qualify(n):
    f=read(WORK/'sector-frozen.json')
    p=tuple(f[k] for k in ('side','period','direction','gain'))
    nom=trial((*p,'edge_nominal',f['seconds'],WORK/'sector-best.json'),sector_mix=f['sector_mix'])
    tests=[((*p,c,f['seconds']),f['sector_mix']) for c in ('edge_floor','edge_battery','edge_noise','edge_fine')]
    with ProcessPoolExecutor(n) as pool:rows=list(pool.map(run,tests))
    write(WORK/'sector-checks.json',dict(nominal=nom,held_out=rows))


def replay():
    config = read('configs/gyro-peg-deep-experimental.json')
    inputs = tuple(config[key] for key in ('side', 'period', 'direction', 'gain'))
    result = trial((*inputs, 'edge_nominal', config['seconds'], WORK/'sector-best.json'),
                   sector_mix=config['sector_mix'])
    if result != read(REPORT)['deep_tests']['nominal']:
        raise ValueError('Replay differs from the archived deep-roll trial')


def long_audit():
    from audit_gyro_peg import StepContact
    from reachy_locomotion.rim_roll import measures
    f=read(WORK/'sector-frozen.json')
    p=tuple(f[k] for k in ('side','period','direction','gain'))
    audit=StepContact()
    result=trial((*p,'edge_nominal',65.),sector_mix=f['sector_mix'],step_audit=audit)
    measured=measures(audit.rows,.056,f['direction'],result['captured_s'] or 1.5)
    write(WORK/'deep-audit.json',dict(episode=result,physics_hz=8000,samples=len(audit.rows),contact=measured))



if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=('fit','qualify','audit','replay'))
    p.add_argument('--workers',type=int,default=6)
    a=p.parse_args()
    if a.phase=='audit':long_audit()
    elif a.phase=='replay':replay()
    else:{'fit':fit,'qualify':qualify}[a.phase](a.workers)
