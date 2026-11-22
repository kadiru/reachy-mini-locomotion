"""Archive the CAD-foot experiments or reproduce and verify the films from that archive."""
import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from film_shell_release import film
from tilted_release import WORK, read, run, write

REPORT=Path('experiments/reports/motor-training-shell-release.json')
FILMS={'tilt':'cad-tilt-catches','side':'cad-side-roll'}


def archive():
    files=('tilt','tilt-recheck','fit','local','facet','qualification','deep','geometry','side','gentle',
           'side-rest','follow-side','side-verify','clearance')
    r={name:read(WORK/(name+'.json')) for name in files}
    # The exploratory 90-degree sweep preceded the per-contact slip audit.
    # Publish slip only for the later runs that measured it at each material point.
    for trial in r['side']:
        trial.pop('integrated_torso_slip_mm',None)
    r.update(evidence='uncalibrated MuJoCo simulation',hardware_ready=False,waist='hold neutral',
             actuation='configs/motors-speed-trained.json',upstream='a9a2a42adcc1abc7d9c90d390889bbebf930c96c')
    r['compute']=[read(WORK/(name+'-meter.json')) for name in ('fit','local','local-gains','pitch','side')]
    r['training_cost']=[]
    stages=('original gain sweep','CAD system identification','CAD regulator gains','pitch trim','gravity-directed side strokes')
    for i,(name,key) in enumerate(zip(stages,('fit',None,'local','facet','follow-side'))):
        m=r['compute'][i]
        if m['status']!='complete' or m['returncode']!=0:
            raise ValueError('Incomplete fitting receipt')
        r['training_cost'].append(dict(stage=name,meter_index=i,rollouts=len(r[key]) if key else 48,
            simulated_rollout_s=sum(v['elapsed_seconds'] for v in r[key]) if key else None,
            workers=int(m['command'][m['command'].index('--workers')+1]),cpu=m['hardware']['cpu'],
            wall_s=m['wall_s'],cpu_s=m['cpu_s'],gross_wh=1000*m['gross']['kwh']))
    r['sha256']={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in
                 ('configs/balance-recovery-experimental.json','configs/motors-speed-trained.json','experiments/tilted_release.py')}
    sources=dict(r['sha256'])
    settings_by_skill={'release_settings':r['qualification']['winners'],
                       'side_settings':[v['job'] for v in r['follow-side']]}
    for name,settings in settings_by_skill.items():
        payload=json.dumps(dict(settings=settings,sources=sources),sort_keys=True)
        r['sha256'][name]=hashlib.sha256(payload.encode()).hexdigest()
    r['films']={}
    for name in FILMS:
        timeline=read(WORK/(name+'-film.json'))
        clips={p['clip'] for chapter in timeline for p in chapter['panels']}
        spec={key:read(WORK/(key+'.json'))['metrics'] for key in sorted(clips)}
        r['films'][name]=dict(timeline=timeline,results=spec)
    write(REPORT,r)


def replay(workers):
    records=read(REPORT)['films']
    for name,info in records.items():
        specs=info['results']
        jobs=[dict(s['job'],record=k) for k,s in specs.items()]
        with ProcessPoolExecutor(workers) as pool:
            observed=list(pool.map(run,jobs))
        for key,actual in zip(specs,observed):
            if actual!=specs[key]:
                raise ValueError(f'{key}: replay differs from archived measurements')
        manifest=WORK/(name+'-film.json')
        write(manifest,info['timeline'])
        film(manifest,WORK/(FILMS[name]+'-replay.mp4'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('task',choices=('archive','replay'))
    p.add_argument('--workers',type=int,default=4)
    a=p.parse_args()
    if a.task=='archive':
        archive()
    else:
        replay(a.workers)
