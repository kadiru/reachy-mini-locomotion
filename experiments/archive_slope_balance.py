"""Assemble the frozen ramp results, measured compute, comparisons and replay manifest."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path

from slope_balance import POLICY, REPORT, WORK, read, run, spec, write

SUPPORT_CONFIGS=(Path('configs/motors-speed-trained.json'),Path('configs/cad-release-catch.json'),
                 Path('configs/balance-recovery-experimental.json'))


def matches(row,result):
    return all(result[k]==v for k,v in row.items() if k in result)


def trace_digest(clip):
    track=read(WORK/(clip+'.json'))['track']
    payload=json.dumps(track,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest()


def select_film(rows,strength,catches):
    adaptive=[v for v in rows if v['controller']=='adaptive' and v['suite'] in ('matched','unseen','steep')]
    def pick_key(r):
        return (r['hold'],r['strict'],r['edge'],r['longest_quiet_s'],-int(r['fell']),r['soft_s'])
    best=max((v for v in rows if v['controller']=='adaptive' and v['bank']),key=pick_key)
    mark=best['clip'] or 'best-frozen'
    original=next((v for v in rows if v['controller']=='original' and v['suite']==best['suite']
                   and v['side']==best['side'] and v['bank']==best['bank']),
                  {k:best[k] for k in ('bank','side','seconds','seed','dt','friction')})
    controls=[dict(original,clip='original-film',controller='original')]
    if best['clip'] is None:controls.append(dict(best,clip=mark))
    chapters=[dict(title=f"{abs(best['bank']):g}° {best['orientation']} ramp  /  {best['longest_quiet_s']:.2f} s of quiet edge balance",
                   panels=[['original-film','Original level-ground policy'],[mark,'Slope-aware policy']],seconds=8.,speed=1.)]
    captured=max(catches,key=lambda v:pick_key(v['result']))
    trial=captured['result']
    nominal=next(v for v in adaptive if v['suite']=='matched' and v['side']==trial['side'] and v['bank']==trial['bank'])
    if trial['hold'] and (not nominal['hold'] or (nominal['bank'],nominal['side'])!=(best['bank'],best['side'])):
        if nominal['hold']:
            source=nominal;name=nominal['clip'];label='Frozen policy'
        else:
            source=trial;name='training-capture';label='Training capture'
            controls.append(dict(source,clip=name,controller='training_candidate',settings=captured['settings']))
        controls.append({k:source[k] for k in ('bank','side','seconds','seed','dt','friction')})
        controls[-1].update(clip='capture-original',controller='original')
        chapters.append(dict(title=f"Flat foot to downhill edge on the {abs(source['bank']):g}° ramp",
                             panels=[['capture-original','Original level-ground policy'],[name,label]],
                             seconds=source['seconds'],speed=1.))
    panels=[]
    for bank,direction in ((6.,'downhill'),(-6.,'uphill')):
        name='six-degree-right-'+('down' if bank>0 else 'up')
        controls.append(dict(bank=bank,side=1,controller='original',seconds=6.,seed=55001,
                             dt=.000125,friction=.7,clip=name))
        panels.append([name,direction.capitalize()+' contact'])
    chapters.append(dict(title='Same right-edge policy on opposite 6° ramps',panels=panels,seconds=6.,speed=1.))
    pair=sorted((v for v in strength if v['family']=='common_level_stroke' and v['scale']==1. and v['side']==1),
                key=lambda r:r['orientation'])
    panels=[]
    for row in pair:
        name='common-'+row['orientation']
        controls.append(dict(row,clip=name,controller='level_only'))
        panels.append([name,row['orientation'].capitalize()+' contact'])
    chapters.append(dict(title="Same right-side head stroke on opposite 5° ramps",panels=panels,seconds=8.,speed=1.))
    for b in sorted({v['bank'] for v in adaptive if v['suite']=='unseen'},key=lambda b:(abs(b),-b)):
        pair=sorted([v for v in adaptive if v['bank']==b and v['suite']=='unseen'],key=lambda r:r['orientation'])
        chapters.append(dict(title=f"Fresh {abs(b):g}° ramp  /  {'left' if b>0 else 'right'} side of the ramp is higher",
                             panels=[[v['clip'],v['orientation'].capitalize()+' edge'] for v in pair],seconds=8.,speed=2.))
    for suite,prefix in [('matched','Training grade'),('steep','Steep ramp')]:
        for b in sorted({v['bank'] for v in adaptive if v['suite']==suite},key=lambda b:(abs(b),-b)):
            pair=sorted([v for v in adaptive if v['bank']==b and v['suite']==suite],key=lambda r:(r['orientation'],r['side']))
            title=f"{prefix}  /  {abs(b):g}°  /  {'left' if b>0 else 'right'} side is higher" if b else 'Level-ground comparison'
            panels=[[v['clip'],(v['orientation'].capitalize() if b else 'Left' if v['side']<0 else 'Right')+' edge'] for v in pair]
            chapters.append(dict(title=title,panels=panels,seconds=8.,speed=2.))
    return chapters,controls


def settings_for(row,policy):
    if row['controller']=='original':
        return None
    if row['controller']=='training_candidate':
        return row['settings']
    side=policy[str(row['side'])]
    if row['controller']=='level_only':
        return dict(next(v['settings'] for v in side['bins'] if v['relative_bank']==0.),bank_gain=0.)
    return side


def classify(r):
    if r['fell']:return 'shell landing'
    if r['joint']<0:return 'joint warning'
    if r['hold']:return 'held edge'
    if r['edge']:return 'edge endpoint'
    if r['flat']:return 'flat return'
    return 'moving'


def plot(rows,barriers,strength,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    fig,(ax,effort,geom)=plt.subplots(3,1,figsize=(10.6,9.3),height_ratios=[1.35,1,1],layout='constrained')
    eligible=[r for r in rows if r['suite'] in ('matched','unseen')]
    banks=sorted({r['bank']*r['side'] for r in eligible})
    styles=[('held edge','#168578'),('edge endpoint','#8ccec2'),('flat return','#b1bdc8'),
            ('shell landing','#c64c43'),('joint warning','#e3a545'),('moving','#a88ab2')]
    labels=['Original left','Original right','Slope-aware left','Slope-aware right']
    mat=[]
    for ctrl in ('original','adaptive'):
        for s in (-1,1):
            data={r['bank']*s:classify(r) for r in eligible if r['controller']==ctrl and r['side']==s}
            mat.append([next(i for i,(label,_) in enumerate(styles) if label==data[b]) for b in banks])
    ax.imshow(mat,interpolation='nearest',cmap=ListedColormap([c for _,c in styles]),vmin=-.5,vmax=5.5,aspect='auto')
    ax.set_yticks(range(4),labels);ax.set_xticks(range(len(banks)),[f'{v:g}°' if v else '0°' for v in banks])
    ax.set_title('Flat start to requested edge in eight seconds',loc='left',weight='bold')
    ax.set_xlabel('Uphill contact   ←   signed cross-slope   →   downhill contact')
    shown={i for row in mat for i in row}
    ax.legend(handles=[Patch(color=c,label=n.capitalize()) for i,(n,c) in enumerate(styles) if i in shown],
              ncol=4,loc='upper center',bbox_to_anchor=(.5,-.22),frameon=False)
    for orientation,color in [('uphill','#9055a2'),('downhill','#17857b')]:
        trials=sorted((r for r in strength if r['side']==1 and r['family']=='common_level_stroke'
                       and r['orientation']==orientation),key=lambda r:r['scale'])
        effort.plot([100*r['scale'] for r in trials],
                    [r['peak_requested_lean']-r['initial_roll'] for r in trials],
                    '.-',label=orientation.capitalize()+' contact',color=color,lw=2)
        for r in trials:
            if r['fell']:
                effort.plot(100*r['scale'],r['peak_requested_lean']-r['initial_roll'],'x',color='#b54737',ms=9)
    effort.plot([],[],'x',color='#b54737',label='Shell landing')
    effort.legend(frameon=False,ncol=3);effort.grid(alpha=.2)
    effort.set_title('Same right-side stroke on opposite 5° ramps',loc='left',weight='bold')
    effort.set_xlabel('Commanded swing strength (%)');effort.set_ylabel('Peak rightward roll\nfrom starting angle (°)')
    for side,name,color in [(-1,'Left edge','#176f69'),(1,'Right edge','#ae6230')]:
        rs=sorted((r for r in barriers if r['side']==side),key=lambda r:r['side']*r['bank'])
        geom.plot([r['side']*r['bank'] for r in rs],[r['com_rise_mm'] for r in rs],'.-',label=name,color=color,lw=2)
    geom.set_title('Center-of-mass rise to the tipping point with a rigid neutral head',loc='left',weight='bold')
    geom.set_xlabel('Signed cross-slope (degrees)');geom.set_ylabel('Rise (mm)');geom.grid(alpha=.2);geom.legend(frameon=False)
    fig.savefig(output,dpi=145);plt.close(fig)


def archive():
    q=read(WORK/'qualification.json');policy=read(POLICY)
    digest=hashlib.sha256(POLICY.read_bytes()).hexdigest()
    if digest!=q['policy_sha256']:raise ValueError('Policy changed following qualification')
    strength=read(WORK/'strength.json')
    chapters,controls=select_film(q['trials'],strength,read(WORK/'coast.json'))
    for row in controls:
        p=settings_for(row,policy['sides'])
        recorded=run(spec(row['bank'],row['side'],p,row['seconds'],row['seed'],row['dt'],row['friction'],row['clip'])+(True,))
        if not matches(row,recorded):raise ValueError(f"Comparison replay differs for {row['clip']}")
        row.update(recorded)
    plans=read(WORK/'plans.json')
    extra=read(WORK/'extra-plans.json')
    trims=read(WORK/'trims.json')
    costs=[];meters=[]
    groups=[('periodic swing','search',['search']),('timed strokes','strokes',['strokes','validate']),
            ('branch planning','plans',[]),('capture and selection','catch',['brakes','coast-search','coast','steep-search','steep-catch','selection','compensation'])]
    for i,(label,key,files) in enumerate(groups):
        m=read(WORK/(key+'-meter.json'));meters.append(m)
        if m['status']!='complete' or m['returncode']!=0:raise ValueError(f'Incomplete meter for {key}')
        episodes=[]
        for f in files:
            for row in read(WORK/(f+'.json')):
                episodes.extend(row['results'] if 'results' in row else [row['result']])
        sim_s=sum(r['elapsed'] for r in episodes)
        if key=='plans':sim_s+=sum(p['simulated_s'] for p in plans)
        if key=='catch':sim_s+=sum(p['simulated_s'] for p in extra)+sum(p['seconds'] for p in trims)
        costs.append(dict(stage=label,meter_index=i,rollouts=len(episodes),workers=6,simulated_rollout_s=sim_s,
                          cpu=m['hardware']['cpu'],cpu_s=m['cpu_s'],wall_s=m['wall_s'],gross_wh=1000*m['gross']['kwh']))
    hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (POLICY,*SUPPORT_CONFIGS)}
    clips={name for chapter in chapters for name,label in chapter['panels']}
    report=dict(evidence='uncalibrated MuJoCo simulation',sha256=hashes,qualification=q,
                barriers=read(WORK/'barriers.json'),strength=strength,
                search_conditions=read(WORK/'selection.json'),branch_plans=plans,energy_guided_plans=extra,
                slope_compensation=read(WORK/'compensation.json'),static_trims=trims,compute=meters,training_cost=costs,
                film=dict(chapters=chapters,controls=controls,trace_sha256={name:trace_digest(name) for name in sorted(clips)}))
    write(REPORT,report);write(WORK/'film.json',chapters)
    plot(q['trials'],report['barriers'],strength,'media/simulation/inclined-edge-results.png')


def replay(render):
    from film_slope_balance import film
    report=read(REPORT);policy=read(POLICY)['sides']
    for filename,digest in report['sha256'].items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=digest:
            raise ValueError(f'Different replay settings in {filename}')
    info=report['film'];needed={p[0] for ch in info['chapters'] for p in ch['panels']}
    rows=[r for r in report['qualification']['trials'] if r['clip'] in needed]+info['controls']
    jobs=[spec(r['bank'],r['side'],settings_for(r,policy),r['seconds'],
               r['seed'],r['dt'],r['friction'],r['clip'])+(True,) for r in rows]
    with ProcessPoolExecutor(4) as pool:
        for row,r in zip(rows,pool.map(run,jobs)):
            name=row['clip']
            if not matches(row,r) or trace_digest(name)!=info['trace_sha256'][name]:
                raise ValueError(f"Replay differs for {name}")
    if render:film(info['chapters'],WORK/'ramp-replay.mp4')
    print(f'Verified {len(rows)} filmed trajectories.')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('task',choices=['archive','verify','replay'])
    a=ap.parse_args()
    if a.task=='archive':archive()
    else:replay(a.task=='replay')
