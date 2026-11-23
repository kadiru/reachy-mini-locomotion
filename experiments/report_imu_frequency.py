"""Build the qualification matrix and its trial-level source data."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean

from imu_frequency_study import CORE, RATES, WORK, FULL_LENGTH
from imu_skill_cases import GROUPS, HISTORICAL, OPEN_LOOP
from reachy_locomotion.contact_probe import PIN

REPORT = Path('experiments/reports')
MEDIA = Path('media/simulation')
TIMED_COMMANDS = set(OPEN_LOOP) | {'fast spin','rim spin','fall primitive','teleop cadence','side shell rocking'}
PERFORMANCE = {'mm/s','mm','mm/swing','rpm','signed turn deg','rim turns'}


def rows(name):
    return [json.loads(line) for line in (WORK / (name+'.jsonl')).read_text().splitlines()]


def classify(raw, order):
    index = {(r['skill'],r['rate'],r['repeat']):r for r in raw}
    if len(index) != len(raw) or len(raw) != len(order)*len(RATES)*2:
        raise ValueError('Expected exactly two unique trials per skill and pipeline')
    for r in raw:
        ref = index[r['skill'],'archive',r['repeat']]
        floor = (.8*ref['value'] if ref['unit'] in PERFORMANCE and ref['passed'] and ref['value']>0 else None)
        r['archive_floor'] = floor
        r['qualified'] = r['passed'] and (floor is None or r['value']>=floor)
    matrix = []
    for name in order:
        cells = []
        for hz in RATES:
            pair = [index[name,hz,r] for r in (0,1)]
            cells.append(dict(rate=hz,passed=sum(r['passed'] for r in pair),qualified=sum(r['qualified'] for r in pair),
                              mean_value=mean(r['value'] for r in pair),unit=pair[0]['unit']))
        minimum = next((c['rate'] for i,c in enumerate(cells[2:],2)
                        if all(x['qualified']==2 for x in cells[i:])),None)
        matrix.append(dict(skill=name,cells=cells,timed_commands=name in TIMED_COMMANDS,
                           screen_floor_hz=None if name in TIMED_COMMANDS else minimum))
    return matrix


def verify_controls(raw, placement):
    trials = {(r['skill'], r['rate'], r['repeat']): r for r in raw}
    for r in raw:
        if r['skill'] in TIMED_COMMANDS:
            original = trials[r['skill'], 'archive', r['repeat']]
            if r['passed'] != original['passed'] or not math.isclose(r['value'], original['value'], abs_tol=1e-9):
                raise ValueError(f"Timed command changed outcome at {r['rate']} Hz for {r['skill']}")
    for r in placement:
        if r['mount'] == 'base':
            original = trials[r['skill'], r['rate'], r['repeat']]
            if r['passed'] != original['passed'] or not math.isclose(r['value'], original['value'], abs_tol=1e-9):
                raise ValueError(f"Independent foot replay changed outcome for {r['skill']}")


def figure(matrix):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    import numpy as np

    split = (len(matrix)+1)//2
    fig, axes = plt.subplots(1,2,figsize=(14.2,10.5))
    fig.subplots_adjust(top=.86,bottom=.09,left=.14,right=.99,wspace=.40)
    palette = ListedColormap(['#f0e8dc','#74c4c7','#154f60'])
    for ax, chunk in zip(axes,(matrix[:split],matrix[split:])):
        arr = np.array([[c['qualified'] for c in r['cells']] for r in chunk])
        ax.imshow(arr,aspect='auto',cmap=palette,norm=BoundaryNorm([-.5,.5,1.5,2.5],3))
        ax.set_ylim(split-.5,-.5)
        ax.set_yticks(range(len(chunk)),[r['skill'] for r in chunk],fontsize=8.5)
        ax.set_xticks(range(9),['Orig.','Stock','25','50','100','200','400','500','1k'],fontsize=9)
        ax.xaxis.tick_top()
        ax.tick_params(which='both',length=0,pad=5)
        ax.axvline(1.5,color='white',linewidth=4)
        for y in range(len(chunk)):
            for x in range(9):
                v=arr[y,x]
                ax.text(x,y,str(v),ha='center',va='center',color='white' if v==2 else '#173037',fontsize=8)
        for edge in ax.spines.values():
            edge.set_visible(False)
    fig.text(.04,.96,'How many trials qualified at each IMU rate?',fontsize=19,weight='bold',color='#18343a')
    fig.text(.04,.926,'Two matched tests per cell  •  Frozen Reachy Mini policies  •  MuJoCo  •  50 Hz motor commands',fontsize=12)
    fig.text(.04,.895,'Orig. = original observations     Stock = modeled 50 Hz daemon     Numeric columns = fresh, wider-band packets per second',fontsize=10)
    fig.text(.04,.048,'Qualification requires task success. Travel, speed and turning also require ≥80% of the matching successful original performance.',fontsize=10)
    fig.text(.04,.024,'Phases and conditions are paired. Two successes establish a screening result. Hardware performance remains unmeasured.',fontsize=10)
    fig.savefig(MEDIA/'imu-frequency.png',dpi=160)
    plt.close(fig)


def overview(matrix, placement):
    import matplotlib.pyplot as plt

    hz = [r for r in RATES if isinstance(r,int)]
    fig, (a,b) = plt.subplots(1,2,figsize=(10.5,4.6),gridspec_kw={'width_ratios':[1.4,1]})
    fig.subplots_adjust(top=.77,left=.065,right=.98,bottom=.22,wspace=.30)
    fig.text(.06,.94,'IMU cadence and sensor location',fontsize=18,weight='bold',color='#173740')
    fig.text(.06,.87,'Frozen controllers in MuJoCo   •   One motor command every 20 ms',fontsize=10,color='#45616a')
    for label, names, color, marker in [('Walking',GROUPS['walking'],'#167779','o'),
                                         ('Turning',GROUPS['turning'],'#a75525','s'),
                                         ('Balance / impacts',CORE,'#495d9d','D')]:
        selected=[r for r in matrix if r['skill'] in names and not r['timed_commands'] and r['cells'][0]['qualified']==2]
        counts=[100*sum(r['cells'][i]['qualified']==2 for r in selected)/len(selected)
                for i in range(2,len(RATES))]
        a.plot(range(len(hz)),counts,marker=marker,color=color,label=f'{label} (n={len(selected)})',linewidth=1.8)
    a.set_title('Cases qualifying in both trials',loc='left',fontsize=11)
    a.set_xticks(range(len(hz)),[str(v) for v in hz],fontsize=8)
    a.set_xlabel('Fresh packets per second')
    a.set_yticks([0,25,50,75,100],['0%','25%','50%','75%','100%'])
    a.set_ylim(-5,105)
    a.legend(loc='lower right',bbox_to_anchor=(1,.08),fontsize=8,frameon=False)
    a.grid(axis='y',alpha=.2)
    subset=[r for r in placement if r['skill']=='original release' and r['rate']==1000]
    mounts=('head','head+FK','base')
    values=[mean(r['roll_rmse_deg'] for r in subset if r['mount']==m) for m in mounts]
    passed=[sum(r['passed'] for r in subset if r['mount']==m) for m in mounts]
    b.barh(range(3),values,color=['#a75525','#507987','#167779'],height=.5)
    b.set_yticks(range(3),['Head','Head + pose','Foot'])
    b.invert_yaxis()
    b.set_xlim(0,max(values)*1.38)
    b.set_xlabel('Foot roll error (RMSE, degrees)')
    b.set_title('Original right catch at 1,000 Hz',loc='left',fontsize=11)
    for i,(v,n) in enumerate(zip(values,passed)):
        inside = v > max(values)*.60
        b.text(v-1 if inside else v+1,i,f'{v:.2f}°  ·  {n}/2 held',fontsize=9,va='center',
               ha='right' if inside else 'left',color='white' if inside else '#173740')
    for ax in (a,b):
        ax.spines[['top','right']].set_visible(False)
    fig.text(.06,.035,'Left: feedback cases qualifying twice with original observations. Right: two releases per mounting with ideal head pose for correction.',fontsize=8)
    fig.savefig(MEDIA/'imu-frequency-overview.png',dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replot',action='store_true',help='Regenerate both figures from the published trial results')
    args = parser.parse_args()
    order = list(CORE) + list(sum(GROUPS.values(),())) + list(HISTORICAL) + list(OPEN_LOOP)
    if args.replot:
        data = json.loads((REPORT/'imu-frequency.json').read_text())
        matrix = classify(data['trials'],order)
        verify_controls(data['trials'],data['placement'])
        figure(matrix)
        overview(matrix,data['placement'])
        return
    motion = rows('extended')
    if (WORK/'reference.jsonl').exists():
        motion = [r for r in motion if r['skill'] not in FULL_LENGTH] + rows('reference')
    raw = rows('core')+motion+rows('historical')+rows('openloop')
    matrix = classify(raw,order)
    raw.sort(key=lambda r:(order.index(r['skill']),RATES.index(r['rate']),r['repeat']))
    extras = {group:rows(group) for group in ('placement','threshold')}
    placements = {(name, mount, rate, repeat) for name in ('original release','40 degree catch','shuffle')
                  for mount in ('base','head','head+FK') for rate in (50,200,1000) for repeat in (0,1)}
    challenges = {('original',rate,i,'wide') for rate in ('stock',50,400,500,1000) for i in range(8)}
    challenges.update(('refit',rate,i,band) for rate in (50,500) for i in range(4) for band in ('wide','narrow'))
    for name, fields, expected in [('placement',('skill','mount','rate','repeat'),placements),
                                   ('threshold',('policy','rate','repeat','band'),challenges)]:
        keys = [tuple(r[k] for k in fields) for r in extras[name]]
        if len(keys) != len(expected) or set(keys) != expected:
            raise ValueError(f'Duplicated or incomplete {name} trials')
    verify_controls(raw,extras['placement'])
    data = dict(evidence='uncalibrated_simulation',command_hz=50,upstream_model=PIN,
                mujoco_version='3.3.7',filter='ahrs==0.4.0 Madgwick',rates_hz=list(RATES[2:]),
                numeric_cutoffs_hz=dict(accelerometer=145,gyro=116),
                matched_phase_fraction=[0,.5],progress_retention_fraction=.8,
                trials=raw,summary=matrix,**extras)
    (REPORT/'imu-frequency.json').write_text(json.dumps(data,allow_nan=False,indent=1)+'\n')
    with (REPORT/'imu-frequency.csv').open('w',newline='') as f:
        fields=['skill','rate','repeat','passed','qualified','value','unit','archive_floor','imu_updates']
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore',lineterminator='\n')
        writer.writeheader()
        writer.writerows(raw)
    figure(matrix)
    overview(matrix,extras['placement'])


if __name__=='__main__':
    main()
