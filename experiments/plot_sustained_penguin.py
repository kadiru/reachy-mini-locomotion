"""Plot recorded sustained motion and every supplied qualification outcome."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('record');p.add_argument('--output',required=True)
    p.add_argument('--reports',nargs='+',required=True);args=p.parse_args()
    out=Path(args.output)
    if out.exists():raise FileExistsError(out)
    r=json.loads(Path(args.record).read_text());s=r['penguin'];end=s['steps'][-1]['end_s']
    rows=[x for x in r['trajectory'] if x['time_s']<=end+1]
    t=np.array([x['time_s'] for x in rows]);xyz=np.array([x['foot_xyz_m'] for x in rows])
    fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    ax[0,0].plot(t,1000*(xyz[:,0]-xyz[0,0]),color='#148b72')
    ax[0,0].set(title=f"{len(s['pairs'])} complete pairs · {s['speed_mm_s']:.3f} mm/s",xlabel='Time (s)',ylabel='Forward (mm)')
    ax[0,1].plot(t,np.degrees([x['heading_from_startup_rad'] for x in rows]),color='#b37b26',lw=1)
    ends=s['steps'][1::2];ax[0,1].scatter([x['end_s'] for x in ends],[x['heading_deg'] for x in ends],c='#148b72',s=20,zorder=3,label='Completed pair')
    for bound in [-20,20]:ax[0,1].axhline(bound,color='gray',ls='--',label='Cycle-end limit' if bound==20 else None)
    ax[0,1].legend(fontsize=8,loc='lower left')
    ax[0,1].set(title='Heading from the original startup reference',xlabel='Time (s)',ylabel='Base yaw (degrees)')
    ax[1,0].plot(t,np.degrees([x['foot_rpy_rad'][0] for x in rows]),color='#5969c8',lw=1)
    ax[1,0].set(title='Body rocking throughout the walk',xlabel='Time (s)',ylabel='Body roll (degrees)')
    groups={}
    for path in args.reports:
        report=json.loads(Path(path).read_text());assert report['completed']
        for c in report['checks']:
            groups.setdefault(c['variant'],[]).append(c)
    labels=[];counts=[]
    for label,cs in groups.items():
        labels.append({'twenty-pairs-0.00025':'20 pairs (0.25 ms)',
                       'twenty-pairs-0.000125':'20 pairs (0.125 ms)'}.get(label,label.replace('-',' ')))
        counts.append([sum(c['passed'] for c in cs),sum(not c['passed'] and c['result']['termination']=='completed' for c in cs),sum(c['result']['termination']!='completed' for c in cs)])
    counts=np.array(counts);left=np.zeros(len(labels))
    for k,(color,label) in enumerate([('#69b88b','Pass'),('#e3bd6a','Missed goal'),('#d76f69','Guard stop')]):
        ax[1,1].barh(labels,counts[:,k],left=left,color=color,label=label)
        for i,n in enumerate(counts[:,k]):
            if n:ax[1,1].text(left[i]+n/2,i,str(n),ha='center',va='center',fontsize=9)
        left+=counts[:,k]
    ax[1,1].set(title='All qualification outcomes',xlabel='Trials');ax[1,1].legend(fontsize=8)
    for a in [ax[0,0],ax[0,1],ax[1,0]]:a.grid(alpha=.2)
    fig.suptitle('Sustained penguin walking · fixed waist target · unchanged motor limits\nUncalibrated simulation; green heading markers are complete left/right pairs')
    fig.savefig(out,dpi=160)
