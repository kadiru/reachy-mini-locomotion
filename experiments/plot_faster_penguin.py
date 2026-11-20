"""Compare measured stepping and show every frozen stress-test outcome."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

root=Path(__file__).resolve().parents[1]
def read(name):return json.loads((root/name).read_text())
new=read('experiments/generated/faster-balance/best-left-four-pairs.json')
old=read('experiments/generated/bilateral/penguin-two-cycles.json')
q=read('experiments/reports/motor-training-faster-balance-feedback-qualified-2026-11-19.json')
assert q['completed']
fig,axs=plt.subplots(2,2,figsize=(12,9),layout='constrained')
for r,color,label in [(old,'#68778e',f"Previous: {old['penguin']['speed_mm_s']:.2f} mm/s"),(new,'#178d6b',f"New: {new['penguin']['speed_mm_s']:.2f} mm/s")]:
 end=r['penguin']['steps'][-1]['end_s'];rows=[x for x in r['trajectory'] if x['time_s']<=end+1]
 t=np.array([x['time_s'] for x in rows]);xyz=np.array([x['foot_xyz_m'] for x in rows])
 axs[0,0].plot(t,(xyz[:,0]-xyz[0,0])*1000,label=label,color=color)
gain=100*(new['penguin']['speed_mm_s']/old['penguin']['speed_mm_s']-1)
axs[0,0].set(title=f'{gain:.1f}% higher speed during stepping',xlabel='Time (s)',ylabel='Forward (mm)');axs[0,0].legend()
rows=[x for x in new['trajectory'] if x['time_s']<=new['penguin']['steps'][-1]['end_s']+1];t=[x['time_s'] for x in rows]
axs[1,0].plot(t,np.degrees([x['foot_rpy_rad'][0] for x in rows]),color='#5969c8')
axs[1,0].set(title='Body rocking preserved',xlabel='Time (s)',ylabel='Body roll (degrees)')
axs[0,1].plot(t,np.degrees([x['heading_from_startup_rad'] for x in rows]),color='#ba7b20')
endsteps=new['penguin']['steps'][1::2]
axs[0,1].scatter([s['end_s'] for s in endsteps],[s['heading_deg'] for s in endsteps],c='#178d6b',s=50,zorder=3)
axs[0,1].axhline(20,ls='--',c='gray');axs[0,1].axhline(-20,ls='--',c='gray')
axs[0,1].set(title=f"Cycle-end heading stays within {new['penguin']['max_completed_cycle_heading_deg']:.2f}°",xlabel='Time (s)',ylabel='Base yaw from startup (degrees)')
for a in [axs[0,0],axs[1,0],axs[0,1]]:a.grid(alpha=.2)
names=[f'development-{i}' for i in range(5)]+[f'fresh-{i}' for i in range(8)]+[f'unseen-{i}' for i in range(8)]
labels=[n.replace('development-','Original ').replace('fresh-','Promoted ').replace('unseen-','Unseen ') for n in names]
values=np.zeros((len(names),2));counts={}
for i,name in enumerate(names):
 for k,side in enumerate([-1,1]):
  c=next(c for c in q['checks'] if c['id']==name and c['side']==side)
  values[i,k]=0 if c['passed'] else 2 if c['result']['termination']!='completed' else 1
axs[1,1].imshow(values,cmap=ListedColormap(['#69b88b','#e3bd6a','#d76f69']),vmin=0,vmax=2,aspect='auto')
axs[1,1].set(yticks=range(len(names)),yticklabels=labels,xticks=[0,1],xticklabels=['Start left','Start right'],title='All single-pair checks at 0.25 ms')
axs[1,1].tick_params(labelsize=8)
for i in range(len(names)):
 for j in range(2):axs[1,1].text(j,i,['PASS','GOAL','GUARD'][int(values[i,j])],ha='center',va='center',fontsize=8)
axs[1,1].axhline(12.5,c='white',lw=2)
fig.suptitle('Faster penguin steps | fixed waist target | unchanged motor limits\nUncalibrated simulation; green markers are complete left-right pairs',fontsize=14)
out=root/'media/simulation/faster-penguin-2026-11-19-chart.png'
if out.exists():raise FileExistsError(out)
fig.savefig(out,dpi=160)
