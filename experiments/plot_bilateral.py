"""Plot recorded motion and every frozen qualification outcome, including failures."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

root=Path(__file__).resolve().parents[1]
r=json.loads((root/'experiments/generated/bilateral/penguin-two-cycles.json').read_text())
q=json.loads((root/'experiments/reports/motor-training-policy-bank-qualified-2026-11-19.json').read_text())
rows=r['trajectory'];t=np.array([x['time_s'] for x in rows]);xyz=np.array([x['foot_xyz_m'] for x in rows]);yaw=np.degrees([x['heading_from_startup_rad'] for x in rows])
roll=np.degrees([x['foot_rpy_rad'][0] for x in rows]);ends=[s['end_s'] for i,s in enumerate(r['penguin']['steps']) if i%2]
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
a=axes[0,0];a.plot(t,(xyz[:,0]-xyz[0,0])*1000,color='#168a65');a.set(title='Two pairs / eight bursts: 88.6 mm forward',ylabel='Forward (mm)',xlabel='Time (s)')
b=axes[1,0];b.plot(t,roll,color='#5969c8');b.set(title='Alternating body roll; no state resets',ylabel='Base roll (degrees)',xlabel='Time (s)');b.axhline(0,color='gray',lw=.5)
c=axes[0,1];c.plot(t,yaw,color='#b47720');c.axhline(20,color='gray',ls='--');c.axhline(-20,color='gray',ls='--');c.set(title='Heading limit applies at pair ends',ylabel='Base yaw from startup (degrees)',xlabel='Time (s)')
for ax in [a,b,c]:
 for end in ends:ax.axvline(end,color='gray',ls=':',alpha=.6)
 ax.grid(alpha=.2)
c.scatter(ends,r['penguin']['cycle_end_headings_deg'],c='#168a65',s=60,zorder=3)
d=axes[1,1];names=[f'development-{i}' for i in range(5)]+[f'fresh-{i}' for i in range(8)]
values=np.zeros((len(names),4))
for row,name in enumerate(names):
 for col,(pivot,side) in enumerate([(True,-1),(True,1),(False,-1),(False,1)]):
  check=next(x for x in q['checks'] if x['id']==name and x['pivot_only']==pivot and x['side']==side)
  values[row,col]=0 if check['passed'] else 2 if check['result']['termination']!='completed' else 1
cmap=ListedColormap(['#69b88b','#e3bd6a','#d76f69']);d.imshow(values,cmap=cmap,vmin=0,vmax=2,aspect='auto')
d.set(yticks=range(len(names)),yticklabels=names,xticks=range(4),xticklabels=['Pivot L','Pivot R','Walk L','Walk R'],title='Frozen qualification at 0.25 ms')
for i in range(len(names)):
 for j in range(4):d.text(j,i,['PASS','WEAK','GUARD'][int(values[i,j])],ha='center',va='center',fontsize=8)
d.tick_params(labelsize=8)
fig.suptitle('Bilateral roll/rate feedback | fixed waist target | uncalibrated simulation\nRecovery 26/26; pivot/walk development 20/20; fresh pivot 13/16, walking 13/16',fontsize=13)
out=root/'media/simulation/bilateral-penguin-2026-11-19-chart.png'
if out.exists():raise FileExistsError(out)
fig.savefig(out,dpi=160)
