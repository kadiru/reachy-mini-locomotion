"""Plot per-direction outcomes directly from the published speed study."""
import argparse
from pathlib import Path
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

STYLE = {
    'edge recovery': ('#23795c','o','Recovered on edge'),
    'completed': ('#307b9d','s','Finished, missed hold criterion'),
    'floor contact': ('#b83d36','X','Body hit floor'),
    'joint stop': ('#ce8629','v','Joint limit stopped trial'),
    'unbalanced at impact': ('#7865a1','D','Unqualified before hit'),
    'launch failure': ('#666666','+','Stopped before hit'),
}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('report',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    if args.output.exists():
        p.error('Choose a new output')
    rows=json.loads(args.report.read_text())['hits']
    pairs=[('left',1),('left',-1),('right',1),('right',-1)]
    fig, axes=plt.subplots(2,1,figsize=(12,8),gridspec_kw={'height_ratios':[1.45,1]})
    selected=[r for r in rows if r['seed']==2026112101 and r['launch_s']==5 and r['dt']==.000125]
    for axis, bounds in zip(axes,[(.25,1.85),(1.9,6.1)]):
        for i,(edge,side) in enumerate(pairs):
            level=3-i
            axis.axhline(level,color='#d9dfe2',lw=1,zorder=0)
            rr=sorted([r for r in selected if r['edge']==edge and r['side']==side and r['impact_m_s'] is not None and bounds[0]<=r['impact_m_s']<=bounds[1]],key=lambda r:r['impact_m_s'])
            # Slight vertical offsets keep finely spaced neighboring observations legible.
            for j,r in enumerate(rr):
                color,marker,_=STYLE[r['outcome']]
                axis.scatter(r['impact_m_s'],level+(.14 if j%2 else -.14),s=47,marker=marker,c=color,zorder=3)
        axis.set(xlim=bounds,ylim=(-.48,3.5),yticks=[3,2,1,0],yticklabels=['Left / left','Left / right','Right / left','Right / right'],
                 ylabel='Loaded edge / ball from',xlabel='Measured incoming ball speed (m/s)')
        axis.spines[['right','top']].set_visible(False)
        axis.grid(axis='x',color='#e0e6e8')
    axes[0].set_title('Gentle to moderate hits',loc='left',fontsize=12)
    axes[1].set_title('Higher speeds',loc='left',fontsize=12)
    fig.suptitle('How fast a rolling ball could Reachy Mini withstand?',fontsize=18,y=.99)
    fig.text(.5,.94,'MuJoCo simulation  ·  10 g, 8 cm ball  ·  release at 5 s  ·  0.125 ms physics  ·  nominal sensor seed',ha='center')
    shown=set(r['outcome'] for r in selected)
    handles=[Line2D([],[],color=color,marker=marker,linestyle='',markersize=8,label=name)
             for key,(color,marker,name) in STYLE.items() if key in shown]
    fig.legend(handles=handles,loc='lower center',ncol=2,frameon=False)
    fig.tight_layout(rect=[0,.075,1,.915])
    fig.savefig(args.output,dpi=145)
    plt.close(fig)


if __name__=='__main__':
    main()
