"""Draw the published lap in two world-coordinate views using its recorded poses."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from train_target_pivot import read


def main():
    r=read('experiments/generated/deep-lift/best.json')
    loop=r['metrics']['loop']
    rows=[x for x in r['trace'] if loop['start_s']<=x['time_s']<=loop['end_s']]
    foot=np.array([x['foot_xyz_m'] for x in rows])*100
    cop=np.array([x['contact_world_m'] for x in rows])*100
    com=np.array([x['com_world_m'] for x in rows])*100
    origin=np.array(loop['center_xy_mm'])/10
    fig,(top,side)=plt.subplots(1,2,figsize=(10,4.6))
    low='#ac571c'
    center='#176d79'
    top.plot(*(cop[:,:2]-origin).T,color=low,lw=2.5,label='Loaded rim point on the floor')
    top.plot(*(foot[:,:2]-origin).T,color=center,lw=2.8,label='Foot center')
    top.plot(*((np.vstack((cop[0,:2],foot[0,:2]))-origin).T),'o--',color='#42505b',lw=1,ms=4)
    top.set(aspect='equal',xlabel='Floor x (cm)',ylabel='Floor y (cm)',title='Looking down on one measured lap')
    top.legend(fontsize=9,loc='upper left',bbox_to_anchor=(.01,.99),framealpha=.95)
    top.set_xlim(-8.3,8.3);top.set_ylim(-8.3,9.2)
    # The side plane contains world vertical and the horizontal low-to-center direction.
    direction=foot[0,:2]-cop[0,:2]
    direction/=np.linalg.norm(direction)
    def project(p):
        return [np.dot(p[:2]-cop[0,:2],direction),p[2]]
    contact,base,mass=map(project,(cop[0],foot[0],com[0]))
    far=2*np.array(base)-contact
    side.axhline(0,color='#66717a',lw=1.4)
    side.plot([contact[0],far[0]],[contact[1],far[1]],color=center,lw=5,solid_capstyle='round')
    side.plot([mass[0],mass[0]],[0,mass[1]],'--',color='#596871')
    for pt,c in [(contact,low),(base,center),(mass,'#685598')]:
        side.scatter(*pt,color=c,s=50,zorder=5)
    for pt,label,offset in [(mass,'Whole-robot center of mass',(6,6)),(base,'Foot center',(6,-19)),(contact,'Loaded rim',(8,5))]:
        side.annotate(label,pt,xytext=offset,textcoords='offset points',fontsize=10)
    side.set(xlim=(-3,14),ylim=(-1,14),aspect='equal',xlabel='Distance along foot diameter (cm)',ylabel='Height (cm)',title='Side view at the start of that lap')
    for ax in (top,side):
        ax.spines[['right','top']].set_visible(False)
        ax.grid(alpha=.15)
    fig.suptitle('How a tilted foot can roll in a small circle',fontweight='bold',fontsize=16)
    fig.text(.5,.026,'MuJoCo circular-foot model  •  30.8° median lean  •  3.59 cm foot-center diameter',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.94))
    fig.savefig(Path('media/simulation/deep-roll-support.png'),dpi=130)
    plt.close(fig)


if __name__=='__main__':
    main()
