"""Fit short sensor-gated head sequences by branching the physical simulator at stroke boundaries."""
import argparse
import copy
import math
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from reachy_locomotion.geometry import euler
from reachy_locomotion.contact_probe import relative_head_pose
from slope_balance import WORK, action_pose, plant, write

STATE=mujoco.mjtState.mjSTATE_INTEGRATION


def snapshot(model,data,servo):
    state=np.empty(mujoco.mj_stateSize(model,STATE))
    mujoco.mj_getState(model,data,state,STATE)
    s=copy.copy(servo)
    s.filtered=servo.filtered.copy()
    s.target_velocity=servo.target_velocity.copy()
    s.queue=servo.queue.copy()
    return state,s


def menu(sim,side,enhanced=False):
    actions=[]
    poses=[(0,0,0),(40,-22,-6),(40,10,-4),(0,25,2),(24,-24,5),(37,-17,5)]
    if enhanced:
        poses.extend([(40,-12,-12),(25,-24,-10),(18,22,12),(0,0,13),(0,0,-13)])
    for roll,y,z in poses:
        for sign in ((1,-1) if roll or y else (1,)):
            for scale in (1.,.92,.82,.7,.55):
                action=[roll*scale*sign,y*scale*sign,z*scale]
                try:
                    joints=sim.checked_ik(action_pose(side,action))
                except ValueError:
                    continue
                targets=sim.initial_targets.copy()
                targets[sim.act]=joints
                actions.append((action,targets))
                break
    return actions


def plan(spec):
    bank,side=spec[:2]
    enhanced=len(spec)>2 and spec[2]
    sim=plant(bank,.000125,.7)
    m=sim.model;d=mujoco.MjData(m)
    d.qpos[:]=sim.initial_qpos;d.qvel[:]=sim.initial_qvel;d.ctrl[:]=sim.initial_ctrl
    servo=sim.servos
    servo.reset(sim.initial_targets)
    for _ in range(round(.5/sim.dt)):
        servo.apply(d);mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d);d.time=0.
    actions=menu(sim,side,enhanced)
    def frame(target):
        servo.command(target)
        for _ in range(sim.substeps):
            servo.apply(d);mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    for _ in range(10):
        frame(actions[0][1])
    initial=snapshot(m,d,servo)
    beam=[dict(state=initial,path=[])]
    shortlist=[]
    explored=0;branches=0;frames=4 if enhanced else 5
    for depth in range(27 if enhanced else 19):
        nodes=[]
        for parent in beam:
            for action_id,(_,target) in enumerate(actions):
                branches+=1
                mujoco.mj_setState(m,d,parent['state'][0],STATE)
                # Reuse only immutable model/config arrays from the parent.
                servo=copy.copy(parent['state'][1])
                servo.filtered=servo.filtered.copy();servo.target_velocity=servo.target_velocity.copy();servo.queue=servo.queue.copy()
                mujoco.mj_forward(m,d)
                bad=False
                for _ in range(frames):
                    old=side*math.degrees(euler(d.xmat[sim.base].reshape(3,3))[0])
                    frame(target)
                    explored+=1
                    phi,pitch,_=np.degrees(euler(d.xmat[sim.base].reshape(3,3)))
                    a=side*phi;v=(a-old)/.02
                    q=d.qpos[sim.joint_qpos]
                    if abs(a)>48 or abs(pitch)>30 or np.any(q<sim.command_limits[:,0]-.003) or np.any(q>sim.command_limits[:,1]+.003):
                        bad=True;break
                if bad: continue
                head=relative_head_pose(m,d)
                head_roll=side*np.degrees(euler(head)[0]);head_y=side*head[1,3]*1000
                cost=math.hypot((a-31.6)/2.3,v/17.)+.018*abs(head_roll-2.3)+.04*abs(head_y+4)
                path=parent['path']+[action_id]
                n=dict(state=snapshot(m,d,servo),path=path,a=a,v=v,cost=cost,reach=a+.09*v,
                       reverse=min(26.,-a-.07*v),hr=head_roll)
                if enhanced:
                    mujoco.mj_subtreeVel(m,d)
                    body=d.xmat[sim.base].reshape(3,3)
                    mass=m.body_subtreemass[sim.base]
                    com=d.subtree_com[sim.base]
                    vel=d.subtree_linvel[sim.base]
                    for name,direction in [('up_energy',side),('return_energy',-side)]:
                        pivot=d.xpos[sim.base]+body@np.array([0.,-direction*.056,0.])
                        dy,dz=(com-pivot)[1:]
                        momentum=direction*(d.subtree_angmom[sim.base,0]+mass*(dy*vel[2]-dz*vel[1]))
                        inertia=mass*(dy*dy+dz*dz)+.004
                        # Signed energy remaining after coasting to the gravity apex.
                        potential=mass*9.81*(math.hypot(dy,dz)-dz)
                        n[name]=momentum*abs(momentum)/(2*inertia)-potential
                    angular=np.zeros(6)
                    mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,sim.base,angular,1)
                    off_axis=.14*abs(pitch)+.008*abs(math.degrees(angular[1]))+.004*abs(math.degrees(angular[2]))
                    n['cost']=off_axis+min(cost,math.hypot((a-27.5)/2.3,v/17.)+.018*abs(head_roll-27)+.04*abs(head_y+19))
                nodes.append(n)
                if a>(14 if enhanced else 20):
                    shortlist.append(dict(cost=n['cost'],actual_roll=a,rate=v,path=path))
        if not nodes:break
        beam=[];used=set()
        # Keep both directions of rocking and approaches that arrive nearly stopped.
        for priority in (('cost','up_energy','return_energy','reach','reverse') if enhanced else ('cost','reach','reverse')):
            chosen=0
            for n in sorted(nodes,key=lambda n:n[priority],reverse=priority!='cost'):
                key=(round(n['a']/2),round(n['v']/16),round(n['hr']/12))
                if key in used:continue
                used.add(key);beam.append(n);chosen+=1
                if chosen==(2 if enhanced else 3):break
        # Stop expanding the tree once it contains several quiet arrivals.
        if depth>=9 and sum(x['cost']<1.25 for x in shortlist)>=12:
            break
    selected=[];bins=set()
    for v in sorted(shortlist,key=lambda x:x['cost']):
        key=(len(v['path']),v['path'][-1],round(v['rate']/10))
        if key in bins:continue
        bins.add(key)
        p=dict(actions=[actions[i][0] for i in v['path']],frames=frames,
               unlock=.20+(len(v['path'])-1)*frames*.02,trigger=28.,min_trigger=20.,lead=.055,catch_model='plane')
        selected.append(dict(settings=p,roll=v['actual_roll'],rate=v['rate'],cost=v['cost']))
        if len(selected)==6:break
    return dict(bank=bank,side=side,branches=branches,physics_control_frames=explored,simulated_s=.02*explored+.7,
                depth=depth+1,nearest=selected)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bank',type=float,nargs='+',default=[5.,-5.,0.,9.,-9.])
    ap.add_argument('--workers',type=int,default=6)
    ap.add_argument('--output',default='plans')
    a=ap.parse_args()
    rows=[]
    with ProcessPoolExecutor(a.workers) as pool:
        for row in pool.map(plan,[(b,s) for b in a.bank for s in (-1,1)]):
            rows.append(row);write(WORK/(a.output+'.json'),rows)
