"""Quasistatic no-slip roll of the rigid neutral assembly on its CAD foot contour."""
import argparse
import math
import mujoco
import numpy as np

from slope_balance import WORK, plant, write


def geometry():
    sim=plant(0.,.000125,.7)
    m=sim.model;d=mujoco.MjData(m)
    d.qpos[:]=sim.initial_qpos
    mujoco.mj_forward(m,d)
    g=sim.foot;mesh=m.geom_dataid[g]
    start=m.mesh_vertadr[mesh]
    verts=m.mesh_vert[start:start+m.mesh_vertnum[mesh]]
    world=np.einsum('ij,kj->ik',verts,d.geom_xmat[g].reshape(3,3))+d.geom_xpos[g]
    body=d.xmat[sim.base].reshape(3,3)
    yz=np.einsum('ij,jk->ik',world-d.xpos[sim.base],body)[:,1:]
    com=((d.subtree_com[sim.base]-d.xpos[sim.base])@body)[1:]
    return yz,com,float(m.body_subtreemass[sim.base])


def barrier(points,com,mass,bank,side,step):
    origin=np.array([0.,-min(points[:,1])])
    vertical=np.array([math.sin(math.radians(bank)),math.cos(math.radians(bank))])
    initial=float(vertical@(origin+com))
    best=(0.,0.)
    old=np.eye(2)
    for angle in np.arange(step,55.000001,step):
        theta=math.radians(side*angle)
        new=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        # At a facet tie choose the point which will carry load in the new orientation.
        z=np.einsum('ij,j->i',points,old[1])
        ids=np.flatnonzero(z<=z.min()+1e-10)
        vertex=points[ids[np.argmin(np.einsum('ij,j->i',points[ids],new[1]))]]
        origin+=(old-new)@vertex
        origin[1]=-np.einsum('ij,j->i',points,new[1]).min()
        rise=float(vertical@(origin+new@com))-initial
        if rise>best[0]:best=(rise,side*angle+bank)
        old=new
    return dict(bank=bank,side=side,gravity_barrier_mj=1000*mass*9.81*best[0],
                com_rise_mm=best[0]*1000,absolute_roll_at_peak_deg=best[1])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--step',type=float,default=.1)
    a=p.parse_args();geom=geometry()
    rows=[barrier(*geom,b,s,a.step) for b in (-9,-7,-5,-3,0,3,5,7,9) for s in (-1,1)]
    write(WORK/'barriers.json',rows)
