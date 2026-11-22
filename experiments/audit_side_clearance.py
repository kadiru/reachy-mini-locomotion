"""Check minimum head clearance and which part of the torso faces the floor in a filmed trial."""
import argparse
import numpy as np
import mujoco

from tilted_release import WORK, UPSTREAM, read, write
from reachy_locomotion.contact_probe import scene_xml


def audit(path):
    r=read(path)
    m=mujoco.MjModel.from_xml_string(scene_xml(UPSTREAM,r['config'])[0])
    d=mujoco.MjData(m)
    floor=m.geom('floor').id
    torso=m.body('body_down_3dprint').id
    head=[]
    panels=[]
    for g in np.flatnonzero(m.geom_contype==8):
        name=m.mesh(m.geom_dataid[g]).name
        if name in ('body_down_3dprint','body_top_3dprint','body_turning_3dprint'):
            panels.append(g)
        elif name!='body_foot_3dprint':
            head.append(g)
    gap=[]
    azimuth=[]
    point=np.zeros(6)
    start=r['metrics']['job'].get('drive_s',2.)
    for row in r['trajectory']:
        d.qpos[:]=row['qpos']
        mujoco.mj_forward(m,d)
        gap.append(1000*min(mujoco.mj_geomDistance(m,d,floor,g,1.,None) for g in head))
        if row['time_s']>=start:
            g=min(panels,key=lambda g:mujoco.mj_geomDistance(m,d,floor,g,1.,None))
            mujoco.mj_geomDistance(m,d,floor,g,1.,point)
            local=d.xmat[torso].reshape(3,3).T@(point[3:]-d.xpos[torso])
            azimuth.append(np.arctan2(local[1],local[0]))
    a=np.degrees(np.unwrap(azimuth))
    return dict(clip=path.stem,minimum_head_floor_clearance_mm=min(gap),
                minimum_during_drive_mm=min(gap[round(start*50):]),sample_hz=50,
                closest_torso_surface_azimuth_span_deg=float(np.ptp(a)),
                closest_torso_surface_azimuth_net_deg=float(a[-1]-a[0]))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('clips',nargs='+')
    args=p.parse_args()
    write(WORK/'clearance.json',[audit(WORK/(clip+'.json')) for clip in args.clips])
