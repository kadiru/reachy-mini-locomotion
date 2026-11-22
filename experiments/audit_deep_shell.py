"""Place the published proxy lap's poses in the CAD scene and check support geometry."""
from pathlib import Path

import mujoco
import numpy as np

from reachy_locomotion.contact_probe import scene_xml
from train_target_pivot import read, write
from tilted_release import WORK, UPSTREAM


def summary(x):
    return dict(min=float(np.min(x)),median=float(np.median(x)),max=float(np.max(x)))


def audit(record):
    run = read(record)
    cfg = dict(run['config'],cad_shell_floor=True)
    model = mujoco.MjModel.from_xml_string(scene_xml(UPSTREAM,cfg)[0])
    data = mujoco.MjData(model)
    pieces = {}
    for g in range(model.ngeom):
        if model.geom_contype[g] == 8:
            mesh = model.geom_dataid[g]
            start, n = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            pieces[g] = np.unique(model.mesh_vert[start:start+n].astype(float),axis=0)
    foot = model.geom('foot_contact_proxy').id
    loop = run['metrics']['loop']
    com_distance, foot_clearance, other_clearance, support_offset = [], [], [], []
    surfaces = set()
    for row, pose in zip(run['trace'],run['trajectory']):
        if not loop['start_s'] <= row['time_s'] <= loop['end_s']:
            continue
        data.qpos[:] = pose
        mujoco.mj_kinematics(model,data)
        contact_xy = np.array(row['contact_world_m'][:2])
        com_distance.append(1000*np.linalg.norm(np.array(row['com_world_m'][:2])-contact_xy))
        z_min = np.inf
        for g,vertices in pieces.items():
            mat = data.geom_xmat[g].reshape(3,3)
            z = data.geom_xpos[g,2] + np.einsum('ij,j->i',vertices,mat[2])
            if g == foot:
                foot_clearance.append(float(z.min()*1000))
                # A near-level facet has multiple equally plausible support vertices.
                low = vertices[z<=z.min()+.00002]
                centroid = data.geom_xpos[g,:2] + np.einsum('ij,j->i',mat[:2],low.mean(axis=0))
                support_offset.append(1000*np.linalg.norm(centroid-contact_xy))
            elif z.min() < z_min:
                z_min = z.min()
                closest = model.mesh(model.geom_dataid[g]).name
        surfaces.add(closest)
        other_clearance.append(float(z_min*1000))
    return dict(frames=len(com_distance),source_lap=loop,proxy_com_to_contact_mm=summary(com_distance),
                cad_foot_signed_gap_mm=summary(foot_clearance),cad_other_signed_gap_mm=summary(other_clearance),
                cad_lowest_patch_to_proxy_contact_mm=summary(support_offset),nearest_other_parts=sorted(surfaces))


if __name__=='__main__':
    write(WORK/'geometry.json',audit(Path('experiments/generated/deep-lift/best.json')))
