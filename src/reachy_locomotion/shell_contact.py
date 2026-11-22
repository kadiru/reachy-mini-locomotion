"""Alternative floor contact using the exposed surfaces in the pinned CAD assembly."""

import numpy as np

SHELLS = {
    'body_foot_3dprint', 'body_turning_3dprint', 'body_down_3dprint', 'body_top_3dprint',
    'head_front_3dprint', 'head_back_3dprint', 'head_mic_3dprint', 'antenna',
    'antenna_holder_l_3dprint', 'antenna_holder_r_3dprint', 'antenna_body_3dprint',
    'antenna_interface_3dprint', 'lens_cap_d30_3dprint', 'lens_cap_d40_3dprint',
    'small_lens_d30', 'big_lens_d40', 'glasses_dolder_3dprint',
}


def use_cad_floor(world, base, friction):
    """Use bit 8 for the exposed shell. Preserve robot self-contacts and external balls."""
    base.remove(base.find("geom[@name='foot_contact_proxy']"))
    floor = world.find("geom[@name='floor']")
    floor.set('contype', '0')
    floor.set('conaffinity', '8')
    for geom in world.iter('geom'):
        name = geom.get('name', '')
        if name == 'pendulum_ball' or name.startswith('disturbance_ball_geom_'):
            # Balls collide with the floor, shell, and each other exactly once.
            geom.attrib.update(contype='2', conaffinity='10')
            floor.set('conaffinity', '10')
    count = 0
    for geom in world.iter('geom'):
        mesh = geom.get('mesh')
        if geom.get('class') != 'visual' or mesh not in SHELLS:
            continue
        name = 'foot_contact_proxy' if mesh == 'body_foot_3dprint' else f'floor_shell_{count}_{mesh}'
        geom.attrib.update(name=name, contype='8', conaffinity='0', condim='6', friction=friction)
        count += 1


def place_on_floor(model, data, root_z):
    """Translate vertically until the lowest CAD shell vertex just touches the plane."""
    lowest = float('inf')
    support = None
    for g in range(model.ngeom):
        if model.geom_contype[g] != 8:
            continue
        mesh = model.geom_dataid[g]
        first, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[first:first+count]
        height = np.einsum('ij,j->i', vertices, data.geom_xmat[g].reshape(3,3)[2])
        if not np.all(np.isfinite(height)):
            raise ValueError('Invalid shell vertex height')
        z = data.geom_xpos[g,2] + height.min()
        if z < lowest:
            lowest = z
            support = data.geom_xpos[g] + np.einsum('ij,j->i',data.geom_xmat[g].reshape(3,3),
                                                     vertices[height.argmin()])
    if lowest == float('inf'):
        raise ValueError('No CAD floor shells to place')
    data.qpos[root_z] -= lowest
    support[2] -= lowest
    return support
