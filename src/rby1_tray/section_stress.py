"""Explicit equivalent circular-section screening; this is not whole-CAD FEM."""
from __future__ import annotations

import numpy as np


def section_basis(axis):
    axis=np.asarray(axis,dtype=float)
    if axis.shape!=(3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis)<1e-12:
        raise ValueError('section axis must be a finite nonzero 3-vector')
    z=axis/np.linalg.norm(axis)
    helper=np.eye(3)[int(np.argmin(np.abs(z)))]
    x=np.cross(helper,z);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))


def circular_section(radius_m,thickness_m):
    radius,thickness=float(radius_m),float(thickness_m)
    if not np.isfinite(radius+thickness) or not 0<thickness<=radius:
        raise ValueError('section requires positive radius and 0 < thickness <= radius')
    inner=radius-thickness
    area=np.pi*(radius**2-inner**2)
    inertia=np.pi*(radius**4-inner**4)/4
    return dict(radius_m=radius,thickness_m=thickness,area_m2=float(area),I_m4=float(inertia),J_m4=float(2*inertia))


def section_von_mises(points_section_m,force_n,moment_nm,section):
    points=np.asarray(points_section_m,dtype=float)
    force=np.asarray(force_n,dtype=float);moment=np.asarray(moment_nm,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or force.shape!=(3,) or moment.shape!=(3,) or not all(np.all(np.isfinite(x)) for x in (points,force,moment)):
        raise ValueError('section field requires finite points and 3-axis wrench')
    x,y,z=points.T
    # Move the proximal-resultant moment to each axial station. Distributed
    # body inertia/contact within a link is not resolved by this beam model.
    mx=moment[0]+z*force[1];my=moment[1]-z*force[0]
    normal=force[2]/section['area_m2']+(mx*y-my*x)/section['I_m4']
    shear_x=force[0]/section['area_m2']-moment[2]*y/section['J_m4']
    shear_y=force[1]/section['area_m2']+moment[2]*x/section['J_m4']
    return np.sqrt(normal**2+3*(shear_x**2+shear_y**2))


def body_interaction_wrench(model,data,body_id,origin_world_m,rotation_world):
    """Parent-on-child, expressed at a requested origin in a requested frame.

    MuJoCo c-vectors use the root tree's subtree COM and world-aligned axes;
    array order is torque then force. Call on a forward/postconstraint scratch
    state, never treat cfrc_int as a body-origin/body-local wrench.
    """
    origin=np.asarray(origin_world_m,dtype=float)
    rotation=np.asarray(rotation_world,dtype=float)
    if origin.shape!=(3,) or rotation.shape!=(3,3) or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-10):
        raise ValueError('requested wrench frame must be orthonormal')
    root=int(model.body_rootid[body_id])
    c_origin=np.asarray(data.subtree_com[root])
    moment=np.asarray(data.cfrc_int[body_id,:3]);force=np.asarray(data.cfrc_int[body_id,3:])
    at_origin=moment+np.cross(c_origin-origin,force)
    return rotation.T@force,rotation.T@at_origin
