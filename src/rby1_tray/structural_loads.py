"""Recorded sensor-load selection and explicit reaction-wrench transport."""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


def transfer_sensor_wrench(force_n, moment_nm, rotation, site_origin_m, load_origin_m):
    """Transport the reaction of a child-side sensor wrench to the CAD load plane.

    All origins are expressed in CAD coordinates. ``rotation`` maps sensor
    components to CAD components. The negative sign represents the distal
    subtree acting on the proximal component, rather than the parent supporting
    the sensor's child subtree.
    """
    force, moment = np.asarray(force_n, dtype=float), np.asarray(moment_nm, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    site, load = np.asarray(site_origin_m, dtype=float), np.asarray(load_origin_m, dtype=float)
    if any(value.shape != (3,) for value in (force, moment, site, load)):
        raise ValueError('wrench and origin vectors must have shape (3,)')
    if rotation.shape != (3,3) or not all(np.all(np.isfinite(v)) for v in (force,moment,rotation,site,load)):
        raise ValueError('wrench transform must have finite vectors and a 3x3 rotation')
    if not np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-10) or not np.isclose(np.linalg.det(rotation),1,rtol=0,atol=1e-10):
        raise ValueError('wrench rotation must be proper orthonormal')
    rotated_force = rotation@force
    return -rotated_force, -(rotation@moment+np.cross(site-load,rotated_force))


def select_recorded_load(path: Path, phases) -> dict[str, Any]:
    """Select maximum local moment norm in declared phases, before FEM solve."""
    chosen, maximum, count = None, -1.0, 0
    with gzip.open(path, 'rt') as stream:
        for index,line in enumerate(stream):
            row=json.loads(line)
            if row['phase'] not in phases:
                continue
            wrench=row['observation']['wrist_wrench_local']['left']
            force=np.asarray(wrench['force_n'],dtype=float)
            torque=np.asarray(wrench['torque_nm'],dtype=float)
            timestamp=float(row['time_s'])
            if force.shape!=(3,) or torque.shape!=(3,) or not np.all(np.isfinite([force,torque])) or not np.isfinite(timestamp) or timestamp<0:
                raise ValueError('recorded wrench and time must be finite SI quantities')
            value=float(np.linalg.norm(torque))
            count+=1
            if value>maximum:
                maximum=value
                chosen=dict(packet_index=index,timestamp_sim_s=timestamp,phase=row['phase'],
                            sensor='left_wrist_ft',raw_wrench=wrench)
    if chosen is None:
        raise ValueError('no load packets match the selected mission phases')
    return dict(**chosen,selected_packet_count=count,
                selection='maximum norm of raw left local wrist torque in configured phases',
                commands_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def validate_mesh_sizes(values):
    """Require three or more positive, strictly decreasing refinement sizes."""
    sizes=np.asarray(values,dtype=float)
    if sizes.ndim!=1 or len(sizes)<3 or not np.all(np.isfinite(sizes)) or np.any(sizes<=0) or np.any(np.diff(sizes)>=0):
        raise ValueError('mesh_sizes_m must contain at least three decreasing positive finite sizes')
    return sizes.tolist()
