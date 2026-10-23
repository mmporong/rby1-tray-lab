"""Verify a solved mesh field before rendering a structural result."""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from .structural import volume_weighted_quantile


def load_verified_level(directory: Path, report: dict):
    levels = report['mesh_levels']
    index = len(levels)-1
    name = f'level-{index}.npz'
    path = directory/name
    expected = report.get('level_artifact_sha256',{}).get(name)
    if not expected or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise ValueError('solved mesh field SHA-256 does not match the analysis report')
    with np.load(path,allow_pickle=False) as archive:
        data={key:archive[key].copy() for key in archive.files}
    nodes,tetra,displacement,stress,volume=(data[key] for key in
        ('nodes_m','tetra','displacement_m','von_mises_pa','element_volume_m3'))
    if (nodes.ndim!=2 or nodes.shape[1:]!=(3,) or displacement.shape!=nodes.shape
        or tetra.ndim!=2 or tetra.shape[1:]!=(4,) or not np.issubdtype(tetra.dtype,np.integer)
        or stress.shape!=(len(tetra),) or volume.shape!=stress.shape
        or np.any(tetra<0) or np.any(tetra>=len(nodes))
        or any(not np.all(np.isfinite(value)) for value in (nodes,displacement,stress,volume))
        or np.any(stress<0) or np.any(volume<=0)):
        raise ValueError('solved mesh fields have invalid shape or values')
    if len(nodes)!=levels[index]['nodes'] or len(tetra)!=levels[index]['tetrahedra']:
        raise ValueError('solved mesh counts differ from the analysis report')
    observed=dict(max_displacement_m=float(np.linalg.norm(displacement,axis=1).max()),
                  peak_von_mises_pa=float(stress.max()),
                  volume_p95_von_mises_pa=volume_weighted_quantile(stress,volume,.95),
                  mesh_volume_m3=float(volume.sum()))
    for key,value in observed.items():
        for expected_metrics in (levels[index],report['metrics']):
            if not np.isclose(value,expected_metrics[key],rtol=1e-10,atol=1e-30):
                raise ValueError(f'solved field {key} differs from the analysis report')
    return data
