"""Linear structural response bases for synchronized six-axis wrench playback.

The basis stores stress tensors, not von Mises scalars.  A recorded wrench is
therefore combined in tensor space before the nonlinear von Mises reduction.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from .structural import distribute_wrench_to_nodes, solve_linear_elasticity


WRENCH_COMPONENTS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
VOIGT_COMPONENTS = ("xx", "yy", "zz", "xy", "yz", "zx")
VERTEX_FLOATS = 42


def sha256_file(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def von_mises(stress_pa_voigt):
    stress = np.asarray(stress_pa_voigt, dtype=float)
    if stress.shape[-1] != 6 or not np.all(np.isfinite(stress)):
        raise ValueError("stress_pa_voigt must be finite with six Voigt components")
    xx, yy, zz, xy, yz, zx = np.moveaxis(stress, -1, 0)
    return np.sqrt(
        0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2)
        + 3.0 * (xy**2 + yz**2 + zx**2)
    )


def extract_boundary_faces(tetra):
    """Return oriented boundary triangles and their parent tetrahedra."""
    tetra = np.asarray(tetra)
    if tetra.ndim != 2 or tetra.shape[1:] != (4,) or tetra.shape[0] == 0:
        raise ValueError("tetra must have shape (M, 4) with M > 0")
    if not np.issubdtype(tetra.dtype, np.integer):
        raise ValueError("tetra must contain integer indices")
    tetra = np.asarray(tetra, dtype=np.int64)
    local_faces = ((1, 2, 3), (0, 3, 2), (0, 1, 3), (0, 2, 1))
    faces = np.concatenate([tetra[:, local] for local in local_faces])
    parents = np.tile(np.arange(len(tetra), dtype=np.int64), 4)
    _, first, counts = np.unique(
        np.sort(faces, axis=1), axis=0, return_index=True, return_counts=True
    )
    if np.any(counts > 2):
        raise ValueError("tetra mesh has a non-manifold face")
    selected = first[counts == 1]
    return faces[selected], parents[selected]


def select_planar_load_patch(nodes_m, tetra, *, z_m=0.0, tolerance_m=2e-7):
    """Recover the exterior triangles lying on the distal load plane."""
    nodes = np.asarray(nodes_m, dtype=float)
    if nodes.ndim != 2 or nodes.shape[1:] != (3,) or not np.all(np.isfinite(nodes)):
        raise ValueError("nodes_m must be a finite (N, 3) array")
    tolerance_m = float(tolerance_m)
    if not np.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("tolerance_m must be finite and positive")
    triangles, _ = extract_boundary_faces(tetra)
    mask = np.all(np.abs(nodes[triangles, 2] - float(z_m)) <= tolerance_m, axis=1)
    patch_triangles = triangles[mask]
    if len(patch_triangles) == 0:
        raise ValueError("no boundary triangles lie on the requested load plane")
    return patch_triangles


def area_weighted_patch(nodes_m, triangles):
    nodes = np.asarray(nodes_m, dtype=float)
    triangles = np.asarray(triangles, dtype=np.int64)
    if triangles.ndim != 2 or triangles.shape[1:] != (3,) or len(triangles) == 0:
        raise ValueError("triangles must have shape (K, 3) with K > 0")
    cross = np.cross(
        nodes[triangles[:, 1]] - nodes[triangles[:, 0]],
        nodes[triangles[:, 2]] - nodes[triangles[:, 0]],
    )
    areas = np.linalg.norm(cross, axis=1) / 2.0
    if np.any(areas <= 0) or not np.all(np.isfinite(areas)):
        raise ValueError("load patch contains a zero-area triangle")
    weights = np.zeros(len(nodes), dtype=float)
    for column in range(3):
        np.add.at(weights, triangles[:, column], areas / 3.0)
    patch = np.flatnonzero(weights > 0)
    return patch, weights[patch], float(np.sum(areas))


def _unit_wrench(index):
    force = np.zeros(3)
    moment = np.zeros(3)
    if index < 3:
        force[index] = 1.0
    else:
        moment[index - 3] = 1.0
    return force, moment


def _relative_validation(result, force_n, moment_nm, characteristic_length_m):
    residual = result["equilibrium_residual"]
    scale_n = max(
        float(np.linalg.norm(force_n)),
        float(np.linalg.norm(moment_nm)) / characteristic_length_m,
        1.0,
    )
    equilibrium = np.linalg.norm(
        np.concatenate(
            (
                residual["force_n"],
                residual["moment_nm"] / characteristic_length_m,
            )
        )
    ) / scale_n
    work = float(result["external_work_j"])
    energy = abs(2.0 * float(result["strain_energy_j"]) - work) / max(abs(work), 1e-30)
    return float(equilibrium), float(energy)


def solve_wrench_basis(
    nodes_m,
    tetra,
    support_node_ids,
    load_triangles,
    youngs_modulus_pa,
    poisson_ratio,
    *,
    origin_m=(0.0, 0.0, 0.0),
    validation_tolerance=1e-7,
):
    """Solve six unit-wrench cases using identical mesh, material, and support."""
    nodes = np.asarray(nodes_m, dtype=float)
    tetra = np.asarray(tetra, dtype=np.int64)
    support = np.asarray(support_node_ids, dtype=np.int64)
    origin = np.asarray(origin_m, dtype=float)
    patch, weights, area_m2 = area_weighted_patch(nodes, load_triangles)
    center = np.average(nodes[patch], axis=0, weights=weights)
    characteristic_length_m = float(
        np.sqrt(np.average(np.sum((nodes[patch] - center) ** 2, axis=1), weights=weights))
    )
    stress_basis = np.empty((6, len(tetra), 6), dtype=float)
    displacement_basis = np.empty((6, len(nodes), 3), dtype=float)
    validations = []
    for index, component in enumerate(WRENCH_COMPONENTS):
        force, moment = _unit_wrench(index)
        loads = distribute_wrench_to_nodes(
            nodes, patch, weights, force, moment, origin_m=origin
        )
        result = solve_linear_elasticity(
            nodes,
            tetra,
            youngs_modulus_pa,
            poisson_ratio,
            support,
            loads,
            moment_origin_m=origin,
        )
        equilibrium, energy = _relative_validation(
            result, force, moment, characteristic_length_m
        )
        if max(equilibrium, energy) > validation_tolerance:
            raise ValueError(f"{component} basis failed equilibrium or energy validation")
        stress_basis[index] = result["element_stress_pa_voigt"]
        displacement_basis[index] = result["displacement_m"]
        validations.append(
            {
                "component": component,
                "equilibrium_relative": equilibrium,
                "energy_identity_relative": energy,
                "strain_energy_j": float(result["strain_energy_j"]),
            }
        )
    return {
        "stress_basis_pa_voigt": stress_basis,
        "displacement_basis_m": displacement_basis,
        "load_patch_node_ids": patch,
        "load_patch_weights_m2": weights,
        "load_patch_area_m2": area_m2,
        "load_patch_characteristic_length_m": characteristic_length_m,
        "validation": validations,
    }


def combine_stress_basis(coefficients, stress_basis_pa_voigt):
    coefficients = np.asarray(coefficients, dtype=float)
    basis = np.asarray(stress_basis_pa_voigt, dtype=float)
    if coefficients.shape[-1:] != (6,):
        raise ValueError("coefficients must end with six wrench components")
    if basis.ndim != 3 or basis.shape[0] != 6 or basis.shape[2] != 6:
        raise ValueError("stress basis must have shape (6, elements, 6)")
    if not np.all(np.isfinite(coefficients)) or not np.all(np.isfinite(basis)):
        raise ValueError("coefficients and basis must be finite")
    return np.einsum("...b,bec->...ec", coefficients, basis, optimize=True)


def compute_frame_metrics(
    coefficients,
    stress_basis_pa_voigt,
    displacement_basis_m,
    *,
    chunk_size=64,
):
    """Return maximum von Mises and displacement for each recorded frame."""
    coefficients = np.asarray(coefficients, dtype=float)
    stress_basis = np.asarray(stress_basis_pa_voigt, dtype=float)
    displacement_basis = np.asarray(displacement_basis_m, dtype=float)
    if coefficients.ndim != 2 or coefficients.shape[1:] != (6,):
        raise ValueError("coefficients must have shape (frames, 6)")
    if displacement_basis.ndim != 3 or displacement_basis.shape[0] != 6 or displacement_basis.shape[2] != 3:
        raise ValueError("displacement basis must have shape (6, nodes, 3)")
    if stress_basis.ndim != 3 or stress_basis.shape[0] != 6 or stress_basis.shape[2] != 6:
        raise ValueError("stress basis must have shape (6, elements, 6)")
    chunk_size = int(chunk_size)
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    maximum_stress = np.empty(len(coefficients), dtype=float)
    maximum_displacement = np.empty(len(coefficients), dtype=float)
    for start in range(0, len(coefficients), chunk_size):
        stop = min(start + chunk_size, len(coefficients))
        chunk = coefficients[start:stop]
        stress = combine_stress_basis(chunk, stress_basis)
        displacement = np.einsum("fb,bnc->fnc", chunk, displacement_basis, optimize=True)
        maximum_stress[start:stop] = np.max(von_mises(stress), axis=1)
        maximum_displacement[start:stop] = np.max(
            np.linalg.norm(displacement, axis=2), axis=1
        )
    return {
        "max_von_mises_pa": maximum_stress,
        "max_displacement_m": maximum_displacement,
    }


def _triangle_normals(vertices):
    normals = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
    magnitude = np.linalg.norm(normals, axis=1)
    if np.any(magnitude <= 0):
        raise ValueError("output contains a zero-area triangle")
    return normals / magnitude[:, None]


def _pack_vertices(vertices, normals, parent_ids, stress_basis):
    expanded_stress = np.moveaxis(stress_basis[:, parent_ids, :], 0, 1).reshape(
        len(parent_ids), 36
    )
    rows = np.concatenate((vertices, normals, expanded_stress[:, None, :].repeat(3, axis=1)), axis=2)
    packed = np.asarray(rows.reshape(-1, VERTEX_FLOATS), dtype="<f4")
    if not np.all(np.isfinite(packed)):
        raise ValueError("packed structural vertices contain non-finite values")
    return packed


def expand_surface_vertices(nodes_m, tetra, stress_basis_pa_voigt):
    triangles, parents = extract_boundary_faces(tetra)
    vertices = np.asarray(nodes_m, dtype=float)[triangles]
    normals = _triangle_normals(vertices)[:, None, :].repeat(3, axis=1)
    return _pack_vertices(vertices, normals, parents, np.asarray(stress_basis_pa_voigt)), triangles, parents


def build_cutaway_vertices(
    nodes_m,
    tetra,
    stress_basis_pa_voigt,
    *,
    plane_normal=(1.0, 1.0, 0.0),
    plane_offset_m=0.0,
    tolerance_m=1e-12,
):
    """Triangulate exact tet/plane intersections and inherit the parent stress basis."""
    nodes = np.asarray(nodes_m, dtype=float)
    tetra = np.asarray(tetra, dtype=np.int64)
    normal = np.asarray(plane_normal, dtype=float)
    normal /= np.linalg.norm(normal)
    axis = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(axis, normal)) > 0.9:
        axis = np.array([0.0, 1.0, 0.0])
    basis_u = np.cross(normal, axis)
    basis_u /= np.linalg.norm(basis_u)
    basis_v = np.cross(normal, basis_u)
    output_triangles = []
    parent_ids = []
    edges = ((0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3))
    for parent, node_ids in enumerate(tetra):
        tet_nodes = nodes[node_ids]
        signed = tet_nodes @ normal - float(plane_offset_m)
        if np.min(signed) > tolerance_m or np.max(signed) < -tolerance_m:
            continue
        points = []
        for first, second in edges:
            if abs(signed[first]) <= tolerance_m:
                points.append(tet_nodes[first])
            if signed[first] * signed[second] < -(tolerance_m**2):
                fraction = signed[first] / (signed[first] - signed[second])
                points.append(tet_nodes[first] + fraction * (tet_nodes[second] - tet_nodes[first]))
        if len(points) < 3:
            continue
        rounded = np.round(np.asarray(points), 14)
        points = np.unique(rounded, axis=0)
        if len(points) < 3:
            continue
        center = np.mean(points, axis=0)
        projection = np.column_stack(((points - center) @ basis_u, (points - center) @ basis_v))
        order = np.argsort(np.arctan2(projection[:, 1], projection[:, 0]))
        polygon = points[order]
        for index in range(1, len(polygon) - 1):
            output_triangles.append([polygon[0], polygon[index], polygon[index + 1]])
            parent_ids.append(parent)
    if not output_triangles:
        raise ValueError("section plane does not intersect the tetrahedral mesh")
    vertices = np.asarray(output_triangles, dtype=float)
    normals = np.broadcast_to(normal, vertices.shape)
    packed = _pack_vertices(
        vertices, normals, np.asarray(parent_ids, dtype=np.int64), np.asarray(stress_basis_pa_voigt)
    )
    return packed, np.asarray(parent_ids, dtype=np.int64)
