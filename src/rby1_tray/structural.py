"""Small-strain isotropic linear elasticity for first-order tetrahedral meshes.

All geometry and result fields use SI units.  This module supplies a compact
post-processing solver; material properties and boundary conditions remain
explicit inputs so a caller can preserve their provenance separately.
"""

from __future__ import annotations

import warnings

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import MatrixRankWarning, spsolve


_VOIGT_COMPONENTS = ("xx", "yy", "zz", "xy", "yz", "zx")


def _as_finite_array(value, *, name, ndim=None, trailing_shape=None, dtype=float):
    array = np.asarray(value, dtype=dtype)
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"{name} must have {ndim} dimensions")
    if trailing_shape is not None and array.shape[-len(trailing_shape):] != trailing_shape:
        raise ValueError(f"{name} must end with shape {trailing_shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _validate_material(youngs_modulus_pa, poisson_ratio):
    youngs_modulus_pa = float(youngs_modulus_pa)
    poisson_ratio = float(poisson_ratio)
    if not np.isfinite(youngs_modulus_pa) or youngs_modulus_pa <= 0:
        raise ValueError("youngs_modulus_pa must be finite and positive")
    if not np.isfinite(poisson_ratio) or not -1.0 < poisson_ratio < 0.5:
        raise ValueError("poisson_ratio must be finite and satisfy -1 < nu < 0.5")
    return youngs_modulus_pa, poisson_ratio


def _elasticity_matrix(youngs_modulus_pa, poisson_ratio):
    factor_pa = youngs_modulus_pa / ((1 + poisson_ratio) * (1 - 2 * poisson_ratio))
    diagonal = 1 - poisson_ratio
    coupling = poisson_ratio
    shear = (1 - 2 * poisson_ratio) / 2
    return factor_pa * np.array(
        [
            [diagonal, coupling, coupling, 0, 0, 0],
            [coupling, diagonal, coupling, 0, 0, 0],
            [coupling, coupling, diagonal, 0, 0, 0],
            [0, 0, 0, shear, 0, 0],
            [0, 0, 0, 0, shear, 0],
            [0, 0, 0, 0, 0, shear],
        ],
        dtype=float,
    )


def _validate_and_orient_mesh(nodes_m, tetra):
    nodes_m = _as_finite_array(nodes_m, name="nodes_m", ndim=2, trailing_shape=(3,))
    tetra = np.asarray(tetra)
    if tetra.ndim != 2 or tetra.shape[1:] != (4,):
        raise ValueError("tetra must have shape (M, 4)")
    if nodes_m.shape[0] < 4 or tetra.shape[0] == 0:
        raise ValueError("mesh must contain at least four nodes and one tetrahedron")
    if not np.issubdtype(tetra.dtype, np.integer):
        if not np.all(np.isfinite(tetra)) or not np.all(tetra == np.floor(tetra)):
            raise ValueError("tetra must contain integer node indices")
    tetra = np.asarray(tetra, dtype=np.int64).copy()
    if np.any(tetra < 0) or np.any(tetra >= nodes_m.shape[0]):
        raise ValueError("tetra contains an out-of-range node index")
    if np.any(np.apply_along_axis(lambda row: np.unique(row).size != 4, 1, tetra)):
        raise ValueError("each tetrahedron must reference four distinct nodes")

    edge_matrix_m = np.stack(
        (
            nodes_m[tetra[:, 1]] - nodes_m[tetra[:, 0]],
            nodes_m[tetra[:, 2]] - nodes_m[tetra[:, 0]],
            nodes_m[tetra[:, 3]] - nodes_m[tetra[:, 0]],
        ),
        axis=1,
    )
    signed_six_volume_m3 = np.linalg.det(edge_matrix_m)
    diagonal_m = float(np.linalg.norm(np.ptp(nodes_m, axis=0)))
    determinant_tolerance_m3 = (
        max(diagonal_m, np.finfo(float).tiny) ** 3
        * np.finfo(float).eps
        * 128
    )
    if np.any(np.abs(signed_six_volume_m3) <= determinant_tolerance_m3):
        raise ValueError("mesh contains a zero-volume or numerically degenerate tetrahedron")
    negative = signed_six_volume_m3 < 0
    if np.any(negative):
        first = tetra[negative, 1].copy()
        tetra[negative, 1] = tetra[negative, 2]
        tetra[negative, 2] = first
        signed_six_volume_m3[negative] *= -1
    return nodes_m.copy(), tetra, signed_six_volume_m3 / 6


def _strain_displacement_matrix(element_nodes_m):
    interpolation = np.column_stack((np.ones(4), element_nodes_m))
    coefficients = np.linalg.inv(interpolation)
    gradients_per_m = coefficients[1:, :].T
    matrix = np.zeros((6, 12), dtype=float)
    for node_index, (gradient_x, gradient_y, gradient_z) in enumerate(gradients_per_m):
        column = 3 * node_index
        matrix[0, column] = gradient_x
        matrix[1, column + 1] = gradient_y
        matrix[2, column + 2] = gradient_z
        matrix[3, column] = gradient_y
        matrix[3, column + 1] = gradient_x
        matrix[4, column + 1] = gradient_z
        matrix[4, column + 2] = gradient_y
        matrix[5, column] = gradient_z
        matrix[5, column + 2] = gradient_x
    return matrix


def _assemble_stiffness(nodes_m, tetra, element_volume_m3, elasticity_pa):
    element_dofs = (3 * tetra[:, :, None] + np.arange(3)).reshape((-1, 12))
    row_indices = np.repeat(element_dofs, 12, axis=1).ravel()
    column_indices = np.tile(element_dofs, (1, 12)).ravel()
    stiffness_values = np.empty((tetra.shape[0], 12, 12), dtype=float)
    strain_matrices = np.empty((tetra.shape[0], 6, 12), dtype=float)
    for element_index, node_indices in enumerate(tetra):
        strain_matrix = _strain_displacement_matrix(nodes_m[node_indices])
        strain_matrices[element_index] = strain_matrix
        stiffness_values[element_index] = (
            strain_matrix.T @ elasticity_pa @ strain_matrix * element_volume_m3[element_index]
        )
    ndof = 3 * nodes_m.shape[0]
    stiffness = coo_matrix(
        (stiffness_values.ravel(), (row_indices, column_indices)),
        shape=(ndof, ndof),
    ).tocsr()
    return stiffness, strain_matrices


def _validate_rigid_body_constraints(nodes_m, tetra, support_node_ids):
    parent = np.arange(nodes_m.shape[0])

    def find(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    def union(first, second):
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    used = np.zeros(nodes_m.shape[0], dtype=bool)
    for element in tetra:
        used[element] = True
        for node in element[1:]:
            union(element[0], node)
    if not np.all(used):
        raise ValueError("mesh contains nodes that do not belong to any tetrahedron")

    support_set = set(map(int, support_node_ids))
    component_roots = {find(node) for node in range(nodes_m.shape[0])}
    for root in component_roots:
        component_nodes = np.array(
            [node for node in range(nodes_m.shape[0]) if find(node) == root], dtype=int
        )
        component_support = np.array(
            [node for node in component_nodes if node in support_set], dtype=int
        )
        if component_support.size == 0:
            raise ValueError("every disconnected mesh component must contain support nodes")
        center_m = np.mean(nodes_m[component_nodes], axis=0)
        rigid_constraints = np.zeros((3 * component_support.size, 6), dtype=float)
        for index, node in enumerate(component_support):
            x_m, y_m, z_m = nodes_m[node] - center_m
            rigid_constraints[3 * index : 3 * index + 3, :3] = np.eye(3)
            rigid_constraints[3 * index : 3 * index + 3, 3:] = np.array(
                [[0, z_m, -y_m], [-z_m, 0, x_m], [y_m, -x_m, 0]]
            )
        if np.linalg.matrix_rank(rigid_constraints) < 6:
            raise ValueError("support nodes do not constrain all rigid-body modes")


def _von_mises(stress_pa_voigt):
    normal = stress_pa_voigt[:, :3]
    shear = stress_pa_voigt[:, 3:]
    normal_difference = (
        (normal[:, 0] - normal[:, 1]) ** 2
        + (normal[:, 1] - normal[:, 2]) ** 2
        + (normal[:, 2] - normal[:, 0]) ** 2
    )
    return np.sqrt(0.5 * normal_difference + 3 * np.sum(shear**2, axis=1))


def resultant_wrench(nodes_m, nodal_forces_n, *, origin_m=None):
    """Return the force and moment resultant of nodal loads."""
    nodes_m = _as_finite_array(nodes_m, name="nodes_m", ndim=2, trailing_shape=(3,))
    nodal_forces_n = _as_finite_array(
        nodal_forces_n, name="nodal_forces_n", ndim=2, trailing_shape=(3,)
    )
    if nodal_forces_n.shape != nodes_m.shape:
        raise ValueError("nodal_forces_n must have the same shape as nodes_m")
    if origin_m is None:
        origin_m = np.zeros(3, dtype=float)
    origin_m = _as_finite_array(origin_m, name="origin_m", ndim=1, trailing_shape=(3,))
    return {
        "force_n": np.sum(nodal_forces_n, axis=0),
        "moment_nm": np.sum(np.cross(nodes_m - origin_m, nodal_forces_n), axis=0),
        "origin_m": origin_m.copy(),
    }


def surface_traction_to_nodal_forces(nodes_m, triangles, traction_pa):
    """Convert constant-per-face tractions to consistent linear-triangle nodal loads.

    ``traction_pa`` may be one three-vector or one vector per triangle.  Triangle
    orientation does not affect the supplied traction direction.
    """
    nodes_m = _as_finite_array(nodes_m, name="nodes_m", ndim=2, trailing_shape=(3,))
    triangles = np.asarray(triangles)
    if triangles.ndim != 2 or triangles.shape[1:] != (3,) or triangles.shape[0] == 0:
        raise ValueError("triangles must have shape (K, 3) with K > 0")
    if not np.issubdtype(triangles.dtype, np.integer):
        if not np.all(np.isfinite(triangles)) or not np.all(triangles == np.floor(triangles)):
            raise ValueError("triangles must contain integer node indices")
    triangles = np.asarray(triangles, dtype=np.int64)
    if np.any(triangles < 0) or np.any(triangles >= nodes_m.shape[0]):
        raise ValueError("triangles contains an out-of-range node index")
    if np.any(np.apply_along_axis(lambda row: np.unique(row).size != 3, 1, triangles)):
        raise ValueError("each triangle must reference three distinct nodes")
    traction_pa = np.asarray(traction_pa, dtype=float)
    if traction_pa.shape == (3,):
        traction_pa = np.broadcast_to(traction_pa, (triangles.shape[0], 3))
    if traction_pa.shape != (triangles.shape[0], 3) or not np.all(np.isfinite(traction_pa)):
        raise ValueError("traction_pa must have shape (3,) or (K, 3) and be finite")
    side_a_m = nodes_m[triangles[:, 1]] - nodes_m[triangles[:, 0]]
    side_b_m = nodes_m[triangles[:, 2]] - nodes_m[triangles[:, 0]]
    area_m2 = np.linalg.norm(np.cross(side_a_m, side_b_m), axis=1) / 2
    if np.any(area_m2 <= 0):
        raise ValueError("triangles contains a zero-area face")
    nodal_forces_n = np.zeros_like(nodes_m)
    per_vertex_force_n = traction_pa * area_m2[:, None] / 3
    for local_node in range(3):
        np.add.at(nodal_forces_n, triangles[:, local_node], per_vertex_force_n)
    return nodal_forces_n


def distribute_wrench_to_nodes(
    nodes_m,
    patch_node_ids,
    node_weights_m2,
    force_n,
    moment_nm,
    *,
    origin_m=None,
):
    """Distribute a target wrench over a surface patch with minimum weighted norm.

    Positive ``node_weights_m2`` are typically accumulated triangle area divided
    by three.  The calculation shifts the requested moment to the area-weighted
    patch center before solving, which keeps it well conditioned and independent
    of the caller's chosen wrench origin.
    """
    nodes_m = _as_finite_array(nodes_m, name="nodes_m", ndim=2, trailing_shape=(3,))
    patch_node_ids = np.asarray(patch_node_ids)
    if patch_node_ids.ndim != 1 or patch_node_ids.size < 3:
        raise ValueError("patch_node_ids must contain at least three nodes")
    if not np.issubdtype(patch_node_ids.dtype, np.integer):
        if not np.all(np.isfinite(patch_node_ids)) or not np.all(
            patch_node_ids == np.floor(patch_node_ids)
        ):
            raise ValueError("patch_node_ids must contain integer node indices")
    patch_node_ids = np.asarray(patch_node_ids, dtype=np.int64)
    if np.unique(patch_node_ids).size != patch_node_ids.size:
        raise ValueError("patch_node_ids must not contain duplicates")
    if np.any(patch_node_ids < 0) or np.any(patch_node_ids >= nodes_m.shape[0]):
        raise ValueError("patch_node_ids contains an out-of-range node index")
    node_weights_m2 = _as_finite_array(
        node_weights_m2, name="node_weights_m2", ndim=1
    )
    if node_weights_m2.shape != patch_node_ids.shape or np.any(node_weights_m2 <= 0):
        raise ValueError("node_weights_m2 must be positive with one value per patch node")
    force_n = _as_finite_array(force_n, name="force_n", ndim=1, trailing_shape=(3,))
    moment_nm = _as_finite_array(
        moment_nm, name="moment_nm", ndim=1, trailing_shape=(3,)
    )
    patch_nodes_m = nodes_m[patch_node_ids]
    center_m = np.average(patch_nodes_m, axis=0, weights=node_weights_m2)
    if origin_m is None:
        origin_m = center_m
    origin_m = _as_finite_array(origin_m, name="origin_m", ndim=1, trailing_shape=(3,))
    center_moment_nm = moment_nm - np.cross(center_m - origin_m, force_n)
    relative_nodes_m = patch_nodes_m - center_m
    characteristic_length_m = float(
        np.sqrt(np.average(np.sum(relative_nodes_m**2, axis=1), weights=node_weights_m2))
    )
    if characteristic_length_m <= np.finfo(float).eps:
        raise ValueError("patch nodes do not span a finite area")

    wrench_matrix = np.zeros((6, 3 * patch_node_ids.size), dtype=float)
    for index, (x_m, y_m, z_m) in enumerate(relative_nodes_m):
        column = 3 * index
        wrench_matrix[:3, column : column + 3] = np.eye(3)
        wrench_matrix[3:, column : column + 3] = np.array(
            [[0, -z_m, y_m], [z_m, 0, -x_m], [-y_m, x_m, 0]]
        ) / characteristic_length_m
    target_scaled = np.concatenate((force_n, center_moment_nm / characteristic_length_m))
    dof_weights_m2 = np.repeat(node_weights_m2, 3)
    weighted_gram = (wrench_matrix * dof_weights_m2) @ wrench_matrix.T
    if np.linalg.matrix_rank(weighted_gram) < 6:
        raise ValueError("patch geometry cannot represent an arbitrary six-axis wrench")
    multipliers = np.linalg.solve(weighted_gram, target_scaled)
    patch_forces_n = dof_weights_m2 * (wrench_matrix.T @ multipliers)
    nodal_forces_n = np.zeros_like(nodes_m)
    nodal_forces_n[patch_node_ids] = patch_forces_n.reshape((-1, 3))
    achieved = resultant_wrench(nodes_m, nodal_forces_n, origin_m=origin_m)
    wrench_scale = max(
        np.linalg.norm(force_n),
        np.linalg.norm(moment_nm) / characteristic_length_m,
        1.0,
    )
    residual_scaled = np.concatenate(
        (
            achieved["force_n"] - force_n,
            (achieved["moment_nm"] - moment_nm) / characteristic_length_m,
        )
    )
    if np.linalg.norm(residual_scaled) > 1e-10 * wrench_scale:
        raise ValueError("wrench distribution failed its resultant check")
    return nodal_forces_n


def evaluate_element_fields(
    nodes_m,
    tetra,
    displacement_m,
    youngs_modulus_pa,
    poisson_ratio,
):
    """Evaluate constant tet4 strain and stress fields for a displacement field."""
    youngs_modulus_pa, poisson_ratio = _validate_material(
        youngs_modulus_pa, poisson_ratio
    )
    nodes_m, tetra, element_volume_m3 = _validate_and_orient_mesh(nodes_m, tetra)
    displacement_m = _as_finite_array(
        displacement_m, name="displacement_m", ndim=2, trailing_shape=(3,)
    )
    if displacement_m.shape != nodes_m.shape:
        raise ValueError("displacement_m must have the same shape as nodes_m")
    elasticity_pa = _elasticity_matrix(youngs_modulus_pa, poisson_ratio)
    strain_voigt = np.empty((tetra.shape[0], 6), dtype=float)
    stress_pa_voigt = np.empty_like(strain_voigt)
    for element_index, node_indices in enumerate(tetra):
        strain_matrix = _strain_displacement_matrix(nodes_m[node_indices])
        element_displacement_m = displacement_m[node_indices].ravel()
        strain_voigt[element_index] = strain_matrix @ element_displacement_m
        stress_pa_voigt[element_index] = elasticity_pa @ strain_voigt[element_index]
    return {
        "tetra_oriented": tetra,
        "element_volume_m3": element_volume_m3,
        "element_strain_voigt": strain_voigt,
        "element_stress_pa_voigt": stress_pa_voigt,
        "von_mises_pa": _von_mises(stress_pa_voigt),
        "voigt_components": _VOIGT_COMPONENTS,
    }


def volume_weighted_quantile(values, element_volume_m3, quantile):
    """Return a deterministic volume-weighted quantile for element results."""
    values = _as_finite_array(values, name="values", ndim=1)
    element_volume_m3 = _as_finite_array(
        element_volume_m3, name="element_volume_m3", ndim=1
    )
    quantile = float(quantile)
    if values.shape != element_volume_m3.shape or values.size == 0:
        raise ValueError("values and element_volume_m3 must be nonempty and have equal shape")
    if np.any(element_volume_m3 <= 0):
        raise ValueError("element_volume_m3 must be positive")
    if not np.isfinite(quantile) or not 0 <= quantile <= 1:
        raise ValueError("quantile must be between zero and one")
    order = np.argsort(values, kind="stable")
    cumulative = np.cumsum(element_volume_m3[order])
    threshold_m3 = quantile * cumulative[-1]
    index = min(np.searchsorted(cumulative, threshold_m3, side="left"), values.size - 1)
    return float(values[order[index]])


def solve_linear_elasticity(
    nodes_m,
    tetra,
    youngs_modulus_pa,
    poisson_ratio,
    support_node_ids,
    nodal_forces_n,
    *,
    moment_origin_m=None,
):
    """Solve a clamped-support, nodally loaded tet4 elasticity problem."""
    youngs_modulus_pa, poisson_ratio = _validate_material(
        youngs_modulus_pa, poisson_ratio
    )
    nodes_m, tetra, element_volume_m3 = _validate_and_orient_mesh(nodes_m, tetra)
    nodal_forces_n = _as_finite_array(
        nodal_forces_n, name="nodal_forces_n", ndim=2, trailing_shape=(3,)
    )
    if nodal_forces_n.shape != nodes_m.shape:
        raise ValueError("nodal_forces_n must have the same shape as nodes_m")
    support_node_ids = np.asarray(support_node_ids)
    if support_node_ids.ndim != 1 or support_node_ids.size == 0:
        raise ValueError("support_node_ids must be a nonempty one-dimensional sequence")
    if not np.issubdtype(support_node_ids.dtype, np.integer):
        if not np.all(np.isfinite(support_node_ids)) or not np.all(
            support_node_ids == np.floor(support_node_ids)
        ):
            raise ValueError("support_node_ids must contain integer node indices")
    support_node_ids = np.unique(np.asarray(support_node_ids, dtype=np.int64))
    if np.any(support_node_ids < 0) or np.any(support_node_ids >= nodes_m.shape[0]):
        raise ValueError("support_node_ids contains an out-of-range node index")
    if support_node_ids.size == nodes_m.shape[0]:
        raise ValueError("at least one node must remain unconstrained")
    _validate_rigid_body_constraints(nodes_m, tetra, support_node_ids)

    elasticity_pa = _elasticity_matrix(youngs_modulus_pa, poisson_ratio)
    stiffness, strain_matrices = _assemble_stiffness(
        nodes_m, tetra, element_volume_m3, elasticity_pa
    )
    ndof = 3 * nodes_m.shape[0]
    constrained_dofs = (3 * support_node_ids[:, None] + np.arange(3)).ravel()
    free_mask = np.ones(ndof, dtype=bool)
    free_mask[constrained_dofs] = False
    free_dofs = np.flatnonzero(free_mask)
    force_vector_n = nodal_forces_n.ravel()
    reduced_stiffness = stiffness[free_dofs][:, free_dofs]
    with warnings.catch_warnings():
        warnings.simplefilter("error", MatrixRankWarning)
        try:
            free_displacement_m = np.asarray(
                spsolve(reduced_stiffness, force_vector_n[free_dofs]), dtype=float
            ).reshape(-1)
        except (MatrixRankWarning, RuntimeError) as error:
            raise ValueError("mesh is not sufficiently constrained or is disconnected") from error
    if not np.all(np.isfinite(free_displacement_m)):
        raise ValueError("elastic solve produced non-finite displacement")
    displacement_vector_m = np.zeros(ndof, dtype=float)
    displacement_vector_m[free_dofs] = free_displacement_m
    displacement_m = displacement_vector_m.reshape((-1, 3))

    strain_voigt = np.empty((tetra.shape[0], 6), dtype=float)
    stress_pa_voigt = np.empty_like(strain_voigt)
    for element_index, node_indices in enumerate(tetra):
        element_displacement_m = displacement_m[node_indices].ravel()
        strain_voigt[element_index] = strain_matrices[element_index] @ element_displacement_m
        stress_pa_voigt[element_index] = elasticity_pa @ strain_voigt[element_index]

    full_residual_n = (stiffness @ displacement_vector_m - force_vector_n).reshape((-1, 3))
    reactions_n = np.zeros_like(nodal_forces_n)
    reactions_n[support_node_ids] = full_residual_n[support_node_ids]
    if moment_origin_m is None:
        moment_origin_m = np.mean(nodes_m, axis=0)
    applied_resultant = resultant_wrench(
        nodes_m, nodal_forces_n, origin_m=moment_origin_m
    )
    reaction_resultant = resultant_wrench(
        nodes_m, reactions_n, origin_m=moment_origin_m
    )
    residual_force_n = applied_resultant["force_n"] + reaction_resultant["force_n"]
    residual_moment_nm = applied_resultant["moment_nm"] + reaction_resultant["moment_nm"]
    strain_energy_j = float(0.5 * displacement_vector_m @ (stiffness @ displacement_vector_m))
    external_work_j = float(force_vector_n @ displacement_vector_m)
    free_dof_residual_norm_n = float(np.linalg.norm(full_residual_n.ravel()[free_dofs]))

    return {
        "nodes_m": nodes_m,
        "tetra_oriented": tetra,
        "displacement_m": displacement_m,
        "element_strain_voigt": strain_voigt,
        "element_stress_pa_voigt": stress_pa_voigt,
        "von_mises_pa": _von_mises(stress_pa_voigt),
        "element_volume_m3": element_volume_m3,
        "reactions_n": reactions_n,
        "applied_resultant": applied_resultant,
        "reaction_resultant": reaction_resultant,
        "equilibrium_residual": {
            "force_n": residual_force_n,
            "moment_nm": residual_moment_nm,
            "force_norm_n": float(np.linalg.norm(residual_force_n)),
            "moment_norm_nm": float(np.linalg.norm(residual_moment_nm)),
            "free_dof_residual_norm_n": free_dof_residual_norm_n,
        },
        "strain_energy_j": strain_energy_j,
        "external_work_j": external_work_j,
        "voigt_components": _VOIGT_COMPONENTS,
        "support_node_ids": support_node_ids,
        "youngs_modulus_pa": youngs_modulus_pa,
        "poisson_ratio": poisson_ratio,
    }
