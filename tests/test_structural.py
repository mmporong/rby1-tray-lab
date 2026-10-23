import unittest

import numpy as np

from rby1_tray.structural import (
    distribute_wrench_to_nodes,
    evaluate_element_fields,
    resultant_wrench,
    solve_linear_elasticity,
    surface_traction_to_nodal_forces,
    volume_weighted_quantile,
)


def _structured_block(length_m, width_m, height_m, nx, ny, nz):
    coordinates = []
    for ix in range(nx + 1):
        for iy in range(ny + 1):
            for iz in range(nz + 1):
                coordinates.append(
                    (length_m * ix / nx, width_m * iy / ny, height_m * iz / nz)
                )
    nodes_m = np.asarray(coordinates)

    def node(ix, iy, iz):
        return ix * (ny + 1) * (nz + 1) + iy * (nz + 1) + iz

    tetra = []
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                vertices = [
                    node(ix, iy, iz),
                    node(ix + 1, iy, iz),
                    node(ix, iy + 1, iz),
                    node(ix, iy, iz + 1),
                    node(ix + 1, iy + 1, iz),
                    node(ix + 1, iy, iz + 1),
                    node(ix, iy + 1, iz + 1),
                    node(ix + 1, iy + 1, iz + 1),
                ]
                tetra.extend(
                    (
                        (vertices[0], vertices[1], vertices[4], vertices[7]),
                        (vertices[0], vertices[4], vertices[2], vertices[7]),
                        (vertices[0], vertices[2], vertices[6], vertices[7]),
                        (vertices[0], vertices[6], vertices[3], vertices[7]),
                        (vertices[0], vertices[3], vertices[5], vertices[7]),
                        (vertices[0], vertices[5], vertices[1], vertices[7]),
                    )
                )
    return nodes_m, np.asarray(tetra, dtype=int), node


class StructuralTests(unittest.TestCase):
    def test_affine_displacement_gives_constant_strain_and_stress(self):
        nodes_m = np.array(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float
        )
        tetra = np.array([[0, 2, 1, 3]])  # Negative orientation is accepted and repaired.
        displacement_gradient = np.array(
            [[0.01, 0.02, 0.03], [0.04, -0.02, 0.05], [0.06, 0.07, 0.03]]
        )
        translation_m = np.array([0.2, -0.1, 0.3])
        displacement_m = nodes_m @ displacement_gradient.T + translation_m
        result = evaluate_element_fields(nodes_m, tetra, displacement_m, 200e9, 0.25)
        expected_strain = np.array(
            [
                displacement_gradient[0, 0],
                displacement_gradient[1, 1],
                displacement_gradient[2, 2],
                displacement_gradient[0, 1] + displacement_gradient[1, 0],
                displacement_gradient[1, 2] + displacement_gradient[2, 1],
                displacement_gradient[2, 0] + displacement_gradient[0, 2],
            ]
        )
        strain_tensor = np.array(
            [
                [expected_strain[0], expected_strain[3] / 2, expected_strain[5] / 2],
                [expected_strain[3] / 2, expected_strain[1], expected_strain[4] / 2],
                [expected_strain[5] / 2, expected_strain[4] / 2, expected_strain[2]],
            ]
        )
        shear_modulus_pa = 200e9 / (2 * (1 + 0.25))
        lame_first_parameter_pa = 200e9 * 0.25 / ((1 + 0.25) * (1 - 2 * 0.25))
        expected_stress_tensor_pa = (
            2 * shear_modulus_pa * strain_tensor
            + lame_first_parameter_pa * np.trace(strain_tensor) * np.eye(3)
        )
        expected_stress_pa_voigt = expected_stress_tensor_pa[
            (0, 1, 2, 0, 1, 2), (0, 1, 2, 1, 2, 0)
        ]
        np.testing.assert_allclose(result["element_strain_voigt"][0], expected_strain)
        np.testing.assert_allclose(
            result["element_stress_pa_voigt"][0], expected_stress_pa_voigt
        )
        self.assertGreater(result["element_volume_m3"][0], 0)
        self.assertGreater(result["von_mises_pa"][0], 0)

    def test_uniform_surface_traction_preserves_resultant_wrench(self):
        nodes_m = np.array(
            [[0, 0, 0], [2, 0, 0], [2, 3, 0], [0, 3, 0]], dtype=float
        )
        triangles = np.array([[0, 1, 2], [0, 2, 3]])
        traction_pa = np.array([0, 0, -5.0])
        nodal_forces_n = surface_traction_to_nodal_forces(
            nodes_m, triangles, traction_pa
        )
        wrench = resultant_wrench(nodes_m, nodal_forces_n)
        np.testing.assert_allclose(wrench["force_n"], [0, 0, -30])
        np.testing.assert_allclose(wrench["moment_nm"], [-45, 30, 0])

    def test_wrench_distribution_is_exact_and_origin_covariant(self):
        nodes_m = np.array(
            [[0, 0, 0], [2, 0, 0], [2, 3, 0], [0, 3, 0], [9, 9, 9]], dtype=float
        )
        patch_nodes = np.array([0, 1, 2, 3])
        weights_m2 = np.array([0.5, 1.0, 1.5, 1.0])
        center_m = np.average(nodes_m[patch_nodes], axis=0, weights=weights_m2)
        force_n = np.array([10.0, -5.0, 20.0])
        center_moment_nm = np.array([1.0, 2.0, 3.0])
        center_forces_n = distribute_wrench_to_nodes(
            nodes_m,
            patch_nodes,
            weights_m2,
            force_n,
            center_moment_nm,
            origin_m=center_m,
        )
        center_wrench = resultant_wrench(nodes_m, center_forces_n, origin_m=center_m)
        np.testing.assert_allclose(center_wrench["force_n"], force_n, atol=1e-12)
        np.testing.assert_allclose(
            center_wrench["moment_nm"], center_moment_nm, atol=1e-12
        )

        shifted_origin_m = np.array([-4.0, 8.0, 2.0])
        shifted_moment_nm = center_moment_nm + np.cross(
            center_m - shifted_origin_m, force_n
        )
        shifted_forces_n = distribute_wrench_to_nodes(
            nodes_m,
            patch_nodes,
            weights_m2,
            force_n,
            shifted_moment_nm,
            origin_m=shifted_origin_m,
        )
        np.testing.assert_allclose(shifted_forces_n, center_forces_n, atol=1e-12)

    def test_wrench_distribution_rejects_rank_deficient_patch(self):
        nodes_m = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
        with self.assertRaisesRegex(ValueError, "six-axis wrench"):
            distribute_wrench_to_nodes(
                nodes_m,
                [0, 1, 2],
                [1, 1, 1],
                [1, 2, 3],
                [4, 5, 6],
            )

    def test_bar_traction_is_close_to_axial_extension_and_balanced(self):
        length_m = 4.0
        width_m = 1.0
        height_m = 1.0
        youngs_modulus_pa = 1e6
        applied_force_n = 1e3
        nodes_m, tetra, node = _structured_block(
            length_m, width_m, height_m, 8, 2, 2
        )
        end_triangles = []
        for iy in range(2):
            for iz in range(2):
                a = node(8, iy, iz)
                b = node(8, iy + 1, iz)
                c = node(8, iy, iz + 1)
                d = node(8, iy + 1, iz + 1)
                end_triangles.extend(((a, b, d), (a, d, c)))
        nodal_forces_n = surface_traction_to_nodal_forces(
            nodes_m,
            np.asarray(end_triangles),
            [applied_force_n / (width_m * height_m), 0, 0],
        )
        support_nodes = np.flatnonzero(np.isclose(nodes_m[:, 0], 0))
        result = solve_linear_elasticity(
            nodes_m,
            tetra,
            youngs_modulus_pa,
            0.2,
            support_nodes,
            nodal_forces_n,
        )
        end_nodes = np.flatnonzero(np.isclose(nodes_m[:, 0], length_m))
        mean_extension_m = np.mean(result["displacement_m"][end_nodes, 0])
        analytic_extension_m = applied_force_n * length_m / youngs_modulus_pa
        self.assertAlmostEqual(mean_extension_m / analytic_extension_m, 1.0, delta=0.08)
        self.assertLess(result["equilibrium_residual"]["force_norm_n"], 1e-8)
        self.assertLess(result["equilibrium_residual"]["moment_norm_nm"], 1e-8)
        self.assertLess(result["equilibrium_residual"]["free_dof_residual_norm_n"], 1e-8)
        self.assertAlmostEqual(
            2 * result["strain_energy_j"], result["external_work_j"], places=11
        )

    def test_solution_scales_linearly_and_is_translation_covariant(self):
        nodes_m, tetra, _ = _structured_block(2.0, 1.0, 1.0, 3, 1, 1)
        support_nodes = np.flatnonzero(np.isclose(nodes_m[:, 0], 0))
        end_nodes = np.flatnonzero(np.isclose(nodes_m[:, 0], 2.0))
        forces_n = np.zeros_like(nodes_m)
        forces_n[end_nodes, 0] = 10 / end_nodes.size
        first = solve_linear_elasticity(nodes_m, tetra, 1e5, 0.25, support_nodes, forces_n)
        translated = solve_linear_elasticity(
            nodes_m + [12, -3, 7], tetra, 1e5, 0.25, support_nodes, forces_n
        )
        scaled = solve_linear_elasticity(
            nodes_m, tetra, 1e5, 0.25, support_nodes, 2 * forces_n
        )
        np.testing.assert_allclose(translated["displacement_m"], first["displacement_m"])
        np.testing.assert_allclose(
            translated["element_stress_pa_voigt"], first["element_stress_pa_voigt"]
        )
        np.testing.assert_allclose(translated["reactions_n"], first["reactions_n"])
        np.testing.assert_allclose(scaled["displacement_m"], 2 * first["displacement_m"])
        np.testing.assert_allclose(
            scaled["element_stress_pa_voigt"], 2 * first["element_stress_pa_voigt"]
        )
        np.testing.assert_allclose(scaled["reactions_n"], 2 * first["reactions_n"])

    def test_volume_weighted_quantile_uses_element_volume(self):
        self.assertEqual(volume_weighted_quantile([1, 5, 10], [8, 1, 1], 0.8), 1)
        self.assertEqual(volume_weighted_quantile([1, 5, 10], [8, 1, 1], 0.81), 5)

    def test_invalid_mesh_material_load_and_boundary_conditions_are_rejected(self):
        nodes_m = np.array(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float
        )
        forces_n = np.zeros_like(nodes_m)
        for arguments in (
            (nodes_m, [[0, 1, 2, 2]], 1e6, 0.3, [0], forces_n),
            (nodes_m, [[0, 1, 2, 3]], -1, 0.3, [0], forces_n),
            (nodes_m, [[0, 1, 2, 3]], 1e6, 0.5, [0], forces_n),
            (nodes_m, [[0, 1, 2, 3]], 1e6, 0.3, [], forces_n),
            (nodes_m, [[0, 1, 2, 3]], 1e6, 0.3, [0, 1, 2, 3], forces_n),
        ):
            with self.subTest(arguments=arguments[2:5]), self.assertRaises(ValueError):
                solve_linear_elasticity(*arguments)
        disconnected_nodes_m = np.vstack((nodes_m, nodes_m + [3, 0, 0]))
        disconnected_tetra = np.array([[0, 1, 2, 3], [4, 5, 6, 7]])
        with self.assertRaises(ValueError):
            solve_linear_elasticity(
                disconnected_nodes_m,
                disconnected_tetra,
                1e6,
                0.3,
                [0, 1, 2],
                np.zeros_like(disconnected_nodes_m),
            )


if __name__ == "__main__":
    unittest.main()
