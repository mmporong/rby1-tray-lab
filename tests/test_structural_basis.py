import unittest

import numpy as np

from rby1_tray.structural import distribute_wrench_to_nodes, solve_linear_elasticity
from rby1_tray.structural_basis import (
    VERTEX_FLOATS,
    area_weighted_patch,
    build_cutaway_vertices,
    combine_stress_basis,
    compute_frame_metrics,
    expand_surface_vertices,
    select_planar_load_patch,
    solve_wrench_basis,
    von_mises,
)


def cube_mesh():
    nodes = np.array(
        [
            [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
            [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1],
        ], dtype=float,
    )
    tetra = np.array(
        [[0,1,3,7],[0,3,2,7],[0,2,6,7],[0,6,4,7],[0,4,5,7],[0,5,1,7]],
        dtype=np.int64,
    )
    support = np.flatnonzero(np.isclose(nodes[:, 2], 1.0))
    return nodes, tetra, support


class StructuralBasisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nodes, cls.tetra, cls.support = cube_mesh()
        cls.triangles = select_planar_load_patch(cls.nodes, cls.tetra, z_m=0.0)
        cls.basis = solve_wrench_basis(
            cls.nodes, cls.tetra, cls.support, cls.triangles, 2.0e6, 0.25
        )

    def test_all_six_basis_cases_satisfy_equilibrium_and_energy(self):
        self.assertEqual(len(self.basis["validation"]), 6)
        for case in self.basis["validation"]:
            self.assertLess(case["equilibrium_relative"], 1e-9)
            self.assertLess(case["energy_identity_relative"], 1e-9)

    def test_tensor_superposition_matches_full_solve(self):
        coefficients = np.array([13.0, -7.0, 19.0, 2.0, -3.0, 1.5])
        patch, weights, _ = area_weighted_patch(self.nodes, self.triangles)
        loads = distribute_wrench_to_nodes(
            self.nodes, patch, weights, coefficients[:3], coefficients[3:], origin_m=np.zeros(3)
        )
        full = solve_linear_elasticity(
            self.nodes, self.tetra, 2e6, 0.25, self.support, loads, moment_origin_m=np.zeros(3)
        )
        stress = combine_stress_basis(coefficients, self.basis["stress_basis_pa_voigt"])
        displacement = np.einsum(
            "b,bnc->nc", coefficients, self.basis["displacement_basis_m"]
        )
        np.testing.assert_allclose(stress, full["element_stress_pa_voigt"], rtol=1e-10, atol=1e-8)
        np.testing.assert_allclose(displacement, full["displacement_m"], rtol=1e-10, atol=1e-12)
        self.assertFalse(np.allclose(
            von_mises(stress),
            np.einsum("b,be->e", coefficients, von_mises(self.basis["stress_basis_pa_voigt"])),
        ))

    def test_frame_metrics_are_chunk_invariant(self):
        coefficients = np.array([[1,2,3,0.1,0.2,0.3],[-4,5,-6,0.5,-0.2,0.8]])
        first = compute_frame_metrics(
            coefficients, self.basis["stress_basis_pa_voigt"], self.basis["displacement_basis_m"], chunk_size=1
        )
        second = compute_frame_metrics(
            coefficients, self.basis["stress_basis_pa_voigt"], self.basis["displacement_basis_m"], chunk_size=8
        )
        np.testing.assert_allclose(first["max_von_mises_pa"], second["max_von_mises_pa"])
        np.testing.assert_allclose(first["max_displacement_m"], second["max_displacement_m"])

    def test_surface_and_cutaway_binary_layout(self):
        surface, triangles, parents = expand_surface_vertices(
            self.nodes, self.tetra, self.basis["stress_basis_pa_voigt"]
        )
        cutaway, cutaway_parents = build_cutaway_vertices(
            self.nodes, self.tetra, self.basis["stress_basis_pa_voigt"],
            plane_normal=(1,0,0), plane_offset_m=0.5,
        )
        self.assertEqual(surface.shape, (3 * len(triangles), VERTEX_FLOATS))
        self.assertEqual(cutaway.shape, (3 * len(cutaway_parents), VERTEX_FLOATS))
        self.assertEqual(surface.dtype, np.dtype("<f4"))
        self.assertEqual(cutaway.dtype, np.dtype("<f4"))
        np.testing.assert_allclose(np.linalg.norm(surface[:,3:6],axis=1), 1)
        np.testing.assert_allclose(np.linalg.norm(cutaway[:,3:6],axis=1), 1)
        expected = self.basis["stress_basis_pa_voigt"][:, parents[0], :].reshape(-1)
        np.testing.assert_allclose(surface[0,6:], expected)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            compute_frame_metrics(
                np.ones((2,5)), self.basis["stress_basis_pa_voigt"], self.basis["displacement_basis_m"]
            )
        with self.assertRaises(ValueError):
            select_planar_load_patch(self.nodes, self.tetra, z_m=5)
        with self.assertRaises(ValueError):
            von_mises(np.ones((2,5)))


if __name__ == "__main__":
    unittest.main()
