#!/usr/bin/env python3
"""Build a six-axis tensor response basis from the verified fine CAD mesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rby1_tray.structural import distribute_wrench_to_nodes, solve_linear_elasticity
from rby1_tray.structural_basis import (
    VERTEX_FLOATS,
    VOIGT_COMPONENTS,
    WRENCH_COMPONENTS,
    area_weighted_patch,
    build_cutaway_vertices,
    combine_stress_basis,
    expand_surface_vertices,
    select_planar_load_patch,
    sha256_file,
    solve_wrench_basis,
)


def relative_error(actual, expected):
    return float(np.linalg.norm(actual - expected) / max(np.linalg.norm(expected), 1e-30))


def validate_random_cases(nodes, tetra, support, triangles, material, basis, count=3):
    patch, weights, _ = area_weighted_patch(nodes, triangles)
    rng = np.random.default_rng(20261023)
    cases = []
    for index in range(count):
        coefficients = np.concatenate((rng.uniform(-80, 80, 3), rng.uniform(-4, 4, 3)))
        loads = distribute_wrench_to_nodes(
            nodes,
            patch,
            weights,
            coefficients[:3],
            coefficients[3:],
            origin_m=np.zeros(3),
        )
        full = solve_linear_elasticity(
            nodes,
            tetra,
            material["youngs_modulus_pa"],
            material["poisson_ratio"],
            support,
            loads,
            moment_origin_m=np.zeros(3),
        )
        combined_stress = combine_stress_basis(coefficients, basis["stress_basis_pa_voigt"])
        combined_displacement = np.einsum(
            "b,bnc->nc", coefficients, basis["displacement_basis_m"], optimize=True
        )
        stress_error = relative_error(combined_stress, full["element_stress_pa_voigt"])
        displacement_error = relative_error(combined_displacement, full["displacement_m"])
        if max(stress_error, displacement_error) > 2e-9:
            raise ValueError("random full solve disagrees with the response basis")
        cases.append(
            {
                "index": index,
                "coefficients": coefficients.tolist(),
                "stress_relative_l2": stress_error,
                "displacement_relative_l2": displacement_error,
            }
        )
    return cases


def build(input_npz: Path, config_path: Path, output: Path, random_cases: int = 3):
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.mkdir(parents=True)
    started = time.monotonic()
    config = json.loads(config_path.read_text())
    with np.load(input_npz) as data:
        nodes = np.asarray(data["nodes_m"], dtype=float)
        tetra = np.asarray(data["tetra"], dtype=np.int64)
        support = np.asarray(data["support_node_ids"], dtype=np.int64)
    triangles = select_planar_load_patch(nodes, tetra, z_m=0.0)
    material = config["material"]
    basis = solve_wrench_basis(
        nodes,
        tetra,
        support,
        triangles,
        material["youngs_modulus_pa"],
        material["poisson_ratio"],
    )
    random_validation = validate_random_cases(
        nodes, tetra, support, triangles, material, basis, count=random_cases
    )
    basis_npz = output / "basis.npz"
    np.savez_compressed(
        basis_npz,
        nodes_m=nodes,
        tetra=tetra,
        support_node_ids=support,
        load_triangles=triangles,
        stress_basis_pa_voigt=basis["stress_basis_pa_voigt"],
        displacement_basis_m=basis["displacement_basis_m"],
    )
    surface, surface_triangles, surface_parents = expand_surface_vertices(
        nodes, tetra, basis["stress_basis_pa_voigt"]
    )
    cutaway, cutaway_parents = build_cutaway_vertices(
        nodes, tetra, basis["stress_basis_pa_voigt"]
    )
    surface_path = output / "surface.bin"
    cutaway_path = output / "cutaway.bin"
    surface.tofile(surface_path)
    cutaway.tofile(cutaway_path)
    artifacts = {
        path.name: {"sha256": sha256_file(path), "bytes": path.stat().st_size}
        for path in (basis_npz, surface_path, cutaway_path)
    }
    validation = {
        "basis_cases": basis["validation"],
        "random_full_solve_cases": random_validation,
        "thresholds": {
            "basis_equilibrium_and_energy_relative": 1e-7,
            "random_superposition_relative_l2": 2e-9,
        },
        "passed": True,
    }
    validation_path = output / "validation.json"
    validation_path.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )
    artifacts[validation_path.name] = {
        "sha256": sha256_file(validation_path),
        "bytes": validation_path.stat().st_size,
    }
    metadata = {
        "schema_version": 1,
        "scope": "single selected CAD flange; linear static one-way response basis",
        "frame_convention": {
            "coordinates": "CAD flange frame translated so distal load origin is [0,0,0] m",
            "wrench_origin_m": [0.0, 0.0, 0.0],
            "wrench_order": list(WRENCH_COMPONENTS),
            "wrench_units": ["N", "N", "N", "N*m", "N*m", "N*m"],
            "stress_voigt_order": list(VOIGT_COMPONENTS),
        },
        "material": material,
        "boundary_conditions": {
            "support": config["support"],
            "load": config["load"],
            "support_nodes": int(len(support)),
            "load_triangles": int(len(triangles)),
            "load_patch_area_m2": basis["load_patch_area_m2"],
        },
        "mesh": {
            "nodes": int(len(nodes)),
            "tetrahedra": int(len(tetra)),
            "surface_triangles": int(len(surface_triangles)),
            "cutaway_triangles": int(len(cutaway_parents)),
        },
        "binary_layout": {
            "endianness": "little",
            "scalar": "float32",
            "primitive": "non-indexed triangles",
            "floats_per_vertex": VERTEX_FLOATS,
            "stride_bytes": VERTEX_FLOATS * 4,
            "fields": [
                {"name": "position_m", "offset_floats": 0, "components": 3},
                {"name": "normal", "offset_floats": 3, "components": 3},
                {
                    "name": "stress_basis_pa_voigt",
                    "offset_floats": 6,
                    "components": 36,
                    "shape": [6, 6],
                },
            ],
            "surface_vertices": int(len(surface)),
            "cutaway_vertices": int(len(cutaway)),
        },
        "source": {
            "input_npz": str(input_npz.relative_to(ROOT)),
            "input_npz_sha256": sha256_file(input_npz),
            "config": str(config_path.relative_to(ROOT)),
            "config_sha256": sha256_file(config_path),
            "source_step_sha256": config["source_step_sha256"],
            "builder_sha256": sha256_file(Path(__file__)),
            "module_sha256": sha256_file(ROOT / "src/rby1_tray/structural_basis.py"),
        },
        "validation": validation,
        "artifacts": artifacts,
        "elapsed_s": time.monotonic() - started,
    }
    metadata_path = output / "basis.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=ROOT / "artifacts/structural-flange/level-2.npz",
    )
    parser.add_argument(
        "--config", type=Path, default=ROOT / "configs/structural-flange.json"
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "artifacts/structural-basis"
    )
    parser.add_argument("--random-cases", type=int, default=3)
    args = parser.parse_args()
    result = build(args.input, args.config, args.output, args.random_cases)
    print(
        json.dumps(
            {
                "mesh": result["mesh"],
                "artifacts": result["artifacts"],
                "elapsed_s": result["elapsed_s"],
            },
            indent=2,
        )
    )
