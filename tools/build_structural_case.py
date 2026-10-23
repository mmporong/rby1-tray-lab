#!/usr/bin/env python3
"""Run an explicit one-way CAD flange load-transfer study with mesh checks."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import gmsh
import numpy as np
from rby1_tray.structural import (
    distribute_wrench_to_nodes, solve_linear_elasticity, volume_weighted_quantile,
)
from rby1_tray.structural_loads import select_recorded_load, transfer_sensor_wrench, validate_mesh_sizes


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mesh_part(brep, settings, h):
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.Terminal', 0)
        gmsh.option.setNumber('General.NumThreads', 1)
        gmsh.open(str(brep))
        volumes = gmsh.model.getEntities(3)
        if len(volumes) != 1:
            raise ValueError('BREP must contain exactly one selected volume')
        volume = volumes[0][1]
        if not np.isclose(gmsh.model.occ.getMass(3, volume), settings['expected_volume_m3'], rtol=1e-7):
            raise ValueError('BREP volume differs from selected STEP component')
        bbox = np.array(gmsh.model.getBoundingBox(3, volume))
        if not np.allclose(bbox, settings['expected_bbox_m'], rtol=0, atol=2e-7):
            raise ValueError('BREP bounding box differs from selected STEP component')
        origin = np.asarray(settings['cad_distal_origin_m'])
        distal, proximal = [], []
        faces = gmsh.model.getBoundary(volumes, oriented=False)
        for dim, tag in faces:
            center = gmsh.model.occ.getCenterOfMass(dim, tag)
            if gmsh.model.getType(dim, tag) == 'Plane':
                if abs(center[2]-origin[2]) < 1e-7:
                    distal.append(tag)
                if abs(center[2]-(bbox[5]-1e-7)) < 2e-7:
                    proximal.append(tag)
        if len(distal) != 1 or len(proximal) != 1:
            raise ValueError('expected one distal and one proximal planar interface')
        gmsh.option.setNumber('Mesh.MeshSizeMin', h)
        gmsh.option.setNumber('Mesh.MeshSizeMax', h)
        gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 0)
        gmsh.option.setNumber('Mesh.ElementOrder', 1)
        gmsh.option.setNumber('Mesh.Algorithm3D', 1)
        gmsh.model.mesh.generate(3)
        node_tags, xyz, _ = gmsh.model.mesh.getNodes()
        tet_tags, tet_nodes = gmsh.model.mesh.getElementsByType(4, volume)
        used = np.unique(tet_nodes)
        coordinates = dict(zip(map(int, node_tags), np.asarray(xyz).reshape(-1, 3)))
        ids = {int(tag):i for i, tag in enumerate(used)}
        nodes = np.array([coordinates[int(tag)] for tag in used]) - origin
        tetra = np.array([ids[int(tag)] for tag in tet_nodes], dtype=int).reshape(-1, 4)
        if tetra.size == 0:
            raise ValueError('CAD tetrahedralization produced no linear tetrahedra')
        def surface_triangles(tags):
            raw = np.concatenate([gmsh.model.mesh.getElementsByType(2, tag)[1] for tag in tags])
            return np.array([ids[int(tag)] for tag in raw], dtype=int).reshape(-1, 3)
        distal_tri = surface_triangles(distal)
        support = np.unique(surface_triangles(proximal))
        jac = np.asarray(gmsh.model.mesh.getElementQualities(tet_tags, 'minDetJac'))
        quality = np.asarray(gmsh.model.mesh.getElementQualities(tet_tags, 'minSICN'))
        if not np.all(np.isfinite(jac)) or np.min(jac) <= 0:
            raise ValueError('tetrahedral mesh has invalid or inverted Jacobians')
        metadata = dict(mesh_size_m=h, nodes=len(nodes), tetrahedra=len(tetra),
                        minimum_jacobian_determinant_m3=float(min(jac)),
                        minimum_scaled_inverse_condition_number=float(min(quality)),
                        distal_face_tags=distal, proximal_face_tags=proximal,
                        cad_boundary_faces=len(faces), connected_cad_volumes=1,
                        support_nodes=len(support))
        return nodes, tetra, distal_tri, support, metadata
    finally:
        gmsh.finalize()


def solve_case(nodes, tetra, triangles, support, force, moment, material):
    weights = np.zeros(len(nodes))
    area = np.linalg.norm(np.cross(nodes[triangles[:,1]]-nodes[triangles[:,0]],
                                   nodes[triangles[:,2]]-nodes[triangles[:,0]]), axis=1)/2
    for column in range(3):
        np.add.at(weights, triangles[:,column], area/3)
    patch = np.flatnonzero(weights > 0)
    load = distribute_wrench_to_nodes(nodes, patch, weights[patch], force, moment, origin_m=[0,0,0])
    result = solve_linear_elasticity(nodes, tetra, material['youngs_modulus_pa'],
                                     material['poisson_ratio'], support, load, moment_origin_m=[0,0,0])
    residual = result['equilibrium_residual']
    force_error = residual['force_norm_n']/max(np.linalg.norm(force), 1e-12)
    moment_error = residual['moment_norm_nm']/max(np.linalg.norm(moment), 1e-12)
    energy_error = abs(2*result['strain_energy_j']-result['external_work_j'])/max(abs(result['external_work_j']),1e-30)
    if max(force_error, moment_error, energy_error) > 1e-7:
        raise ValueError('elastic solve failed equilibrium or energy identity')
    metrics = dict(max_displacement_m=float(np.linalg.norm(result['displacement_m'],axis=1).max()),
                   peak_von_mises_pa=float(result['von_mises_pa'].max()),
                   volume_p95_von_mises_pa=volume_weighted_quantile(result['von_mises_pa'],result['element_volume_m3'],.95),
                   force_balance_relative=float(force_error), moment_balance_relative=float(moment_error),
                   energy_identity_relative=float(energy_error), strain_energy_j=result['strain_energy_j'],
                   external_work_j=result['external_work_j'], mesh_volume_m3=float(result['element_volume_m3'].sum()),
                   reaction_force_n=result['reaction_resultant']['force_n'].tolist(),
                   reaction_moment_nm=result['reaction_resultant']['moment_nm'].tolist())
    return result, metrics


def build(brep, config, commands, output):
    settings = json.loads(config.read_text())
    sizes = validate_mesh_sizes(settings['mesh_sizes_m'])
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    load = select_recorded_load(commands, settings['load_phase_filter'])
    rotation = np.array(settings['R_cad_from_sensor'], dtype=float)
    offset = np.array(settings['sensor_origin_in_cad_m'])-settings['cad_distal_origin_m']
    force, moment = transfer_sensor_wrench(load['raw_wrench']['force_n'],load['raw_wrench']['torque_nm'],
                                           rotation,settings['sensor_origin_in_cad_m'],settings['cad_distal_origin_m'])
    levels = []
    level_hashes = {}
    final_mesh = None
    final_metrics = None
    for index, h in enumerate(sizes):
        nodes, tetra, triangles, support, mesh = mesh_part(brep, settings, h)
        result, metrics = solve_case(nodes,tetra,triangles,support,force,moment,settings['material'])
        levels.append(dict(**mesh, **metrics))
        final_mesh = nodes, tetra, triangles, support
        final_metrics = metrics
        np.savez_compressed(output/f'level-{index}.npz', nodes_m=nodes, tetra=result['tetra_oriented'],
                            displacement_m=result['displacement_m'], von_mises_pa=result['von_mises_pa'],
                            element_stress_pa_voigt=result['element_stress_pa_voigt'],
                            element_volume_m3=result['element_volume_m3'], support_node_ids=support)
        level_hashes[f'level-{index}.npz'] = digest(output/f'level-{index}.npz')
        print(json.dumps(dict(level=index,**levels[-1])),flush=True)
    if final_mesh is None or final_metrics is None:
        raise ValueError('mesh refinement produced no solved level')
    nodes, tetra, triangles, support = final_mesh
    tolerance = settings['convergence_relative_tolerance']
    changes = {key:abs(levels[-1][key]-levels[-2][key])/max(abs(levels[-1][key]),1e-30)
               for key in ('max_displacement_m','volume_p95_von_mises_pa','strain_energy_j')}
    converged = all(value <= tolerance for value in changes.values())
    alternate_support = support[np.linalg.norm(nodes[support,:2],axis=1) >= settings['sensitivity_support_inner_radius_m']]
    _, sensitivity = solve_case(nodes,tetra,triangles,alternate_support,force,moment,settings['material'])
    report = dict(schema_version=1, title=settings['part_label'], snapshot_time_s=load['timestamp_sim_s'],
                  phase=load['phase'], material=settings['material'],
                  metrics=dict(**final_metrics, mesh_converged=converged),
                  assumptions=[
                      '재료 E=69 GPa, ν=0.33은 가정값이며 제조사 재료 카드가 아닙니다.',
                      '상단 평면의 병진 3축 완전 고정과 하단 평면의 등가 6축 분포 하중을 가정합니다. 볼트 프리로드·접촉 해석은 포함하지 않습니다.',
                      '공식 Fast Grip과 현재 시뮬레이터 손가락은 형상이 다릅니다. 축 방향과 센서 원점을 CAD 하단 면 중심에 일치시킨 전달 가정입니다.',
                      f"{load['timestamp_sim_s']:.3f}초의 고정 시점 정적 해석입니다. 주 영상의 현재 재생 시각에 따라 바뀌지 않습니다.",
                      'MuJoCo 기록 하중을 전달하는 단방향 선형탄성 후처리입니다. 이 CAD 부품의 자중·관성 체적력, 진동, 소성, 피로, 파손 안전율은 계산하지 않습니다.',
                      '모서리·완전 고정 경계의 국부 최대 응력은 메시와 체결 가정에 민감합니다. 수렴 판정은 체적 가중 P95·최대 변위·변형 에너지를 사용합니다.'
                  ], load_provenance=load,
                  transferred_load=dict(force_n=force.tolist(), moment_nm=moment.tolist(),
                                        origin_cad_m=settings['cad_distal_origin_m'],
                                        sensor_origin_offset_m=offset.tolist(), R_cad_from_sensor=rotation.tolist(),
                                        sign='negative raw sensor wrench: distal subtree reaction on proximal CAD flange'),
                  mesh_levels=levels, level_artifact_sha256=level_hashes,
                  convergence=dict(relative_changes=changes,tolerance=tolerance,passed=converged,
                                                      excludes='peak element stress is reported but not a converged safety metric'),
                  support_sensitivity=dict(alternate='proximal outer annulus r>=25mm clamped',
                                           support_nodes=len(alternate_support),metrics=sensitivity),
                  source=dict(step_sha256=settings['source_step_sha256'],brep_sha256=digest(brep),config_sha256=digest(config),
                              gmsh_version=gmsh.__version__,selected_volume_tag=settings['selected_import_volume_tag'],
                              scripts={p.relative_to(ROOT).as_posix():digest(p) for p in [Path(__file__), ROOT/'src/rby1_tray/structural.py',ROOT/'src/rby1_tray/structural_loads.py']}),
                  elapsed_s=time.monotonic()-started, artifact_sha256={})
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brep',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/structural-flange.json')
    parser.add_argument('--commands',type=Path,default=ROOT/'evidence/mobile-native/controller-data/commands.jsonl.gz')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=build(args.brep,args.config,args.commands,args.output)
    print(json.dumps({'convergence':report['convergence'],'elapsed_s':report['elapsed_s']},indent=2))
