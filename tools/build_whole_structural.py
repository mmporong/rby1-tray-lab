#!/usr/bin/env python3
"""Replay exact recorded poses; export whole-body section screening and CAD basis."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import sys
import time
from typing import Any

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
import numpy as np
from rby1_tray.cli import rendering_environment
from rby1_tray.playback import nearest_step_count
from rby1_tray.section_stress import section_basis,circular_section,section_von_mises,body_interaction_wrench
from rby1_tray.structural_basis import compute_frame_metrics
from tools.build_load_view import physical_vector


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def visual_geometry(model,data,body):
    mujoco=importlib.import_module('mujoco')
    rb=data.xmat[body].reshape(3,3)
    pieces=[];geoms=[]
    for geom in range(int(model.body_geomadr[body]),int(model.body_geomadr[body]+model.body_geomnum[body])):
        kind=int(model.geom_type[geom])
        is_mesh=(kind==mujoco.mjtGeom.mjGEOM_MESH and model.geom_contype[geom]==0)
        is_camera=('camera_case' in model.geom(geom).name and kind==mujoco.mjtGeom.mjGEOM_BOX)
        if not (is_mesh or is_camera):continue
        if is_mesh:
            mesh=int(model.geom_dataid[geom]);v=int(model.mesh_vertadr[mesh]);f=int(model.mesh_faceadr[mesh])
            vertices=np.asarray(model.mesh_vert[v:v+model.mesh_vertnum[mesh]],dtype=float)
            faces=np.asarray(model.mesh_face[f:f+model.mesh_facenum[mesh]],dtype=int)
        else:
            vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)],dtype=float)*model.geom_size[geom]
            faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
        relative_r=rb.T@data.geom_xmat[geom].reshape(3,3)
        relative_p=rb.T@(data.geom_xpos[geom]-data.xpos[body])
        triangles=(vertices@relative_r.T+relative_p)[faces]
        normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        length=np.linalg.norm(normal,axis=1);keep=length>1e-14
        triangles,normal=triangles[keep],normal[keep]/length[keep,None]
        pieces.append(np.concatenate((triangles,np.repeat(normal[:,None,:],3,axis=1)),axis=2).reshape(-1,6))
        geoms.append(geom)
    return (np.concatenate(pieces) if pieces else np.empty((0,6))),geoms


def part_label(name):
    if name=='base':return '차체 · 지배 부착부 하중 근사'
    if name.startswith('wheel_'):return '휠 '+name[6:8].upper()
    if name.startswith('link_torso_'):return '몸통 '+str(int(name.rsplit('_',1)[1])+1)
    for side,korean in (('left','왼팔'),('right','오른팔')):
        if name.startswith(f'link_{side}_arm_'):return korean+' '+str(int(name.rsplit('_',1)[1])+1)
    return {'NECK_0':'목 연결부','EE_BODY_L':'왼쪽 그리퍼 하우징','EE_BODY_R':'오른쪽 그리퍼 하우징',
            'ee_finger_l1':'왼손 손가락 1','ee_finger_l2':'왼손 손가락 2','ee_finger_r1':'오른손 손가락 1','ee_finger_r2':'오른손 손가락 2',
            'link_head_1':'헤드 회전부','link_head_2':'헤드 틸트부','head_camera_case':'헤드 카메라',
            'left_wrist_camera_case':'왼쪽 손목 카메라','right_wrist_camera_case':'오른쪽 손목 카메라'}.get(name,name)


def make_section(model,body,vertices):
    name=model.body(body).name
    children=[i for i in range(1,model.nbody) if model.body_parentid[i]==body]
    origin=np.zeros(3)
    if name=='base':
        origin=(vertices.min(axis=0)+vertices.max(axis=0))/2;axis=np.array([1.,0,0])
    else:
        offsets=[np.asarray(model.body_pos[i]) for i in children if np.linalg.norm(model.body_pos[i])>.025]
        if offsets:axis=max(offsets,key=lambda x:np.linalg.norm(x))
        else:
            _,vectors=np.linalg.eigh(np.cov(vertices.T));axis=vectors[:,-1]
    basis=section_basis(axis);local=(vertices-origin)@basis
    radius=max(float(np.max(np.linalg.norm(local[:,:2],axis=1))),.005)
    thickness=float(np.clip(radius*.10,.001,.006))
    section: dict[str,Any]=circular_section(radius,thickness)
    section.update(origin_body_m=origin.tolist(),basis_body=basis.ravel().tolist(),
                   source='assumed circular tube enclosing rendered surface, thickness=clamp(0.10R,1mm,6mm); not measured internal CAD')
    return section,local


def build(bundle,basis_dir,output):
    rendering_environment()
    mujoco=importlib.import_module('mujoco')
    from rby1_tray.mission import MobileSession
    from rby1_tray.scene import load_config
    with gzip.open(bundle/'states.json.gz','rt') as stream:reference=json.load(stream)
    basis_meta=json.loads((basis_dir/'basis.json').read_text())
    for name,entry in basis_meta['artifacts'].items():
        if sha(basis_dir/name)!=entry['sha256']:raise ValueError('FEM basis artifact hash mismatch')
    output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();session=MobileSession(load_config(bundle/'config.json'))
    scratch=mujoco.MjData(session.model)
    try:
        model=session.model;parts=[];geometry=[];local_points=[];body_ids=[];offset=0;coverage={}
        for body in range(1,model.nbody):
            if model.body_rootid[body]!=model.body('base').id:continue
            values,geoms=visual_geometry(model,session.data,body)
            if not len(values):continue
            section,points=make_section(model,body,values[:,:3])
            name=model.body(body).name
            assumptions=['원형 중공 등가 단면과 명시된 가정 두께를 사용합니다. 외피 위치에 사상한 공칭 응력 추정이며 전신 CAD FEM이 아닙니다.',
                         '횡방향 전단 V/A는 공칭 평균 지표입니다. 링크 안의 분포 관성·접촉과 실제 체결부 국부 응력은 해석하지 않습니다.']
            if name=='base':assumptions.append('자유 베이스의 cfrc_int를 차체 응력으로 사용하지 않습니다. 휠 4개·몸통 1개 부착부의 개별 반력을 차체 등가 단면에 전달한 뒤 최대 지표의 부착부 한 개를 표시합니다.')
            visual_only=bool(model.body_mass[body]<=1e-12)
            if visual_only:assumptions=['질량·관성이 없는 시각 외장입니다. 별도 구조 하중 경로가 정의되지 않아 응력을 계산하지 않으며 회색으로 표시합니다.']
            parts.append(dict(id=name,label=part_label(name),kind='section',vertex_offset=offset,vertex_count=len(values),section=section,assumptions=assumptions,
                              analysis_status='visual_only' if visual_only else 'section_assumption'))
            geometry.append(values);local_points.append(points);body_ids.append(body);offset+=len(values);coverage[name]=geoms
        parts.append(dict(id='left_flange_fem',label='왼쪽 연결 플랜지 · CAD FEM',kind='fem',vertex_offset=offset,vertex_count=0,
                          assumptions=['공식 Fast Grip CAD와 현재 손가락의 형상이 다릅니다. 동일 축·원점을 가정해 손목 wrench를 전달합니다.',
                                       'E=69 GPa, ν=0.33과 상단 완전 고정 가정. 각 시각의 준정적 선형 응답이며 부품 자체 관성·볼트 접촉은 포함하지 않습니다.']))
        n,p=len(reference),len(parts);frames=np.zeros((n,p,19),dtype=np.float32)
        maximum_difference=0.;maximum_ft_residual=0.;base_sources=[]
        base=model.body('base').id;base_children=[i for i in range(1,model.nbody) if model.body_parentid[i]==base]
        for index,row in enumerate(reference):
            steps=nearest_step_count(float(session.data.time),row['actual_time_s'],session.config['physics_dt_s'])
            session.step(steps);actual=session.snapshot()
            error=float(np.max(np.abs(physical_vector(actual)-physical_vector(row['state']))));maximum_difference=max(maximum_difference,error)
            if error>1e-9 or actual['phase']!=row['state']['phase']:raise RuntimeError(f'physics replay mismatch at {index}: {error}')
            # Recompute derived accelerations/wrenches at the rendered pose on a
            # clone. Integrator/solver warm-start and original sensor caches stay intact.
            mujoco.mj_copyData(scratch,model,session.data);mujoco.mj_forward(model,scratch);mujoco.mj_rnePostConstraint(model,scratch)
            for k,(part,body,points) in enumerate(zip(parts,body_ids,local_points)):
                section=part['section'];rotation=scratch.xmat[body].reshape(3,3);basis=np.array(section['basis_body']).reshape(3,3)
                origin=scratch.xpos[body]+rotation@np.array(section['origin_body_m'])
                if body==base:
                    candidates=[]
                    for child in base_children:
                        f,m=body_interaction_wrench(model,scratch,child,origin,rotation@basis)
                        f,m=-f,-m;peak=float(section_von_mises(points,f,m,section).max());candidates.append((peak,child,f,m))
                    peak,source,force,moment=max(candidates,key=lambda x:x[0]);base_sources.append(model.body(source).name)
                else:
                    force,moment=body_interaction_wrench(model,scratch,body,origin,rotation@basis)
                    peak=float(section_von_mises(points,force,moment,section).max())
                frames[index,k]=np.concatenate((scratch.xpos[body],rotation.ravel(),force,moment,[peak]))
            site=model.site('left_ft_frame').id;ftbody=int(model.site_bodyid[site]);r=scratch.site_xmat[site].reshape(3,3)
            force,moment=body_interaction_wrench(model,scratch,ftbody,scratch.site_xpos[site],r)
            for side in ('left','right'):
                s=model.site(f'{side}_ft_frame').id;b=int(model.site_bodyid[s]);sr=scratch.site_xmat[s].reshape(3,3)
                f,m=body_interaction_wrench(model,scratch,b,scratch.site_xpos[s],sr)
                sensors=np.concatenate((scratch.sensor(f'{side}_wrist_force_n').data,scratch.sensor(f'{side}_wrist_torque_nm').data))
                maximum_ft_residual=max(maximum_ft_residual,float(np.max(np.abs(np.concatenate((f,m))-sensors))))
            frames[index,-1,:18]=np.concatenate((scratch.site_xpos[site],r.ravel(),-force,-moment))
            if index%150==0:print(json.dumps(dict(frame=index,sim_time_s=float(scratch.time),matched_error=error)),flush=True)
        if maximum_ft_residual>1e-7:raise ValueError(f'FT/cfrc frame transform failed: {maximum_ft_residual}')
        with np.load(basis_dir/'basis.npz',allow_pickle=False) as basis_data:
            print('computing synchronized FEM metrics',flush=True)
            metrics=compute_frame_metrics(frames[:,-1,12:18].astype(float),basis_data['stress_basis_pa_voigt'],basis_data['displacement_basis_m'],chunk_size=8)
        frames[:,-1,18]=metrics['max_von_mises_pa']
        geometry_values=np.concatenate(geometry).astype('<f4');geometry_values.tofile(output/'geometry.bin');frames.astype('<f4').tofile(output/'frames.bin')
        shutil.copyfile(basis_dir/'surface.bin',output/'fem-surface.bin');shutil.copyfile(basis_dir/'cutaway.bin',output/'fem-cutaway.bin')
        frame_times=[row['actual_time_s'] for row in reference]
        manifest=dict(schema_version=1,fps=60,base_speed=8,frame_count=n,reference_states_sha256=sha(bundle/'states.json.gz'),
                      frame_times_s=frame_times,frame_stride_floats=p*19,part_stride_floats=19,geometry_stride_floats=6,
                      parts=parts,fem_part_id='left_flange_fem',fem_stride_floats=42,
                      fem_surface_vertex_count=basis_meta['binary_layout']['surface_vertices'],
                      fem_cutaway_vertex_count=basis_meta['binary_layout']['cutaway_vertices'],
                      max_von_mises_pa=float(frames[:,:,18].max()),
                      scope='전신의 등가 단면 응력 추정과 선택 플랜지의 준정적 CAD FEM을 원본 영상 시각에 맞춰 표시합니다.',
                      assumptions=['전신은 내력과 가정한 등가 단면을 연결한 응력 추정입니다. 제조사 전체 CAD의 요소 해석·실물 강도 판정이 아닙니다.',
                                   '실제 CAD 플랜지는 응력 텐서 기저해를 현재 6축 하중으로 합성합니다. 재료·구속·원점 정합 가정은 유지합니다.',
                                   '전신과 부품 상세는 원본 영상의 동일 프레임·시뮬레이션 시각을 사용합니다.'],
                      assets={name:sha(output/name) for name in ('geometry.bin','frames.bin','fem-surface.bin','fem-cutaway.bin')},
                      validation=dict(reference_frames=n,maximum_matched_physical_difference=maximum_difference,
                                      maximum_wrist_ft_crosscheck_absolute=maximum_ft_residual,
                                      derived_dynamics_on_scratch_only=True,visual_robot_body_count=len(body_ids),visual_geom_coverage=coverage,
                                      section_analysis_body_count=sum(part.get('analysis_status')=='section_assumption' for part in parts),
                                      visual_only_body_ids=[part['id'] for part in parts if part.get('analysis_status')=='visual_only'],
                                      base_dominant_mount_per_frame=base_sources,FEM_basis_validation=basis_meta['validation'],
                                      fem_max_displacement_m=np.asarray(metrics['max_displacement_m']).tolist()),
                      source=dict(physics_commit=json.loads((bundle/'manifest.json').read_text())['source_commit'],
                                  basis_metadata_sha256=sha(basis_dir/'basis.json'),
                                  script_sha256=sha(__file__),section_module_sha256=sha(ROOT/'src/rby1_tray/section_stress.py')),
                      elapsed_s=time.monotonic()-started)
        (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(frames=n,parts=p,vertices=len(geometry_values),maximum_difference=maximum_difference,ft_residual=maximum_ft_residual,elapsed_s=manifest['elapsed_s'])),flush=True)
    finally:session.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,default=ROOT/'evidence/playback-native')
    parser.add_argument('--basis',type=Path,default=ROOT/'artifacts/structural-basis')
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();build(a.bundle,a.basis,a.output)
