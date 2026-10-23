#!/usr/bin/env python3
"""Render solved element stress on CAD boundary triangles and a true mesh section."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import cast

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from rby1_tray.structural_evidence import load_verified_level

BACKGROUND = '#101b24'
TEXT = '#d8e3ea'


def boundary_faces(tetra):
    faces = np.concatenate([tetra[:, indices] for indices in ((0,1,2),(0,1,3),(0,2,3),(1,2,3))])
    parents = np.tile(np.arange(len(tetra)),4)
    _, first, counts = np.unique(np.sort(faces,axis=1),axis=0,return_index=True,return_counts=True)
    if np.any(counts > 2):
        raise ValueError('nonmanifold tetrahedral boundary')
    selected = first[counts == 1]
    return faces[selected], parents[selected]


def section_polygons(nodes, tetra, stresses):
    # x+y=0 passes through two diagonal bolt bores. Values remain constant
    # within each linear tetrahedron; no invented sub-element stress gradient.
    normal = np.array([1.,1.,0.])/np.sqrt(2)
    basis = np.array([[1.,-1.,0.],[0.,0.,1.]])/np.array([[np.sqrt(2)],[1.]])
    polygons, values = [], []
    for tet, stress in zip(nodes[tetra], stresses):
        signed = tet @ normal
        if np.min(signed)>0 or np.max(signed)<0:
            continue
        points = []
        for a,b in ((0,1),(0,2),(0,3),(1,2),(1,3),(2,3)):
            if abs(signed[a]) < 1e-12:
                points.append(tet[a])
            if signed[a]*signed[b] < 0:
                fraction = signed[a]/(signed[a]-signed[b])
                points.append(tet[a]+fraction*(tet[b]-tet[a]))
        if len(points)<3:
            continue
        projected = np.unique(np.round(np.asarray(points) @ basis.T,12),axis=0)
        if len(projected)<3:
            continue
        center=projected.mean(axis=0)
        angles=np.arctan2(projected[:,1]-center[1],projected[:,0]-center[0])
        polygons.append(projected[np.argsort(angles)]*1000)
        values.append(stress)
    if not polygons:
        raise ValueError('section did not intersect the tetrahedral volume')
    return polygons,np.asarray(values)


def render(directory):
    report_path=directory/'report.json'
    report=json.loads(report_path.read_text())
    data=load_verified_level(directory,report)
    nodes,tetra,stress=data['nodes_m'],data['tetra'],data['von_mises_pa']/1e6
    triangles,parents=boundary_faces(tetra)
    maximum=float(np.max(stress))
    norm=Normalize(0,maximum)
    cmap=plt.get_cmap('turbo')
    title=f"LEFT INTERFACE FLANGE  |  {report['snapshot_time_s']:.3f} s fixed load"
    subtitle='Assumed material / clamp / coaxial transfer  |  static one-way FEM'
    metrics=report['metrics']
    footer=(f"P95 {metrics['volume_p95_von_mises_pa']/1e6:.3f} MPa   |   "
            f"peak {maximum:.3f} MPa   |   displacement {metrics['max_displacement_m']*1e6:.3f} um\n"
            f"{len(tetra):,} tet4 elements   |   actual size 63 x 11 mm   |   no deformation exaggeration")
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'text.color':TEXT,
                         'axes.labelcolor':TEXT,'xtick.color':TEXT,'ytick.color':TEXT})
    for kind in ('surface','cutaway'):
        fig=plt.figure(figsize=(9.6,7.2),dpi=120,facecolor=BACKGROUND)
        if kind=='surface':
            ax3d=cast(Axes3D,fig.add_axes((.04,.16,.76,.67),projection='3d',facecolor=BACKGROUND))
            collection=Poly3DCollection(nodes[triangles]*1000,facecolors=cmap(norm(stress[parents])),
                                       edgecolors='none',linewidths=0,antialiased=False)
            ax3d.add_collection3d(collection)
            ax3d.set(xlim=(-33,33),ylim=(-33,33),zlim=(0,11))
            ax3d.set_box_aspect((66,66,14));ax3d.view_init(elev=-27,azim=-65)
            ax3d.set_axis_off()
            fig.text(.055,.82,'DISTAL FACE + RECESSES',fontsize=12,color='#93b4c8')
        else:
            ax=fig.add_axes((.08,.30,.72,.36),facecolor=BACKGROUND)
            polygons,values=section_polygons(nodes,tetra,stress)
            collection=PolyCollection(polygons,facecolors=cmap(norm(values)),edgecolors='none',antialiased=False)
            ax.add_collection(collection);ax.autoscale_view();ax.set_aspect('equal')
            ax.set_xlabel('diagonal coordinate (mm)');ax.set_ylabel('height (mm)')
            for spine in ax.spines.values():spine.set_color('#426072')
            fig.text(.08,.78,'TRUE ELEMENT SECTION  |  x + y = 0',fontsize=12,color='#93b4c8')
            fig.text(.08,.69,'Blind bores and internal recesses remain empty.',fontsize=12)
        color_ax=fig.add_axes((.86,.29,.025,.41))
        bar=fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),cax=color_ax)
        bar.set_label('von Mises (MPa)',color=TEXT);bar.ax.tick_params(colors=TEXT)
        fig.text(.055,.94,title,fontsize=16,weight='bold')
        fig.text(.055,.89,subtitle,fontsize=11,color='#a7bdca')
        fig.text(.055,.065,footer,fontsize=12,linespacing=1.8)
        fig.savefig(directory/f'{kind}.png',facecolor=BACKGROUND)
        plt.close(fig)
    report['artifact_sha256']={f'{kind}.png':hashlib.sha256((directory/f'{kind}.png').read_bytes()).hexdigest()
                               for kind in ('surface','cutaway')}
    report['rendering']={'element_field':'constant von Mises per tet4; boundary triangles inherit adjacent tetra',
                         'section':'exact plane intersections with tetrahedra at x+y=0; no geometric hole filling',
                         'color_range_mpa':[0,maximum],'deformation_scale':0,
                         'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                         'field_verifier_sha256':hashlib.sha256((ROOT/'src/rby1_tray/structural_evidence.py').read_bytes()).hexdigest()}
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'images':report['artifact_sha256'],'range_mpa':[0,maximum]},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    render(parser.parse_args().directory)
