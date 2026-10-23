#!/usr/bin/env python3
"""Extract one identified STEP volume to local BREP without modifying topology."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import gmsh
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def prepare(step: Path, config: Path, output: Path):
    settings = json.loads(config.read_text())
    digest = hashlib.sha256(step.read_bytes()).hexdigest()
    if digest != settings['source_step_sha256']:
        raise ValueError('STEP hash differs from the reviewed component selection')
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.Terminal', 0)
        gmsh.option.setString('Geometry.OCCTargetUnit', 'M')
        gmsh.option.setNumber('Geometry.OCCImportLabels', 1)
        gmsh.model.occ.importShapes(str(step))
        gmsh.model.occ.synchronize()
        volume = int(settings['selected_import_volume_tag'])
        if (3, volume) not in gmsh.model.getEntities(3):
            raise ValueError('selected volume is absent')
        bbox = gmsh.model.getBoundingBox(3, volume)
        mass = gmsh.model.occ.getMass(3, volume)
        if not np.allclose(bbox, settings['expected_bbox_m'], rtol=0, atol=2e-7):
            raise ValueError('selected volume bounding box differs')
        if not np.isclose(mass, settings['expected_volume_m3'], rtol=1e-7, atol=0):
            raise ValueError('selected volume measure differs')
        faces = [dict(tag=t, type=gmsh.model.getType(d, t),
                      area_m2=gmsh.model.occ.getMass(d, t),
                      center_m=gmsh.model.occ.getCenterOfMass(d, t),
                      bbox_m=gmsh.model.getBoundingBox(d, t))
                 for d, t in gmsh.model.getBoundary([(3, volume)], oriented=False)]
        # Upstream's visible-only BREP export avoids expensive recursive deletion
        # and leaves the source assembly and selected solid topology untouched.
        gmsh.model.setVisibility(gmsh.model.getEntities(), 0)
        gmsh.model.setVisibility([(3, volume)], 1, recursive=True)
        gmsh.option.setNumber('Geometry.OCCExportOnlyVisible', 1)
        brep = output / 'left-interface-flange.brep'
        gmsh.write(str(brep))
        report = dict(schema_version=1, gmsh_version=gmsh.__version__,
                      source_step_sha256=digest, selected_import_volume_tag=volume,
                      imported_entity_counts={str(d): len(gmsh.model.getEntities(d)) for d in range(4)},
                      name=gmsh.model.getEntityName(3, volume), bbox_m=bbox,
                      volume_m3=mass, faces=faces,
                      brep_sha256=hashlib.sha256(brep.read_bytes()).hexdigest(),
                      elapsed_s=time.monotonic()-started,
                      scope='OCC import and positive volume; not a CAD integrity certification')
        (output / 'cad-selection.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'faces'}, indent=2))
        return report
    finally:
        gmsh.finalize()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/structural-flange.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.step, args.config, args.output)
