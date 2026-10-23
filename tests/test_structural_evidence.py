import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from rby1_tray.structural_evidence import load_verified_level


class StructuralEvidenceTests(unittest.TestCase):
    def test_mesh_hash_and_summary_are_checked_before_display(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);path=directory/'level-0.npz'
            np.savez(path,nodes_m=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]]),
                     tetra=np.array([[0,1,2,3]]),displacement_m=np.array([[0,0,0],[.1,0,0],[0,0,0],[0,0,0]]),
                     von_mises_pa=np.array([5.]),element_volume_m3=np.array([1/6]))
            metrics=dict(max_displacement_m=.1,peak_von_mises_pa=5.,volume_p95_von_mises_pa=5.,mesh_volume_m3=1/6)
            report=dict(mesh_levels=[dict(nodes=4,tetrahedra=1,**metrics)],metrics=metrics,
                        level_artifact_sha256={'level-0.npz':hashlib.sha256(path.read_bytes()).hexdigest()})
            self.assertEqual(len(load_verified_level(directory,report)['tetra']),1)
            wrong=json.loads(json.dumps(report));wrong['metrics']['peak_von_mises_pa']=10
            with self.assertRaisesRegex(ValueError,'differs'):
                load_verified_level(directory,wrong)
            with path.open('ab') as stream:stream.write(b'tamper')
            with self.assertRaisesRegex(ValueError,'SHA-256'):
                load_verified_level(directory,report)


if __name__=='__main__':unittest.main()
