import gzip
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from rby1_tray.structural_loads import transfer_sensor_wrench, select_recorded_load, validate_mesh_sizes


class StructuralLoadTests(unittest.TestCase):
    def test_sensor_reaction_rotates_and_transports_equivalent_nodal_wrench(self):
        points=np.array([[.02,0,.01],[-.01,.03,-.02],[0,-.02,.04]])
        forces=np.array([[1,2,3],[-2,1,4],[.5,-1,2]])
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        site=np.array([.1,.2,.3]);load=np.array([-.2,.1,.4])
        raw_force=forces.sum(axis=0)
        raw_moment=np.cross(points,forces).sum(axis=0)
        actual_force,actual_moment=transfer_sensor_wrench(raw_force,raw_moment,rotation,site,load)
        cad_points=points@rotation.T+site
        cad_forces=-forces@rotation.T
        np.testing.assert_allclose(actual_force,cad_forces.sum(axis=0),atol=1e-13)
        np.testing.assert_allclose(actual_moment,np.cross(cad_points-load,cad_forces).sum(axis=0),atol=1e-13)
        f,m=transfer_sensor_wrench([0,0,5.23],[0,0,0],np.eye(3),[0,0,0],[0,0,0])
        np.testing.assert_array_equal(f,[0,0,-5.23]);np.testing.assert_array_equal(m,[0,0,0])

    def test_invalid_rotation_and_nonfinite_inputs_are_rejected(self):
        for rotation in [np.diag([1,1,-1]),np.ones((3,3)),np.eye(2),np.eye(3)*np.nan]:
            with self.assertRaises(ValueError):
                transfer_sensor_wrench([1,2,3],[0,0,0],rotation,[0,0,0],[0,0,0])
        with self.assertRaises(ValueError):
            transfer_sensor_wrench([np.nan,0,0],[0,0,0],np.eye(3),[0,0,0],[0,0,0])

    def test_load_selection_obeys_phase_filter_and_preserves_packet_index(self):
        def packet(phase,time,torque):
            return dict(phase=phase,time_s=time,observation={'wrist_wrench_local':{
                'left':{'force_n':[1,2,3],'torque_nm':torque}}})
        rows=[packet('home',0,[99,0,0]),packet('carry',1,[3,4,0]),packet('carry',2,[0,0,6])]
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'commands.jsonl.gz'
            with gzip.open(path,'wt') as stream:
                for row in rows:stream.write(json.dumps(row)+'\n')
            result=select_recorded_load(path,['carry'])
            self.assertEqual(result['packet_index'],2)
            self.assertEqual(result['timestamp_sim_s'],2)
            self.assertEqual(result['selected_packet_count'],2)
            self.assertEqual(len(result['commands_sha256']),64)
            with self.assertRaises(ValueError):select_recorded_load(path,['absent'])

    def test_refinement_sizes_require_three_decreasing_levels(self):
        self.assertEqual(validate_mesh_sizes([.003,.002,.001]),[.003,.002,.001])
        for values in [[],[.001],[.002,.001],[.001,.002,.003],[.003,.003,.002],[.003,.002,0],[.003,.002,np.nan]]:
            with self.assertRaises(ValueError):validate_mesh_sizes(values)


if __name__=='__main__':unittest.main()
