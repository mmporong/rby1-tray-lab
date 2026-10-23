from types import SimpleNamespace
import unittest
import numpy as np
from rby1_tray.section_stress import section_basis,circular_section,section_von_mises,body_interaction_wrench


class SectionStressTests(unittest.TestCase):
    def test_axial_bending_torsion_and_transverse_nominal_shear(self):
        s=circular_section(.02,.003);r=s['radius_m'];p=np.array([[r,0,0],[0,r,0.]])
        np.testing.assert_allclose(section_von_mises(p,[0,0,10],[0,0,0],s),10/s['area_m2'])
        np.testing.assert_allclose(section_von_mises(p,[0,0,0],[0,2,0],s),[2*r/s['I_m4'],0])
        np.testing.assert_allclose(section_von_mises(p,[0,0,0],[0,0,2],s),np.sqrt(3)*2*r/s['J_m4'])
        np.testing.assert_allclose(section_von_mises([[0,0,0]],[5,0,0],[0,0,0],s),[np.sqrt(3)*5/s['area_m2']])

    def test_axial_station_transport_and_load_linearity(self):
        s=circular_section(.03,.004);p=np.array([[.03,0,.2],[0,.03,.1]])
        a=section_von_mises(p,[1,2,3],[.4,.5,.6],s)
        b=section_von_mises(p,[2,4,6],[.8,1,1.2],s)
        np.testing.assert_allclose(b,2*a)
        # A lateral force shifted along +z has My=-z*Fx.
        got=section_von_mises([[.03,0,.2]],[1,0,0],[0,0,0],s)[0]
        expected=np.sqrt((.2*.03/s['I_m4'])**2+3/s['area_m2']**2)
        self.assertAlmostEqual(got/expected,1)

    def test_wrench_uses_tree_root_com_and_requested_origin(self):
        model=SimpleNamespace(body_rootid=np.array([0,1,1]))
        data=SimpleNamespace(subtree_com=np.array([[0,0,0],[.1,.2,.3],[9,9,9.]]),cfrc_int=np.array([[0]*6,[0]*6,[1,2,3,4,5,6.]]))
        rotation=section_basis([1,0,0]);origin=np.array([.3,.1,.2])
        force,moment=body_interaction_wrench(model,data,2,origin,rotation)
        np.testing.assert_allclose(force,rotation.T@[4,5,6])
        np.testing.assert_allclose(moment,rotation.T@(np.array([1,2,3])+np.cross([-.2,.1,.1],[4,5,6])))

    def test_section_frames_and_invalid_properties(self):
        for axis in ([1,0,0],[0,1,0],[0,0,-1],[1,2,3]):
            b=section_basis(axis);np.testing.assert_allclose(b.T@b,np.eye(3),atol=1e-14);self.assertAlmostEqual(np.linalg.det(b),1)
        for radius,thickness in ((0,.1),(.1,0),(.1,.2),(np.nan,.1)):
            with self.assertRaises(ValueError):circular_section(radius,thickness)
        with self.assertRaises(ValueError):section_basis([0,0,0])


if __name__=='__main__':unittest.main()
