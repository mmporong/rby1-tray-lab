import json
from pathlib import Path
import shutil
import subprocess
import unittest


SCRIPT = Path(__file__).parents[1] / "src/rby1_tray/web/structural-diagnostics.js"


def run_node(expression):
    program = f"""
const d = require({json.dumps(str(SCRIPT))});
const result = {expression};
process.stdout.write(JSON.stringify(result));
"""
    completed = subprocess.run(
        ["node", "-e", program], check=True, capture_output=True, text=True
    )
    return json.loads(completed.stdout)


@unittest.skipUnless(shutil.which("node"), "Node.js is required for JavaScript numeric regression tests")
class StructuralDiagnosticsTests(unittest.TestCase):
    def assertClose(self, actual, expected, relative=1e-12, absolute=1e-12):
        self.assertLessEqual(abs(actual - expected), absolute + relative * abs(expected))

    def test_joint_load_matches_names_and_preserves_over_limit_value(self):
        got = run_node("d.jointLoads({names:['b','a'],actuator_force:[12,-5]}, {available:true,joints:[{name:'a',limit_nm:10},{name:'b',limit_nm:10}]})")
        self.assertTrue(got['available'])
        self.assertEqual([j['torque_nm'] for j in got['joints']], [-5, 12])
        self.assertEqual([j['load_fraction'] for j in got['joints']], [.5, 1.2])
        self.assertEqual([j['fill_fraction'] for j in got['joints']], [.5, 1])

    def test_invalid_joint_load_is_unavailable_instead_of_false_zero(self):
        states = ["null", "{names:['a','a'],actuator_force:[0,1]}", "{names:['b'],actuator_force:[0]}", "{names:['a'],actuator_force:[NaN]}"]
        for state in states:
            got = run_node(f"d.jointLoads({state},{{available:true,joints:[{{name:'a',limit_nm:10}}]}})")
            self.assertFalse(got['available'])
            self.assertEqual(got['joints'], [])

    def test_resultant_resolves_signed_axial_torsion_and_norms(self):
        got = run_node("d.resultant([3,4,-12],[6,8,-10])")
        self.assertEqual(got["axial_n"], -12)
        self.assertEqual(got["shear_n"], 5)
        self.assertEqual(got["bending_nm"], 10)
        self.assertEqual(got["torsion_nm"], -10)
        self.assertEqual(got["force_norm_n"], 13)
        self.assertClose(got["moment_norm_nm"], 10 * 2**0.5)

    def test_section_point_pure_axial_bending_shear_and_torsion(self):
        section = '{A:2,I:4,J:5}'
        axial = run_node(f"d.sectionPoint([2,3,0],[0,0,10],[0,0,0],{section})")
        self.assertEqual(axial["axial_pa"], 5)
        self.assertEqual(axial["total_pa"], 5)

        bending = run_node(f"d.sectionPoint([2,3,0],[0,0,0],[8,12,0],{section})")
        self.assertEqual(bending["bending_pa"], 0)
        self.assertEqual(bending["total_pa"], 0)
        bending_nonzero = run_node(f"d.sectionPoint([2,3,0],[0,0,0],[8,0,0],{section})")
        self.assertEqual(bending_nonzero["bending_pa"], 6)
        self.assertEqual(bending_nonzero["bending_vm_pa"], 6)

        shear = run_node(f"d.sectionPoint([0,0,0],[6,8,0],[0,0,0],{section})")
        self.assertEqual(shear["shear_x_pa"], 3)
        self.assertEqual(shear["shear_y_pa"], 4)
        self.assertClose(shear["total_pa"], 5 * 3**0.5)
        self.assertClose(shear["shear_vm_pa"], 5 * 3**0.5)

        torsion = run_node(f"d.sectionPoint([2,3,0],[0,0,0],[0,0,10],{section})")
        self.assertEqual(torsion["torsion_x_pa"], -6)
        self.assertEqual(torsion["torsion_y_pa"], 4)
        self.assertClose(torsion["total_pa"], (3 * 52) ** 0.5)
        self.assertClose(torsion["torsion_vm_pa"], (3 * 52) ** 0.5)

    def test_station_moment_transport_and_opposing_stresses(self):
        transported = run_node(
            "d.sectionPoint([2,3,7],[11,13,0],[17,19,23],{area_m2:2,I_m4:4,J_m4:5})"
        )
        self.assertEqual(transported["station_moment_nm"], [108, -58, 23])

        # At this point axial tension and bending compression cancel. Isolated
        # indicators therefore must not be presented as additive percentages.
        cancelled = run_node("d.sectionPoint([1,0,0],[0,0,8],[0,16,0],{A:2,I:4,J:5})")
        self.assertEqual(cancelled["axial_pa"], 4)
        self.assertEqual(cancelled["bending_pa"], -4)
        self.assertEqual(cancelled["axial_vm_pa"], 4)
        self.assertEqual(cancelled["bending_vm_pa"], 4)
        self.assertEqual(cancelled["normal_pa"], 0)
        self.assertEqual(cancelled["total_pa"], 0)

    def test_fem_metrics_hydrostatic_shear_and_rotation_invariance(self):
        hydro = run_node("d.femTensorMetrics([9,9,9,0,0,0])")
        self.assertEqual(hydro["hydrostatic_pa"], 9)
        self.assertEqual(hydro["von_mises_pa"], 0)

        shear = run_node("d.femTensorMetrics([0,0,0,4,0,0])")
        self.assertClose(shear["shear_vm_pa"], 4 * 3**0.5)
        self.assertEqual(shear["normal_deviatoric_vm_pa"], 0)
        self.assertEqual(shear["von_mises_pa"], shear["shear_vm_pa"])

        # Rotate an arbitrary symmetric tensor by an orthonormal matrix in JS,
        # then verify all scalar invariants stay unchanged.
        pair = run_node("""(() => {
          const s=[[11,2,3],[2,-5,7],[3,7,13]];
          const a=.47,c=Math.cos(a),n=Math.sin(a);
          const r=[[c,-n,0],[n,c,0],[0,0,1]];
          const mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((v,x,k)=>v+x*b[k][j],0)));
          const rt=r[0].map((_,i)=>r.map(row=>row[i]));
          const q=mul(mul(r,s),rt);
          return [d.femTensorMetrics([11,-5,13,2,7,3]),
                  d.femTensorMetrics([q[0][0],q[1][1],q[2][2],q[0][1],q[1][2],q[2][0]])];
        })()""")
        for key in ("von_mises_pa", "hydrostatic_pa", "normal_deviatoric_vm_pa", "shear_vm_pa"):
            # The normal/shear split depends on the chosen axes; total VM and
            # hydrostatic are invariant, while the split recombines in quadrature.
            if key in ("von_mises_pa", "hydrostatic_pa"):
                self.assertClose(pair[0][key], pair[1][key])
        for metrics in pair:
            self.assertClose(
                metrics["von_mises_pa"],
                (metrics["normal_deviatoric_vm_pa"] ** 2 + metrics["shear_vm_pa"] ** 2) ** 0.5,
            )

    def test_invalid_inputs_fail_closed(self):
        cases = run_node("""[
          () => d.resultant([1,2],[1,2,3]),
          () => d.sectionPoint([0,0,0],[0,0,0],[0,0,0],{A:0,I:1,J:1}),
          () => d.femTensorMetrics([1,2,3,4,5,NaN])
        ].map(fn => { try { fn(); return false; } catch (_) { return true; } })""")
        self.assertEqual(cases, [True, True, True])


if __name__ == "__main__":
    unittest.main()
