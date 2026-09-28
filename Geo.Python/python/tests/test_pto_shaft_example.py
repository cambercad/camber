"""Geometry and fit checks for the photo-referenced DEMA 67991 example."""
import importlib.util
import os
import sys
import unittest

EXAMPLE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "examples", "pto_shaft"))
if EXAMPLE not in sys.path:
    sys.path.insert(0, EXAMPLE)

spec = importlib.util.spec_from_file_location(
    "pto_shaft_assembly", os.path.join(EXAMPLE, "assembly.py"))
pto = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pto)


class PtoShaftExampleTests(unittest.TestCase):
    def test_both_published_poses_have_closed_fitted_parts(self):
        for extended, overall in ((False, 800), (True, 1100)):
            with self.subTest(overall=overall):
                assembly, solids, occurrences, solution = pto.build(extended)
                self.assertTrue(solution.converged, solution.message)
                self.assertFalse(solution.unsatisfied)
                self.assertGreaterEqual(len(assembly.constraints), 25)
                for name, solid in solids.items():
                    with self.subTest(pose=overall, component=name):
                        self.assertTrue(solid.is_watertight())
                        self.assertGreater(solid.volume(), 0)
                low, high = assembly.bounds()
                self.assertAlmostEqual(high.z - low.z, overall, delta=.02)
                self.assertAlmostEqual(high.x - low.x, 120, delta=.02)
                self.assertEqual([], assembly.interferences(min_volume=.001))
                self.assertGreaterEqual(pto.MIN_VISIBLE_ENGAGEMENT, 100)
                self.assertAlmostEqual(pto.CROSS_SPACING +
                                       (pto.TELESCOPIC_TRAVEL if extended else 0),
                                       occurrences["output_tube_yoke"].pose[2], delta=.02)

    def test_joint_detail_uses_reused_bearing_cartridges(self):
        _, solids, _, _ = pto.build()
        detail = pto.build_joint_detail(solids)
        self.assertEqual(4, len(detail.subassemblies))
        self.assertGreaterEqual(len(detail.leaves()), 100)
        self.assertEqual([], detail.interferences(min_volume=.001))
        self.assertTrue(any("spline_space" in name
                            for name in solids["input_yoke"].patch_names))
        self.assertTrue(any("spline_space" in name
                            for name in solids["output_yoke"].patch_names))


if __name__ == "__main__":
    unittest.main()
