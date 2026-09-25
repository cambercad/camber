"""Regression coverage for the CadQuery assembly documentation ports."""

import os
import runpy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import set_progress_log
from camber.cqcompat import Assembly, Location, Workplane


GALLERY = Path(__file__).resolve().parents[1] / "examples" / "cadquery_assembly_gallery"
sys.path.insert(0, str(GALLERY))


def build_sample(prefix):
    path = next(GALLERY.glob(prefix + "*.py"))
    return runpy.run_path(str(path))["build"]()


class AssemblyBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)

    def test_all_supported_documentation_samples_build(self):
        for prefix, count in (("02_", 2), ("04_", 2), ("05_", 2),
                              ("06_", 2), ("06b_", 2),
                              ("07_", 2), ("08_", 2), ("09_", 3),
                              ("10_", 2), ("11_", 2)):
            with self.subTest(sample=prefix):
                result = build_sample(prefix)
                self.assertEqual(count, len(result.leaves()))
                self.assertGreater(len(result.display_scene().patches), 0)

    def test_point_on_line_reaches_box_corner(self):
        result = build_sample("08_")
        constraints = result._assembly.constraints
        for mate in constraints[1:]:
            self.assertAlmostEqual(0.5, mate.first.world_point.x, places=5)
            self.assertAlmostEqual(0.5, mate.first.world_point.y, places=5)
            self.assertAlmostEqual(0.5, mate.first.world_point.z, places=5)

    def test_fixed_point_uses_grounded_world_datum(self):
        result = build_sample("09_")
        for mate in result._assembly.constraints[1:]:
            for value in mate.first.world_point:
                self.assertAlmostEqual(0.5, value, places=5)
        first, sphere, _ = result.leaves()
        for value in first.bounds[0]:
            self.assertAlmostEqual(-0.5, value, delta=0.001)
        for lower, upper in zip(sphere.bounds[0], sphere.bounds[1]):
            self.assertAlmostEqual(0.5, (lower + upper) / 2, delta=0.001)

    def test_cross_part_is_rejected_without_remeshing(self):
        a = Workplane().box(1, 1, 1)
        b = Workplane().box(1, 1, 1)
        assembly = Assembly().add(a, name="a")
        with self.assertRaisesRegex(ValueError, "share one Part"):
            assembly.add(b, name="b")

    def test_instance_names_and_location(self):
        result = build_sample("02_")
        names = [patch["name"] for patch in result.display_scene().patches]
        self.assertTrue(any(name.startswith("cone0/") for name in names))
        self.assertTrue(any(name.startswith("cone1/") for name in names))
        self.assertEqual((0, 0, 0), tuple(result.leaves()[0].frame.origin))
        self.assertEqual((0, 0, 0, 1), Location().orientation)

    def test_parametric_surface_is_open_and_can_be_mated(self):
        for prefix in ("06_", "06b_"):
            result = build_sample(prefix)
            self.assertFalse(result.leaves()[0].solid.is_volume)
            self.assertTrue(result._assembly.solve().converged)

    def test_unimplemented_samples_state_the_gap(self):
        for prefix in ("01_", "03_"):
            with self.subTest(sample=prefix):
                with self.assertRaises(NotImplementedError):
                    build_sample(prefix)


if __name__ == "__main__":
    unittest.main()
