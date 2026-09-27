"""Regression coverage for the CadQuery assembly documentation ports."""

import os
import runpy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber import set_progress_log
from camber import Part
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

    def test_independent_parts_with_different_lattices_can_be_assembled(self):
        a = Workplane(size=10, tolerance=0.01).box(2, 2, 2)
        b = Workplane(size=100, tolerance=0.001).box(2, 2, 2)
        result = Assembly().add(a, name="a").add(
            b, name="b", loc=Location((8, 0, 0)))

        leaves = result.leaves()
        self.assertEqual(2, len(leaves))
        self.assertNotEqual(a.part.smallest_unit(), b.part.smallest_unit())
        self.assertEqual((8, 0, 0), tuple(leaves[1].frame.origin))
        self.assertEqual((0, 0, 0), tuple(leaves[0].frame.origin))

    def test_independent_subassembly_keeps_its_internal_parts_and_mates(self):
        child_part = Part((-20, -20, -20), (20, 20, 20), tolerance=0.01)
        child = child_part.assembly("independent_module")
        first = child.add_part(child_part.cuboid((0, 0, 0), (10, 10, 10)))
        child.fix(first)

        parent_part = Part((-200, -200, -200), (200, 200, 200), tolerance=0.001)
        parent = parent_part.assembly("independent_parent")
        occurrence = parent.add_subassembly(child, (30, 0, 0))
        parent.fix(occurrence)

        self.assertEqual(1, len(child.constraints))
        self.assertEqual(1, len(parent.leaves()))
        leaf = parent.leaves()[0]
        self.assertAlmostEqual(30, leaf.frame.origin.x, places=6)
        self.assertAlmostEqual(10, leaf.bounds[1].x - leaf.bounds[0].x, delta=0.01)

    def test_repeated_subassembly_definition_has_occurrence_scoped_parts(self):
        component_part = Part((-20, -20, -20), (20, 20, 20), tolerance=0.01)
        component = component_part.assembly("reusable_component")
        body = component.add_part(component_part.cuboid((0, 0, 0), (4, 4, 4)))
        component.fix(body)

        module_part = Part((-40, -40, -40), (40, 40, 40), tolerance=0.01)
        module = module_part.assembly("reusable_module")
        module.fix(module.add_subassembly(component, (2, 0, 0)))

        parent_part = Part((-100, -100, -100), (100, 100, 100), tolerance=0.001)
        parent = parent_part.assembly("repeated_parent")
        left = parent.add_subassembly(module, (10, 0, 0))
        right = parent.add_subassembly(module, (30, 0, 0))

        left_part = left.subassemblies[0].parts[0]
        right_part = right.subassemblies[0].parts[0]
        self.assertEqual(left.assembly.name, right.assembly.name)
        self.assertEqual(left_part.name, right_part.name)
        parent.fix(left_part)
        parent.fix(right_part)
        self.assertTrue(parent.solve().converged)

        leaves = parent.leaves()
        self.assertEqual(2, len(leaves))
        self.assertEqual([12, 32], sorted(round(leaf.frame.origin.x) for leaf in leaves))
        self.assertNotEqual(leaves[0].path, leaves[1].path)

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
