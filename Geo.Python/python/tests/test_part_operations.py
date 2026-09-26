import math
import unittest

import camber

from camber import Part, PartOperation, Sketch, Solid, set_progress_log
from camber.view import _load_operations, _operation_row_text


class PartOperationTests(unittest.TestCase):
    def test_operations_live_on_their_inputs_and_carry_doc_groups(self):
        self.assertFalse(hasattr(Part, "extrude"))
        self.assertFalse(hasattr(Part, "shell"))
        self.assertFalse(hasattr(Part, "fillet"))
        self.assertEqual("Build solids", Sketch.extrude.__api_group__)
        self.assertEqual("Hollow and draft", Solid.shell.__api_group__)
        self.assertEqual("Booleans", Part.batch_subtract.__api_group__)

    def test_extrude_taper_angle_uses_signed_draft_and_records_normal_extrude(self):
        set_progress_log(False)
        part = Part((-10, -10, -2), (10, 10, 4), tolerance=.005)
        sketch = part.sketch("xy", name="draft_profile")
        sketch.add_rectangle((-3, -2.5), (3, 2.5))

        drafted = sketch.extrude(1, taper_angle=math.pi / 18, name="drafted")

        self.assertTrue(drafted.is_watertight())
        points, _ = drafted.mesh()
        top = [point for point in points if point.z > .99]
        offset = math.tan(math.pi / 18)
        self.assertAlmostEqual(-3 + offset, min(point.x for point in top), places=2)
        self.assertAlmostEqual(3 - offset, max(point.x for point in top), places=2)
        self.assertEqual("Extrude", part.operations[-1].kind)
        self.assertIn("taperAngle=", part.operations[-1].details)

    def test_tapered_extrude_rejects_a_collapsed_profile(self):
        set_progress_log(False)
        part = Part((-10, -10, -2), (10, 10, 12), tolerance=.01)
        sketch = part.sketch("xy", name="small_profile")
        sketch.add_rectangle((-0.5, -0.5), (0.5, 0.5))

        with self.assertRaises(Exception):
            sketch.extrude(10, taper_angle=math.pi / 4)


    def test_extrude_until_named_face_uses_native_surface_trim(self):
        set_progress_log(False)
        part = Part((-6, -6, -6), (12, 12, 12), tolerance=.001)
        target = part.cuboid((-2, -2, 5), (2, 2, 10), name="target")
        bottom = next(name for name in target.patch_names if name.endswith("ExtrudeBottom"))
        sketch = part.sketch("xy", name="profile")
        sketch.add_rectangle((-.5, -.5), (.5, .5))

        surface = target.face_surface(bottom)
        direct = sketch.extrude_until_surface(surface)
        by_name = sketch.extrude_until_face(target, bottom)

        self.assertFalse(surface.is_volume)
        self.assertTrue(direct.is_watertight())
        self.assertTrue(by_name.is_watertight())
        self.assertAlmostEqual(5, direct.volume(), delta=.01)
        self.assertAlmostEqual(5, by_name.volume(), delta=.01)

    def test_feature_ledger_records_successful_model_operations(self):
        set_progress_log(False)
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.01)
        base = part.cuboid((0, 0, 0), (4, 4, 4), name="base")
        tool = part.cylinder(origin=(2, 2, -1), radius=.5, height=6, name="tool")
        drilled = part.subtract(base, tool, name="drilled")
        edge = next(name for name in drilled.curve_names if "ExtrudeTop" in name)
        finished = drilled.fillet(edge, .2, name="finished")

        operations = part.operations
        self.assertEqual(["Cuboid", "Cylinder", "Subtract", "Fillet"], [item.kind for item in operations])
        self.assertTrue(all(isinstance(item, PartOperation) for item in operations))
        self.assertEqual("finished", operations[-1].result)
        self.assertEqual(("drilled",), operations[-1].inputs)
        self.assertEqual((edge.split(":", 1)[1],), operations[-1].entities)
        self.assertIn("radius=0.2", operations[-1].details)
        self.assertEqual(operations, tuple(_load_operations(part)))
        self.assertIn("Fillet: finished", _operation_row_text(operations[-1]))
        self.assertEqual(finished.name, operations[-1].result)

    def test_failed_operation_is_not_recorded(self):
        set_progress_log(False)
        part = Part((-2, -2, -2), (2, 2, 2), tolerance=.01)
        body = part.cuboid((0, 0, 0), (1, 1, 1), name="body")
        before = part.operations
        with self.assertRaises(Exception):
            body.fillet("not-a-curve", .2)
        self.assertEqual(before, part.operations)

    def test_batch_subtract_accepts_pattern_generator_and_records_subtract(self):
        set_progress_log(False)
        part = Part((-2, -2, -2), (14, 8, 5), tolerance=.001)
        base = part.cuboid((0, 0, 0), (10, 6, 2), name="base")
        tool = part.cuboid((1, 1, -1), (3, 3, 3), name="tool")
        copies = tool.pattern_linear(2, (5, 0, 0), name="cutters")

        result = part.batch_subtract(base, (copy for copy in copies), name="perforated")

        self.assertTrue(result.is_watertight())
        self.assertAlmostEqual(104, result.volume(), delta=.001)
        self.assertAlmostEqual(120, base.volume(), places=6)
        self.assertEqual("Subtract", part.operations[-1].kind)
        self.assertEqual(("base", "tool", "cutters_1"), part.operations[-1].inputs)
        self.assertIs(base, part.batch_subtract(base, []))
        self.assertFalse(hasattr(part, "cut"))
        self.assertEqual(1, camber.BOOLEAN_SUBTRACT)
        self.assertFalse(hasattr(camber, "BOOLEAN_DIFFERENCE"))


if __name__ == "__main__":
    unittest.main()
