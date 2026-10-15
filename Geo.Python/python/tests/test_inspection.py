"""Headless bounds and named distance, using the same display names as the viewer."""
import unittest

from camber import Part, set_progress_log


class InspectionTests(unittest.TestCase):
    def setUp(self):
        set_progress_log(False)
        self.part = Part((-20, -20, -20), (20, 20, 20), tolerance=.01, unit="mm")
        sketch = self.part.sketch("xy", name="profile")
        sketch.add_rectangle((0, 0), (4, 6))
        self.solid = sketch.extrude(3, name="block")

    def test_bounds_match_the_extruded_box(self):
        low, high = self.solid.bounds()
        self.assertAlmostEqual(0, low[0], places=6)
        self.assertAlmostEqual(0, low[1], places=6)
        self.assertAlmostEqual(0, low[2], places=6)
        self.assertAlmostEqual(4, high[0], places=5)
        self.assertAlmostEqual(6, high[1], places=5)
        self.assertAlmostEqual(3, high[2], places=5)
        part_low, part_high = self.part.bounds()
        self.assertEqual(tuple(low), tuple(part_low))
        self.assertEqual(tuple(high), tuple(part_high))

    def test_distance_between_opposite_faces_is_the_height(self):
        top = next(name for name in self.solid.patch_names if name.endswith("ExtrudeTop"))
        bottom = next(name for name in self.solid.patch_names if name.endswith("ExtrudeBottom"))
        measured = self.solid.distance(top, bottom)
        self.assertAlmostEqual(3, measured.length, places=5)
        same = self.part.distance(top, bottom)
        self.assertAlmostEqual(measured.length, same.length, places=6)
        self.assertEqual(top, measured.first_name)

    def test_missing_name_reports_existing_faces(self):
        with self.assertRaisesRegex(ValueError, "block-ExtrudeTop"):
            self.solid.distance("missing", self.solid.patch_names[0])

    def test_undeclared_unit_stays_none(self):
        plain = Part((-1, -1, -1), (1, 1, 1))
        self.assertIsNone(plain.unit)
        self.assertIsNone(plain.bounds())
        with self.assertRaisesRegex(ValueError, "unit"):
            Part((-1, -1, -1), (1, 1, 1), unit="mm m")
