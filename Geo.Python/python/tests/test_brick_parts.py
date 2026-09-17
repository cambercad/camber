"""Physical dimensions, underside features and stack fit of original parts."""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camber import Part, set_progress_log
from brick_catalog import PARTS
from brick_parts import build_rectangular, WORKING_LOW, WORKING_HIGH


class BasicPartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.owner = Part(WORKING_LOW, WORKING_HIGH, tolerance=0.025)
        cls.bodies = {
            key: build_rectangular(cls.owner, PARTS[key])
            for key in ("brick_1x1", "brick_2x4", "plate_2x4", "tile_2x2")
        }

    def test_closed_solids_and_actual_envelopes(self):
        for key, body in self.bodies.items():
            with self.subTest(key=key):
                self.assertTrue(body.is_watertight())
                self.assertGreater(body.signed_volume(), 0)
                points, triangles = body.mesh()
                used = {i for triangle in triangles for i in triangle}
                extent = [
                    max(points[i][axis] for i in used)
                    - min(points[i][axis] for i in used)
                    for axis in range(3)
                ]
                spec = PARTS[key]
                expected = (
                    spec.studs_x * 8 - 0.2,
                    spec.studs_y * 8 - 0.2,
                    spec.height_plates * 3.2 + (0 if spec.family == "tile" else 1.6),
                )
                for actual, target in zip(extent, expected):
                    self.assertAlmostEqual(actual, target, delta=0.002)

    def test_actual_cavity_roof_stud_core_and_side_clutch_ribs(self):
        body = self.bodies["brick_2x4"]
        roof = self.owner.raycast(body, (4, 0, -1), (0, 0, 1))
        stud_core = self.owner.raycast(body, (4, 4, -1), (0, 0, 1))
        self.assertIsNotNone(roof)
        self.assertIsNotNone(stud_core)
        self.assertAlmostEqual(roof.point.z, 8.4, delta=0.002)
        self.assertAlmostEqual(stud_core.point.z, 10.6, delta=0.002)
        rib = self.owner.raycast(body, (4, 4, 1), (1, 0, 0))
        wall = self.owner.raycast(body, (4, 0, 1), (1, 0, 0))
        self.assertAlmostEqual(rib.point.x, 6.4, delta=0.002)
        self.assertAlmostEqual(wall.point.x, 6.7, delta=0.002)
        # The central underside tube has an actual open bore and separate wall.
        tube = self.owner.raycast(body, (0, 0, 1), (1, 0, 0))
        outer = self.owner.raycast(body, (4, 0, 1), (-1, 0, 0))
        self.assertAlmostEqual(tube.point.x, 2.4, delta=0.03)
        self.assertAlmostEqual(outer.point.x, 3.25, delta=0.03)

    def test_nominal_stack_clearance_and_rejection_of_lateral_misalignment(self):
        body = self.bodies["brick_2x4"]
        assembly = self.owner.assembly("stack_fit")
        assembly.add_part(body)
        assembly.add_part(body, (0, 0, 9.6))
        self.assertEqual(assembly.interferences(min_volume=0.002), [])
        # A small lateral shift must engage walls/ribs; empty shells would pass
        # the first check but fail to represent the actual locating features.
        misplaced = self.owner.assembly("misaligned_stack")
        misplaced.add_part(body)
        misplaced.add_part(body, (0.4, 0, 9.6))
        self.assertGreater(sum(item.volume for item in misplaced.interferences()), 0.1)

    def test_tile_has_continuous_top_and_separator_rebate(self):
        body = self.bodies["tile_2x2"]
        top = self.owner.raycast(body, (0, 0, 5), (0, 0, -1))
        rebate = self.owner.raycast(body, (10, 0, 0.35), (-1, 0, 0))
        side = self.owner.raycast(body, (10, 0, 1), (-1, 0, 0))
        self.assertAlmostEqual(top.point.z, 3.2, delta=0.002)
        self.assertAlmostEqual(side.point.x - rebate.point.x, 0.25, delta=0.002)


if __name__ == "__main__":
    unittest.main()
