"""Physical section and envelope checks for the original special-part catalogue."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, set_progress_log
from brick_catalog import SPECIAL_PARTS, PARTS
from brick_parts import (
    WORKING_LOW,
    WORKING_HIGH,
    PITCH,
    CLEARANCE,
    PLATE_HEIGHT,
    STUD_HEIGHT,
    WALL,
    ROOF,
    EDGE_ROUND,
    cylinder,
)
from brick_shapes import build_special


class SpecialPartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        set_progress_log(False)
        cls.models = {}
        failures = []
        # Complete the independent build matrix before running fit checks.
        # One bad variant must not hide the other configurations, nor require
        # a second construction pass just to discover their failures.
        for spec in SPECIAL_PARTS:
            try:
                part = Part(WORKING_LOW, WORKING_HIGH, tolerance=0.02)
                solid = build_special(part, spec)
                if not solid.is_watertight() or solid.signed_volume() <= 0:
                    raise AssertionError("not a closed, positively oriented solid")
                cls.models[spec.key] = (part, solid)
            except Exception as error:
                failures.append((spec.key, error))
        if failures:
            details = "\n\n".join(f"{key}: {error}" for key, error in failures)
            raise RuntimeError(
                f"{len(cls.models)}/{len(SPECIAL_PARTS)} special variants passed; "
                f"{len(failures)} failed:\n{details}"
            ) from failures[0][1]

    def test_all_25_variants_are_closed_positive_solids_with_the_catalogue_envelope(
        self,
    ):
        self.assertEqual(25, len(self.models))
        for spec in SPECIAL_PARTS:
            with self.subTest(part=spec.key):
                _, solid = self.models[spec.key]
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.signed_volume(), 0)
                points, triangles = solid.mesh()
                used = {v for triangle in triangles for v in triangle}
                positions = [points[v] for v in used]
                top = spec.height_plates * PLATE_HEIGHT
                if spec.family == "roof_ridge":
                    # A tangent round removes the sharp roof apex. Its exact
                    # circular crown sits below the nominal construction tip.
                    slope = (top - PLATE_HEIGHT) / (
                        (spec.studs_y * PITCH - CLEARANCE) / 2
                    )
                    top -= EDGE_ROUND * (math.sqrt(1 + slope * slope) - 1)
                else:
                    top += STUD_HEIGHT
                self.assertAlmostEqual(min(p.z for p in positions), 0, delta=0.002)
                # The curved ridge crown uses the authored .01 mm chord
                # deviation; flat platforms retain the stricter lattice bound.
                top_tolerance = 0.012 if spec.family == "roof_ridge" else 0.002
                self.assertAlmostEqual(
                    max(p.z for p in positions), top, delta=top_tolerance
                )
                self.assertAlmostEqual(
                    max(p.x for p in positions) - min(p.x for p in positions),
                    spec.studs_x * PITCH - CLEARANCE,
                    delta=0.045,
                )
                self.assertAlmostEqual(
                    max(p.y for p in positions) - min(p.y for p in positions),
                    spec.studs_y * PITCH - CLEARANCE,
                    delta=0.045,
                )

    def test_slope_and_ridge_have_true_normal_roof_thickness(self):
        for key, y in [("slope_2x2", 3.0), ("roof_ridge_2x2", 3.0)]:
            with self.subTest(part=key):
                part, solid = self.models[key]
                spec = PARTS[key]
                x = (spec.studs_x * PITCH - CLEARANCE) / 2 - WALL - 0.25
                outer = part.raycast(solid, (x, y, 20), (0, 0, -1))
                self.assertIsNotNone(outer)
                # Use the actual outer face normal, independently of the
                # builder's slope equation. The next exit is the inner skin.
                normal = outer.normal.normalized()
                self.assertGreater(normal.z, 0.5)
                start = outer.point - normal * 0.02
                inner = part.raycast(solid, start, -normal)
                self.assertIsNotNone(inner)
                self.assertLess(inner.normal.dot(normal), -0.999)
                self.assertAlmostEqual(
                    (outer.point - inner.point).dot(normal), ROOF, delta=0.003
                )

    def test_slope_has_only_the_high_rear_stud_row_and_an_open_underside(self):
        part, solid = self.models["slope_2x3"]
        for x in (-4, 4):
            rear = part.raycast(solid, (x, -8, 20), (0, 0, -1))
            self.assertAlmostEqual(rear.point.z, 9.6 + STUD_HEIGHT, delta=0.002)
            forward = part.raycast(solid, (x, 8, 20), (0, 0, -1))
            self.assertLess(forward.point.z, 6)
        cavity = part.raycast(solid, (6, -8, -1), (0, 0, 1))
        self.assertIsNotNone(cavity)
        self.assertGreater(cavity.point.z, 8)

    def test_round_tube_receives_a_stud_and_roof_remains_closed(self):
        part, solid = self.models["round_2x2_h3"]
        nominal = cylinder(part, "round_receiver_gauge", 2.39, STUD_HEIGHT, z=0.2)
        self.assertLess(part.intersect(solid, nominal).volume(), 1e-7)
        oversize = cylinder(part, "round_receiver_oversize", 2.6, STUD_HEIGHT, z=0.2)
        self.assertGreater(part.intersect(solid, oversize).volume(), 0.5)
        # A tube bore leads to the roof, not through the top skin.
        hit = part.raycast(solid, (0, 0, -1), (0, 0, 1))
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.point.z, 9.6 - ROOF, delta=0.002)
        _, large = self.models["round_4x4_h3"]
        lp = large._part
        for x, y in [(12, 4), (4, 12), (-12, -4), (-4, -12)]:
            hit = lp.raycast(large, (x, y, 20), (0, 0, -1))
            self.assertAlmostEqual(hit.point.z, 9.6 + STUD_HEIGHT, delta=0.002)
        self.assertIsNone(lp.raycast(large, (12, 12, 20), (0, 0, -1)))

    def test_arch_opening_is_curved_with_real_jambs_and_a_continuous_upper_web(self):
        for key in ["arch_1x4_h3", "arch_1x5_h12", "arch_1x12_h9"]:
            with self.subTest(part=key):
                part, solid = self.models[key]
                spec = PARTS[key]
                height = spec.height_plates * PLATE_HEIGHT
                length = spec.studs_y * PITCH - CLEARANCE
                half_span = (length - 2 * PITCH) / 2
                # The centre ray in X lies inside the hollow upper skin. A
                # ray in the side web resolves the actual circular soffit.
                x = (PITCH - CLEARANCE) / 2 - WALL / 2
                crown = part.raycast(solid, (x, 0, -1), (0, 0, 1))
                flank = part.raycast(solid, (x, half_span * 0.7, -1), (0, 0, 1))
                jamb = part.raycast(solid, (x, length / 2 - WALL / 2, -1), (0, 0, 1))
                self.assertAlmostEqual(crown.point.z, height - 2 * ROOF, delta=0.03)
                self.assertLess(flank.point.z, crown.point.z - 0.2)
                self.assertAlmostEqual(jamb.point.z, 0, delta=0.002)
                self.assertIsNone(part.raycast(solid, (-10, 0, 1), (1, 0, 0)))
                top_skin = part.raycast(solid, (0, 0, -1), (0, 0, 1))
                self.assertGreaterEqual(top_skin.point.z, height - ROOF - 0.002)


if __name__ == "__main__":
    unittest.main()
