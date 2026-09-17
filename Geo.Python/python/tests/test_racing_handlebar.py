"""Drop-bar tangent layout and actual plug-to-tube receiving geometry."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, vec3, set_progress_log
from racing_handlebar import BAR_END, TAIL_DIRECTION, drop_bar_guide, bar_end_plug


class HandlebarTests(unittest.TestCase):
    def test_tangent_tail_and_hollow_plugs_fit_both_sides(self):
        set_progress_log(False)
        part = Part((-420, -300, -50), (1420, 300, 1120), tolerance=.2)
        for side in (-1, 1):
            with self.subTest(side=side):
                guide, frame = drop_bar_guide(part, side)
                plug = bar_end_plug(part, side)
                self.assertTrue(plug.is_watertight())
                origin = vec3(BAR_END[0], side*200, BAR_END[1])
                for label, outside, inside in (("carbon", 11.9, 10.3), ("tape", 13.2, 11.9)):
                    profile = part.sketch(frame=frame, constrained=True)
                    outer = profile.add_circle((0, 0), outside)
                    inner = profile.add_circle((0, 0), inside)
                    profile.radius(outer, outside).fix(outer@"center")
                    profile.radius(inner, inside).concentric(inner, outer)
                    tube = part.extrude_along_sketch(profile, guide,
                            name=f"{label}_{side}", max_deviation=.03)
                    self.assertTrue(tube.is_watertight())
                    # Every tail-end vertex lies on the specified seating plane;
                    # a frame accidentally carried across a kink fails by mm.
                    points, _ = tube.mesh()
                    distal = [v for v in points if v.x < 884]
                    self.assertGreater(len(distal), 20)
                    self.assertLess(max(abs((v-origin).dot(TAIL_DIRECTION)) for v in distal), .001)
                    contact = part.intersect(tube, plug)
                    self.assertLess(contact.volume(), .01)
                    points, triangles = contact.mesh()
                    # Permit only lattice-sized nominal seating contact, never
                    # overlap anywhere along the inserted stem or retaining ribs.
                    # CoordinateConverter uses one million divisions over the
                    # longest model dimension (1840 mm), truncating each axis.
                    grid_projection = 1840/999999 * sum(abs(c) for c in TAIL_DIRECTION)
                    for index in {i for triangle in triangles for i in triangle}:
                        self.assertLessEqual(abs((points[index]-origin).dot(TAIL_DIRECTION)), grid_projection)
                # The stem is hollow and the cap has a solid 2 mm centre wall.
                hit = part.raycast(plug, origin-TAIL_DIRECTION*20, TAIL_DIRECTION)
                self.assertIsNotNone(hit)
                self.assertAlmostEqual(1, (hit.point-origin).dot(TAIL_DIRECTION), delta=.002)


if __name__ == "__main__":
    unittest.main()
