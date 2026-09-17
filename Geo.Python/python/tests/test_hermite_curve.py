"""Public guide evaluation and asymmetric closed milling sweep regressions."""
import math
import unittest
from camber import Curve, Frame, Part, set_progress_log

set_progress_log(False)


class HermiteCurveTests(unittest.TestCase):
    def test_directions_and_normalized_parameter(self):
        curve = Curve.hermite([(0, 0, 0), (2, 0, 0), (2, 3, 0)],
                              [(7, 0, 0), (1, 1, 0), (0, 8, 0)])
        self.assertEqual(tuple(curve.point(.5)), (2, 0, 0))
        self.assertEqual(tuple(curve.tangent(0)), (1, 0, 0))
        self.assertAlmostEqual(sum(v*v for v in curve.tangent(.5)), 1)
        for parameter in (-.01, 1.01, math.inf, math.nan):
            with self.subTest(parameter=parameter), self.assertRaises(Exception):
                curve.point(parameter)

    def test_invalid_knot_data_rejected(self):
        cases = [([], []), ([(0, 0, 0), (1, 0, 0)], [(1, 0, 0)]),
                 ([(0, 0, 0), (1, 0, 0)], [(1, 0, 0), (0, 0, 0)]),
                 ([(0, 0, 0), (math.inf, 0, 0)], [(1, 0, 0)]*2),
                 ([(0, 0, 0)]*2, [(1, 0, 0)]*2),
                 ([(0, 0, 0), (2, 1, 0), (0, 0, 0)],
                  [(1, 0, 0), (0, 1, 0), (0, -1, 0)])]
        for points, directions in cases:
            with self.subTest(points=points), self.assertRaises(Exception):
                Curve.hermite(points, directions)

    def test_closed_asymmetric_profile_and_reference_orientation(self):
        points, directions = [], []
        for index in range(24):
            angle = 2*math.pi*index/24
            points.append((20*math.cos(angle), 20*math.sin(angle), 2*math.sin(2*angle)))
            directions.append((-20*math.sin(angle), 20*math.cos(angle), 4*math.cos(2*angle)))
        points.append(points[0]); directions.append(directions[0])
        curve = Curve.hermite(points, directions)
        tangent = curve.tangent(0)
        self.assertEqual(tuple(curve.point(0)), tuple(curve.point(1)))
        for reference in (None, (0, 0, 1)):
            with self.subTest(reference=reference):
                part = Part((-30, -30, -30), (30, 30, 30), tolerance=.03)
                frame = Frame(points[0], x=(1, 0, 0), y=(0, tangent.z, -tangent.y), z=tangent)
                profile = part.sketch(frame=frame)
                profile.add_rectangle((.25, -.2), (1.25, .4))
                solid = part.extrude_along_curve(profile, curve, reference_direction=reference)
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 70)
                self.assertLess(solid.volume(), 85)


if __name__ == '__main__':
    unittest.main()
