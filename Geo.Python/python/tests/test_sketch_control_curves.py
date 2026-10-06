"""Analytic sketch control curves through the installed native bridge."""
import unittest

from camber import Frame, LoftOptions, Part


class SketchControlCurveTests(unittest.TestCase):
    def test_control_curves_survive_matched_multisection_loft(self):
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.01)
        sections, starts = [], []
        for i, width in enumerate((2, 3, 2.5)):
            sketch = part.sketch(frame=Frame((0, 0, i*3)))
            upper = sketch.add_bezier([(-width, 0), (-width, 2), (width, 2), (width, 0)])
            sketch.add_bspline([(width, 0), (width, -2), (-width, -2), (-width, 0)])
            sections.append(sketch)
            starts.append(upper.name)
        solid = part.loft(sections, LoftOptions(correspondence="vertices"), first_curves=starts)
        self.assertTrue(solid.is_watertight())
        self.assertGreater(solid.volume(), 0)
        self.assertEqual(len(solid.patch_names), 4)  # two authored sides and two caps

    def test_bezier_and_bspline_form_one_closed_extrudable_profile(self):
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.005)
        sketch = part.sketch(name="control_profile")
        upper = sketch.add_bezier([(-2, 0), (-2, 2), (2, 2), (2, 0)], name="upper")
        lower = sketch.add_bspline([(2, 0), (2, -2), (-2, -2), (-2, 0)], name="lower")
        self.assertEqual(sketch.curve_count, 2)
        self.assertNotEqual(upper.name, lower.name)
        solid = sketch.extrude(3)
        self.assertTrue(solid.is_watertight())
        self.assertGreater(solid.volume(), 0)

    def test_rational_quarter_circle_and_nonunit_knots(self):
        part = Part((-5, -5, -5), (5, 5, 5), tolerance=.001)
        sketch = part.sketch()
        sketch.add_bspline([(2, 0), (2, 2), (0, 2)], degree=2,
                           knots=[2, 2, 2, 5, 5, 5], weights=[1, 2**-.5, 1])
        sketch.add_line((0, 2), (0, 0))
        sketch.add_line((0, 0), (2, 0))
        solid = sketch.extrude(1)
        self.assertTrue(solid.is_watertight())
        self.assertAlmostEqual(solid.volume(), 3.141592653589793, delta=.01)


if __name__ == "__main__":
    unittest.main()
