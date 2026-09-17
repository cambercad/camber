"""An explicit solve must distinguish inconsistent dimensions from success."""
import unittest

from camber import Part, set_progress_log


class SketchSolveResultTests(unittest.TestCase):
    def test_inconsistent_fixed_endpoint_raises_at_solve(self):
        set_progress_log(False)
        part = Part((-10, -10, -10), (10, 10, 10), tolerance=.01)
        sketch = part.sketch("xy", constrained=True)
        sketch.solve_after_every_constraint = False
        line = sketch.add_line((0, 0), (2, 0))
        sketch.fix(line @ 0).fix(line @ 1)
        self.assertIs(sketch.solve(), sketch)
        sketch.coincident(line @ 0, ("xy", 1, 0))
        with self.assertRaisesRegex(Exception, "Sketch constraint solve failed"):
            sketch.solve()

    def test_tangent_arc_endpoint_stays_coincident(self):
        part = Part((-30, -30, -10), (30, 30, 10), tolerance=.01)
        sketch = part.sketch("xy", constrained=True)
        sketch.solve_after_every_constraint = False
        crown = sketch.add_arc((13, 0), (0, 13), (-13, 0))
        shoulder = sketch.add_arc((8.8, -9.9), (12, -6), (13, 0))
        sketch.fix(crown @ "center").radius(crown, 13)
        sketch.fix(crown @ 0).fix(crown @ 1).fix(shoulder @ 0)
        sketch.coincident(shoulder @ 1, crown @ 0)
        sketch.tangent_circles(crown, shoulder)
        sketch.solve()
        for point, expected in ((shoulder @ 0, (8.8, -9.9)),
                                (shoulder @ 1, (13, 0))):
            actual = sketch.eval_xy(point)
            self.assertAlmostEqual(actual[0], expected[0], delta=1e-6)
            self.assertAlmostEqual(actual[1], expected[1], delta=1e-6)
        self.assertAlmostEqual(sketch.eval_xy(shoulder @ "center")[1], 0, delta=1e-5)

    def test_solved_tangent_crown_extrudes_as_one_closed_profile(self):
        part = Part((-40, 280, -10), (40, 350, 10), tolerance=.01)
        sketch = part.sketch("xy", constrained=True)
        sketch.solve_after_every_constraint = False
        left = sketch.add_arc((-10.6, 314.1), (-13, 319), (-14, 325))
        crown = sketch.add_arc((-14, 325), (0, 339), (14, 325))
        right = sketch.add_arc((14, 325), (13, 319), (10.6, 314.1))
        base = sketch.add_line((10.6, 314.1), (-10.6, 314.1))
        axis = sketch.add_line((-20, 325), (20, 325), construction=True)
        sketch.fix(base @ 0).fix(base @ 1).fix(axis @ 0).fix(axis @ 1)
        sketch.radius(crown, 14).fix(crown @ "center")
        sketch.point_on_line(crown @ 0, axis).point_on_line(crown @ 1, axis)
        sketch.coincident(left @ 1, crown @ 0).coincident(right @ 0, crown @ 1)
        sketch.fix(left @ 0).fix(right @ 1)
        sketch.tangent_circles(crown, left).tangent_circles(crown, right)
        sketch.solve()
        solid = part.extrude(sketch, 1)
        self.assertTrue(solid.is_watertight())
        self.assertGreater(solid.volume(), 400)
