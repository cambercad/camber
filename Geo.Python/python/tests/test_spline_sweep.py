"""Regression coverage for the spline guides and narrow caps used by the bike."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Frame, Part, set_progress_log

set_progress_log(False)


class SplineSweepTests(unittest.TestCase):
    def test_spline_guide_is_not_silently_discarded(self):
        part = Part((-20, -20, -20), (50, 50, 50), tolerance=.05)
        guide = part.sketch("xy", name="guide")
        guide.add_spline([(0, 0), (10, 0), (20, 10)], start_tangent=(1, 0))
        profile = part.sketch(frame=Frame((0, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0)))
        profile.add_circle((0, 0), 1)
        swept = part.extrude_along_sketch(profile, guide, name="spline_pipe")
        self.assertTrue(swept.is_watertight())
        self.assertGreater(swept.volume(), math.pi*24*.9)
        points, _ = swept.mesh()
        self.assertGreater(max(p.y for p in points), 9.9)
        self.assertGreater(max(p.x for p in points), 19.9)

    def test_thin_tilted_sweep_cap_keeps_its_area(self):
        part = Part((-50, -50, -50), (100, 100, 100), tolerance=.05)
        direction = (1/math.sqrt(2), 0, 1/math.sqrt(2))
        frame = Frame((0, 0, 0), x=(0, 1, 0),
                      y=(-direction[2], 0, direction[0]), z=direction)
        profile = part.sketch(frame=frame, name="thin_profile")
        profile.add_rectangle((-1, -10), (1, 10))
        guide = part.sketch(frame=Frame((0, 0, 0), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)))
        guide.add_line((0, 0), (20, 20))
        swept = part.extrude_along_sketch(profile, guide, name="thin_diagonal")
        self.assertTrue(swept.is_watertight())
        self.assertAlmostEqual(swept.volume(), 40*math.sqrt(800), delta=1)


if __name__ == "__main__":
    unittest.main()
