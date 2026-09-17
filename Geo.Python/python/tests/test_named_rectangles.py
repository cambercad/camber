import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camber import Part, set_progress_log


class NamedRectangleTests(unittest.TestCase):
    def test_named_rectangles_remain_constrainable(self):
        set_progress_log(False)
        for centered in (False, True):
            with self.subTest(centered=centered):
                p = Part((-50, -50, -50), (50, 50, 50))
                sk = p.sketch("xy", name="plate", constrained=True)
                sk.solve_after_every_constraint = False
                names = ("bottom", "right", "top", "left")
                if centered:
                    sk.add_rectangle_centered((0, 0), 10, 6, names=names)
                else:
                    sides = sk.add_rectangle((-5, -3), (5, 3), names=names)
                    self.assertEqual(names, tuple(side.name for side in sides))
                sk.horizontal("bottom")
                sk.fix("bottom@0.000")
                sk.length("bottom", 14)
                sk.length("right", 8)
                sk.solve()
                x, y = sk.eval_xy("right@1.000")
                self.assertAlmostEqual(9, x, places=5)
                self.assertAlmostEqual(5, y, places=5)
                self.assertTrue(p.extrude(sk, 2).is_watertight())
