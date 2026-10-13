"""Command glyph direction, sharing and clipping without GPU/image fixtures."""
import inspect
import math
import unittest

from camber import modeling_icons, sketch_icons


class DrawCommands:
    def __init__(self):
        self.commands = []

    def AddLine(self, a, b, color, thickness=1):
        self.commands.append(("line", (a, b), color, thickness))

    def AddCircle(self, center, radius, color, segments=0, thickness=1):
        self.commands.append(("circle", ((center[0]-radius, center[1]-radius),
                                          (center[0]+radius, center[1]+radius)), color, thickness))

    def AddCircleFilled(self, center, radius, color, segments=0):
        self.AddCircle(center, radius, color, thickness=0)

    def AddRect(self, a, b, color, rounding=0, flags=0, thickness=1):
        self.commands.append(("rect", (a, b), color, thickness))

    def AddRectFilled(self, a, b, color):
        self.AddRect(a, b, color, thickness=0)

    def AddTriangleFilled(self, a, b, c, color):
        self.commands.append(("triangle", (a, b, c), color, 0))


def commands(drawer):
    drawlist = DrawCommands()
    drawer(drawlist, 0, 0, "ink", "accent")
    return drawlist.commands


class CommandIconTests(unittest.TestCase):
    def test_undo_redo_share_the_same_directional_glyph_across_workflows(self):
        undo, redo = commands(sketch_icons.draw_undo), commands(sketch_icons.draw_redo)
        self.assertEqual(undo, commands(modeling_icons.undo))
        self.assertEqual(redo, commands(modeling_icons.redo))
        self.assertEqual(len(undo), len(redo))
        for left, right in zip(undo, redo):
            self.assertEqual(left[0], right[0])
            self.assertEqual(left[2:], right[2:])
            for a, b in zip(left[1], right[1]):
                self.assertAlmostEqual(24-a[0], b[0])
                self.assertAlmostEqual(a[1], b[1])
        self.assertEqual(2, len(commands(sketch_icons.draw_cancel)))
        self.assertGreater(len(undo), 2)  # Undo is a curved return arrow, not Cancel's cross.

    def test_all_command_icons_stay_inside_their_box_with_consistent_strokes(self):
        drawers = [(name, fn) for name, fn in vars(modeling_icons).items()
                   if callable(fn) and not name.startswith("_") and len(inspect.signature(fn).parameters) == 5]
        drawers.extend((name, fn) for name, fn in vars(sketch_icons).items()
                       if callable(fn) and name.startswith("draw_"))
        for name, drawer in drawers:
            with self.subTest(icon=name):
                glyph = commands(drawer)
                self.assertTrue(glyph)
                for _, points, _, thickness in glyph:
                    self.assertLessEqual(thickness, 2)
                    for point in points:
                        for value in point:
                            self.assertTrue(math.isfinite(value))
                            self.assertGreaterEqual(value-thickness/2, .5)
                            self.assertLessEqual(value+thickness/2, 23.5)


if __name__ == "__main__":
    unittest.main()
