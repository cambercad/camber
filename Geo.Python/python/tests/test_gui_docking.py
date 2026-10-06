"""Dock layout behavior independent of an OpenGL context."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.gui import DockLayout


class DockLayoutTests(unittest.TestCase):
    def test_split_hide_and_restore_viewport(self):
        docks = DockLayout()
        docks.add("Features", "left")
        self.assertEqual((282, 0, 718, 600), docks.layout(1000, 600))
        self.assertEqual((0, 0, 278, 600), docks.rects["Features"][1])
        docks.set_visible("Features", False)
        self.assertEqual((0, 0, 1000, 600), docks.layout(1000, 600))
        docks.set_visible("Features", True)
        self.assertEqual((282, 0, 718, 600), docks.layout(1000, 600))

    def test_tabs_and_reparenting(self):
        docks = DockLayout()
        docks.add("Parts", "left")
        docks.add("Constraints", "center", "Parts")
        docks.layout(1000, 600)
        self.assertIs(docks._leaf("Parts"), docks._leaf("Constraints"))
        self.assertEqual("Constraints", docks._leaf("Parts")["active"])
        docks.move("Constraints", DockLayout.VIEWPORT, "right")
        docks.layout(1000, 600)
        self.assertIsNot(docks._leaf("Parts"), docks._leaf("Constraints"))
        self.assertEqual(2, len(docks.splitters))

    def test_drop_target(self):
        docks = DockLayout()
        docks.add("Features", "left")
        docks.layout(1000, 600)
        self.assertEqual((DockLayout.VIEWPORT, "right"), docks.drop_target((990, 300))[:2])
        self.assertEqual(("Features", "center"), docks.drop_target((140, 300))[:2])


if __name__ == "__main__":
    unittest.main()
