"""Sketch inference uses screen distance independently of the drawing grid."""
import unittest
from unittest.mock import patch

from camber import sketch_ui


class SketchSnappingTests(unittest.TestCase):
    def test_zoom_controls_geometry_snap_radius_without_changing_grid(self):
        state = dict(pending=[(0, 0)], actions=[], grid=10, snap_grid=False)
        uv = (2, 2)
        with patch("camber.view._world_per_pixel", return_value=.04):
            state["snap_tolerance"] = sketch_ui._pick_tolerance(None, None, state)
        self.assertEqual(uv, sketch_ui._snap(uv, state))
        self.assertEqual((0, 0), sketch_ui._snap((.02, .02), state))
        with patch("camber.view._world_per_pixel", return_value=1):
            state["snap_tolerance"] = sketch_ui._pick_tolerance(None, None, state)
        self.assertEqual((0, 0), sketch_ui._snap(uv, state))

    def test_alt_bypasses_geometry_grid_and_inferred_references(self):
        state = dict(pending=[(0, 0)], actions=[], grid=10, snap_grid=True,
                     snap_tolerance=4, ignore_snap=True, hover_hit="origin")
        uv = (2, 2)
        self.assertEqual((uv, None), sketch_ui._snap_ref(uv, state))
        self.assertEqual(uv, sketch_ui._snap(uv, state))
        state["ignore_snap"] = False
        state["pending"] = []
        state["snap_tolerance"] = .1
        self.assertEqual((10, 10), sketch_ui._snap((7, 7), state))
