"""Closed contours and downstream edits through the full modeling GUI."""
import math
import os
from pathlib import Path
import tempfile
import unittest

from camber import set_progress_log
from camber.gui_testing import GuiDriver
from camber.modeling import ModelSession, _Workspace


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class SketchWorkflowTests(unittest.TestCase):
    def test_slot_tangency_and_reopened_radius_edit_rebuild_solid(self):
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            model = ModelSession(Path(folder) / "slotted_plate.py")
            self.assertTrue(model.rebuild(), model.error)
            workspace = _Workspace(model)
            workspace.viewer.window.set_visible(False)
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame)

            def point(x, y):
                ndc = workspace.viewer.camera.project((x, y, 0))
                driver.click(x=(ndc[0]+1)*driver.window.width/2,
                             y=(1-ndc[1])*driver.window.height/2)

            def draw(tool, points):
                driver.click(tool, panel="Model")
                for xy in points:
                    point(*xy)
                driver.key("ESCAPE")

            def enter(label, value):
                driver.click(label, panel="Properties")
                driver.text(str(value), replace=True)

            try:
                driver.click("Create sketch", panel="Properties")
                driver.click("Create sketch", panel="Properties")
                driver.scroll(900, 600, -2)
                draw("Rectangle", ((-25, -15), (25, 15)))
                driver.click("Horizontal", panel="Model")
                point(0, -15)
                driver.click("Length", panel="Model")
                point(0, -15)
                enter("##sk_dimension", 50)
                driver.click("Apply##sk_act_dimension_apply", panel="Properties")
                state = workspace.sketch_session["state"]
                before = (len(state["actions"]), state["sketch"].constraint_count)
                driver.click("Vertical", panel="Model")
                point(0, -15)
                self.assertIn("previous sketch retained", state["status"])
                self.assertEqual(before, (len(state["actions"]), state["sketch"].constraint_count))
                self.assertFalse(state.get("live_invalid"))
                driver.key("ESCAPE")
                draw("Circle", ((0, 0), (6, 0)))
                driver.click("Radius", panel="Model")
                point(6, 0)
                enter("##sk_dimension", 6)
                driver.click("Apply##sk_act_dimension_apply", panel="Properties")
                draw("Line", ((-15, -4), (-15, 4)))
                draw("Arc", ((-15, 4), (-9, 4), (-12, 7)))
                draw("Line", ((-9, 4), (-9, -4)))
                draw("Arc", ((-9, -4), (-15, -4), (-12, -7)))
                for side, end in ((-15, 7), (-9, 7), (-9, -7), (-15, -7)):
                    driver.click("Tangent", panel="Model")
                    point(side, 0)
                    point(-12, end)
                    self.assertIn("solved", workspace.sketch_session["state"]["status"])
                self.assertEqual(9, workspace.sketch_session["state"]["sketch"].curve_count)
                driver.click("Finish sketch", panel="Model")
                driver.click("Extrude", panel="Properties")
                enter("height##Sketch.extrude", 4)
                driver.click("Create feature", panel="Properties")
                self.assertIsNone(workspace.pending, model.error or workspace.status)
                self.assertTrue(model.displayed.is_watertight())
                old_volume = model.displayed.volume()
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
                self.assertFalse(model.dirty)
                # Open the consumed sketch as a CAD user does, via the feature tree.
                item = driver.control("Extrude1##extrude1", panel="Features")
                x, y, width, height = item.rect
                driver.click(x=x+8, y=y+height/2)
                driver.click("Sketch1##sketch1", panel="Features", count=2)
                driver.click("Filter sketch", panel="Features")
                driver.text("Radius", replace=True)
                label = next(c.label for c in workspace.ui.controls
                             if c.panel == "Features" and c.label.startswith("Radius "))
                driver.click(label, panel="Features", count=2)
                enter("##sk_dimension", 7)
                driver.click("Apply##sk_act_dimension_apply", panel="Properties")
                driver.click("Finish sketch", panel="Model")
                self.assertAlmostEqual(13*math.pi*4, old_volume-model.displayed.volume(), delta=1)
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
                loaded = ModelSession(model.path)
                self.assertTrue(loaded.rebuild(), loaded.error)
                self.assertAlmostEqual(model.displayed.volume(), loaded.displayed.volume(), delta=.01)
            finally:
                workspace.viewer.close()
