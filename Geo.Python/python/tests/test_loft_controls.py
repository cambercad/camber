"""Public loft controls and a complete GUI-authored rounded-tip workflow."""
import math
import os
from pathlib import Path
import tempfile
import unittest

from camber import Curve3D, Frame, Part, set_progress_log


class LoftInputTests(unittest.TestCase):
    def test_guides_tangents_and_sharp_closure_reach_the_kernel(self):
        part = Part((-30, -30, -30), (30, 30, 40), tolerance=.05)
        sections = [part.sketch(frame=Frame(origin=(0, 0, z))) for z in (0, 20)]
        for section in sections:
            section.add_circle((0, 0), 10, name='rim')
        guided = part.loft(sections, guides=[Curve3D.line((10, 0, 0), (10, 0, 20))])
        tangent = part.loft(sections, start_tangent=(0, 0, 20), end_tangent=(0, 0, 20))
        pointed = part.loft(sections, end_point=(0, 0, 25), end_closure='sharp')
        for solid in (guided, tangent, pointed):
            self.assertTrue(solid.is_watertight())
            self.assertGreater(solid.volume(), 0)
        self.assertGreater(pointed.volume(), tangent.volume())

    def test_invalid_tip_style_and_vectors_fail_before_geometry_construction(self):
        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.05)
        for arguments in ({"start_closure": "rounded"}, {"end_closure": "unknown"},
                          {"start_tangent": (0, 0, 0)}, {"end_tangent": (0, math.inf, 1)},
                          {"end_point": (0, 0, math.nan)}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                part.loft([], **arguments)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class LoftGuiWorkflowTests(unittest.TestCase):
    def test_rounded_tips_use_numeric_controls_and_survive_save_reopen(self):
        from camber.gui_testing import GuiDriver
        from camber.modeling import ModelSession, _Workspace
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            model = ModelSession(Path(folder) / "rounded_body.py")
            self.assertTrue(model.rebuild(), model.error)
            workspace = _Workspace(model)
            workspace.viewer.window.set_visible(False)
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame)

            def reveal(label):
                for amount in (None, 15, *([-3]*15)):
                    if amount is not None:
                        driver.scroll(driver.window.width-70, 550, amount)
                    try:
                        return driver.control(label, panel="Properties")
                    except LookupError:
                        pass
                self.fail("Missing control " + label)

            def enter(label, value):
                reveal(label)
                driver.click(label, panel="Properties")
                driver.text(str(value), replace=True)

            def point(x, y, z):
                ndc = workspace.viewer.camera.project((x, y, z))
                driver.click(x=(ndc[0]+1)*driver.window.width/2,
                             y=(1-ndc[1])*driver.window.height/2)

            try:
                driver.click("Create sketch", panel="Properties")
                driver.click("Create sketch", panel="Properties")
                driver.click("Circle", panel="Model")
                point(0, 0, 0)
                point(10, 0, 0)
                driver.click("Finish sketch", panel="Model")
                driver.click("Solid##tab", panel="Model")
                driver.click("Sketch", panel="Model")
                if not any(c.label == "frame##Part.sketch/mode" for c in workspace.ui.controls):
                    driver.click("More options##Part.sketch", panel="Properties")
                reveal("frame##Part.sketch/mode")
                driver.click("frame##Part.sketch/mode", panel="Properties")
                driver.key("HOME")
                driver.key("RETURN")
                enter("frame##Part.sketch/origin##Z", 20)
                driver.click("Create sketch", panel="Properties")
                driver.click("Circle", panel="Model")
                point(0, 0, 20)
                point(15, 0, 20)
                driver.click("Finish sketch", panel="Model")
                driver.click("Solid##tab", panel="Model")
                driver.click("Loft", panel="Model")
                for index in (1, 2):
                    label = f"sketch{index}##sketches##Part.loft"
                    reveal(label)
                    driver.click(label, panel="Properties")
                reveal("End conditions##Part.loft")
                driver.click("End conditions##Part.loft", panel="Properties")
                for end, z in (("start", -5), ("end", 25)):
                    label = f"Close {end} to a point##{end}_point##Part.loft/enabled"
                    reveal(label)
                    driver.click(label, panel="Properties")
                    enter(f"{end}_point##Part.loft##Z", z)
                label = "Specify start tangent · direction and strength##start_tangent##Part.loft/enabled"
                reveal(label)
                driver.click(label, panel="Properties")
                reveal("Create feature")
                self.assertFalse(driver.control("Create feature", panel="Properties").enabled)
                reveal(label)
                driver.click(label, panel="Properties")
                reveal("Create feature")
                driver.click("Create feature", panel="Properties")
                self.assertIsNone(workspace.pending, model.error or workspace.form_build_error)
                self.assertTrue(model.displayed.is_watertight())
                self.assertGreater(model.displayed.volume(), 0)
                source, volume = model.source, model.displayed.volume()
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
                self.assertFalse(model.dirty)
            finally:
                workspace.viewer.close()
            reopened = ModelSession(model.path)
            self.assertTrue(reopened.rebuild(), reopened.error)
            self.assertEqual(source, reopened.source)
            self.assertTrue(reopened.displayed.is_watertight())
            self.assertAlmostEqual(volume, reopened.displayed.volume(), delta=.001)


if __name__ == "__main__":
    unittest.main()
