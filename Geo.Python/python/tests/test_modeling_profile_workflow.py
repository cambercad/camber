"""Resume an analytical engineering profile through its consumed feature tree."""
import ast
import os
from pathlib import Path
import tempfile
import unittest

from camber import set_progress_log
from camber.gui_testing import GuiDriver
from camber.modeling import ModelSession, _Workspace
from camber.sketch_curves import CURVES


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class AnalyticalProfileWorkflowTests(unittest.TestCase):
    def test_eleven_airfoil_sections_offer_named_loft_starts_without_python_input(self):
        """Curved contour matching is a form selection, including after reopening."""
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            model = ModelSession(Path(folder) / "airfoil_loft.py")
            self.assertTrue(model.rebuild(), model.error)
            workspace = _Workspace(model)
            workspace.viewer.window.set_visible(False)
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame)

            def reveal(label):
                for scroll in (None, 60, *([-3]*45)):
                    if scroll is not None:
                        driver.scroll(driver.window.width-65, 550, scroll)
                    try:
                        driver.control(label, panel="Properties")
                        return
                    except LookupError:
                        pass
                raise LookupError(label)

            def enter(label, value):
                reveal(label)
                driver.click(label, panel="Properties")
                driver.text(str(value), replace=True)

            def choose(label, index):
                reveal(label)
                driver.click(label, panel="Properties")
                driver.key("HOME")
                for _ in range(index):
                    driver.key("DOWN")
                driver.key("RETURN")

            def operation(key):
                from camber.modeling_commands import operations
                query = operations()[key].label
                driver.click("Operations##tab", panel="Model")
                driver.click("Find operation", panel="Model")
                driver.text(query, replace=True)
                entries = sorted((op.owner.__name__+" · "+op.group+" · "+op.label, name)
                                 for name, op in operations().items()
                                 if query.casefold() in (name+" "+op.label+" "+op.group).casefold())
                index = next(i for i, (_, name) in enumerate(entries) if name == key)
                driver.click("Operation", panel="Model")
                driver.key("HOME")
                for _ in range(index):
                    driver.key("DOWN")
                driver.key("RETURN")
                driver.click("Set up", panel="Model")
                self.assertEqual(key, workspace.pending.key)

            try:
                for index in range(11):
                    operation("Part.sketch")
                    try:
                        reveal("frame##Part.sketch/mode")
                    except LookupError:
                        reveal("More options##Part.sketch")
                        driver.click("More options##Part.sketch", panel="Properties")
                    choose("frame##Part.sketch/mode", 0)
                    enter("frame##Part.sketch/origin##Z", index*6)
                    enter("Feature name", "section_"+str(index+1))
                    reveal("Create sketch")
                    driver.click("Create sketch", panel="Properties")
                    driver.click("Profile", panel="Model")
                    choose("Curve type##profile", list(CURVES).index("Airfoil"))
                    driver.click("Apply profile", panel="Properties")
                    driver.click("Finish sketch", panel="Model")
                operation("Part.loft")
                for index in range(11):
                    label = "section_"+str(index+1)+"##sketches##Part.loft"
                    reveal(label)
                    driver.click(label, panel="Properties")
                reveal("More options##Part.loft")
                driver.click("More options##Part.loft", panel="Properties")
                reveal("Customize loft##options##Part.loft")
                driver.click("Customize loft##options##Part.loft", panel="Properties")
                choose("options##Part.loft/correspondence", 4)
                reveal("Starting curve##first_curves##Part.loft/10")
                self.assertEqual(["Hermite1"]*11, ast.literal_eval(workspace.fields["first_curves"]))
                reveal("Create feature")
                driver.click("Create feature", panel="Properties")
                self.assertIsNone(workspace.pending, model.error or workspace.form_build_error)
                self.assertTrue(model.displayed.is_watertight())
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
                reopened = ModelSession(model.path)
                self.assertTrue(reopened.rebuild(), reopened.error)
                self.assertAlmostEqual(model.displayed.volume(), reopened.displayed.volume(), delta=.01)
            finally:
                workspace.viewer.close()

    def test_airfoil_resumes_parameter_edit_without_losing_multi_curve_definition(self):
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            model = ModelSession(Path(folder) / "turbine_vane.py")
            self.assertTrue(model.rebuild(), model.error)
            workspace = _Workspace(model)
            workspace.viewer.window.set_visible(False)
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame)

            def enter(label, value):
                driver.click(label, panel="Properties")
                driver.text(str(value), replace=True)

            try:
                driver.click("Create sketch", panel="Properties")
                driver.click("Create sketch", panel="Properties")
                driver.click("Profile", panel="Model")
                driver.click("Curve type##profile", panel="Properties")
                driver.key("HOME")
                for _ in range(list(CURVES).index("Airfoil")):
                    driver.key("DOWN")
                driver.key("RETURN")
                driver.click("Apply profile", panel="Properties")
                self.assertEqual(3, workspace.sketch_session["state"]["sketch"].curve_count)
                driver.click("Finish sketch", panel="Model")
                driver.click("Extrude", panel="Properties")
                enter("height##Sketch.extrude", 5)
                driver.click("Create feature", panel="Properties")
                self.assertIsNone(workspace.pending, model.error or workspace.form_build_error)
                original = model.displayed.volume()
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
            finally:
                workspace.viewer.close()
            loaded = ModelSession(model.path)
            self.assertTrue(loaded.rebuild(), loaded.error)
            block = loaded.document.block("sketch1")
            self.assertTrue(block.editable, block.reason)
            workspace = _Workspace(loaded)
            workspace.viewer.window.set_visible(False)
            driver = GuiDriver(workspace.ui, workspace.viewer._draw_frame)
            try:
                item = driver.control("Extrude1##extrude1", panel="Features")
                x, y, width, height = item.rect
                driver.click(x=x+8, y=y+height/2)
                driver.click("Sketch1##sketch1", panel="Features", count=2)
                self.assertIsNotNone(workspace.sketch_session, workspace.status)
                label = next(c.label for c in workspace.ui.controls if c.panel == "Features" and "##sketch_action_0" in c.label)
                driver.click(label, panel="Features", count=2)
                driver.click("Pick leading edge 1##profile_pick_leading_edge_0", panel="Properties")
                ndc = workspace.viewer.camera.project((2, 3, 0))
                driver.click(x=(ndc[0]+1)*driver.window.width/2,
                             y=(1-ndc[1])*driver.window.height/2)
                self.assertIsNone(workspace.sketch_session["state"]["profile_edit"]["pick"])
                enter("Chord length##profile_chord_length", 80)
                driver.click("Apply profile", panel="Properties")
                driver.click("Finish sketch", panel="Model")
                self.assertIsNone(workspace.sketch_session, workspace.status)
                self.assertTrue(loaded.displayed.is_watertight())
                self.assertLess(loaded.displayed.volume(), original)
                driver.key("S", modifiers=workspace.viewer._pyglet.window.key.MOD_CTRL)
                final = ModelSession(model.path)
                self.assertTrue(final.rebuild(), final.error)
                self.assertAlmostEqual(final.displayed.volume(), loaded.displayed.volume(), delta=.01)
            finally:
                workspace.viewer.close()


if __name__ == "__main__":
    unittest.main()
