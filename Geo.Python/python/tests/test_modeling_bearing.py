"""Build and reopen a bearing using only actual GUI input events."""
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from camber import set_progress_log
from camber.modeling import ModelSession, _Workspace
from camber.gui_testing import GuiDriver


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class BearingGuiWorkflowTests(unittest.TestCase):
    def test_build_races_ball_and_mated_circular_pattern_from_empty_documents(self):
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            root = Path(folder)
            session = ModelSession(root / "inner_race.py")
            assert session.rebuild(), session.error
            w = _Workspace(session)
            w.viewer.window.set_visible(False)
            driver = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)

            def enter(label, value):
                driver.click(label, panel="Properties")
                driver.text(str(value), replace=True)

            def point(x, y):
                ndc = w.viewer.camera.project((x, y, 0))
                driver.click(x=(ndc[0]+1)*w.viewer.window.width/2,
                             y=(1-ndc[1])*w.viewer.window.height/2)

            def choose(label, index):
                driver.click(label, panel="Properties")
                driver.key("HOME")
                for _ in range(index):
                    driver.key("DOWN")
                driver.key("RETURN")

            def save():
                driver.key("S", modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
                assert not w.model.dirty, w.status

            def file_action(label, path):
                # Replay a user's file-picker selection at the native dialog boundary.
                class Dialog:
                    def __init__(self, **options):
                        self.handlers = {}
                    def push_handlers(self, **handlers):
                        self.handlers = handlers
                    def open(self):
                        for event, callback in self.handlers.items():
                            callback(str(path) if event == "on_dialog_save" else [str(path)])
                with patch("pyglet.window.dialog.FileSaveDialog", Dialog), patch("pyglet.window.dialog.FileOpenDialog", Dialog):
                    if label == "Insert":
                        driver.click("Assembly##tab", panel="Model")
                    else:
                        driver.click("File##tab", panel="Model")
                    driver.click(label, panel="Model")
                    driver.frame()

            def race(low, high):
                driver.click("Create sketch", panel="Properties")
                driver.click("Create sketch", panel="Properties")
                driver.click("Rectangle", panel="Model")
                point(-7, low)
                point(7, high)
                assert w.sketch_session["state"]["sketch"].curve_count == 4
                driver.click("Finish sketch", panel="Model")
                driver.click("Revolve", panel="Properties")
                driver.click("Create feature", panel="Properties")
                assert w.pending is None, w.model.error or w.form_build_error or w.status
                driver.click("Sketch", panel="Model")
                driver.click("Create sketch", panel="Properties")
                driver.click("Circle", panel="Model")
                point(0, 16.75)
                point(3.8, 16.75)
                driver.click("Finish sketch", panel="Model")
                driver.click("Revolve", panel="Properties")
                driver.click("Create feature", panel="Properties")
                driver.click("Subtract", panel="Model")
                choose("a##Part.subtract", 0)
                choose("b##Part.subtract", 1)
                driver.click("Create feature", panel="Properties")
                assert w.pending is None, w.model.error or w.form_build_error or w.status
                save()
            try:
                race(10, 14.5)
                driver.screenshot(root / "inner.png")
                file_action("New part", root / "outer_race.py")
                race(19, 23.5)
                driver.screenshot(root / "outer.png")
                file_action("New part", root / "ball.py")
                driver.click("Create##tab", panel="Model")
                driver.click("Sphere", panel="Model")
                for axis in "XYZ":
                    enter("center##Part.sphere##"+axis, 0)
                enter("radius##Part.sphere", 3.75)
                driver.click("Create feature", panel="Properties")
                save()
                file_action("New assembly", root / "bearing.py")
                file_action("Insert", root / "inner_race.py")
                file_action("Insert", root / "outer_race.py")
                driver.click("Concentric", panel="Model")
                choose("a##Assembly.concentric/component", 0)
                choose("b##Assembly.concentric/component", 1)
                choose("a##Assembly.concentric/entity", 0)
                choose("b##Assembly.concentric/entity", 3)
                driver.click("Create mate", panel="Properties")
                assert w.pending is None, w.model.error or w.form_build_error or w.status
                driver.click("Coincident", panel="Model")
                choose("a##Assembly.coincident/component", 0)
                choose("a##Assembly.coincident/entity", 0)
                choose("b##Assembly.coincident/component", 1)
                choose("b##Assembly.coincident/entity", 0)
                choose("opposite_normals##Assembly.coincident", 1)
                driver.click("Create mate", panel="Properties")
                assert w.pending is None, w.model.error or w.form_build_error or w.status
                save()
                file_action("Insert", root / "ball.py")
                for axis, value in zip("XYZ", (0, 16.75, 0)):
                    enter("position##Assembly.add_part##"+axis, value)
                driver.click("Apply", panel="Properties")
                driver.click("Fix", panel="Model")
                choose("part##Assembly.fix", 2)
                driver.click("Create mate", panel="Properties")
                driver.click("Circular", panel="Model")
                choose("seed##Assembly.pattern_circular", 2)
                enter("count##Assembly.pattern_circular", 9)
                driver.click("More options##Assembly.pattern_circular", panel="Properties")
                choose("axis##Assembly.pattern_circular/mode", 0)
                choose("axis##Assembly.pattern_circular/orientation", 5)
                driver.click("Create feature", panel="Properties")
                assert w.pending is None, w.model.error or w.form_build_error or w.status
                assert len(w.model.displayed.parts) == 11
                save()
                file_action("Open", root / "bearing.py")
                assert len(w.model.displayed.parts) == 11
                self.assertEqual("assembly", w.model.selected)
                driver.click("Assembly##tab", panel="Model")
                driver.move(1000, 200)
                driver.screenshot(root / "bearing_light.png")
                driver.click("Dark", panel="Document")
                driver.move(1000, 200)
                driver.screenshot(root / "bearing_dark.png")
                parts = w.model.displayed.parts
                for part in parts[:2]:
                    for coordinate in part.pose:
                        self.assertAlmostEqual(0, coordinate, places=5)
                for part in parts[2:]:
                    x, y, z = part.pose
                    self.assertAlmostEqual(0, x, places=5)
                    self.assertAlmostEqual(16.75, math.hypot(y, z), places=5)
                self.assertEqual(9, len({tuple(round(v, 4) for v in part.pose) for part in parts[2:]}))
                self.assertTrue((root/"bearing_light.png").is_file())
                self.assertTrue((root/"bearing_dark.png").is_file())
                driver.click("Ball instance · 2##pattern_circular1[1]", panel="Features")
                driver.click("Open component model", panel="Properties")
                self.assertEqual(root/"ball.py", w.model.path)
                driver.click("Back to assembly", panel="Document")
                self.assertEqual(root/"bearing.py", w.model.path)
                self.assertEqual(11, len(w.model.displayed.parts))
            finally:
                w.viewer.close()
