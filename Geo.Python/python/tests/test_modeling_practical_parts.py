"""Real part workflow regression through window input events only."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from camber import set_progress_log
from camber.gui_testing import GuiDriver
from camber.modeling import ModelSession, _Workspace


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class PracticalPartGuiWorkflowTests(unittest.TestCase):
    def test_thin_nested_profile_keeps_clearance_with_coarse_grid(self):
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            session = ModelSession(Path(folder) / "seal_frame.py")
            self.assertTrue(session.rebuild(), session.error)
            w = _Workspace(session)
            w.viewer.window.set_visible(False)
            d = GuiDriver(w.ui, w.viewer._draw_frame)
            try:
                d.click("Create sketch", panel="Properties")
                d.click("Create sketch", panel="Properties")
                d.click("Rectangle", panel="Model")
                for point in ((-25, -18, 0), (25, 18, 0), (-23, -16, 0), (23, 16, 0)):
                    ndc = w.viewer.camera.project(point)
                    d.click(x=(ndc[0]+1)*d.window.width/2,
                            y=(1-ndc[1])*d.window.height/2)
                d.click("Finish sketch", panel="Model")
                d.click("Extrude", panel="Properties")
                d.click("height##Sketch.extrude", panel="Properties")
                d.text("5", replace=True)
                d.click("Create feature", panel="Properties")
                self.assertIsNone(w.pending, w.model.error or w.status)
                self.assertTrue(w.model.displayed.is_watertight())
                self.assertAlmostEqual((50*36-46*32)*5, w.model.displayed.volume(), delta=.1)
            finally:
                w.viewer.close()

    def test_perpendicular_profiles_picked_hole_edit_undo_and_reopen(self):
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            path = Path(folder)/"bracket.py"
            session = ModelSession(path)
            self.assertTrue(session.rebuild(), session.error)
            w = _Workspace(session)
            w.viewer.window.set_visible(False)
            d = GuiDriver(w.ui, w.viewer._draw_frame, capture=w.viewer.capture_image)

            def enter(label, value):
                d.click(label, panel="Properties")
                d.text(str(value), replace=True)

            def choose(label, index):
                d.click(label, panel="Properties")
                d.key("HOME")
                for _ in range(index):
                    d.key("DOWN")
                d.key("RETURN")

            def point(x, y, z=0):
                ndc = w.viewer.camera.project((x,y,z))
                d.click(x=(ndc[0]+1)*d.window.width/2,
                        y=(1-ndc[1])*d.window.height/2)

            def extrude(height, name):
                d.click("Finish sketch", panel="Model")
                d.click("Extrude", panel="Properties")
                enter("height##Sketch.extrude",height)
                enter("Feature name",name)
                d.click("Create feature", panel="Properties")
                self.assertIsNone(w.pending, w.model.error or w.status)

            try:
                d.click("Create sketch",panel="Properties")
                d.click("Create sketch",panel="Properties")
                d.click("Rectangle",panel="Model")
                point(-20,-16)
                point(20,16)
                extrude(5,"base_plate")
                d.click("Sketch",panel="Model")
                choose("plane##Part.sketch",1)
                d.click("Create sketch",panel="Properties")
                d.click("Rectangle",panel="Model")
                point(-20,0,0)
                point(20,0,28)
                extrude(5,"upright")
                d.click("Union",panel="Model")
                choose("a##Part.union",0)
                choose("b##Part.union",1)
                enter("Feature name","bracket")
                d.click("Create feature",panel="Properties")
                blank_volume = w.model.displayed.volume()
                d.click("Hole",panel="Model")
                d.click("Pick planar face##mouth##Solid.hole",panel="Properties")
                self.assertIsNotNone(w._pick_reference, w.status)
                point(0,-5,18)
                self.assertIn(".to_global(", w.fields["mouth"], (w.status,w.picked))
                offset = float(w.frame_fields["mouth##Solid.hole"].rows["origin"][0])
                d.scroll(d.window.width-70, 550, 12)
                enter("mouth##Solid.hole/origin##X", offset+1)
                self.assertIn(".face_frame(", w.fields["mouth"])
                enter("diameter##Solid.hole",0)
                self.assertFalse(d.control("Create feature",panel="Properties").enabled)
                enter("diameter##Solid.hole",8)
                enter("Feature name","upright_hole")
                d.click("Create feature",panel="Properties")
                self.assertIsNone(w.pending, w.model.error or w.status)
                import math
                self.assertAlmostEqual(math.pi*4**2*5,
                                       blank_volume-w.model.displayed.volume(), delta=2, msg=w.model.source)
                self.assertIn(".to_global(Frame(origin=",w.model.source)
                d.click("Upright hole##upright_hole",panel="Features",count=2)
                enter("diameter##Solid.hole",6)
                d.click("Apply",panel="Properties")
                edited = w.model.source
                self.assertIn("diameter=6",edited)
                d.key("Z",modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
                self.assertIn("diameter=8",w.model.source)
                d.key("Y",modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
                self.assertEqual(edited,w.model.source)
                d.key("S",modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
                self.assertFalse(w.model.dirty)
                class Dialog:
                    def __init__(self, **kwargs): pass
                    def push_handlers(self, **handlers): self.handlers=handlers
                    def open(self): self.handlers["on_dialog_open"]([str(path)])
                with patch("pyglet.window.dialog.FileOpenDialog",Dialog):
                    d.click("File##tab",panel="Model")
                    d.click("Open",panel="Model")
                    d.frame()
                self.assertEqual(edited,w.model.source)
                for theme in ("Dark","Light"):
                    d.click(theme,panel="Document")
                    image=d.screenshot()
                    self.assertEqual((1920,1080),(image.width,image.height))
            finally:
                w.viewer.close()





    def test_dimension_validation_and_shell_face_picking_from_empty_document(self):
        import ast
        set_progress_log(False)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            model = ModelSession(Path(folder)/"enclosure.py")
            self.assertTrue(model.rebuild(), model.error)
            w = _Workspace(model)
            w.viewer.window.set_visible(False)
            d = GuiDriver(w.ui,w.viewer._draw_frame,capture=w.viewer.capture_image)
            def point(x,y,z=0):
                ndc=w.viewer.camera.project((x,y,z))
                d.click(x=(ndc[0]+1)*d.window.width/2,y=(1-ndc[1])*d.window.height/2)
            def enter(label,value):
                d.click(label,panel="Properties")
                d.text(str(value),replace=True)
            try:
                d.click("Create sketch",panel="Properties")
                d.click("Create sketch",panel="Properties")
                d.click("Rectangle",panel="Model")
                point(-25,-18)
                point(25,18)
                d.click("Length",panel="Model")
                point(0,-18)
                for invalid in ("0", "-1", "invalid", "nan"):
                    enter("##sk_dimension",invalid)
                    self.assertFalse(d.control("Apply##sk_act_dimension_apply",panel="Properties").enabled)
                # Clicking the viewport and pressing Enter must not commit an old value.
                d.click(x=1200,y=800)
                d.key("RETURN")
                self.assertIsNotNone(w.sketch_session["state"]["dimension"])
                self.assertFalse(w.sketch_session["state"]["dimension"]["valid"])
                enter("##sk_dimension",50)
                d.click("Apply##sk_act_dimension_apply",panel="Properties")
                d.click("Length",panel="Model")
                point(25,0)
                enter("##sk_dimension",36)
                d.click("Apply##sk_act_dimension_apply",panel="Properties")
                d.click("Finish sketch",panel="Model")
                d.click("Extrude",panel="Properties")
                enter("height##Sketch.extrude",20)
                enter("Feature name","enclosure_blank")
                d.click("Create feature",panel="Properties")
                d.click("Shell",panel="Model")
                enter("thickness##Solid.shell",0)
                self.assertFalse(d.control("Create feature",panel="Properties").enabled)
                enter("thickness##Solid.shell",2)
                d.click("Pick faces##faces##Solid.shell",panel="Properties")
                point(0,0,20)
                face=w.picked[-1]
                self.assertEqual((face,),ast.literal_eval(w.fields["faces"]),w.status)
                # Repeated clicks do not duplicate the reference. Unchecking removes it.
                point(0,0,20)
                self.assertEqual((face,),ast.literal_eval(w.fields["faces"]))
                d.click(face+"##faces##Solid.shell",panel="Properties")
                self.assertIsNone(ast.literal_eval(w.fields["faces"]))
                point(0,0,20)
                d.click("Create feature",panel="Properties")
                self.assertIsNone(w.pending,w.model.error or w.status)
                self.assertTrue(w.model.displayed.is_watertight())
                self.assertAlmostEqual(50*36*20-46*32*18,w.model.displayed.volume(),delta=2)
                d.key("S",modifiers=w.viewer._pyglet.window.key.MOD_CTRL)
                self.assertFalse(w.model.dirty)
                lengths = [tuple(ast.literal_eval(arg) for arg in node.args)
                           for node in ast.walk(ast.parse(w.model.source))
                           if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                           and node.func.attr == "length"]
                self.assertIn(("Line1", 50), lengths)
                self.assertIn(("Line2", 36), lengths)
                self.assertIn(repr(face),w.model.source)
            finally:
                w.viewer.close()
