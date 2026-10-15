"""Real pyglet events, GUI frames, kernel picks and GL annotations.

Only the blocking app.run loop is replaced by a deterministic event driver.
No measurement, ray-cast, release-detection or drawing handler is mocked.
"""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class AnnotationLayoutTests(unittest.TestCase):
    def test_clustered_labels_fit_without_overlap(self):
        from camber.inspection import _place_label
        for desired in ((400, 300), (790, 590), (2, 2)):
            boxes = []
            for _ in range(6):
                _place_label(desired, (180, 22), boxes, (800, 600))
            for i, (x, y, w, h) in enumerate(boxes):
                self.assertGreaterEqual(x, 0)
                self.assertGreaterEqual(y, 0)
                self.assertLessEqual(x+w, 800)
                self.assertLessEqual(y+h, 600)
                for bx, by, bw, bh in boxes[:i]:
                    self.assertTrue(x+w <= bx or bx+bw <= x or y+h <= by or by+bh <= y)

    def test_measurement_label_is_vertically_centered_in_its_background(self):
        from types import SimpleNamespace
        from camber.inspection import Measurement, _draw_annotations

        class Drawable:
            def draw(self):
                pass

            def delete(self):
                pass

        drawn_lines = []

        class Shapes:
            Circle = Rectangle = lambda *args, **kwargs: Drawable()

            @staticmethod
            def Line(*args, **kwargs):
                drawn_lines.append(args)
                return Drawable()

        labels = []

        class Label(Drawable):
            def __init__(self, text, **kwargs):
                self.content_width, self.content_height = 160, 12
                self.anchor_y = kwargs.get("anchor_y", "baseline")
                self.x, self.y = kwargs["x"], kwargs["y"]
                labels.append(self)

        viewer = SimpleNamespace(
            inspection=SimpleNamespace(measurements=[Measurement((-.5, 0, 0), (.5, 0, 0), 1,
                                                                 label="Minimum distance")]),
            camera=SimpleNamespace(eye=(0, 0, -5), ortho=False, fov=1.0, aspect=1.0,
                                   frame=lambda: ((0, 0, 1), (0, 1, 0), (1, 0, 0))),
            window=SimpleNamespace(width=640, height=480),
            _ui=SimpleNamespace(viewport_rect=(0, 0, 640, 480)),
            _pyglet=SimpleNamespace(shapes=Shapes, text=SimpleNamespace(Label=Label)),
            _np=__import__("numpy"),
            _gl=SimpleNamespace(GL_DEPTH_TEST=1, glDisable=lambda _flag: None))
        _draw_annotations(viewer)
        self.assertEqual(1, len(labels))
        self.assertEqual("center", labels[0].anchor_y)
        drawn_lines.clear()
        viewer.camera.ortho = True
        viewer.camera.zoom = 1.0
        viewer.inspection.measurements = []
        _draw_annotations(viewer)
        self.assertTrue(drawn_lines)
        self.assertGreater(min(args[0] for args in drawn_lines), viewer.window.width/2)


class LiveSectionPlaneTests(unittest.TestCase):
    def test_rotation_preserves_position_relative_to_translated_model_center(self):
        from types import SimpleNamespace
        from camber.inspection import _ViewerInspection

        viewer = SimpleNamespace(section_plane=None, get_bounding_box=lambda: ((8, 18, 28), (12, 22, 32)))
        state = _ViewerInspection(viewer)
        state._orient_section((0, 0, 1))
        self.assertEqual(((0, 0, 1), 30), viewer.section_plane)
        state.set_section((0, 0, 2), 62, reverse=True)
        state._orient_section((3, 4, 0))
        self.assertEqual(23, state.offset)
        self.assertAlmostEqual(-23, viewer.section_plane[1])
        self.assertAlmostEqual(1, sum(v*v for v in viewer.section_plane[0]))
        state._orient_section((0, 1, 0), (0, 19, 0))
        self.assertEqual(((0, -1, 0), -19), viewer.section_plane)

    def test_invalid_plane_does_not_replace_the_last_valid_plane(self):
        from types import SimpleNamespace
        from camber.inspection import _ViewerInspection

        viewer = SimpleNamespace(section_plane=None)
        state = _ViewerInspection(viewer)
        state.set_section((1, 0, 0), 3)
        for normal, offset in (((0, 0, 0), 0), ((float("nan"), 0, 1), 0), ((0, 0, 1), float("inf"))):
            with self.assertRaises(ValueError):
                state.set_section(normal, offset)
            self.assertEqual(((1, 0, 0), 3), viewer.section_plane)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires GL integration tests")
class LiveSectionRenderingTests(unittest.TestCase):
    def scene(self, visible=False):
        from camber import Part, set_progress_log
        from camber.host import Viewer
        from camber.glview import _as_scene, pack_scene
        from camber.inspection import _ViewerInspection

        set_progress_log(False)
        part = Part((-5, -5, -5), (5, 5, 5), tolerance=.01)
        block = part.cuboid((-2, -2, -2), (2, 2, 2), name="section_block")
        hole = part.cuboid((-.7, -.7, -3), (.7, .7, 3), name="section_hole")
        body = part.subtract(block, hole, name="section_tube")
        viewer = Viewer(size=(800, 640) if visible else (320, 320), visible=visible)
        self.addCleanup(viewer.close)
        viewer.set_solid(pack_scene(_as_scene(body), colors={"*": (.4, .5, .6)}))
        viewer.checker = False
        viewer.camera.center, viewer.camera.eye, viewer.camera.up = (0, 0, 0), (0, 0, 10), (0, 1, 0)
        viewer.camera.ortho, viewer.camera.zoom = True, 3
        return viewer, _ViewerInspection(viewer), body

    def test_live_caps_preserve_holes_and_reuse_mesh_and_cap_buffers(self):
        import numpy as np

        viewer, state, _ = self.scene()
        mesh = viewer._gpu_parts()[0]["mesh"]
        original = viewer.capture_rgba()
        state.set_section((0, 0, 1), 0)
        cut = viewer.capture_rgba()

        def orange(image):
            return (image[:, :, 0] > 220) & (image[:, :, 1] > 125) & \
                (image[:, :, 1] < 200) & (image[:, :, 2] < 110)

        self.assertGreater(np.count_nonzero(orange(cut)), 1000)
        self.assertFalse(orange(cut)[160, 160], "the through-hole must not be capped")
        cap = viewer._section_cap
        for normal, offset in (((0, 0, 1), 1), ((1, 0, 1), .3), ((1, 1, 1), -.2)):
            state.set_section(normal, offset)
            viewer.capture_rgba()
            self.assertIs(mesh, viewer._gpu_parts()[0]["mesh"])
            self.assertIs(cap, viewer._section_cap)
        state.set_section((0, 0, 1), -3)
        empty = viewer.capture_rgba()
        self.assertFalse(np.any(orange(empty)))
        np.testing.assert_array_equal(empty[160, 120], empty[0, 0])
        state.set_section((0, 0, 1), -3, reverse=True)
        np.testing.assert_array_equal(viewer.capture_rgba(), original)
        viewer.section_plane = None
        np.testing.assert_array_equal(viewer.capture_rgba(), original)

    def test_section_tab_enable_and_drag_update_the_rendered_plane(self):
        from camber.view import _selected_plane_context

        viewer, state, body = self.scene(visible=True)
        ui, mouse = viewer._ui, viewer._pyglet.window.mouse
        face = next(name for name in body.patch_names if "section_block" in name and "ExtrudeTop" in name)
        selected = [face]
        viewer.set_user_callback(lambda: state.draw(ui, selected, lambda: _selected_plane_context(body, selected)))

        def frame():
            viewer.window.switch_to()
            viewer.on_draw()

        def click(x, y):
            ui.on_mouse_press(x, viewer.window.height-y, mouse.LEFT, 0)
            frame()
            ui.on_mouse_release(x, viewer.window.height-y, mouse.LEFT, 0)
            frame()

        def click_label(text, checkbox=False):
            label = next(args for kind, args, _ in ui._commands
                         if kind == "text_align" and args[2] == text)
            click(label[0][0]-(16 if checkbox else 0), label[0][1])

        frame()
        rect = next(rect for _, label, rect, _ in ui._tab_hits if label == "Cross section")
        click(rect[0]+10, rect[1]+12)
        label = next(args for kind, args, _ in ui._commands
                     if kind == "text_align" and args[2] == "Enable cross section")
        click(label[0][0]-16, label[0][1])
        self.assertEqual((0, 0, 1), viewer.section_plane[0])
        self.assertAlmostEqual(0, viewer.section_plane[1], delta=.0001)
        mesh, cap = viewer._gpu_parts()[0]["mesh"], viewer._section_cap

        for field, fraction in (("section_position", .75), ("section_elevation", .5), ("section_azimuth", .625)):
            editor = next(editor for identity, editor in ui._editors.items()
                          if identity[1] == "##"+field+"_value")
            x, y, _, h = editor.rect
            # The slider precedes the numeric field, with eight pixels spacing.
            x, y = x-198+190*fraction, y+h/2
            ui.on_mouse_press(x, viewer.window.height-y, mouse.LEFT, 0)
            frame()
            frame()
            ui.on_mouse_release(x, viewer.window.height-y, mouse.LEFT, 0)
            frame()
        self.assertAlmostEqual(1, state.offset, delta=.0001)
        self.assertAlmostEqual(2**-.5, state.normal[0])
        self.assertAlmostEqual(2**-.5, state.normal[1])
        self.assertAlmostEqual(0, state.normal[2])
        self.assertIs(mesh, viewer._gpu_parts()[0]["mesh"])
        self.assertIs(cap, viewer._section_cap)
        click_label("Use selected face")
        context = _selected_plane_context(body, selected)
        self.assertAlmostEqual(state.offset, sum(a*b for a, b in zip(state.normal, context["frame"].origin)))
        click_label("Align to view")
        self.assertEqual(viewer.camera.frame()[0], state.normal)
        click_label("Keep opposite side", checkbox=True)
        self.assertTrue(state.reverse)
        plane = viewer.section_plane
        click_label("Enable cross section", checkbox=True)
        self.assertIsNone(viewer.section_plane)
        click_label("Enable cross section", checkbox=True)
        self.assertEqual(plane, viewer.section_plane)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires GL integration tests")
class SectionInteractionTests(unittest.TestCase):
    def test_keyboard_mouse_measurement_cancel_and_reopen(self):
        from camber import Part, section, set_progress_log
        from camber.host import Viewer, run_solid
        from camber.view import _CAD
        import numpy as np

        set_progress_log(False)
        part = Part((-20, -20, -20), (20, 20, 20), tolerance=.01)
        body = part.cuboid((-2, -2, -2), (2, 2, 2), name="interaction_block")
        cut = section(body)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        folder = Path(os.environ.get("CAMBER_SECTION_EVENT_ARTIFACT_DIR", temporary.name))
        folder.mkdir(parents=True, exist_ok=True)
        viewers = []
        previous_press, previous_orbit = _CAD["press"], _CAD["orbiting"]
        self.addCleanup(lambda: _CAD.update(press=previous_press, orbiting=previous_orbit))
        _CAD.update(press=None, orbiting=False)

        def create(*args, **kwargs):
            viewer = Viewer(*args, **kwargs, visible=True, size=(800, 600))
            viewers.append(viewer)
            return viewer

        def save(viewer, pixels, name):
            image = viewer._pyglet.image.ImageData(pixels.shape[1], pixels.shape[0], "RGBA",
                                                  pixels.tobytes(), pitch=-pixels.shape[1]*4)
            path = folder/name
            with path.open("wb") as stream:
                image.save(str(path), file=stream)

        def drive(viewer):
            self.addCleanup(viewer.close)
            window = viewer.window
            key, mouse = viewer._pyglet.window.key, viewer._pyglet.window.mouse
            viewer.checker = False
            viewer.camera.center = (0, 0, 0)
            viewer.camera.eye = (0, 0, 10)
            viewer.camera.up = (0, 1, 0)
            viewer.camera.ortho = True
            viewer.camera.zoom = 3

            def frame():
                window.switch_to()
                window.dispatch_events()
                viewer.on_draw()

            def event(name, *args):
                window.dispatch_event(name, *args)
                frame()

            def press(symbol, hold=False):
                event("on_key_press", symbol, 0)
                if hold:
                    frame()  # held M must not toggle repeatedly
                event("on_key_release", symbol, 0)

            def click(x, y):
                event("on_mouse_motion", x, y, 0, 0)
                event("on_mouse_press", x, y, mouse.LEFT, 0)
                event("on_mouse_release", x, y, mouse.LEFT, 0)

            frame()
            if len(viewers) == 2:
                self.assertEqual(3, len(cut.measurements))
                save(viewer, viewer.capture_rgba("Reopened section"), "reopened.png")
                viewer.close()
                return
            before = viewer.capture_rgba("Section measurement")
            press(key.M, hold=True)
            click(20, 300)  # outside solid: miss must not establish an endpoint
            self.assertEqual(0, len(cut.measurements))
            click(300, 300)
            self.assertEqual(0, len(cut.measurements))
            click(500, 300)
            self.assertEqual(1, len(cut.measurements))
            first = cut.measurements[0]
            self.assertAlmostEqual(2, first.length, delta=.0002)
            self.assertAlmostEqual(0, first.start[2], delta=.0002)
            self.assertAlmostEqual(0, first.end[2], delta=.0002)
            after = viewer.capture_rgba("Section measurement")
            self.assertGreater(np.count_nonzero(np.any(before != after, axis=2)), 300)
            save(viewer, after, "two_picks.png")

            # Escape discards the pending first pick, not completed dimensions.
            press(key.M)
            click(300, 300)
            press(key.ESCAPE)
            self.assertFalse(viewer._closed)
            self.assertEqual((first,), cut.measurements)
            press(key.M)
            click(400, 250)
            self.assertEqual(1, len(cut.measurements))
            click(400, 350)
            self.assertEqual(2, len(cut.measurements))
            self.assertAlmostEqual(1, cut.measurements[1].length, delta=.0002)

            # M again also cancels a partial operation and starts fresh next time.
            press(key.M)
            click(300, 300)
            press(key.M)
            press(key.M)
            click(350, 250)
            self.assertEqual(2, len(cut.measurements))
            click(450, 350)
            self.assertEqual(3, len(cut.measurements))
            self.assertAlmostEqual(2**.5, cut.measurements[2].length, delta=.0002)
            snapshot = cut.measurements
            viewer.camera.orbit(.12, .06)
            frame()
            self.assertEqual(snapshot, cut.measurements)
            save(viewer, viewer.capture_rgba("Dimensions after orbit"), "orbited.png")
            viewer.close()
            self.assertEqual(snapshot, cut.measurements)

        with patch("camber.host.Viewer", side_effect=create), patch.object(Viewer, "show", drive):
            run_solid(cut, title="Section event-path test", colors={"*": (.65, .68, .72)})
            run_solid(cut, title="Section reopen test", colors={"*": (.65, .68, .72)})
        (folder/"measurements.json").write_text(json.dumps([
            {"start": m.start, "end": m.end, "length": m.length, "label": m.label}
            for m in cut.measurements], indent=2))
