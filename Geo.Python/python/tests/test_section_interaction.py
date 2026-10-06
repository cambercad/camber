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
                                                                 "Minimum distance")]),
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
