"""Real ImGui contexts and rendering; EGL matches pyglet's headless context."""
import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires GL integration tests")
class ViewerImguiLifecycleTests(unittest.TestCase):
    def setUp(self):
        from camber.imgui_compat import import_imgui
        self.imgui = import_imgui()
        self.previous = self.imgui.get_current_context()
        self.external = self.imgui.create_context()
        self.imgui.set_current_context(self.external)
        self.viewers = []

    def tearDown(self):
        for viewer in reversed(self.viewers):
            viewer.close()
        self.imgui.destroy_context(self.external)
        self.imgui.set_current_context(self.previous)

    def viewer(self):
        from camber.host import Viewer
        viewer = Viewer(visible=True, size=(64, 64))
        self.viewers.append(viewer)
        return viewer

    def test_two_viewers_draw_and_close_without_using_each_others_context(self):
        first, second = self.viewer(), self.viewer()
        self.assertEqual(self.external, self.imgui.get_current_context())
        self.assertNotEqual(first._imgui_context, second._imgui_context)
        first._letter(first._pyglet.window.key.A, True)
        self.assertEqual({"A"}, first._imgui_letters)
        self.assertEqual(set(), second._imgui_letters)
        for viewer in (first, second, first):
            def tick(viewer=viewer):
                self.assertEqual(viewer._imgui_context, self.imgui.get_current_context())
                self.assertIs(viewer._imgui_letters, self.imgui._camber_letters)
                self.imgui.text("Owned context")
            viewer._tick = tick
            viewer.window.switch_to()
            viewer.on_draw()
            self.assertEqual(self.external, self.imgui.get_current_context())
        # Backend event handlers retain their own IO even when another context is current.
        second._impl.on_mouse_motion(17, 19, 0, 0)
        self.assertEqual((17, 45), tuple(second._impl.io.mouse_pos))
        self.assertNotEqual(tuple(first._impl.io.mouse_pos), tuple(second._impl.io.mouse_pos))
        self.imgui.set_current_context(second._imgui_context)
        owned = first._imgui_context
        with patch.object(self.imgui, "destroy_context", wraps=self.imgui.destroy_context) as destroy:
            first.close()
            first.close()
            destroy.assert_called_once_with(owned)
        self.assertEqual(second._imgui_context, self.imgui.get_current_context())
        second.window.switch_to()
        second.on_draw()
        second.close()
        self.assertIsNone(self.imgui.get_current_context())

    def test_shader_failure_releases_owned_context_and_restores_external_context(self):
        from camber.host import Viewer
        contexts = []
        create = self.imgui.create_context
        def track_context():
            context = create()
            contexts.append(context)
            return context
        with patch.object(self.imgui, "create_context", side_effect=track_context), \
             patch.object(self.imgui, "destroy_context", wraps=self.imgui.destroy_context) as destroy, \
             patch.object(Viewer, "_compile", side_effect=RuntimeError("shader failed")):
            with self.assertRaisesRegex(RuntimeError, "shader failed"):
                self.viewer()
            destroy.assert_called_once_with(contexts[0])
        self.assertEqual(self.external, self.imgui.get_current_context())
        self.viewer()  # A later UI renderer can still initialize.

    def test_close_from_ui_callback_stops_frame_and_ignores_queued_draw(self):
        viewer = self.viewer()
        viewer._tick = viewer.unshow
        with patch.object(viewer, "_draw_scene", wraps=viewer._draw_scene) as draw_scene:
            viewer.on_draw()
            self.assertTrue(viewer._closed)
            self.assertIsNone(viewer._imgui_context)
            self.assertIsNone(viewer.window.context)
            self.assertEqual(self.external, self.imgui.get_current_context())
            viewer.on_draw()  # A queued event must not touch the closed GL window.
            draw_scene.assert_not_called()
            self.assertEqual(self.external, self.imgui.get_current_context())

    def test_draw_exception_restores_external_context(self):
        viewer = self.viewer()
        viewer._tick = lambda: (_ for _ in ()).throw(RuntimeError("UI failed"))
        with self.assertRaisesRegex(RuntimeError, "UI failed"):
            viewer.on_draw()
        self.assertEqual(self.external, self.imgui.get_current_context())
