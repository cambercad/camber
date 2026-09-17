import json
import os
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock, patch

from camber.clipboard import copy_to_clipboard, create_window


class ClipboardTests(unittest.TestCase):
    def test_uses_live_native_window(self):
        window = Mock()
        self.assertTrue(copy_to_clipboard('part.edge("rundung")', window=window))
        window.set_clipboard_text.assert_called_once_with('part.edge("rundung")')

    def test_native_failure_is_reported(self):
        window = Mock()
        window.set_clipboard_text.side_effect = OSError("clipboard unavailable")
        with patch("sys.stderr") as stderr:
            self.assertFalse(copy_to_clipboard("text", window=window))
        self.assertIn("clipboard unavailable", stderr.write.call_args.args[0])

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires a desktop clipboard")
    def test_imgui_text_widgets_use_the_native_clipboard(self):
        from camber.host import Viewer
        viewer = Viewer(visible=True, size=(64, 64))
        try:
            with viewer._ui_context():
                viewer._imgui.set_clipboard_text("Face: Bögen")
                self.assertEqual("Face: Bögen", viewer.window.get_clipboard_text())
        finally:
            viewer.close()

    @unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires a desktop clipboard")
    def test_another_process_can_paste_unicode_and_multiline_text(self):
        import pyglet
        window = create_window(visible=False)
        child = None
        try:
            text = 'sketch.add_line("Bögen", (1, 2))\n# Größe: 19 mm'
            self.assertTrue(copy_to_clipboard(text, window=window))
            child = subprocess.Popen([sys.executable, "-c",
                "import json, pyglet; w=pyglet.window.Window(visible=False); "
                "print(json.dumps(w.get_clipboard_text())); w.close()"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic() + 10
            while child.poll() is None and time.monotonic() < deadline:
                pyglet.app.platform_event_loop.step(0)
                window.dispatch_events()
                pyglet.clock.tick()
                time.sleep(.01)
            if child.poll() is None:
                self.fail("Clipboard request was not served by the live viewer window")
            output, error = child.communicate()
            self.assertEqual(0, child.returncode, error)
            self.assertEqual(text, json.loads(output))
        finally:
            if child is not None and child.poll() is None:
                child.kill()
                child.communicate()
            window.close()


if __name__ == "__main__":
    unittest.main()
