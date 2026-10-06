"""Rendering regressions for the standalone pyglet controls, without CAD imports."""
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch


GUI_PATH = Path(__file__).resolve().parents[1] / "camber" / "gui.py"
spec = importlib.util.spec_from_file_location("standalone_gui", GUI_PATH)
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class ControlRenderingTests(unittest.TestCase):
    def setUp(self):
        import pyglet

        self.window = pyglet.window.Window(420, 240, visible=False)
        self.ui = gui.UI(self.window)

    def tearDown(self):
        self.ui.shutdown()
        self.window.close()

    def draw(self):
        self.window.switch_to()
        self.window.clear()
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
        self.ui.Begin("Controls", self.ui.ImGuiWindowFlags_NoDecoration)
        self.ui.InputText("Name", "Example", 160)
        self.ui.Checkbox("Enabled", True)
        self.ui.DataGrid(("Name", "Status"), (("Alice", True), ("Bob", False)), widths=(2, 1))
        self.ui.End()
        self.ui.render()

    def pixels(self):
        import pyglet

        image = pyglet.image.get_buffer_manager().get_color_buffer().get_image_data()
        return image.get_data("RGBA", self.window.width*4)

    def test_caret_blink_does_not_recreate_or_shift_other_controls(self):
        self.draw()
        eid, editor = next(iter(self.ui._editors.items()))
        self.ui._focus = eid
        editor.caret.position = len(editor.document.text)
        with patch.object(gui.time, "monotonic", return_value=0.1):
            self.draw()
            lit_pixels = self.pixels()
        before = list(self.ui._cache)
        with patch.object(gui.time, "monotonic", return_value=0.8):
            self.draw()
            dim_pixels = self.pixels()
        after = self.ui._cache
        self.assertEqual([entry[0][0] for entry in before], [entry[0][0] for entry in after])
        self.assertEqual(len(before), len(after))
        self.assertEqual([id(entry[1]) for entry in before], [id(entry[1]) for entry in after])
        changed = [(a[0][0], a[0][1], b[0][1]) for a, b in zip(before, after)
                   if a[0][1] != b[0][1]]
        self.assertEqual(1, len(changed))
        self.assertEqual("line", changed[0][0])
        self.assertEqual(0, changed[0][2][2][3])
        caret_x = round(changed[0][1][0][0])
        changed_x = {index // 4 % self.window.width
                     for index in range(0, len(lit_pixels), 4)
                     if lit_pixels[index:index+4] != dim_pixels[index:index+4]}
        self.assertTrue(changed_x)
        self.assertTrue(all(abs(x-caret_x) <= 2 for x in changed_x))

    def test_grid_hover_preserves_checkbox_primitives(self):
        self.draw()
        before_pixels = self.pixels()
        before = list(self.ui._cache)
        label = next(args for kind, args, _ in self.ui._commands
                     if kind == "text_align" and args[2] == "Alice")
        x, y = label[0]
        self.ui.on_mouse_motion(x, self.window.height-y, 1, 0)
        self.draw()
        after_pixels = self.pixels()
        after = self.ui._cache
        self.assertEqual([entry[0][0] for entry in before], [entry[0][0] for entry in after])
        self.assertEqual([id(entry[1]) for entry in before], [id(entry[1]) for entry in after])
        changed = [(a[0][0], a[0][1], b[0][1]) for a, b in zip(before, after)
                   if a[0][1] != b[0][1]]
        self.assertEqual(["rect"], [entry[0] for entry in changed])
        box = next(args for kind, args, _ in self.ui._commands
                   if kind == "rrect" and args[1][0]-args[0][0] == 14
                   and args[0][1] >= y-12)
        bx, by = box[0]
        for row in range(round(by)+3, round(by)+11):
            for column in range(round(bx)+3, round(bx)+11):
                offset = ((self.window.height-1-row)*self.window.width+column)*4
                self.assertEqual(before_pixels[offset:offset+4], after_pixels[offset:offset+4])

    def test_text_cursor_has_light_and_dark_pixels(self):
        self.draw()
        editor = next(iter(self.ui._editors.values()))
        x, y, _, h = editor.rect
        self.ui.on_mouse_motion(x+8, self.window.height-y-h//2, 1, 0)
        cursor = self.ui._cursors[self.window.CURSOR_TEXT]
        self.assertTrue(cursor.hw_drawable)
        pixels = cursor.texture.get_image_data().get_data("RGBA", cursor.texture.width*4)
        rgba = set(zip(*[iter(pixels)]*4))
        self.assertIn((0, 0, 0, 255), rgba)
        self.assertIn((255, 255, 255, 255), rgba)
        self.assertIn((0, 0, 0, 0), rgba)


if __name__ == "__main__":
    unittest.main()
