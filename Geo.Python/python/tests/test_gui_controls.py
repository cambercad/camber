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
    def test_tree_and_selection_labels_fit_the_row_and_reveal_full_text_on_hover(self):
        text = "Point at curve parameter 12 — 0.375"

        def frame(width, selected):
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((width, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Tree", self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.TreeNode(text+"##tree", depth=2, leaf=True, selected=selected)
            tree = self.ui._last
            self.ui.Selectable(text+"##list", selected)
            rows = (tree, self.ui._last)
            self.ui.End()
            self.ui.render()
            labels = [args for kind, args, _ in self.ui._commands
                      if kind == "text_align"]
            self.assertEqual(2, len(labels))
            for row, args in zip(rows, labels):
                self.assertLessEqual(args[0][0]+self.ui.CalcTextSize(args[2])[0], row[0]+row[2]-8)
                if self.ui.CalcTextSize(text)[0] > row[0]+row[2]-8-args[0][0]:
                    self.assertTrue(args[2].endswith("…"))
                else:
                    self.assertEqual(text, args[2])
            return rows, labels

        for theme in ("light", "dark"):
            self.ui.set_theme(theme)
            for width in (220, 280):
                for selected in (False, True):
                    self.ui.on_mouse_motion(-1, -1, 0, 0)
                    rows, labels = frame(width, selected)
                    for (x, y, w, h), args in zip(rows, labels):
                        if args[2] == text:
                            continue
                        self.ui.on_mouse_motion(x+w/2, self.window.height-y-h/2, 0, 0)
                        frame(width, selected)
                        self.assertTrue(any(kind == "text" and args[2] == text
                                            for kind, args, _ in self.ui._foreground))

    def test_unhandled_key_taps_survive_release_until_the_frame_is_rendered(self):
        from pyglet.window import key
        self.ui.on_key_press(key.ESCAPE, key.MOD_NUMLOCK | key.MOD_CAPSLOCK)
        self.ui.on_key_release(key.ESCAPE, 0)
        self.ui.new_frame()
        self.assertFalse(self.ui.IsKeyDown(key.ESCAPE))
        self.assertTrue(self.ui.IsKeyPressed(key.ESCAPE, modifiers=0))
        self.ui.render()
        self.assertFalse(self.ui.IsKeyPressed(key.ESCAPE))
        self.ui.on_key_press(key.L, key.MOD_CTRL)
        self.ui.on_key_release(key.L, 0)
        self.assertFalse(self.ui.IsKeyPressed(key.L, modifiers=0))
        self.assertTrue(self.ui.IsKeyPressed(key.L, modifiers=key.MOD_CTRL))
        self.ui.on_deactivate()
        self.assertFalse(self.ui.IsKeyPressed(key.L))

    def test_button_text_is_crisp_at_fractional_layout_positions(self):
        def glyph_pixels(width, height):
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420,240),self.ui.ImGuiCond_Always)
            self.ui.Begin("Buttons",self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.Button("Cancel",width=width,height=height)
            self.ui.End()
            self.ui.render()
            label = next(obj for signature,obj in self.ui._cache
                         if signature[0] == "text_align" and obj.text == "Cancel")
            pixels = self.pixels()
            x, y = round(label.x), round(label.y)
            return b"".join(bytes(pixels[((y+dy)*self.window.width+x-25)*4:
                                   ((y+dy)*self.window.width+x+25)*4])
                            for dy in range(-9,9))
        for theme in ("light","dark"):
            self.ui.set_theme(theme)
            reference = glyph_pixels(100,32)
            for width,height in ((101,32),(100.5,32),(101,33),(101.5,32.5)):
                self.assertEqual(reference,glyph_pixels(width,height),(theme,width,height))

    def test_scrolling_preserves_field_and_combo_backgrounds(self):
        def frame():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420,240),self.ui.ImGuiCond_Always)
            self.ui.Begin("Form",self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.BeginChild("Fields",height=190)
            rects = []
            for index in range(18):
                self.ui.TextUnformatted("Coordinates " + str(index))
                if index % 3 == 0:
                    self.ui.ComboBox(str(index),"Custom",("Custom","Default"),width=300)
                    rects.append(self.ui._last)
                else:
                    for component in range(3):
                        if component:
                            self.ui.SameLine()
                        self.ui.InputText(str(index)+"/"+str(component),"32.0",width=90)
                        rects.append(self.ui._last)
            child = self.ui._current
            self.ui.EndChild()
            self.ui.End()
            self.ui.render()
            orders = {signature[3] for signature,obj in self.ui._cache
                      if signature[2] == self.ui._clip(child)}
            self.assertLessEqual(len(orders),8)  # Layers batch across disjoint controls.
            pixels = self.pixels()
            clip = self.ui._clip(child)
            for x,y,w,h in rects:
                row = round(y+h/2)
                if clip[1]+2 <= row < clip[1]+clip[3]-2:
                    column = round(x+w*.75)
                    offset = ((self.window.height-1-row)*self.window.width+column)*4
                    self.assertEqual(list(self.ui.theme["field"]),list(pixels[offset:offset+4]),(child.scroll,row))
            return child
        for theme in ("light","dark"):
            self.ui.set_theme(theme)
            child = frame()
            for amount in (-1,-2,-3,1,4,-4,2,-1):
                x,y,w,h = child.rect
                self.ui.on_mouse_scroll(x+20,self.window.height-y-60,0,amount)
                child = frame()

    def test_ribbon_hover_preserves_cached_icon_primitives(self):
        def icon(dl, x, y, ink, accent):
            dl.AddLine((x+3,y+3),(x+21,y+21),ink,2)
            dl.AddCircle((x+12,y+12),6,accent,12,2)
        def frame(mouse):
            self.ui.new_frame()
            self.ui.io.MousePos = mouse
            self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420,240),self.ui.ImGuiCond_Always)
            self.ui.Begin("Ribbon",self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.RibbonButton("Command",icon)
            rect = self.ui._last
            self.ui.End()
            self.ui.render()
            return rect, [obj for signature,obj in self.ui._cache if signature[0] in ("line","arc")]
        rect, initial = frame((-10,-10))
        _, hovered = frame((rect[0]+20,rect[1]+20))
        _, restored = frame((-10,-10))
        self.assertTrue(initial)
        self.assertEqual(initial,hovered)
        self.assertEqual(initial,restored)

    def test_tooltip_wraps_and_uses_measured_font_and_padding(self):
        self.ui.new_frame()
        self.ui.io.MousePos = (415,235)
        text = "Place the feature on a planar face. "*8 + "\nDouble-click to edit parameters."
        self.ui.SetTooltip(text)
        background, inner, label = self.ui._foreground
        args = label[1]
        width = args[4]
        _, height = self.ui.CalcTextSize(text,wrap_width=width)
        self.assertEqual(self.ui._font_size(),args[3])
        self.assertEqual(width+16,background[1][1][0]-background[1][0][0])
        self.assertEqual(height+12,background[1][1][1]-background[1][0][1])
        self.assertGreater(height,30)
        self.assertLessEqual(background[1][1][0],self.window.width)
        self.assertLessEqual(background[1][1][1],self.window.height)
        self.ui.render()
        rendered = [obj for signature,obj in self.ui._cache if signature[0] == "text"][-1]
        self.assertEqual(height,rendered.content_height)

    def test_invalid_text_field_keeps_error_border_when_focused(self):
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((420,240),self.ui.ImGuiCond_Always)
        self.ui.Begin("Form",self.ui.ImGuiWindowFlags_NoDecoration)
        self.ui._focus = self.ui._id("Value")
        self.ui.InputText("Value","-",error="Enter a number")
        borders = [args for kind,args,_ in self.ui._commands if kind == "rrect"]
        self.assertEqual(self.ui.theme["error"],borders[-2][3])
        self.ui.End()
        self.ui.render()

    def test_validation_message_and_disabled_primary_use_theme_colors(self):
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0,0),self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((420,240),self.ui.ImGuiCond_Always)
        self.ui.Begin("Form",self.ui.ImGuiWindowFlags_NoDecoration)
        self.ui.ValidationMessage("Enter a number")
        self.ui.Button("Apply",primary=True,enabled=False)
        texts = [args for kind,args,_ in self.ui._commands if kind in ("text", "text_align")]
        self.assertEqual(self.ui.theme["error"],texts[0][1])
        self.assertEqual(self.ui.theme["muted"],texts[-1][1])
        self.ui.End()
        self.ui.render()

    def test_ribbon_columns_and_group_labels_align_without_a_header(self):
        self.ui.dock_panel("Ribbon", "top", size=124, header=False)
        self.ui.new_frame()
        self.ui.Begin("Ribbon")
        self.assertEqual(0, self.ui._current.title_h)
        def icon(dl, x, y, ink, accent):
            dl.AddLine((x,y),(x+24,y+24),ink)
        compact = []
        for label in ("Create", "Modify"):
            self.ui.BeginRibbonGroup(label)
            self.ui.RibbonButton("Primary", icon)
            self.ui.BeginRibbonColumn()
            column = []
            for command in ("First", "Second", "Third"):
                self.ui.RibbonButton(command, icon, width=100, height=22, icon_size=16, compact=True)
                column.append(self.ui._last)
            self.ui.EndRibbonColumn()
            self.ui.EndRibbonGroup()
            compact.append(column)
        self.assertEqual(1,len({r[0] for r in compact[0]}))
        self.assertEqual([22,22],[b[1]-a[1] for a,b in zip(compact[0],compact[0][1:])])
        self.assertGreater(compact[1][0][0],compact[0][0][0]+100)
        footers = [args[0][1] for kind,args,_ in self.ui._commands
                   if kind == "text_align" and args[2] in ("Create", "Modify")]
        self.assertEqual(footers[0],footers[1])
        self.ui.End()
        self.assertEqual(0,self.ui._panels["Ribbon"].max_scroll)
        self.ui.render()

    def test_ribbon_group_can_start_with_multiple_compact_columns(self):
        self.ui.dock_panel("Ribbon", "top", size=124, header=False)
        self.ui.new_frame()
        self.ui.Begin("Ribbon")
        self.ui.BeginRibbonGroup("Tools")
        positions = []
        for column in range(2):
            self.ui.BeginRibbonColumn()
            for row in range(3):
                self.ui.RibbonButton(f"Tool {column}-{row}", lambda *args: None,
                                     width=100, height=22, compact=True)
                positions.append(self.ui._last)
            self.ui.EndRibbonColumn()
        self.ui.EndRibbonGroup()
        self.ui.End()
        self.assertEqual(positions[0][1], positions[3][1])
        self.assertGreater(positions[3][0], positions[0][0]+100)
        self.assertEqual(0, self.ui._panels["Ribbon"].max_scroll)
        self.ui.render()

    def test_repeated_frames_keep_the_window_camera_and_gpu_buffers_stable(self):
        import warnings
        projection, view = self.window.projection, self.window.view
        with warnings.catch_warnings(record=True) as records:
            warnings.simplefilter("always")
            for _ in range(40):
                self.draw()
        self.assertEqual(projection,self.window.projection)
        self.assertEqual(view,self.window.view)
        self.assertFalse(any("UniformBuffer" in str(r.message) or "RingBuffer" in str(r.message) for r in records))

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

        image = pyglet.graphics.framebuffer.get_screenshot()
        return image.get_bytes("RGBA", self.window.width*4)

    def test_child_scroll_keeps_footer_visible_and_clickable(self):
        def frame():
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            parent = self.ui._current
            self.ui.BeginChild("Fields", height=170)
            child = self.ui._current
            for index in range(30):
                self.ui.InputText(str(index), "Value", width=self.ui.GetContentRegionAvail()[0])
            self.ui.EndChild()
            self.ui.Button("Apply", height=30)
            footer = self.ui._last
            self.ui.End()
            self.ui.render()
            return parent, child, footer
        parent, child, footer = frame()
        self.assertGreater(child.max_scroll, 0)
        bars = [panel for kind, args, panel in self.ui._commands
                if kind == "rrect" and args[2] in (3, 4)]
        self.assertIn(parent, bars)
        x, y, w, h = child.rect
        self.ui.on_mouse_scroll(x+20, self.window.height-y-30, 0, -4)
        self.assertGreater(child.scroll, 0)
        self.assertEqual(0, parent.scroll)
        _, _, after = frame()
        self.assertEqual(footer, after)
        self.assertIs(parent, self.ui._panel_at((footer[0]+5, footer[1]+5)))
        self.assertGreaterEqual(footer[1], y+h)
        self.assertLessEqual(footer[1]+footer[3], parent.rect[1]+parent.rect[3])

    def test_fixed_dock_size_survives_window_resize(self):
        layout = gui.DockLayout()
        layout.add("Toolbar", "top", size=150)
        layout.add("Properties", "right", size=280)
        layout.layout(1200, 800)
        height = layout.rects["Toolbar"][1][3]
        width = layout.rects["Properties"][1][2]
        layout.layout(1400, 1000)
        self.assertEqual(height, layout.rects["Toolbar"][1][3])
        self.assertEqual(width, layout.rects["Properties"][1][2])

    def test_wrapped_text_stays_within_panel_insets(self):
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((180, 220), self.ui.ImGuiCond_Always)
        self.ui.Begin("Help", self.ui.ImGuiWindowFlags_NoDecoration)
        self.ui.TextWrapped("This description is longer than the available width of the property panel.")
        x, y, w, h = self.ui._last
        self.assertLessEqual(x+w, 170)
        self.assertGreater(h, 30)
        self.ui.End()
        self.ui.render()

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
        pixels = cursor.texture.get_bytes("RGBA", cursor.texture.width*4)
        rgba = set(zip(*[iter(pixels)]*4))
        self.assertIn((0, 0, 0, 255), rgba)
        self.assertIn((255, 255, 255, 255), rgba)
        self.assertIn((0, 0, 0, 0), rgba)


if __name__ == "__main__":
    unittest.main()
