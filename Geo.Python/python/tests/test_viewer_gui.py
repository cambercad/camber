"""Pyglet-only controls and viewer lifecycle; run GL tests with CAMBER_TEST_GL=1."""
import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


class GuiContentRegionTests(unittest.TestCase):
    def test_sketch_tooltips_explain_every_palette_tool(self):
        from camber import sketch_icons as icons

        groups = (icons.CURVE_TOOLS, icons.CONSTRAINT_TOOLS, icons.DIMENSION_TOOLS)
        for prefix, group in zip(("", "con_", "dim_"), groups):
            for tool, label, _drawer in group:
                description = icons._TOOLTIPS[prefix + tool]
                self.assertNotEqual(label, description)
                self.assertTrue(description.endswith("."))

    def test_available_content_uses_each_inset_once(self):
        from types import SimpleNamespace
        from camber.gui import UI

        ui = UI.__new__(UI)
        ui._current = SimpleNamespace(rect=(0, 0, 231, 100),
                                      cursor=(10, 20), padding=(10, 8),
                                      bottom=20, right=10, spacing=(8, 6))
        ui._nav_order = []
        ui._button_at = lambda rect, *args, **kwargs: rect
        self.assertEqual((211, 72), ui.GetContentRegionAvail())
        row = ui.Selectable("constraint", selected=True)
        self.assertEqual((10, 20, 211, 24), row)


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class ViewerGuiTests(unittest.TestCase):
    def setUp(self):
        self.viewers = []

    def tearDown(self):
        for viewer in reversed(self.viewers):
            viewer.close()

    def viewer(self):
        from camber.host import Viewer
        viewer = Viewer(size=(640,480))
        self.viewers.append(viewer)
        return viewer

    def draw(self, viewer, callback):
        viewer._tick = callback
        viewer.window.switch_to()
        viewer.on_draw()

    def click(self, ui, x, y):
        from pyglet.window import mouse
        ui.on_mouse_press(x,ui.window.height-y,mouse.LEFT,0)
        ui.on_mouse_release(x,ui.window.height-y,mouse.LEFT,0)

    def test_two_windows_have_independent_input_and_release_resources(self):
        first,second = self.viewer(),self.viewer()
        self.assertIsNot(first._ui,second._ui)
        first._letter(first._pyglet.window.key.A,True)
        self.assertEqual({"A"},first._ui.shortcut_letters)
        self.assertEqual(set(),second._ui.shortcut_letters)
        for v in (first,second,first):
            self.draw(v,lambda v=v: (v._ui.Begin("Test"),v._ui.Text("Owned UI"),v._ui.End()))
        first.close()
        first.close()
        self.assertIsNone(first._ui)
        self.assertIsNone(first.window.context)
        self.draw(second,lambda: (second._ui.Begin("Test"),second._ui.Text("Still alive"),second._ui.End()))

    def test_dock_tab_splitter_and_camera_pick_region(self):
        v = self.viewer()
        ui, mouse = v._ui, v._pyglet.window.mouse
        ui.dock_panel("Features", "left")
        ui.dock_panel("History", "center", "Features")

        def panels():
            for title in ("Features", "History"):
                if ui.Begin(title):
                    ui.Text(title)
                ui.End()

        self.draw(v, panels)
        self.assertEqual("History", ui.docks._leaf("Features")["active"])
        self.assertEqual((181, 0, 459, 480), ui.viewport_rect)
        self.assertEqual((181, 0, 459, 480), v.camera.viewport)
        self.click(ui, 25, 15)
        self.draw(v, panels)
        self.assertEqual("Features", ui.docks._leaf("Features")["active"])
        node, bar, _ = ui.docks.splitters[0]
        bx, by, _, _ = bar
        ui.on_mouse_press(bx+2, v.window.height-by-40, mouse.LEFT, 0)
        ui.on_mouse_drag(bx+52, v.window.height-by-40, 50, 0, mouse.LEFT, 0)
        ui.on_mouse_release(bx+52, v.window.height-by-40, mouse.LEFT, 0)
        self.draw(v, panels)
        self.assertGreater(ui.viewport_rect[0], 220)
        tab = next(rect for title, rect in ui._dock_tabs if title == "Features")
        tx, ty = tab[:2]
        ui.on_mouse_press(tx+10, v.window.height-ty-10, mouse.LEFT, 0)
        ui.on_mouse_drag(630, v.window.height-200, 630-tx-10, ty+10-200, mouse.LEFT, 0)
        ui.on_mouse_release(630, v.window.height-200, mouse.LEFT, 0)
        self.draw(v, panels)
        self.assertIsNot(ui.docks._leaf("Features"), ui.docks._leaf("History"))
        ui.set_dock_visible("Features", False)
        ui.set_dock_visible("History", False)
        self.draw(v, panels)
        self.assertEqual((0, 0, 640, 480), ui.viewport_rect)

    def test_long_docked_list_culls_clipped_gpu_items(self):
        v = self.viewer()
        ui = v._ui
        ui.dock_panel("Items")
        def panel():
            if ui.Begin("Items"):
                for i in range(1000):
                    ui.Selectable("Item " + str(i))
            ui.End()
        self.draw(v, panel)
        self.assertLess(len(ui._cache), 100)

    def test_docked_scrollbar_wheel_and_track(self):
        v = self.viewer()
        ui, mouse = v._ui, v._pyglet.window.mouse
        ui.dock_panel("Items")
        def panel():
            if ui.Begin("Items"):
                for i in range(80):
                    ui.Selectable("Item " + str(i))
            ui.End()
        self.draw(v, panel)
        p = ui._panels["Items"]
        self.assertGreater(p.max_scroll, 0)
        track, thumb = ui._scroll_geometry(p)
        self.assertLess(thumb[3], track[3])
        ui.on_mouse_scroll(30, v.window.height-70, 0, -3)
        self.draw(v, panel)
        self.assertEqual(96, p.scroll)
        ui.on_mouse_press(track[0]+4, v.window.height-track[1]-track[3]+4, mouse.LEFT, 0)
        ui.on_mouse_release(track[0]+4, v.window.height-track[1]-track[3]+4, mouse.LEFT, 0)
        self.draw(v, panel)
        self.assertGreater(p.scroll, 96)

    def test_grouped_grid_selects_rows_and_scrolls(self):
        v = self.viewer()
        ui = v._ui
        selected = [-1]
        rows = [("Alice", "North", True), ("Bob", "North", False)] + [
            ("Person " + str(i), "South", True) for i in range(35)]
        def panel():
            ui.Begin("Grid")
            selected[0] = ui.DataGrid(("Name", "Group", "Status"), rows,
                                      selected[0], group_by=1)
            ui.End()
        self.draw(v, panel)
        label = next(args for kind, args, _ in ui._commands
                     if kind == "text_align" and args[2] == "Alice")
        self.click(ui, *label[0])
        self.draw(v, panel)
        self.assertEqual(0, selected[0])
        self.assertGreater(ui._panels["Grid"].max_scroll, 0)

    def test_custom_slider_responds_to_pointer(self):
        v = self.viewer()
        ui = v._ui
        value = [0.0]
        def panel():
            ui.Begin("Controls")
            _, value[0] = ui.SliderFloat("Level", value[0], width=200)
            ui.End()
        self.draw(v, panel)
        x, y, w, h = ui._last
        self.click(ui, x+w//2, y+h//2)
        self.draw(v, panel)
        self.assertAlmostEqual(.5, value[0], places=2)

    def test_button_label_is_centered_in_its_rect(self):
        v = self.viewer()
        ui = v._ui
        def panel():
            ui.Begin("Controls")
            ui.Button("Apply", 120, 30)
            ui.End()
        self.draw(v, panel)
        x, y, w, h = ui._last
        labels = [args for kind, args, _ in ui._commands
                  if kind == "text_align" and args[2] == "Apply"]
        self.assertEqual(1, len(labels))
        self.assertEqual(((x+w/2, y+h/2), "center"), (labels[0][0], labels[0][4]))

    def test_collapsed_window_restores_its_expanded_height(self):
        v = self.viewer()
        ui = v._ui

        def panel():
            ui.SetNextWindowPos((20, 20))
            ui.SetNextWindowSize((260, 220))
            ui.Begin("Inspect")
            if not ui._current.collapsed:
                ui.Text("Measurement details")
            ui.End()

        self.draw(v, panel)
        self.click(ui, 33, 33)
        self.draw(v, panel)
        self.assertTrue(ui._panels["Inspect"].collapsed)
        self.assertEqual(26, ui._panels["Inspect"].rect[3])

        self.click(ui, 33, 33)
        self.draw(v, panel)
        self.assertFalse(ui._panels["Inspect"].collapsed)
        self.assertEqual(220, ui._panels["Inspect"].rect[3])

    def test_focused_text_field_draws_caret_in_clipped_ui(self):
        v = self.viewer()
        ui = v._ui
        def panel():
            ui.Begin("Controls")
            ui.InputText("Name", "Example", 160)
            ui.End()
        self.draw(v, panel)
        eid, editor = next(iter(ui._editors.items()))
        ui._focus = eid
        editor.caret.position = len(editor.document.text)
        with patch("camber.gui.time.monotonic", return_value=.1):
            self.draw(v, panel)
        x, y, w, h = editor.rect
        carets = [args for kind, args, _ in ui._commands
                  if kind == "line" and args[0][0] == args[1][0]
                  and x < args[0][0] < x+w and y < args[0][1] < y+h]
        self.assertEqual(1, len(carets))

    def test_text_cursor_and_theme_recolor_without_reordering_field(self):
        v = self.viewer()
        ui = v._ui
        def panel():
            ui.Begin("Controls")
            ui.InputText("Name", "Example", 160)
            ui.End()
        self.draw(v, panel)
        editor = next(iter(ui._editors.values()))
        x, y, w, h = editor.rect
        ui.on_mouse_motion(x+8, v.window.height-y-h//2, 0, 0)
        self.assertEqual(v.window.CURSOR_TEXT, ui._cursor_name)
        ui.on_mouse_motion(0, 0, 0, 0)
        self.assertEqual(v.window.CURSOR_DEFAULT, ui._cursor_name)
        outer = next(obj for signature, obj in ui._cache
                     if signature[0] == "rrect" and signature[1][0] == (x, y))
        ui.set_theme("dark")
        self.draw(v, panel)
        recolored = next(obj for signature, obj in ui._cache
                         if signature[0] == "rrect" and signature[1][0] == (x, y))
        self.assertIs(outer, recolored)
        self.assertEqual(ui.theme["text"], editor.document.get_style("color", 0))
        ui.set_theme("light")
        self.draw(v, panel)

    def test_choice_controls_mouse_and_keyboard(self):
        v = self.viewer()
        ui = v._ui
        choice, selected = ["Medium"], [0]
        def panel():
            ui.Begin("Controls")
            if ui.RadioButton("Fast", selected[0] == 1):
                selected[0] = 1
            changed, choice[0] = ui.ComboBox("Size", choice[0],
                                             ("Small", "Medium", "Large"), 150)
            ui.End()
        self.draw(v, panel)
        self.click(ui, 30, 58)
        self.draw(v, panel)
        self.assertEqual(1, selected[0])
        anchor = ui._last
        self.click(ui, anchor[0]+20, anchor[1]+13)
        self.draw(v, panel)
        self.assertIsNotNone(ui._combo_popup)
        _, _, popup, _ = ui._combo_popup
        self.click(ui, popup[0]+20, popup[1]+2*26+13)
        self.draw(v, panel)
        self.assertEqual("Large", choice[0])
        self.click(ui, anchor[0]+20, anchor[1]+13)
        self.draw(v, panel)
        from pyglet.window import key
        ui.on_key_press(key.UP, 0)
        self.draw(v, panel)
        ui.on_key_press(key.RETURN, 0)
        self.draw(v, panel)
        self.assertEqual("Medium", choice[0])

    def test_long_combo_scrolls_without_scrolling_its_panel(self):
        v = self.viewer()
        ui = v._ui
        value = ["Option 0"]
        options = ["Option " + str(i) for i in range(20)]
        def panel():
            ui.Begin("Controls")
            _, value[0] = ui.ComboBox("Choice", value[0], options, 180)
            ui.End()
        self.draw(v, panel)
        x, y, w, h = ui._last
        self.click(ui, x+w//2, y+h//2)
        self.draw(v, panel)
        _, _, popup, _ = ui._combo_popup
        ui.on_mouse_scroll(popup[0]+20, v.window.height-popup[1]-20, 0, -5)
        self.draw(v, panel)
        self.assertEqual(5, ui._combo_scroll)
        self.assertEqual(0, ui._panels["Controls"].scroll)
        self.click(ui, popup[0]+20, popup[1]+13)
        self.draw(v, panel)
        self.assertEqual("Option 5", value[0])

    def test_switch_tree_and_multiline_editing(self):
        v = self.viewer()
        ui = v._ui
        enabled, notes, expanded = [False], ["One"], [True]
        def panel():
            ui.Begin("Controls")
            _, enabled[0] = ui.Switch("Enabled", enabled[0])
            _, expanded[0] = ui.TreeNode("Root", default_open=True)
            if expanded[0]:
                ui.TreeNode("Child", 1, leaf=True)
            _, notes[0] = ui.InputTextMultiline("Notes", notes[0], 200, 80)
            ui.End()
        self.draw(v, panel)
        switch = next(args[0] for kind, args, _ in ui._commands
                      if kind == "rrect" and args[2] == 7)
        self.click(ui, switch[0]+8, switch[1]+8)
        self.draw(v, panel)
        self.assertTrue(enabled[0])
        root = next(args[0] for kind, args, _ in ui._commands
                    if kind == "text_align" and args[2] == "Root")
        self.click(ui, *root)
        self.draw(v, panel)
        self.assertFalse(expanded[0])
        eid, editor = next((eid, editor) for eid, editor in ui._editors.items()
                           if editor.multiline)
        ui._focus = eid
        editor.caret.position = len(editor.document.text)
        ui.on_text("\n")
        ui.on_text("Two")
        self.draw(v, panel)
        self.assertEqual("One\nTwo", notes[0])

    def test_tabs_buttons_and_slider_are_keyboard_operable(self):
        v = self.viewer()
        ui = v._ui
        from pyglet.window import key
        active, level = ["First"], [.5]
        def panel():
            ui.Begin("Controls")
            for name in ("First", "Second"):
                if ui.TabButton(name, active[0] == name):
                    active[0] = name
                ui.SameLine()
            _, level[0] = ui.SliderFloat("Level", level[0], width=160)
            ui.End()
        self.draw(v, panel)
        ui.on_key_press(key.TAB, 0)
        self.assertEqual(ui._nav_order[0], ui._focus)
        ui.on_key_press(key.TAB, 0)
        self.assertEqual(ui._nav_order[1], ui._focus)
        ui.on_key_press(key.RETURN, 0)
        self.draw(v, panel)
        self.assertEqual("Second", active[0])
        ui.on_key_press(key.TAB, 0)
        ui.on_key_press(key.RIGHT, 0)
        self.draw(v, panel)
        self.assertAlmostEqual(.51, level[0])

    def test_tabs_switch_without_rendering_both_contents(self):
        v = self.viewer()
        ui = v._ui
        shown = []
        def panel():
            ui.Begin("Assembly")
            ui.BeginTabBar("tabs")
            for label in ("Parts","Constraints"):
                if ui.BeginTabItem(label):
                    shown.append(label)
                    ui.Text(label+" content")
                    ui.EndTabItem()
            ui.EndTabBar()
            ui.End()
        self.draw(v,panel)
        self.assertEqual(["Parts"],shown)
        _,_,rect,_ = ui._tab_hits[1]
        self.click(ui,rect[0]+5,rect[1]+5)
        shown.clear()
        self.draw(v,panel)
        self.assertEqual(["Constraints"],shown)
        shown.clear()
        self.draw(v,panel)
        self.assertEqual(["Constraints"],shown)

    def test_checkbox_scroll_and_clipping(self):
        v = self.viewer()
        ui = v._ui
        value = [False]
        def panel():
            ui.SetNextWindowSize((300,130))
            ui.Begin("Options")
            changed,value[0] = ui.Checkbox("Wireframe",value[0])
            for i in range(25):
                ui.Text("Row "+str(i))
            ui.End()
        self.draw(v,panel)
        self.click(ui,35,52)
        self.draw(v,panel)
        self.assertTrue(value[0])
        p = ui._panels["Options"]
        self.assertGreater(p.max_scroll,100)
        ui.on_mouse_scroll(70,v.window.height-90,0,-4)
        self.draw(v,panel)
        self.assertEqual(128,p.scroll)
        self.assertTrue(ui.io.WantCaptureMouse)

    def test_numeric_edit_validation_escape_tab_and_clipboard(self):
        v = self.viewer()
        ui,key = v._ui,v._pyglet.window.key
        values = [12.5,3.0]
        focus = [True]
        def panel():
            ui.Begin("Dimensions")
            for i in range(2):
                if focus[0] and i == 0:
                    ui.SetKeyboardFocusHere()
                _,values[i] = ui.InputDouble(str(i),values[i])
            ui.End()
            focus[0] = False
        self.draw(v,panel)
        self.assertTrue(ui.io.WantCaptureKeyboard)
        ui.on_text("27.25")
        self.draw(v,panel)
        self.assertEqual(27.25,values[0])
        ui.on_key_press(key.A,key.MOD_CTRL)
        ui.on_text("nan")
        self.draw(v,panel)
        self.assertEqual(27.25,values[0])
        ui.on_key_press(key.ESCAPE,0)
        self.draw(v,panel)
        self.assertEqual(12.5,values[0])
        eid = ui._focus_order[0]
        rect = ui._editors[eid].rect
        self.click(ui,rect[0]+5,rect[1]+5)
        ui.on_key_press(key.A,key.MOD_CTRL)
        ui.on_key_press(key.C,key.MOD_CTRL)
        self.assertEqual("12.5000",v.window.get_clipboard_text())
        ui.on_key_press(key.TAB,0)
        self.assertEqual(ui._focus_order[1],ui._focus)
        ui._focus = None
        ui.on_key_press(key.TAB,0)
        self.assertEqual(ui._focus_order[0],ui._focus)
        v.on_key_press(key.W,0)
        self.assertFalse(v._wireframe)

    def test_title_close_drag_and_resize(self):
        v = self.viewer()
        ui,mouse = v._ui,v._pyglet.window.mouse
        opened = [True]
        def panel():
            _,opened[0] = ui.Begin("Movable",opened[0])
            ui.Text("Test")
            ui.End()
        self.draw(v,panel)
        ui.on_mouse_press(45,460,mouse.LEFT,0)
        ui.on_mouse_drag(75,450,30,-10,mouse.LEFT,0)
        ui.on_mouse_release(75,450,mouse.LEFT,0)
        self.draw(v,panel)
        self.assertEqual((42,22),ui._panels["Movable"].rect[:2])
        x,y,w,h = ui._panels["Movable"].rect
        ui.on_mouse_press(x+w-5,480-y-h+5,mouse.LEFT,0)
        ui.on_mouse_drag(x+w+15,480-y-h-5,20,-10,mouse.LEFT,0)
        ui.on_mouse_release(x+w+15,480-y-h-5,mouse.LEFT,0)
        self.draw(v,panel)
        self.assertEqual((w+20,h+10),ui._panels["Movable"].rect[2:])
        x,y,w,h = ui._panels["Movable"].rect
        self.click(ui,x+w-15,y+10)
        self.draw(v,panel)
        self.assertFalse(opened[0])

    def test_native_close_with_render_toggles_and_close_inside_callback(self):
        v = self.viewer()
        v._wireframe = v._solid_transparent = True
        v._transparency = "pretty"
        v.window._allow_dispatch_event = True
        v.window.dispatch_event("on_close")
        self.assertTrue(v._closed)
        self.assertIsNone(v.window.context)
        v.on_draw()
        v = self.viewer()
        self.draw(v,v.close)
        self.assertTrue(v._closed)
        v.on_draw()

    def test_initialization_failure_closes_ui_and_later_window_works(self):
        from camber.gui import UI
        from camber.host import Viewer
        with patch.object(Viewer,"_compile",side_effect=RuntimeError("shader failed")), \
             patch.object(UI,"shutdown",autospec=True,side_effect=UI.shutdown) as shutdown:
            with self.assertRaisesRegex(RuntimeError,"shader failed"):
                Viewer(size=(100,100))
            shutdown.assert_called_once()
        self.viewer()

    def test_text_undo_redo_and_paste(self):
        v = self.viewer()
        ui,key = v._ui,v._pyglet.window.key
        value = [5.0]
        def panel():
            ui.Begin("Edit")
            _,value[0] = ui.InputDouble("value",value[0])
            ui.End()
        self.draw(v,panel)
        ui.on_key_press(key.TAB,0)
        ui.on_key_press(key.A,key.MOD_CTRL)
        ui.on_text("17")
        self.draw(v,panel)
        self.assertEqual(17,value[0])
        ui.on_key_press(key.Z,key.MOD_CTRL)
        self.draw(v,panel)
        self.assertEqual(5,value[0])
        ui.on_key_press(key.Y,key.MOD_CTRL)
        self.draw(v,panel)
        self.assertEqual(17,value[0])
        ui.on_key_press(key.A,key.MOD_CTRL)
        with patch.object(v.window, "get_clipboard_text", return_value="42.5"):
            ui.on_key_press(key.V,key.MOD_CTRL)
        self.draw(v,panel)
        self.assertEqual(42.5,value[0])

    def test_sketch_icons_and_dimension_editor_render(self):
        from camber import sketch_icons as icons
        v = self.viewer()
        ui = v._ui
        def panel():
            icons.begin_palette(ui)
            icons.icon_button(ui,"line","Line",icons.draw_line,True)
            icons.dimension_editor(ui,"length",10,True)
            icons.end_palette(ui)
        self.draw(v,panel)
        self.draw(v,panel)
        self.assertTrue(any(kind == "rrect" and args[-1] == ui.theme["selected"]
                            for kind,args,_ in ui._commands))
        with patch.object(ui, "IsItemHovered", return_value=True), \
                patch.object(ui, "SetTooltip") as tooltip:
            self.draw(v,panel)
        tooltip.assert_called_with("Draw a line between two points.")
        ui.set_theme("dark")
        self.draw(v,panel)
        self.assertTrue(any(kind == "rrect" and args[-1] == ui.theme["selected"]
                            for kind,args,_ in ui._commands))

    def test_sketch_action_labels_share_the_standard_button_baseline(self):
        from camber import sketch_icons as icons

        v = self.viewer()
        ui = v._ui
        def panel():
            icons.begin_palette(ui)
            icons.action_button(ui, "Finish", primary=True)
            ui.SameLine()
            icons.action_button(ui, "Cancel")
            icons.end_palette(ui)
        self.draw(v, panel)
        labels = [args for kind,args,_ in ui._commands
                  if kind == "text_align" and args[2] in ("Finish", "Cancel")]
        self.assertEqual(2, len(labels))
        self.assertEqual(labels[0][0][1], labels[1][0][1])
        self.assertTrue(all(args[4] == "center" for args in labels))

    def test_constraint_label_matches_sketch_blue_at_half_size(self):
        from camber import sketch_ui
        from camber.sketch_constraints_viz import CONSTRAINT_COLOR

        v = self.viewer()
        ui = v._ui
        def panel():
            sketch_ui._draw_constraint_labels(v, ui, {
                "cons_labels": [("PoC", (0, 0, 0), "cons", 0)]})
        self.draw(v, panel)
        label = next(args for kind,args,_ in ui._foreground
                     if kind == "text" and args[2] == "PoC")
        self.assertEqual(tuple(round(channel*255) for channel in CONSTRAINT_COLOR)+(255,),
                         label[1])
        self.assertLessEqual(label[3], 32)

    def test_camber_panels_use_native_theme_and_controls(self):
        from camber.view import _draw_assembly_panel, _draw_operations_panel
        from types import SimpleNamespace

        v = self.viewer()
        ui = v._ui
        ui.dock_panel("Assembly (M hide)", "left")
        selected, opened, hidden = [-1], [True], set()
        constraints = [{"kind": "Concentric", "entities": ["pipe1:axis", "pipe2:axis"]}]

        def assembly():
            _draw_assembly_panel(ui, constraints, selected, opened, ["pipe1", "pipe2"], hidden)

        self.draw(v, assembly)
        self.assertIn("Assembly (M hide)", ui._panels)
        checkbox = next(args for kind,args,_ in ui._commands
                        if kind == "text_align" and args[2] == "pipe1")
        self.click(ui, checkbox[0][0], checkbox[0][1])
        self.draw(v, assembly)
        self.assertEqual({"pipe1"}, hidden)
        tab = next(rect for identity,label,rect,panel in ui._tab_hits if label == "Constraints")
        self.click(ui, tab[0]+8, tab[1]+12)
        self.draw(v, assembly)
        self.assertTrue(any(kind == "text_align" and "Concentric" in args[2]
                            for kind,args,_ in ui._commands))
        v.on_key_press(v._pyglet.window.key.D, 0)
        self.assertEqual("dark", ui.theme_name)
        self.draw(v, assembly)
        self.assertTrue(any(kind == "rect" and args[-1] == ui.theme["content"]
                            for kind,args,_ in ui._dock_chrome()))

        operations = [SimpleNamespace(index=1, kind="Extrude", result="body", inputs=(),
                                      details="", entities=("body:top",))]
        ui.set_dock_visible("Assembly (M hide)", False)
        ui.dock_panel("Features (M hide)", "left")
        def features():
            _draw_operations_panel(ui, operations, [-1], [True])
        self.draw(v, features)
        self.assertTrue(any(kind == "text_align" and "Extrude" in args[2]
                            for kind,args,_ in ui._commands))

    def test_theme_shortcut_does_not_interrupt_text_editing(self):
        v = self.viewer()
        ui, key = v._ui, v._pyglet.window.key
        def panel():
            ui.Begin("Edit")
            ui.InputText("Name", "Part", 160)
            ui.End()
        self.draw(v, panel)
        ui._focus = ui._focus_order[0]
        v.on_key_press(key.D, 0)
        self.assertEqual("light", ui.theme_name)
        v.on_key_release(key.D, 0)
        ui._focus = None
        v.on_key_press(key.D, 0)
        self.assertEqual("dark", ui.theme_name)

    def test_overlapping_panels_do_not_click_through(self):
        v = self.viewer()
        ui = v._ui
        clicks = []
        def panel():
            for name in ("behind","front"):
                ui.SetNextWindowPos((12,12))
                ui.Begin(name)
                if ui.Button("Action"):
                    clicks.append(name)
                ui.End()
        self.draw(v,panel)
        self.click(ui,40,52)
        self.draw(v,panel)
        self.assertEqual(["front"],clicks)

    def test_consumed_escape_does_not_leak_to_cad_input(self):
        v = self.viewer()
        ui,key = v._ui,v._pyglet.window.key
        def panel():
            ui.Begin("Edit")
            ui.InputDouble("value",2)
            ui.End()
        self.draw(v,panel)
        rect = ui._editors[ui._focus_order[0]].rect
        self.click(ui,rect[0]+5,rect[1]+5)
        self.draw(v,panel)
        v.window._allow_dispatch_event = True
        v.window.dispatch_event("on_key_press",key.ESCAPE,0)
        self.assertFalse(ui.IsKeyDown(key.ESCAPE))
        self.assertFalse(v.key_down("ESCAPE"))
        v.window.dispatch_event("on_key_release",key.ESCAPE,0)
        v.window.dispatch_event("on_key_press",key.ESCAPE,0)
        self.assertTrue(v.key_down("ESCAPE"))

    def test_real_sketch_session_and_no_imgui_import(self):
        from camber import Part
        from camber.sketch_ui import begin_sketch_session,teardown_sketch_graphics
        v = self.viewer()
        part = Part((-10,-10,-10),(10,10,10))
        sketch = part.sketch(constrained=True)
        sketch.add_line((-2,0),(2,0))
        session = begin_sketch_session(v,v._ui,part,existing_sketch=sketch,look_at=True)
        try:
            self.draw(v,session["tick"])
            self.draw(v,session["tick"])
            self.assertFalse(any(n == "imgui" or n.startswith("imgui.") for n in sys.modules))
        finally:
            teardown_sketch_graphics(v,session["viz"])


if __name__ == "__main__":
    unittest.main()
