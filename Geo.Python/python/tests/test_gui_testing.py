"""Input and screenshot testing of the GUI independently of the CAD package."""
import importlib.util
import os
from pathlib import Path
import unittest


def load(name):
    path = Path(__file__).parents[1]/"camber"/(name+".py")
    spec = importlib.util.spec_from_file_location("standalone_"+name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gui, testing = load("gui"), load("gui_testing")


@unittest.skipUnless(os.environ.get("CAMBER_TEST_GL") == "1", "requires OpenGL")
class GuiDriverTests(unittest.TestCase):
    def test_click_dispatches_keyboard_modifiers(self):
        from pyglet.window import key, mouse
        events=[]
        self.window.push_handlers(on_mouse_press=lambda *args: events.append(args),
                                  on_mouse_release=lambda *args: events.append(args))
        self.driver.click(x=310,y=200,modifiers=key.MOD_ALT)
        self.assertEqual(2,len(events))
        self.assertTrue(all(args[-2:] == (mouse.LEFT,key.MOD_ALT) for args in events))

    def test_drag_dispatches_selected_button_and_modifiers(self):
        from pyglet.window import mouse,key
        events=[]
        self.window.push_handlers(on_mouse_press=lambda *args: events.append(("press",args)),
                                  on_mouse_drag=lambda *args: events.append(("drag",args)),
                                  on_mouse_release=lambda *args: events.append(("release",args)))
        self.driver.drag((310,200),(330,210),steps=2,button="middle",modifiers=key.MOD_SHIFT)
        self.assertEqual(["press","drag","drag","release"],[event for event,_ in events])
        for _,args in events:
            self.assertEqual((mouse.MIDDLE,key.MOD_SHIFT),args[-2:])
        self.assertEqual((10,-5),events[1][1][2:4])
        with self.assertRaisesRegex(ValueError,"Drag button"):
            self.driver.drag((310,200),(330,210),button="unknown")

    def test_click_outside_window_is_rejected(self):
        for x, y in ((-1, 20), (20, -1), (420, 20), (20, 240)):
            with self.subTest(x=x, y=y), self.assertRaisesRegex(ValueError, "inside the window"):
                self.driver.click(x=x, y=y)

    def setUp(self):
        import pyglet
        self.window = pyglet.window.Window(420, 240, visible=False)
        self.ui = gui.UI(self.window)
        self.value, self.enabled, self.clicks = "Initial", False, 0
        self.driver = testing.GuiDriver(self.ui, self.draw)

    def tearDown(self):
        self.ui.shutdown()
        self.window.close()

    def draw(self):
        self.window.clear()
        self.ui.new_frame()
        self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
        self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
        self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
        _, self.value = self.ui.InputText("Name", self.value, width=200)
        _, self.enabled = self.ui.Checkbox("Enabled", self.enabled)
        if self.ui.Button("Apply"):
            self.clicks += 1
        self.ui.Button("Unavailable", enabled=False)
        self.ui.End()
        self.ui.render()

    def test_click_text_tab_and_keyboard_activation_use_window_events(self):
        self.driver.click("Name", panel="Form")
        self.driver.text("Edited", replace=True)
        self.assertEqual("Edited", self.value)
        self.assertFalse(self.ui.io.KeyCtrl)
        self.driver.key("TAB")
        self.driver.key("SPACE")
        self.assertTrue(self.enabled)
        self.driver.key("TAB")
        self.driver.key("RETURN")
        self.assertEqual(1, self.clicks)
        self.driver.click("Apply")
        self.assertEqual(2, self.clicks)

    def test_disabled_and_missing_controls_fail_without_changing_state(self):
        with self.assertRaises(ValueError):
            self.driver.click("Unavailable")
        with self.assertRaises(LookupError):
            self.driver.click("Missing")
        self.assertEqual(0, self.clicks)

    def test_click_uses_layout_after_hover_redraw(self):
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Moving form", self.ui.ImGuiWindowFlags_NoDecoration)
            if self.ui.GetIO().MousePos[0] >= 0:
                self.ui.Spacing(70)
            if self.ui.Button("Moved button"):
                self.clicks += 1
            self.ui.End()
            self.ui.render()
        driver = testing.GuiDriver(self.ui, draw)
        driver.click("Moved button")
        self.assertEqual(1, self.clicks)

    def test_numeric_validity_reflects_draft_without_replacing_last_value(self):
        number = {"value": 10.0, "valid": True}
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Numbers", self.ui.ImGuiWindowFlags_NoDecoration)
            _, number["value"] = self.ui.InputDouble("Dimension", number["value"])
            number["valid"] = self.ui.IsItemValid()
            self.ui.End()
            self.ui.render()
        driver = testing.GuiDriver(self.ui, draw)
        driver.click("Dimension")
        # These drafts are permitted by the numeric input filter but cannot
        # yet be parsed as a number. Alphabetic input is intentionally ignored.
        for draft in ("-", ".", "-."):
            driver.text(draft, replace=True)
            self.assertFalse(number["valid"])
            self.assertEqual(10.0, number["value"])
        driver.text("12.5", replace=True)
        self.assertTrue(number["valid"])
        self.assertEqual(12.5, number["value"])

    def test_long_checkbox_labels_stay_bounded_and_keep_full_tooltip(self):
        from unittest.mock import patch
        label = "Body:[Mounting plate-Line2,Mounting plate-ExtrudeTop] "*3
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Selection", self.ui.ImGuiWindowFlags_NoDecoration)
            _, self.enabled = self.ui.Checkbox(label, self.enabled)
            self.ui.End()
            self.ui.render()
        driver = testing.GuiDriver(self.ui, draw)
        x, y, width, height = driver.control(label).rect
        self.assertLessEqual(x+width, self.window.width)
        with patch.object(self.ui, "SetTooltip", wraps=self.ui.SetTooltip) as tooltip:
            driver.move(x+width/2, y+height/2)
            tooltip.assert_called_with(label)
        driver.click(label)
        self.assertTrue(self.enabled)

    def test_tabs_are_visible_to_inspection_and_do_not_hide_same_named_buttons(self):
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.BeginTabBar("Pages")
            for label in ("First", "Second"):
                if self.ui.BeginTabItem(label):
                    if self.ui.Button(label):
                        self.clicks += 1
                    self.ui.EndTabItem()
            self.ui.EndTabBar()
            self.ui.End()
            self.ui.render()
        self.driver.draw = draw
        self.driver.frame()
        self.driver.click("First")
        self.assertEqual(1, self.clicks)
        self.driver.click("Second##tab")
        self.driver.click("Second")
        self.assertEqual(2, self.clicks)

    def test_combo_home_end_and_arrow_keys_select_offscreen_options(self):
        options = tuple(str(index) for index in range(20))
        self.choice = options[-1]
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            _, self.choice = self.ui.ComboBox("Choice", self.choice, options)
            self.ui.End()
            self.ui.render()
        self.driver.draw = draw
        self.driver.frame()
        self.driver.click("Choice")
        self.driver.key("HOME")
        self.driver.key("RETURN")
        self.assertEqual("0", self.choice)
        self.driver.click("Choice")
        self.driver.key("END")
        self.driver.key("UP")
        self.driver.key("RETURN")
        self.assertEqual("18", self.choice)

    def test_screenshots_include_controls_in_both_themes_without_desktop_capture(self):
        captures = []
        for theme in ("light", "dark"):
            self.ui.set_theme(theme)
            image = self.driver.screenshot()
            self.assertEqual((420, 240), (image.width, image.height))
            captures.append(image.get_bytes("RGBA", 420*4))
        self.assertNotEqual(captures[0], captures[1])
        self.assertTrue(any(captures[0]))

    def test_windows_multiline_messages_measure_and_render_without_glyph_errors(self):
        text = "Cannot apply\r\nPrevious geometry retained\rCheck the inputs"
        normalized = text.replace("\r\n", "\n").replace("\r", "\n")
        self.assertEqual(self.ui.CalcTextSize(normalized, wrap_width=200),
                         self.ui.CalcTextSize(text, wrap_width=200))
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.ValidationMessage(text)
            self.ui.End()
            self.ui.render()
        self.driver.draw = draw
        self.driver.screenshot()
        messages = [args[2] for kind, args, panel in self.ui._commands if kind == "text"]
        self.assertIn(normalized, messages)

    def test_drag_uses_window_mouse_events(self):
        self.slider = .2
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            _, self.slider = self.ui.SliderFloat("Amount", self.slider, width=200)
            self.ui.End()
            self.ui.render()
        self.driver.draw = draw
        self.driver.frame()
        x, y, width, height = self.driver.control("Amount").rect
        self.driver.drag((x+width*.2, y+height/2), (x+width*.8, y+height/2))
        self.assertAlmostEqual(.8, self.slider)
        self.assertFalse(any(self.ui.io.MouseDown))

    def test_scrolling_exposes_controls_and_hides_clipped_controls(self):
        def draw():
            self.window.clear()
            self.ui.new_frame()
            self.ui.SetNextWindowPos((0, 0), self.ui.ImGuiCond_Always)
            self.ui.SetNextWindowSize((420, 240), self.ui.ImGuiCond_Always)
            self.ui.Begin("Form", self.ui.ImGuiWindowFlags_NoDecoration)
            self.ui.BeginChild("Fields", height=170)
            for index in range(12):
                self.ui.InputText("Row "+str(index), "Value", width=200)
            self.ui.EndChild()
            self.ui.Button("Apply")
            self.ui.End()
            self.ui.render()
        self.driver.draw = draw
        self.driver.frame()
        with self.assertRaises(LookupError):
            self.driver.control("Row 11")
        self.driver.scroll(100, 80, -20)
        self.driver.click("Row 11")
        with self.assertRaises(LookupError):
            self.driver.control("Row 0")
        self.assertTrue(self.driver.control("Apply").enabled)
