"""pyimgui 2 (snake_case) plus the CamelCase names the sketch UI already uses."""


def import_imgui():
    try:
        import imgui
    except ImportError:
        raise ImportError(
            "The viewer needs pyglet and imgui. Install with:\n"
            "  pip install pyglet imgui[pyglet] numpy"
        )
    _patch(imgui)
    return imgui


def _patch(imgui):
    if getattr(imgui, "_camber_patched", False):
        return
    imgui._camber_patched = True
    imgui._camber_letters = set()

    imgui.ImVec2 = imgui.Vec2
    imgui.ImVec4 = imgui.Vec4
    for src, dst in _CONSTANTS:
        if not hasattr(imgui, src):
            setattr(imgui, src, getattr(imgui, dst))

    imgui.GetIO = lambda: _IOProxy(imgui.get_io())
    imgui.GetWindowDrawList = lambda: _DrawListProxy(imgui.get_window_draw_list())
    imgui.GetForegroundDrawList = lambda: _DrawListProxy(imgui.get_foreground_draw_list())
    imgui.GetBackgroundDrawList = lambda: _DrawListProxy(imgui.get_background_draw_list())
    imgui.Begin = _begin
    imgui.End = imgui.end
    imgui.Text = imgui.text
    imgui.TextUnformatted = imgui.text_unformatted
    imgui.Separator = imgui.separator
    imgui.SameLine = imgui.same_line
    imgui.Button = imgui.button
    imgui.Checkbox = imgui.checkbox
    imgui.Selectable = imgui.selectable
    imgui.InvisibleButton = _invisible_button
    imgui.InputDouble = imgui.input_double
    imgui.InputFloat = imgui.input_float
    imgui.PushItemWidth = imgui.push_item_width
    imgui.PopItemWidth = imgui.pop_item_width
    imgui.SetKeyboardFocusHere = imgui.set_keyboard_focus_here
    imgui.SetTooltip = imgui.set_tooltip
    imgui.IsItemHovered = imgui.is_item_hovered
    imgui.IsAnyItemHovered = imgui.is_any_item_hovered
    imgui.IsWindowHovered = imgui.is_window_hovered
    imgui.IsMouseDown = imgui.is_mouse_down
    imgui.IsMouseClicked = imgui.is_mouse_clicked
    imgui.IsMouseReleased = imgui.is_mouse_released
    imgui.IsMouseDoubleClicked = imgui.is_mouse_double_clicked
    imgui.IsKeyDown = _is_key_down
    imgui.GetCursorScreenPos = imgui.get_cursor_screen_pos
    imgui.GetContentRegionAvail = imgui.get_content_region_available
    imgui.CalcTextSize = imgui.calc_text_size
    imgui.GetFontSize = imgui.get_font_size
    imgui.GetFont = lambda: None
    imgui.GetColorU32 = _color_u32
    imgui.ColorConvertFloat4ToU32 = _color_u32
    imgui.SetClipboardText = imgui.set_clipboard_text
    imgui.GetWindowPos = imgui.get_window_position
    imgui.GetCursorPosX = imgui.get_cursor_pos_x
    imgui.TextWrapped = imgui.text_wrapped
    imgui.SetNextWindowPos = _set_next_window_pos
    imgui.SetNextWindowSize = _set_next_window_size
    imgui.SetNextWindowSizeConstraints = _set_next_window_size_constraints
    imgui.SetNextWindowBgAlpha = imgui.set_next_window_bg_alpha
    imgui.PushStyleVar = imgui.push_style_var
    imgui.PopStyleVar = imgui.pop_style_var
    imgui.PushStyleColor = _push_style_color
    imgui.PopStyleColor = imgui.pop_style_color
    imgui.PushTextWrapPos = imgui.push_text_wrap_pos
    imgui.PopTextWrapPos = imgui.pop_text_wrap_pos
    imgui.BeginTabBar = imgui.begin_tab_bar
    imgui.EndTabBar = imgui.end_tab_bar
    imgui.BeginTabItem = imgui.begin_tab_item
    imgui.EndTabItem = imgui.end_tab_item
    imgui.CollapsingHeader = imgui.collapsing_header


def apply_viewer_style(imgui):
    """Light widgets with a see-through window fill over the 3D view."""
    try:
        imgui.style_colors_light()
    except Exception:
        pass
    try:
        colors = imgui.get_style().colors
    except Exception:
        return
    alpha = 0.55
    for name in (
        "COLOR_WINDOW_BACKGROUND",
        "COLOR_POPUP_BACKGROUND",
        "COLOR_TITLE_BACKGROUND",
        "COLOR_TITLE_BACKGROUND_ACTIVE",
        "COLOR_TITLE_BACKGROUND_COLLAPSED",
        "COLOR_MENUBAR_BACKGROUND",
        "COLOR_TAB",
        "COLOR_TAB_ACTIVE",
        "COLOR_TAB_UNFOCUSED",
        "COLOR_TAB_UNFOCUSED_ACTIVE",
        "COLOR_FRAME_BACKGROUND",
    ):
        idx = getattr(imgui, name, None)
        if idx is None:
            continue
        try:
            r, g, b, _a = colors[int(idx)]
            colors[int(idx)] = (float(r), float(g), float(b), alpha)
        except Exception:
            pass


def _xy(p):
    if hasattr(p, "x"):
        return float(p.x), float(p.y)
    return float(p[0]), float(p[1])


def _begin(title, p_open=None, flags=0):
    import imgui
    if isinstance(p_open, int) and not isinstance(p_open, bool):
        flags = p_open
        p_open = None
    if p_open is None:
        return imgui.begin(title, False, flags)
    return imgui.begin(title, True, flags)


def _invisible_button(button_id, size, flags=0):
    import imgui
    w, h = _xy(size)
    return imgui.invisible_button(button_id, w, h, flags)


def _is_key_down(key):
    import imgui
    try:
        return bool(imgui.is_key_down(imgui.get_key_index(key)))
    except Exception:
        try:
            return bool(imgui.is_key_down(int(key)))
        except Exception:
            return False


def _color_u32(color, *rest):
    import imgui
    if rest:
        vals = (color,) + rest
    elif hasattr(color, "__len__"):
        vals = tuple(color)
    else:
        vals = (color,)
    r = float(vals[0]) if len(vals) > 0 else 1.0
    g = float(vals[1]) if len(vals) > 1 else 1.0
    b = float(vals[2]) if len(vals) > 2 else 1.0
    a = float(vals[3]) if len(vals) > 3 else 1.0
    return imgui.get_color_u32_rgba(r, g, b, a)


def _set_next_window_pos(pos, cond=0, pivot=None):
    import imgui
    x, y = _xy(pos)
    px, py = (0.0, 0.0) if pivot is None else _xy(pivot)
    imgui.set_next_window_position(x, y, cond, px, py)


def _set_next_window_size(size, cond=0):
    import imgui
    w, h = _xy(size)
    imgui.set_next_window_size(w, h, cond)


def _set_next_window_size_constraints(size_min, size_max):
    import imgui
    a, b = _xy(size_min)
    c, d = _xy(size_max)
    imgui.set_next_window_size_constraints((a, b), (c, d))


def _push_style_color(idx, *color):
    import imgui
    if len(color) == 1 and hasattr(color[0], "__len__"):
        c = color[0]
        imgui.push_style_color(idx, float(c[0]), float(c[1]), float(c[2]), float(c[3]) if len(c) > 3 else 1.0)
        return
    if len(color) >= 3:
        a = float(color[3]) if len(color) > 3 else 1.0
        imgui.push_style_color(idx, float(color[0]), float(color[1]), float(color[2]), a)
        return
    imgui.push_style_color(idx, 1.0, 1.0, 1.0, 1.0)


class _IOProxy(object):
    def __init__(self, io):
        self._io = io

    def __getattr__(self, name):
        io = self._io
        if hasattr(io, name):
            return getattr(io, name)
        snake = _camel_to_snake(name)
        if hasattr(io, snake):
            return getattr(io, snake)
        raise AttributeError(name)

    @property
    def WantCaptureMouse(self):
        return self._io.want_capture_mouse

    @property
    def WantCaptureKeyboard(self):
        return self._io.want_capture_keyboard

    @property
    def KeyShift(self):
        return self._io.key_shift

    @property
    def KeyCtrl(self):
        return self._io.key_ctrl

    @property
    def MouseDelta(self):
        return self._io.mouse_delta

    @property
    def MouseWheel(self):
        return self._io.mouse_wheel

    @property
    def DisplaySize(self):
        return self._io.display_size

    @property
    def MousePos(self):
        return self._io.mouse_pos

    @property
    def MouseDown(self):
        return self._io.mouse_down

    @property
    def MouseClicked(self):
        return getattr(self._io, "mouse_clicked", None)

    @property
    def MouseReleased(self):
        return getattr(self._io, "mouse_released", None)

    @property
    def MouseDoubleClicked(self):
        return getattr(self._io, "mouse_double_clicked", None)

    @property
    def KeysDown(self):
        return self._io.keys_down


class _DrawListProxy(object):
    def __init__(self, dl):
        self._dl = dl

    def __getattr__(self, name):
        return getattr(self._dl, name)

    def AddLine(self, a, b, col, th=1.0):
        x1, y1 = _xy(a)
        x2, y2 = _xy(b)
        self._dl.add_line(x1, y1, x2, y2, col, th)

    def AddCircle(self, c, r, col, segs=0, th=1.0):
        x, y = _xy(c)
        self._dl.add_circle(x, y, r, col, segs, th)

    def AddCircleFilled(self, c, r, col, segs=0):
        x, y = _xy(c)
        self._dl.add_circle_filled(x, y, r, col, segs)

    def AddRect(self, a, b, col, rounding=0.0, flags=0, th=1.0):
        x1, y1 = _xy(a)
        x2, y2 = _xy(b)
        self._dl.add_rect(x1, y1, x2, y2, col, rounding, flags, th)

    def AddRectFilled(self, a, b, col, rounding=0.0, flags=0):
        x1, y1 = _xy(a)
        x2, y2 = _xy(b)
        self._dl.add_rect_filled(x1, y1, x2, y2, col, rounding, flags)

    def AddTriangleFilled(self, a, b, c, col):
        x1, y1 = _xy(a)
        x2, y2 = _xy(b)
        x3, y3 = _xy(c)
        self._dl.add_triangle_filled(x1, y1, x2, y2, x3, y3, col)

    def AddText(self, *args):
        if len(args) >= 5:
            pos, col, text = args[2], args[3], args[4]
        elif len(args) >= 3:
            pos, col, text = args[0], args[1], args[2]
        else:
            return
        x, y = _xy(pos)
        self._dl.add_text(x, y, col, text)


def _camel_to_snake(name):
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i and (not name[i - 1].isupper() or (i + 1 < len(name) and name[i + 1].islower())):
            out.append("_")
        out.append(ch.lower())
    return "".join(out)


_CONSTANTS = (
    ("ImGuiCond_FirstUseEver", "FIRST_USE_EVER"),
    ("ImGuiCond_Always", "ALWAYS"),
    ("ImGuiHoveredFlags_AnyWindow", "HOVERED_ANY_WINDOW"),
    ("ImGuiCol_Text", "COLOR_TEXT"),
    ("ImGuiCol_TitleBg", "COLOR_TITLE_BACKGROUND"),
    ("ImGuiCol_TitleBgActive", "COLOR_TITLE_BACKGROUND_ACTIVE"),
    ("ImGuiCol_TitleBgCollapsed", "COLOR_TITLE_BACKGROUND_COLLAPSED"),
    ("ImGuiCol_Header", "COLOR_HEADER"),
    ("ImGuiCol_HeaderHovered", "COLOR_HEADER_HOVERED"),
    ("ImGuiCol_HeaderActive", "COLOR_HEADER_ACTIVE"),
    ("ImGuiCol_WindowBg", "COLOR_WINDOW_BACKGROUND"),
    ("ImGuiStyleVar_WindowPadding", "STYLE_WINDOW_PADDING"),
    ("ImGuiStyleVar_ItemSpacing", "STYLE_ITEM_SPACING"),
    ("ImGuiWindowFlags_NoTitleBar", "WINDOW_NO_TITLE_BAR"),
    ("ImGuiWindowFlags_NoResize", "WINDOW_NO_RESIZE"),
    ("ImGuiWindowFlags_NoMove", "WINDOW_NO_MOVE"),
    ("ImGuiWindowFlags_NoSavedSettings", "WINDOW_NO_SAVED_SETTINGS"),
    ("ImGuiWindowFlags_NoFocusOnAppearing", "WINDOW_NO_FOCUS_ON_APPEARING"),
    ("ImGuiWindowFlags_NoNav", "WINDOW_NO_NAV"),
    ("ImGuiWindowFlags_NoCollapse", "WINDOW_NO_COLLAPSE"),
    ("ImGuiWindowFlags_AlwaysVerticalScrollbar", "WINDOW_ALWAYS_VERTICAL_SCROLLBAR"),
    ("ImGuiWindowFlags_NoScrollbar", "WINDOW_NO_SCROLLBAR"),
    ("ImGuiWindowFlags_AlwaysAutoResize", "WINDOW_ALWAYS_AUTO_RESIZE"),
    ("ImGuiWindowFlags_NoInputs", "WINDOW_NO_INPUTS"),
    ("ImGuiWindowFlags_NoMouseInputs", "WINDOW_NO_MOUSE_INPUTS"),
    ("ImGuiWindowFlags_NoDecoration", "WINDOW_NO_DECORATION"),
    ("ImGuiWindowFlags_NoBackground", "WINDOW_NO_BACKGROUND"),
    ("ImGuiWindowFlags_NoScrollWithMouse", "WINDOW_NO_SCROLL_WITH_MOUSE"),
    ("ImGuiWindowFlags_NoClose", "WINDOW_NONE"),
    ("ImGuiKey_Enter", "KEY_ENTER"),
    ("ImGuiKey_Escape", "KEY_ESCAPE"),
    ("ImGuiKey_Backspace", "KEY_BACKSPACE"),
    ("ImGuiKey_Tab", "KEY_TAB"),
    ("ImGuiKey_Delete", "KEY_DELETE"),
    ("ImGuiKey_A", "KEY_A"),
    ("ImGuiKey_C", "KEY_C"),
    ("ImGuiKey_V", "KEY_V"),
    ("ImGuiKey_X", "KEY_X"),
    ("ImGuiKey_Y", "KEY_Y"),
    ("ImGuiKey_Z", "KEY_Z"),
)


def create_pyglet_renderer(window):
    """Modern pyglet event adapter without pyimgui's removed distutils import.

    The actual GUI drawing stays in pyimgui's existing GL renderer. This
    adapter translates pyglet events into the legacy pyimgui 2 IO structure.
    """
    import time
    import imgui
    from pyglet.window import key, mouse
    from imgui.integrations.opengl import ProgrammablePipelineRenderer

    class Renderer(ProgrammablePipelineRenderer):
        def __init__(self):
            super().__init__()
            self._last_time = time.perf_counter()
            pairs = [(key.TAB, imgui.KEY_TAB), (key.LEFT, imgui.KEY_LEFT_ARROW),
                     (key.RIGHT, imgui.KEY_RIGHT_ARROW), (key.UP, imgui.KEY_UP_ARROW),
                     (key.DOWN, imgui.KEY_DOWN_ARROW), (key.PAGEUP, imgui.KEY_PAGE_UP),
                     (key.PAGEDOWN, imgui.KEY_PAGE_DOWN), (key.HOME, imgui.KEY_HOME),
                     (key.END, imgui.KEY_END), (key.INSERT, imgui.KEY_INSERT),
                     (key.DELETE, imgui.KEY_DELETE), (key.BACKSPACE, imgui.KEY_BACKSPACE),
                     (key.SPACE, imgui.KEY_SPACE), (key.RETURN, imgui.KEY_ENTER),
                     (key.ESCAPE, imgui.KEY_ESCAPE), (key.NUM_ENTER, imgui.KEY_PAD_ENTER)]
            pairs += [(getattr(key, letter), getattr(imgui, "KEY_"+letter)) for letter in "ACVXYZ"]
            self._keys = dict(pairs)
            for index in self._keys.values():
                self.io.key_map[index] = index
            window.push_handlers(self)
            self.process_inputs()

        def process_inputs(self):
            now = time.perf_counter()
            self.io.delta_time = max(now-self._last_time, 1e-6)
            self._last_time = now
            w, h = window.get_size()
            fw, fh = window.get_framebuffer_size()
            self.io.display_size = w, h
            self.io.display_fb_scale = fw/max(w, 1), fh/max(h, 1)

        def on_mouse_motion(self, x, y, dx, dy):
            self.io.mouse_pos = x, window.height-y

        def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
            self.on_mouse_motion(x, y, dx, dy)

        def on_mouse_press(self, x, y, button, modifiers):
            self.on_mouse_motion(x, y, 0, 0)
            for flag, index in ((mouse.LEFT, 0), (mouse.RIGHT, 1), (mouse.MIDDLE, 2)):
                if button & flag:
                    self.io.mouse_down[index] = True

        def on_mouse_release(self, x, y, button, modifiers):
            for flag, index in ((mouse.LEFT, 0), (mouse.RIGHT, 1), (mouse.MIDDLE, 2)):
                if button & flag:
                    self.io.mouse_down[index] = False

        def on_mouse_scroll(self, x, y, sx, sy):
            self.io.mouse_wheel += sy
            self.io.mouse_wheel_horizontal += sx

        def _key(self, symbol, modifiers, down):
            if symbol in self._keys:
                self.io.keys_down[self._keys[symbol]] = down
            self.io.key_ctrl = bool(modifiers & key.MOD_CTRL)
            self.io.key_shift = bool(modifiers & key.MOD_SHIFT)
            self.io.key_alt = bool(modifiers & key.MOD_ALT)
            self.io.key_super = bool(modifiers & key.MOD_COMMAND)

        def on_key_press(self, symbol, modifiers):
            self._key(symbol, modifiers, True)

        def on_key_release(self, symbol, modifiers):
            self._key(symbol, modifiers, False)

        def on_text(self, text):
            for char in text:
                self.io.add_input_character(ord(char))

        def shutdown(self):
            window.remove_handlers(self)
            super().shutdown()

    return Renderer()
