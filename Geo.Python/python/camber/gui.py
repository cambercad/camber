"""Reusable pyglet UI with dockable panels and batched custom controls.

Coordinates are logical pixels, top-down. No application model or other UI
toolkit is required.
"""
import math
import sys
import time
from types import SimpleNamespace


def _inside(rect, point):
    x, y, w, h = rect
    return x <= point[0] < x + w and y <= point[1] < y + h


def _visible_color(color, visible):
    return color if visible else (color[0], color[1], color[2], 0)


class DockLayout:
    """Small retained dock tree: split regions, tab groups, and one viewport."""

    VIEWPORT = "__viewport__"

    def __init__(self):
        self.root = {"tabs": [self.VIEWPORT], "active": self.VIEWPORT}
        self.visible = {}
        self.rects = {}
        self.splitters = []
        self.viewport = (0, 0, 1, 1)
        self._size = None
        self._dirty = True

    def _leaf(self, title, node=None):
        node = self.root if node is None else node
        if "tabs" in node:
            return node if title in node["tabs"] else None
        return self._leaf(title, node["first"]) or self._leaf(title, node["second"])

    def add(self, title, side="left", target=VIEWPORT, fraction=.28):
        if self._leaf(title):
            return
        if side not in ("left", "right", "top", "bottom", "center"):
            raise ValueError("dock side must be left, right, top, bottom, or center")
        if target == self.VIEWPORT and side == "center":
            side = "right"
        self._insert(title, target, side, fraction)
        self.visible[title] = True
        self._dirty = True

    def _insert(self, title, target, side, fraction):
        leaf = self._leaf(target)
        if leaf is None:
            raise ValueError("unknown dock target: " + str(target))
        if side == "center":
            leaf["tabs"].append(title)
            leaf["active"] = title
            return
        old = dict(leaf)
        new = {"tabs": [title], "active": title}
        first = side in ("left", "top")
        leaf.clear()
        leaf.update(axis="x" if side in ("left", "right") else "y",
                    ratio=fraction if first else 1-fraction,
                    first=new if first else old, second=old if first else new)

    def _remove(self, node, title):
        if "tabs" in node:
            if title not in node["tabs"]:
                return node
            node["tabs"].remove(title)
            if not node["tabs"]:
                return None
            if node["active"] == title:
                node["active"] = node["tabs"][0]
            return node
        node["first"] = self._remove(node["first"], title)
        node["second"] = self._remove(node["second"], title)
        if node["first"] is None:
            return node["second"]
        if node["second"] is None:
            return node["first"]
        return node

    def move(self, title, target, side):
        if title == self.VIEWPORT or self._leaf(title) is None:
            return
        if self._leaf(title) is self._leaf(target) and side == "center":
            self._leaf(title)["active"] = title
            return
        if title == target:
            return
        self.root = self._remove(self.root, title)
        self._insert(title, target, side, .28)
        self._dirty = True

    def set_visible(self, title, visible):
        visible = bool(visible)
        if self.visible.get(title) != visible:
            self.visible[title] = visible
            self._dirty = True

    def _has_visible(self, node):
        if "tabs" in node:
            return any(t == self.VIEWPORT or self.visible.get(t, False) for t in node["tabs"])
        return self._has_visible(node["first"]) or self._has_visible(node["second"])

    def layout(self, width, height, bounds=None):
        width, height = max(1, int(width)), max(1, int(height))
        bounds = bounds or (0, 0, width, height)
        size = (width, height, bounds)
        if not self._dirty and size == self._size:
            return self.viewport
        self._size, self._dirty = size, False
        self.rects.clear()
        self.splitters.clear()

        def visit(node, rect):
            if "tabs" in node:
                tabs = [t for t in node["tabs"] if t == self.VIEWPORT or self.visible.get(t, False)]
                if not tabs:
                    return
                if node["active"] not in tabs:
                    node["active"] = tabs[0]
                for title in tabs:
                    self.rects[title] = (node, rect)
                if self.VIEWPORT in tabs:
                    self.viewport = rect
                return
            a, b = node["first"], node["second"]
            if not self._has_visible(a):
                visit(b, rect)
                return
            if not self._has_visible(b):
                visit(a, rect)
                return
            x, y, w, h = rect
            horizontal = node["axis"] == "x"
            total = w if horizontal else h
            split = max(120, min(total-120, round(total*node["ratio"]))) if total >= 245 else max(1, total//2)
            node["ratio"] = split / max(total, 1)
            if horizontal:
                ra, bar, rb = (x, y, split-2, h), (x+split-2, y, 4, h), (x+split+2, y, max(0,w-split-2), h)
            else:
                ra, bar, rb = (x, y, w, split-2), (x, y+split-2, w, 4), (x, y+split+2, w, max(0,h-split-2))
            self.splitters.append((node, bar, rect))
            visit(a, ra)
            visit(b, rb)

        visit(self.root, bounds)
        return self.viewport

    def drop_target(self, point):
        seen = set()
        for node, rect in self.rects.values():
            if id(node) in seen:
                continue
            seen.add(id(node))
            if not _inside(rect, point):
                continue
            x, y, w, h = rect
            distances = ((point[0]-x)/max(w,1), (x+w-point[0])/max(w,1),
                         (point[1]-y)/max(h,1), (y+h-point[1])/max(h,1))
            near = min(range(4), key=distances.__getitem__)
            side = ("left", "right", "top", "bottom")[near] if distances[near] < .3 else "center"
            if node["active"] == self.VIEWPORT and side == "center":
                side = "right"
            preview = {"left": (x,y,w*.28,h), "right": (x+w*.72,y,w*.28,h),
                       "top": (x,y,w,h*.28), "bottom": (x,y+h*.72,w,h*.28),
                       "center": rect}[side]
            return node["active"], side, preview
        return None


class _DrawList:
    def __init__(self, ui, foreground=False):
        self.ui, self.foreground = ui, foreground

    def _add(self, kind, *args):
        self.ui._emit(kind, *args, foreground=self.foreground,clip=not self.foreground)

    def AddLine(self, a, b, col, th=1):
        self._add("line", tuple(a), tuple(b), col, th)

    def AddRectFilled(self, a, b, col, rounding=0, flags=0):
        if rounding:
            self._add("rrect", tuple(a), tuple(b), rounding, col)
        else:
            self._add("rect", tuple(a), tuple(b), col)

    def AddRect(self, a, b, col, rounding=0, flags=0, th=1):
        x, y = a
        u, v = b
        for p, q in (((x, y), (u, y)), ((u, y), (u, v)),
                     ((u, v), (x, v)), ((x, v), (x, y))):
            self.AddLine(p, q, col, th)

    def AddCircle(self, c, r, col, segs=0, th=1):
        self._add("arc", tuple(c), r, col, th)

    def AddCircleFilled(self, c, r, col, segs=0):
        self._add("circle", tuple(c), r, col)

    def AddTriangleFilled(self, a, b, c, col):
        self._add("triangle", tuple(a), tuple(b), tuple(c), col)

    def AddText(self, *args):
        p, col, text = args[2:5] if len(args) >= 5 else args[:3]
        size = args[1] if len(args) >= 5 else self.ui._font_size()
        self._add("text", tuple(p), col, str(text), size)

    def AddTextAligned(self, pos, col, text, align="left"):
        self._add("text_align", tuple(pos), col, str(text), self.ui._font_size(), align)


class UI:
    # Compatibility surface for callers using immediate-mode controls.
    ImGuiCond_Always = 1
    ImGuiCond_FirstUseEver = 4
    ImGuiHoveredFlags_AnyWindow = 1
    ImGuiStyleVar_WindowPadding = 0
    ImGuiStyleVar_ItemSpacing = 1
    ImGuiCol_Text = 0
    ImGuiCol_TitleBg = 1
    ImGuiCol_TitleBgActive = 2
    ImGuiCol_TitleBgCollapsed = 3
    ImGuiCol_Header = 4
    ImGuiCol_HeaderHovered = 5
    ImGuiCol_HeaderActive = 6
    ImGuiCol_WindowBg = 7
    ImGuiWindowFlags_NoTitleBar = 1
    ImGuiWindowFlags_NoResize = 2
    ImGuiWindowFlags_NoMove = 4
    ImGuiWindowFlags_NoCollapse = 8
    ImGuiWindowFlags_AlwaysVerticalScrollbar = 16
    ImGuiWindowFlags_NoScrollbar = 32
    ImGuiWindowFlags_AlwaysAutoResize = 64
    ImGuiWindowFlags_NoInputs = 128
    ImGuiWindowFlags_NoMouseInputs = 128
    ImGuiWindowFlags_NoBackground = 256
    ImGuiWindowFlags_NoScrollWithMouse = 512
    ImGuiWindowFlags_NoDecoration = 1 | 2 | 8 | 32
    ImGuiWindowFlags_NoSavedSettings = 0
    ImGuiWindowFlags_NoFocusOnAppearing = 0
    ImGuiWindowFlags_NoNav = 0
    ImGuiWindowFlags_NoClose = 0

    def __init__(self, window):
        from pyglet.window import key
        self.window = window
        self.io = SimpleNamespace(DisplaySize=window.get_size(), MousePos=(-1, -1),
                                  MouseDelta=(0, 0), MouseWheel=0, MouseDown=[False]*3,
                                  MouseClicked=[False]*3, MouseReleased=[False]*3,
                                  MouseDoubleClicked=[False]*3, KeysDown={},
                                  KeyCtrl=False, KeyShift=False, KeyAlt=False,
                                  WantCaptureMouse=False, WantCaptureKeyboard=False)
        for name, symbol in (("Enter", key.RETURN), ("Escape", key.ESCAPE),
                             ("Backspace", key.BACKSPACE), ("Tab", key.TAB),
                             ("Delete", key.DELETE)):
            setattr(self, "ImGuiKey_"+name, symbol)
        for name in "ACVXYZ":
            setattr(self, "ImGuiKey_"+name, getattr(key, name))
        self.shortcut_letters = set()
        self.docks = DockLayout()
        self.insets = (0, 0, 0, 0)
        self.font_name = "Segoe UI" if sys.platform == "win32" else None
        self.theme_name = "light"
        self.theme = self._palette("light")
        self._cursor_name = None
        self._cursors = {}
        self._dock_tabs = []
        self._dock_press = None
        self._dock_drag = None
        self._split_drag = None
        self._panels = {}
        self._shown = []
        self._current = None
        self._commands = []
        self._foreground = []
        self._cache = []
        self._batch = None
        self._clip_class = None
        self._next = {}
        self._colors = []
        self._styles = []
        self._widths = []
        self._wrap = []
        self._tabs = {}
        self._tree_open = {}
        self._tabbar = None
        self._tab_hits = []
        self._editors = {}
        self._focus = None
        self._focus_visible = False
        self._keyboard_activate = None
        self._keyboard_delta = 0
        self._consumed_keys = set()
        self._focus_next = False
        self._focus_order = []
        self._nav_order = []
        self._combo_open = None
        self._combo_popup = None
        self._combo_press = None
        self._combo_scroll = 0
        self._combo_nav = 0
        self._combo_commit = False
        self._combo_keyboard = False
        self._combo_seen = False
        self._last = (0, 0, 0, 0)
        self._measurements = {}
        self._active = None
        self._drag = None
        self._last_click = (-1, 0, (-100, -100))
        self._press_pos = (-1, -1)
        self._press_panel = None
        self._release_pos = (-1, -1)
        self._closed = False
        self._drawlist = _DrawList(self)
        self._frontlist = _DrawList(self, True)
        window.push_handlers(self)

    @staticmethod
    def _palette(name):
        palettes = {
            "light": {
                "window": (244,244,244,255), "title": (242,240,240,255),
                "content": (255,255,255,255), "control": (248,248,248,255),
                "hover": (232,242,251,255), "selected": (108,179,231,255),
                "text": (31,31,31,255), "muted": (111,111,111,255),
                "border": (207,207,207,255), "accent": (0,120,215,255),
                "accent_hover": (0,103,188,255),
                "field": (255,255,255,255), "input_border": (137,137,137,255),
                "scroll_track": (248,248,248,255), "scroll_thumb": (178,178,178,255),
                "scroll_hover": (128,128,128,255), "check": (201,201,201,255),
                "group": (230,230,230,255), "check_mark": (255,255,255,255),
            },
            "dark": {
                "window": (37,37,37,255), "title": (31,31,31,255),
                "content": (28,28,28,255), "control": (48,48,48,255),
                "hover": (57,65,72,255), "selected": (31,95,145,255),
                "text": (223,223,223,255), "muted": (165,165,165,255),
                "border": (64,64,64,255), "accent": (77,170,252,255),
                "accent_hover": (103,188,255,255),
                "field": (39,39,39,255), "input_border": (124,124,124,255),
                "scroll_track": (37,37,37,255), "scroll_thumb": (105,105,105,255),
                "scroll_hover": (150,150,150,255), "check": (83,83,83,255),
                "group": (55,55,55,255), "check_mark": (255,255,255,255),
            },
        }
        if name not in palettes:
            raise ValueError("theme must be 'light' or 'dark'")
        return palettes[name].copy()

    def set_theme(self, name):
        """Switch between the built-in light and dark palettes at runtime."""
        if name == self.theme_name:
            return
        self.theme = self._palette(name)
        self.theme_name = name
        for editor in self._editors.values():
            editor.document.set_style(0, len(editor.document.text), {"color": self.theme["text"]})
            editor.layout.selection_color = self.theme["text"]
            editor.layout.selection_background_color = self.theme["selected"]

    def GetIO(self):
        return self.io

    @property
    def text_input_focused(self):
        return self._focus in self._editors

    def dock_panel(self, title, side="left", target=DockLayout.VIEWPORT, fraction=.28):
        self.docks.add(title, side, target, fraction)

    def set_dock_visible(self, title, visible):
        self.docks.set_visible(title, visible)

    @property
    def viewport_rect(self):
        return self._layout_docks()

    def _layout_docks(self):
        width, height = self.window.get_size()
        left, top, right, bottom = self.insets
        return self.docks.layout(width, height,
                                 (left, top, max(1, width-left-right), max(1, height-top-bottom)))

    def _panel_at(self, point):
        return next((p for p in reversed(self._shown)
                     if not p.flags & 128 and _inside(p.rect, point)), None)

    def _capture(self):
        self.io.WantCaptureMouse = (self._panel_at(self.io.MousePos) is not None or self._drag is not None
                                    or self._active is not None or self._dock_press is not None
                                    or self._split_drag is not None or any(_inside(bar, self.io.MousePos)
                                    for _, bar, _ in self.docks.splitters))
        self.io.WantCaptureKeyboard = self._focus is not None

    def _update_cursor(self):
        point = self.io.MousePos
        name = self.window.CURSOR_DEFAULT
        for node, bar, _ in self.docks.splitters:
            if _inside(bar, point):
                name = (self.window.CURSOR_SIZE_LEFT_RIGHT if node["axis"] == "x"
                        else self.window.CURSOR_SIZE_UP_DOWN)
                break
        else:
            panel = self._panel_at(point)
            if panel is not None:
                for eid in reversed(self._focus_order):
                    editor = self._editors.get(eid)
                    if editor and editor.panel is panel and _inside(editor.rect, point) and _inside(self._clip(panel), point):
                        name = self.window.CURSOR_TEXT
                        break
                else:
                    if not panel.docked and not panel.flags & 2:
                        x, y, w, h = panel.rect
                        if point[0] > x+w-14 and point[1] > y+h-14:
                            name = self.window.CURSOR_SIZE_UP_LEFT
        if name != self._cursor_name:
            if name not in self._cursors:
                self._cursors[name] = (self._text_cursor() if name == self.window.CURSOR_TEXT
                                       else self.window.get_system_mouse_cursor(name))
            self.window.set_mouse_cursor(self._cursors[name])
            self._cursor_name = name

    @staticmethod
    def _text_cursor():
        """A two-tone I-beam that remains legible on light and dark fields."""
        from pyglet.image import ImageData
        from pyglet.window import ImageMouseCursor

        rows = ("....WWW....", "...WBBBW...") + ("....WBW....",)*14 + (
            "...WBBBW...", "....WWW....")
        colors = {".": (0, 0, 0, 0), "W": (255, 255, 255, 255),
                  "B": (0, 0, 0, 255)}
        pixels = bytes(channel for row in rows for mark in row for channel in colors[mark])
        image = ImageData(len(rows[0]), len(rows), "RGBA", pixels, pitch=-4*len(rows[0]))
        return ImageMouseCursor(image, len(rows[0])//2, len(rows)//2, acceleration=True)

    def on_mouse_motion(self, x, y, dx, dy):
        self.io.MousePos = (x, self.window.height-y)
        if dx or dy:
            self._combo_keyboard = False
        a, b = self.io.MouseDelta
        self.io.MouseDelta = (a+dx, b-dy)
        self._capture()
        self._update_cursor()

    def on_mouse_leave(self, x, y):
        self.io.MousePos = (-1, -1)
        self._update_cursor()

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.on_mouse_motion(x, y, dx, dy)
        if self._split_drag:
            node, rect = self._split_drag
            coordinate = self.io.MousePos[0] if node["axis"] == "x" else self.io.MousePos[1]
            start = rect[0] if node["axis"] == "x" else rect[1]
            span = rect[2] if node["axis"] == "x" else rect[3]
            node["ratio"] = max(.05, min(.95, (coordinate-start)/max(span, 1)))
            self.docks._dirty = True
            self._layout_docks()
            return
        if self._dock_press and math.dist(self.io.MousePos, self._press_pos) > 5:
            self._dock_drag = self._dock_press
            return
        if self._drag:
            panel, mode = self._drag
            px, py, w, h = panel.rect
            if mode == "scroll":
                track, thumb = self._scroll_geometry(panel)
                panel.scroll = max(0,min(panel.max_scroll,
                    panel.scroll-dy*panel.max_scroll/max(1,track[3]-thumb[3])))
            else:
                panel.rect = (px, py, max(120, w+dx), max(80, h-dy)) if mode == "resize" else (px+dx, py-dy, w, h)
        editor = self._editors.get(self._focus)
        if editor and self.io.MouseDown[0]:
            editor.caret.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

    def on_mouse_press(self, x, y, button, modifiers):
        from pyglet.window import mouse
        self.on_mouse_motion(x, y, 0, 0)
        index = {mouse.LEFT: 0, mouse.RIGHT: 1, mouse.MIDDLE: 2}.get(button)
        if index is None:
            return
        self.io.MouseDown[index] = self.io.MouseClicked[index] = True
        now = time.monotonic()
        old_button, old_time, old_pos = self._last_click
        self.io.MouseDoubleClicked[index] = old_button == button and now-old_time < .4 and math.dist(old_pos, (x,y)) < 5
        self._last_click = (button, now, (x,y))
        if index == 0:
            self._focus_visible = False
            self._press_pos = self.io.MousePos
            if self._combo_popup:
                identity, anchor, popup, first = self._combo_popup
                if _inside(popup, self.io.MousePos):
                    self._combo_press = (identity, first + int((self.io.MousePos[1]-popup[1])//26))
                    return
                if not _inside(anchor, self.io.MousePos):
                    self._combo_open = self._combo_popup = None
            for node, bar, rect in self.docks.splitters:
                if _inside(bar, self._press_pos):
                    self._split_drag = (node, rect)
                    self._capture()
                    return
            for title, rect in reversed(self._dock_tabs):
                if _inside(rect, self._press_pos):
                    self._dock_press = title
                    self._capture()
                    return
            self._press_panel = self._panel_at(self.io.MousePos)
            hit = next((eid for eid in reversed(self._focus_order)
                        if _inside(self._editors[eid].rect, self.io.MousePos)
                        and _inside(self._clip(self._editors[eid].panel),self.io.MousePos)
                        and self._panel_at(self.io.MousePos) is self._editors[eid].panel), None)
            self._focus = hit
            if hit:
                self._editors[hit].original = self._editors[hit].document.text
                self._editors[hit].caret.on_mouse_press(x, y, button, modifiers)
            p = self._panel_at(self.io.MousePos)
            if p and not hit:
                px, py, w, h = p.rect
                track, thumb = self._scroll_geometry(p)
                if p.max_scroll and not p.flags & 32 and _inside(track, self.io.MousePos):
                    self._drag = (p, "scroll")
                    if not _inside(thumb, self.io.MousePos):
                        p.scroll = max(0, min(p.max_scroll,
                            (self.io.MousePos[1]-track[1]-thumb[3]/2)
                            *p.max_scroll/max(1,track[3]-thumb[3])))
                elif not p.flags & 2 and x > px+w-14 and self.io.MousePos[1] > py+h-14:
                    self._drag = (p, "resize")
                elif not p.flags & (1 | 4) and self.io.MousePos[1] < py+26 and px+24 < x < px+w-28:
                    self._drag = (p, "move")
        self._capture()

    def on_mouse_release(self, x, y, button, modifiers):
        from pyglet.window import mouse
        self.on_mouse_motion(x, y, 0, 0)
        index = {mouse.LEFT: 0, mouse.RIGHT: 1, mouse.MIDDLE: 2}.get(button)
        if index is not None:
            self.io.MouseDown[index] = False
            self.io.MouseReleased[index] = True
        if index == 0:
            self._release_pos = self.io.MousePos
            if self._combo_press:
                self._capture()
                return
            if self._dock_press:
                if self._dock_drag:
                    target = self.docks.drop_target(self._release_pos)
                    if target:
                        self.docks.move(self._dock_press, target[0], target[1])
                else:
                    leaf = self.docks._leaf(self._dock_press)
                    if leaf and any(t == self._dock_press and _inside(r, self._release_pos)
                                    for t, r in self._dock_tabs):
                        leaf["active"] = self._dock_press
                self._dock_press = self._dock_drag = self._split_drag = None
                self._capture()
                return
            self._split_drag = None
            self._drag = None
            for identity,label,rect,panel in self._tab_hits:
                if _inside(rect,self._press_pos) and _inside(rect,self._release_pos) and self._panel_at(self._release_pos) is panel:
                    self._tabs[identity] = label
                    panel.scroll = 0
        self._capture()

    def on_mouse_scroll(self, x, y, sx, sy):
        self.on_mouse_motion(x, y, 0, 0)
        if self._combo_popup and _inside(self._combo_popup[2], self.io.MousePos):
            self._combo_scroll = max(0, self._combo_scroll-int(sy))
            visible = max(1, self._combo_popup[2][3]//26)
            self._combo_nav = max(self._combo_scroll,
                                  min(self._combo_nav, self._combo_scroll+visible-1))
            return
        p = self._panel_at(self.io.MousePos)
        if p and not p.flags & 512:
            p.scroll = max(0, min(p.max_scroll, p.scroll-sy*32))
        else:
            self.io.MouseWheel += sy

    def on_key_press(self, symbol, modifiers):
        from pyglet.window import key
        self.io.KeysDown[symbol] = True
        self.io.KeyCtrl = bool(modifiers & key.MOD_CTRL)
        self.io.KeyShift = bool(modifiers & key.MOD_SHIFT)
        self.io.KeyAlt = bool(modifiers & key.MOD_ALT)
        editor = self._editors.get(self._focus)
        if self._combo_open is not None and editor is None:
            if symbol == key.ESCAPE:
                self._combo_open = self._combo_popup = None
                self._consumed_keys.add(symbol)
                return
            if symbol == key.TAB:
                self._combo_open = self._combo_popup = None
            if symbol in (key.UP, key.DOWN):
                self._combo_nav += -1 if symbol == key.UP else 1
                self._combo_keyboard = True
                self._consumed_keys.add(symbol)
                return
            if symbol == key.RETURN:
                self._combo_commit = True
                self._consumed_keys.add(symbol)
                return
        if symbol == key.TAB and self._nav_order:
            step = -1 if self.io.KeyShift else 1
            index = self._nav_order.index(self._focus) if self._focus in self._nav_order else (-1 if step == 1 else 0)
            self._focus = self._nav_order[(index+step) % len(self._nav_order)]
            self._focus_visible = True
            target = self._editors.get(self._focus)
            if target:
                target.original = target.document.text
            self._consumed_keys.add(symbol)
            self.io.KeysDown[symbol] = False
            self._capture()
            return
        if editor is None and self._focus is not None:
            if symbol in (key.RETURN, key.SPACE):
                self._keyboard_activate = self._focus
            elif symbol in (key.LEFT, key.RIGHT):
                self._keyboard_delta = -1 if symbol == key.LEFT else 1
            elif symbol == key.ESCAPE:
                self._focus = None
            else:
                return
            self._consumed_keys.add(symbol)
            self.io.KeysDown[symbol] = False
            self._capture()
            return
        if editor:
            self._consumed_keys.add(symbol)
            self.io.KeysDown[symbol] = False
            self.shortcut_letters.discard(key.symbol_string(symbol))
            if symbol == key.ESCAPE:
                editor.document.text = editor.original
                editor.commit = True
                self._focus = None
            elif self.io.KeyCtrl and symbol in (key.Z,key.Y):
                source,target = (editor.redo,editor.undo) if symbol == key.Y or self.io.KeyShift else (editor.undo,editor.redo)
                if source:
                    target.append(self._snapshot(editor))
                    text,pos,mark = source.pop()
                    editor.document.text = text
                    editor.caret.position,editor.caret.mark = pos,mark
                    editor.commit = True
            elif self.io.KeyCtrl and symbol in (key.A, key.C, key.X, key.V):
                caret, doc = editor.caret, editor.document
                if symbol == key.A:
                    caret.mark, caret.position = 0, len(doc.text)
                else:
                    a, b = sorted((caret.position, caret.mark if caret.mark is not None else caret.position))
                    if symbol in (key.C, key.X):
                        self.window.set_clipboard_text(doc.text[a:b])
                        if symbol == key.X:
                            self._edit(editor,caret.on_text,"")
                    else:
                        self._edit(editor,caret.on_text,self.window.get_clipboard_text())
        self._capture()

    def on_key_release(self, symbol, modifiers):
        self.io.KeysDown[symbol] = False
        self._consumed_keys.discard(symbol)
        from pyglet.window import key
        self.io.KeyCtrl = bool(modifiers & key.MOD_CTRL)
        self.io.KeyShift = bool(modifiers & key.MOD_SHIFT)
        self.io.KeyAlt = bool(modifiers & key.MOD_ALT)

    def on_deactivate(self):
        self.io.KeysDown.clear()
        self._consumed_keys.clear()
        self.io.MouseDown[:] = [False]*3
        self.shortcut_letters.clear()
        self._drag = self._active = None
        self._dock_press = self._dock_drag = self._split_drag = None
        self._capture()

    def on_text(self, text):
        editor = self._editors.get(self._focus)
        if editor:
            if text in ("\r", "\n"):
                if editor.multiline:
                    self._edit(editor, editor.caret.on_text, "\n")
                else:
                    editor.commit = True
                    self._focus = None
            elif text != "\t":
                self._edit(editor,editor.caret.on_text,text)
        self._capture()

    def on_text_motion(self, motion):
        editor = self._editors.get(self._focus)
        if editor:
            self._edit(editor,editor.caret.on_text_motion,motion)

    def on_text_motion_select(self, motion):
        editor = self._editors.get(self._focus)
        if editor:
            editor.caret.on_text_motion_select(motion)

    @staticmethod
    def _snapshot(editor):
        return (editor.document.text,editor.caret.position,editor.caret.mark)

    def _edit(self, editor, callback, *args):
        before = self._snapshot(editor)
        callback(*args)
        if editor.document.text != before[0]:
            editor.undo.append(before)
            del editor.undo[:-100]
            editor.redo.clear()
            editor.commit = True

    def new_frame(self):
        self.io.DisplaySize = self.window.get_size()
        self._capture()
        self._shown = []
        self._commands = []
        self._foreground = []
        self._focus_order = []
        self._nav_order = []
        self._tab_hits = []
        self._current = None
        self._next.clear()
        self._colors.clear()
        self._styles.clear()
        self._widths.clear()
        self._wrap.clear()
        self._tabbar = None
        self._combo_seen = False
        self._combo_popup = None

    def _emit(self, kind, *args, foreground=False, clip=True):
        command = (kind, args, self._current if clip else None)
        (self._foreground if foreground else self._commands).append(command)

    def _style(self, index, default):
        return next((value for i, value in reversed(self._styles) if i == index), default)

    def _color(self, index, default):
        return next((value for i, value in reversed(self._colors) if i == index), default)

    def _font_size(self):
        return 14 * (self._current.scale if self._current else 1)

    def SetNextWindowPos(self, pos, cond=0, pivot=(0,0)):
        self._next["pos"] = (tuple(pos), cond, tuple(pivot))

    def SetNextWindowSize(self, size, cond=0):
        self._next["size"] = (tuple(size), cond)

    def SetNextWindowSizeConstraints(self, minimum, maximum):
        self._next["limits"] = (tuple(minimum), tuple(maximum))

    def SetNextWindowBgAlpha(self, alpha):
        self._next["alpha"] = alpha

    def Begin(self, title, opened=None, flags=0):
        if isinstance(opened, int) and not isinstance(opened, bool):
            flags, opened = opened, None
        fresh = title not in self._panels
        p = self._panels.setdefault(title, SimpleNamespace(rect=(12,12,320,200), scroll=0,
                    max_scroll=0, collapsed=False, scale=1, flags=flags))
        if not hasattr(p, "expanded_height"):
            p.expanded_height = p.rect[3]
        self._layout_docks()
        dock = self.docks.rects.get(title)
        if dock is None and self.docks._leaf(title):
            p.docked = p.hidden = True
            self._current = p
            self._next.clear()
            return (False, opened) if opened is not None else False
        if dock:
            node, rect = dock
            x, y, w, h = rect
            p.rect, p.flags, p.docked = rect, flags | 2 | 4, True
            p.hidden = node["active"] != title
            p.padding, p.spacing = self._style(0, (10, 8)), self._style(1, (8, 6))
            p.title_h = p.clip_top = 31
            p.cursor = (x+p.padding[0], y+p.title_h+p.padding[1]-p.scroll)
            p.bottom, p.right = p.cursor[1], p.cursor[0]
            p.previous = None
            self._current = p
            if not p.hidden:
                self._shown.append(p)
            self._next.clear()
            return (not p.hidden, opened) if opened is not None else not p.hidden
        p.docked = p.hidden = False
        x,y,w,h = p.rect
        size, cond = self._next.get("size", ((w,h), 4))
        auto_w = auto_h = False
        if fresh or cond in (0,1):
            auto_w, auto_h = size[0] == 0, size[1] == 0
            w,h = size[0] or w, size[1] or h
        p.auto_w = auto_w or bool(flags & 64 and "size" not in self._next)
        p.auto_h = auto_h or bool(flags & 64 and "size" not in self._next)
        p.limits = self._next.get("limits", ((0,0),(float("inf"),float("inf"))))
        pos, cond, pivot = self._next.get("pos", ((x,y),4,(0,0)))
        if fresh or cond in (0,1):
            x,y = pos[0]-w*pivot[0], pos[1]-h*pivot[1]
        p.pivot, p.anchor = pivot, pos
        p.rect = (x,y,w,min(h,max(80,self.window.height-y-10)))
        p.flags = flags
        p.padding = self._style(0, (10,8))
        p.spacing = self._style(1, (8,6))
        p.title_h = 0 if flags & 1 else 26
        p.clip_top = p.title_h
        p.cursor = (x+p.padding[0],y+p.title_h+p.padding[1]-p.scroll)
        p.bottom, p.right = p.cursor[1], p.cursor[0]
        p.previous = None
        self._current = p
        self._shown.append(p)
        bg = list(self._color(7,self.theme["window"]))
        if "alpha" in self._next:
            bg[3] = round(self._next["alpha"]*255)
        self._next.clear()
        p.background_index = len(self._commands)
        if not flags & 256:
            self._emit("rect", (x,y), (x+w,y+h), tuple(bg), clip=False)
        if p.title_h:
            self._emit("rect", (x,y),(x+w,y+26), self._color(1,self.theme["title"]), clip=False)
            if not flags & 8:
                if self._button_at((x+3,y+3,20,20), title+"/collapse", "▸" if p.collapsed else "▾",
                                   clip_content=False, tab_stop=False):
                    p.collapsed = not p.collapsed
                    if p.collapsed:
                        p.expanded_height = p.rect[3]
                    else:
                        p.rect = (x, y, w, min(p.expanded_height,
                                              max(80, self.window.height-y-10)))
            self._emit("text_align", (x+(26 if not flags & 8 else 8),y+13), self.theme["text"], title.split("##")[0],14,"left",clip=False)
            if opened is not None:
                if self._button_at((x+w-26,y+3,22,20), title+"/close", "×",
                                   clip_content=False, tab_stop=False):
                    opened = False
            if not flags & 8 and self.io.MouseDoubleClicked[0] and _inside((x+26,y,w-54,26),self.io.MousePos):
                p.collapsed = not p.collapsed
                if p.collapsed:
                    p.expanded_height = p.rect[3]
                else:
                    p.rect = (x, y, w, min(p.expanded_height,
                                          max(80, self.window.height-y-10)))
        return (not p.collapsed and opened is not False, opened) if opened is not None else not p.collapsed

    def End(self):
        p = self._current
        if p is None:
            raise RuntimeError("End without Begin")
        if p.docked:
            if not p.hidden:
                x, y, w, h = p.rect
                p.max_scroll = max(0, p.bottom+p.scroll-y+p.padding[1]-h)
                p.scroll = min(p.scroll, p.max_scroll)
                self._draw_scrollbar(p)
            self._current = None
            return
        x,y,w,h = p.rect
        content_h = p.bottom + p.scroll - y + p.padding[1]
        if p.auto_h:
            h = min(max(24,content_h),max(24,self.window.height-y-10))
        if p.auto_w:
            w = max(40,p.right-x+p.padding[0])
        lo, hi = p.limits
        w, h = max(lo[0],min(w,hi[0])), max(lo[1],min(h,hi[1]))
        if p.auto_w or p.auto_h:
            x,y = p.anchor[0]-w*p.pivot[0],p.anchor[1]-h*p.pivot[1]
        dx,dy = x-p.rect[0],y-p.rect[1]
        if dx or dy:
            for i in range(p.background_index+1,len(self._commands)):
                kind,args,clip = self._commands[i]
                count = {"line":2,"rect":2,"rrect":2,"triangle":3,"circle":1,"arc":1,"text":1,"text_align":1}.get(kind,0)
                args = tuple((a[0]+dx,a[1]+dy) if j<count else a for j,a in enumerate(args))
                self._commands[i] = kind,args,clip
        p.rect = (x,y,w,26 if p.collapsed else h)
        if not p.collapsed:
            p.expanded_height = p.rect[3]
        # The background precedes content, but its final size is known only now.
        if not p.flags & 256:
            kind,args,clip = self._commands[p.background_index]
            self._commands[p.background_index] = (kind, ((x,y),(x+w,y+p.rect[3]),args[2]),clip)
        p.max_scroll = max(0,content_h-h)
        p.scroll = min(p.scroll,p.max_scroll)
        self._draw_scrollbar(p)
        self._current = None

    def _scroll_geometry(self, p):
        x, y, w, h = p.rect
        track = (x+w-13, y+p.title_h+5, 9, max(1,h-p.title_h-10))
        thumb_h = max(26, track[3]*track[3]/max(1,track[3]+p.max_scroll))
        thumb_h = min(track[3], thumb_h)
        top = track[1]+(track[3]-thumb_h)*p.scroll/max(1,p.max_scroll)
        return track, (track[0]+1, top, 7, thumb_h)

    def _draw_scrollbar(self, p):
        if not p.max_scroll or p.flags & 32:
            return
        track, thumb = self._scroll_geometry(p)
        x, y, w, h = track
        self._emit("rrect", (x,y), (x+w,y+h), 4, self.theme["scroll_track"], clip=False)
        x, y, w, h = thumb
        color = self.theme["scroll_hover"] if _inside(track,self.io.MousePos) else self.theme["scroll_thumb"]
        self._emit("rrect", (x,y), (x+w,y+h), 3, color, clip=False)

    def _clip(self, p):
        if p is None:
            return (0,0,*self.io.DisplaySize)
        x,y,w,h = p.rect
        return (x,y+p.clip_top,max(0,w-(14 if p.max_scroll and not p.flags & 32 else 0)),max(0,h-p.clip_top))

    def _hover(self, rect):
        p = self._current
        return _inside(rect,self.io.MousePos) and _inside(self._clip(p),self.io.MousePos) and self._panel_at(self.io.MousePos) is p

    def _item(self, w, h):
        p = self._current
        if p is None:
            # Viewer callbacks may draw a single unpanelled diagnostic label.
            self.Begin("##diagnostic", self.ImGuiWindowFlags_NoDecoration | self.ImGuiWindowFlags_NoInputs)
            p = self._current
        x,y = p.cursor
        self._last = (x,y,w,h)
        p.previous = self._last
        p.right,p.bottom = max(p.right,x+w),max(p.bottom,y+h)
        p.cursor = (p.rect[0]+p.padding[0],p.bottom+p.spacing[1])
        return self._last

    def SameLine(self, offset=None):
        p = self._current
        x,y,w,h = p.previous
        p.cursor = (p.rect[0]+offset if offset is not None else x+w+p.spacing[0],y)

    def GetCursorScreenPos(self):
        return self._current.cursor

    def GetCursorPosX(self):
        return self._current.cursor[0]-self._current.rect[0]

    def GetContentRegionAvail(self):
        p = self._current
        return (max(0,p.rect[0]+p.rect[2]-p.cursor[0]-p.padding[0]),
                max(0,p.rect[1]+p.rect[3]-p.cursor[1]-p.padding[1]))

    def GetWindowPos(self):
        return self._current.rect[:2]

    def _button_at(self, rect, identity, label="", selected=False, invisible=False,
                   clip_content=True, centered=False, bordered=False, flat=False,
                   primary=False, enabled=True, tab_stop=True):
        if enabled and tab_stop:
            self._nav_order.append(identity)
        hovered = enabled and (self._hover(rect) if clip_content else _inside(rect,self.io.MousePos))
        if enabled and hovered and self.io.MouseClicked[0] and _inside(rect,self._press_pos) and self._current is self._press_panel:
            self._active = identity
            if tab_stop:
                self._focus = identity
        clicked = enabled and self.io.MouseReleased[0] and self._active == identity and hovered and self._current is self._press_panel
        if enabled and self._keyboard_activate == identity:
            clicked = True
            self._keyboard_activate = None
        if not invisible:
            x,y,w,h = rect
            color = (self.theme["accent_hover"] if hovered else self.theme["accent"]) if primary else self._color(
                6 if selected else 5 if hovered else 4,
                self.theme["selected"] if selected else self.theme["hover"] if hovered else
                self.theme["content"] if flat else self.theme["control"])
            if bordered:
                self._emit("rrect",(x,y),(x+w,y+h),3,
                           self.theme["accent"] if primary else self.theme["border"],clip=clip_content)
                self._emit("rrect",(x+1,y+1),(x+w-1,y+h-1),2,color,clip=clip_content)
            else:
                self._emit("rect",(x,y),(x+w,y+h),color,clip=clip_content)
            self._emit("text_align",(x+w/2 if centered else x+8,y+h/2),
                       (255,255,255,255) if primary else self._color(0,self.theme["text"] if enabled else self.theme["muted"]),
                       label,self._font_size(),
                       "center" if centered else "left",clip=clip_content)
        if enabled and self._focus_visible and self._focus == identity:
            x,y,w,h = rect
            for a,b in (((x,y),(x+w,y)), ((x+w,y),(x+w,y+h)),
                        ((x+w,y+h),(x,y+h)), ((x,y+h),(x,y))):
                self._emit("line", a, b, self.theme["accent"], 1, clip=clip_content)
        return clicked

    def Button(self, label, width=0, height=0, *, primary=False, enabled=True):
        if isinstance(width,(tuple,list)):
            width,height = width
        shown = label.split("##")[0]
        rect = self._item(width or self.CalcTextSize(shown)[0]+16,height or self._font_size()+10)
        return self._button_at(rect,self._id(label),shown,centered=True,bordered=True,
                               primary=primary,enabled=enabled)

    def _tab_at(self, rect, identity, label, selected, clip_content=True):
        clicked = self._button_at(rect, identity, invisible=True, clip_content=clip_content)
        x, y, w, h = rect
        hovered = self._hover(rect) if clip_content else _inside(rect, self.io.MousePos)
        self._emit("rect", (x,y), (x+w,y+h),
                   _visible_color(self.theme["hover"], hovered and not selected),
                   clip=clip_content)
        self._emit("rect", (x,y+h-2), (x+w,y+h),
                   _visible_color(self.theme["accent"], selected),
                   clip=clip_content)
        self._emit("text_align", (x+w/2,y+h/2),
                   self.theme["text"] if selected else self.theme["muted"],
                   label, self._font_size(), "center", clip=clip_content)
        return clicked

    def TabButton(self, label, selected=False, width=0, height=26):
        shown = label.split("##")[0]
        rect = self._item(width or self.CalcTextSize(shown)[0]+20, height)
        return self._tab_at(rect, self._id(label), shown, selected)

    def InvisibleButton(self, label, size, flags=0):
        return self._button_at(self._item(*size),self._id(label),invisible=True)

    def Checkbox(self, label, value):
        shown = label.split("##")[0]
        rect = self._item(self.CalcTextSize(shown)[0]+28,24)
        changed = self._button_at(rect,self._id(label),invisible=True)
        x,y,w,h = rect
        border = self.theme["accent"] if self._hover(rect) else self.theme["input_border"]
        self._emit("rrect", (x,y+4), (x+16,y+20), 2, border)
        self._emit("rrect", (x+1,y+5), (x+15,y+19), 1,
                   self.theme["check"] if value else self.theme["field"])
        self._emit("text_align", (x+8,y+12),
                   _visible_color(self.theme["check_mark"], value),
                   "✓", 12, "center")
        self._emit("text_align", (x+24,y+12), self.theme["text"], shown, self._font_size(), "left")
        return changed, not value if changed else value

    def RadioButton(self, label, selected=False):
        shown = label.split("##")[0]
        rect = self._item(self.CalcTextSize(shown)[0]+28, 24)
        clicked = self._button_at(rect, self._id(label), invisible=True)
        x, y, _, _ = rect
        self._emit("circle", (x+8,y+12), 8,
                   self.theme["accent"] if self._hover(rect) else self.theme["input_border"])
        self._emit("circle", (x+8,y+12), 7, self.theme["field"])
        self._emit("circle", (x+8,y+12), 4,
                   _visible_color(self.theme["accent"], selected))
        self._emit("text_align", (x+24,y+12), self.theme["text"], shown, self._font_size(), "left")
        return clicked

    def Switch(self, label, value):
        shown = label.split("##")[0]
        rect = self._item(self.CalcTextSize(shown)[0]+44, 24)
        changed = self._button_at(rect, self._id(label), invisible=True)
        x, y, _, _ = rect
        state = not value if changed else value
        self._emit("rrect", (x,y+5), (x+30,y+19), 7,
                   self.theme["accent"] if state else self.theme["input_border"])
        self._emit("circle", (x+22 if state else x+8,y+12), 5,
                   self.theme["check_mark"])
        self._emit("text_align", (x+38,y+12), self.theme["text"], shown,
                   self._font_size(), "left")
        return changed, state

    def TreeNode(self, label, depth=0, selected=False, *, leaf=False, default_open=False):
        """Draw one tree row; return ``(clicked, expanded)`` for caller-driven children."""
        identity = self._id(label)
        expanded = self._tree_open.setdefault(identity, default_open)
        rect = self._item(self.GetContentRegionAvail()[0], 24)
        clicked = self._button_at(rect, identity, invisible=True)
        if clicked and not leaf:
            expanded = self._tree_open[identity] = not expanded
        x, y, w, h = rect
        self._emit("rect", (x,y), (x+w,y+h),
                   self.theme["selected"] if selected else
                   _visible_color(self.theme["hover"], self._hover(rect)))
        indent = x+max(0,depth)*16
        if not leaf:
            self._emit("text_align", (indent+8,y+h/2), self.theme["muted"],
                       "▾" if expanded else "▸", self._font_size(), "center")
        self._emit("text_align", (indent+20,y+h/2), self.theme["text"],
                   label.split("##")[0], self._font_size(), "left")
        return clicked, expanded

    def ListBox(self, label, options, selected=-1, width=0):
        """A simple list; the containing panel provides scrolling."""
        width = width or min(240, self.GetContentRegionAvail()[0])
        spacing = self._current.spacing
        self._current.spacing = (spacing[0], 0)
        try:
            for index, option in enumerate(options):
                rect = self._item(width, 26)
                shown = self._fit_text(str(option), width-16)
                if self._button_at(rect, (self._id(label), index), shown, index == selected, flat=True):
                    selected = index
        finally:
            self._current.spacing = spacing
        return selected

    def ComboBox(self, label, value, options, width=0):
        """Return ``(changed, value)`` with a clipped, keyboard-accessible popup."""
        options = tuple(options)
        if not options:
            raise ValueError("combo box needs at least one option")
        identity = self._id(label)
        rect = self._item(width or min(240, self.GetContentRegionAvail()[0]), 26)
        if self._button_at(rect, identity, invisible=True):
            if self._combo_open == identity:
                self._combo_open = self._combo_popup = None
            else:
                self._focus = None
                self._combo_open = identity
                self._combo_scroll = 0
                self._combo_nav = options.index(value) if value in options else 0
                self._combo_keyboard = False
        x, y, w, h = rect
        active = self._combo_open == identity
        border = self.theme["accent"] if active or self._hover(rect) else self.theme["input_border"]
        self._emit("rrect", (x,y), (x+w,y+h), 2, border)
        self._emit("rrect", (x+1,y+1), (x+w-1,y+h-1), 1, self.theme["field"])
        self._emit("text_align", (x+8,y+h/2), self.theme["text"],
                   self._fit_text(str(value), w-34), self._font_size(), "left")
        self._emit("text_align", (x+w-14,y+h/2), self.theme["muted"], "▾", 14, "center")
        if not active:
            return False, value
        self._combo_seen = True
        below = self.window.height-y-h-4
        above = y-4
        visible = min(len(options), 8, max(1, int(max(below,above)//26)))
        self._combo_nav = max(0, min(len(options)-1, self._combo_nav))
        self._combo_scroll = max(0, min(len(options)-visible, self._combo_scroll))
        if self._combo_nav < self._combo_scroll:
            self._combo_scroll = self._combo_nav
        if self._combo_nav >= self._combo_scroll+visible:
            self._combo_scroll = self._combo_nav-visible+1
        pop_h = visible*26
        pop_y = y+h+2 if below >= pop_h else y-pop_h-2
        pop_y = max(0, min(self.window.height-pop_h, pop_y))
        pop_x = max(0, min(self.window.width-w, x))
        popup = (pop_x, pop_y, w, pop_h)
        self._combo_popup = (identity, rect, popup, self._combo_scroll)
        self._emit("rrect", (pop_x-1,pop_y-1), (pop_x+w+1,pop_y+pop_h+1), 2,
                   self.theme["border"], foreground=True, clip=False)
        self._emit("rect", (pop_x,pop_y), (pop_x+w,pop_y+pop_h), self.theme["field"], foreground=True, clip=False)
        for index in range(self._combo_scroll, self._combo_scroll+visible):
            row = (pop_x, pop_y+(index-self._combo_scroll)*26, w, 26)
            if not self._combo_keyboard and _inside(row, self.io.MousePos):
                self._combo_nav = index
            if index == self._combo_nav:
                self._emit("rect", (row[0],row[1]), (row[0]+w,row[1]+26),
                           self.theme["selected"], foreground=True, clip=False)
            self._emit("text_align", (pop_x+8,row[1]+13), self.theme["text"],
                       self._fit_text(str(options[index]), w-16),
                       self._font_size(), "left", foreground=True, clip=False)
        choice = None
        if self._combo_commit:
            choice = self._combo_nav
        elif self.io.MouseReleased[0] and self._combo_press and self._combo_press[0] == identity:
            index = self._combo_press[1]
            row = (pop_x, pop_y+(index-self._combo_scroll)*26, w, 26)
            if 0 <= index < len(options) and _inside(row, self.io.MousePos):
                choice = index
        self._combo_commit = False
        self._combo_press = None
        if choice is not None:
            self._combo_open = self._combo_popup = None
            return options[choice] != value, options[choice]
        return False, value

    def Selectable(self, label, selected=False):
        rect = self._item(self.GetContentRegionAvail()[0],24)
        return self._button_at(rect,self._id(label),label.split("##")[0],selected,flat=True)

    def DataGrid(self, columns, rows, selected=-1, widths=None, group_by=None):
        """Draw a compact scrollable grid in the current panel; return the selected row index."""
        if not columns:
            return selected
        width = self.GetContentRegionAvail()[0]
        weights = widths or [1]*len(columns)
        if len(weights) != len(columns) or any(weight <= 0 for weight in weights):
            raise ValueError("grid widths must be positive and match columns")
        sizes = [width*weight/sum(weights) for weight in weights]
        spacing = self._current.spacing
        self._current.spacing = (spacing[0], 0)
        x, y, _, h = self._item(width, 25)
        self._emit("rect", (x,y), (x+width,y+h), self.theme["field"])
        self._emit("line", (x,y+h), (x+width,y+h), self.theme["border"], 1)
        pos = x
        for label, size in zip(columns, sizes):
            self._emit("text_align", (pos+8,y+h/2), self.theme["muted"], str(label), 12, "left")
            pos += size
            self._emit("line", (pos,y+2), (pos,y+h-2), self.theme["border"], 1)
        previous_group = object()
        for index, row in enumerate(rows):
            group = row[group_by] if group_by is not None else None
            if group_by is not None and group != previous_group:
                gx, gy, gw, gh = self._item(width, 23)
                self._emit("rect", (gx,gy), (gx+gw,gy+gh), self.theme["group"])
                self._emit("text_align", (gx+10,gy+gh/2), self.theme["muted"],
                           "›  " + str(group), 12, "left")
                previous_group = group
            rx, ry, rw, rh = self._item(width, 24)
            if self.io.MouseReleased[0] and self._press_panel is self._current and \
                    _inside((rx,ry,rw,rh),self._press_pos) and self._hover((rx,ry,rw,rh)):
                selected = index
            visible = ry+rh > self._clip(self._current)[1] and ry < self._clip(self._current)[1]+self._clip(self._current)[3]
            if not visible:
                continue
            self._emit("rect", (rx,ry), (rx+rw,ry+rh),
                       self.theme["selected"] if selected == index else
                       _visible_color(self.theme["hover"], self._hover((rx,ry,rw,rh))))
            pos = rx
            for value, size in zip(row, sizes):
                if isinstance(value, bool):
                    bx, by = pos+size/2-7, ry+rh/2-7
                    self._emit("rrect", (bx,by), (bx+14,by+14), 2, self.theme["check"])
                    self._emit("text_align", (bx+7,by+7),
                               _visible_color(self.theme["check_mark"], value),
                               "✓", 11, "center")
                    pos += size
                    continue
                shown = self._fit_text(str(value), size-12)
                self._emit("text_align", (pos+8,ry+rh/2), self.theme["text"], shown, 12, "left")
                pos += size
        self._current.spacing = spacing
        return selected

    def SliderFloat(self, label, value, minimum=0.0, maximum=1.0, width=0):
        if maximum <= minimum:
            raise ValueError("slider maximum must exceed minimum")
        previous = value
        rect = self._item(width or min(240, self.GetContentRegionAvail()[0]), 26)
        identity = self._id(label)
        self._nav_order.append(identity)
        if self._hover(rect) and self.io.MouseClicked[0] and self._current is self._press_panel:
            self._active = identity
            self._focus = identity
        if self._active == identity and (self.io.MouseDown[0] or self.io.MouseReleased[0]):
            value = minimum + max(0.0, min(1.0, (self.io.MousePos[0]-rect[0])/max(1,rect[2]))) * (maximum-minimum)
        value = max(minimum, min(maximum, value))
        if self._focus == identity and self._keyboard_delta:
            value = max(minimum, min(maximum,
                        value+self._keyboard_delta*(maximum-minimum)/100))
            self._keyboard_delta = 0
        x,y,w,h = rect
        self._emit("rrect", (x,y+10), (x+w,y+16), 3, self.theme["border"])
        if value > minimum:
            self._emit("rrect", (x,y+10), (x+w*(value-minimum)/(maximum-minimum),y+16), 3, self.theme["accent"])
        self._emit("circle", (x+w*(value-minimum)/(maximum-minimum),y+13), 6, self.theme["accent"])
        self._emit("text", (x+w+8,y+4), self.theme["text"], label.split("##")[0], self._font_size())
        return value != previous, value

    def ProgressBar(self, fraction, width=0):
        rect = self._item(width or self.GetContentRegionAvail()[0], 12)
        x,y,w,h = rect
        self._emit("rrect", (x,y), (x+w,y+h), 6, self.theme["control"])
        if fraction > 0:
            self._emit("rrect", (x,y), (x+w*min(1.0,fraction),y+h), 6, self.theme["accent"])

    def _id(self, label):
        return (id(self._current),label)

    def BeginTabBar(self, name):
        p = self._current
        origin = (p.cursor[0],p.cursor[1]+p.scroll)
        p.clip_top = origin[1]+30-p.rect[1]
        self._tabbar = SimpleNamespace(identity=self._id(name), origin=origin, x=origin[0],
                                      bottom=p.cursor[1]+30, right=p.right)
        return True

    def BeginTabItem(self, label):
        b,p = self._tabbar,self._current
        self._tabs.setdefault(b.identity,label)
        w = self.CalcTextSize(label)[0]+20
        rect = (b.x,b.origin[1],w,24)
        selected = self._tabs[b.identity] == label
        self._tab_hits.append((b.identity,label,rect,p))
        if self._tab_at(rect,(b.identity,label),label,selected,clip_content=False):
            self._tabs[b.identity] = label
            selected = True
        b.x += w+4
        p.cursor = (b.origin[0],b.origin[1]+30-p.scroll)
        p.bottom = b.origin[1]+30-p.scroll
        p.previous = None
        return selected

    def EndTabItem(self):
        b,p = self._tabbar,self._current
        b.bottom,b.right = p.bottom,p.right

    def EndTabBar(self):
        b,p = self._tabbar,self._current
        p.bottom,p.right = b.bottom,max(b.right,b.x)
        p.cursor = (b.origin[0],b.bottom+p.spacing[1])
        self._tabbar = None

    def CalcTextSize(self, text, hide=False, wrap_width=-1):
        import pyglet
        text = str(text).split("##")[0] if hide else str(text)
        identity = (text,self.font_name,self._font_size(),wrap_width)
        if identity in self._measurements:
            return self._measurements[identity]
        font = pyglet.font.load(name=self.font_name,size=self._font_size()*72/96)
        lines = text.split("\n")
        widths = [sum(g.advance+p.x_advance for g,p in zip(*font.get_glyphs(line))) for line in lines]
        width = max(widths,default=0)
        count = len(lines)
        if wrap_width > 0:
            # Use pyglet's actual wrapping rules rather than guessing line count.
            doc = pyglet.text.document.UnformattedDocument(text)
            doc.set_style(0,len(text),dict(font_name=self.font_name,font_size=self._font_size()*72/96))
            layout = pyglet.text.layout.TextLayout(doc,width=wrap_width,multiline=True)
            try:
                size = (layout.content_width,layout.content_height)
            finally:
                layout.delete()
        else:
            size = (width,count*(self._font_size()+3))
        if len(self._measurements) >= 512:
            self._measurements.clear()
        self._measurements[identity] = size
        return size

    def _fit_text(self, text, width):
        if self.CalcTextSize(text)[0] <= width:
            return text
        lo, hi = 0, len(text)
        while lo < hi:
            mid = (lo+hi+1)//2
            if self.CalcTextSize(text[:mid]+"…")[0] <= width:
                lo = mid
            else:
                hi = mid-1
        return text[:lo]+"…" if lo else ""

    def TextUnformatted(self, text):
        width = self._wrap[-1] if self._wrap else -1
        w,h = self.CalcTextSize(text,wrap_width=width)
        x,y,_,_ = self._item(w,h)
        self._emit("text",(x,y),self._color(0,self.theme["text"]),str(text),self._font_size(),width)

    Text = TextUnformatted
    TextWrapped = TextUnformatted

    def SectionHeader(self, label):
        x, y, w, h = self._item(self.GetContentRegionAvail()[0], 24)
        self._emit("rect", (x,y), (x+w,y+h), self.theme["group"])
        self._emit("text_align", (x+w/2,y+h/2), self._color(0,self.theme["muted"]),
                   str(label), self._font_size(), "center", True)

    def Separator(self):
        x,y,w,h = self._item(self.GetContentRegionAvail()[0],2)
        self._emit("line",(x,y),(x+w,y),self.theme["border"],1)

    def InputDouble(self, label, value, step=0, fast_step=0, fmt="%.4f"):
        eid = self._id(label)
        width = self._widths[-1] if self._widths else min(200,self.GetContentRegionAvail()[0])
        rect = self._item(width,26)
        editor = self._field_editor(eid, fmt % value, rect)
        changed = False
        try:
            parsed = float(editor.document.text)
            valid = math.isfinite(parsed)
        except ValueError:
            parsed,valid = value,False
        if valid and (self._focus == eid or editor.commit):
            changed,value = parsed != value,parsed
            editor.last_value = value
        elif self._focus != eid and not editor.commit and value != editor.last_value:
            editor.document.text = fmt % value
            editor.last_value = value
        editor.commit = False
        self._field_draw(eid, rect, valid)
        return changed,value

    def InputText(self, label, value, width=0, hint=""):
        """Single-line editable text, returning (changed, value)."""
        eid = self._id(label)
        width = width or (self._widths[-1] if self._widths else min(240,self.GetContentRegionAvail()[0]))
        rect = self._item(width,26)
        editor = self._field_editor(eid, str(value), rect)
        changed = editor.document.text != value
        if self._focus != eid and not editor.commit and value != editor.last_value:
            editor.document.text = str(value)
            changed = False
        if changed:
            value = editor.document.text
        editor.last_value = value
        editor.commit = False
        self._field_draw(eid, rect, True, hint)
        return changed,value

    def InputTextMultiline(self, label, value, width=0, height=96, hint=""):
        eid = self._id(label)
        rect = self._item(width or min(320, self.GetContentRegionAvail()[0]), height)
        editor = self._field_editor(eid, str(value), rect, multiline=True)
        changed = editor.document.text != value
        if self._focus != eid and not editor.commit and value != editor.last_value:
            editor.document.text = str(value)
            changed = False
        if changed:
            value = editor.document.text
        editor.last_value = value
        editor.commit = False
        self._field_draw(eid, rect, True, hint)
        return changed, value

    def _field_editor(self, eid, initial, rect, multiline=False):
        import pyglet
        width = rect[2]
        editor = self._editors.get(eid)
        if editor is None:
            doc = pyglet.text.document.UnformattedDocument(initial)
            doc.set_style(0,len(doc.text),dict(font_name=self.font_name,font_size=11,color=self.theme["text"]))
            layout = pyglet.text.layout.IncrementalTextLayout(
                doc,width=width-12,height=rect[3]-6,multiline=multiline)
            caret = pyglet.text.caret.Caret(layout,color=self.theme["text"][:3])
            editor = SimpleNamespace(document=doc,layout=layout,caret=caret,rect=rect,
                                     multiline=multiline,
                                     original=doc.text,commit=False,last_value=initial,undo=[],redo=[])
            self._editors[eid] = editor
        self._focus_order.append(eid)
        self._nav_order.append(eid)
        editor.rect = rect
        editor.panel = self._current
        x,y,w,h = rect
        editor.layout.x,editor.layout.y = x+6,self.window.height-y-h+3
        editor.layout.width = max(1,width-12)
        editor.layout.height = max(1,h-6)
        if self._focus_next:
            self._focus = eid
            editor.original = editor.document.text
            editor.caret.mark,editor.caret.position = 0,len(editor.document.text)
            self._focus_next = False
        return editor

    def _field_draw(self, eid, rect, valid, hint=""):
        editor = self._editors[eid]
        x,y,w,h = rect
        # Pyglet's Caret owns a separate vertex list created before the layout
        # joins our clipped batch. Keep it for editing and draw its visual in
        # the same command stream as the field instead.
        editor.caret.visible = False
        self._capture()
        border = self.theme["accent"] if self._focus == eid else self.theme["input_border"] if valid else (196,75,75,255)
        self._emit("rrect",(x,y),(x+w,y+h),2,border)
        self._emit("rrect",(x+1,y+1),(x+w-1,y+h-1),1,self.theme["field"])
        self._emit("editor",eid)
        focused = self._focus == eid
        if focused:
            layout = editor.layout
            cx, baseline = layout.get_point_from_position(editor.caret.position)
            font = editor.document.get_font(max(0, editor.caret.position-1))
            cx += layout.left
            baseline += layout.bottom + layout.content_height
            top = self.window.height - baseline - font.ascent
            bottom = self.window.height - baseline - font.descent
            cx = max(x+5, min(x+w-5, cx))
            top = max(y+3, min(y+h-3, top))
            bottom = max(y+3, min(y+h-3, bottom))
        else:
            cx, top, bottom = x+6, y+4, y+h-4
        self._emit("line", (cx, top), (cx, bottom),
                   _visible_color(self.theme["text"], focused and time.monotonic() % 1 < .55), 1)
        if hint and not editor.document.text:
            self._emit("text_align", (x+8,y+13 if editor.multiline else y+h/2), self.theme["muted"],
                       hint, self._font_size(), "left")

    InputFloat = InputDouble

    def SetKeyboardFocusHere(self):
        self._focus_next = True

    def SetTooltip(self, text):
        x,y = self.io.MousePos
        w,h = self.CalcTextSize(text)
        x = min(x+12,max(0,self.window.width-w-12))
        y = min(y+18,max(0,self.window.height-h-12))
        self._emit("rect",(x,y),(x+w+12,y+h+8),self.theme["title"],foreground=True,clip=False)
        self._emit("text",(x+6,y+4),self.theme["text"],text,14,foreground=True,clip=False)

    def IsItemHovered(self):
        return self._hover(self._last)

    def IsWindowHovered(self, flags=0):
        return self._panel_at(self.io.MousePos) is not None

    IsAnyItemHovered = IsWindowHovered

    def IsMouseDown(self, button):
        return self.io.MouseDown[button]

    def IsMouseClicked(self, button):
        return self.io.MouseClicked[button]

    def IsMouseReleased(self, button):
        return self.io.MouseReleased[button]

    def IsMouseDoubleClicked(self, button):
        return self.io.MouseDoubleClicked[button]

    def IsKeyDown(self, key):
        return self.io.KeysDown.get(key,False)

    def GetWindowDrawList(self):
        return self._drawlist

    def GetForegroundDrawList(self):
        return self._frontlist

    GetBackgroundDrawList = GetWindowDrawList

    def GetFontSize(self):
        return self._font_size()

    def GetFont(self):
        return None

    def SetWindowFontScale(self, scale):
        self._current.scale = scale

    def PushItemWidth(self, width):
        self._widths.append(width)

    def PopItemWidth(self):
        self._widths.pop()

    def PushTextWrapPos(self, width):
        self._wrap.append(max(1,width-self.GetCursorPosX()))

    def PopTextWrapPos(self):
        self._wrap.pop()

    def PushStyleVar(self, index, value):
        self._styles.append((index,tuple(value)))

    def PopStyleVar(self, count=1):
        del self._styles[-count:]

    def PushStyleColor(self, index, *value):
        self._colors.append((index,self.GetColorU32(value[0] if len(value)==1 else value)))

    def PopStyleColor(self, count=1):
        del self._colors[-count:]

    @staticmethod
    def GetColorU32(value):
        return tuple(round(max(0,min(1,c))*255) for c in value)

    ColorConvertFloat4ToU32 = GetColorU32

    def SetClipboardText(self, text):
        self.window.set_clipboard_text(text)

    def _dock_chrome(self):
        self._layout_docks()
        commands = []
        hits = []

        def emit(kind, *args):
            commands.append((kind, args, None))

        seen = set()
        for node, rect in self.docks.rects.values():
            if id(node) in seen or node["active"] == DockLayout.VIEWPORT:
                continue
            seen.add(id(node))
            x, y, w, h = rect
            emit("rect", (x,y), (x+w,y+h), self.theme["content"])
            emit("rect", (x,y), (x+w,y+31), self.theme["title"])
            emit("rect", (x,y+30), (x+w,y+31), self.theme["border"])
            tab_x = x+4
            for title in node["tabs"]:
                if not self.docks.visible.get(title, False):
                    continue
                label = title.split("##")[0]
                tab_w = min(max(72, 8*len(label)+18), max(72, w-8))
                tab_rect = (tab_x, y+3, tab_w, 26)
                hits.append((title, tab_rect))
                if node["active"] == title:
                    emit("rect", (tab_x,y+28), (tab_x+tab_w,y+31), self.theme["accent"])
                emit("text_align", (tab_x+8,y+16), self.theme["text"] if node["active"] == title else self.theme["muted"],
                     label, 14, "left", node["active"] == title)
                tab_x += tab_w+2
        for node, bar, _ in self.docks.splitters:
            x, y, w, h = bar
            if node["axis"] == "x":
                emit("rect", (x+w/2-.5,y), (x+w/2+.5,y+h), self.theme["border"])
            else:
                emit("rect", (x,y+h/2-.5), (x+w,y+h/2+.5), self.theme["border"])
        if self._dock_drag:
            target = self.docks.drop_target(self.io.MousePos)
            if target:
                x, y, w, h = target[2]
                self._foreground.append(("rect", ((x,y), (x+w,y+h),
                                          (*self.theme["accent"][:3], 70)), None))
        self._dock_tabs = hits
        return commands

    def _command_visible(self, command):
        kind, args, panel = command
        clip = self._clip(panel)
        if kind in ("rect", "rrect", "line"):
            a, b = args[:2]
            bounds = (min(a[0], b[0]), min(a[1], b[1]),
                      abs(a[0]-b[0])+1, abs(a[1]-b[1])+1)
        elif kind in ("text", "text_align"):
            x, y = args[0]
            width = max(1, len(args[2])*args[3])
            if kind == "text_align":
                bounds = (x-width/2 if args[4] == "center" else x, y-args[3]/2-4,
                          width, args[3]+8)
            else:
                bounds = (x, y, width, args[3]+6)
        elif kind in ("circle", "arc"):
            (x, y), radius = args[:2]
            bounds = (x-radius, y-radius, 2*radius, 2*radius)
        elif kind == "triangle":
            points = args[:3]
            xs, ys = [p[0] for p in points], [p[1] for p in points]
            bounds = (min(xs), min(ys), max(xs)-min(xs)+1, max(ys)-min(ys)+1)
        elif kind == "editor":
            bounds = self._editors[args[0]].rect
        else:
            return True
        return (bounds[0] < clip[0]+clip[2] and bounds[0]+bounds[2] > clip[0] and
                bounds[1] < clip[1]+clip[3] and bounds[1]+bounds[3] > clip[1])

    def render(self):
        """Cached batched primitives; only changed commands allocate GPU objects."""
        import pyglet
        from pyglet import gl, shapes
        from pyglet.math import Mat4
        if self._current is not None:
            self.End()
        commands = [command for command in self._dock_chrome()+self._commands+self._foreground
                    if self._command_visible(command)]
        if self._batch is None:
            self._batch = pyglet.graphics.Batch()
        batch = self._batch
        if self._clip_class is None:
            window = self.window
            class ClipGroup(pyglet.graphics.Group):
                current = None
                def __init__(self, order, rect):
                    super().__init__(order=order)
                    self.rect = rect
                def __eq__(group, other):
                    return type(group) is type(other) and group.order == other.order and group.rect == other.rect
                def __hash__(group):
                    return hash((type(group), group.order, group.rect))
                def set_state(group):
                    if ClipGroup.current == group.rect:
                        return
                    sx,sy = (a/max(1,b) for a,b in zip(window.get_framebuffer_size(),window.get_size()))
                    x,y,w,h = group.rect
                    gl.glEnable(gl.GL_SCISSOR_TEST)
                    gl.glScissor(round(x*sx),round((window.height-y-h)*sy),max(0,round(w*sx)),max(0,round(h*sy)))
                    ClipGroup.current = group.rect
                def unset_state(group):
                    pass
            self._clip_class = ClipGroup
        ClipGroup = self._clip_class
        segment = None
        segment_start = 0
        groups = {}
        color_slots = {"rect": 2, "rrect": 3, "line": 2,
                       "circle": 2, "arc": 2, "triangle": 3,
                       "text": 1, "text_align": 1}
        for i,(kind,args,p) in enumerate(commands):
            clip = self._clip(p)
            if p is not None:
                identity = (id(p), clip)
                if identity != segment:
                    segment, segment_start = identity, i
                order = 4*segment_start + (2 if kind == "editor" else 1 if kind in ("text", "text_align") else 0)
            else:
                segment = None
                order = 4*i
            signature = (kind,args,clip,order,self.window.height)
            key = (order,clip)
            group = groups.get(key)
            if group is None:
                group = groups[key] = ClipGroup(order,clip)
            if kind == "editor":
                if i < len(self._cache) and self._cache[i][0] == signature:
                    continue
                editor = self._editors[args[0]]
                editor.layout.batch = batch
                editor.layout.group = group
                if i < len(self._cache):
                    if self._cache[i][1] is not None:
                        self._cache[i][1].delete()
                    self._cache[i] = (signature,None)
                else:
                    self._cache.append((signature,None))
                continue
            if i < len(self._cache):
                old, obj = self._cache[i]
                if old == signature:
                    continue
                color_index = color_slots.get(kind)
                if (obj is not None and color_index is not None and old[0] == kind
                        and old[2:] == signature[2:]
                        and old[1][:color_index]+old[1][color_index+1:] ==
                            args[:color_index]+args[color_index+1:]):
                    obj.color = args[color_index]
                    self._cache[i] = (signature, obj)
                    continue
            if i < len(self._cache):
                if self._cache[i][1] is not None:
                    self._cache[i][1].delete()
            a = args
            if kind == "rect":
                (x,y),(u,v),color = a
                obj = shapes.Rectangle(x,self.window.height-v,max(0,u-x),max(0,v-y),color=color,batch=batch,group=group)
            elif kind == "rrect":
                (x,y),(u,v),radius,color = a
                width, height = max(0,u-x), max(0,v-y)
                radius = min(radius,width/2,height/2)
                obj = (shapes.RoundedRectangle(x,self.window.height-v,width,height,radius=radius,
                                               segments=4,color=color,batch=batch,group=group)
                       if radius >= 1 else
                       shapes.Rectangle(x,self.window.height-v,width,height,color=color,batch=batch,group=group))
            elif kind == "line":
                (x,y),(u,v),color,th = a
                obj = shapes.Line(x,self.window.height-y,u,self.window.height-v,thickness=th,color=color,batch=batch,group=group)
            elif kind in ("circle","arc"):
                (x,y),r,color = a[:3]
                cls = shapes.Circle if kind == "circle" else shapes.Arc
                kw = {} if kind == "circle" else dict(thickness=a[3])
                obj = cls(x,self.window.height-y,r,color=color,batch=batch,group=group,**kw)
            elif kind == "triangle":
                (x,y),(u,v),(s,t),color = a
                obj = shapes.Triangle(x,self.window.height-y,u,self.window.height-v,s,self.window.height-t,color=color,batch=batch,group=group)
            elif kind in ("text", "text_align"):
                (x,y),color,text,size = a[:4]
                if kind == "text_align":
                    # Segoe UI's line box sits below its visible glyphs. Lift centered
                    # labels slightly so their ink, rather than the line box, is centered.
                    obj = pyglet.text.Label(text,x=x,y=self.window.height-y+2,anchor_x=a[4],anchor_y="center",
                                            font_name=self.font_name,font_size=size*72/96,
                                            weight="bold" if len(a)>5 and a[5] else "normal",
                                            color=color,batch=batch,group=group)
                else:
                    width = a[4] if len(a)>4 and a[4]>0 else None
                    obj = pyglet.text.Label(text,x=x,y=self.window.height-y,anchor_y="top",font_size=size*72/96,
                                            font_name=self.font_name,color=color,width=width,
                                            multiline=width is not None,batch=batch,group=group)
            else:
                raise ValueError("Unknown GUI primitive: "+kind)
            if i < len(self._cache):
                self._cache[i] = (signature,obj)
            else:
                self._cache.append((signature,obj))
        for _,obj in self._cache[len(commands):]:
            if obj is not None:
                obj.delete()
        del self._cache[len(commands):]
        for eid in list(self._editors):
            if eid not in self._focus_order:
                self._editors[eid].caret.delete()
                self._editors[eid].layout.delete()
                del self._editors[eid]
                if self._focus == eid:
                    self._focus = None
        if self._focus is not None and self._focus not in self._nav_order:
            self._focus = None
        if not self._combo_seen:
            self._combo_open = self._combo_popup = None
        projection,view = self.window.projection,self.window.view
        enabled = gl.glIsEnabled(gl.GL_SCISSOR_TEST)
        scissor = (gl.GLint*4)()
        gl.glGetIntegerv(gl.GL_SCISSOR_BOX,scissor)
        try:
            self.window.projection = Mat4.orthogonal_projection(0,self.window.width,0,self.window.height,-1,1)
            self.window.view = Mat4()
            ClipGroup.current = None
            batch.draw()
        finally:
            self.window.projection,self.window.view = projection,view
            gl.glScissor(*scissor)
            (gl.glEnable if enabled else gl.glDisable)(gl.GL_SCISSOR_TEST)
        if self.io.MouseReleased[0]:
            self._active = None
        self.io.MouseDelta = (0,0)
        self.io.MouseWheel = 0
        self.io.MouseClicked[:] = [False]*3
        self.io.MouseReleased[:] = [False]*3
        self.io.MouseDoubleClicked[:] = [False]*3
        self._keyboard_activate = None
        self._keyboard_delta = 0
        self._capture()
        self._update_cursor()

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self.window.set_mouse_cursor(None)
        self.window.remove_handlers(self)
        for _,obj in self._cache:
            if obj is not None:
                obj.delete()
        self._cache.clear()
        for editor in self._editors.values():
            editor.caret.delete()
            editor.layout.delete()
        self._editors.clear()
        self._batch = None
        self._commands.clear()
        self._foreground.clear()
