"""Batched window chrome with native desktop dragging, resizing and snapping.

The UI owns the title and buttons; the operating system owns window movement.
The Windows adapter is isolated here and uses only the standard library.
"""
import sys
from functools import cache


@cache
def chrome_window_class(base):
    """Keep pyglet's client-size calculations consistent with a custom frame."""
    if sys.platform != "win32":
        return base

    class ChromeWindow(base):
        _custom_frame = False

        def _client_to_window_size(self, width, height, dpi):
            return (width, height) if self._custom_frame else super()._client_to_window_size(width, height, dpi)

        def _client_to_window_size_dpi(self, width, height):
            return (width, height) if self._custom_frame else super()._client_to_window_size_dpi(width, height)

        def _client_to_window_pos(self, x, y):
            return (x, y) if self._custom_frame else super()._client_to_window_pos(x, y)

    return ChromeWindow


class WindowChrome:
    height = 36
    button_width = 46
    resize_border = 6

    def __init__(self, window, ui, *, panel="Window", title=None, height=36):
        self.window, self.ui = window, ui
        self.panel, self.title = panel, title
        self.height = height
        self.drag_rect = None
        self._native = _WindowsFrame(self) if sys.platform == "win32" else None
        if panel is not None:
            ui.dock_panel(panel, "top", size=self.height, header=False)

    def hit_test(self, x, y, width, height, maximized=False):
        """Windows hit code for logical, top-down client coordinates."""
        if not 0 <= x < width or not 0 <= y < height:
            return 1  # HTCLIENT
        if not maximized:
            left, right = x < self.resize_border, x >= width-self.resize_border
            top, bottom = y < self.resize_border, y >= height-self.resize_border
            if top:
                return 13 if left else 14 if right else 12
            if bottom:
                return 16 if left else 17 if right else 15
            if left or right:
                return 10 if left else 11
        drag = self.drag_rect
        draggable = (drag[0] <= x < drag[0]+drag[2] and drag[1] <= y < drag[1]+drag[3]
                     if drag is not None else x < width-3*self.button_width-8)
        if y < self.height and draggable:
            return 2  # HTCAPTION: native drag, double click and system menu
        return 1

    @property
    def maximized(self):
        return self._native.maximized if self._native else False

    def draw(self):
        ui = self.ui
        if ui.Begin(self.panel):
            width = ui.GetContentRegionAvail()[0]
            title = self.title() if callable(self.title) else self.title
            title = title or self.window.caption
            x, y = ui.GetCursorScreenPos()
            ui.TextEllipsis(title, max(1, width-3*self.button_width-16), height=24)
            self.drag_rect = (x, 0, max(1, width-3*self.button_width-16), self.height)
            self.draw_buttons()
        ui.End()

    def draw_buttons(self):
        """Append caption buttons to an existing toolbar; no additional row."""
        ui = self.ui
        for index, (label, icon) in enumerate((
                ("Minimize", self._minimize_icon),
                ("Restore" if self.maximized else "Maximize", self._maximize_icon),
                ("Close", self._close_icon))):
            ui.SameLine(offset=max(0, self.window.width-3*self.button_width-8)+index*self.button_width)
            if ui.RibbonButton("##window_"+label, icon, width=self.button_width,
                               height=28, icon_size=18, tooltip=label):
                if index == 0:
                    self.window.minimize()
                elif index == 1:
                    if self._native:
                        self._native.toggle_maximize()
                    else:
                        self.window.maximize()
                else:
                    self.window.dispatch_event("on_close")

    @staticmethod
    def _minimize_icon(dl, x, y, ink, accent):
        dl.AddLine((x+5, y+12), (x+19, y+12), ink, 1.4)

    def _maximize_icon(self, dl, x, y, ink, accent):
        if self.maximized:
            dl.AddRect((x+8, y+5), (x+19, y+16), ink, th=1.3)
            dl.AddRectFilled((x+5, y+8), (x+16, y+19), self.ui.theme["window"])
            dl.AddRect((x+5, y+8), (x+16, y+19), ink, th=1.3)
        else:
            dl.AddRect((x+5, y+5), (x+19, y+19), ink, th=1.3)

    @staticmethod
    def _close_icon(dl, x, y, ink, accent):
        dl.AddLine((x+6, y+6), (x+18, y+18), ink, 1.5)
        dl.AddLine((x+18, y+6), (x+6, y+18), ink, 1.5)


class _WindowsFrame:
    """Subclass the native parent and GL child without replacing pyglet handlers."""

    def __init__(self, chrome):
        import ctypes as c
        from ctypes import wintypes as w
        self.chrome, self.c, self.w = chrome, c, w
        self.user = c.WinDLL("user32", use_last_error=True)
        self.common = c.WinDLL("comctl32", use_last_error=True)
        pointer = c.c_size_t
        result = c.c_ssize_t
        self.callback_type = c.WINFUNCTYPE(result, w.HWND, w.UINT, pointer, result, pointer, pointer)
        self.common.SetWindowSubclass.argtypes = [w.HWND, self.callback_type, pointer, pointer]
        self.common.SetWindowSubclass.restype = w.BOOL
        self.common.RemoveWindowSubclass.argtypes = [w.HWND, self.callback_type, pointer]
        self.common.DefSubclassProc.argtypes = [w.HWND, w.UINT, pointer, result]
        self.common.DefSubclassProc.restype = result
        self.user.GetWindowLongW.argtypes = [w.HWND, c.c_int]
        self.user.SetWindowLongW.argtypes = [w.HWND, c.c_int, w.LONG]
        self.user.SetWindowPos.argtypes = [w.HWND, w.HWND, c.c_int, c.c_int, c.c_int, c.c_int, w.UINT]
        self.user.IsZoomed.argtypes = [w.HWND]
        self.user.IsZoomed.restype = w.BOOL
        self.user.ShowWindow.argtypes = [w.HWND, c.c_int]
        self.user.ScreenToClient.argtypes = [w.HWND, c.POINTER(w.POINT)]
        self.user.GetClientRect.argtypes = [w.HWND, c.POINTER(w.RECT)]
        self.user.GetDpiForWindow.argtypes = [w.HWND]
        self.user.GetDpiForWindow.restype = w.UINT
        self.user.GetSystemMetricsForDpi.argtypes = [c.c_int,w.UINT]
        self.user.MonitorFromWindow.argtypes = [w.HWND, w.DWORD]
        self.user.MonitorFromWindow.restype = w.HANDLE
        self.hwnd = chrome.window._hwnd
        self.child = chrome.window._view_hwnd
        self.callback = self.callback_type(self._message)
        self._handles = []
        try:
            for handle in (self.hwnd, self.child):
                if not self.common.SetWindowSubclass(handle, self.callback, id(self), 0):
                    raise c.WinError(c.get_last_error())
                self._handles.append(handle)
            # Keep standard desktop capabilities, while NCCALCSIZE removes only
            # the system-drawn frame. Native hit testing supplies its geometry.
            style = self.user.GetWindowLongW(self.hwnd, -16)
            client = w.RECT()
            self.user.GetClientRect(self.hwnd, c.byref(client))
            self.user.SetWindowLongW(self.hwnd, -16, (style & ~0x80000000) | 0x00CF0000)
            chrome.window._custom_frame = True
            self.user.SetWindowPos(self.hwnd, None, 0, 0, client.right, client.bottom, 0x0026)
        except Exception:
            for handle in self._handles:
                self.common.RemoveWindowSubclass(handle, self.callback, id(self))
            raise

    @property
    def maximized(self):
        return bool(self.user.IsZoomed(self.hwnd))

    def toggle_maximize(self):
        self.user.ShowWindow(self.hwnd, 9 if self.maximized else 3)

    def _message(self, hwnd, message, wp, lp, identity, data):
        c, w = self.c, self.w
        if message == 0x0082:  # WM_NCDESTROY
            self.common.RemoveWindowSubclass(hwnd, self.callback, identity)
        elif message == 0x0083 and hwnd == self.hwnd:  # WM_NCCALCSIZE
            if self.maximized:
                # Windows retains an invisible resize border around a maximized
                # outer rectangle. Exclude it from the client, as a system frame
                # would, so the custom toolbar stays inside the monitor work area.
                rect = c.cast(lp,c.POINTER(w.RECT)).contents
                dpi = self.user.GetDpiForWindow(hwnd)
                border = self.user.GetSystemMetricsForDpi(32,dpi)+self.user.GetSystemMetricsForDpi(92,dpi)
                rect.left += border
                rect.top += border
                rect.right -= border
                rect.bottom -= border
            return 0
        elif message == 0x0084:  # WM_NCHITTEST, screen pixels -> logical client
            point = w.POINT(c.c_short(lp & 65535).value, c.c_short((lp >> 16) & 65535).value)
            self.user.ScreenToClient(self.hwnd, c.byref(point))
            rect = w.RECT()
            self.user.GetClientRect(self.hwnd, c.byref(rect))
            width, height = self.chrome.window.get_size()
            code = self.chrome.hit_test(point.x*width/max(1, rect.right),
                                       point.y*height/max(1, rect.bottom), width, height, self.maximized)
            # GL child passes non-client regions through to its parent, which
            # then lets Windows run its ordinary move/resize interaction.
            return -1 if hwnd == self.child and code != 1 else code
        elif message == 0x0024 and hwnd == self.hwnd:  # WM_GETMINMAXINFO
            result = self.common.DefSubclassProc(hwnd, message, wp, lp)
            self._work_area(lp)
            return result
        elif message == 0x02E4 and hwnd == self.hwnd:  # WM_GETDPISCALEDSIZE
            # The OS already supplies the proposed scaled size. Pyglet's default
            # handler adds the change in system-frame thickness, which this
            # custom client does not have. Its WM_DPICHANGED handler remains intact.
            return 1
        return self.common.DefSubclassProc(hwnd, message, wp, lp)

    def _work_area(self, address):
        c, w = self.c, self.w

        class MonitorInfo(c.Structure):
            _fields_ = [("size", w.DWORD), ("monitor", w.RECT), ("work", w.RECT), ("flags", w.DWORD)]

        class MinMax(c.Structure):
            _fields_ = [(name, w.POINT) for name in ("reserved", "max_size", "max_position", "min_track", "max_track")]

        info = MonitorInfo()
        info.size = c.sizeof(info)
        self.user.GetMonitorInfoW.argtypes = [w.HANDLE, c.POINTER(MonitorInfo)]
        if self.user.GetMonitorInfoW(self.user.MonitorFromWindow(self.hwnd, 2), c.byref(info)):
            bounds = c.cast(address, c.POINTER(MinMax)).contents
            bounds.max_position = w.POINT(info.work.left-info.monitor.left, info.work.top-info.monitor.top)
            bounds.max_size = w.POINT(info.work.right-info.work.left, info.work.bottom-info.work.top)
