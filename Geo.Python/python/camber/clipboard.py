"""System clipboard access through the live viewer's native window backend."""

import sys
from functools import cache


def copy_to_clipboard(text, window=None):
    """Copy text to the OS clipboard; return whether the backend accepted it.

    The window stays alive to serve selection requests on platforms where the
    clipboard is owned by its source application. ImGui's private text buffer
    is not a system clipboard implementation.
    """
    try:
        if window is None:
            import pyglet
            window = next(iter(pyglet.app.windows), None)
        if window is None:
            raise RuntimeError("no live viewer window")
        window.set_clipboard_text(text or "")
        return True
    except Exception as error:
        sys.stderr.write(f"camber: could not copy to clipboard: {error}\n")
        return False


@cache
def _window_class():
    import pyglet
    base = pyglet.window.Window
    if not base.__module__.startswith("pyglet.window.xlib"):
        return base
    from ctypes import byref, c_ubyte
    from pyglet.libs.x11 import xlib
    from pyglet.window.xlib import XlibEventHandler

    class ClipboardWindow(base):
        # The installed X11 backend sends len(text) bytes for UTF-8 requests,
        # truncating every non-ASCII payload. Handle that protocol branch with
        # its byte count; retain the backend's other selection handling.
        @XlibEventHandler(xlib.SelectionRequest)
        def _event_selection_request(self, event):
            request = event.xselectionrequest
            if (request.selection != self._clipboard_atom or request.target != self._utf8_atom
                    or xlib.XGetSelectionOwner(self._x_display, self._clipboard_atom) != self._window):
                return super()._event_selection_request(event)
            payload = (self._clipboard_str or "").encode("utf-8")
            property_atom = request.property or request.target
            xlib.XChangeProperty(self._x_display, request.requestor, property_atom,
                                 request.target, 8, xlib.PropModeReplace,
                                 (c_ubyte * len(payload)).from_buffer_copy(payload), len(payload))
            reply = xlib.XEvent()
            reply.xselection.type = xlib.SelectionNotify
            reply.xselection.display = request.display
            reply.xselection.requestor = request.requestor
            reply.xselection.selection = request.selection
            reply.xselection.target = request.target
            reply.xselection.property = property_atom
            reply.xselection.time = request.time
            xlib.XSendEvent(self._x_display, request.requestor, 0, 0, byref(reply))
            xlib.XFlush(self._x_display)

    return ClipboardWindow


def create_window(*args, **kwargs):
    """Create the viewer window with native clipboard protocol corrections."""
    return _window_class()(*args, **kwargs)
