import sys


def copy_to_clipboard(text):
    text = text or ""
    try:
        from .imgui_compat import import_imgui
        imgui = import_imgui()
        setter = getattr(imgui, "SetClipboardText", None) or getattr(imgui, "set_clipboard_text", None)
        if setter is not None:
            setter(text)
            return
    except Exception:
        pass
    if sys.platform == "win32":
        try:
            _copy_win32(text)
            return
        except Exception:
            pass
    try:
        import tkinter
        root = tkinter.Tk()
        root.withdraw()
        root.clipboard_clear()
        root.clipboard_append(text)
        root.update()
        root.destroy()
    except Exception:
        sys.stderr.write("camber: could not copy to clipboard\n")


def _copy_win32(text):
    import ctypes
    CF_UNICODETEXT = 13
    GMEM_MOVEABLE = 0x0002
    user32 = ctypes.windll.user32
    kernel32 = ctypes.windll.kernel32
    payload = text.encode("utf-16-le") + b"\x00\x00"
    if not user32.OpenClipboard(None):
        raise OSError("OpenClipboard failed")
    try:
        user32.EmptyClipboard()
        handle = kernel32.GlobalAlloc(GMEM_MOVEABLE, len(payload))
        if not handle:
            raise OSError("GlobalAlloc failed")
        locked = kernel32.GlobalLock(handle)
        ctypes.memmove(locked, payload, len(payload))
        kernel32.GlobalUnlock(handle)
        if not user32.SetClipboardData(CF_UNICODETEXT, handle):
            raise OSError("SetClipboardData failed")
    finally:
        user32.CloseClipboard()
