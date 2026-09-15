"""SolidWorks-style sketch icons drawn with ImGui (same glyphs as GeoScriptViewer, no image files)."""

import math

INK = (0.18, 0.18, 0.20)
INK_ON_ACCENT = (0.98, 0.98, 0.98)
ACCENT = (1.00, 0.65, 0.16)
BTN_W = 60
BTN_H = 60
ICON = 24
COLS = 3
PAD = 12
ITEM_GAP = 8
CONTENT_W = COLS * BTN_W + (COLS - 1) * ITEM_GAP
PANEL_W = PAD * 2 + CONTENT_W


def _u32(imgui, rgb, a=1.0):
    r, g, b = rgb
    try:
        return imgui.GetColorU32((r, g, b, a))
    except Exception:
        pass
    try:
        return imgui.ColorConvertFloat4ToU32((r, g, b, a))
    except Exception:
        pass
    return int(a * 255) << 24 | int(b * 255) << 16 | int(g * 255) << 8 | int(r * 255)


def _dl(imgui):
    getter = getattr(imgui, "GetWindowDrawList", None) or getattr(imgui, "get_window_draw_list", None)
    return getter()


def _pos(imgui):
    getter = getattr(imgui, "GetCursorScreenPos", None) or getattr(imgui, "get_cursor_screen_pos", None)
    p = getter()
    return (float(p[0]), float(p[1])) if hasattr(p, "__len__") else (float(p.x), float(p.y))


def _pt(ox, oy, x, y, s=ICON):
    return (ox + x * s / 24.0, oy + y * s / 24.0)


def _text_size(imgui, text):
    try:
        size = imgui.CalcTextSize(text)
        if hasattr(size, "__len__"):
            return float(size[0]), float(size[1])
        return float(size.x), float(size.y)
    except Exception:
        return len(text) * 7.0, 14.0


def _line(dl, a, b, col, th=1.6):
    dl.AddLine(a, b, col, th)


def _dot(dl, c, col, r=2.0):
    dl.AddCircleFilled(c, r, col, 12)


def _circ(dl, c, r, col, th=1.6, segs=28):
    try:
        dl.AddCircle(c, r, col, segs, th)
    except TypeError:
        dl.AddCircle(c, r, col, segs)


def _circ_fill(dl, c, r, col, segs=16):
    dl.AddCircleFilled(c, r, col, segs)


def _arc(dl, ox, oy, cx, cy, rx, ry, start_deg, sweep_deg, col, th=1.6, n=20):
    # Screen Y-down: standard cos/sin matches GDI+ (clockwise from east).
    start = start_deg * math.pi / 180.0
    sweep = sweep_deg * math.pi / 180.0
    pts = []
    for i in range(n + 1):
        t = start + sweep * i / n
        pts.append(_pt(ox, oy, cx + rx * math.cos(t), cy + ry * math.sin(t)))
    for i in range(len(pts) - 1):
        _line(dl, pts[i], pts[i + 1], col, th)


def _arrows(dl, a, b, col, th=1.6, s=3.5):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-3:
        return
    dx /= length
    dy /= length
    px, py = -dy, dx
    _line(dl, a, (a[0] + dx * s + px * 2, a[1] + dy * s + py * 2), col, th)
    _line(dl, a, (a[0] + dx * s - px * 2, a[1] + dy * s - py * 2), col, th)
    _line(dl, b, (b[0] - dx * s + px * 2, b[1] - dy * s + py * 2), col, th)
    _line(dl, b, (b[0] - dx * s - px * 2, b[1] - dy * s - py * 2), col, th)


def draw_line(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 4, 19), _pt(ox, oy, 19, 5), ink, 1.6)
    _dot(dl, _pt(ox, oy, 4, 19), accent, 2.2)
    _dot(dl, _pt(ox, oy, 19, 5), accent, 2.2)


def draw_polyline(dl, ox, oy, ink, accent):
    pts = [(3, 18), (10, 10), (18, 14), (21, 6)]
    for i in range(len(pts) - 1):
        _line(dl, _pt(ox, oy, *pts[i]), _pt(ox, oy, *pts[i + 1]), ink, 1.6)
    for p in pts:
        _dot(dl, _pt(ox, oy, *p), accent, 1.8)


def draw_circle(dl, ox, oy, ink, accent):
    c = _pt(ox, oy, 12, 12)
    r = 9.0 * ICON / 24.0
    _circ(dl, c, r, ink, 1.6, 32)
    _dot(dl, c, accent, 2.2)


def draw_arc(dl, ox, oy, ink, accent):
    # GDI arc in rect (3,7,18,16), start 200°, sweep 140° — GDI clockwise from east.
    cx, cy = _pt(ox, oy, 3 + 9, 7 + 8)
    rx = 9.0 * ICON / 24.0
    ry = 8.0 * ICON / 24.0
    start = 200.0 * math.pi / 180.0
    sweep = 140.0 * math.pi / 180.0
    n = 24
    pts = []
    for i in range(n + 1):
        t = start + sweep * i / n
        pts.append((cx + rx * math.cos(t), cy + ry * math.sin(t)))
    for i in range(len(pts) - 1):
        _line(dl, pts[i], pts[i + 1], ink, 1.6)
    _dot(dl, pts[0], accent, 2.0)
    _dot(dl, pts[n // 2], accent, 2.0)
    _dot(dl, pts[-1], accent, 2.0)


def draw_rectangle(dl, ox, oy, ink, accent):
    a = _pt(ox, oy, 4, 6)
    b = _pt(ox, oy, 20, 18)
    dl.AddRect(a, b, ink, 0.0, 0, 1.6)


def draw_normal(dl, ox, oy, ink, accent):
    p0 = _pt(ox, oy, 4, 16)
    p1 = _pt(ox, oy, 14, 10)
    p2 = _pt(ox, oy, 20, 14)
    p3 = _pt(ox, oy, 10, 20)
    _line(dl, p0, p1, ink, 1.5)
    _line(dl, p1, p2, ink, 1.5)
    _line(dl, p2, p3, ink, 1.5)
    _line(dl, p3, p0, ink, 1.5)
    _line(dl, _pt(ox, oy, 12, 15), _pt(ox, oy, 12, 4), accent, 1.8)
    dl.AddTriangleFilled(_pt(ox, oy, 12, 3), _pt(ox, oy, 9, 8), _pt(ox, oy, 15, 8), accent)


def draw_undo(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 6, 6), _pt(ox, oy, 18, 18), accent, 1.7)
    _line(dl, _pt(ox, oy, 18, 6), _pt(ox, oy, 6, 18), accent, 1.7)


def draw_select(dl, ox, oy, ink, accent):
    points = [_pt(ox, oy, 5, 3), _pt(ox, oy, 18, 13), _pt(ox, oy, 12, 14), _pt(ox, oy, 9, 21)]
    for i in range(len(points)):
        _line(dl, points[i], points[(i + 1) % len(points)], ink, 1.6)
    _dot(dl, _pt(ox, oy, 18, 13), accent, 2.0)


def draw_coincident(dl, ox, oy, ink, accent):
    r = 5.0 * ICON / 24.0
    c0 = _pt(ox, oy, 9, 12)
    c1 = _pt(ox, oy, 15, 12)
    _circ_fill(dl, c0, r, ink)
    _circ_fill(dl, c1, r, accent)
    _circ(dl, c0, r, ink, 1.5)
    _circ(dl, c1, r, ink, 1.5)


def draw_horizontal(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 3, 12), _pt(ox, oy, 21, 12), ink, 1.6)
    _dot(dl, _pt(ox, oy, 4, 12), accent, 2.2)
    _dot(dl, _pt(ox, oy, 20, 12), accent, 2.2)


def draw_vertical(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 12, 3), _pt(ox, oy, 12, 21), ink, 1.6)
    _dot(dl, _pt(ox, oy, 12, 4), accent, 2.2)
    _dot(dl, _pt(ox, oy, 12, 20), accent, 2.2)


def draw_parallel(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 5, 7), _pt(ox, oy, 19, 5), ink, 1.6)
    _line(dl, _pt(ox, oy, 5, 17), _pt(ox, oy, 19, 15), ink, 1.6)


def draw_perpendicular(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 5, 19), _pt(ox, oy, 5, 5), ink, 1.6)
    _line(dl, _pt(ox, oy, 5, 19), _pt(ox, oy, 19, 19), ink, 1.6)
    a = _pt(ox, oy, 5, 15)
    b = _pt(ox, oy, 9, 19)
    try:
        dl.AddRect(a, b, accent, 0.0, 0, 1.6)
    except TypeError:
        dl.AddRect(a, b, accent)


def draw_tangent(dl, ox, oy, ink, accent):
    cx, cy, r = 10.0, 13.5, 6.0
    _circ(dl, _pt(ox, oy, cx, cy), r * ICON / 24.0, ink, 1.6)
    rad = 30.0 * math.pi / 180.0
    nx = math.cos(rad)
    ny = -math.sin(rad)
    tx, ty = -ny, nx
    contact = (cx + r * nx, cy + r * ny)
    half = 9.0
    _line(
        dl,
        _pt(ox, oy, contact[0] - tx * half, contact[1] - ty * half),
        _pt(ox, oy, contact[0] + tx * half, contact[1] + ty * half),
        accent,
        1.7,
    )


def draw_equal(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 5, 9), _pt(ox, oy, 19, 9), ink, 1.6)
    _line(dl, _pt(ox, oy, 5, 15), _pt(ox, oy, 19, 15), ink, 1.6)


def draw_midpoint(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 3, 12), _pt(ox, oy, 21, 12), ink, 1.6)
    a = _pt(ox, oy, 10, 8)
    b = _pt(ox, oy, 14, 16)
    try:
        dl.AddRectFilled(a, b, accent)
    except TypeError:
        dl.AddRectFilled(a, b, accent)


def draw_concentric(dl, ox, oy, ink, accent):
    c = _pt(ox, oy, 12, 12)
    _circ(dl, c, 8.0 * ICON / 24.0, ink, 1.6)
    _circ(dl, c, 4.0 * ICON / 24.0, accent, 1.7)


def draw_fix(dl, ox, oy, ink, accent):
    a = _pt(ox, oy, 7, 11)
    b = _pt(ox, oy, 17, 19)
    try:
        dl.AddRect(a, b, ink, 0.0, 0, 1.6)
    except TypeError:
        dl.AddRect(a, b, ink)
    _arc(dl, ox, oy, 12, 10, 4, 5, 180, 180, ink, 1.6, 16)
    key = _pt(ox, oy, 11, 13)
    key_b = _pt(ox, oy, 13, 17)
    try:
        dl.AddRectFilled(key, key_b, accent)
    except TypeError:
        dl.AddRectFilled(key, key_b, accent)


def draw_length(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 4, 8), _pt(ox, oy, 20, 8), ink, 1.6)
    _line(dl, _pt(ox, oy, 4, 8), _pt(ox, oy, 4, 14), accent, 1.5)
    _line(dl, _pt(ox, oy, 20, 8), _pt(ox, oy, 20, 14), accent, 1.5)
    a = _pt(ox, oy, 5, 16)
    b = _pt(ox, oy, 19, 16)
    _line(dl, a, b, accent, 1.6)
    _arrows(dl, a, b, accent, 1.6)


def draw_radius(dl, ox, oy, ink, accent):
    _arc(dl, ox, oy, 12, 12, 9, 9, 220, 100, ink, 1.6, 18)
    _line(dl, _pt(ox, oy, 12, 12), _pt(ox, oy, 19, 6), accent, 1.6)
    _dot(dl, _pt(ox, oy, 12, 12), accent, 2.2)


def draw_distance(dl, ox, oy, ink, accent):
    _dot(dl, _pt(ox, oy, 5, 12), ink, 2.2)
    _dot(dl, _pt(ox, oy, 19, 12), ink, 2.2)
    a = _pt(ox, oy, 7, 12)
    b = _pt(ox, oy, 17, 12)
    _line(dl, a, b, accent, 1.6)
    _arrows(dl, a, b, accent, 1.6)


def draw_angle(dl, ox, oy, ink, accent):
    vx, vy, leg = 6.0, 18.0, 14.0
    rad = 60.0 * math.pi / 180.0
    _line(dl, _pt(ox, oy, vx, vy), _pt(ox, oy, vx + leg, vy), ink, 1.6)
    _line(
        dl,
        _pt(ox, oy, vx, vy),
        _pt(ox, oy, vx + leg * math.cos(rad), vy - leg * math.sin(rad)),
        ink,
        1.6,
    )
    _arc(dl, ox, oy, vx, vy, 7, 7, 300, 60, accent, 1.7, 14)


def symbol_drawer(symbol):
    def draw(dl, ox, oy, ink, accent):
        try:
            dl.AddText((ox + 4, oy + 3), accent, symbol)
        except Exception:
            _line(dl, _pt(ox, oy, 5, 12), _pt(ox, oy, 19, 12), ink, 1.6)
    return draw


def _imgui_size(imgui, width, height):
    ctor = getattr(imgui, "ImVec2", None)
    if ctor is not None:
        try:
            return ctor(float(width), float(height))
        except Exception:
            pass
    return (float(width), float(height))


def _invisible_button(imgui, button_id, width, height):
    size = _imgui_size(imgui, width, height)
    clicked = False
    try:
        clicked = imgui.InvisibleButton(button_id, size)
    except TypeError:
        try:
            clicked = imgui.InvisibleButton(button_id, size, 0)
        except Exception:
            clicked = imgui.Button(button_id, size)
    if isinstance(clicked, tuple):
        clicked = clicked[0]
    return bool(clicked)


def icon_button(imgui, tool_id, label, drawer, active):
    p = _pos(imgui)
    clicked = _invisible_button(imgui, "##sk_" + tool_id, BTN_W, BTN_H)
    hovered = False
    try:
        hovered = bool(imgui.IsItemHovered())
    except Exception:
        pass
    dl = _dl(imgui)
    if active:
        ink = _u32(imgui, INK_ON_ACCENT)
        label_col = INK_ON_ACCENT
        bg = _u32(imgui, (0.18, 0.42, 0.82), 0.95)
    elif hovered:
        ink = _u32(imgui, INK)
        label_col = INK
        bg = _u32(imgui, (0.78, 0.80, 0.84), 0.95)
    else:
        ink = _u32(imgui, INK)
        label_col = INK
        bg = _u32(imgui, (0.88, 0.88, 0.90), 0.94)
    accent = _u32(imgui, ACCENT)
    x1, y1 = p[0] + BTN_W, p[1] + BTN_H
    try:
        dl.AddRectFilled(p, (x1, y1), bg, 5.0)
    except TypeError:
        dl.AddRectFilled(p, (x1, y1), bg)
    tw, th = _text_size(imgui, label)
    ox = p[0] + (BTN_W - ICON) * 0.5
    avail = BTN_H - th - 8
    oy = p[1] + max(4.0, (avail - ICON) * 0.5)
    drawer(dl, ox, oy, ink, accent)
    try:
        dl.AddText(
            (p[0] + (BTN_W - tw) * 0.5, p[1] + BTN_H - th - 5),
            _u32(imgui, label_col),
            label)
    except Exception:
        pass
    if hovered:
        try:
            imgui.SetTooltip(label)
        except Exception:
            pass
    return clicked


def dimension_button_width():
    return (CONTENT_W - ITEM_GAP) * 0.5


def dimension_editor(imgui, kind, value, focus=False):
    """Value box that stays inside the 3-column palette width.

    A default ImGui InputDouble is wider than PANEL_W (label + ~250px field).
    That forces a horizontal scrollbar, which shrinks the content region and
    wraps the icon grid from 3 columns to 2.
    """
    caption = (kind or "value").replace("_", " ").title()
    wrapped_text(imgui, caption)
    changed = False
    new_value = float(value)
    pushed = False
    try:
        imgui.PushItemWidth(float(CONTENT_W))
        pushed = True
    except Exception:
        pushed = False
    if focus:
        try:
            imgui.SetKeyboardFocusHere()
        except Exception:
            pass
    try:
        result = imgui.InputDouble("##sk_dimension", new_value, 0.0, 0.0, "%.4f")
        if isinstance(result, tuple):
            changed = bool(result[0])
            new_value = float(result[1])
        else:
            new_value = float(result)
            changed = abs(new_value - float(value)) > 1e-15
    except Exception:
        try:
            result = imgui.InputFloat("##sk_dimension", float(new_value))
            if isinstance(result, tuple):
                changed = bool(result[0])
                new_value = float(result[1])
        except Exception:
            pass
    if pushed:
        try:
            imgui.PopItemWidth()
        except Exception:
            pass
    apply_w = dimension_button_width()
    apply_clicked = action_button(imgui, "Apply", width=apply_w, button_id="dimension_apply")
    imgui.SameLine()
    cancel_clicked = action_button(imgui, "Cancel", width=apply_w, button_id="dimension_cancel")
    return changed, new_value, apply_clicked, cancel_clicked


def action_button(imgui, label, width=78, height=26, button_id=None):
    p = _pos(imgui)
    clicked = _invisible_button(imgui, "##sk_act_" + (button_id or label), width, height)
    hovered = False
    try:
        hovered = bool(imgui.IsItemHovered())
    except Exception:
        pass
    dl = _dl(imgui)
    bg = _u32(imgui, (0.78, 0.80, 0.84) if hovered else (0.88, 0.88, 0.90), 0.95)
    try:
        dl.AddRectFilled(p, (p[0] + width, p[1] + height), bg, 5.0)
    except TypeError:
        dl.AddRectFilled(p, (p[0] + width, p[1] + height), bg)
    try:
        tw, th = _text_size(imgui, label)
        dl.AddText(
            (p[0] + (width - tw) * 0.5, p[1] + (height - th) * 0.5),
            _u32(imgui, INK),
            label)
    except Exception:
        pass
    return clicked


def _push_style_var(imgui, name, value):
    push = getattr(imgui, "PushStyleVar", None)
    if push is None:
        return False
    idx = getattr(imgui, name, None)
    if idx is None:
        return False
    try:
        push(int(idx), value)
        return True
    except Exception:
        try:
            push(idx, value)
            return True
        except Exception:
            return False


def _push_palette_style(imgui):
    count = 0
    if _push_style_var(imgui, "ImGuiStyleVar_WindowPadding", _imgui_size(imgui, PAD, 10)):
        count += 1
    if _push_style_var(imgui, "ImGuiStyleVar_ItemSpacing", _imgui_size(imgui, ITEM_GAP, 6)):
        count += 1
    return count


def _pop_palette_style(imgui, count):
    if not count:
        return
    pop = getattr(imgui, "PopStyleVar", None)
    if pop is None:
        return
    try:
        pop(int(count))
    except TypeError:
        for _ in range(int(count)):
            try:
                pop()
            except Exception:
                pass


_PALETTE_STYLE_COUNT = [0]


def begin_palette(imgui, title="Sketch"):
    _PALETTE_STYLE_COUNT[0] = _push_palette_style(imgui)
    try:
        io = imgui.GetIO()
        size = io.DisplaySize
        w = float(size[0] if hasattr(size, "__len__") else size.x)
        imgui.SetNextWindowPos((w - 10, 10), cond=1, pivot=(1.0, 0.0))
    except Exception:
        try:
            imgui.SetNextWindowPos((10, 10), cond=1)
        except Exception:
            pass
    try:
        imgui.SetNextWindowSize((PANEL_W, 0), cond=1)
    except Exception:
        pass
    try:
        imgui.SetNextWindowSizeConstraints(
            _imgui_size(imgui, PANEL_W, 0),
            _imgui_size(imgui, PANEL_W, 10000),
        )
    except Exception:
        pass
    flags = 0
    for name in (
        "ImGuiWindowFlags_NoCollapse",
        "ImGuiWindowFlags_AlwaysAutoResize",
        "ImGuiWindowFlags_NoClose",
        "ImGuiWindowFlags_NoScrollbar",
        "ImGuiWindowFlags_NoScrollWithMouse",
    ):
        flags |= int(getattr(imgui, name, 0) or 0)
    try:
        return imgui.Begin(title, flags)
    except TypeError:
        try:
            return imgui.Begin(title)
        except TypeError:
            return imgui.Begin(title)


def end_palette(imgui):
    end = getattr(imgui, "End", None)
    if end is not None:
        end()
    _pop_palette_style(imgui, _PALETTE_STYLE_COUNT[0])
    _PALETTE_STYLE_COUNT[0] = 0


def _text_width(imgui, text):
    w, _h = _text_size(imgui, text)
    return w


def _wrap_to_width(imgui, text, wrap_w):
    """Break long names (pipe:face-Name) so they stay inside wrap_w."""
    if not text:
        return ""
    wrap_w = max(float(wrap_w), 8.0)
    if _text_width(imgui, text) <= wrap_w:
        return text
    breaks = set(" :/-_.")
    lines = []
    current = ""
    i = 0
    while i < len(text):
        ch = text[i]
        trial = current + ch
        if _text_width(imgui, trial) <= wrap_w or not current:
            current = trial
            i += 1
            continue
        cut = 0
        for j, c in enumerate(current):
            if c in breaks:
                cut = j + 1
        if cut > 0:
            lines.append(current[:cut].rstrip())
            current = current[cut:]
            continue
        lines.append(current)
        current = ""
    if current:
        lines.append(current.rstrip())
    return "\n".join(lines)


def wrapped_text(imgui, text):
    if text is None:
        text = ""
    wrap_w = float(CONTENT_W)
    try:
        avail = imgui.GetContentRegionAvail()
        aw = float(avail[0] if hasattr(avail, "__len__") else avail.x)
        if 8.0 < aw < wrap_w + 1.0:
            wrap_w = aw
    except Exception:
        pass
    text = _wrap_to_width(imgui, text, wrap_w)
    try:
        imgui.PushTextWrapPos(imgui.GetCursorPosX() + wrap_w)
    except Exception:
        try:
            imgui.PushTextWrapPos(wrap_w)
        except Exception:
            pass
    try:
        imgui.TextUnformatted(text)
    except Exception:
        try:
            imgui.Text(text)
        except Exception:
            pass
    try:
        imgui.PopTextWrapPos()
    except Exception:
        pass


CURVE_TOOLS = (
    ("select", "Select", draw_select),
    ("line", "Line", draw_line),
    ("polyline", "Multi", draw_polyline),
    ("arc", "Arc", draw_arc),
    ("circle", "Circle", draw_circle),
    ("rectangle", "Rect", draw_rectangle),
)

CONSTRAINT_TOOLS = (
    ("coincident", "Coinc", draw_coincident),
    ("horizontal", "Horiz", draw_horizontal),
    ("vertical", "Vert", draw_vertical),
    ("parallel", "Para", draw_parallel),
    ("perpendicular", "Perp", draw_perpendicular),
    ("tangent", "Tang", draw_tangent),
    ("equal", "Equal", draw_equal),
    ("midpoint", "Mid", draw_midpoint),
    ("concentric", "Conc", draw_concentric),
    ("fix", "Fix", draw_fix),
)

DIMENSION_TOOLS = (
    ("length", "Length", draw_length),
    ("radius", "Radius", draw_radius),
    ("distance", "Dist", draw_distance),
    ("angle", "Angle", draw_angle),
)
