"""Sketch icons drawn with pyglet primitives; no image assets required."""

import math

BTN_W = 60
BTN_H = 60
ICON = 24
COLS = 3
PAD = 12
ITEM_GAP = 8
CONTENT_W = COLS * BTN_W + (COLS - 1) * ITEM_GAP
PANEL_W = PAD * 2 + CONTENT_W


def _rgba(rgb, alpha=1.0):
    return tuple(round(max(0.0, min(1.0, channel))*255) for channel in (*rgb, alpha))


def _dl(ui):
    return ui.GetWindowDrawList()


def _pos(ui):
    return ui.GetCursorScreenPos()


def _pt(ox, oy, x, y, s=ICON):
    return (ox + x * s / 24.0, oy + y * s / 24.0)


def _text_size(ui, text):
    try:
        size = ui.CalcTextSize(text)
        if hasattr(size, "__len__"):
            return float(size[0]), float(size[1])
        return float(size.x), float(size.y)
    except Exception:
        return len(text) * 7.0, 14.0


def _line(dl, a, b, col, th=1.6):
    dl.AddLine(a, b, col, _stroke(dl, th))


def _stroke(dl, thickness):
    # Keep ribbon-size outlines legible without making larger icons heavier.
    return max(thickness, 1.25 / max(getattr(dl, "scale", 1), 1e-6))


def _dot(dl, c, col, r=2.0):
    dl.AddCircleFilled(c, r, col, 12)


def _circ(dl, c, r, col, th=1.6, segs=28):
    try:
        dl.AddCircle(c, r, col, segs, _stroke(dl, th))
    except TypeError:
        dl.AddCircle(c, r, col, segs)


def _arc(dl, ox, oy, cx, cy, rx, ry, start_deg, sweep_deg, col, th=1.6, n=20):
    # Screen Y-down: standard cos/sin matches GDI+ (clockwise from east).
    start = start_deg * math.pi / 180.0
    sweep = sweep_deg * math.pi / 180.0
    if hasattr(dl, "AddEllipseArc"):
        dl.AddEllipseArc(_pt(ox, oy, cx, cy), (rx, ry), start, sweep, col, _stroke(dl, th))
        return
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
    _arc(dl, ox, oy, 12, 15, 9, 8, 200, 140, ink)
    for parameter in (0, .5, 1):
        t = start + sweep*parameter
        _dot(dl, (cx + rx*math.cos(t), cy + ry*math.sin(t)), accent, 2.0)


def draw_rectangle(dl, ox, oy, ink, accent):
    a = _pt(ox, oy, 4, 6)
    b = _pt(ox, oy, 20, 18)
    dl.AddRect(a, b, ink, 0.0, 0, _stroke(dl, 1.6))


def draw_profile(dl, ox, oy, ink, accent):
    _arc(dl, ox, oy, 12, 17, 8, 11, 180, 180, ink)
    for point in ((4, 17), (12, 6), (20, 17)):
        _dot(dl, _pt(ox, oy, *point), accent, 1.8)


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
    _return_arrow(dl, ox, oy, ink, accent, reverse=False)


def draw_redo(dl, ox, oy, ink, accent):
    _return_arrow(dl, ox, oy, ink, accent, reverse=True)


def draw_cancel(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 6, 6), _pt(ox, oy, 18, 18), ink, 1.6)
    _line(dl, _pt(ox, oy, 18, 6), _pt(ox, oy, 6, 18), ink, 1.6)


def _return_arrow(dl, ox, oy, ink, accent, *, reverse):
    tip, tail = (20, 16) if reverse else (4, 8)
    _line(dl, _pt(ox, oy, tip, 8), _pt(ox, oy, 12, 8), ink, 1.6)
    _arc(dl, ox, oy, 12, 14, 6, 6, -90, -180 if reverse else 180, ink, 1.6)
    _line(dl, _pt(ox, oy, tip, 8), _pt(ox, oy, tail, 4), accent, 1.6)
    _line(dl, _pt(ox, oy, tip, 8), _pt(ox, oy, tail, 12), accent, 1.6)


def draw_select(dl, ox, oy, ink, accent):
    points = [_pt(ox, oy, *point) for point in
              ((5, 3), (5, 19), (9, 15), (13, 21), (16, 19), (12, 13), (20, 12))]
    for i in range(len(points)):
        _line(dl, points[i], points[(i + 1) % len(points)], ink, 1.6)


def draw_coincident(dl, ox, oy, ink, accent):
    r = 4.0 * ICON / 24.0
    c0 = _pt(ox, oy, 9, 12)
    c1 = _pt(ox, oy, 15, 12)
    _circ(dl, c0, r, ink, 1.5)
    _circ(dl, c1, r, ink, 1.5)
    _dot(dl, _pt(ox, oy, 12, 12), accent, 1.5)


def draw_horizontal(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 3, 12), _pt(ox, oy, 21, 12), ink, 1.6)
    _dot(dl, _pt(ox, oy, 4, 12), accent, 2.2)
    _dot(dl, _pt(ox, oy, 20, 12), accent, 2.2)


def draw_vertical(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 12, 3), _pt(ox, oy, 12, 21), ink, 1.6)
    _dot(dl, _pt(ox, oy, 12, 4), accent, 2.2)
    _dot(dl, _pt(ox, oy, 12, 20), accent, 2.2)


def draw_parallel(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 5, 10), _pt(ox, oy, 19, 8), ink, 1.6)
    _line(dl, _pt(ox, oy, 5, 16), _pt(ox, oy, 19, 14), ink, 1.6)


def draw_perpendicular(dl, ox, oy, ink, accent):
    _line(dl, _pt(ox, oy, 5, 19), _pt(ox, oy, 5, 5), ink, 1.6)
    _line(dl, _pt(ox, oy, 5, 19), _pt(ox, oy, 19, 19), ink, 1.6)
    a = _pt(ox, oy, 5, 15)
    b = _pt(ox, oy, 9, 19)
    try:
        dl.AddRect(a, b, accent, 0.0, 0, _stroke(dl, 1.6))
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
    for x in (4, 20):
        _line(dl, _pt(ox, oy, x, 10), _pt(ox, oy, x, 14), ink, 1.5)
    _dot(dl, _pt(ox, oy, 12, 12), accent, 2.2)


def draw_concentric(dl, ox, oy, ink, accent):
    c = _pt(ox, oy, 12, 12)
    _circ(dl, c, 8.0 * ICON / 24.0, ink, 1.6)
    _circ(dl, c, 4.0 * ICON / 24.0, accent, 1.7)


def draw_fix(dl, ox, oy, ink, accent):
    a = _pt(ox, oy, 7, 11)
    b = _pt(ox, oy, 17, 19)
    try:
        dl.AddRect(a, b, ink, 0.0, 0, _stroke(dl, 1.6))
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


def _invisible_button(ui, button_id, width, height):
    return ui.InvisibleButton(button_id, (width, height))


_TOOLTIPS = {
    "profile": "Create or edit an ellipse, spline, gear, airfoil or text outline using coordinates and dimensions.",
    "select": "Select curves or drag sketch handles.",
    "line": "Draw a line between two points.",
    "polyline": "Draw connected line segments.",
    "arc": "Draw an arc through three points.",
    "circle": "Draw a circle from its center and radius.",
    "rectangle": "Draw a rectangle from opposite corners.",
    "con_coincident": "Make two points occupy the same position.",
    "con_horizontal": "Keep a line horizontal.",
    "con_vertical": "Keep a line vertical.",
    "con_parallel": "Keep two lines parallel.",
    "con_perpendicular": "Keep two lines at right angles.",
    "con_tangent": "Make line tangent to circle/arc. Coincident joins ends; Radius holds size.",
    "con_equal": "Match two line lengths or circular radii.",
    "con_midpoint": "Place a point at a line's midpoint.",
    "con_concentric": "Give two circles or arcs the same center.",
    "con_fix": "Keep a point at its current position.",
    "dim_length": "Set a line's length.",
    "dim_radius": "Set a circle or arc radius.",
    "dim_distance": "Set the distance between two points.",
    "dim_angle": "Set the angle between two lines.",
    "fit": "Center the view on the sketch plane.",
    "undo": "Undo the last sketch action.",
    "redo": "Restore the last undone sketch edit.",
}


def icon_button(ui, tool_id, label, drawer, active):
    p = _pos(ui)
    clicked = _invisible_button(ui, "##sk_" + tool_id, BTN_W, BTN_H)
    hovered = False
    try:
        hovered = bool(ui.IsItemHovered())
    except Exception:
        pass
    dl = _dl(ui)
    ink = ui.theme["text"]
    bg = ui.theme["selected" if active else "hover" if hovered else "control"]
    accent = ui.theme["accent"]
    x1, y1 = p[0] + BTN_W, p[1] + BTN_H
    try:
        dl.AddRectFilled(p, (x1, y1), bg, 5.0)
    except TypeError:
        dl.AddRectFilled(p, (x1, y1), bg)
    tw, th = _text_size(ui, label)
    ox = p[0] + (BTN_W - ICON) * 0.5
    avail = BTN_H - th - 8
    oy = p[1] + max(4.0, (avail - ICON) * 0.5)
    drawer(dl, ox, oy, ink, accent)
    try:
        dl.AddText(
            (p[0] + (BTN_W - tw) * 0.5, p[1] + BTN_H - th - 5),
            ink,
            label)
    except Exception:
        pass
    if hovered:
        try:
            ui.SetTooltip(_TOOLTIPS.get(tool_id, label))
        except Exception:
            pass
    return clicked


def dimension_button_width():
    return (CONTENT_W - ITEM_GAP) * 0.5


RELATION_LABELS = {
    "point_on_line": "Point on line", "point_on_circle": "Point on circle",
    "tangent_circles": "Tangent circles", "horizontal_distance": "Horizontal offset",
    "vertical_distance": "Vertical offset", "distance_to_line": "Distance to line",
    "coincident_on_curve": "Point at curve parameter",
}


def relation_label(kind):
    return RELATION_LABELS.get(kind, (kind or "Value").replace("_", " ").capitalize())


def dimension_error(kind, value):
    if not math.isfinite(value):
        return "Enter a finite number"
    if kind == "coincident_on_curve":
        return "Curve parameter must be between 0 and 1" if not 0 <= value <= 1 else ""
    if kind in ("horizontal_distance", "vertical_distance"):
        return ""
    if kind == "distance_to_line":
        return "Distance must be nonnegative" if value < 0 else ""
    return "Enter a number greater than zero" if value <= 0 else ""


def dimension_editor(ui, kind, value, focus=False):
    """Value box spanning the current panel, with Apply and Cancel beneath it.

    Enter while the value field is focused applies a valid number.
    """
    caption = relation_label(kind)
    wrapped_text(ui, caption)
    changed = False
    new_value = float(value)
    width = float(CONTENT_W)
    try:
        avail = ui.GetContentRegionAvail()
        aw = float(avail[0] if hasattr(avail, "__len__") else avail.x)
        if aw > 1.0:
            width = aw
    except Exception:
        pass
    pushed = False
    try:
        ui.PushItemWidth(width)
        pushed = True
    except Exception:
        pushed = False
    if focus:
        try:
            ui.SetKeyboardFocusHere()
        except Exception:
            pass
    try:
        result = ui.InputDouble("##sk_dimension", new_value, 0.0, 0.0, "%.4f",
                               error=dimension_error(kind, new_value))
        if isinstance(result, tuple):
            changed = bool(result[0])
            new_value = float(result[1])
        else:
            new_value = float(result)
            changed = abs(new_value - float(value)) > 1e-15
    except Exception:
        try:
            result = ui.InputFloat("##sk_dimension", float(new_value))
            if isinstance(result, tuple):
                changed = bool(result[0])
                new_value = float(result[1])
        except Exception:
            pass
    if pushed:
        try:
            ui.PopItemWidth()
        except Exception:
            pass
    submitted = False
    try:
        submitted = bool(ui.IsItemSubmitted())
    except Exception:
        submitted = False
    valid = ui.IsItemValid() and not dimension_error(kind, new_value)
    apply_w = dimension_button_width()
    apply_clicked = action_button(ui, "Apply", width=apply_w,
                                  button_id="dimension_apply", primary=True, enabled=valid)
    if submitted and valid:
        apply_clicked = True
    ui.SameLine()
    cancel_clicked = action_button(ui, "Cancel", width=apply_w, button_id="dimension_cancel")
    if not valid:
        ui.ValidationMessage(dimension_error(kind, new_value) or "Enter a valid number")
    return changed, new_value, apply_clicked, cancel_clicked, valid


def action_button(ui, label, width=78, height=26, button_id=None, primary=False, enabled=True):
    return ui.Button(label + "##sk_act_" + (button_id or label), width, height,
                     primary=primary, enabled=enabled)


def begin_palette(ui, title="Sketch"):
    ui.PushStyleVar(ui.ImGuiStyleVar_WindowPadding, (PAD, 10))
    ui.PushStyleVar(ui.ImGuiStyleVar_ItemSpacing, (ITEM_GAP, 6))
    width = ui.GetIO().DisplaySize[0]
    ui.SetNextWindowPos((width-10, 10), ui.ImGuiCond_Always, (1.0, 0.0))
    ui.SetNextWindowSize((PANEL_W, 0), ui.ImGuiCond_Always)
    ui.SetNextWindowSizeConstraints((PANEL_W, 0), (PANEL_W, 10000))
    return ui.Begin(title, ui.ImGuiWindowFlags_NoCollapse |
                    ui.ImGuiWindowFlags_AlwaysAutoResize)


def end_palette(ui):
    ui.End()
    ui.PopStyleVar(2)


def _text_width(ui, text):
    w, _h = _text_size(ui, text)
    return w


def _wrap_to_width(ui, text, wrap_w):
    """Break long names (pipe:face-Name) so they stay inside wrap_w."""
    if not text:
        return ""
    wrap_w = max(float(wrap_w), 8.0)
    if _text_width(ui, text) <= wrap_w:
        return text
    breaks = set(" :/-_.")
    lines = []
    current = ""
    i = 0
    while i < len(text):
        ch = text[i]
        trial = current + ch
        if _text_width(ui, trial) <= wrap_w or not current:
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


def wrapped_text(ui, text):
    if text is None:
        text = ""
    wrap_w = float(CONTENT_W)
    try:
        avail = ui.GetContentRegionAvail()
        aw = float(avail[0] if hasattr(avail, "__len__") else avail.x)
        if 8.0 < aw < wrap_w + 1.0:
            wrap_w = aw
    except Exception:
        pass
    text = _wrap_to_width(ui, text, wrap_w)
    try:
        ui.PushTextWrapPos(ui.GetCursorPosX() + wrap_w)
    except Exception:
        try:
            ui.PushTextWrapPos(wrap_w)
        except Exception:
            pass
    try:
        ui.TextUnformatted(text)
    except Exception:
        try:
            ui.Text(text)
        except Exception:
            pass
    try:
        ui.PopTextWrapPos()
    except Exception:
        pass


CURVE_TOOLS = (
    ("select", "Select", draw_select),
    ("line", "Line", draw_line),
    ("polyline", "Multi", draw_polyline),
    ("arc", "Arc", draw_arc),
    ("circle", "Circle", draw_circle),
    ("rectangle", "Rect", draw_rectangle),
    ("profile", "Profile", draw_profile),
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
