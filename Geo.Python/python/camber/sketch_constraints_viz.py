"""C# GeoAPIVisualizer-style sketch constraint and dimension glyphs.

Layout is 2D in the sketch plane. The GUI lifts segments into the same
camera-facing overlays as the sketch strokes and draws labels in screen space.
"""

import math

from . import naming as _naming
from . import pick as _pick

CONSTRAINT_COLOR = (0.7, 0.2, 0.8)
DIMENSION_COLOR = (1.0, 0.5, 0.0)
KIND_CONS = "cons"
KIND_DIM = "dim"

_GEOM_KINDS = ("line", "circle", "arc", "rectangle")
_MINIMUM_OFFSET = 0.01
_OFFSET_PERCENT = 0.05
_GOLDEN_RATIO = 1.61803398875


DIMENSION_KINDS = ("length", "radius", "distance", "angle")
LABEL_SCALE = 4.5


class ConstraintOverlay(object):
    def __init__(self):
        self.cons_segs = []
        self.dim_segs = []
        self.arrows = []
        self.labels = []
        self.hit_segs = []
        self.action_index = -1

    def cons_seg(self, a, b):
        self.cons_segs.append((a, b))
        self.hit_segs.append((a, b, self.action_index, KIND_CONS))

    def dim_seg(self, a, b):
        self.dim_segs.append((a, b))
        self.hit_segs.append((a, b, self.action_index, KIND_DIM))

    def label(self, text, xy, kind):
        self.labels.append((text, xy, kind, self.action_index))


def build_constraint_overlay(actions, curves):
    """Return glyphs for recorded constraint actions using solved curve geometry."""
    overlay = ConstraintOverlay()
    index = 0
    for action_index, action in enumerate(actions or ()):
        kind = action.get("kind")
        if kind in _GEOM_KINDS:
            continue
        overlay.action_index = action_index
        try:
            _visualize_action(overlay, action, curves or [], index)
        except Exception:
            pass
        index += 1
    return overlay


def nearest_constraint(overlay, uv, label_radius=0.25, seg_tol=0.12, label_kinds=None, seg_kinds=None):
    """Action index of the closest label, else closest glyph segment, or -1."""
    best = -1
    best_d = float(label_radius)
    for _text, xy, kind, action_index in overlay.labels:
        if label_kinds is not None and kind not in label_kinds:
            continue
        d = _dist(uv, xy)
        if d <= best_d:
            best_d = d
            best = action_index
    if best >= 0:
        return best
    best_d = float(seg_tol)
    for item in overlay.hit_segs:
        a, b, action_index = item[0], item[1], item[2]
        kind = item[3] if len(item) > 3 else None
        if seg_kinds is not None and kind not in seg_kinds:
            continue
        d = _point_seg_dist(uv, a, b)
        if d <= best_d:
            best_d = d
            best = action_index
    return best


def nearest_dimension(overlay, uv, label_radius=0.45, seg_tol=0.25):
    """Action index of the closest numeric dimension glyph, or -1."""
    return nearest_constraint(
        overlay, uv,
        label_radius=label_radius,
        seg_tol=seg_tol,
        label_kinds=(KIND_DIM,),
        seg_kinds=(KIND_DIM,),
    )


def is_dimension_action(action):
    return action is not None and action.get("kind") in DIMENSION_KINDS


def label_offset(constraint_index, average_length):
    length = max(_MINIMUM_OFFSET, float(average_length) * _OFFSET_PERCENT)
    angle = 2.0 * math.pi * ((int(constraint_index) * _GOLDEN_RATIO) % 1.0)
    return (math.cos(angle) * length, math.sin(angle) * length)


def point_xy(point, curves):
    if point is None:
        raise ValueError("missing point")
    if isinstance(point, str):
        return _named_point_xy(point, curves)
    if point[0] == "xy":
        return (float(point[1]), float(point[2]))
    index = int(point[0])
    role = int(point[1])
    action = curves[index]
    for handle_role, uv in _pick.sketch_handle_uvs(action):
        if handle_role == role:
            return uv
    raise ValueError("invalid sketch point reference")


def _named_point_xy(name, curves):
    owner, local = _naming.parse_qualified(name)
    local = local or name
    if _naming.is_origin_name(local):
        return (0.0, 0.0)
    parsed = _naming.parse_sketch_curve_address(local)
    if parsed is None:
        raise ValueError("invalid sketch point reference")
    action = _curve(curves, parsed["curve"])
    role = _role_from_address(action.get("kind"), parsed)
    for handle_role, uv in _pick.sketch_handle_uvs(action):
        if handle_role == role:
            return uv
    raise ValueError("invalid sketch point reference")


def _role_from_address(kind, parsed):
    if parsed.get("center"):
        return 2
    uniform = float(parsed.get("uniform") or 0.0)
    if kind == "circle":
        if abs(uniform - 0.25) < 1e-9:
            return 4
        if abs(uniform - 0.5) < 1e-9:
            return 5
        if abs(uniform - 0.75) < 1e-9:
            return 6
        return 3
    if abs(uniform - 0.5) < 1e-9:
        return 3
    if abs(uniform - 1.0) < 1e-9:
        return 1
    return 0


def format_length(value):
    return "{0:.2f}".format(float(value))


def format_radius(value):
    return "R{0:.4g}".format(float(value))


def format_angle_deg(value):
    return "{0:.1f}\u00b0".format(float(value))


def _visualize_action(overlay, action, curves, index):
    kind = action.get("kind")
    if kind == "horizontal":
        _viz_hv(overlay, curves, action, index, "H")
    elif kind == "vertical":
        _viz_hv(overlay, curves, action, index, "V")
    elif kind == "coincident":
        _viz_coincident(overlay, curves, action, index)
    elif kind == "fix":
        _viz_fix(overlay, curves, action, index)
    elif kind == "parallel":
        _viz_parallel(overlay, curves, action, index)
    elif kind == "perpendicular":
        _viz_perpendicular(overlay, curves, action, index)
    elif kind == "equal":
        _viz_equal(overlay, curves, action, index)
    elif kind == "midpoint":
        _viz_midpoint(overlay, curves, action, index)
    elif kind == "concentric":
        _viz_concentric(overlay, curves, action, index)
    elif kind == "tangent":
        _viz_tangent(overlay, curves, action, index)
    elif kind == "length":
        _viz_length(overlay, curves, action, index)
    elif kind == "distance":
        _viz_distance(overlay, curves, action, index)
    elif kind == "radius":
        _viz_radius(overlay, curves, action, index)
    elif kind == "angle":
        _viz_angle(overlay, curves, action, index)


def _viz_hv(overlay, curves, action, index, symbol):
    line = _curve(curves, action["curves"][0])
    start, end = _line_ends(line)
    anchor = _mid(start, end)
    _cons_label(overlay, symbol, anchor, _line_len(line), index)


def _viz_coincident(overlay, curves, action, index):
    p0, p1 = action["points"][0], action["points"][1]
    a = point_xy(p0, curves)
    b = point_xy(p1, curves)
    symbol, anchor = _coincident_symbol(p0, p1, a, b, curves)
    length = 0.0
    if symbol == "PoL":
        line = _derived_curve(p0, p1, curves, "line")
        length = _line_len(line) * 0.5 if line is not None else 0.0
    elif symbol == "PoC":
        circ = _derived_curve(p0, p1, curves, "circle")
        if circ is not None:
            length = 2.0 * math.pi * float(circ["radius"])
    elif symbol == "?":
        arc = _derived_curve(p0, p1, curves, "arc")
        if arc is not None:
            length = _arc_length(arc)
    _cons_label(overlay, symbol, anchor, length, index)


def _viz_fix(overlay, curves, action, index):
    anchor = point_xy(action["points"][0], curves)
    _cons_label(overlay, "PoP", anchor, 0.0, index)


def _viz_parallel(overlay, curves, action, index):
    _ = index
    _parallel_marks(overlay, _curve(curves, action["curves"][0]))
    _parallel_marks(overlay, _curve(curves, action["curves"][1]))


def _viz_perpendicular(overlay, curves, action, index):
    _ = index
    line1 = _curve(curves, action["curves"][0])
    line2 = _curve(curves, action["curves"][1])
    s1, e1 = _line_ends(line1)
    s2, e2 = _line_ends(line2)
    len1 = _dist(s1, e1)
    len2 = _dist(s2, e2)
    if len1 < 1e-10 or len2 < 1e-10:
        return
    apex = _intersect_supporting(s1, e1, s2, e2)
    if apex is None:
        apex = _mid(_mid(s1, e1), _mid(s2, e2))
    u = _unit_toward(apex, _mid(s1, e1), _vsub(e1, s1))
    v = _unit_toward(apex, _mid(s2, e2), _vsub(e2, s2))
    size = max(_MINIMUM_OFFSET * 6.0, min(len1, len2) * 0.14)
    a = _vadd(apex, _vmul(u, size))
    corner = _vadd(a, _vmul(v, size))
    c = _vadd(apex, _vmul(v, size))
    overlay.cons_seg(a, corner)
    overlay.cons_seg(corner, c)


def _viz_equal(overlay, curves, action, index):
    c0 = _curve(curves, action["curves"][0])
    c1 = _curve(curves, action["curves"][1])
    if c0["kind"] == "line" and c1["kind"] == "line":
        mid1 = _mid(*_line_ends(c0))
        mid2 = _mid(*_line_ends(c1))
        avg = (_line_len(c0) + _line_len(c1)) * 0.5
        _cons_label(overlay, "=", mid1, avg, index)
        _cons_label(overlay, "=", mid2, avg, index)
        return
    if c0["kind"] in ("circle", "arc") and c1["kind"] in ("circle", "arc"):
        center1, r1 = _circular(c0)
        center2, r2 = _circular(c1)
        avg = (r1 + r2) * 0.5
        _cons_label(overlay, "R=", center1, avg, index)
        _cons_label(overlay, "R=", center2, avg, index)


def _viz_midpoint(overlay, curves, action, index):
    point = point_xy(action["points"][0], curves)
    line = _curve(curves, action["curves"][0])
    _cons_label(overlay, "M", point, _line_len(line), index)


def _viz_concentric(overlay, curves, action, index):
    _ = index
    center, r1 = _circular(_curve(curves, action["curves"][0]))
    _center2, r2 = _circular(_curve(curves, action["curves"][1]))
    r_mark = max(_MINIMUM_OFFSET * 5.0, min(r1, r2) * 0.12)
    _ring(overlay, center, r_mark, 24)
    _ring(overlay, center, r_mark * 1.7, 24)
    c = r_mark * 0.55
    overlay.cons_seg((center[0] - c, center[1]), (center[0] + c, center[1]))
    overlay.cons_seg((center[0], center[1] - c), (center[0], center[1] + c))


def _viz_tangent(overlay, curves, action, index):
    a = _curve(curves, action["curves"][0])
    b = _curve(curves, action["curves"][1])
    line = a if a["kind"] == "line" else b if b["kind"] == "line" else None
    circular = a if a["kind"] in ("circle", "arc") else b if b["kind"] in ("circle", "arc") else None
    if line is not None and circular is not None:
        _viz_tangent_line_circle(overlay, line, circular)
        return
    if a["kind"] in ("circle", "arc") and b["kind"] in ("circle", "arc"):
        c1, r1 = _circular(a)
        c2, r2 = _circular(b)
        anchor = _mid(c1, c2)
        _cons_label(overlay, "T", anchor, (r1 + r2), index)


def _viz_tangent_line_circle(overlay, line, circular):
    start, end = _line_ends(line)
    center, _radius = _circular(circular)
    line_dir = _unit(_vsub(end, start))
    to_center = _vsub(center, start)
    contact = _vadd(start, _vmul(line_dir, _dot(to_center, line_dir)))
    rad = _vsub(contact, center)
    r_len = _len(rad)
    if r_len < 1e-12:
        return
    rad = _vmul(rad, 1.0 / r_len)
    tang = (-rad[1], rad[0])
    mark = max(_MINIMUM_OFFSET * 5.0, _line_len(line) * 0.08)
    overlay.cons_seg(_vadd(contact, _vmul(tang, -mark)), _vadd(contact, _vmul(tang, mark)))
    overlay.cons_seg(
        _vadd(contact, _vmul(rad, -mark * 0.55)),
        _vadd(contact, _vmul(rad, mark * 0.15)),
    )


def _viz_length(overlay, curves, action, index):
    line = _curve(curves, action["curves"][0])
    p1, p2 = _line_ends(line)
    value = float(action["value"])
    _distance_2d(overlay, p1, p2, value, (0.0, 0.05 * value), index)


def _viz_distance(overlay, curves, action, index):
    p1 = point_xy(action["points"][0], curves)
    p2 = point_xy(action["points"][1], curves)
    value = float(action["value"])
    _distance_2d(overlay, p1, p2, value, (0.0, 0.05 * value), index)


def _viz_radius(overlay, curves, action, index):
    _ = index
    curve = _curve(curves, action["curves"][0])
    center, radius = _circular(curve)
    mid = _radius_rim(curve, center, radius)
    overlay.dim_seg(center, mid)
    overlay.label(format_radius(action["value"]), _mid(center, mid), KIND_DIM)


def _viz_angle(overlay, curves, action, index):
    _ = index
    line1 = _curve(curves, action["curves"][0])
    line2 = _curve(curves, action["curves"][1])
    s1, e1 = _line_ends(line1)
    s2, e2 = _line_ends(line2)
    d1 = _vsub(e1, s1)
    d2 = _vsub(e2, s2)
    len1 = _len(d1)
    len2 = _len(d2)
    if len1 < 1e-10 or len2 < 1e-10:
        return
    d1 = _vmul(d1, 1.0 / len1)
    d2 = _vmul(d2, 1.0 / len2)
    apex = _intersect_supporting(s1, e1, s2, e2)
    if apex is None:
        apex = _mid(_mid(s1, e1), _mid(s2, e2))
    radius = max(_MINIMUM_OFFSET * 8.0, min(len1, len2) * 0.28)
    r1 = _vsub(_mid(s1, e1), apex)
    r2 = _vsub(_mid(s2, e2), apex)
    if _len(r1) < 1e-12:
        r1 = d1
    if _len(r2) < 1e-12:
        r2 = d2
    r1 = _unit(r1)
    r2 = _unit(r2)
    a_start = math.atan2(r1[1], r1[0])
    a_end = math.atan2(r2[1], r2[0])
    sweep = a_end - a_start
    while sweep > math.pi:
        sweep -= 2.0 * math.pi
    while sweep < -math.pi:
        sweep += 2.0 * math.pi
    wanted = abs(math.radians(float(action["value"])))
    alt = sweep - 2.0 * math.pi if sweep >= 0.0 else sweep + 2.0 * math.pi
    if abs(abs(alt) - wanted) < abs(abs(sweep) - wanted):
        sweep = alt
    segments = int(math.ceil(abs(sweep) / (math.pi / 24.0)))
    segments = max(8, min(48, segments))
    pts = []
    for i in range(segments + 1):
        a = a_start + sweep * (i / float(segments))
        pts.append((apex[0] + math.cos(a) * radius, apex[1] + math.sin(a) * radius))
    for i in range(len(pts) - 1):
        overlay.dim_seg(pts[i], pts[i + 1])
    tick = radius * 0.15
    for ang in (a_start, a_start + sweep):
        direction = (math.cos(ang), math.sin(ang))
        overlay.dim_seg(
            _vadd(apex, _vmul(direction, radius - tick)),
            _vadd(apex, _vmul(direction, radius + tick)),
        )
    mid_ang = a_start + sweep * 0.5
    label = _vadd(apex, (math.cos(mid_ang) * radius * 1.15, math.sin(mid_ang) * radius * 1.15))
    overlay.label(format_angle_deg(action["value"]), label, KIND_DIM)


def _distance_2d(overlay, p1, p2, distance, anchor_offset, index):
    _ = index
    direction = _vsub(p2, p1)
    length = _len(direction)
    if length < 1e-10:
        return
    direction = _vmul(direction, 1.0 / length)
    perp = (-direction[1], direction[0])
    dim_offset = float(anchor_offset[1])
    dim_p1 = _vadd(p1, _vmul(perp, dim_offset))
    dim_p2 = _vadd(p2, _vmul(perp, dim_offset))
    center = _vadd(_mid(dim_p1, dim_p2), _vmul(direction, float(anchor_offset[0])))
    overlay.label(format_length(distance), center, KIND_DIM)
    if abs(dim_offset) > 1e-6:
        overlay.dim_seg(p1, dim_p1)
        overlay.dim_seg(p2, dim_p2)
    overlay.dim_seg(dim_p1, dim_p2)
    overlay.arrows.append((dim_p1, direction))
    overlay.arrows.append((dim_p2, _vmul(direction, -1.0)))


def _cons_label(overlay, text, anchor, average_length, index):
    pos = _vadd(anchor, label_offset(index, average_length))
    overlay.label(text, pos, KIND_CONS)
    overlay.cons_seg(anchor, pos)


def _parallel_marks(overlay, line):
    start, end = _line_ends(line)
    direction = _vsub(end, start)
    length = _len(direction)
    if length < 1e-10:
        return
    direction = _vmul(direction, 1.0 / length)
    normal = (-direction[1], direction[0])
    mid = _mid(start, end)
    half = max(_MINIMUM_OFFSET * 4.0, length * 0.07)
    slash = _unit((_vadd(_vmul(direction, 0.55), normal)))
    slash = _vmul(slash, half)
    sep = half * 0.55
    for sign in (-1.0, 1.0):
        center = _vadd(mid, _vmul(direction, sign * sep))
        overlay.cons_seg(_vsub(center, slash), _vadd(center, slash))


def _coincident_symbol(p0, p1, a, b, curves):
    for point, xy in ((p0, a), (p1, b)):
        symbol = _derived_symbol(point, curves)
        if symbol:
            return symbol, xy
    return "PoP", _mid(a, b)


def _derived_symbol(point, curves):
    curve, role = _point_curve_role(point, curves)
    if curve is None:
        return None
    kind = curve.get("kind")
    if kind == "circle" and role in (3, 4, 5, 6):
        return "PoC"
    if kind == "line" and role == 3:
        return "PoL"
    if kind == "arc" and role == 3:
        return "?"
    return None


def _derived_curve(p0, p1, curves, kind):
    for point in (p0, p1):
        curve, _role = _point_curve_role(point, curves)
        if curve is not None and curve.get("kind") == kind:
            return curve
    return None


def _point_curve_role(point, curves):
    if point is None:
        return None, None
    if isinstance(point, str):
        owner, local = _naming.parse_qualified(point)
        local = local or point
        if _naming.is_origin_name(local):
            return None, None
        parsed = _naming.parse_sketch_curve_address(local)
        if parsed is None:
            return None, None
        action = _curve(curves, parsed["curve"])
        return action, _role_from_address(action.get("kind"), parsed)
    if point[0] == "xy":
        return None, None
    index = int(point[0])
    role = int(point[1])
    if index < 0 or index >= len(curves):
        return None, None
    return curves[index], role


def _curve(curves, index):
    if isinstance(index, str):
        owner, local = _naming.parse_qualified(index)
        key = local or index
        if "@" in key:
            key = key.rsplit("@", 1)[0]
        for i, action in enumerate(curves):
            if _pick.sketch_curve_display_name(action, i) == key:
                return action
        raise ValueError("unknown sketch curve {0!r}".format(index))
    return curves[int(index)]


def _line_ends(action):
    return action["p0"], action["p1"]


def _line_len(action):
    return _dist(*_line_ends(action))


def _circular(action):
    if action["kind"] == "circle":
        return action["center"], float(action["radius"])
    return _arc_center_radius(action)


def _radius_rim(action, center, radius):
    if action["kind"] == "circle":
        return (center[0] - radius, center[1])
    if action["kind"] == "arc":
        return action["mid"]
    return (center[0] + radius, center[1])


def _arc_center_radius(action):
    a, b, c = action["start"], action["mid"], action["end"]
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return a, max(_dist(a, b), _dist(b, c))
    ux = (
        (a[0] * a[0] + a[1] * a[1]) * (b[1] - c[1])
        + (b[0] * b[0] + b[1] * b[1]) * (c[1] - a[1])
        + (c[0] * c[0] + c[1] * c[1]) * (a[1] - b[1])
    ) / d
    uy = (
        (a[0] * a[0] + a[1] * a[1]) * (c[0] - b[0])
        + (b[0] * b[0] + b[1] * b[1]) * (a[0] - c[0])
        + (c[0] * c[0] + c[1] * c[1]) * (b[0] - a[0])
    ) / d
    center = (ux, uy)
    return center, _dist(a, center)


def _arc_length(action):
    center, radius = _arc_center_radius(action)
    a0 = math.atan2(action["start"][1] - center[1], action["start"][0] - center[0])
    a1 = math.atan2(action["mid"][1] - center[1], action["mid"][0] - center[0])
    a2 = math.atan2(action["end"][1] - center[1], action["end"][0] - center[0])
    return abs(_pick._arc_sweep(a0, a1, a2)) * radius


def _intersect_supporting(s1, e1, s2, e2):
    d1 = _vsub(e1, s1)
    d2 = _vsub(e2, s2)
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        return None
    delta = _vsub(s2, s1)
    t = (delta[0] * d2[1] - delta[1] * d2[0]) / cross
    return _vadd(s1, _vmul(d1, t))


def _unit_toward(origin, target, fallback):
    direction = _vsub(target, origin)
    length = _len(direction)
    if length > 1e-12:
        return _vmul(direction, 1.0 / length)
    return _unit(fallback)


def _ring(overlay, center, radius, n):
    pts = []
    for i in range(n + 1):
        ang = 2.0 * math.pi * i / n
        pts.append((center[0] + math.cos(ang) * radius, center[1] + math.sin(ang) * radius))
    for i in range(len(pts) - 1):
        overlay.cons_seg(pts[i], pts[i + 1])


def _point_seg_dist(p, a, b):
    ab = _vsub(b, a)
    length = _len(ab)
    if length < 1e-12:
        return _dist(p, a)
    t = _dot(_vsub(p, a), ab) / (length * length)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return _dist(p, _vadd(a, _vmul(ab, t)))


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _vmul(a, s):
    return (a[0] * s, a[1] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _len(a):
    return math.hypot(a[0], a[1])


def _dist(a, b):
    return _len(_vsub(a, b))


def _mid(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)


def _unit(a, fallback=(1.0, 0.0)):
    length = _len(a)
    if length < 1e-12:
        return fallback
    return _vmul(a, 1.0 / length)
