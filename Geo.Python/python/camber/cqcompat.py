"""CadQuery-shaped fluent API on top of camber (Workplane / Sketch).

This is not a drop-in CadQuery replacement. Solids are camber meshes with named
patches; some selectors and modeling modes remain unsupported.

Typical use::

    from camber.cqcompat import Workplane

    result = (
        Workplane("XY")
        .box(2, 2, 1)
        .faces(">Z")
        .circle(0.25)
        .extrude(0.4)
    )
    result.val().save_stl("boss.stl")
"""

import math
import re

from .api import (
    LOFT_STYLE_RULED,
    Curve3D,
    Frame,
    LoftOptions,
    Part,
    Solid,
    Surface,
)
from .vec import vec2, vec3

Vector = vec3
from .cqassembly import Assembly, Color, Location

_AXES = {
    "X": (1.0, 0.0, 0.0),
    "Y": (0.0, 1.0, 0.0),
    "Z": (0.0, 0.0, 1.0),
}

# CadQuery Plane.named() — origin + xDir + normal.
_NAMED_PLANES = {
    "XY": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "YZ": ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
    "ZX": ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    "XZ": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "YX": ((0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
    "ZY": ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    "FRONT": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "BACK": ((-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    "LEFT": ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    "RIGHT": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
    "TOP": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "BOTTOM": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
}

_SELECTOR_SPLIT = re.compile(r"\s+and\s+|\s+", re.IGNORECASE)


def named_plane(name, origin=None):
    """Return a camber Frame for a CadQuery plane name (``XY``, ``front``, …)."""
    key = (name or "XY").strip().upper()
    if key not in _NAMED_PLANES:
        raise ValueError("unknown workplane {0!r}".format(name))
    xdir, normal = _NAMED_PLANES[key]
    origin = vec3(0, 0, 0) if origin is None else vec3(origin)
    x = _unit(xdir)
    z = _unit(normal)
    y = _cross(z, x)
    if _norm(y) < 1e-12:
        raise ValueError("degenerate plane {0!r}".format(name))
    y = _unit(y)
    x = _cross(y, z)
    return Frame(origin, x, y, z)


def _dot(a, b):
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def _cross(a, b):
    return vec3(
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def _norm(v):
    return math.sqrt(_dot(v, v))


def _unit(v):
    n = _norm(v)
    if n < 1e-15:
        return vec3(0, 0, 0)
    return vec3(float(v[0]) / n, float(v[1]) / n, float(v[2]) / n)


def _xy2(point):
    if isinstance(point, vec2):
        return (point.x, point.y)
    if isinstance(point, vec3):
        return (point.x, point.y)
    return (float(point[0]), float(point[1]))


def _sample_parametric_curve(func, N, start, stop, tolerance):
    """Sample a planar callback, refining chords until sampled error is within tolerance."""
    if not callable(func):
        raise TypeError("parametricCurve func must be callable")
    if isinstance(N, bool) or int(N) != N or int(N) < 2:
        raise ValueError("parametricCurve N must be an integer >= 2")
    N = int(N)
    start, stop, tolerance = float(start), float(stop), float(tolerance)
    if not math.isfinite(start) or not math.isfinite(stop) or start == stop:
        raise ValueError("parametricCurve requires a finite, nonzero parameter interval")
    if not math.isfinite(stop - start):
        raise ValueError("parametricCurve parameter interval is too large")
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("parametricCurve tolerance must be finite and > 0")

    max_samples = 100000
    if N + 1 > max_samples:
        raise ValueError("parametricCurve exceeded the 100000 sample limit")
    max_depth = 16
    evaluated = {}

    def evaluate(t):
        if t in evaluated:
            return evaluated[t]
        if len(evaluated) >= max_samples:
            raise ValueError("parametricCurve exceeded the 100000 sample limit")
        try:
            value = func(t)
        except Exception as exc:
            raise ValueError("parametricCurve callback failed at t={0!r}".format(t)) from exc
        if isinstance(value, (vec2, vec3)):
            if isinstance(value, vec3) and abs(float(value.z)) > tolerance:
                raise NotImplementedError("parametricCurve currently accepts planar 2D points only")
            point = (float(value.x), float(value.y))
        else:
            try:
                if len(value) != 2:
                    raise NotImplementedError("parametricCurve currently accepts planar 2D points only")
                point = (float(value[0]), float(value[1]))
            except TypeError as exc:
                raise TypeError("parametricCurve callback must return a 2D point") from exc
        if not all(math.isfinite(coordinate) for coordinate in point):
            raise ValueError("parametricCurve callback returned a non-finite point at t={0!r}".format(t))
        evaluated[t] = point
        return point

    def point_chord_distance(point, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 == 0:
            return math.hypot(point[0] - a[0], point[1] - a[1])
        fraction = max(0.0, min(1.0,
            ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
        return math.hypot(point[0] - a[0] - fraction * dx,
                          point[1] - a[1] - fraction * dy)

    def refine(t0, p0, t1, p1, depth):
        span = t1 - t0
        probes = [t0 + span * f for f in (0.25, 0.5, 0.75)]
        if max(point_chord_distance(evaluate(t), p0, p1) for t in probes) <= tolerance:
            return [p1]
        if depth >= max_depth:
            raise ValueError("parametricCurve could not meet tolerance before maximum subdivision depth")
        tm = probes[1]
        pm = evaluate(tm)
        return refine(t0, p0, tm, pm, depth + 1) + refine(tm, pm, t1, p1, depth + 1)

    parameters = [start + (stop - start) * i / N for i in range(N + 1)]
    points = [evaluate(parameters[0])]
    for t0, t1 in zip(parameters, parameters[1:]):
        points.extend(refine(t0, evaluate(t0), t1, evaluate(t1), 0))
    return points


def _as_vec3(point):
    if isinstance(point, vec3):
        return point
    if point is None:
        return vec3(0, 0, 0)
    if isinstance(point, (int, float)):
        return vec3(point)
    if len(point) == 2:
        return vec3(float(point[0]), float(point[1]), 0.0)
    return vec3(point)


def _center_shifts(sizes, centered):
    """Local lower-corner offset so a prism of `sizes` matches CadQuery centering."""
    n = len(sizes)
    if centered is True:
        flags = (True,) * n
    elif centered is False:
        flags = (False,) * n
    else:
        flags = tuple(bool(v) for v in centered)
        if len(flags) != n:
            raise ValueError("centered must have one flag per axis")
    return tuple((-0.5 * float(s) if flag else 0.0) for s, flag in zip(sizes, flags))


def _frame_plus_local(frame, local):
    origin = frame.origin + float(local[0]) * frame.x + float(local[1]) * frame.y + float(local[2]) * frame.z
    return Frame(origin, frame.x, frame.y, frame.z)


def _world_from_local(frame, local):
    return frame.origin + float(local[0]) * frame.x + float(local[1]) * frame.y + float(local[2]) * frame.z


def _rotate_vec(v, axis, angle):
    if abs(angle) < 1e-15:
        return vec3(v)
    k = _unit(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    return v * c + _cross(k, v) * s + k * _dot(k, v) * (1.0 - c)


def _rotate_frame(frame, degrees):
    rx, ry, rz = (math.radians(float(d)) for d in degrees)
    x, y, z = vec3(frame.x), vec3(frame.y), vec3(frame.z)
    if rx:
        y, z = _rotate_vec(y, x, rx), _rotate_vec(z, x, rx)
    if ry:
        x, z = _rotate_vec(x, y, ry), _rotate_vec(z, y, ry)
    if rz:
        x, y = _rotate_vec(x, z, rz), _rotate_vec(y, z, rz)
    return Frame(frame.origin, _unit(x), _unit(y), _unit(z))


def _plane_from_normal(origin, normal, hint_x):
    z = _unit(normal)
    x = vec3(hint_x) - z * _dot(hint_x, z)
    if _norm(x) < 1e-9:
        fallback = min(_AXES.values(), key=lambda axis: abs(_dot(axis, z)))
        x = vec3(fallback) - z * _dot(fallback, z)
    x = _unit(x)
    y = _unit(_cross(z, x))
    x = _unit(_cross(y, z))
    return Frame(origin, x, y, z)


def _project_to_plane(point, origin, normal):
    n = _unit(normal)
    return vec3(point) - n * _dot(vec3(point) - vec3(origin), n)


def _dist2(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return dx * dx + dy * dy


def _looks_like_name(selector):
    if not selector:
        return False
    return selector[0] not in "><|+#-#%"


def _parse_selector(selector):
    """Split a CadQuery string selector into ``(op, arg)`` predicates."""
    if selector is None or str(selector).strip() == "":
        return []
    text = str(selector).strip()
    if _looks_like_name(text):
        return [("name", text)]
    preds = []
    for token in _SELECTOR_SPLIT.split(text):
        if not token:
            continue
        if token[0] == "%":
            preds.append(("type", token[1:].upper()))
            continue
        if len(token) >= 2 and token[0] in "><|+#-#" and token[1:].upper() in _AXES:
            preds.append((token[0], token[1:].upper()))
            continue
        raise ValueError("unsupported selector {0!r}".format(selector))
    return preds


def _extreme(items, axis, sign, key="center"):
    return max(items, key=lambda item: sign * _dot(item[key], axis))


def _select_faces(faces, selector):
    """Filter a registered face list with a CadQuery string selector."""
    return _select_items(faces, selector, kind="face")


def _select_edges(edges, selector):
    """Filter a registered edge list with a CadQuery string selector."""
    return _select_items(edges, selector, kind="edge")


def _select_items(items, selector, kind):
    items = list(items or [])
    text = str(selector).strip() if selector is not None else ""
    if re.search(r"\s+and\s+", text, re.IGNORECASE):
        clauses = re.split(r"\s+and\s+", text, flags=re.IGNORECASE)
        names = None
        for clause in clauses:
            matched = {item["name"] for item in _select_items(items, clause.strip(" ()"), kind)}
            names = matched if names is None else names & matched
        return [item for item in items if item["name"] in names]
    if text.lower().startswith("not "):
        names = {item["name"] for item in _select_items(items, text[4:].strip(" ()"), kind)}
        return [item for item in items if item["name"] not in names]
    ranked = re.fullmatch(r"(>>|>|<)([XYZ])\[(\d+)\]", text, re.IGNORECASE)
    if ranked:
        axis = _AXES[ranked.group(2).upper()]
        ordered = sorted(items, key=lambda item: _dot(item["center"], axis),
                         reverse=ranked.group(1) != "<")
        index = int(ranked.group(3))
        tolerance = max((item.get("rank_tolerance", 1e-7) for item in ordered), default=1e-7)
        ranks = []
        for item in ordered:
            value = _dot(item["center"], axis)
            if not ranks or abs(value - ranks[-1]) > tolerance:
                ranks.append(value)
        return ([item for item in ordered
                 if abs(_dot(item["center"], axis) - ranks[index]) <= tolerance]
                if index < len(ranks) else [])
    if re.fullmatch(r">>[XYZ]", text, re.IGNORECASE):
        axis = _AXES[text[-1].upper()]
        if not items:
            return []
        extreme = max(_dot(item["center"], axis) for item in items)
        tolerance = max(item.get("rank_tolerance", 1e-7) for item in items)
        return [item for item in items
                if abs(_dot(item["center"], axis) - extreme) <= tolerance]
    if selector and re.search(r"\s+or\s+", str(selector), re.IGNORECASE):
        chosen = []
        names = set()
        for clause in re.split(r"\s+or\s+", str(selector), flags=re.IGNORECASE):
            for item in _select_items(items, clause, kind):
                if item["name"] not in names:
                    names.add(item["name"])
                    chosen.append(item)
        return chosen
    preds = _parse_selector(selector)
    if not preds:
        return items
    if preds[0][0] == "name":
        name = preds[0][1]
        return [
            item for item in items
            if item["name"] == name or item["name"].endswith("-" + name) or name in item["name"]
        ]
    chosen = items
    for pred in preds:
        if not chosen:
            return []
        op, arg = pred
        if op == "type":
            wanted = arg.lower()
            chosen = [item for item in chosen if (item.get("kind") or "").lower() == wanted]
            continue
        axis = _AXES[arg]
        if op == ">":
            chosen = [_extreme(chosen, axis, 1.0)]
        elif op == "<":
            chosen = [_extreme(chosen, axis, -1.0)]
        elif op == "+":
            field = "normal" if kind == "face" else "direction"
            chosen = [item for item in chosen if _dot(item[field], axis) > 0.5]
        elif op == "-":
            field = "normal" if kind == "face" else "direction"
            chosen = [item for item in chosen if _dot(item[field], axis) < -0.5]
        elif op == "|":
            if kind == "face":
                chosen = [item for item in chosen if item.get("kind") == "plane" and
                          abs(_dot(item["normal"], axis)) < 0.35]
            else:
                chosen = [item for item in chosen if abs(_dot(item["direction"], axis)) > 0.85]
        elif op == "#":
            if kind == "face":
                chosen = [item for item in chosen if item.get("kind") == "plane" and
                          abs(_dot(item["normal"], axis)) > 0.85]
            else:
                chosen = [item for item in chosen if abs(_dot(item["direction"], axis)) < 0.35]
        else:
            raise ValueError("unsupported selector token {0!r}".format(op))
    return chosen


def _radius_arc_mid(start, end, radius):
    """Mid-point of the minor arc from ``start`` to ``end`` with signed radius.

    Positive radius puts the center to the left of the directed chord (CCW).
    """
    x0, y0 = _xy2(start)
    x1, y1 = _xy2(end)
    dx, dy = x1 - x0, y1 - y0
    chord = math.hypot(dx, dy)
    r = float(radius)
    if chord < 1e-15:
        raise ValueError("radiusArc endpoints coincide")
    if abs(r) + 1e-12 < 0.5 * chord:
        raise ValueError("radius too small for radiusArc")
    h = math.sqrt(max(r * r - 0.25 * chord * chord, 0.0))
    nx, ny = -dy / chord, dx / chord
    if r < 0:
        h = -h
    mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    return (mx + (h - abs(r)) * nx, my + (h - abs(r)) * ny)


def _prism_faces(name, frame, xmin, xmax, ymin, ymax, zmin, zmax):
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    faces = [
        {"name": name + "-ExtrudeTop", "center": _world_from_local(frame, (cx, cy, zmax)),
         "normal": vec3(frame.z), "kind": "plane"},
        {"name": name + "-ExtrudeBottom", "center": _world_from_local(frame, (cx, cy, zmin)),
         "normal": -vec3(frame.z), "kind": "plane"},
        {"name": name + "-south", "center": _world_from_local(frame, (cx, ymin, cz)),
         "normal": -vec3(frame.y), "kind": "plane"},
        {"name": name + "-north", "center": _world_from_local(frame, (cx, ymax, cz)),
         "normal": vec3(frame.y), "kind": "plane"},
        {"name": name + "-west", "center": _world_from_local(frame, (xmin, cy, cz)),
         "normal": -vec3(frame.x), "kind": "plane"},
        {"name": name + "-east", "center": _world_from_local(frame, (xmax, cy, cz)),
         "normal": vec3(frame.x), "kind": "plane"},
    ]
    edges = [
        {"name": "[{0}-south,{0}-west]".format(name),
         "direction": vec3(frame.z), "center": _world_from_local(frame, (xmin, ymin, cz)), "kind": "line"},
        {"name": "[{0}-south,{0}-east]".format(name),
         "direction": vec3(frame.z), "center": _world_from_local(frame, (xmax, ymin, cz)), "kind": "line"},
        {"name": "[{0}-east,{0}-north]".format(name),
         "direction": vec3(frame.z), "center": _world_from_local(frame, (xmax, ymax, cz)), "kind": "line"},
        {"name": "[{0}-north,{0}-west]".format(name),
         "direction": vec3(frame.z), "center": _world_from_local(frame, (xmin, ymax, cz)), "kind": "line"},
        {"name": "[{0}-south,{0}-ExtrudeTop]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (cx, ymin, zmax)), "kind": "line"},
        {"name": "[{0}-north,{0}-ExtrudeTop]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (cx, ymax, zmax)), "kind": "line"},
        {"name": "[{0}-west,{0}-ExtrudeTop]".format(name),
         "direction": vec3(frame.y), "center": _world_from_local(frame, (xmin, cy, zmax)), "kind": "line"},
        {"name": "[{0}-east,{0}-ExtrudeTop]".format(name),
         "direction": vec3(frame.y), "center": _world_from_local(frame, (xmax, cy, zmax)), "kind": "line"},
        {"name": "[{0}-south,{0}-ExtrudeBottom]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (cx, ymin, zmin)), "kind": "line"},
        {"name": "[{0}-north,{0}-ExtrudeBottom]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (cx, ymax, zmin)), "kind": "line"},
        {"name": "[{0}-west,{0}-ExtrudeBottom]".format(name),
         "direction": vec3(frame.y), "center": _world_from_local(frame, (xmin, cy, zmin)), "kind": "line"},
        {"name": "[{0}-east,{0}-ExtrudeBottom]".format(name),
         "direction": vec3(frame.y), "center": _world_from_local(frame, (xmax, cy, zmin)), "kind": "line"},
    ]
    return faces, edges


def _revolve_caps(name, frame, z0, z1, radius, side_name="side"):
    cz = 0.5 * (z0 + z1)
    faces = [
        {"name": name + "-ExtrudeTop", "center": _world_from_local(frame, (0.0, 0.0, z1)),
         "normal": vec3(frame.z), "kind": "plane"},
        {"name": name + "-ExtrudeBottom", "center": _world_from_local(frame, (0.0, 0.0, z0)),
         "normal": -vec3(frame.z), "kind": "plane"},
        {"name": name + "-side", "center": _world_from_local(frame, (radius, 0.0, cz)),
         "normal": vec3(frame.x), "kind": "cylinder"},
    ]
    edges = [
        {"name": "[{0}-{1},{0}-ExtrudeTop]".format(name, side_name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (0.0, 0.0, z1)), "kind": "circle"},
        {"name": "[{0}-{1},{0}-ExtrudeBottom]".format(name, side_name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (0.0, 0.0, z0)), "kind": "circle"},
    ]
    return faces, edges


class _Wire(object):
    def __init__(self, segments, kind="path", name=None, mode=None, tag=None,
                 construction=False):
        self.segments = list(segments)
        self.kind = kind
        self.name = name
        self.mode = mode
        self.tag = tag
        self.construction = bool(construction)

    @staticmethod
    def circle(center, radius, name=None):
        return _Wire([("circle", _xy2(center), float(radius))], kind="circle", name=name)

    @staticmethod
    def ellipse(center, x_radius, y_radius, rotation=0.0, name=None):
        return _Wire([("ellipse", _xy2(center), float(x_radius), float(y_radius),
                       float(rotation))], kind="ellipse", name=name)

    @staticmethod
    def rect(xmin, xmax, ymin, ymax, names=None):
        if names is None:
            wire = _Wire.from_points(
                [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)],
                closed=True,
            )
            wire.kind = "rect"
            return wire
        south, east, north, west = names
        return _Wire([
            ("line", (xmin, ymin), (xmax, ymin), south),
            ("line", (xmax, ymin), (xmax, ymax), east),
            ("line", (xmax, ymax), (xmin, ymax), north),
            ("line", (xmin, ymax), (xmin, ymin), west),
        ], kind="rect")

    @staticmethod
    def from_points(points, closed=True, name=None):
        pts = [_xy2(p) for p in points]
        if closed and pts and _dist2(pts[0], pts[-1]) > 1e-18:
            pts = pts + [pts[0]]
        segs = [("line", pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        return _Wire(segs, kind="path", name=name)

    def translated(self, dx, dy):
        segs = []
        for seg in self.segments:
            if seg[0] == "circle":
                c = seg[1]
                segs.append(("circle", (c[0] + dx, c[1] + dy), seg[2]))
            elif seg[0] == "ellipse":
                c = seg[1]
                segs.append(("ellipse", (c[0] + dx, c[1] + dy), seg[2], seg[3], seg[4]))
            elif seg[0] == "line":
                extra = (seg[3],) if len(seg) > 3 else ()
                segs.append(("line", (seg[1][0] + dx, seg[1][1] + dy),
                             (seg[2][0] + dx, seg[2][1] + dy)) + extra)
            elif seg[0] == "arc":
                segs.append((
                    "arc",
                    (seg[1][0] + dx, seg[1][1] + dy),
                    (seg[2][0] + dx, seg[2][1] + dy),
                    (seg[3][0] + dx, seg[3][1] + dy),
                ))
            elif seg[0] == "spline":
                segs.append(("spline", [(x + dx, y + dy) for x, y in seg[1]]))
            elif seg[0] == "sampled":
                segs.append(("sampled", [(x + dx, y + dy) for x, y in seg[1]]))
            else:
                segs.append(seg)
        return _Wire(segs, kind=self.kind, name=self.name, mode=self.mode,
                     tag=self.tag, construction=self.construction)

    def mapped_xy(self, map_point):
        """Map a wire through a rigid change of 2D sketch coordinates."""
        segs = []
        for seg in self.segments:
            if seg[0] == "circle":
                segs.append(("circle", map_point(seg[1]), seg[2]))
            elif seg[0] == "ellipse":
                center, rx, ry, angle = seg[1:]
                major = (center[0] + rx * math.cos(angle), center[1] + rx * math.sin(angle))
                minor = (center[0] - ry * math.sin(angle), center[1] + ry * math.cos(angle))
                mapped_center = map_point(center)
                mapped_major = map_point(major)
                mapped_minor = map_point(minor)
                major_vector = (mapped_major[0] - mapped_center[0],
                                mapped_major[1] - mapped_center[1])
                minor_vector = (mapped_minor[0] - mapped_center[0],
                                mapped_minor[1] - mapped_center[1])
                segs.append(("ellipse", mapped_center, math.hypot(*major_vector),
                             math.hypot(*minor_vector),
                             math.atan2(major_vector[1], major_vector[0])))
            elif seg[0] == "line":
                extra = (seg[3],) if len(seg) > 3 else ()
                segs.append(("line", map_point(seg[1]), map_point(seg[2])) + extra)
            elif seg[0] == "arc":
                segs.append(("arc", map_point(seg[1]), map_point(seg[2]), map_point(seg[3])))
            elif seg[0] == "spline":
                segs.append(("spline", [map_point(p) for p in seg[1]]))
            elif seg[0] == "sampled":
                segs.append(("sampled", [map_point(p) for p in seg[1]]))
            else:
                segs.append(seg)
        return _Wire(segs, kind=self.kind, name=self.name, mode=self.mode,
                     tag=self.tag, construction=self.construction)

    def draw(self, sketch):
        if self.construction:
            return []
        curves = []
        for i, seg in enumerate(self.segments):
            if seg[0] == "circle":
                curves.append(sketch.add_circle(seg[1], seg[2], name=self.name))
            elif seg[0] == "ellipse":
                curves.append(sketch.add_ellipse(seg[1], (seg[2], seg[3]),
                                                 rotation=seg[4], name=self.name))
            elif seg[0] == "line":
                name = seg[3] if len(seg) > 3 else (self.name if i == 0 else None)
                curves.append(sketch.add_line(seg[1], seg[2], name=name))
            elif seg[0] == "arc":
                name = seg[4] if len(seg) > 4 and seg[4] else self.name
                curves.append(sketch.add_arc(seg[1], seg[2], seg[3], name=name))
            elif seg[0] == "spline":
                curves.append(sketch.add_spline(seg[1], name=self.name))
            elif seg[0] == "sampled":
                curves.append(sketch.add_sampled_curve(seg[1], name=self.name))
        return curves


class Sketch(object):
    """CadQuery-shaped 2D sketch. ``finalize()`` returns the parent Workplane."""

    def __init__(self, parent=None):
        self._parent = parent if parent is not None else Workplane("XY")
        self._offset = (0.0, 0.0, 0.0)
        self._wires = []
        self._selected = None
        self._selected_vertex_points = None
        self._tags = {}
        self._cursor = (0.0, 0.0)
        self._open = None
        self._open_names = []
        self._open_curves = []
        self._constraints = []

    @staticmethod
    def _mode(mode):
        if mode is None:
            return None
        names = {"a": "a", "add": "a", "s": "s", "subtract": "s",
                 "i": "i", "intersect": "i", "r": "r", "remove": "r",
                 "c": "c", "construction": "c"}
        try:
            return names[str(mode).lower()]
        except KeyError:
            raise ValueError("sketch mode must be add, subtract, intersect, remove, or construction")

    def _add_wire(self, wire, mode=None, tag=None):
        wire.mode = self._mode(mode)
        wire.tag = None if tag is None else str(tag)
        wire.construction = wire.mode == "c"
        if wire.construction and not wire.tag:
            raise ValueError("construction geometry requires a tag")
        self._wires.append(wire)
        if wire.tag:
            self._tags.setdefault(wire.tag, []).append(wire)
        self._selected = None
        return wire

    def _active_wires(self, tag=None):
        if tag is not None:
            try:
                return list(self._tags[str(tag)])
            except KeyError:
                raise ValueError("unknown sketch tag {0!r}".format(tag))
        return list(self._wires if self._selected is None else self._selected)

    def reset(self):
        """Clear the active selector and return to the complete sketch."""
        self._selected = None
        self._selected_vertex_points = None
        self._parent = self._parent._spawn(_locations=[(0.0, 0.0)])
        return self

    def copy(self):
        result = Sketch(self._parent)
        result._wires = list(self._wires)
        result._selected_vertex_points = (None if self._selected_vertex_points is None
                                          else list(self._selected_vertex_points))
        result._offset = self._offset
        result._tags = {key: list(value) for key, value in self._tags.items()}
        result._cursor = self._cursor
        result._open = None if self._open is None else list(self._open)
        result._open_names = list(self._open_names)
        result._open_curves = list(self._open_curves)
        result._constraints = list(self._constraints)
        return result

    def constrain(self, *args):
        """Record a named sketch constraint for the native Camber solver."""
        if len(args) not in (3, 4):
            raise TypeError("constrain expects (curve, constraint, value) or (curve, other, constraint, value)")
        if len(args) == 3:
            first, kind, value = args
            second = None
        else:
            first, second, kind, value = args
        key = str(kind).replace("_", "").replace(" ", "").lower()
        self._constraints.append((str(first), None if second is None else str(second), key, value))
        return self

    def solve(self):
        """Solve recorded constraints using Camber's native 2D sketch solver."""
        if not self._constraints:
            return self
        wires = list(self._wires)
        if self._open_curves:
            wires.append(_Wire(self._open_curves))
        if not wires:
            raise ValueError("solve needs sketch geometry")
        part = self._parent._ensure_part()
        native = part.sketch(frame=self._parent._frame, constrained=True,
                             name=part._generate_name("cq_constraint"))
        curve_names = {}
        for wire in wires:
            for curve in wire.draw(native):
                curve_names[curve.name] = curve
        coincident_pairs = {}
        for first, second, kind, value in self._constraints:
            if first not in curve_names or (second is not None and second not in curve_names):
                raise ValueError("constraint references an unknown named sketch curve")
            if kind in ("fixed", "fixedpoint"):
                native.fix(first + "@0.000")
            elif kind == "coincident":
                if second is None:
                    raise ValueError("Coincident constraint needs two curves")
                curve_pair = tuple(sorted((first, second)))
                used_pairs = coincident_pairs.setdefault(curve_pair, set())
                endpoints = [first + "@0.000", first + "@1.000",
                             second + "@0.000", second + "@1.000"]
                positions = [native.eval_xy(point) for point in endpoints]
                pairs = []
                for i in range(2):
                    for j in range(2, 4):
                        point_a, point_b = endpoints[i], endpoints[j]
                        pair_key = tuple(sorted((point_a, point_b)))
                        if pair_key in used_pairs:
                            continue
                        distance = math.hypot(positions[i][0] - positions[j][0],
                                              positions[i][1] - positions[j][1])
                        pairs.append((distance, point_a, point_b, pair_key))
                if not pairs:
                    raise ValueError("Coincident constraints already use every endpoint pair for these curves")
                _, point_a, point_b, pair_key = min(pairs, key=lambda pair: pair[0])
                used_pairs.add(pair_key)
                native.coincident(point_a, point_b)
            elif kind == "angle":
                if second is None:
                    raise ValueError("Angle constraint needs two curves")
                first_kind = curve_names[first].kind
                second_kind = curve_names[second].kind
                if first_kind == second_kind == "line":
                    native.angle(first, second, float(value))
                elif {first_kind, second_kind} == {"line", "arc"}:
                    native.angle(first, second, float(value))
                else:
                    raise NotImplementedError("Angle supports lines and line/arc pairs")
            else:
                raise NotImplementedError("constraint kind {0!r} is not yet supported".format(kind))
        native.solve()
        actions = native._solved_actions()
        updated = []
        for action in actions:
            if action["kind"] == "line":
                updated.append(("line", action["p0"], action["p1"], action["name"]))
            elif action["kind"] == "arc":
                updated.append(("arc", action["start"], action["mid"], action["end"], action["name"]))
        if self._open_curves:
            self._open_curves = updated
            self._open = [updated[0][1]]
            for segment in updated:
                self._open.append(segment[2] if segment[0] == "line" else segment[3])
            self._cursor = self._open[-1]
        elif self._wires:
            self._wires = [_Wire(updated)]
        return self

    def wires(self, selector=None):
        """Select closed contours for distribution or offset operations."""
        def is_closed(wire):
            if wire.kind == "circle":
                return True
            if not wire.segments:
                return False
            first = wire.segments[0][1]
            last_segment = wire.segments[-1]
            last = last_segment[3] if last_segment[0] == "arc" else last_segment[2]
            return _dist2(first, last) <= max(self._parent._tolerance, 1e-8) ** 2
        selected = [wire for wire in self._active_wires() if is_closed(wire)]
        if selector:
            raise NotImplementedError("sketch wire selectors are not yet supported")
        self._selected = selected
        return self

    def distribute(self, count):
        """Place locations evenly along the selected wire perimeter."""
        if isinstance(count, bool) or int(count) != count or int(count) < 1:
            raise ValueError("distribute count must be a positive integer")
        wires = self._active_wires()
        if not wires:
            raise ValueError("distribute needs selected sketch wires")
        wire = wires[0]
        n = int(count)
        if wire.kind == "circle":
            center, radius = wire.segments[0][1:]
            locations = [(center[0] + radius * math.cos(2 * math.pi * i / n),
                          center[1] + radius * math.sin(2 * math.pi * i / n))
                         for i in range(n)]
        else:
            samples = []
            for segment in wire.segments:
                if segment[0] == "line":
                    samples.append((segment[1], segment[2]))
                elif segment[0] == "arc":
                    # Uniform arc sampling is adequate here because this method
                    # places points by perimeter length, not for boundary meshing.
                    a, m, b = segment[1:4]
                    samples.extend(((a, m), (m, b)))
                else:
                    raise NotImplementedError("distribute supports line and arc contours")
            lengths = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in samples]
            total = sum(lengths)
            if total <= 1e-12:
                raise ValueError("cannot distribute points on a zero-length wire")
            locations = []
            for i in range(n):
                target = total * i / n
                for (a, b), length in zip(samples, lengths):
                    if target <= length or (a, b) == samples[-1]:
                        t = 0.0 if length <= 1e-15 else target / length
                        locations.append((a[0] + (b[0] - a[0]) * t,
                                          a[1] + (b[1] - a[1]) * t))
                        break
                    target -= length
        self._parent = self._parent._spawn(_locations=locations)
        return self

    def offset(self, distance, mode="a"):
        """Offset selected closed contours using Camber's native sketch offset."""
        selected = self._active_wires()
        if not selected:
            raise ValueError("offset needs selected sketch wires")
        wp = self._parent._spawn(_pending=selected, _locations=[(0.0, 0.0)])
        offset_wp = wp.offset2D(float(distance))
        result = self.copy()
        new_wires = list(offset_wp._pending)
        mode = self._mode(mode)
        if mode == "r":
            result._wires = [wire for wire in result._wires if wire not in selected]
            for wire in new_wires:
                wire.mode = "a"
            result._wires.extend(new_wires)
            result._selected = new_wires
        else:
            for wire in new_wires:
                result._add_wire(wire, mode)
        return result

    def trapezoid(self, width, height, angle=90, mode="a", tag=None):
        """Add an isosceles trapezoid; ``angle`` is the base angle in degrees."""
        width, height, angle = float(width), float(height), float(angle)
        if not all(math.isfinite(v) for v in (width, height, angle)) or width <= 0 or height <= 0 or not 0 < angle < 180:
            raise ValueError("trapezoid needs positive dimensions and an angle between 0 and 180 degrees")
        inset = height / math.tan(math.radians(angle))
        top = width - 2.0 * inset
        if top <= 0:
            raise ValueError("trapezoid angle and dimensions produce a collapsed top edge")
        for ox, oy in self._parent._locations:
            self._add_wire(_Wire.from_points([
                (ox - width / 2.0, oy - height / 2.0),
                (ox + width / 2.0, oy - height / 2.0),
                (ox + top / 2.0, oy + height / 2.0),
                (ox - top / 2.0, oy + height / 2.0),
            ], closed=True), mode, tag)
        return self

    def regularPolygon(self, radius, nSides, angle=0, mode="a", tag=None):
        """Add a regular polygon by circumradius, matching CadQuery's sketch API."""
        radius, angle = float(radius), float(angle)
        if isinstance(nSides, bool) or int(nSides) != nSides or int(nSides) < 3:
            raise ValueError("regularPolygon needs an integer side count >= 3")
        if not math.isfinite(radius) or radius <= 0 or not math.isfinite(angle):
            raise ValueError("regularPolygon radius must be positive and angle finite")
        wires = []
        for ox, oy in self._parent._locations:
            points = []
            for i in range(int(nSides)):
                theta = math.radians(angle) + 2 * math.pi * i / int(nSides)
                points.append((ox + radius * math.cos(theta), oy + radius * math.sin(theta)))
            wires.append(_Wire.from_points(points, closed=True))
        for wire in wires:
            self._add_wire(wire, mode, tag)
        return self

    def segment(self, point1, point2=None, tag=None):
        """Add a line segment; with one point, continue from the previous endpoint."""
        end = _xy2(point1)
        if point2 is None:
            if self._open is None:
                self._open = [end]
                self._open_names = [tag]
                self._open_curves = []
                self._cursor = end
                return self
            start = self._cursor
        else:
            start = _xy2(point1)
            end = _xy2(point2)
            if self._open is None:
                self._open = [start]
                self._open_names = []
                self._open_curves = []
        if self._open is None:
            raise ValueError("segment requires at least one endpoint")
        self._open.append(end)
        self._open_names.append(tag)
        self._open_curves.append(("line", start, end) + ((tag,) if tag else ()))
        self._cursor = end
        return self

    def close(self):
        """Close the current edge chain and store it as one sketch wire."""
        if self._open is None or len(self._open) < 3:
            raise ValueError("close requires at least three points in the current chain")
        segments = list(self._open_curves)
        if _dist2(self._cursor, self._open[0]) > 1e-18:
            segments.append(("line", self._cursor, self._open[0]))
        self._add_wire(_Wire(segments))
        self._cursor = self._open[0]
        self._open = None
        self._open_names = []
        self._open_curves = []
        return self

    def assemble(self, tag=None):
        """Store the current edge chain for later face-based operations."""
        if self._open is not None:
            self.close()
        if tag and self._wires:
            tagged = self._tags.setdefault(str(tag), [])
            for wire in self._wires:
                if wire not in tagged:
                    wire.tag = str(tag)
                    tagged.append(wire)
        return self

    def arc(self, *args):
        """Add an arc by center/radius/angles or by start/mid/end points."""
        if len(args) in (3, 4) and isinstance(args[1], (tuple, list)):
            start, middle, end = map(_xy2, args[:3])
            name = args[3] if len(args) == 4 else None
            arc_segment = ("arc", start, middle, end, name) if name else ("arc", start, middle, end)
            if self._open is not None:
                if _dist2(self._cursor, start) > max(self._parent._tolerance, 1e-8) ** 2:
                    raise ValueError("arc start must meet the current segment endpoint")
                self._open_curves.append(arc_segment)
                self._open.append(end)
            else:
                self._add_wire(_Wire([arc_segment]), tag=name)
            self._cursor = end
            return self
        if len(args) != 4:
            raise TypeError("arc expects (center, radius, startAngle, endAngle) or (start, mid, end, name)")
        center = _xy2(args[0])
        radius, start, end = float(args[1]), float(args[2]), float(args[3])
        if not all(math.isfinite(v) for v in (*center, radius, start, end)) or radius <= 0 or start == end:
            raise ValueError("arc needs a positive radius and finite, distinct angles")
        sweep = end - start
        if abs(sweep) >= 360.0 - 1e-10:
            self._add_wire(_Wire.circle(center, radius))
            return self
        count = max(2, int(math.ceil(abs(sweep) / 5.0)))
        points = []
        for i in range(count + 1):
            theta = math.radians(start + sweep * i / count)
            points.append((center[0] + radius * math.cos(theta),
                           center[1] + radius * math.sin(theta)))
        self._add_wire(_Wire.from_points(points, closed=False))
        return self

    def hull(self):
        """Replace the current circles/segments with their sampled convex hull."""
        points = []
        if self._open is not None:
            points.extend(self._open)
        for wire in self._wires:
            for segment in wire.segments:
                if segment[0] == "circle":
                    center, radius = segment[1], segment[2]
                    points.extend((center[0] + radius * math.cos(2 * math.pi * i / 96),
                                   center[1] + radius * math.sin(2 * math.pi * i / 96))
                                  for i in range(96))
                elif segment[0] == "line":
                    points.extend((segment[1], segment[2]))
                elif segment[0] == "arc":
                    points.extend(segment[1:4])
        points = sorted(set(points))
        if len(points) < 3:
            raise ValueError("hull requires at least three non-collinear points")

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower, upper = [], []
        for point in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-12:
                lower.pop()
            lower.append(point)
        for point in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-12:
                upper.pop()
            upper.append(point)
        hull = lower[:-1] + upper[:-1]
        if len(hull) < 3:
            raise ValueError("hull requires non-collinear geometry")
        self._wires = [_Wire.from_points(hull, closed=True)]
        self._tags = {}
        self._open = None
        return self

    def rect(self, xLen, yLen, centered=True, mode="a", tag=None, angle=0):
        """Add a rectangle, optionally rotated in the sketch plane."""
        angle = float(angle)
        if not math.isfinite(angle):
            raise ValueError("rectangle angle must be finite")
        wires = _rect_wires(xLen, yLen, centered, self._parent._locations)
        if angle:
            ca, sa = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            wires = [wire.mapped_xy(lambda point: (point[0] * ca - point[1] * sa,
                                                    point[0] * sa + point[1] * ca))
                     for wire in wires]
        for wire in wires:
            self._add_wire(wire, mode, tag)
        return self

    def circle(self, radius, mode="a", tag=None):
        """Add a circle at the parent Workplane locations."""
        for wire in _circle_wires(radius, self._parent._locations):
            self._add_wire(wire, mode, tag)
        return self

    def ellipse(self, x_radius, y_radius, rotation_angle=0.0, mode="a", tag=None):
        """Add a closed ellipse at each parent Workplane location."""
        for wire in _ellipse_wires(x_radius, y_radius, rotation_angle, self._parent._locations):
            self._add_wire(wire, mode, tag)
        return self

    def rarray(self, xSpacing, ySpacing, xCount, yCount, center=True):
        """Set sketch locations to a centered rectangular array."""
        self._parent = self._parent.rarray(xSpacing, ySpacing, xCount, yCount, center)
        return self

    def slot(self, length, diameter, mode="a", angle=0, tag=None):
        """Add a closed, round-ended slot at each current sketch location."""
        length, diameter, angle = float(length), float(diameter), float(angle)
        if not all(math.isfinite(value) for value in (length, diameter, angle)) or diameter <= 0 or length < diameter:
            raise ValueError("slot needs finite dimensions with length >= positive diameter")
        half, radius = length * .5, diameter * .5
        straight = half - radius
        ca, sa = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        for ox, oy in self._parent._locations:
            def point(x, y):
                return (ox + x * ca - y * sa, oy + x * sa + y * ca)
            segments = [
                ("line", point(-straight, -radius), point(straight, -radius)),
                ("arc", point(straight, -radius), point(half, 0), point(straight, radius)),
                ("line", point(straight, radius), point(-straight, radius)),
                ("arc", point(-straight, radius), point(-half, 0), point(-straight, -radius)),
            ]
            self._add_wire(_Wire(segments), mode, tag)
        return self

    def clean(self):
        """Compatibility no-op: the profile builder already normalizes loops."""
        return self

    def face(self, other, mode="a"):
        """Combine another sketch's planar regions using a boolean mode."""
        if not isinstance(other, Sketch):
            raise TypeError("face() expects another cqcompat Sketch")
        mode = self._mode(mode)
        for wire in other._wires:
            clone = _Wire(list(wire.segments), kind=wire.kind, name=wire.name,
                          mode=mode, tag=wire.tag, construction=wire.construction)
            self._add_wire(clone, mode, wire.tag)
        return self

    def __sub__(self, other):
        return self.copy().face(other, mode="s")

    def __add__(self, other):
        return self.copy().face(other, mode="a")

    def __and__(self, other):
        return self.copy().face(other, mode="i")

    def push(self, locs):
        """Set 2D locations (same as Workplane.pushPoints)."""
        self._parent = self._parent._spawn(_locations=[_xy2(p) for p in locs])
        return self

    def vertices(self, selector=None, tag=None):
        """Select contour vertices; subsequent primitives are placed at them."""
        wires = self._active_wires(tag)
        vertex_wires = [wire for wire in wires
                        if any(segment[0] in ("line", "arc") for segment in wire.segments)]
        points = []
        for wire in vertex_wires:
            for segment in wire.segments:
                if segment[0] == "line":
                    points.extend((segment[1], segment[2]))
                elif segment[0] == "arc":
                    points.extend((segment[1], segment[3]))
        unique = []
        tolerance = max(self._parent._tolerance, 1e-8)
        for point in points:
            if not any(_dist2(point, prior) <= tolerance * tolerance for prior in unique):
                unique.append(point)
        if selector:
            match = re.fullmatch(r"([<>])([XY]{1,2})", str(selector).strip().upper())
            if not match:
                raise ValueError("unsupported sketch vertex selector {0!r}".format(selector))
            sign = -1 if match.group(1) == "<" else 1
            axes = match.group(2)
            coordinate = lambda p: sum(p[0 if axis == "X" else 1] for axis in axes)
            extreme = max((sign * coordinate(p) for p in unique), default=None)
            if extreme is not None:
                unique = [p for p in unique if abs(sign * coordinate(p) - extreme) <= tolerance]
        self._parent = self._parent._spawn(_locations=unique)
        self._selected = vertex_wires
        self._selected_vertex_points = list(unique)
        return self

    def edges(self, selector=None, tag=None):
        """Select sketch edges and set locations to their midpoints."""
        wires = self._active_wires(tag)
        items = []
        for wi, wire in enumerate(wires):
            for si, segment in enumerate(wire.segments):
                if segment[0] == "line":
                    a, b = segment[1], segment[2]
                    dx, dy = b[0] - a[0], b[1] - a[1]
                    length = math.hypot(dx, dy)
                    if length <= 1e-12:
                        continue
                    kind = "line"
                else:
                    continue
                items.append({"name": "edge{0}-{1}".format(wi, si),
                              "center": ((a[0] + b[0]) * .5, (a[1] + b[1]) * .5, 0.0),
                              "direction": (dx / length, dy / length, 0.0), "kind": kind,
                              "wire": wire})
        chosen = _select_edges(items, selector)
        self._parent = self._parent._spawn(
            _locations=[item["center"] for item in chosen])
        self._selected = list(dict.fromkeys(item["wire"] for item in chosen))
        return self

    def moved(self, x=0, y=0, z=0):
        """Return a copy placed by local XYZ offsets from this sketch plane."""
        result = self.copy()
        result._parent = self._parent._spawn(
            _frame=_frame_plus_local(self._parent._frame, (float(x), float(y), float(z))))
        result._offset = tuple(self._offset[i] + float(value)
                               for i, value in enumerate((x, y, z)))
        return result

    def finalize(self):
        """Return a Workplane with this sketch's wires pending."""
        wires = [wire for wire in self._wires if not wire.construction]
        return self._parent._spawn(_pending=self._parent._pending + wires)

    def fillet(self, radius):
        """Round selected polygon vertices with tangent circular arcs."""
        radius = float(radius)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("sketch fillet radius must be positive and finite")
        selected = self._active_wires()
        replacements = []
        for wire in selected:
            if len(wire.segments) < 3 or any(seg[0] != "line" for seg in wire.segments):
                raise NotImplementedError("2D fillet currently supports closed polygon wires")
            points = [wire.segments[0][1]] + [seg[2] for seg in wire.segments]
            if _dist2(points[0], points[-1]) > 1e-16:
                raise ValueError("2D fillet requires a closed contour")
            corners = []
            count = len(points) - 1
            for i in range(count):
                prev, vertex, nxt = points[(i - 1) % count], points[i], points[(i + 1) % count]
                tolerance = max(self._parent._tolerance, 1e-8)
                if (self._selected_vertex_points is not None and
                        not any(_dist2(vertex, point) <= tolerance * tolerance
                                for point in self._selected_vertex_points)):
                    corners.append(None)
                    continue
                def unit2(vector):
                    length = math.hypot(vector[0], vector[1])
                    if length <= 1e-15:
                        raise ValueError("cannot fillet a zero-length edge")
                    return (vector[0] / length, vector[1] / length)
                incoming = unit2((vertex[0] - prev[0], vertex[1] - prev[1]))
                outgoing = unit2((nxt[0] - vertex[0], nxt[1] - vertex[1]))
                cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
                dot = max(-1.0, min(1.0, incoming[0] * outgoing[0] + incoming[1] * outgoing[1]))
                angle = math.acos(dot)
                if abs(cross) < 1e-10 or angle < 1e-8:
                    raise ValueError("cannot fillet a straight or reversing vertex")
                distance = radius / math.tan(angle * .5)
                if distance >= min(_dist2(prev, vertex), _dist2(vertex, nxt)) * .5:
                    raise ValueError("fillet radius does not fit the adjacent edges")
                start = (vertex[0] - incoming[0] * distance,
                         vertex[1] - incoming[1] * distance)
                end = (vertex[0] + outgoing[0] * distance,
                       vertex[1] + outgoing[1] * distance)
                sign = 1.0 if cross > 0 else -1.0
                center = (start[0] - sign * incoming[1] * radius,
                          start[1] + sign * incoming[0] * radius)
                r0 = (start[0] - center[0], start[1] - center[1])
                r1 = (end[0] - center[0], end[1] - center[1])
                middle_dir = unit2((r0[0] / radius + r1[0] / radius,
                                    r0[1] / radius + r1[1] / radius))
                middle = (center[0] + radius * middle_dir[0],
                          center[1] + radius * middle_dir[1])
                corners.append((start, middle, end))
            segments = []
            for i, corner in enumerate(corners):
                start = corner[0] if corner else points[i]
                prior = corners[i - 1]
                prior_end = prior[2] if prior else points[i - 1]
                if _dist2(prior_end, start) > 1e-18:
                    segments.append(("line", prior_end, start))
                if corner:
                    segments.append(("arc", corner[0], corner[1], corner[2]))
            replacements.append(_Wire(segments, kind="path", name=wire.name,
                                      mode=wire.mode, tag=wire.tag))
        replacements_by_source = {id(source): replacement
                                  for source, replacement in zip(selected, replacements)}
        self._wires = [replacements_by_source.get(id(wire), wire) for wire in self._wires]
        for tag, tagged in self._tags.items():
            self._tags[tag] = [next((new for old, new in zip(selected, replacements) if old is item), item)
                               for item in tagged]
        self._selected = replacements
        return self

    def chamfer(self, length):
        """Cut equal-length corners from selected closed polygon wires."""
        length = float(length)
        if not math.isfinite(length) or length <= 0:
            raise ValueError("sketch chamfer length must be positive and finite")
        selected = self._active_wires()
        replacements = []
        for wire in selected:
            if len(wire.segments) < 3 or any(seg[0] != "line" for seg in wire.segments):
                raise NotImplementedError("2D chamfer currently supports closed polygon wires")
            points = [wire.segments[0][1]] + [seg[2] for seg in wire.segments]
            if _dist2(points[0], points[-1]) > 1e-16:
                raise ValueError("2D chamfer requires a closed contour")
            count = len(points) - 1
            corners = []
            for i in range(count):
                prev, vertex, nxt = points[(i - 1) % count], points[i], points[(i + 1) % count]
                tolerance = max(self._parent._tolerance, 1e-8)
                if (self._selected_vertex_points is not None and
                        not any(_dist2(vertex, point) <= tolerance * tolerance
                                for point in self._selected_vertex_points)):
                    corners.append(None)
                    continue
                def toward(other):
                    dx, dy = other[0] - vertex[0], other[1] - vertex[1]
                    edge_len = math.hypot(dx, dy)
                    if edge_len <= 1e-15:
                        raise ValueError("cannot chamfer a zero-length edge")
                    if length >= edge_len:
                        raise ValueError("chamfer length does not fit the adjacent edge")
                    return (vertex[0] + dx * length / edge_len,
                            vertex[1] + dy * length / edge_len)
                corners.append((toward(prev), toward(nxt)))
            segments = []
            for i, corner in enumerate(corners):
                before = corner[0] if corner else points[i]
                after = corner[1] if corner else points[i]
                previous = corners[i - 1]
                previous_after = previous[1] if previous else points[i - 1]
                if _dist2(previous_after, before) > 1e-18:
                    segments.append(("line", previous_after, before))
                if corner:
                    segments.append(("line", before, after))
            replacements.append(_Wire(segments, kind="path", name=wire.name,
                                      mode=wire.mode, tag=wire.tag))
        replacements_by_source = {id(source): replacement
                                  for source, replacement in zip(selected, replacements)}
        self._wires = [replacements_by_source.get(id(wire), wire) for wire in self._wires]
        self._selected = replacements
        return self


def _rect_wires(x_len, y_len, centered, locations, names=None):
    sx, sy = _center_shifts((x_len, y_len), centered)
    xmin, xmax = sx, sx + float(x_len)
    ymin, ymax = sy, sy + float(y_len)
    wires = []
    for index, (ox, oy) in enumerate(locations):
        if names:
            side_names = names
        elif len(locations) == 1:
            side_names = ("south", "east", "north", "west")
        else:
            side_names = tuple("rect{0}-{1}".format(index + 1, side)
                               for side in ("south", "east", "north", "west"))
        wires.append(_Wire.rect(xmin + ox, xmax + ox, ymin + oy, ymax + oy,
                                names=side_names))
    return wires


def _circle_wires(radius, locations, name=None):
    return [_Wire.circle(loc, radius, name=name) for loc in locations]


def _ellipse_wires(x_radius, y_radius, rotation_angle, locations):
    rx, ry, angle = float(x_radius), float(y_radius), float(rotation_angle)
    if not all(math.isfinite(value) for value in (rx, ry, angle)) or rx <= 0 or ry <= 0:
        raise ValueError("ellipse radii must be positive and all parameters finite")
    angle = math.radians(angle)
    return [_Wire.ellipse(location, rx, ry, angle) for location in locations]


def _named_rect_sides(wires):
    if len(wires) != 1:
        return False
    for wire in wires:
        names = [seg[3] for seg in wire.segments if seg[0] == "line" and len(seg) > 3]
        if len(names) == 4:
            return True
    return False


def _wire_bounds(wires):
    xs = []
    ys = []
    for wire in wires:
        for seg in wire.segments:
            if seg[0] == "circle":
                cx, cy = seg[1]
                r = float(seg[2])
                xs.extend((cx - r, cx + r))
                ys.extend((cy - r, cy + r))
            elif seg[0] == "ellipse":
                cx, cy = seg[1]
                rx, ry, angle = map(float, seg[2:])
                ex = math.hypot(rx * math.cos(angle), ry * math.sin(angle))
                ey = math.hypot(rx * math.sin(angle), ry * math.cos(angle))
                xs.extend((cx - ex, cx + ex))
                ys.extend((cy - ey, cy + ey))
            elif seg[0] in ("line", "arc"):
                for pt in seg[1:4] if seg[0] == "arc" else seg[1:3]:
                    xs.append(float(pt[0]))
                    ys.append(float(pt[1]))
            elif seg[0] == "spline":
                for pt in seg[1]:
                    xs.append(float(pt[0]))
                    ys.append(float(pt[1]))
    if not xs:
        return None
    return (min(xs), max(xs), min(ys), max(ys))


def _wire_signed_area(wire):
    points = []
    for segment in wire.segments:
        if segment[0] != "line":
            return 1.0
        if not points:
            points.append(segment[1])
        points.append(segment[2])
    return 0.5 * sum(points[i][0] * points[i + 1][1] -
                     points[i + 1][0] * points[i][1]
                     for i in range(len(points) - 1))


class Workplane(object):
    """CadQuery-shaped modelling context backed by a camber ``Part``."""

    def __init__(
            self, inPlane="XY", origin=None, obj=None, part=None,
            size=1000.0, tolerance=0.01):
        """Start on a named plane (``XY``, ``front``, …) or a Frame.

        ``size`` is the Part AABB half-extent when this Workplane creates a Part.
        ``obj`` may be a camber Solid to continue from. ``part`` reuses an existing Part.
        """
        self._size = float(size)
        self._tolerance = float(tolerance)
        if obj is not None and isinstance(obj, (Solid, Surface)):
            part = obj._part
        if isinstance(inPlane, Workplane):
            self._copy_from(inPlane)
            if origin is not None:
                self._frame = Frame(vec3(origin), self._frame.x, self._frame.y, self._frame.z)
            if obj is not None:
                self._solid = self._as_solid(obj)
                self._stack_solids = []
            return
        if isinstance(inPlane, Frame):
            self._frame = inPlane
            if origin is not None:
                self._frame = Frame(vec3(origin), self._frame.x, self._frame.y, self._frame.z)
        else:
            self._frame = named_plane(inPlane, origin)
        self.part = part
        self._solid = self._as_solid(obj)
        self._stack_solids = []
        self._pending = []
        self._cursor = (0.0, 0.0)
        self._open = None
        self._open_segments = []
        self._locations = [(0.0, 0.0)]
        self._center2d = (0.0, 0.0)
        self._sections = []
        self._faces = []
        self._edges = []
        self._selected_faces = []
        self._selected_edges = []
        self._selected_vertices = []
        self._construction_vertices = []
        self._tags = {}

    def _copy_from(self, other):
        self.part = other.part
        self._size = other._size
        self._tolerance = other._tolerance
        self._frame = other._frame
        self._solid = other._solid
        self._stack_solids = list(other._stack_solids)
        self._pending = list(other._pending)
        self._cursor = other._cursor
        self._open = None if other._open is None else list(other._open)
        self._open_segments = list(other._open_segments)
        self._locations = list(other._locations)
        self._center2d = other._center2d
        self._sections = list(other._sections)
        self._faces = list(other._faces)
        self._edges = list(other._edges)
        self._selected_faces = list(other._selected_faces)
        self._selected_edges = list(other._selected_edges)
        self._selected_vertices = list(other._selected_vertices)
        self._construction_vertices = list(other._construction_vertices)
        self._tags = dict(other._tags)

    def _spawn(self, **updates):
        wp = Workplane.__new__(Workplane)
        wp._copy_from(self)
        for key, value in updates.items():
            setattr(wp, key, value)
        return wp

    def _ensure_part(self):
        if self.part is None:
            self.part = Part(vec3(-self._size), vec3(self._size), tolerance=self._tolerance)
        return self.part

    def _new_name(self, prefix):
        return self._ensure_part()._generate_name(prefix)

    def _as_solid(self, obj):
        if obj is None:
            return None
        if isinstance(obj, (Solid, Surface)):
            return obj
        if isinstance(obj, Workplane):
            return obj._solid
        raise TypeError("expected a camber Solid or Workplane")

    def _wires(self):
        wires = list(self._pending)
        if self._open_segments:
            segments = list(self._open_segments)
            if _dist2(self._cursor, self._open[0]) > 1e-18:
                segments.append(("line", self._cursor, self._open[0]))
            wires.append(_Wire(segments))
        return wires

    def _draw_wires(self, sketch, wires):
        for wire in wires:
            wire.draw(sketch)
        return sketch

    def _active_frame(self):
        """Current sketch plane: selected face if any, otherwise the workplane."""
        if not self._selected_faces:
            return self._frame
        face = self._selected_faces[0]
        origin = vec3(face["center"])
        normal = vec3(face["normal"])
        kernel = None
        if self.part is not None:
            try:
                kernel = self.part.plane_frame(face["name"])
            except Exception:
                kernel = None
        if kernel is not None:
            return self._aligned_face_frame(kernel, origin, normal)
        return _plane_from_normal(origin, normal, self._frame.x)

    @staticmethod
    def _aligned_face_frame(frame, origin, normal):
        if _dot(vec3(frame.z), vec3(normal)) < 0:
            return Frame(origin, frame.x, -vec3(frame.y), -vec3(frame.z))
        return Frame(origin, frame.x, frame.y, frame.z)

    def _profile_sketch(self, wires, name, frame=None):
        if not wires:
            raise ValueError("no pending 2D geometry")
        sk = self._ensure_part().sketch(frame=frame or self._active_frame(), name=name)
        self._draw_wires(sk, wires)
        return sk

    def _apply_combine(self, solid, faces, edges, combine):
        if solid is None:
            return self._cleared()
        if self._solid is None or combine is False:
            return self._cleared(_solid=solid, _faces=list(faces), _edges=list(edges))
        name = self._solid.name
        if combine is True or combine in ("a", "union"):
            merged = self.part.union(self._solid, solid, name=name)
            return self._cleared(
                _solid=merged,
                _faces=self._faces + list(faces),
                _edges=self._edges + list(edges),
            )
        if combine in ("cut", "s", "difference"):
            merged = self.part.subtract(self._solid, solid, name=name)
            return self._cleared(_solid=merged, _faces=list(self._faces), _edges=list(self._edges))
        if combine in ("intersect", "i"):
            merged = self.part.intersect(self._solid, solid, name=name)
            return self._cleared(_solid=merged, _faces=list(self._faces), _edges=list(self._edges))
        raise ValueError("unknown combine mode {0!r}".format(combine))

    def _cleared(self, **updates):
        defaults = dict(
            _stack_solids=[],
            _pending=[],
            _open=None,
            _open_segments=[],
            _cursor=(0.0, 0.0),
            _sections=[],
            _selected_faces=[],
            _selected_edges=[],
            _selected_vertices=[],
            _construction_vertices=[],
        )
        defaults.update(updates)
        return self._spawn(**defaults)

    def _union_many(self, solids, faces, edges, combine, name_prefix):
        if not solids:
            raise ValueError(name_prefix + " produced no solids")
        if len(solids) == 1:
            solid = solids[0]
        else:
            solid = self.part.batch_union(solids)
        return self._apply_combine(solid, faces, edges, combine)

    def workplane(self, offset=0, invert=False, centerOption="CenterOfMass"):
        """New plane offset along +Z of the current plane or selected face."""
        offset = float(offset)
        if self._selected_faces:
            sections = list(self._sections)
            if self._pending:
                sections.append((self._active_frame(), list(self._pending)))
            face = self._selected_faces[0]
            origin = vec3(face["center"])
            normal = vec3(face["normal"])
            if str(centerOption) == "ProjectedOrigin":
                origin = _project_to_plane(self._frame.origin, origin, normal)
            elif self._selected_vertices:
                origin = self._selected_vertices[0]
            try:
                kernel = self._ensure_part().plane_frame(face["name"]) if self.part is not None else None
            except Exception:
                kernel = None
            if kernel is not None:
                frame = self._aligned_face_frame(kernel, origin, normal)
            else:
                frame = _plane_from_normal(origin, normal, self._frame.x)
        else:
            frame = self._frame
            sections = list(self._sections)
            pending = list(self._pending)
            if pending:
                sections.append((self._frame, pending))
            frame = _frame_plus_local(frame, (self._center2d[0], self._center2d[1], offset))
            if invert:
                frame = Frame(frame.origin, -vec3(frame.x), vec3(frame.y), -vec3(frame.z))
            return self._spawn(
                _frame=frame,
                _pending=[],
                _open=None,
                _open_segments=[],
                _cursor=(0.0, 0.0),
                _sections=sections,
                _locations=[(0.0, 0.0)],
                _center2d=(0.0, 0.0),
                _selected_faces=[],
                _selected_edges=[],
                _selected_vertices=[],
                _construction_vertices=[],
            )
        if invert:
            frame = Frame(frame.origin, -vec3(frame.x), vec3(frame.y), -vec3(frame.z))
        frame = _frame_plus_local(frame, (0.0, 0.0, offset))
        return self._spawn(
            _frame=frame,
            _pending=[],
            _open=None,
            _open_segments=[],
            _cursor=(0.0, 0.0),
            _sections=sections,
            _locations=[(0.0, 0.0)],
            _center2d=(0.0, 0.0),
            _selected_faces=[],
            _selected_edges=[],
            _selected_vertices=[],
            _construction_vertices=[],
        )

    def transformed(self, rotate=(0, 0, 0), offset=(0, 0, 0)):
        """Rotate then translate the workplane (degrees, local XYZ)."""
        frame = _rotate_frame(self._frame, rotate)
        frame = _frame_plus_local(frame, offset)
        return self._spawn(_frame=frame)

    def copyWorkplane(self, other):
        """Continue the current solid on another Workplane's sketch frame."""
        if not isinstance(other, Workplane):
            raise TypeError("copyWorkplane expects a Workplane")
        return self._spawn(_frame=other._active_frame(), _pending=[], _open=None,
                           _open_segments=[], _cursor=(0.0, 0.0),
                           _locations=[(0.0, 0.0)], _center2d=(0.0, 0.0),
                           _selected_faces=[], _selected_edges=[], _selected_vertices=[],
                           _construction_vertices=[])

    def tag(self, name):
        """Save the current workplane and face references under a reusable tag."""
        if not name:
            raise ValueError("tag name must be nonempty")
        tags = dict(self._tags)
        tags[str(name)] = (self._active_frame(), list(self.faces()._selected_faces))
        return self._spawn(_tags=tags)

    def workplaneFromTagged(self, name):
        """Restore a workplane frame saved by ``tag`` while keeping this solid."""
        try:
            saved = self._tags[str(name)]
        except KeyError:
            raise ValueError("unknown workplane tag {0!r}".format(name))
        frame = saved[0] if isinstance(saved, tuple) else saved
        return self._spawn(_frame=frame, _pending=[], _open=None,
                           _open_segments=[], _cursor=(0.0, 0.0),
                           _locations=[(0.0, 0.0)], _center2d=(0.0, 0.0),
                           _selected_faces=[], _selected_edges=[], _selected_vertices=[],
                           _construction_vertices=[])

    def center(self, x, y):
        """Shift the 2D origin and set the current point to the new center."""
        dx, dy = float(x), float(y)
        center = (self._center2d[0] + dx, self._center2d[1] + dy)
        pending = list(self._pending)
        if self._open_segments:
            segments = list(self._open_segments)
            if _dist2(self._cursor, self._open[0]) > 1e-18:
                segments.append(("line", self._cursor, self._open[0]))
            pending.append(_Wire(segments))
        return self._spawn(
            _center2d=center,
            _locations=[(px + dx, py + dy) for px, py in self._locations],
            _pending=pending,
            _cursor=center,
            _open=None,
            _open_segments=[],
        )

    def _path_point(self, point):
        x, y = _xy2(point)
        return (x + self._center2d[0], y + self._center2d[1])

    def _line_to_point(self, end):
        pts = list(self._open or [self._cursor])
        pts.append(end)
        return self._spawn(_cursor=end, _open=pts,
                           _open_segments=self._open_segments + [("line", self._cursor, end)])

    def moveTo(self, x, y):
        """Set the 2D cursor and start a new open path."""
        pt = self._path_point((x, y))
        return self._spawn(_cursor=pt, _open=[pt], _open_segments=[], _locations=[pt])

    def move(self, xDist, yDist):
        """Relative move of the 2D cursor."""
        point = (self._cursor[0] + float(xDist), self._cursor[1] + float(yDist))
        return self._spawn(_cursor=point, _open=[point], _open_segments=[], _locations=[point])

    def lineTo(self, x, y):
        """Line from the cursor to an absolute 2D point."""
        return self._line_to_point(self._path_point((x, y)))

    def line(self, xDist, yDist):
        """Relative line from the cursor."""
        return self._line_to_point((self._cursor[0] + float(xDist),
                                    self._cursor[1] + float(yDist)))

    def hLine(self, distance):
        """Horizontal relative line."""
        return self.line(distance, 0.0)

    def vLine(self, distance):
        """Vertical relative line."""
        return self.line(0.0, distance)

    def hLineTo(self, x):
        """Horizontal line to absolute X."""
        return self._line_to_point((float(x) + self._center2d[0], self._cursor[1]))

    def vLineTo(self, y):
        """Vertical line to absolute Y."""
        return self._line_to_point((self._cursor[0], float(y) + self._center2d[1]))

    def close(self):
        """Close the open path into a pending wire."""
        if not self._open or len(self._open) < 2:
            raise ValueError("close() needs a path with at least one segment")
        segments = list(self._open_segments)
        if _dist2(self._cursor, self._open[0]) > 1e-18:
            segments.append(("line", self._cursor, self._open[0]))
        wire = _Wire(segments)
        return self._spawn(_pending=self._pending + [wire], _open=None,
                           _open_segments=[], _cursor=self._open[0])

    def rect(self, xLen, yLen, centered=True, forConstruction=False):
        """Pending rectangle or construction rectangle for vertex locations."""
        if forConstruction:
            dx, dy = _center_shifts((xLen, yLen), centered)
            vertices = []
            for ox, oy in self._locations:
                vertices.extend([(ox + dx, oy + dy), (ox + dx + xLen, oy + dy),
                                 (ox + dx + xLen, oy + dy + yLen), (ox + dx, oy + dy + yLen)])
            return self._spawn(_construction_vertices=vertices)
        return self._spawn(_pending=self._pending + _rect_wires(xLen, yLen, centered, self._locations))

    def circle(self, radius, forConstruction=False):
        """Pending circle. Construction flag is ignored (not drawn)."""
        if forConstruction:
            return self
        return self._spawn(_pending=self._pending + _circle_wires(radius, self._locations))

    def ellipse(self, x_radius, y_radius, rotation_angle=0.0, forConstruction=False):
        """Add a closed ellipse for each workplane location.

        Radii are semiaxis lengths. ``rotation_angle`` is in degrees, matching
        CadQuery; the native sketch stores the equivalent angle in radians.
        Construction ellipses are ignored, like construction circles here.
        """
        if forConstruction:
            return self
        return self._spawn(_pending=self._pending + _ellipse_wires(
            x_radius, y_radius, rotation_angle, self._locations))

    def polygon(self, nSides, diameter, forConstruction=False):
        """Regular polygon inscribed in a circle of ``diameter``."""
        if int(nSides) < 3:
            raise ValueError("polygon needs at least 3 sides")
        if forConstruction:
            return self
        radius = 0.5 * float(diameter)
        wires = []
        for ox, oy in self._locations:
            pts = []
            for i in range(int(nSides)):
                ang = 2.0 * math.pi * i / float(nSides)
                pts.append((ox + radius * math.cos(ang), oy + radius * math.sin(ang)))
            wires.append(_Wire.from_points(pts, closed=True))
        return self._spawn(_pending=self._pending + wires)

    def polyline(self, listOfXY, includeCurrent=False):
        """Open path through 2D points (does not close)."""
        pts = [self._path_point(p) for p in listOfXY]
        if includeCurrent:
            pts = [self._cursor] + pts
        if len(pts) < 2:
            raise ValueError("polyline needs at least two points")
        segments = [("line", pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        return self._spawn(_cursor=pts[-1], _open=pts, _open_segments=segments)

    def threePointArc(self, point1, point2):
        """Arc from the cursor through ``point1`` to ``point2``."""
        start = self._cursor
        mid = self._path_point(point1)
        end = self._path_point(point2)
        pts = list(self._open or [start])
        pts.append(end)
        return self._spawn(_cursor=end, _open=pts,
                           _open_segments=self._open_segments + [("arc", start, mid, end)])

    def radiusArc(self, endPoint, radius):
        """Arc from the cursor to ``endPoint`` with signed radius (CCW if positive)."""
        end = self._path_point(endPoint)
        mid = _radius_arc_mid(self._cursor, end, radius)
        start = self._cursor
        pts = list(self._open or [start])
        pts.append(end)
        return self._spawn(_cursor=end, _open=pts,
                           _open_segments=self._open_segments + [("arc", start, mid, end)])

    def _mirror_open(self, axis):
        if not self._open_segments:
            raise ValueError("mirror requires an open 2D path")
        cx, cy = self._center2d
        flip = (lambda p: (2 * cx - p[0], p[1])) if axis == "Y" else (
            lambda p: (p[0], 2 * cy - p[1]))
        start = self._open[0]
        end = self._cursor
        segments = list(self._open_segments)
        reflected_end = flip(end)
        if _dist2(end, reflected_end) > 1e-18:
            segments.append(("line", end, reflected_end))
        for seg in reversed(self._open_segments):
            if seg[0] == "line":
                segments.append(("line", flip(seg[2]), flip(seg[1])))
            elif seg[0] == "arc":
                segments.append(("arc", flip(seg[3]), flip(seg[2]), flip(seg[1])))
            else:
                raise NotImplementedError("mirror only supports line and arc paths")
        reflected_start = flip(start)
        if _dist2(reflected_start, start) > 1e-18:
            segments.append(("line", reflected_start, start))
        return self._spawn(_pending=self._pending + [_Wire(segments)],
                           _open=None, _open_segments=[], _cursor=start)

    def mirrorX(self):
        """Complete an open 2D path by reflecting it across local X."""
        return self._mirror_open("X")

    def mirrorY(self):
        """Complete an open 2D path by reflecting it across local Y."""
        return self._mirror_open("Y")

    def slot2D(self, length, diameter, angle=0):
        """Stadium outline. ``angle`` is degrees in the workplane."""
        half = 0.5 * float(length)
        r = 0.5 * float(diameter)
        if half < r:
            raise ValueError("slot2D length must be >= diameter")
        straight = half - r
        ang = math.radians(float(angle))
        ca, sa = math.cos(ang), math.sin(ang)

        def rot(x, y):
            return (x * ca - y * sa, x * sa + y * ca)

        wires = []
        for ox, oy in self._locations:
            def pt(x, y):
                rx, ry = rot(x, y)
                return (ox + rx, oy + ry)

            segs = [
                ("line", pt(-straight, -r), pt(straight, -r)),
                ("arc", pt(straight, -r), pt(half, 0.0), pt(straight, r)),
                ("line", pt(straight, r), pt(-straight, r)),
                ("arc", pt(-straight, r), pt(-half, 0.0), pt(-straight, -r)),
            ]
            wires.append(_Wire(segs, kind="path"))
        return self._spawn(_pending=self._pending + wires)

    def pushPoints(self, pntList):
        """Set 2D locations for the next primitive (rect/circle/box)."""
        cx, cy = self._center2d
        return self._spawn(_locations=[(cx + x, cy + y) for x, y in (_xy2(p) for p in pntList)])

    def rarray(self, xSpacing, ySpacing, xCount, yCount, center=True):
        """Rectangular array of locations in the workplane."""
        if int(xCount) < 1 or int(yCount) < 1:
            raise ValueError("rarray counts must be >= 1")
        xs = []
        ys = []
        for i in range(int(xCount)):
            xs.append(float(xSpacing) * (i - 0.5 * (int(xCount) - 1) if center else i))
        for j in range(int(yCount)):
            ys.append(float(ySpacing) * (j - 0.5 * (int(yCount) - 1) if center else j))
        locs = [(x, y) for y in ys for x in xs]
        return self._spawn(_locations=locs)

    def polarArray(self, radius, startAngle, angle, count, fill=True, rotate=True):
        """Polar array of locations. Angles in degrees. ``rotate`` is unused."""
        if int(count) < 1:
            raise ValueError("polarArray count must be >= 1")
        step = float(angle) / float(count) if fill else float(angle)
        locs = []
        for i in range(int(count)):
            a = math.radians(float(startAngle) + step * i)
            locs.append((float(radius) * math.cos(a), float(radius) * math.sin(a)))
        return self._spawn(_locations=locs)

    def sketch(self):
        """Enter a CadQuery-shaped 2D Sketch; call ``finalize()`` to resume."""
        return Sketch(self)

    def placeSketch(self, *sketches):
        """Append wires from ``cqcompat.Sketch`` instances as pending geometry."""
        wires = []
        for item in sketches:
            if isinstance(item, Sketch):
                wires.extend(wire for wire in item._wires if not wire.construction)
            else:
                raise TypeError("placeSketch expects camber.cqcompat.Sketch instances")
        if len(sketches) > 1:
            sections = [(_frame_plus_local(self._active_frame(), item._offset),
                         [wire for wire in item._wires if not wire.construction])
                        for item in sketches]
            return self._spawn(_sections=self._sections + sections, _pending=[])
        if len(sketches) == 1:
            frame = _frame_plus_local(self._active_frame(), sketches[0]._offset)
            return self._spawn(_frame=frame,
                               _pending=self._pending + wires)
        return self._spawn(_pending=self._pending + wires)

    def box(self, length, width, height, centered=True, combine=True, clean=True):
        """Cuboid along the current workplane. ``clean`` is unused."""
        solids = []
        faces = []
        edges = []
        base = self._active_frame()
        for loc in self._locations:
            frame = _frame_plus_local(base, (loc[0], loc[1], 0.0))
            solid, fcs, edgs = self._make_box(frame, length, width, height, centered)
            solids.append(solid)
            faces.extend(fcs)
            edges.extend(edgs)
        return self._union_many(solids, faces, edges, combine, "box")

    def _make_box(self, frame, length, width, height, centered):
        dx, dy, dz = _center_shifts((length, width, height), centered)
        xmin, xmax = dx, dx + float(length)
        ymin, ymax = dy, dy + float(width)
        zmin, zmax = dz, dz + float(height)
        name = self._new_name("box")
        sketch_frame = _frame_plus_local(frame, (0.0, 0.0, zmin))
        sk = self.part.sketch(frame=sketch_frame, name=name + "_sk")
        _Wire.rect(xmin, xmax, ymin, ymax, names=("south", "east", "north", "west")).draw(sk)
        solid = sk.extrude(zmax - zmin, name=name)
        faces, edges = _prism_faces(name, frame, xmin, xmax, ymin, ymax, zmin, zmax)
        return solid, faces, edges

    def cylinder(self, height, radius, direct=None, angle=360, centered=True, combine=True, clean=True):
        """Cylinder. ``direct`` is the axis (default workplane Z). Full 360° only."""
        if direct is not None:
            raise NotImplementedError("cylinder(direct=...) needs a kernel change")
        if abs(float(angle) - 360.0) > 1e-9:
            raise NotImplementedError("partial cylinder angle needs a kernel change")
        solids = []
        faces = []
        edges = []
        if centered is True:
            z_centered = True
        elif centered is False:
            z_centered = False
        else:
            z_centered = bool(centered[2] if len(centered) > 2 else centered[-1])
        z0 = -0.5 * float(height) if z_centered else 0.0
        z1 = z0 + float(height)
        base = self._active_frame()
        for loc in self._locations:
            frame = _frame_plus_local(base, (loc[0], loc[1], 0.0))
            name = self._new_name("cyl")
            sketch_frame = _frame_plus_local(frame, (0.0, 0.0, z0))
            sk = self.part.sketch(frame=sketch_frame, name=name + "_sk")
            sk.add_circle((0.0, 0.0), float(radius), name="side")
            solid = sk.extrude(z1 - z0, name=name)
            fcs, edgs = _revolve_caps(name, frame, z0, z1, float(radius))
            solids.append(solid)
            faces.extend(fcs)
            edges.extend(edgs)
        return self._union_many(solids, faces, edges, combine, "cylinder")

    def sphere(self, radius, direct=None, angle1=-90, angle2=90, angle3=360, centered=True, combine=True, clean=True):
        """Full sphere at the workplane origin. Partial spheres are not implemented."""
        if direct is not None or abs(float(angle3) - 360.0) > 1e-9:
            raise NotImplementedError("partial sphere needs a kernel change")
        solids = []
        base = self._active_frame()
        for loc in self._locations:
            origin = _world_from_local(base, (loc[0], loc[1], 0.0))
            solids.append(self._ensure_part().sphere(origin, float(radius), name=self._new_name("sph")))
        return self._union_many(solids, [], [], combine, "sphere")

    def parametricSurface(self, func, N=40, start=0, stop=1):
        """Sample one open parametric surface for display and assembly mates.

        ``N`` is the number of segments in each parameter direction. This is
        an explicitly sampled surface, not an analytic/NURBS face.
        """
        if not callable(func) or isinstance(N, bool) or int(N) != N or not 2 <= N <= 1000:
            raise ValueError("parametricSurface needs a callable and 2 <= N <= 1000")
        start, stop, N = float(start), float(stop), int(N)
        if not math.isfinite(start) or not math.isfinite(stop) or start == stop:
            raise ValueError("parametricSurface parameter range must be finite and nonzero")
        frame = self._active_frame()

        def evaluate(u, v):
            point = vec3(func(u, v))
            if any(not math.isfinite(c) for c in point):
                raise ValueError("parametricSurface callback returned a non-finite point")
            return _world_from_local(frame, point)

        step = (stop - start) / N
        points = [evaluate(start + i * step, start + j * step)
                  for i in range(N + 1) for j in range(N + 1)]
        triangles = []
        for i in range(N):
            for j in range(N):
                a = i * (N + 1) + j
                b = (i + 1) * (N + 1) + j
                triangles.extend(((a, b, b + 1), (a, b + 1, a + 1)))
        midpoint = (start + stop) / 2
        du = evaluate(midpoint + step / 2, midpoint) - evaluate(midpoint - step / 2, midpoint)
        dv = evaluate(midpoint, midpoint + step / 2) - evaluate(midpoint, midpoint - step / 2)
        normal = _unit(_cross(du, dv))
        if _norm(normal) < 1e-12:
            raise ValueError("parametricSurface is singular at its center")
        surface = self._ensure_part().solid_from_mesh(points, triangles,
                                                       name=self._new_name("surface"))
        if not isinstance(surface, Surface):
            raise ValueError("parametricSurface must remain open")
        faces = [{"name": surface.patch_names[0].split(":")[-1],
                  "center": evaluate(midpoint, midpoint),
                  "normal": normal, "kind": "curved"}]
        return self._cleared(_solid=surface, _faces=faces, _edges=[])

    def extrude(self, until, combine=True, clean=True, both=False, taper=0):
        """Extrude pending 2D wires; ``taper`` is a draft angle in degrees."""
        taper_degrees = float(taper)
        if not math.isfinite(taper_degrees):
            raise ValueError("extrude taper must be a finite angle in degrees")
        taper_angle = math.radians(taper_degrees)
        if isinstance(until, str):
            if until.lower() != "next":
                raise NotImplementedError("only extrude('next') is supported for face-terminated extrusions")
            if taper_angle:
                raise NotImplementedError("tapered extrude('next') is not supported")
            if both:
                raise ValueError("extrude('next') cannot be combined with both=True")
            if self._solid is None:
                raise ValueError("extrude('next') needs an existing target solid")
            wires = self._wires()
            name = self._new_name("ext")
            sk = self._profile_sketch(wires, name + "_sk")
            solid = sk.extrude_until_next(self._solid, name=name)
            return self._apply_combine(solid, [], [], combine)
        if both and taper_angle:
            raise ValueError("tapered extrude cannot be combined with both=True")
        return self._extrude_pending(float(until), combine=combine, both=both, twist=0.0,
                                     taper_angle=taper_angle)

    def twistExtrude(self, distance, angleDegrees, combine=True, clean=True):
        """Extrude with twist (degrees along the height)."""
        distance = float(distance)
        if abs(distance) < 1e-15:
            raise ValueError("twistExtrude distance must be non-zero")
        # camber twist is radians per unit length; CadQuery uses total degrees.
        twist = math.radians(float(angleDegrees)) / distance
        return self._extrude_pending(distance, combine=combine, both=False, twist=twist)

    def _extrude_pending(self, height, combine, both, twist, taper_angle=0.0):
        wires = self._wires()
        name = self._new_name("ext")
        if any(wire.mode is not None for wire in wires):
            solid = None
            for index, wire in enumerate(wires):
                if wire.construction:
                    continue
                mode = wire.mode or "a"
                wire_name = name if index == 0 else self._new_name("ext_profile")
                sk = self._profile_sketch([wire], wire_name + "_sk")
                if both:
                    operand = sk.extrude(abs(height), name=wire_name, both_sides=True,
                                                twist=twist)
                else:
                    operand = sk.extrude(height, name=wire_name, twist=twist,
                                                taper_angle=taper_angle)
                if solid is None:
                    if mode in ("s", "i"):
                        raise ValueError("a subtract/intersect sketch mode needs an existing profile")
                    solid = operand
                elif mode == "r":
                    solid = operand
                elif mode == "s":
                    solid = self.part.subtract(solid, operand, name=name)
                elif mode == "i":
                    solid = self.part.intersect(solid, operand, name=name)
                else:
                    solid = self.part.union(solid, operand, name=name)
            if solid is None:
                raise ValueError("sketch contains no profile geometry to extrude")
            return self._apply_combine(solid, [], [], combine)
        sk = self._profile_sketch(wires, name + "_sk")
        if both:
            solid = sk.extrude(abs(height), name=name, both_sides=True, twist=twist)
            z0, z1 = -abs(height), abs(height)
        else:
            solid = sk.extrude(height, name=name, twist=twist,
                                      taper_angle=taper_angle)
            if height >= 0:
                z0, z1 = 0.0, height
            else:
                z0, z1 = height, 0.0
        faces, edges = self._pending_topology(name, z0, z1, wires)
        return self._apply_combine(solid, faces, edges, combine)

    def _pending_topology(self, name, z0, z1, wires):
        frame = self._active_frame()
        bounds = _wire_bounds(wires)
        if bounds is not None and _named_rect_sides(wires):
            xmin, xmax, ymin, ymax = bounds
            return _prism_faces(name, frame, xmin, xmax, ymin, ymax, z0, z1)
        faces = [
            {"name": name + "-ExtrudeTop", "center": _world_from_local(frame, (0.0, 0.0, z1)),
             "normal": vec3(frame.z), "kind": "plane"},
            {"name": name + "-ExtrudeBottom", "center": _world_from_local(frame, (0.0, 0.0, z0)),
             "normal": -vec3(frame.z), "kind": "plane"},
        ]
        edges = []
        circle_index = 0
        for wire in wires:
            if wire.kind == "circle":
                circle_index += 1
                radius = wire.segments[0][2]
                center = wire.segments[0][1]
                faces.append({
                    "name": name + "-side",
                    "center": _world_from_local(
                        frame, (center[0] + radius, center[1], 0.5 * (z0 + z1))),
                    "normal": vec3(frame.x),
                    "kind": "cylinder",
                })
                edges.extend(_revolve_caps(
                    name, frame, z0, z1, radius,
                    side_name="Circle{0}".format(circle_index),
                )[1])
        return faces, edges

    def revolve(self, angleDegrees=360, axisStart=None, axisEnd=None, combine=True, clean=True):
        """Revolve pending wires. Default axis is workplane Y through the origin."""
        source = self._active_frame()
        origin, x_axis, y_axis = self._revolve_axes(axisStart, axisEnd)
        z_axis = _unit(_cross(x_axis, y_axis))
        y_axis = _unit(_cross(z_axis, x_axis))
        frame = Frame(origin, x_axis, y_axis, z_axis)

        def to_revolve(point):
            delta = _world_from_local(source, (point[0], point[1], 0.0)) - origin
            if abs(_dot(delta, z_axis)) > 1e-8:
                raise ValueError("revolve profile and axis must share a plane")
            return (_dot(delta, x_axis), _dot(delta, y_axis))

        wires = [wire.mapped_xy(to_revolve) for wire in self._wires()]
        name = self._new_name("rev")
        sk = self._profile_sketch(wires, name + "_sk", frame=frame)
        solid = sk.revolve(math.radians(float(angleDegrees)), name=name)
        return self._apply_combine(solid, [], [], combine)

    def _revolve_axes(self, axis_start, axis_end):
        if axis_start is None and axis_end is None:
            # CadQuery default: workplane Y. Camber revolve axis is sketch X.
            frame = self._active_frame()
            return vec3(frame.origin), vec3(frame.y), vec3(frame.x)
        frame = self._active_frame()
        start = self._axis_point(axis_start)
        end = self._axis_point(axis_end)
        direction = end - start
        if _norm(direction) < 1e-12:
            raise ValueError("revolve axis is degenerate")
        x_axis = _unit(direction)
        if abs(_dot(x_axis, frame.z)) > 1e-9:
            raise ValueError("revolve axis must lie in the profile plane")
        hint = frame.x if abs(_dot(frame.x, x_axis)) <= 0.95 else frame.y
        y_axis = hint - x_axis * _dot(hint, x_axis)
        return start, x_axis, _unit(y_axis)

    def _axis_point(self, point):
        frame = self._active_frame()
        if point is None:
            return vec3(frame.origin)
        if len(point) == 2:
            point = (point[0], point[1], 0.0)
        return _world_from_local(frame, point)

    def sweep(self, path, combine=True, clean=True):
        """Sweep pending profile along ``path`` (a Workplane with an open 2D path)."""
        wires = self._wires()
        name = self._new_name("swp")
        profile = self._profile_sketch(wires, name + "_profile")
        if isinstance(path, Curve3D):
            solid = profile.extrude_along_curve(path, name=name)
        elif isinstance(path, Workplane):
            guide_wires = path._wires()
            if not guide_wires:
                raise ValueError("sweep path has no pending 2D geometry")
            guide = path._profile_sketch(guide_wires, name + "_guide")
            solid = profile.extrude_along_sketch(guide, name=name)
        else:
            raise TypeError("sweep path must be a Workplane or camber Curve3D")
        return self._apply_combine(solid, [], [], combine)

    def loft(self, ruled=False, combine=True, clean=True):
        """Loft stacked ``workplane()`` sections. ``ruled=True`` uses LOFT_STYLE_RULED."""
        sections = list(self._sections)
        wires = self._wires()
        if wires:
            sections.append((self._frame, wires))
        if len(sections) < 2:
            raise ValueError("loft needs at least two sections (use workplane(offset=...) between profiles)")
        sketches = []
        name = self._new_name("loft")
        for i, (frame, section_wires) in enumerate(sections):
            sketches.append(self._profile_sketch(section_wires, "{0}_s{1}".format(name, i), frame=frame))
        options = LoftOptions()
        # CadQuery aligns closed section seams to avoid arbitrary rotation
        # between unlike profiles (e.g. rectangle -> ellipse -> slot).
        options._n.alignment_mode = 1  # LoftAlignmentMode.MinimumTwist
        if ruled:
            native = options._n
            if hasattr(native, "style"):
                native.style = LOFT_STYLE_RULED
            elif hasattr(native, "Style"):
                native.Style = LOFT_STYLE_RULED
        solid = self.part.loft(sketches, options=options, name=name)
        return self._apply_combine(solid, [], [], combine)

    def cut(self, toCut, clean=True, tol=None):
        """Boolean cut: this solid minus ``toCut`` (Workplane or Solid)."""
        other = self._as_solid(toCut)
        if self._solid is None or other is None:
            raise ValueError("cut needs two solids")
        return self._cleared(
            _solid=self.part.subtract(self._solid, other, name=self._solid.name),
            _faces=list(self._faces),
            _edges=list(self._edges),
        )

    def union(self, toUnion=None, clean=True, glue=False, tol=None):
        """Boolean union with another Workplane or Solid."""
        other = self._as_solid(toUnion)
        if self._solid is None or other is None:
            raise ValueError("union needs two solids")
        other_faces = list(getattr(toUnion, "_faces", []) or [])
        other_edges = list(getattr(toUnion, "_edges", []) or [])
        return self._cleared(
            _solid=self.part.union(self._solid, other, name=self._solid.name),
            _faces=self._faces + other_faces,
            _edges=self._edges + other_edges,
        )

    def intersect(self, toIntersect, clean=True, tol=None):
        """Boolean intersection with another Workplane or Solid."""
        other = self._as_solid(toIntersect)
        if self._solid is None or other is None:
            raise ValueError("intersect needs two solids")
        return self._cleared(
            _solid=self.part.intersect(self._solid, other, name=self._solid.name),
            _faces=list(self._faces),
            _edges=list(self._edges),
        )

    def cutBlind(self, until, clean=True, both=False, taper=0):
        """Cut pending profiles to a depth; ``taper`` is in degrees."""
        taper_degrees = float(taper)
        if not math.isfinite(taper_degrees):
            raise ValueError("cutBlind taper must be a finite angle in degrees")
        if isinstance(until, Surface):
            if taper_degrees:
                raise NotImplementedError("tapered cutBlind to a surface is not supported")
            if both or self._solid is None or self._stack_solids:
                raise ValueError("cutBlind(surface) needs one target solid and cannot use both=True")
            frame = self._active_frame()
            inward = Frame(frame.origin, frame.x, -vec3(frame.y), -vec3(frame.z))
            name = self._new_name("cut")
            sketch = self._profile_sketch(self._wires(), name + "_sk", frame=inward)
            cutter = sketch.extrude_until_surface(until, name=name + "_tool")
            solid = self.part.subtract(self._solid, cutter, name=self._solid.name)
            return self._cleared(_solid=solid, _faces=list(self._faces), _edges=list(self._edges))
        if isinstance(until, str):
            if until.lower() != "last":
                raise ValueError("cutBlind string extent must be 'last'")
            solids = self.vals()
            if not solids:
                raise ValueError("cut needs an existing solid")
            frame = self._active_frame()
            origin, normal = vec3(frame.origin), vec3(frame.z)
            depth = max(_dot(point - origin, normal)
                        for solid in solids for point in solid.mesh()[0])
            if depth <= 0:
                raise ValueError("cutBlind('last') has no solid in the workplane normal direction")
            return self._cut_pending(depth, both=False, taper_degrees=taper_degrees)
        return self._cut_pending(float(until), both=both, taper_degrees=taper_degrees)

    def cutThruAll(self, clean=True, taper=0):
        """Cut pending wires through the part AABB, optionally with draft in degrees."""
        return self._cut_pending(2.0 * self._size, both=True, taper_degrees=float(taper))

    def _cut_pending(self, height, both, taper_degrees=0.0):
        solids = self.vals()
        if not solids:
            raise ValueError("cut needs an existing solid")
        wires = self._wires()
        frame = self._active_frame()
        if taper_degrees:
            return self._cut_pending_tapered(solids, wires, frame, height, both, taper_degrees)
        results = []
        for solid in solids:
            part = solid._part
            name = part._generate_name("cut")
            if any(wire.mode is not None for wire in wires):
                cutter = None
                for index, wire in enumerate(wires):
                    if wire.construction:
                        continue
                    operation = wire.mode or "a"
                    tool_name = name if index == 0 else part._generate_name("cut_profile")
                    sk = part.sketch(frame=frame, name=tool_name + "_sk")
                    wire.draw(sk)
                    if both:
                        operand = sk.extrude_two_sides(abs(height), abs(height), name=tool_name)
                    elif height >= 0:
                        operand = sk.extrude(height, name=tool_name)
                    else:
                        operand = sk.extrude_two_sides(0.0, abs(height), name=tool_name)
                    if cutter is None:
                        if operation in ("s", "i"):
                            raise ValueError("a subtract/intersect sketch mode needs an existing profile")
                        cutter = operand
                    elif operation == "r":
                        cutter = operand
                    elif operation == "s":
                        cutter = part.subtract(cutter, operand, name=name + "_profile")
                    elif operation == "i":
                        cutter = part.intersect(cutter, operand, name=name + "_profile")
                    else:
                        cutter = part.union(cutter, operand, name=name + "_profile")
                if cutter is None:
                    raise ValueError("sketch contains no cutting profiles")
            else:
                sk = part.sketch(frame=frame, name=name + "_sk")
                self._draw_wires(sk, wires)
                if both:
                    cutter = sk.extrude_two_sides(abs(height), abs(height), name=name)
                elif height >= 0:
                    cutter = sk.extrude(height, name=name)
                else:
                    cutter = sk.extrude_two_sides(0.0, abs(height), name=name)
            results.append(part.subtract(solid, cutter, name=solid.name))
        return self._cleared(
            _solid=results[0] if len(results) == 1 else None,
            _stack_solids=[] if len(results) == 1 else results,
            _faces=list(self._faces), _edges=list(self._edges))

    def _cut_pending_tapered(self, solids, wires, frame, height, both, taper_degrees):
        """Build a drafted cutter per closed contour, then subtract using exact CSG."""
        if not math.isfinite(taper_degrees):
            raise ValueError("cut taper must be a finite angle in degrees")
        angle = math.radians(taper_degrees)
        results = []
        for solid in solids:
            part = solid._part
            name = part._generate_name("cut")
            cutter = None
            operands = []
            profiles = [wire for wire in wires if not wire.construction]
            additive_only = all((wire.mode or "a") == "a" for wire in profiles)
            for wire in profiles:
                if not self._is_closed_cut_wire(wire):
                    raise ValueError("tapered cuts require closed profile wires")
                mode = wire.mode or "a"
                if not additive_only and cutter is None and mode in ("s", "i"):
                    raise ValueError("a subtract/intersect sketch mode needs an existing profile")
                tool_name = name if not operands else part._generate_name("cut_profile")
                sk = self._profile_sketch([wire], tool_name + "_sk", frame=frame)
                operand = self._tapered_cut_operand(part, sk, wire, frame, height, both,
                                                    angle, tool_name)
                operands.append(operand)
                if additive_only:
                    continue
                if cutter is None or mode == "r":
                    cutter = operand
                elif mode == "s":
                    cutter = part.subtract(cutter, operand, name=name + "_profile")
                elif mode == "i":
                    cutter = part.intersect(cutter, operand, name=name + "_profile")
                else:
                    cutter = part.union(cutter, operand, name=name + "_profile")
            if not profiles:
                raise ValueError("sketch contains no cutting profiles")
            if additive_only:
                result = part.batch_subtract(solid, operands, name=solid.name)
            else:
                if cutter is None:
                    raise ValueError("sketch contains no cutting profiles")
                result = part.subtract(solid, cutter, name=solid.name)
            results.append(result)
        return self._cleared(
            _solid=results[0] if len(results) == 1 else None,
            _stack_solids=[] if len(results) == 1 else results,
            _faces=list(self._faces), _edges=list(self._edges))

    @staticmethod
    def _is_closed_cut_wire(wire):
        if wire.kind in ("circle", "ellipse"):
            return True
        if not wire.segments:
            return False
        def endpoint(segment, start):
            if segment[0] == "line":
                return segment[1] if start else segment[2]
            if segment[0] == "arc":
                return segment[1] if start else segment[3]
            if segment[0] in ("spline", "sampled") and segment[1]:
                return segment[1][0] if start else segment[1][-1]
            return None

        first = endpoint(wire.segments[0], True)
        last = endpoint(wire.segments[-1], False)
        if first is None or last is None:
            return False
        return _dist2(first, last) <= 1e-16

    @staticmethod
    def _tapered_cut_operand(part, sketch, wire, frame, height, both, angle, name):
        if not both:
            return sketch.extrude(height, name=name, taper_angle=angle)
        positive = sketch.extrude(abs(height), name=name + "_positive",
                                taper_angle=angle)
        reverse_frame = Frame(frame.origin, frame.x, -vec3(frame.y), -vec3(frame.z))
        reverse_sketch = part.sketch(frame=reverse_frame, name=name + "_negative_sk")
        wire.mapped_xy(lambda point: (point[0], -point[1])).draw(reverse_sketch)
        negative = reverse_sketch.extrude(abs(height), name=name + "_negative",
                                taper_angle=angle)
        return part.union(positive, negative, name=name + "_symmetric")

    def hole(self, diameter, depth=None, clean=True):
        """Circular hole. ``depth=None`` means thru-all."""
        wp = self.circle(0.5 * float(diameter))
        if depth is None:
            return wp.cutThruAll()
        return wp.cutBlind(-abs(float(depth)))

    def cboreHole(self, diameter, cboreDiameter, cboreDepth, depth=None, clean=True):
        """Counterbore: thru/blind hole then a larger blind cut."""
        holed = self.hole(diameter, depth=depth)
        return self._spawn(_solid=holed._solid, _stack_solids=holed._stack_solids).circle(
            0.5 * float(cboreDiameter)).cutBlind(-abs(float(cboreDepth)))

    def cskHole(self, diameter, cskDiameter, cskAngle, depth=None, clean=True):
        """Drill each selected point and cut its conical countersink."""
        diameter, cskDiameter, cskAngle = map(float, (diameter, cskDiameter, cskAngle))
        if diameter <= 0 or cskDiameter <= diameter or not 0 < cskAngle < 180:
            raise ValueError("cskHole needs 0 < diameter < cskDiameter and 0 < cskAngle < 180")
        sink_depth = (cskDiameter - diameter) / (2 * math.tan(math.radians(cskAngle) / 2))
        holed = self.hole(diameter, depth=depth)
        part = self._ensure_part()
        frame = self._active_frame()
        cutters = []
        for x, y in self._locations:
            name = self._new_name("csk")
            at_point = _frame_plus_local(frame, (x, y, 0))
            outer = part.sketch(frame=at_point, name=name + "_outer")
            outer.add_circle((0, 0), cskDiameter / 2)
            inner = part.sketch(frame=_frame_plus_local(at_point, (0, 0, -sink_depth)),
                                name=name + "_inner")
            inner.add_circle((0, 0), diameter / 2)
            cutters.append(part.loft([outer, inner], name=name))
        solid = part.batch_subtract(holed.val(), cutters, name=holed.val().name)
        return self._cleared(_solid=solid, _faces=list(self._faces), _edges=list(self._edges))

    def faces(self, selector=None, tag=None):
        """Select faces, including planar sides created by arbitrary profiles."""
        if tag is None:
            faces = list(self._faces)
        else:
            try:
                saved = self._tags[str(tag)]
            except KeyError:
                raise ValueError("unknown workplane tag {0!r}".format(tag))
            faces = list(saved[1]) if isinstance(saved, tuple) else []
        if tag is None and self._solid is not None and self.part is not None:
            known = {item["name"] for item in faces}
            for index, name in enumerate(self._solid.patch_names):
                short = name.split(":")[-1]
                if short in known:
                    continue
                try:
                    frame = self.part.plane_frame(name)
                except Exception:
                    frame = None
                kind = "plane" if frame is not None else "curved"
                if frame is None:
                    frame = self._solid.patch_frame_at(index)
                faces.append({"name": name, "center": frame.origin,
                              "normal": frame.z, "kind": kind})
        faces = [dict(face, rank_tolerance=self._tolerance) for face in faces]
        chosen = _select_faces(faces, selector)
        return self._spawn(_selected_faces=chosen, _selected_edges=[], _selected_vertices=[])

    def edges(self, selector=None):
        """Select registered edges (``|Z``, ``#Z``, named patches)."""
        frame = self._active_frame()
        edges = list(self._edges)
        if self._selected_faces:
            face_names = {face["name"] for face in self._selected_faces}
            edges = [edge for edge in edges if any(
                face_name in edge["name"] for face_name in face_names)]
        chosen = _select_edges(edges, selector)
        return self._spawn(_frame=frame, _selected_edges=chosen, _selected_faces=[])

    def toPending(self):
        """Convert selected straight edges into pending planar wire profiles.

        Edge curves are recovered from the solid mesh, so this is exact for
        straight feature edges represented by their mesh vertices.
        """
        if not self._selected_edges:
            raise ValueError("toPending() needs selected edges")
        if self._solid is None:
            raise ValueError("toPending() needs a solid")
        frame = self._active_frame()
        points, _ = self._solid.mesh()
        tolerance = max(self._tolerance, 1e-7)
        segments = []
        for edge in self._selected_edges:
            if edge.get("kind") != "line":
                raise NotImplementedError("toPending() currently supports straight edges")
            center = vec3(edge["center"])
            direction = _unit(edge["direction"])
            on_edge = []
            for point in points:
                delta = vec3(point) - center
                along = _dot(delta, direction)
                if _norm(delta - direction * along) <= tolerance:
                    on_edge.append((along, vec3(point)))
            if len(on_edge) < 2:
                raise ValueError("could not recover edge endpoints from the solid mesh")
            start, end = min(on_edge, key=lambda item: item[0])[1], max(
                on_edge, key=lambda item: item[0])[1]
            segments.append((self._to_local_xy(start, frame), self._to_local_xy(end, frame)))

        wires = []
        while segments:
            first = segments.pop(0)
            path = [first[0], first[1]]
            while segments and _dist2(path[-1], path[0]) > tolerance * tolerance:
                match = next((i for i, seg in enumerate(segments)
                              if _dist2(seg[0], path[-1]) <= tolerance * tolerance or
                              _dist2(seg[1], path[-1]) <= tolerance * tolerance), None)
                if match is None:
                    break
                start, end = segments.pop(match)
                path.append(end if _dist2(start, path[-1]) <= tolerance * tolerance else start)
            wires.append(_Wire.from_points(path, closed=_dist2(path[-1], path[0]) <= tolerance * tolerance))
        return self._spawn(_frame=frame, _pending=self._pending + wires,
                           _selected_edges=[], _selected_faces=[])

    @staticmethod
    def _to_local_xy(point, frame):
        delta = vec3(point) - vec3(frame.origin)
        return (_dot(delta, vec3(frame.x)), _dot(delta, vec3(frame.y)))

    def fillet(self, radius):
        """Fillet currently selected edges."""
        names = [edge["name"] for edge in self._selected_edges]
        if not names:
            raise ValueError("fillet needs selected edges, e.g. .edges('|Z').fillet(0.2)")
        if self._solid is None:
            raise ValueError("fillet needs a solid")
        solid = self._solid.fillet(names, float(radius), name=self._solid.name)
        return self._cleared(_solid=solid, _faces=list(self._faces), _edges=list(self._edges))

    def chamfer(self, length, length2=None):
        """Chamfer currently selected edges. Asymmetric ``length2`` is not implemented."""
        if length2 is not None:
            raise NotImplementedError("asymmetric chamfer needs a kernel change")
        names = [edge["name"] for edge in self._selected_edges]
        if not names:
            raise ValueError("chamfer needs selected edges, e.g. .edges('|Z').chamfer(0.2)")
        if self._solid is None:
            raise ValueError("chamfer needs a solid")
        solid = self._solid.chamfer(names, float(length), name=self._solid.name)
        return self._cleared(_solid=solid, _faces=list(self._faces), _edges=list(self._edges))

    def val(self):
        """The current camber Solid (raises if none)."""
        if self._selected_faces and self._solid is not None:
            return self._solid.face_surface(self._selected_faces[0]["name"])
        if self._stack_solids:
            return self._stack_solids[0]
        if self._solid is None:
            raise ValueError("Workplane has no solid; extrude or box first")
        return self._solid

    def vals(self):
        """Solids on the current stack."""
        return list(self._stack_solids) if self._stack_solids else (
            [] if self._solid is None else [self._solid])

    def all(self):
        """One Workplane per solid on the current stack."""
        return [self._cleared(_solid=solid, _faces=[], _edges=[])
                for solid in self.vals()]

    def largestDimension(self):
        """Diagonal of the Part AABB used when this Workplane created a Part."""
        return 2.0 * self._size * math.sqrt(3.0)

    def show(self, title="Camber"):
        """Open the 3D viewer on the current solid or complete solid stack."""
        solids = self.vals()
        if not solids:
            raise ValueError("Workplane has no solid to show")
        if len(solids) == 1:
            solids[0].show(title=title)
            return self

        # A display operation must not boolean-union the stack: that changes
        # the model and drops the individual solids' named edges and anchors.
        from .display import DisplayScene, decode_native_solid
        from .view import show as show_scene

        scene = DisplayScene()
        for solid in solids:
            scene.extend(decode_native_solid(solid._n))
        show_scene(scene, title=title)
        return self

    def __add__(self, other):
        """Same as ``union``."""
        return self.union(other)

    def __sub__(self, other):
        """Same as ``cut``."""
        return self.cut(other)

    def __and__(self, other):
        """Same as ``intersect``."""
        return self.intersect(other)

    def vertices(self, selector=None):
        """Select vertices on the selected planar face."""
        if self._construction_vertices:
            if selector:
                raise ValueError("construction vertex selectors are not yet supported")
            return self._spawn(_locations=list(self._construction_vertices))
        if self._solid is None or not self._selected_faces:
            raise NotImplementedError("vertices() currently needs a selected planar face")
        face = self._selected_faces[0]
        center, normal = vec3(face["center"]), _unit(face["normal"])
        points, _ = self._solid.mesh()
        candidates = [p for p in points if abs(_dot(p - center, normal)) < max(self._tolerance, 1e-6)]
        if not candidates:
            raise ValueError("selected face has no mesh vertices")
        if selector:
            match = re.fullmatch(r"([<>])([XYZ]{1,3})", str(selector).strip().upper())
            if not match:
                raise ValueError("unsupported vertex selector {0!r}".format(selector))
            sign = -1 if match.group(1) == "<" else 1
            axes = match.group(2)
            point = max(candidates, key=lambda p: sign * sum(getattr(p, axis.lower()) for axis in axes))
            candidates = [point]
        return self._spawn(_selected_vertices=candidates)

    def wires(self, selector=None):
        """Not implemented (needs BRep topology)."""
        raise NotImplementedError("wires() needs BRep topology")

    def shells(self, selector=None):
        """Not implemented (needs BRep topology)."""
        raise NotImplementedError("shells() needs BRep topology")

    def solids(self, selector=None):
        """No-op selector; returns this Workplane (single solid)."""
        return self

    def translate(self, vec):
        """Return an independent translated solid."""
        if self._solid is None:
            raise ValueError("translate needs a solid")
        moved = self._solid.pattern_linear(2, _as_vec3(vec))[1]
        return self._cleared(_solid=moved, _faces=[], _edges=[])

    def rotate(self, axisStart, axisEnd, angleDegrees):
        """Return an independent rotation about a world-space axis."""
        if self._solid is None:
            raise ValueError("rotate needs a solid")
        start, end = _as_vec3(axisStart), _as_vec3(axisEnd)
        if _norm(end - start) < 1e-12:
            raise ValueError("rotate axis is degenerate")
        axis = _plane_from_normal(start, end - start, self._frame.x)
        moved = self._solid.pattern_circular(
            2, axis=axis, angle=math.radians(float(angleDegrees)))[1]
        return self._cleared(_solid=moved, _faces=[], _edges=[])

    def rotateAboutCenter(self, axis, angleDegrees):
        """Rotate the solid around an axis through its bounding-box center."""
        solid = self.val()
        points, _ = solid.mesh()
        center = vec3(*(
            (min(getattr(point, axis_name) for point in points) +
             max(getattr(point, axis_name) for point in points)) * 0.5
            for axis_name in ("x", "y", "z")))
        direction = _as_vec3(axis)
        if _norm(direction) < 1e-12:
            raise ValueError("rotation axis is degenerate")
        return self.rotate(center, center + direction, angleDegrees)

    def mirror(self, mirrorPlane="XY", basePointVector=(0, 0, 0), union=False):
        """Reflect a solid across a named plane; optionally union the source."""
        if self._solid is None:
            raise ValueError("mirror needs a solid")
        if isinstance(mirrorPlane, Workplane):
            if not mirrorPlane._selected_faces:
                raise ValueError("mirror workplane must have a selected face")
            face = mirrorPlane._selected_faces[0]
            try:
                plane = self.part.plane_frame(face["name"])
            except Exception:
                plane = None
            if plane is None:
                plane = _plane_from_normal(face["center"], face["normal"], self._frame.x)
        else:
            plane = mirrorPlane if isinstance(mirrorPlane, Frame) else named_plane(mirrorPlane, basePointVector)
        mirrored = self._solid.mirror(plane=plane)
        if union:
            mirrored = self.part.union(self._solid, mirrored)
        return self._cleared(_solid=mirrored, _faces=[], _edges=[])

    def shell(self, thickness, kind="arc"):
        """Shell inward for negative thickness, outward for positive thickness.

        ``kind="arc"`` requests round outward joins; ``"intersection"``
        keeps the existing sharp joins. Complex round offsets can fail.
        """
        thickness = float(thickness)
        if thickness == 0:
            raise ValueError("shell thickness must be nonzero")
        if kind not in ("intersection", "arc"):
            raise ValueError("shell kind must be 'intersection' or 'arc'")
        if self._solid is None:
            raise ValueError("shell needs a solid")
        faces = [face["name"] for face in self._selected_faces]
        join = "round" if thickness > 0 and kind == "arc" else "sharp"
        shelled = self._solid.shell(abs(thickness), faces=faces,
                                  outward=thickness > 0, join=join)
        return self._cleared(_solid=shelled, _faces=[], _edges=[])

    def split(self, keepTop=False, keepBottom=False):
        """Split at this workplane; keep either or both closed halves."""
        if not keepTop and not keepBottom:
            raise ValueError("split() must keep at least one half")
        if self._solid is None:
            raise ValueError("split() needs a solid")

        frame = self._active_frame()
        mesh_points, triangles = self._solid.mesh()
        used = {index for triangle in triangles for index in triangle}
        if not used:
            raise ValueError("split() needs a nonempty solid")
        points = [mesh_points[index] for index in used]
        bounds = [(min(getattr(point, axis) for point in points),
                   max(getattr(point, axis) for point in points))
                  for axis in ("x", "y", "z")]
        box_corners = [vec3(x, y, z)
                       for x in bounds[0] for y in bounds[1] for z in bounds[2]]
        projected = [self._to_local_xy(point, frame) for point in box_corners]
        u0, u1 = min(p[0] for p in projected), max(p[0] for p in projected)
        v0, v1 = min(p[1] for p in projected), max(p[1] for p in projected)
        distances = [_dot(point - vec3(frame.origin), vec3(frame.z)) for point in box_corners]
        if min(distances) >= 0 or max(distances) <= 0:
            raise ValueError("split plane must cross the solid")
        margin = max(self._tolerance * 4, (u1 - u0 + v1 - v0) * 0.01)
        sketch = self.part.sketch(frame=frame, name=self._new_name("split_profile"))
        sketch.add_rectangle((u0 - margin, v0 - margin), (u1 + margin, v1 + margin))
        top = bottom = None
        if keepTop:
            cutter = sketch.extrude(max(distances) + margin,
                                       name=self._new_name("split_top_tool"))
            top = self.part.intersect(self._solid, cutter, name=self._new_name("split_top"))
        if keepBottom:
            cutter = sketch.extrude_two_sides(
                0, -min(distances) + margin,
                name=self._new_name("split_bottom_tool"))
            bottom = self.part.intersect(self._solid, cutter, name=self._new_name("split_bottom"))
        if top is not None and bottom is not None:
            return self._cleared(_stack_solids=[top, bottom], _faces=[], _edges=[])
        return self._cleared(_solid=top or bottom, _faces=[], _edges=[])

    def offset2D(self, d, kind="arc", forConstruction=False):
        """Offset pending closed profiles; ``kind`` is ``arc`` or ``intersection``."""
        wires = self._wires()
        if not wires:
            raise ValueError("offset2D() needs a pending closed profile")
        joins = {"arc": "round", "round": "round",
                 "intersection": "miter", "miter": "miter"}
        join = joins.get(str(kind).lower())
        if join is None:
            raise ValueError("kind must be 'arc' or 'intersection'")

        results, vertices = [], []
        for wire in wires:
            if wire.kind == "circle":
                center, radius = wire.segments[0][1], wire.segments[0][2]
                new_radius = float(radius) + float(d)
                if new_radius <= 0:
                    raise ValueError("offset collapses a circle")
                results.append(_Wire.circle(center, new_radius))
                continue
            if (not wire.segments or wire.segments[0][0] != "line" or
                    wire.segments[-1][0] != "line" or
                    _dist2(wire.segments[0][1], wire.segments[-1][2]) > self._tolerance ** 2):
                raise ValueError("offset2D() needs closed profiles")
            sketch = self._ensure_part().sketch(
                frame=self._active_frame(), name=self._new_name("offset"))
            curves = wire.draw(sketch)
            before = 1  # This sketch contains exactly one closed source wire.
            side = "out" if float(d) >= 0 else "in"
            if _wire_signed_area(wire) < 0:
                side = "in" if side == "out" else "out"
            sketch.offset(curves, abs(float(d)), side=side, join=join)
            polylines = sketch.polylines()[before:]
            if not polylines:
                raise ValueError("offset produced no profile")
            for polyline in polylines:
                points = [(float(point.x), float(point.y)) for point in polyline]
                if len(points) < 3:
                    continue
                if _dist2(points[0], points[-1]) > 1e-16:
                    points.append(points[0])
                vertices.extend(points[:-1])
                results.append(_Wire.from_points(points, closed=True))
        if not results:
            raise ValueError("offset produced no profile")
        if forConstruction:
            return self._spawn(_pending=[], _construction_vertices=vertices,
                               _locations=list(vertices), _open=None, _open_segments=[],
                               _selected_edges=[], _selected_faces=[])
        return self._spawn(_pending=results,
                           _open=None, _open_segments=[],
                           _selected_edges=[], _selected_faces=[])

    def spline(self, listOfXY, tangents=None, periodic=False, includeCurrent=False):
        """Add a cubic Hermite spline to the current 2D path."""
        if tangents is not None or periodic:
            raise NotImplementedError("tangents and periodic splines are not yet supported")
        points = [self._path_point(p) for p in listOfXY]
        if includeCurrent:
            points.insert(0, self._cursor)
        if len(points) < 2:
            raise ValueError("spline needs at least two points")
        if self._open_segments and _dist2(self._cursor, points[0]) > 1e-18:
            raise ValueError("spline must begin at the current 2D point")
        open_points = list(self._open or [points[0]]) + points[1:]
        return self._spawn(_cursor=points[-1], _open=open_points,
                           _open_segments=self._open_segments + [("spline", points)])

    def parametricCurve(self, func, N=400, start=0, stop=1, tol=None,
                        minDeg=1, maxDeg=6, smoothing=(1, 1, 1), makeWire=True):
        """Add a sampled planar callback curve as a pending sketch wire.

        Sampling starts with ``N`` parameter intervals and adaptively refines
        chords to the explicit ``tol`` or the Part's tessellation tolerance.
        Error is estimated at quarter points on each parameter span; arbitrary
        callbacks cannot provide a strict error bound without derivative or
        curvature information.
        Curves must return 2D points. The native sketch stores the samples as
        one sampled curve; spline fitting controls and ``makeWire=False`` are
        not supported.
        """
        if not makeWire:
            raise NotImplementedError("parametricCurve currently creates sketch wires only")
        if minDeg != 1 or maxDeg != 6 or smoothing not in (None, (1, 1, 1)):
            raise NotImplementedError("parametricCurve spline fitting controls are not supported for sampled wires")
        tolerance = self._ensure_part().max_deviation if tol is None else float(tol)
        points = [self._path_point(point)
                  for point in _sample_parametric_curve(func, N, start, stop, tolerance)]
        if _dist2(points[0], points[-1]) <= tolerance * tolerance:
            points[-1] = points[0]
        wire = _Wire([("sampled", points)], kind="path")
        return self._spawn(_pending=self._pending + [wire])

    def each(self, callback, useLocalCoordinates=False, combine=True, clean=True):
        """Not implemented."""
        raise NotImplementedError("each() is not implemented")

    def eachpoint(self, callback, useLocalCoordinates=False, combine=False, clean=True):
        """Build one solid per pushed point using ``callback(location)``.

        The callback receives a world-space ``vec3`` at each workplane location.
        By default it is responsible for placing its result there. With
        ``useLocalCoordinates=True``, it receives the local XY point and its
        result is transformed from the active workplane to that point.
        """
        if not callable(callback):
            raise TypeError("eachpoint callback must be callable")

        frame = self._active_frame()
        solids = []
        for x, y in self._locations:
            world_location = _world_from_local(frame, (x, y, 0.0))
            location = vec3(x, y, 0.0) if useLocalCoordinates else world_location
            result = callback(location)
            solid = result.val() if isinstance(result, Workplane) else result
            if not isinstance(solid, Solid):
                raise TypeError("eachpoint callback must return a Solid or a Workplane containing one Solid")

            if not useLocalCoordinates and (solid._part is self.part or not combine):
                solids.append(solid)
                continue

            points, triangles = solid.mesh()
            if useLocalCoordinates:
                points = [_world_from_local(_frame_plus_local(frame, (x, y, 0.0)),
                                            (point.x, point.y, point.z))
                          for point in points]
            # The target Part owns the returned stack, independent of the
            # callback's temporary Part.
            solids.append(self._ensure_part().solid_from_mesh(
                points, triangles, name=self._new_name("eachpoint")))

        if not solids:
            raise ValueError("eachpoint needs at least one location")
        if combine:
            solid = solids[0] if len(solids) == 1 else self._ensure_part().batch_union(solids)
            return self._cleared(_solid=solid, _stack_solids=[])
        return self._cleared(_solid=None, _stack_solids=solids)

    def __repr__(self):
        solid = None if self._solid is None else self._solid.name
        return "Workplane(solid={0!r}, pending={1}, faces={2})".format(
            solid, len(self._pending), len(self._selected_faces))


__all__ = [
    "Workplane",
    "Sketch",
    "Vector",
    "named_plane",
]
