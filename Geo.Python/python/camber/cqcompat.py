"""CadQuery-shaped fluent API on top of camber (Workplane / Sketch).

This is not a drop-in CadQuery replacement. Solids are camber meshes with named
patches, so selectors only see faces and edges the adapter registered (caps,
named sides, and the usual ``|Z`` / ``>Z`` / ``#Z`` filters). Operations that
need BRep topology or a kernel transform (shell, solid move/rotate/mirror,
2D offset, splines, …) raise ``NotImplementedError``.

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
    Curve,
    Frame,
    LoftOptions,
    Part,
    Solid,
)
from .vec import vec2, vec3

Vector = vec3

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
        fallback = (1.0, 0.0, 0.0) if abs(z.z) < 0.9 else (0.0, 1.0, 0.0)
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
                chosen = [item for item in chosen if abs(_dot(item["normal"], axis)) < 0.35]
            else:
                chosen = [item for item in chosen if abs(_dot(item["direction"], axis)) > 0.85]
        elif op == "#":
            if kind == "face":
                chosen = [item for item in chosen if abs(_dot(item["normal"], axis)) > 0.85]
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


def _revolve_caps(name, frame, z0, z1, radius):
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
        {"name": "[{0}-side,{0}-ExtrudeTop]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (0.0, 0.0, z1)), "kind": "circle"},
        {"name": "[{0}-side,{0}-ExtrudeBottom]".format(name),
         "direction": vec3(frame.x), "center": _world_from_local(frame, (0.0, 0.0, z0)), "kind": "circle"},
    ]
    return faces, edges


class _Wire(object):
    def __init__(self, segments, kind="path", name=None):
        self.segments = list(segments)
        self.kind = kind
        self.name = name

    @staticmethod
    def circle(center, radius, name=None):
        return _Wire([("circle", _xy2(center), float(radius))], kind="circle", name=name)

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
            else:
                segs.append(seg)
        return _Wire(segs, kind=self.kind, name=self.name)

    def swapped_xy(self):
        """Map CadQuery (x, y) onto a camber revolve sketch (axis=X, radius=Y)."""
        segs = []
        for seg in self.segments:
            if seg[0] == "circle":
                c = seg[1]
                segs.append(("circle", (c[1], c[0]), seg[2]))
            elif seg[0] == "line":
                extra = (seg[3],) if len(seg) > 3 else ()
                segs.append(("line", (seg[1][1], seg[1][0]), (seg[2][1], seg[2][0])) + extra)
            elif seg[0] == "arc":
                segs.append((
                    "arc",
                    (seg[1][1], seg[1][0]),
                    (seg[2][1], seg[2][0]),
                    (seg[3][1], seg[3][0]),
                ))
            else:
                segs.append(seg)
        return _Wire(segs, kind=self.kind, name=self.name)

    def draw(self, sketch):
        for i, seg in enumerate(self.segments):
            if seg[0] == "circle":
                sketch.add_circle(seg[1], seg[2], name=self.name)
            elif seg[0] == "line":
                name = seg[3] if len(seg) > 3 else (self.name if i == 0 else None)
                sketch.add_line(seg[1], seg[2], name=name)
            elif seg[0] == "arc":
                sketch.add_arc(seg[1], seg[2], seg[3], name=self.name)
        return self


class Sketch(object):
    """CadQuery-shaped 2D sketch. ``finalize()`` returns the parent Workplane."""

    def __init__(self, parent):
        self._parent = parent
        self._wires = []
        self._cursor = (0.0, 0.0)
        self._open = None

    def rect(self, xLen, yLen, centered=True, mode="a"):
        """Add a rectangle at the parent Workplane locations. ``mode`` is unused."""
        self._wires.extend(_rect_wires(xLen, yLen, centered, self._parent._locations))
        return self

    def circle(self, radius, mode="a"):
        """Add a circle at the parent Workplane locations. ``mode`` is unused."""
        self._wires.extend(_circle_wires(radius, self._parent._locations))
        return self

    def push(self, locs):
        """Set 2D locations (same as Workplane.pushPoints)."""
        self._parent = self._parent._spawn(_locations=[_xy2(p) for p in locs])
        return self

    def finalize(self):
        """Return a Workplane with this sketch's wires pending."""
        return self._parent._spawn(_pending=self._parent._pending + self._wires)

    def fillet(self, radius):
        """Not implemented on cqcompat.Sketch."""
        raise NotImplementedError("2D sketch fillet needs a kernel change")

    def vertices(self, selector=None):
        """Not implemented on cqcompat.Sketch."""
        raise NotImplementedError("sketch vertices are not exposed by camber")


def _rect_wires(x_len, y_len, centered, locations, names=None):
    sx, sy = _center_shifts((x_len, y_len), centered)
    xmin, xmax = sx, sx + float(x_len)
    ymin, ymax = sy, sy + float(y_len)
    wires = []
    for ox, oy in locations:
        wires.append(_Wire.rect(xmin + ox, xmax + ox, ymin + oy, ymax + oy, names=names))
    return wires


def _circle_wires(radius, locations, name=None):
    return [_Wire.circle(loc, radius, name=name) for loc in locations]


def _named_rect_sides(wires):
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
            elif seg[0] in ("line", "arc"):
                for pt in seg[1:4] if seg[0] == "arc" else seg[1:3]:
                    xs.append(float(pt[0]))
                    ys.append(float(pt[1]))
    if not xs:
        return None
    return (min(xs), max(xs), min(ys), max(ys))


class Workplane(object):
    """CadQuery-shaped modelling context backed by one shared camber ``Part``."""

    def __init__(
            self, inPlane="XY", origin=None, obj=None, part=None,
            size=1000.0, tolerance=0.01):
        """Start on a named plane (``XY``, ``front``, …) or a Frame.

        ``size`` is the Part AABB half-extent when this Workplane creates a Part.
        ``obj`` may be a camber Solid to continue from. ``part`` reuses an existing Part.
        """
        self._size = float(size)
        self._tolerance = float(tolerance)
        if obj is not None and isinstance(obj, Solid):
            part = obj._part
        if isinstance(inPlane, Workplane):
            self._copy_from(inPlane)
            if origin is not None:
                self._frame = Frame(vec3(origin), self._frame.x, self._frame.y, self._frame.z)
            if obj is not None:
                self._solid = self._as_solid(obj)
            return
        if isinstance(inPlane, Frame):
            self._frame = inPlane
            if origin is not None:
                self._frame = Frame(vec3(origin), self._frame.x, self._frame.y, self._frame.z)
        else:
            self._frame = named_plane(inPlane, origin)
        self.part = part
        self._solid = self._as_solid(obj)
        self._pending = []
        self._cursor = (0.0, 0.0)
        self._open = None
        self._locations = [(0.0, 0.0)]
        self._sections = []
        self._faces = []
        self._edges = []
        self._selected_faces = []
        self._selected_edges = []

    def _copy_from(self, other):
        self.part = other.part
        self._size = other._size
        self._tolerance = other._tolerance
        self._frame = other._frame
        self._solid = other._solid
        self._pending = list(other._pending)
        self._cursor = other._cursor
        self._open = None if other._open is None else list(other._open)
        self._locations = list(other._locations)
        self._sections = list(other._sections)
        self._faces = list(other._faces)
        self._edges = list(other._edges)
        self._selected_faces = list(other._selected_faces)
        self._selected_edges = list(other._selected_edges)

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
        if isinstance(obj, Solid):
            return obj
        if isinstance(obj, Workplane):
            return obj._solid
        raise TypeError("expected a camber Solid or Workplane")

    def _wires(self):
        wires = list(self._pending)
        if self._open and len(self._open) >= 2:
            wires.append(_Wire.from_points(self._open, closed=True))
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
            return Frame(origin, kernel.x, kernel.y, kernel.z)
        return _plane_from_normal(origin, normal, self._frame.x)

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
            merged = self.part.cut(self._solid, solid, name=name)
            return self._cleared(_solid=merged, _faces=list(self._faces), _edges=list(self._edges))
        if combine in ("intersect", "i"):
            merged = self.part.intersect(self._solid, solid, name=name)
            return self._cleared(_solid=merged, _faces=list(self._faces), _edges=list(self._edges))
        raise ValueError("unknown combine mode {0!r}".format(combine))

    def _cleared(self, **updates):
        defaults = dict(
            _pending=[],
            _open=None,
            _cursor=(0.0, 0.0),
            _sections=[],
            _selected_faces=[],
            _selected_edges=[],
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
            face = self._selected_faces[0]
            origin = vec3(face["center"])
            normal = vec3(face["normal"])
            if str(centerOption) == "ProjectedOrigin":
                origin = _project_to_plane(self._frame.origin, origin, normal)
            try:
                kernel = self._ensure_part().plane_frame(face["name"]) if self.part is not None else None
            except Exception:
                kernel = None
            if kernel is not None:
                frame = kernel
                if str(centerOption) != "ProjectedOrigin":
                    frame = Frame(origin, kernel.x, kernel.y, kernel.z)
            else:
                frame = _plane_from_normal(origin, normal, self._frame.x)
        else:
            frame = self._frame
            sections = list(self._sections)
            pending = list(self._pending)
            if pending:
                sections.append((self._frame, pending))
            frame = _frame_plus_local(frame, (0.0, 0.0, offset))
            if invert:
                frame = Frame(frame.origin, -vec3(frame.x), vec3(frame.y), -vec3(frame.z))
            return self._spawn(
                _frame=frame,
                _pending=[],
                _open=None,
                _cursor=(0.0, 0.0),
                _sections=sections,
                _selected_faces=[],
                _selected_edges=[],
            )
        if invert:
            frame = Frame(frame.origin, -vec3(frame.x), vec3(frame.y), -vec3(frame.z))
        frame = _frame_plus_local(frame, (0.0, 0.0, offset))
        return self._spawn(
            _frame=frame,
            _pending=[],
            _open=None,
            _cursor=(0.0, 0.0),
            _selected_faces=[],
            _selected_edges=[],
        )

    def transformed(self, rotate=(0, 0, 0), offset=(0, 0, 0)):
        """Rotate then translate the workplane (degrees, local XYZ)."""
        frame = _rotate_frame(self._frame, rotate)
        frame = _frame_plus_local(frame, offset)
        return self._spawn(_frame=frame)

    def center(self, x, y):
        """Shift the workplane origin in the current XY."""
        return self._spawn(_frame=_frame_plus_local(self._frame, (float(x), float(y), 0.0)))

    def moveTo(self, x, y):
        """Set the 2D cursor and start a new open path."""
        pt = (float(x), float(y))
        return self._spawn(_cursor=pt, _open=[pt])

    def move(self, xDist, yDist):
        """Relative move of the 2D cursor."""
        return self.moveTo(self._cursor[0] + float(xDist), self._cursor[1] + float(yDist))

    def lineTo(self, x, y):
        """Line from the cursor to an absolute 2D point."""
        end = (float(x), float(y))
        pts = list(self._open or [self._cursor])
        if not pts:
            pts = [self._cursor]
        pts.append(end)
        return self._spawn(_cursor=end, _open=pts)

    def line(self, xDist, yDist):
        """Relative line from the cursor."""
        return self.lineTo(self._cursor[0] + float(xDist), self._cursor[1] + float(yDist))

    def hLine(self, distance):
        """Horizontal relative line."""
        return self.line(distance, 0.0)

    def vLine(self, distance):
        """Vertical relative line."""
        return self.line(0.0, distance)

    def hLineTo(self, x):
        """Horizontal line to absolute X."""
        return self.lineTo(x, self._cursor[1])

    def vLineTo(self, y):
        """Vertical line to absolute Y."""
        return self.lineTo(self._cursor[0], y)

    def close(self):
        """Close the open path into a pending wire."""
        if not self._open or len(self._open) < 2:
            raise ValueError("close() needs a path with at least one segment")
        wire = _Wire.from_points(self._open, closed=True)
        return self._spawn(_pending=self._pending + [wire], _open=None, _cursor=self._open[0])

    def rect(self, xLen, yLen, centered=True, forConstruction=False):
        """Pending rectangle. Construction flag is ignored (not drawn)."""
        if forConstruction:
            return self
        return self._spawn(_pending=self._pending + _rect_wires(xLen, yLen, centered, self._locations))

    def circle(self, radius, forConstruction=False):
        """Pending circle. Construction flag is ignored (not drawn)."""
        if forConstruction:
            return self
        return self._spawn(_pending=self._pending + _circle_wires(radius, self._locations, name="side"))

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
        pts = [_xy2(p) for p in listOfXY]
        if includeCurrent:
            pts = [self._cursor] + pts
        if len(pts) < 2:
            raise ValueError("polyline needs at least two points")
        return self._spawn(_cursor=pts[-1], _open=pts)

    def threePointArc(self, point1, point2):
        """Arc from the cursor through ``point1`` to ``point2``."""
        start = self._cursor
        mid = _xy2(point1)
        end = _xy2(point2)
        wire = _Wire([("arc", start, mid, end)], kind="path")
        pts = list(self._open or [start])
        pts.append(end)
        return self._spawn(_pending=self._pending + [wire], _cursor=end, _open=pts)

    def radiusArc(self, endPoint, radius):
        """Arc from the cursor to ``endPoint`` with signed radius (CCW if positive)."""
        end = _xy2(endPoint)
        mid = _radius_arc_mid(self._cursor, end, radius)
        return self.threePointArc(mid, end)

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
        return self._spawn(_locations=[_xy2(p) for p in pntList])

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
                wires.extend(item._wires)
            else:
                raise TypeError("placeSketch expects camber.cqcompat.Sketch instances")
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
        solid = self.part.extrude(sk, zmax - zmin, name=name)
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
            solid = self.part.extrude(sk, z1 - z0, name=name)
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

    def extrude(self, until, combine=True, clean=True, both=False, taper=0):
        """Extrude pending 2D wires. ``taper`` is not implemented."""
        if taper:
            raise NotImplementedError("tapered extrude needs a kernel change")
        return self._extrude_pending(float(until), combine=combine, both=both, twist=0.0)

    def twistExtrude(self, distance, angleDegrees, combine=True, clean=True):
        """Extrude with twist (degrees along the height)."""
        distance = float(distance)
        if abs(distance) < 1e-15:
            raise ValueError("twistExtrude distance must be non-zero")
        # camber twist is radians per unit length; CadQuery uses total degrees.
        twist = math.radians(float(angleDegrees)) / distance
        return self._extrude_pending(distance, combine=combine, both=False, twist=twist)

    def _extrude_pending(self, height, combine, both, twist):
        wires = self._wires()
        name = self._new_name("ext")
        sk = self._profile_sketch(wires, name + "_sk")
        if both:
            solid = self.part.extrude(sk, abs(height), name=name, both_sides=True, twist=twist)
            z0, z1 = -abs(height), abs(height)
        else:
            solid = self.part.extrude(sk, height, name=name, twist=twist)
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
        for wire in wires:
            if wire.kind == "circle":
                radius = wire.segments[0][2]
                center = wire.segments[0][1]
                faces.append({
                    "name": name + "-side",
                    "center": _world_from_local(
                        frame, (center[0] + radius, center[1], 0.5 * (z0 + z1))),
                    "normal": vec3(frame.x),
                    "kind": "cylinder",
                })
                edges.extend(_revolve_caps(name, frame, z0, z1, radius)[1])
        return faces, edges

    def revolve(self, angleDegrees=360, axisStart=None, axisEnd=None, combine=True, clean=True):
        """Revolve pending wires. Default axis is workplane X through the origin."""
        wires = [wire.swapped_xy() for wire in self._wires()]
        origin, x_axis, y_axis = self._revolve_axes(axisStart, axisEnd)
        z_axis = _unit(_cross(x_axis, y_axis))
        y_axis = _unit(_cross(z_axis, x_axis))
        frame = Frame(origin, x_axis, y_axis, z_axis)
        name = self._new_name("rev")
        sk = self._profile_sketch(wires, name + "_sk", frame=frame)
        solid = self.part.revolve(sk, math.radians(float(angleDegrees)), name=name)
        return self._apply_combine(solid, [], [], combine)

    def _revolve_axes(self, axis_start, axis_end):
        if axis_start is None and axis_end is None:
            # CadQuery default: workplane Y. Camber revolve axis is sketch X.
            frame = self._active_frame()
            return vec3(frame.origin), vec3(frame.y), vec3(frame.x)
        frame = self._active_frame()
        start = self._axis_point(axis_start)
        end = self._axis_point(axis_end)
        x_axis = _unit(end - start)
        if _norm(x_axis) < 1e-12:
            raise ValueError("revolve axis is degenerate")
        hint = frame.x
        if abs(_dot(hint, x_axis)) > 0.95:
            hint = frame.z
        y_axis = hint - x_axis * _dot(hint, x_axis)
        if _norm(y_axis) < 1e-9:
            y_axis = frame.y - x_axis * _dot(frame.y, x_axis)
        return start, x_axis, _unit(y_axis)

    def _axis_point(self, point):
        frame = self._active_frame()
        if point is None:
            return vec3(frame.origin)
        if len(point) == 2:
            return _world_from_local(frame, (float(point[0]), float(point[1]), 0.0))
        return _as_vec3(point)

    def sweep(self, path, combine=True, clean=True):
        """Sweep pending profile along ``path`` (a Workplane with an open 2D path)."""
        wires = self._wires()
        name = self._new_name("swp")
        profile = self._profile_sketch(wires, name + "_profile")
        if isinstance(path, Curve):
            solid = self.part.extrude_along_curve(profile, path, name=name)
        elif isinstance(path, Workplane):
            guide_wires = path._wires()
            if not guide_wires:
                raise ValueError("sweep path has no pending 2D geometry")
            guide = path._profile_sketch(guide_wires, name + "_guide")
            solid = self.part.extrude_along_sketch(profile, guide, name=name)
        else:
            raise TypeError("sweep path must be a Workplane or camber Curve")
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
            _solid=self.part.cut(self._solid, other, name=self._solid.name),
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
        """Cut pending wires to a depth. ``taper`` is not implemented."""
        if taper:
            raise NotImplementedError("tapered cutBlind needs a kernel change")
        return self._cut_pending(float(until), both=both)

    def cutThruAll(self, clean=True, taper=0):
        """Cut pending wires through the part AABB. ``taper`` is not implemented."""
        if taper:
            raise NotImplementedError("tapered cutThruAll needs a kernel change")
        return self._cut_pending(2.0 * self._size, both=True)

    def _cut_pending(self, height, both):
        if self._solid is None:
            raise ValueError("cut needs an existing solid")
        wires = self._wires()
        name = self._new_name("cut")
        sk = self._profile_sketch(wires, name + "_sk")
        if both:
            cutter = self.part.extrude_two_sides(sk, abs(height), abs(height), name=name)
        elif height >= 0:
            cutter = self.part.extrude(sk, height, name=name)
        else:
            cutter = self.part.extrude_two_sides(sk, 0.0, abs(height), name=name)
        cut = self.part.cut(self._solid, cutter, name=self._solid.name)
        return self._cleared(_solid=cut, _faces=list(self._faces), _edges=list(self._edges))

    def hole(self, diameter, depth=None, clean=True):
        """Circular hole. ``depth=None`` means thru-all."""
        wp = self.circle(0.5 * float(diameter))
        if depth is None:
            return wp.cutThruAll()
        return wp.cutBlind(float(depth))

    def cboreHole(self, diameter, cboreDiameter, cboreDepth, depth=None, clean=True):
        """Counterbore: thru/blind hole then a larger blind cut."""
        wp = self.hole(diameter, depth=depth)
        return wp.circle(0.5 * float(cboreDiameter)).cutBlind(float(cboreDepth))

    def faces(self, selector=None):
        """Select registered faces (``>Z``, ``|Z``, named patches)."""
        chosen = _select_faces(self._faces, selector)
        return self._spawn(_selected_faces=chosen, _selected_edges=[])

    def edges(self, selector=None):
        """Select registered edges (``|Z``, ``#Z``, named patches)."""
        chosen = _select_edges(self._edges, selector)
        return self._spawn(_selected_edges=chosen, _selected_faces=[])

    def fillet(self, radius):
        """Fillet currently selected edges."""
        names = [edge["name"] for edge in self._selected_edges]
        if not names:
            raise ValueError("fillet needs selected edges, e.g. .edges('|Z').fillet(0.2)")
        if self._solid is None:
            raise ValueError("fillet needs a solid")
        solid = self.part.fillet(self._solid, names, float(radius), name=self._solid.name)
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
        solid = self.part.chamfer(self._solid, names, float(length), name=self._solid.name)
        return self._cleared(_solid=solid, _faces=list(self._faces), _edges=list(self._edges))

    def val(self):
        """The current camber Solid (raises if none)."""
        if self._solid is None:
            raise ValueError("Workplane has no solid; extrude or box first")
        return self._solid

    def vals(self):
        """List of solids (empty or one item)."""
        return [] if self._solid is None else [self._solid]

    def largestDimension(self):
        """Diagonal of the Part AABB used when this Workplane created a Part."""
        return 2.0 * self._size * math.sqrt(3.0)

    def show(self, title="Camber"):
        """Open the 3D viewer on ``val()``."""
        self.val().show(title=title)
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
        """Not implemented (needs BRep topology)."""
        raise NotImplementedError("vertices() needs BRep topology")

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
        """Not implemented (kernel has no solid transform yet)."""
        raise NotImplementedError("solid translate needs a kernel change")

    def rotate(self, axisStart, axisEnd, angleDegrees):
        """Not implemented (kernel has no solid transform yet)."""
        raise NotImplementedError("solid rotate needs a kernel change")

    def mirror(self, mirrorPlane="XY", basePointVector=(0, 0, 0), union=False):
        """Not implemented (kernel has no solid transform yet)."""
        raise NotImplementedError("solid mirror needs a kernel change")

    def shell(self, thickness):
        """Not implemented (no offset-solid in the kernel yet)."""
        raise NotImplementedError("shell / offset-solid needs a kernel change")

    def offset2D(self, d, kind="arc", forConstruction=False):
        """Not implemented on Workplane; use camber.Sketch.offset instead."""
        raise NotImplementedError("2D offset needs a kernel change")

    def spline(self, listOfXY, tangents=None, periodic=False, includeCurrent=False):
        """Not implemented on Workplane; use camber.Sketch.add_spline instead."""
        raise NotImplementedError("spline sketch needs a kernel change")

    def each(self, callback, useLocalCoordinates=False, combine=True, clean=True):
        """Not implemented."""
        raise NotImplementedError("each() is not implemented")

    def eachpoint(self, callback, useLocalCoordinates=False, combine=True, clean=True):
        """Not implemented."""
        raise NotImplementedError("eachpoint() is not implemented")

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
