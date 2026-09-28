from __future__ import annotations

import math
import json
import os
import sys
from collections import namedtuple
from typing import Callable, Iterable, Literal, Sequence

from .vec import _xy, _xyz, vec2, vec3

Point2Like = vec2 | Sequence[float]
Point3Like = vec3 | Sequence[float] | float
EntityName = str
EntitySelection = EntityName | Sequence[EntityName]

BOOLEAN_UNION = 0  # a + b / Part.union
BOOLEAN_SUBTRACT = 1  # a - b / Part.subtract
BOOLEAN_INTERSECT = 2  # a & b / Part.intersect
BOOLEAN_RESOLVE = 3
BOOLEAN_NO_OP_INTERSECTION_CONTOUR_ONLY = 4

LOFT_STYLE_RULED = 0
LOFT_STYLE_SMOOTH_CATMULL_ROM = 1
LOFT_STYLE_HERMITE = 2

_NACA_TE_TRIM = 0.01
_TYPES = None


def api_group(name: str) -> Callable[[Callable[..., object]], Callable[..., object]]:
    """Attach a reference-page category without changing call behavior."""
    def decorate(method):
        method.__api_group__ = name
        return method
    return decorate


def _env_flag(name, default=True):
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


# One console line before each native call. Opt out with set_progress_log(False)
# or CAMBER_PROGRESS=0.
_progress_log = _env_flag("CAMBER_PROGRESS", True)


def set_progress_log(enabled: bool) -> None:
    """Print ``camber: <call>`` before each native API call. On by default."""
    global _progress_log
    _progress_log = bool(enabled)


def progress_log_enabled() -> bool:
    return _progress_log


def _log_progress(label):
    if not _progress_log:
        return
    sys.stdout.write("camber: {0}\n".format(label))
    sys.stdout.flush()


def _native_mod():
    global _TYPES
    if _TYPES is None:
        from _camber_native.__dotwrap_generated.main import (
            NativeAssembly,
            NativeAssemblyLeaf,
            NativeAssemblyLeaves,
            NativeAssemblyAxisDatum,
            NativeAssemblyPart,
            NativeAssemblyPlaneDatum,
            NativeAssemblyPointDatum,
            NativeBooleanChain,
            NativeCurve,
            NativeCurveList,
            NativeFrame,
            NativeGeom,
            NativeLoftOptions,
            NativePart,
            NativeSketch,
            NativeSketchList,
            NativeSolid,
            NativeSolidList,
        )
        try:
            from _camber_native.__dotwrap_generated.main import NativeAssemblyOccurrence
        except ImportError:
            NativeAssemblyOccurrence = None
        _TYPES = {
            "NativeAssembly": NativeAssembly,
            "NativeAssemblyLeaf": NativeAssemblyLeaf,
            "NativeAssemblyLeaves": NativeAssemblyLeaves,
            "NativeAssemblyAxisDatum": NativeAssemblyAxisDatum,
            "NativeAssemblyOccurrence": NativeAssemblyOccurrence,
            "NativeAssemblyPart": NativeAssemblyPart,
            "NativeAssemblyPlaneDatum": NativeAssemblyPlaneDatum,
            "NativeAssemblyPointDatum": NativeAssemblyPointDatum,
            "NativeBooleanChain": NativeBooleanChain,
            "NativeCurve": NativeCurve,
            "NativeCurveList": NativeCurveList,
            "NativeFrame": NativeFrame,
            "NativeGeom": NativeGeom,
            "NativeLoftOptions": NativeLoftOptions,
            "NativePart": NativePart,
            "NativeSketch": NativeSketch,
            "NativeSketchList": NativeSketchList,
            "NativeSolid": NativeSolid,
            "NativeSolidList": NativeSolidList,
        }
    return _TYPES


def _native():
    t = _native_mod()
    return t["NativePart"], t["NativeSketch"], t["NativeSolid"]


def _name(name):
    if name is None:
        return ""
    return str(name)


def _require(obj, attr):
    fn = getattr(obj, attr, None)
    if fn is None:
        raise AttributeError(
            "this camber wheel has no '{0}'; rebuild with Geo.Python/publish-wheel.ps1".format(attr)
        )

    def wrapped(*args, **kwargs):
        _log_progress(attr)
        return fn(*args, **kwargs)

    return wrapped


def _invoke(obj, attr, *args):
    return _require(obj, attr)(*args)


def _xy_point_ref(point):
    if isinstance(point, str):
        return None
    if point is None or not hasattr(point, "__len__") or len(point) < 2:
        return None
    if point[0] == "xy" and len(point) >= 3:
        return (float(point[1]), float(point[2]))
    return None


def _handle_point_ref(point):
    if isinstance(point, str):
        return None
    if point is None or not hasattr(point, "__len__") or len(point) < 2:
        return None
    if point[0] == "xy":
        return None
    try:
        return (int(point[0]), int(point[1]))
    except (TypeError, ValueError):
        return None


def _named_point_ref(point):
    if isinstance(point, str):
        name = point.strip()
        if not name:
            return None
        from . import naming as _naming
        return _naming.canonicalize_point_name(name)
    return None


def _native_point_name(name):
    from . import naming as _naming
    if _naming.is_origin_name(name):
        return "Origin"
    return name


class SketchCurve(object):
    """Named 2D curve from add_line / add_circle / add_arc. `curve @ 1.000` is \"name@1.000\"."""

    def __init__(self, name, kind=None):
        """``name`` is the kernel curve id; ``kind`` is ``line`` / ``circle`` / ``arc``."""
        self.name = name
        self.kind = kind

    def __matmul__(self, param):
        """Point address: ``curve @ 0`` start, ``curve @ 1`` end, ``curve @ \"center\"``."""
        name = (self.name or "").strip()
        if not name:
            raise ValueError("curve has no name")
        if isinstance(param, str):
            suffix = param.strip()
            if not suffix:
                raise ValueError("empty @ address")
            from . import naming as _naming
            if _naming.is_center_param(suffix):
                suffix = _naming.CENTER
            return name + "@" + suffix
        try:
            value = float(param)
        except (TypeError, ValueError):
            raise TypeError("curve @ expects a number or an address like \"center\"")
        from . import naming as _naming
        return _naming.format_sketch_curve_address(name, value)

    def __str__(self):
        return self.name or ""

    def __repr__(self):
        return "SketchCurve(name={0!r})".format(self.name)


def _curve_index(sketch, curve):
    if isinstance(curve, SketchCurve):
        curve = curve.name
    try:
        return int(curve)
    except (TypeError, ValueError):
        pass
    name = str(curve).strip()
    if not name:
        raise ValueError("curve name is empty")
    from . import naming as _naming
    from . import pick as _pick
    owner, local = _naming.parse_qualified(name)
    key = local or name
    parsed = _naming.parse_sketch_curve_address(key)
    if parsed is not None:
        key = parsed["curve"]
    native_index = getattr(sketch._n, "constraint_curve_index", None)
    if native_index is not None:
        return int(native_index(key))
    dumped = []
    try:
        dumped = sketch._solved_actions()
    except Exception:
        dumped = []
    for index, action in enumerate(dumped):
        action_name = (action.get("name") or "").strip()
        if action_name == key or action_name == name:
            return index
        if _pick.sketch_curve_display_name(action, index) == key:
            return index
    raise ValueError("unknown sketch curve {0!r}".format(curve))


def _endpoint_value(point):
    if isinstance(point, str):
        name = point.strip()
        if name:
            from . import naming as _naming
            return "name", _naming.canonicalize_point_name(name)
    if point is not None and not isinstance(point, str) and hasattr(point, "__len__"):
        if len(point) == 2 and point[0] != "xy":
            try:
                return "xy", (float(point[0]), float(point[1]))
            except (TypeError, ValueError):
                pass
    return _classify_point(point)


def _eval_named_point(sketch, name):
    from . import naming as _naming
    owner, local = _naming.parse_qualified(name)
    key = local or name
    xy = _try_native_eval_point(getattr(sketch, "_n", None), name, key)
    if xy is not None:
        return xy
    if _naming.is_origin_name(key):
        return (0.0, 0.0)
    parsed = _naming.parse_sketch_curve_address(key)
    if parsed is not None:
        xy = _eval_address_from_dump(sketch, parsed)
        if xy is not None:
            return xy
    raise ValueError("cannot evaluate named sketch point {0!r}".format(name))


def _try_native_eval_point(native, name, local):
    if native is None:
        return None
    candidates = []
    from . import naming as _naming
    if _naming.is_origin_name(name) or _naming.is_origin_name(local):
        candidates.append("Origin")
    for candidate in (name, local):
        if candidate and candidate not in candidates:
            candidates.append(candidate)
    fx = getattr(native, "evaluate_constraint_point_x", None)
    fy = getattr(native, "evaluate_constraint_point_y", None)
    if callable(fx) and callable(fy):
        for candidate in candidates:
            try:
                return (float(fx(candidate)), float(fy(candidate)))
            except Exception:
                continue
    fn = getattr(native, "evaluate_constraint_point", None) or getattr(
        native, "EvaluateConstraintPoint", None)
    if not callable(fn):
        return None
    for candidate in candidates:
        try:
            result = fn(candidate)
            if result is not None and hasattr(result, "__len__") and len(result) >= 2:
                return (float(result[0]), float(result[1]))
        except Exception:
            continue
    return None


def _eval_address_from_dump(sketch, parsed):
    dumped = []
    try:
        dumped = sketch._solved_actions()
    except Exception:
        dumped = []
    if not dumped:
        return None
    from . import pick as _pick
    curve = parsed["curve"]
    action = None
    for index, item in enumerate(dumped):
        action_name = (item.get("name") or "").strip()
        if action_name == curve or _pick.sketch_curve_display_name(item, index) == curve:
            action = item
            break
    if action is None:
        return None
    kind = action.get("kind")
    if parsed.get("center"):
        if kind == "circle":
            cx, cy = action["center"]
            return (float(cx), float(cy))
        if kind == "arc":
            return _arc_center_xy(action)
        return None
    uniform = float(parsed.get("uniform") or 0.0)
    if kind == "line":
        p0, p1 = action["p0"], action["p1"]
        return (
            float(p0[0]) + uniform * (float(p1[0]) - float(p0[0])),
            float(p0[1]) + uniform * (float(p1[1]) - float(p0[1])),
        )
    if kind == "circle":
        cx, cy = action["center"]
        radius = float(action["radius"])
        ang = 2.0 * math.pi * uniform
        return (float(cx) + radius * math.cos(ang), float(cy) + radius * math.sin(ang))
    if kind == "arc":
        return _arc_point_xy(action, uniform)
    return None


def _arc_center_xy(action):
    a, b, c = action["start"], action["mid"], action["end"]
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return None
    ux = (
        (a[0] * a[0] + a[1] * a[1]) * (b[1] - c[1])
        + (b[0] * b[0] + b[1] * b[1]) * (c[1] - a[1])
        + (c[0] * c[0] + c[1] * c[1]) * (b[1] - a[1])
    ) / d
    uy = (
        (a[0] * a[0] + a[1] * a[1]) * (c[0] - b[0])
        + (b[0] * b[0] + b[1] * b[1]) * (a[0] - c[0])
        + (c[0] * c[0] + c[1] * c[1]) * (b[0] - a[0])
    ) / d
    return (float(ux), float(uy))


def _arc_point_xy(action, uniform):
    center = _arc_center_xy(action)
    if center is None:
        start, end = action["start"], action["end"]
        return (
            float(start[0]) + uniform * (float(end[0]) - float(start[0])),
            float(start[1]) + uniform * (float(end[1]) - float(start[1])),
        )
    start = action["start"]
    mid = action["mid"]
    end = action["end"]
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(mid[1] - center[1], mid[0] - center[0])
    a2 = math.atan2(end[1] - center[1], end[0] - center[0])

    def wrap(delta):
        while delta <= -math.pi:
            delta += 2.0 * math.pi
        while delta > math.pi:
            delta -= 2.0 * math.pi
        return delta

    mid_delta = wrap(a1 - a0)
    end_delta = wrap(a2 - a0)
    if mid_delta * end_delta < 0:
        if end_delta > 0:
            end_delta -= 2.0 * math.pi
        else:
            end_delta += 2.0 * math.pi
    ang = a0 + end_delta * uniform
    radius = math.hypot(start[0] - center[0], start[1] - center[1])
    return (center[0] + radius * math.cos(ang), center[1] + radius * math.sin(ang))


def _classify_point(point):
    named = _named_point_ref(point)
    if named is not None:
        return "name", named
    xy = _xy_point_ref(point)
    if xy is not None:
        return "xy", xy
    if _handle_point_ref(point) is not None:
        raise ValueError(
            "use a named address such as \"Line1@1.000\" or sketch @ \"origin\", "
            "not a (curve, role) handle {0!r}".format(point)
        )
    raise ValueError(
        "point must be a name (\"Line1@1.000\", sketch @ \"origin\") or an XY constant (\"xy\", x, y)"
    )


def _as_names(name):
    if isinstance(name, str):
        return (name,)
    return tuple(name)


def _call_native(native, names, *args):
    last = None
    for name in names:
        fn = getattr(native, name, None)
        if fn is None:
            continue
        try:
            return _invoke(native, name, *args)
        except TypeError as ex:
            last = ex
    if last is not None:
        raise last
    return _invoke(native, names[0], *args)


def _dispatch_two_points(
        native, point_a, point_b,
        name_name, name_xy, two_constants, extra=()):
    kind_a, a = _classify_point(point_a)
    kind_b, b = _classify_point(point_b)
    extra = extra or ()
    if kind_a == "xy" and kind_b == "xy":
        raise ValueError(two_constants)
    if kind_a == "name" and kind_b == "name":
        _call_native(native, _as_names(name_name), _native_point_name(a), _native_point_name(b), *extra)
        return
    if kind_a == "name" and kind_b == "xy":
        _require(native, name_xy)(_native_point_name(a), b[0], b[1], *extra)
        return
    if kind_a == "xy" and kind_b == "name":
        _require(native, name_xy)(_native_point_name(b), a[0], a[1], *extra)
        return
    raise ValueError("unsupported point pair")


def _join_names(edges):
    if isinstance(edges, str):
        return edges
    return "|".join(edges)


def _as_frame(pose):
    if isinstance(pose, Frame):
        return pose
    return Frame(pose)


def _sketch_list(sketches):
    lst = _native_mod()["NativeSketchList"]()
    for sk in sketches:
        lst.add(sk._n)
    return lst


def _curve_list(curves):
    lst = _native_mod()["NativeCurveList"]()
    for c in curves:
        lst.add(c._n)
    return lst


def _solid_list(solids):
    lst = _native_mod()["NativeSolidList"]()
    for s in solids:
        lst.add(s._n)
    return lst


class Frame(object):
    """World pose: origin + right-handed orthonormal axes (GeoAPI CoordinateSystem)."""

    def __init__(self, origin: Point3Like | None = None, x: Point3Like | None = None,
                 y: Point3Like | None = None, z: Point3Like | None = None) -> None:
        """World origin plus orthonormal axes. Defaults to identity at (0,0,0)."""
        self.origin = vec3(0, 0, 0) if origin is None else vec3(origin)
        self.x = vec3(1, 0, 0) if x is None else vec3(x)
        self.y = vec3(0, 1, 0) if y is None else vec3(y)
        self.z = vec3(0, 0, 1) if z is None else vec3(z)

    @staticmethod
    def from_plane(origin: Point3Like, normal: Point3Like,
                   x: Point3Like, y: Point3Like) -> Frame:
        """Same pose as Curves.Plane3D(origin, normal, x, y).GetCoordinateSystem()."""
        return Frame(origin, x=x, y=y, z=normal)

    def offset(self, delta: Point3Like) -> Frame:
        """Translate origin by ``delta`` (vec3 or 3-tuple). Axes unchanged."""
        return Frame(self.origin + delta, self.x, self.y, self.z)

    def to_local(self, other: Frame) -> Frame:
        """Express ``other`` in this frame. Returns a Frame."""
        return Frame._from_native(self._native().to_local(_as_frame(other)._native()))

    def to_global(self, local: Frame) -> Frame:
        """Map a frame given in this local space into world. Returns a Frame."""
        return Frame._from_native(self._native().to_global(_as_frame(local)._native()))

    def _native(self):
        NativeFrame = _native_mod()["NativeFrame"]
        o, x, y, z = self.origin, self.x, self.y, self.z
        return NativeFrame(o.x, o.y, o.z, x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z)

    @staticmethod
    def _from_native(n):
        return Frame((n.ox, n.oy, n.oz), (n.xx, n.xy, n.xz), (n.yx, n.yy, n.yz), (n.zx, n.zy, n.zz))

    def __repr__(self):
        return "Frame(origin={0}, x={1}, y={2}, z={3})".format(self.origin, self.x, self.y, self.z)


def frame_from_axis(origin: Point3Like, axis: Literal["x", "y", "z"] = "z") -> Frame:
    """Pose whose local +Z is world +axis (CreateCylinder extrudes along local +Z)."""
    origin = vec3(origin)
    axis = (axis or "z").lower()
    if axis == "x":
        return Frame(origin, x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
    if axis == "y":
        return Frame(origin, x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0))
    return Frame(origin)


class Curve3D(object):
    """Generic handle for any 3D curve (analytic, spline, or sampled)."""

    def __init__(self, native: object, part: Part | None = None) -> None:
        self._n = native
        self._part = part

    @property
    def name(self) -> EntityName:
        """Kernel entity name."""
        return self._n.name

    @staticmethod
    def line(start: Point3Like, end: Point3Like, name: EntityName | None = None, *,
             up: Point3Like | None = None) -> Curve3D:
        """World-space line; optional ``up`` controls its sweep frame orientation."""
        x0, y0, z0 = _xyz(start)
        x1, y1, z1 = _xyz(end)
        if up is not None:
            ux, uy, uz = _xyz(up)
            return Curve3D(_require(_native_mod()["NativeCurve"], "linear")(
                x0, y0, z0, x1, y1, z1, ux, uy, uz, _name(name)))
        return Curve3D(_require(_native_mod()["NativeCurve"], "line")(x0, y0, z0, x1, y1, z1, _name(name)))

    @staticmethod
    def hermite(points: Sequence[Point3Like], tangent_directions: Sequence[Point3Like],
                name: EntityName | None = None) -> Curve3D:
        """Cubic guide through points, with one nonzero tangent direction per knot.

        Direction magnitudes are ignored; adjacent chord lengths set derivative
        magnitudes. Repeat the first point and direction to close the curve.
        The closed seam is C1, including unequal first/last chord lengths.
        Cubic interpolation does not guarantee C2 acceleration continuity.
        """
        from .geom import _pack_points3
        return Curve3D(_require(_native_mod()["NativeCurve"], "hermite")(
            _pack_points3(points), _pack_points3(tangent_directions), _name(name)))

    @staticmethod
    def sampled(points: Sequence[Point3Like], name: EntityName | None = None) -> Curve3D:
        """Create a generic piecewise-linear 3D curve through sampled points."""
        from .geom import _pack_points3
        return Curve3D(_require(_native_mod()["NativeCurve"], "sampled")(
            _pack_points3(points), _name(name)))

    def point(self, u: float) -> vec3:
        """Point at normalized parameter u in [0,1], not normalized arc length."""
        return vec3(tuple(map(float, _require(self._n, "point")(float(u)).split())))

    def tangent(self, u: float) -> vec3:
        """Unit tangent at normalized parameter u in [0,1]."""
        return vec3(tuple(map(float, _require(self._n, "tangent")(float(u)).split())))

    def tessellate(self, max_deviation: float | None = None) -> list[vec3]:
        """Sample this 3D curve within a deviation tolerance in world units.

        Curves returned by ``Surface.intersection_curves`` inherit their owning
        Part's default tolerance when this argument is omitted. Standalone
        curves default to 0.01 world units.
        """
        deviation = (self._part.max_deviation if self._part is not None else 0.01) \
            if max_deviation is None else float(max_deviation)
        if not math.isfinite(deviation) or deviation <= 0:
            raise ValueError("max_deviation must be finite and positive")
        from .geom import _unpack_points3
        return _unpack_points3(_require(self._n, "tessellate")(deviation))

    @staticmethod
    def helix(origin: Point3Like, x_axis: Point3Like, y_axis: Point3Like,
              radius: float, pitch: float, turns: float, right_handed: bool = True,
              name: EntityName | None = None) -> Curve3D:
        """Helix in the plane of ``x_axis``/``y_axis``; axis is their cross. ``pitch`` is z per turn."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve3D(_require(_native_mod()["NativeCurve"], "helix")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(radius), float(pitch), float(turns), 1 if right_handed else 0, _name(name)))

    @staticmethod
    def circle(origin: Point3Like, x_axis: Point3Like, y_axis: Point3Like,
               radius: float, name: EntityName | None = None) -> Curve3D:
        """Full circle in the x/y plane of the given axes."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve3D(_require(_native_mod()["NativeCurve"], "circle")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz, float(radius), _name(name)))

    @staticmethod
    def arc(origin: Point3Like, x_axis: Point3Like, y_axis: Point3Like, radius: float,
            start_angle: float, sweep_angle: float, name: EntityName | None = None) -> Curve3D:
        """Circular arc. Angles in radians, from ``x_axis`` toward ``y_axis``."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve3D(_require(_native_mod()["NativeCurve"], "arc")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(radius), float(start_angle), float(sweep_angle), _name(name)))

    @staticmethod
    def spiral(origin: Point3Like, x_axis: Point3Like, y_axis: Point3Like,
               start_radius: float, end_radius: float, z_per_turn: float, turns: float,
               right_handed: bool = True, name: EntityName | None = None) -> Curve3D:
        """Planar spiral with optional z advance (``z_per_turn``). Radii in the x/y plane."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve3D(_require(_native_mod()["NativeCurve"], "spiral")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(start_radius), float(end_radius), float(z_per_turn), float(turns),
            1 if right_handed else 0, _name(name)))

    @staticmethod
    def torus_knot(p: float, q: float, radius: float,
                   name: EntityName | None = None) -> Curve3D:
        """Create a torus-knot curve with the kernel's parameterization."""
        return Curve3D(_require(_native_mod()["NativeCurve"], "torus_knot")(
            float(p), float(q), float(radius), _name(name)))

    def __repr__(self):
        return "Curve3D(name={0!r})".format(self.name)


class LoftOptions(object):
    """General settings for profile correspondence, loft shape and crease handling."""

    def __init__(self, *,
                 correspondence: Literal["arc_length", "uniform", "features", "vertices"] | None = None,
                 style: Literal["ruled", "smooth_catmull_rom", "hermite"] | None = None,
                 crease_policy: Literal["all_profiles", "first_profile", "none"] | None = None) -> None:
        """Create general loft settings; omitted values use the kernel defaults."""
        NativeLoftOptions = _native_mod()["NativeLoftOptions"]
        self._n = NativeLoftOptions()
        if correspondence is not None:
            self.correspondence = correspondence
        if style is not None:
            self.style = style
        if crease_policy is not None:
            self.crease_policy = crease_policy

    @property
    def correspondence(self) -> Literal["arc_length", "uniform", "features", "vertices"]:
        """Section matching: 'arc_length', 'uniform', 'features', or 'vertices'.

        'vertices' matches ordered polygon corners; all sections must have the
        same vertex count. Sharp polygon corners retain separate face normals.
        """
        return ("arc_length", "uniform", "features", "vertices")[self._n.correspondence_mode]

    @correspondence.setter
    def correspondence(self, value: Literal["arc_length", "uniform", "features", "vertices"]) -> None:
        choices = ("arc_length", "uniform", "features", "vertices")
        if value not in choices:
            raise ValueError("correspondence must be one of " + ", ".join(choices))
        self._n.correspondence_mode = choices.index(value)

    @property
    def style(self) -> Literal["ruled", "smooth_catmull_rom", "hermite"]:
        """Loft interpolation between sections."""
        return ("ruled", "smooth_catmull_rom", "hermite")[self._n.style]

    @style.setter
    def style(self, value: Literal["ruled", "smooth_catmull_rom", "hermite"]) -> None:
        choices = ("ruled", "smooth_catmull_rom", "hermite")
        if value not in choices:
            raise ValueError("style must be one of " + ", ".join(choices))
        self._n.style = choices.index(value)

    @property
    def crease_policy(self) -> Literal["all_profiles", "first_profile", "none"]:
        """Which profile joints create crease columns in the loft mesh."""
        return ("all_profiles", "first_profile", "none")[self._n.crease_policy]

    @crease_policy.setter
    def crease_policy(self, value: Literal["all_profiles", "first_profile", "none"]) -> None:
        choices = ("all_profiles", "first_profile", "none")
        if value not in choices:
            raise ValueError("crease_policy must be one of " + ", ".join(choices))
        self._n.crease_policy = choices.index(value)

    def __repr__(self):
        return "LoftOptions(...)"


class AssemblyPointDatum(object):
    """Named point on an assembly part (for constraints)."""
    def __init__(self, native, part=None, reference=None):
        self._n = native
        self._part = part
        self.reference = reference
        self.entity = _datum_entity(native, part, reference)


class AssemblyAxisDatum(object):
    """Named axis on an assembly part (for constraints)."""
    def __init__(self, native, part=None, reference=None):
        self._n = native
        self._part = part
        self.reference = reference
        self.entity = _datum_entity(native, part, reference)


class AssemblyPlaneDatum(object):
    """Named plane on an assembly part (for constraints)."""
    def __init__(self, native, part=None, reference=None):
        self._n = native
        self._part = part
        self.reference = reference
        self.entity = _datum_entity(native, part, reference)


def _datum_entity(native, part, reference):
    name = ""
    if native is not None:
        try:
            name = getattr(native, "entity", None) or ""
        except Exception:
            name = ""
    if name:
        return str(name)
    if part is not None and reference:
        return "{0}:{1}".format(part.name, reference)
    return ""


class AssemblyPart(object):
    """An instance of a Solid in an Assembly (pose + named datums)."""
    def __init__(self, native: object) -> None:
        self._n = native

    @property
    def name(self) -> EntityName:
        """Instance name."""
        return self._n.name

    @property
    def pose(self) -> tuple[float, float, float]:
        """Translation in the owning assembly as ``(x, y, z)``."""
        return (self._n.pose_x, self._n.pose_y, self._n.pose_z)

    def axis(self, reference: EntityName) -> AssemblyAxisDatum:
        """Axis datum from a named entity on this part (edge/axis string)."""
        return AssemblyAxisDatum(_invoke(self._n, "add_axis_datum", str(reference)), self, str(reference))

    def point(self, reference: EntityName) -> AssemblyPointDatum:
        """Point datum from a named entity on this part."""
        return AssemblyPointDatum(_invoke(self._n, "add_point_datum", str(reference)), self, str(reference))

    def plane(self, reference: EntityName) -> AssemblyPlaneDatum:
        """Plane datum from a named entity on this part."""
        return AssemblyPlaneDatum(_invoke(self._n, "add_plane_datum", str(reference)), self, str(reference))

    def axis_at(self, point: Point3Like, direction: Point3Like) -> AssemblyAxisDatum:
        """Axis through ``point`` along ``direction`` in part-local coordinates."""
        px, py, pz = _xyz(point)
        dx, dy, dz = _xyz(direction)
        return AssemblyAxisDatum(_invoke(self._n, "add_axis_datum_at", px, py, pz, dx, dy, dz), self)

    def point_at(self, point: Point3Like) -> AssemblyPointDatum:
        """Point datum in part-local coordinates."""
        px, py, pz = _xyz(point)
        return AssemblyPointDatum(_invoke(self._n, "add_point_datum_at", px, py, pz), self)

    def plane_at(self, origin: Point3Like, normal: Point3Like) -> AssemblyPlaneDatum:
        """Plane datum at ``origin`` with ``normal``, both in part-local coordinates."""
        px, py, pz = _xyz(origin)
        nx, ny, nz = _xyz(normal)
        return AssemblyPlaneDatum(_invoke(self._n, "add_plane_datum_at", px, py, pz, nx, ny, nz), self)


class AssemblyOccurrence(object):
    """Placement of an Assembly definition inside a parent Assembly.

    ``parts`` and ``subassemblies`` return references scoped to this placement
    for parent mates. Flexible placements have independent internal mate state.
    """

    def __init__(self, native: object, parent: Assembly | None = None) -> None:
        self._n = native
        self._parent = parent

    @property
    def name(self) -> str:
        """Nested assembly name."""
        return self._n.name

    @property
    def flexible(self) -> bool:
        """Whether this placement has independent internal mates and geometry."""
        return bool(self._n.flexible)

    @property
    def pose(self) -> tuple[float, float, float]:
        """Translation of this occurrence in the parent assembly as ``(x, y, z)``."""
        return (self._n.pose_x, self._n.pose_y, self._n.pose_z)

    @property
    def assembly(self) -> Assembly:
        """The nested Assembly (its parts, mates, and further sub-assemblies)."""
        part = self._parent._part if self._parent is not None else None
        return Assembly(_invoke(self._n, "get_child"), part)

    @property
    def parts(self) -> list[AssemblyPart]:
        """Direct parts in this occurrence, suitable for parent-level mates."""
        count = int(self._n.part_count)
        return [AssemblyPart(_invoke(self._n, "get_part", i)) for i in range(count)]

    @property
    def subassemblies(self) -> list[AssemblyOccurrence]:
        """Nested occurrences in this placement, scoped to the parent assembly."""
        count = int(self._n.sub_assembly_count)
        return [
            AssemblyOccurrence(_invoke(self._n, "get_sub_assembly", i), self._parent)
            for i in range(count)
        ]

    def __repr__(self):
        return "AssemblyOccurrence({0!r})".format(self.name)


class PartOperation(namedtuple("PartOperation", "index kind result inputs entities details")):
    """Immutable ``Part.operations`` record: index, kind, result name, input names, selected entity names, and parameter details."""
    __slots__ = ()

class AssemblyLeaf(namedtuple("AssemblyLeaf", "path part solid frame bounds")):
    """A recursive assembly leaf in its current world pose.

    ``solid`` is a placed geometry snapshot and may also be an open Surface.
    ``frame`` is its world pose. ``path`` identifies this occurrence.
    Prefix a local patch, curve, or point name
    with ``path + ':'`` when referring to the occurrence in an assembly.
    """
    __slots__ = ()

class Interference(namedtuple("Interference", "first second volume geometry")):
    """Overlapping occurrence paths, volume in cubic model units, and a Solid to display."""
    __slots__ = ()


class MateResidual(namedtuple("MateResidual", "index kind label entities residuals max_residual tolerance satisfied")):
    """One mate's normalized equation errors, captured at the reported pose.

    Residuals are dimensionless solver errors, not distances in model units.
    An unsatisfied mate locates error; it does not identify a minimal conflict set.
    """
    __slots__ = ()

    def __repr__(self):
        return "MateResidual(index={0}, label={1!r}, max_residual={2:.3g}, satisfied={3})".format(
            self.index, self.label, self.max_residual, self.satisfied)


class AssemblyConstraintDatum(namedtuple("AssemblyConstraintDatum", "kind part path local_point local_direction world_point world_direction")):
    """Immutable constraint anchor in body-local and solved assembly coordinates."""
    __slots__ = ()


class AssemblyConstraint(namedtuple("AssemblyConstraint", "index kind label first second value entities")):
    """One authored assembly constraint, suitable for inspection or physics export."""
    __slots__ = ()


class AssemblySolveResult(namedtuple("AssemblySolveResult", "converged sum_squared_error num_parameters num_equations message characteristic_length mates")):
    """Immutable solver outcome and per-mate errors at the resulting pose.

    Parameter/equation counts describe the solver system, not remaining degrees
    of freedom. Contact mates have their own regularized residual tolerance.
    ``converged`` is the solver outcome; ``unsatisfied`` separately checks every
    equality mate at its stricter tolerance, even in a regularized contact solve.
    """
    __slots__ = ()

    @property
    def unsatisfied(self) -> tuple[MateResidual, ...]:
        """Unsatisfied mates, largest normalized error first."""
        return tuple(sorted((mate for mate in self.mates if not mate.satisfied),
                            key=lambda mate: (-mate.max_residual, mate.index)))

    @classmethod
    def _from_json(cls, value):
        report = json.loads(value)
        mates = tuple(MateResidual(item['index'], item['kind'], item['label'],
                      tuple(item['entities']), tuple(float(x) for x in item['residuals']),
                      float(item['max_residual']), float(item['tolerance']), item['satisfied'])
                      for item in report['mates'])
        return cls(report['converged'], float(report['sum_squared_error']),
                   report['num_parameters'], report['num_equations'], report['message'],
                   float(report['characteristic_length']), mates)

    def __repr__(self):
        failures = self.unsatisfied
        summary = "AssemblySolveResult(converged={0}, parameters={1}, equations={2}, unsatisfied={3}".format(
            self.converged, self.num_parameters, self.num_equations, len(failures))
        if failures:
            summary += ", worst={0!r} ({1:.3g})".format(failures[0].label, failures[0].max_residual)
        if self.message:
            summary += ", message={0!r}".format(self.message)
        return summary + ")"


class Assembly(object):
    """Part assembly solved by C# Geo assembly constraints.

    Parts and nested assemblies can be mixed: ``add_part`` places a Solid,
    ``add_subassembly`` places another Assembly (which may itself contain
    parts and sub-assemblies). Nested assemblies are rigid by default; use a
    flexible occurrence when its internal joints must move independently.
    """

    def __init__(self, native: object, part: Part | None = None) -> None:
        self._n = native
        self._part = part

    @property
    def constraints(self) -> tuple[AssemblyConstraint, ...]:
        """Immutable authored constraints with local and solved world anchors."""
        records = json.loads(_require(self._n, "constraint_records")())
        def datum(value):
            if value is None:
                return None
            return AssemblyConstraintDatum(value["kind"], value["part"], value["path"],
                vec3(*value["local_point"]), vec3(*value["local_direction"]),
                vec3(*value["world_point"]), vec3(*value["world_direction"]))
        return tuple(AssemblyConstraint(item["index"], item["kind"], item["label"],
            datum(item["first"]), datum(item["second"]), item["value"],
            tuple(item["entities"])) for item in records)

    @property
    def name(self) -> EntityName:
        """Assembly instance name."""
        return self._n.name

    @property
    def solve_after_every_constraint(self) -> bool:
        """If True, the kernel solves after each constraint."""
        return bool(self._n.solve_after_every_constraint)

    @solve_after_every_constraint.setter
    def solve_after_every_constraint(self, value: bool) -> None:
        self._n.solve_after_every_constraint = 1 if value else 0

    def leaves(self) -> tuple[AssemblyLeaf, ...]:
        """Return recursive part occurrences with world frames and exact surface bounds.

        The leaf's ``solid`` is already in assembly coordinates; ``frame``
        reports its pose. ``bounds`` is ``(minimum, maximum)`` in the same space.
        """
        native = _require(self._n, "leaves")()
        result = []
        for i in range(int(native.count)):
            leaf = _invoke(native, "get", i)
            frame = Frame((leaf.ox, leaf.oy, leaf.oz),
                          (leaf.xx, leaf.xy, leaf.xz),
                          (leaf.yx, leaf.yy, leaf.yz),
                          (leaf.zx, leaf.zy, leaf.zz))
            bounds = (vec3(leaf.min_x, leaf.min_y, leaf.min_z),
                      vec3(leaf.max_x, leaf.max_y, leaf.max_z))
            result.append(AssemblyLeaf(leaf.path,
                          AssemblyPart(_invoke(leaf, "get_part")),
                          _body(_invoke(leaf, "get_solid"), self._part), frame, bounds))
        return tuple(result)

    def bounds(self) -> tuple[vec3, vec3] | None:
        """World-space ``(minimum, maximum)`` over recursive part occurrences, or None when empty."""
        leaves = self.leaves()
        if not leaves:
            return None
        minimum = vec3(min(leaf.bounds[0].x for leaf in leaves),
                       min(leaf.bounds[0].y for leaf in leaves),
                       min(leaf.bounds[0].z for leaf in leaves))
        maximum = vec3(max(leaf.bounds[1].x for leaf in leaves),
                       max(leaf.bounds[1].y for leaf in leaves),
                       max(leaf.bounds[1].z for leaf in leaves))
        return minimum, maximum

    def save_obj(self, path: str) -> None:
        """Write placed leaves as named OBJ objects with hard-edge normals and UVs."""
        _invoke(self._n, "save_wavefront_obj", path)

    def interferences(self, *, min_volume: float = 0) -> list[Interference]:
        """Return positive overlaps, largest first, including nested parts.

        Uses current occurrence poses without solving or modifying the model.
        Each result has ``first`` and ``second`` component paths, ``volume``
        in cubic model units, and ``geometry`` for show()/render_views().
        Only volumes strictly greater than ``min_volume`` are returned.
        Touching surfaces are excluded; failed geometry checks raise an error
        identifying the pair, rather than being reported as collision-free.
        This checks the tessellated solids, not analytic minimum clearance.
        """
        min_volume = float(min_volume)
        if not math.isfinite(min_volume) or min_volume < 0:
            raise ValueError("min_volume must be finite and nonnegative")
        result = _invoke(self._n, "interferences", min_volume)
        return [Interference(_invoke(result, "first", i), _invoke(result, "second", i),
                             float(_invoke(result, "volume", i)),
                             Solid(_invoke(result, "geometry", i), self._part))
                for i in range(int(result.count))]

    def add_part(self, solid: Solid, position: Point3Like = (0, 0, 0),
                 orientation: Sequence[float] = (0, 0, 0, 1)) -> AssemblyPart:
        """Place a solid from any Part at ``position`` with quaternion ``orientation`` (x,y,z,w). Returns AssemblyPart."""
        px, py, pz = _xyz(position)
        if len(orientation) != 4:
            raise ValueError("orientation must be quaternion (x, y, z, w)")
        qx, qy, qz, qw = orientation
        return AssemblyPart(_invoke(
            self._n, "add_part",
            solid._n, px, py, pz, float(qx), float(qy), float(qz), float(qw)))

    def pattern_linear(self, seed: AssemblyPart | AssemblyOccurrence, count: int,
                       step: Point3Like) -> list[AssemblyPart | AssemblyOccurrence]:
        """Return ``count`` instances including seed, spaced by assembly-frame step.

        Copies share the solid definition and are mated rigidly to the seed,
        so moving the seed moves the pattern. The seed may be a direct part
        or subassembly; nested hierarchy and internal mates are preserved.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("count must be a positive integer")
        x, y, z = _xyz(step)
        nested = isinstance(seed, AssemblyOccurrence)
        if not nested and not isinstance(seed, AssemblyPart):
            raise TypeError("seed must be an assembly part or subassembly occurrence")
        method = "pattern_linear_subassembly" if nested else "pattern_linear"
        result = _invoke(self._n, method, seed._n, count, x, y, z)
        return [seed] + [(AssemblyOccurrence(_invoke(result, "get", i), self) if nested
                         else AssemblyPart(_invoke(result, "get", i)))
                        for i in range(1, int(result.count))]

    def pattern_circular(self, seed: AssemblyPart | AssemblyOccurrence, count: int,
                         axis: Frame | None = None,
                         angle: float = 2*math.pi) -> list[AssemblyPart | AssemblyOccurrence]:
        """Pattern around axis.z; a full circle omits the duplicate endpoint.

        A partial sweep includes both endpoints. Count includes the seed.
        Axis is a Frame in assembly coordinates; defaults to the world Z axis.
        Copies rotate with the pattern and remain rigidly mated to the seed.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("count must be a positive integer")
        axis = Frame() if axis is None else _as_frame(axis)
        nested = isinstance(seed, AssemblyOccurrence)
        if not nested and not isinstance(seed, AssemblyPart):
            raise TypeError("seed must be an assembly part or subassembly occurrence")
        method = "pattern_circular_subassembly" if nested else "pattern_circular"
        result = _invoke(self._n, method, seed._n, count, axis._native(), float(angle))
        return [seed] + [(AssemblyOccurrence(_invoke(result, "get", i), self) if nested
                         else AssemblyPart(_invoke(result, "get", i)))
                        for i in range(1, int(result.count))]

    def mirror(self, seed: AssemblyPart | AssemblyOccurrence, plane: Frame | None = None,
               name: EntityName | None = None) -> AssemblyPart | AssemblyOccurrence:
        """Mirror a direct part or subassembly in the XY plane of a Frame.

        Creates opposite-handed geometry and retains nested parts and mates.
        After creation, recorded rigid mates make the result follow its seed;
        the mirror plane defines the initial placement, not a moving symmetry mate.
        """
        if not isinstance(seed, (AssemblyPart, AssemblyOccurrence)):
            raise TypeError("seed must be an AssemblyPart or AssemblyOccurrence")
        plane = Frame() if plane is None else _as_frame(plane)
        if isinstance(seed, AssemblyOccurrence):
            return AssemblyOccurrence(_invoke(self._n, "mirror_sub_assembly",
                seed._n, plane._native(), _name(name)), self)
        return AssemblyPart(_invoke(self._n, "mirror_part",
            seed._n, plane._native(), _name(name)))

    def add_subassembly(self, assembly: Assembly, position: Point3Like = (0, 0, 0),
                        orientation: Sequence[float] = (0, 0, 0, 1), *,
                        flexible: bool = False) -> AssemblyOccurrence:
        """Place an assembly definition as a rigid or flexible child.

        Returns an occurrence. Use ``occurrence.parts`` (or its nested
        ``subassemblies``) when making parent-level mates so repeated placements
        remain unambiguous. By default occurrences share the child definition's
        internal pose. ``flexible=True`` makes an independent copy of its mate
        state and geometry, allowing each occurrence's joints to move separately.
        """
        px, py, pz = _xyz(position)
        if len(orientation) != 4:
            raise ValueError("orientation must be quaternion (x, y, z, w)")
        qx, qy, qz, qw = orientation
        return AssemblyOccurrence(_invoke(
            self._n, "add_sub_assembly",
            assembly._n, px, py, pz, float(qx), float(qy), float(qz), float(qw),
            1 if flexible else 0), self)

    @property
    def parts(self) -> list[AssemblyPart]:
        """Parts added with ``add_part`` on this assembly (not nested leaves)."""
        count = int(self._n.part_count)
        return [AssemblyPart(_invoke(self._n, "get_part", i)) for i in range(count)]

    @property
    def subassemblies(self) -> list[AssemblyOccurrence]:
        """Child assemblies added with ``add_subassembly``."""
        count = int(self._n.sub_assembly_count)
        return [
            AssemblyOccurrence(_invoke(self._n, "get_sub_assembly", i), self)
            for i in range(count)
        ]

    def world_pose(self, part: AssemblyPart | AssemblyOccurrence) -> tuple[float, float, float]:
        """Translation of a direct or nested part in this assembly's frame."""
        return (
            _invoke(self._n, "get_world_pose_x", part._n),
            _invoke(self._n, "get_world_pose_y", part._n),
            _invoke(self._n, "get_world_pose_z", part._n),
        )

    def fix(self, part: AssemblyPart | AssemblyOccurrence) -> None:
        """Lock an AssemblyPart or AssemblyOccurrence in world (ground)."""
        if isinstance(part, AssemblyOccurrence):
            _invoke(self._n, "fix_sub_assembly", part._n)
            return
        _invoke(self._n, "fix_part", part._n)

    def coincident(self, a: AssemblyPointDatum | AssemblyAxisDatum | AssemblyPlaneDatum,
                   b: AssemblyPointDatum | AssemblyAxisDatum | AssemblyPlaneDatum,
                   opposite_normals: bool | None = None) -> None:
        """Coincide two matching datums (point/point, axis/axis, plane/plane)."""
        if isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            if opposite_normals is None:
                _invoke(self._n, "set_coincident_planes", a._n, b._n)
            else:
                _invoke(self._n, "set_coincident_planes_oriented",
                    a._n, b._n, 1 if opposite_normals else 0)
        elif isinstance(a, AssemblyAxisDatum) and isinstance(b, AssemblyAxisDatum):
            if opposite_normals is not None:
                raise TypeError("opposite_normals applies only to plane datums")
            _invoke(self._n, "set_coincident_axes", a._n, b._n)
        elif isinstance(a, AssemblyPointDatum) and isinstance(b, AssemblyPointDatum):
            if opposite_normals is not None:
                raise TypeError("opposite_normals applies only to plane datums")
            _invoke(self._n, "set_coincident_points", a._n, b._n)
        else:
            raise TypeError("coincident datums must have matching point, axis, or plane types")

    def parallel(self, a: AssemblyAxisDatum | AssemblyPlaneDatum,
                 b: AssemblyAxisDatum | AssemblyPlaneDatum) -> None:
        """Keep two matching axis or plane datums parallel."""
        if isinstance(a, AssemblyAxisDatum) and isinstance(b, AssemblyAxisDatum):
            _invoke(self._n, "set_parallel_axes", a._n, b._n)
        elif isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            _invoke(self._n, "set_parallel_planes", a._n, b._n)
        else:
            raise TypeError("parallel expects two axis datums or two plane datums")

    def concentric(self, a: AssemblyAxisDatum, b: AssemblyAxisDatum) -> None:
        """Concentric axes/cylinders (two datums)."""
        _invoke(self._n, "set_concentric", a._n, b._n)

    def perpendicular(self, a: AssemblyAxisDatum | AssemblyPlaneDatum,
                      b: AssemblyAxisDatum | AssemblyPlaneDatum) -> None:
        """Keep two matching axis or plane datums perpendicular."""
        if isinstance(a, AssemblyAxisDatum) and isinstance(b, AssemblyAxisDatum):
            _invoke(self._n, "set_perpendicular_axes", a._n, b._n)
        elif isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            _invoke(self._n, "set_perpendicular_planes", a._n, b._n)
        else:
            raise TypeError("perpendicular expects two axis datums or two plane datums")

    def angle(self, a: AssemblyAxisDatum, b: AssemblyAxisDatum, radians: float) -> None:
        """Angle between two axis datums, in radians."""
        if not isinstance(a, AssemblyAxisDatum) or not isinstance(b, AssemblyAxisDatum):
            raise TypeError("angle currently expects two axis datums")
        _invoke(self._n, "set_angle_axes", a._n, b._n, float(radians))

    def distance(self, a: AssemblyPointDatum | AssemblyPlaneDatum,
                 b: AssemblyPointDatum | AssemblyPlaneDatum, value: float) -> None:
        """Distance between two point datums, or a signed plane offset along B's normal."""
        if isinstance(a, AssemblyPointDatum) and isinstance(b, AssemblyPointDatum):
            _invoke(self._n, "set_distance_points", a._n, b._n, float(value))
        elif isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            _invoke(self._n, "set_distance_planes", a._n, b._n, float(value))
        else:
            raise TypeError("distance expects two point datums or two plane datums")

    def on_plane(self, point: AssemblyPointDatum, plane: AssemblyPlaneDatum) -> None:
        """Constrain a point datum to lie on a plane datum."""
        if not isinstance(point, AssemblyPointDatum) or not isinstance(plane, AssemblyPlaneDatum):
            raise TypeError("on_plane expects a point datum and a plane datum")
        _invoke(self._n, "set_point_on_plane", point._n, plane._n)

    def contact(self, point: AssemblyPointDatum, plane: AssemblyPlaneDatum) -> None:
        """Keep a point on the positive side of a plane datum."""
        if not isinstance(point, AssemblyPointDatum) or not isinstance(plane, AssemblyPlaneDatum):
            raise TypeError("contact expects a point datum and a plane datum")
        _invoke(self._n, "set_contact", point._n, plane._n)

    def solve(self) -> AssemblySolveResult:
        """Solve assembly mates and return an immutable AssemblySolveResult.

        Inspect ``result.converged`` and ``result.unsatisfied`` for diagnostics.
        """
        return AssemblySolveResult._from_json(_invoke(self._n, "solve_constraints"))

    def plane_frame(self, reference: EntityName) -> Frame | None:
        """World Frame of a named assembly plane, or None."""
        native = _require(self._n, "get_plane_frame")(str(reference))
        if native is None:
            return None
        return Frame._from_native(native)

    def show(self, title: str = "Camber") -> None:
        """Open the 3D viewer on this assembly."""
        from .view import show as _show
        _show(self, title=title)


class Part(object):
    """CAD session: operating box, sketches, solids, booleans, import/export.

    ``low`` / ``high`` are the world AABB (vec3, 3-tuple, or a scalar on all axes).
    ``tolerance`` is default tessellation when methods pass ``max_deviation=-1``.
    Solids support ``a + b`` (union), ``a - b`` (cut), ``a & b`` (intersect).
    """

    def __init__(self, low: Point3Like, high: Point3Like, tolerance: float = 0.01) -> None:
        """Create an independent CAD session without resetting other sessions."""
        NativePart, _, _ = _native()
        lx, ly, lz = _xyz(low)
        hx, hy, hz = _xyz(high)
        _log_progress("Part")
        self._n = NativePart(lx, ly, lz, hx, hy, hz, float(tolerance))

    @property
    def name(self) -> str:
        """Part instance name."""
        return self._n.name

    @property
    def operations(self) -> tuple[PartOperation, ...]:
        """Successful solid-producing features in chronological order.

        Each record has ``kind``, ``result``, input names, selected entities, and
        a concise CAD-style parameter summary. It is read-only history, not replay.
        """
        result = []
        for i in range(int(self._n.operation_count)):
            operation = _require(self._n, "get_operation")(i)
            result.append(PartOperation(int(operation.index), operation.kind,
                          operation.result_name,
                          tuple(_invoke(operation, "input_at", j) for j in range(int(operation.input_count))),
                          tuple(_invoke(operation, "entity_at", j) for j in range(int(operation.entity_count))),
                          operation.details))
        return tuple(result)
    @property
    def max_deviation(self) -> float:
        """Default tessellation tolerance (world units)."""
        return self._n.max_deviation


    def _selectable_names(self, kind):
        count = int(getattr(self._n, kind + "_count"))
        at = _require(self._n, kind + "_name_at")
        return tuple(at(i) for i in range(count))

    @property
    def patch_names(self) -> tuple[EntityName, ...]:
        """Canonical renderer patch names across all solids in this Part."""
        return self._selectable_names("patch")

    @property
    def curve_names(self) -> tuple[EntityName, ...]:
        """Canonical renderer mesh-edge curve names across all solids in this Part."""
        return self._selectable_names("curve")

    @property
    def point_names(self) -> tuple[EntityName, ...]:
        """Canonical renderer edge-anchor names across all solids in this Part."""
        return self._selectable_names("point")

    def smallest_unit(self) -> float:
        """Lattice step: operating-box extent / slices (~1e-6 of the box)."""
        return _require(self._n, "smallest_unit")()

    def _generate_name(self, prefix):
        """Next unique kernel name with this prefix."""
        return _require(_native_mod()["NativePart"], "generate_name")(prefix)

    @api_group("Create")
    def assembly(self, name: EntityName | None = None) -> Assembly:
        """Create or get an Assembly attached to this part."""
        return Assembly(_require(self._n, "get_assembly")(_name(name)), self)

    @api_group("Create")
    def sketch(self, plane: str = "xy", name: EntityName | None = None,
               origin_name: EntityName | None = None, frame: Frame | None = None,
               constrained: bool = False) -> Sketch:
        """2D sketch. ``plane`` is ``xy``/``xz``/``yz``; or pass ``frame=``.

        ``constrained=True`` attaches the constraint solver (coincident, length, …).
        """
        if constrained:
            if frame is not None:
                return Sketch(_require(self._n, "get_constraint_sketcher_on_frame")(
                    _as_frame(frame)._native(), _name(name)), self)
            return Sketch(_require(self._n, "get_constraint_sketcher")(plane or "xy", _name(name)), self)
        if frame is not None:
            return Sketch(_require(self._n, "sketch_on_frame")(_as_frame(frame)._native(), _name(name)), self)
        if origin_name:
            return Sketch(_require(self._n, "sketch_at")(plane or "xy", origin_name, _name(name)), self)
        return Sketch(_invoke(self._n, "sketch", plane or "xy", _name(name)), self)


    @api_group("Create")
    def section_sketch(self, solid: Solid, plane: Frame | None = None,
                       name: EntityName | None = None) -> Sketch:
        """Return the cross-section of ``solid`` as sampled curves in a Sketch.

        ``plane`` is a Frame and defaults to XY. Coplanar face overlap is omitted;
        each curve is one contiguous intersection with a source surface patch.
        offset the plane slightly to take a transverse cut at a model boundary.
        """
        if not isinstance(solid, Solid) or solid._part is not self:
            raise TypeError("section_sketch requires a Solid from this Part")
        plane = Frame() if plane is None else plane
        if not isinstance(plane, Frame):
            raise TypeError("plane must be a Frame")
        return Sketch(_require(self._n, "section_sketch")(solid._n, plane._native(), _name(name)), self)
    def _unregister_sketch(self, sketch):
        """Drop a sketch from the part (does not undo solids already built from it)."""
        _require(self._n, "unregister_sketch")(sketch._n)

    def _extrude(self, sketch, height, name=None, both_sides=False, max_deviation=-1, twist=0,
                taper_angle=0):
        """Extrude along the sketch-plane normal.

        ``height`` is in world units; ``twist`` is radians per unit length;
        ``taper_angle`` is radians and positive angles narrow along extrusion.
        Taper cannot be combined with ``both_sides`` or ``twist``.
        """
        n = _name(name)
        md = float(max_deviation)
        tw = float(twist)
        if both_sides:
            if float(taper_angle) != 0:
                raise ValueError("tapered extrusion cannot be combined with both_sides")
            return Solid(_require(self._n, "extrude_two_sides")(sketch._n, height, height, md, tw, n), self)
        return Solid(_require(self._n, "extrude")(
            sketch._n, height, md, tw, n, float(taper_angle)), self)

    def _extrude_two_sides(self, sketch, plus_z, minus_z=0.0, name=None, max_deviation=-1, twist=0):
        """Extrude ``plus_z`` along +normal and ``minus_z`` along −normal."""
        return Solid(_require(self._n, "extrude_two_sides")(
            sketch._n, plus_z, minus_z, float(max_deviation), float(twist), _name(name)), self)

    def _extrude_until_next(self, sketch, target, name=None, max_deviation=-1):
        """Extrude along the nearer of the two sketch-normal directions to ``target``."""
        return Solid(_require(self._n, "extrude_until_next")(
            sketch._n, target._n, float(max_deviation), _name(name)), self)

    def _face_surface(self, solid, patch_name, name=None):
        """Extract a named face as an oriented open surface for exact CSG trimming."""
        return Surface(_require(self._n, "extract_face_surface")(
            solid._n, str(patch_name), _name(name)), self)

    def _extrude_until_surface(self, sketch, surface, name=None, max_deviation=-1):
        """Extrude to an open surface that fully spans the sketch projection."""
        return Solid(_require(self._n, "extrude_until_surface")(
            sketch._n, surface._n, float(max_deviation), _name(name)), self)

    def _extrude_until_face(self, sketch, target, patch_name, name=None, max_deviation=-1):
        """Extrude to one named face of a target solid."""
        return Solid(_require(self._n, "extrude_until_face")(
            sketch._n, target._n, str(patch_name), float(max_deviation), _name(name)), self)

    def _project_sketch(self, sketch, solid, name=None, max_deviation=-1):
        """Tessellate sketch and project onto solid along the sketch-plane normal."""
        return ProjectedSketch(_require(self._n, "project_sketch_onto_mesh")(
            sketch._n, solid._n, float(max_deviation), _name(name)), self)

    def _extrude_projected(self, projected, height, name=None):
        """Extrude a ProjectedSketch along stored surface normals (negative = into solid)."""
        return Solid(_require(self._n, "extrude_projected_sketch")(
            projected._n, float(height), _name(name)), self)

    def _revolve(self, sketch, angle, name=None, max_deviation=-1):
        """Revolve the sketch about sketch X by ``angle`` radians."""
        return Solid(_require(self._n, "revolve")(sketch._n, float(angle), float(max_deviation), _name(name)), self)

    @api_group("Primitives")
    def cylinder(self, origin: Point3Like, radius: float, height: float,
                 name: EntityName | None = None, axis: Literal["x", "y", "z"] = "z",
                 max_deviation: float = -1) -> Solid:
        """Cylinder from a point or Frame. ``axis`` is ``x``/``y``/``z`` if origin is a point.

        Along ``pose.z`` when ``origin`` is a Frame; base at origin, height in +Z.
        """
        if isinstance(origin, Frame):
            pose = origin
        else:
            pose = frame_from_axis(origin, axis)
        return Solid(_require(self._n, "create_cylinder")(
            _as_frame(pose)._native(), float(radius), float(height), float(max_deviation), _name(name)), self)

    @api_group("Primitives")
    def cylinder_revolve(self, pose: Frame, radius: float, height: float,
                         name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Cylinder as a revolve of a rectangle (same pose convention as cylinder)."""
        return Solid(_require(self._n, "create_cylinder_revolve")(
            _as_frame(pose)._native(), float(radius), float(height), float(max_deviation), _name(name)), self)

    @api_group("Primitives")
    def sphere(self, center: Point3Like, radius: float,
               name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Sphere at ``center`` (point or Frame)."""
        return Solid(_require(self._n, "create_sphere")(
            _as_frame(center)._native(), float(radius), float(max_deviation), _name(name)), self)

    @api_group("Primitives")
    def cube(self, pose: Frame | Point3Like, extent: float,
             name: EntityName | None = None) -> Solid:
        """Axis-aligned cube of side ``extent`` in the pose (center at origin)."""
        return Solid(_require(self._n, "create_cube")(_as_frame(pose)._native(), float(extent), _name(name)), self)

    @api_group("Primitives")
    def cuboid(self, pose_or_min: Frame | Point3Like,
               extents_or_max: Point3Like, name: EntityName | None = None) -> Solid:
        """Pose + local extents, or world AABB min/max."""
        if isinstance(pose_or_min, Frame):
            ex, ey, ez = _xyz(extents_or_max)
            return Solid(_require(self._n, "create_cuboid")(
                pose_or_min._native(), ex, ey, ez, _name(name)), self)
        mn = vec3(pose_or_min)
        mx = vec3(extents_or_max)
        return Solid(_require(self._n, "create_cuboid_aabb")(
            mn.x, mn.y, mn.z, mx.x, mx.y, mx.z, _name(name)), self)

    def create_metric_thread_for_bolt_negative(
            self, pose: Frame, major_diameter: float, pitch: float, length: float,
            name: EntityName | None = None, max_deviation: float = -1,
            right_handed: bool = True, outer_radius: float = -1) -> Solid:
        """ISO 60° external-thread cutter (subtract from a shank). Axis is pose.z."""
        return Solid(_require(self._n, "create_metric_thread_for_bolt_negative")(
            _as_frame(pose)._native(),
            float(major_diameter),
            float(pitch),
            float(length),
            float(max_deviation),
            1 if right_handed else 0,
            _name(name),
            float(outer_radius)), self)

    def create_metric_thread_for_nut_negative(
            self, pose: Frame, major_diameter: float, pitch: float, length: float,
            name: EntityName | None = None, max_deviation: float = -1,
            right_handed: bool = True, include_bore_chamfers: bool = True) -> Solid:
        """ISO 60° internal-thread cutter (subtract from a nut). Axis is pose.z."""
        fn = _require(self._n, "create_metric_thread_for_nut_negative")
        args = (
            _as_frame(pose)._native(),
            float(major_diameter),
            float(pitch),
            float(length),
            float(max_deviation),
            1 if right_handed else 0,
            _name(name),
        )
        try:
            return Solid(fn(*args, 1 if include_bore_chamfers else 0), self)
        except TypeError:
            return Solid(fn(*args), self)

    def create_metric_thread_for_hole_negative(
            self, pose: Frame, major_diameter: float, pitch: float, length: float,
            name: EntityName | None = None, max_deviation: float = -1,
            right_handed: bool = True, include_bore_chamfers: bool = False) -> Solid:
        """ISO 60° internal-thread cutter (subtract from a holed plate). Axis is pose.z.

        Same solid as create_metric_thread_for_nut_negative. Bore chamfers default off.
        Falls back to the nut export if the wheel was built before the hole-named export.
        """
        native = getattr(self._n, "create_metric_thread_for_hole_negative", None)
        if native is None:
            native = _require(self._n, "create_metric_thread_for_nut_negative")
        args = (
            _as_frame(pose)._native(),
            float(major_diameter),
            float(pitch),
            float(length),
            float(max_deviation),
            1 if right_handed else 0,
            _name(name),
        )
        try:
            return Solid(native(*args, 1 if include_bore_chamfers else 0), self)
        except TypeError:
            return Solid(native(*args), self)

    def add_line(self, start: Point3Like | EntityName, end: Point3Like | EntityName,
                 name: EntityName | None = None) -> Curve3D:
        """3D construction line from coordinates, named points, or one of each."""
        start_is_name = isinstance(start, str)
        end_is_name = isinstance(end, str)
        if start_is_name and end_is_name:
            native = _require(self._n, "add_line_by_names")
            return Curve3D(native(start.strip(), end.strip(), _name(name)))
        if start_is_name:
            x1, y1, z1 = _xyz(end)
            native = _require(self._n, "add_line_from_start_name")
            return Curve3D(native(start.strip(), x1, y1, z1, _name(name)))
        if end_is_name:
            x0, y0, z0 = _xyz(start)
            native = _require(self._n, "add_line_to_end_name")
            return Curve3D(native(x0, y0, z0, end.strip(), _name(name)))
        x0, y0, z0 = _xyz(start)
        x1, y1, z1 = _xyz(end)
        return Curve3D(_require(self._n, "add_line")(x0, y0, z0, x1, y1, z1, _name(name)))

    def add_plane(self, plane_name: EntityName, origin_anchor_name: EntityName) -> None:
        """Named construction plane at a named origin point."""
        _require(self._n, "add_plane")(plane_name, origin_anchor_name)

    def plane_frame(self, plane: str) -> Frame | None:
        """Frame of a named part plane, or None."""
        native = _require(self._n, "get_plane_frame")(str(plane))
        if native is None:
            return None
        return Frame._from_native(native)

    def _extrude_along_curve(self, sketch, curve, name=None, max_deviation=-1, twist=0,
                            reference_direction=None):
        """Sweep a profile along a 3D Curve3D.

        Optional reference_direction projects a fixed world direction onto
        each normal plane to control profile orientation. It must never be
        parallel to the path tangent. The initial sketch sets profile clocking.
        """
        from .geom import _pack_points3
        reference = "" if reference_direction is None else _pack_points3([reference_direction])
        return Solid(_require(self._n, "extrude_along_curve")(
            sketch._n, curve._n, float(max_deviation), float(twist), _name(name), reference), self)

    def _extrude_along_curve_strip(self, sketch, curves, name=None, max_deviation=-1,
                                   twist=0, reference_direction=None):
        """Sweep the sketch along connected Curves with mitered sharp joins."""
        from .geom import _pack_points3
        reference = "" if reference_direction is None else _pack_points3([reference_direction])
        return Solid(_require(self._n, "extrude_along_curve_strip")(
            sketch._n, _curve_list(curves), float(max_deviation), float(twist),
            _name(name), reference), self)

    def _extrude_along_sketch(self, profile, guide, name=None, max_deviation=-1):
        """Sweep ``profile`` along a 3D path taken from ``guide`` sketch curves."""
        return Solid(_require(self._n, "extrude_along_sketch")(
            profile._n, guide._n, float(max_deviation), _name(name)), self)

    @api_group("Multi-profile")
    def loft(self, sketches: Sequence[Sketch], options: LoftOptions | None = None,
             name: EntityName | None = None, max_deviation: float = -1, *,
             first_curves: Sequence[str] | None = None) -> Solid:
        """Loft through Sketches, preserving their authored start points as connectors.

        ``options`` is LoftOptions. Symmetric sections are not automatically
        rotated according to their proximity to the sketch origin.
        ``first_curves`` optionally supplies one starting edge name per sketch,
        in section order. Names must resolve uniquely within their own sketches.
        Each selected curve's start-to-end direction defines section traversal;
        clockwise sections are not reversed to counterclockwise.
        This selects matched correspondence when options are omitted. Sections
        must have equal curve counts; each curve produces a separate side face.
        Arc/line and spline sections retain their exact NURBS shapes and require
        compatible rational weights after degree elevation and knot insertion.
        """
        sketches = list(sketches)
        if first_curves is not None:
            if isinstance(first_curves, str):
                raise TypeError("first_curves must be a list of curve names, one per section")
            first_curves = list(first_curves)
            if len(first_curves) != len(sketches):
                raise ValueError("first_curves must contain one curve name per section")
            if any(not isinstance(n, str) or not n.strip() or "|" in n for n in first_curves):
                raise ValueError("first_curves entries must be nonempty curve names without '|'")
        if options is None:
            options = LoftOptions(correspondence="vertices" if first_curves is not None else None)
        return Solid(_require(self._n, "loft")(
            _sketch_list(sketches), options._n, float(max_deviation), _name(name),
            "" if first_curves is None else _join_names(first_curves)), self)

    @api_group("Multi-profile")
    def loft_surface(self, sections: Sequence[Sketch], *, guides: Sequence[Curve3D] | None = None,
                     start_tangent: Point3Like | None = None,
                     end_tangent: Point3Like | None = None,
                     name: EntityName | None = None, max_deviation: float = -1) -> Surface:
        """Create an uncapped NURBS sheet through sketch sections.

        Sections retain authored curve order; degrees and knots are matched exactly.
        Rational sections must share weights after this conversion.
        Optional ``guides`` control the two side boundaries: use lines or
        ``Curve3D.hermite`` curves with one point at each section endpoint, in order.
        Optional tangents are world-space derivative vectors (direction and magnitude)
        for the start/end of the loft. Both point in the direction of section order.
        The loft parameter runs from zero to one with equal intervals per section.
        Conflicting guides/tangents raise an error; inputs are never fitted or snapped.
        """
        from .geom import _pack_points3
        def tangent(value):
            return "" if value is None else _pack_points3([value])
        return Surface(_require(self._n, "loft_surface")(
            _sketch_list(sections), _curve_list(() if guides is None else guides),
            tangent(start_tangent), tangent(end_tangent), float(max_deviation), _name(name)), self)

    @api_group("Surface modeling")
    def sew(self, surfaces: Iterable[Surface], *, make_solid: bool = False,
            name: EntityName | None = None) -> Surface | Solid:
        """Sew exactly coincident surface boundaries.

        Orientations are reconciled automatically. ``make_solid=True`` requires
        every exact boundary edge to be paired and returns a Solid; otherwise an
        open Surface is returned. No distance-based snapping is performed.
        """
        surfaces = list(surfaces)
        if not surfaces or any(not isinstance(surface, Surface) or surface._part is not self
                               for surface in surfaces):
            raise TypeError("surfaces must contain Surfaces from this Part")
        native = _require(self._n, "sew")(
            _solid_list(surfaces), bool(make_solid), _name(name))
        return Solid(native, self) if make_solid else Surface(native, self)

    @api_group("Booleans")
    def boolean(self, a: Solid, b: Solid, operation: int,
                name: EntityName | None = None) -> Solid:
        """CSG: ``operation`` is BOOLEAN_UNION / SUBTRACT / INTERSECT."""
        return Solid(_require(self._n, "boolean")(a._n, b._n, int(operation), _name(name)), self)

    @api_group("Booleans")
    def union(self, a: Solid, b: Solid, name: EntityName | None = None) -> Solid:
        """Boolean union. Same as ``a + b``."""
        return Solid(_invoke(self._n, "union", a._n, b._n, _name(name)), self)

    @api_group("Booleans")
    def subtract(self, a: Solid, b: Solid, name: EntityName | None = None) -> Solid:
        """Boolean subtraction ``a minus b``. Same as ``a - b``."""
        return Solid(_invoke(self._n, "subtract", a._n, b._n, _name(name)), self)

    @api_group("Booleans")
    def intersect(self, a: Solid, b: Solid, name: EntityName | None = None) -> Solid:
        """Boolean intersection. Same as ``a & b``."""
        return Solid(_invoke(self._n, "intersect", a._n, b._n, _name(name)), self)

    def _trim_by_surface(self, solid, surface, *, side="normal", name=None):
        """Trim a closed ``solid`` by an open ``surface``.

        ``side`` selects the retained half-space relative to the surface's
        triangle normals: ``"normal"`` keeps the normal side and
        ``"opposite"`` keeps the other side.
        """
        if not isinstance(solid, Solid) or isinstance(solid, Surface) or solid._part is not self:
            raise TypeError("solid must be a Solid from this Part")
        if not isinstance(surface, Surface) or surface._part is not self:
            raise TypeError("surface must be a Surface from this Part")
        if side == "normal":
            keep_normal = 1
        elif side == "opposite":
            keep_normal = 0
        else:
            raise ValueError("side must be 'normal' or 'opposite'")
        return Solid(_require(self._n, "trim_by_surface")(
            solid._n, surface._n, keep_normal, _name(name)), self)

    @api_group("Booleans")
    def batch_union(self, meshes: Iterable[Solid], name: EntityName | None = None) -> Solid | None:
        """Union many solids. Empty list → None; one item returned as-is."""
        if not meshes:
            return None
        if len(meshes) == 1:
            return meshes[0]
        return Solid(_require(self._n, "batch_union")(_solid_list(meshes)), self)

    @api_group("Booleans")
    def batch_subtract(self, base: Solid, cutters: Iterable[Solid],
                       name: EntityName | None = None) -> Solid:
        """Subtract the union of ``cutters`` from ``base`` in one Boolean step.

        ``cutters`` may be any iterable of solids. An empty iterable leaves
        ``base`` unchanged; the input solids are never modified.
        """
        if not isinstance(base, Solid) or base._part is not self:
            raise TypeError("base must be a Solid from this Part")
        cutters = list(cutters)
        if any(not isinstance(cutter, Solid) or cutter._part is not self
               for cutter in cutters):
            raise TypeError("cutters must be Solids from this Part")
        if not cutters:
            return base
        return Solid(_require(self._n, "batch_subtract")(
            base._n, _solid_list(cutters), _name(name)), self)

    @api_group("Booleans")
    def batch_boolean_chain(self, mesh_a: Solid,
                            steps: Sequence[tuple[Solid, int]]) -> Solid:
        """Apply ``steps`` as ``[(solid, BOOLEAN_*), ...]`` in order onto ``mesh_a``."""
        chain = _native_mod()["NativeBooleanChain"]()
        for solid, op in steps:
            chain.add(solid._n, int(op))
        return Solid(_require(self._n, "batch_boolean_chain")(mesh_a._n, chain), self)

    @api_group("Import")
    def solid_from_mesh(self, positions: Sequence[Point3Like],
                        triangles: Sequence[Sequence[int]],
                        name: EntityName | None = None) -> Surface | Solid:
        """Build a Solid or open Surface from world-space vertices and triangles."""
        from .geom import _pack_points3, _pack_triangles
        native = _require(self._n, "solid_from_mesh")(
            _pack_points3(positions), _pack_triangles(triangles), _name(name))
        return _body(native, self)

    def _raycast(self, solid, origin, direction):
        """Nearest mesh hit from ``origin`` along ``direction``, or ``None``.

        ``hit.t`` is the fraction of the kernel's internal cast segment:
        zero at its origin and one at its far end. Use ``hit.point`` for
        the world-space intersection.
        """
        from .geom import _parse_ray_hit
        ox, oy, oz = _xyz(origin)
        dx, dy, dz = _xyz(direction)
        packed = _require(self._n, "raycast")(
            solid._n, float(ox), float(oy), float(oz), float(dx), float(dy), float(dz))
        return _parse_ray_hit(packed)

    def _pattern_linear(self, solid, count, step, name=None):
        """Return independent solids at equal XYZ steps, including ``solid``.

        ``count`` includes the unchanged seed. ``name`` prefixes the new names.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("count must be a positive integer")
        x, y, z = _xyz(step)
        result = _invoke(self._n, "pattern_linear", solid._n, count,
                         float(x), float(y), float(z), _name(name))
        return [solid] + [Solid(_invoke(result, "get", i), self) for i in range(1, int(result.count))]

    def _pattern_circular(self, solid, count, axis=None, angle=2*math.pi, name=None):
        """Return copies rotated around ``axis.z``; count includes the seed.

        Axis defaults to Frame(). A full sweep (exactly ±2*pi radians) omits
        the duplicate endpoint; a partial sweep includes both endpoints.
        ``name`` prefixes the new names. The source remains unchanged.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("count must be a positive integer")
        axis = Frame() if axis is None else _as_frame(axis)
        result = _invoke(self._n, "pattern_circular", solid._n, count,
                         axis._native(), float(angle), _name(name))
        return [solid] + [Solid(_invoke(result, "get", i), self) for i in range(1, int(result.count))]

    def _mirror(self, solid, plane=None, name=None):
        """Return a new solid reflected in the XY plane of ``plane`` (a Frame).

        The default is the world XY plane. Source geometry and names stay intact.
        """
        plane = Frame() if plane is None else _as_frame(plane)
        return Solid(_invoke(self._n, "mirror", solid._n, plane._native(), _name(name)), self)

    @api_group("Create")
    def copy_solid(self, source: Solid, name: EntityName) -> Solid:
        """Deep-copy a solid, rewriting its entity prefixes to the required new name.

        Cross-Part copies require the same coordinate lattice (origin and step).
        Geometry is copied exactly; this operation does not resample another
        session's integer coordinates onto a different working lattice.
        """
        if name is None or str(name).strip() == "":
            raise ValueError("copy_solid requires a new part name")
        name = str(name)
        if source is not None and source.name == name:
            raise ValueError("copy_solid requires a name different from the source ({0!r})".format(name))
        if source._part is not self:
            def lattice(part):
                packed = _require(part._n, "pack_working_volume")().split()
                return tuple(float(value) for value in packed[1:4]), part.smallest_unit()
            if lattice(source._part) != lattice(self):
                raise ValueError("copy_solid requires matching working coordinate lattices; use Parts with the same working volume")
        return Solid(_require(self._n, "copy_solid_as_instance")(source._n, name), self)

    def _fillet(self, solid, edges, radius, name=None, max_deviation=-1):
        """Fillet named edges. ``edges`` is a string or list of names.

        Continues across supported planar/cylindrical tangent seams. Surviving
        original faces keep their names; sharp boundaries are preserved.
        """
        return Solid(_require(self._n, "fillet")(
            solid._n, _join_names(edges), float(radius), float(max_deviation), _name(name)), self)

    def _chamfer(self, solid, edges, distance, name=None, max_deviation=-1):
        """Chamfer named edges. ``edges`` is a string or list of names."""
        return Solid(_require(self._n, "chamfer")(
            solid._n, _join_names(edges), float(distance), float(max_deviation), _name(name)), self)

    def _hole(self, solid, mouth, diameter, depth=None, *, name=None, max_deviation=-1):
        """Drill a flat-bottom hole. ``mouth.z`` points into the solid; no depth means through."""
        return Solid(_require(self._n, "drill_hole")(
            solid._n, _as_frame(mouth)._native(), float(diameter),
            -1.0 if depth is None else float(depth), float(max_deviation), _name(name)), self)

    def _counterbore_hole(self, solid, mouth, diameter, counterbore_diameter,
                         counterbore_depth, depth=None, *, name=None, max_deviation=-1):
        """Drill a hole with a flat-bottom counterbore. No drill depth means through."""
        return Solid(_require(self._n, "counterbore_hole")(
            solid._n, _as_frame(mouth)._native(), float(diameter),
            -1.0 if depth is None else float(depth), float(counterbore_diameter),
            float(counterbore_depth), float(max_deviation), _name(name)), self)

    def _countersink_hole(self, solid, mouth, diameter, countersink_diameter,
                         included_angle, depth=None, *, name=None, max_deviation=-1):
        """Drill a hole with a conical countersink; ``included_angle`` is in radians."""
        return Solid(_require(self._n, "countersink_hole")(
            solid._n, _as_frame(mouth)._native(), float(diameter),
            -1.0 if depth is None else float(depth), float(countersink_diameter),
            float(included_angle), float(max_deviation), _name(name)), self)

    def _draft_faces(self, solid, faces, neutral_frame, angle, *, pull_direction=None,
                    name=None, max_deviation=-1):
        """Draft named prism side faces about a neutral cap plane.

        ``neutral_frame.z`` is the default pull direction. Currently limited to
        straight, convex polygonal prisms; unsupported geometry fails explicitly.
        """
        neutral = _as_frame(neutral_frame)
        direction = neutral.z if pull_direction is None else vec3(pull_direction)
        return Solid(_require(self._n, "draft_prismatic_faces")(
            solid._n, _join_names(faces), neutral._native(),
            direction.x, direction.y, direction.z, float(angle),
            float(max_deviation), _name(name)), self)

    def _shell(self, solid, thickness, faces=None, name=None, max_deviation=-1, outward=False, join="sharp"):
        """Hollow a solid inward, or expand it with ``outward=True``.

        Inward shelling keeps exterior dimensions and opening boundaries fixed. Planar,
        cylindrical, and spherical patches use exact analytic offsets; other
        or metadata-free patches use a welded triangle-normal offset. Collapsed,
        inverted, or unsupported topology-changing offsets fail explicitly;
        union seams may be rebuilt from their operands with CSG. Cavity faces
        are selectable as ``ShellInner_<source-patch>``. Without ``faces``, the
        cavity is fully enclosed. ``join="round"`` rounds outward joins with
        radius equal to the thickness; sharp joins remain the default. Inward
        round joins and unsupported concave offset corners fail explicitly.
        """
        if join not in ("sharp", "round"):
            raise ValueError("join must be 'sharp' or 'round'")
        return Solid(_require(self._n, "shell")(
            solid._n, float(thickness), _join_names(faces or []), float(max_deviation), _name(name), bool(outward), join == "round"), self)

    def _thicken(self, surface, thickness, *, both_sides=False, name=None):
        """Turn an oriented surface into a solid using its surface normals."""
        if not isinstance(surface, Surface) or surface._part is not self:
            raise TypeError("surface must be a Surface from this Part")
        return Solid(_require(self._n, "thicken")(
            surface._n, float(thickness), bool(both_sides), _name(name)), self)

    def _rib(self, base, path, thickness, to, *, frame=None, name=None, max_deviation=-1):
        """Union a constant-width polyline rib, terminated by an exact extrusion.

        ``path`` is an open sequence of 2D points in ``frame``. ``frame.z``
        points into the rib. ``to`` is a positive height, a target Solid
        (next intersection), a Surface, or ``(target_solid, face_name)``.
        A face/surface must cover the whole rib profile; incomplete trims fail.
        """
        if not isinstance(base, Solid) or base._part is not self:
            raise TypeError("base must be a Solid from this Part")
        frame = Frame() if frame is None else frame
        if not isinstance(frame, Frame):
            raise TypeError("frame must be a Frame")
        points = [_xy(point) for point in path]
        width = float(thickness)
        if len(points) < 2 or not math.isfinite(width) or width <= 0:
            raise ValueError("rib needs an open path of at least two points and positive thickness")
        if any(not all(math.isfinite(v) for v in point) for point in points):
            raise ValueError("rib path coordinates must be finite")
        if any(points[i] == points[i + 1] for i in range(len(points) - 1)):
            raise ValueError("rib path cannot contain zero-length segments")
        if isinstance(to, bool):
            raise TypeError("to must be a height, Solid, Surface, or (Solid, face_name)")
        if isinstance(to, (int, float)):
            if not math.isfinite(to) or to <= 0:
                raise ValueError("rib height must be finite and positive")
            mode = "height"
        elif isinstance(to, Surface) and to._part is self:
            mode = "surface"
        elif isinstance(to, Solid) and to._part is self:
            mode = "next"
        elif (isinstance(to, tuple) and len(to) == 2 and
              isinstance(to[0], Solid) and to[0]._part is self):
            mode = "face"
        else:
            raise TypeError("to must be a height, Solid, Surface, or (Solid, face_name) from this Part")

        sketch = self.sketch(frame=frame, name=self._generate_name("rib_profile"))
        centerline = [sketch.add_line(points[i], points[i + 1], construction=True)
                      for i in range(len(points) - 1)]
        sketch.offset(centerline, width / 2, side="both", join="miter",
                      end_cap="butt", open_mode="outline")
        tool_name = self._generate_name("rib_tool")
        if mode == "height":
            rib = sketch.extrude(to, name=tool_name,
                               max_deviation=max_deviation)
        elif mode == "surface":
            rib = sketch.extrude_until_surface(to, name=tool_name,
                                              max_deviation=max_deviation)
        elif mode == "next":
            rib = sketch.extrude_until_next(to, name=tool_name,
                                           max_deviation=max_deviation)
        else:
            rib = sketch.extrude_until_face(to[0], to[1], name=tool_name,
                                           max_deviation=max_deviation)
        return self.union(base, rib, name=base.name if name is None else name)

    @api_group("Inspect")
    def solid(self, name: EntityName) -> Surface | Solid | None:
        """Look up a Solid or Surface already registered on this part, or None."""
        found = _require(self._n, "get_mesh_from_name")(name)
        if found is None:
            return None
        return _body(found, self)

    @api_group("Import")
    def load_stl(self, path: str, group_border_angle_deg: float,
                 name: EntityName | None = None, require_watertight: bool = True) -> Surface | Solid:
        """Import STL as a Solid or, when allowed, an open Surface."""
        native = _require(self._n, "load_stl_file")(
            path, float(group_border_angle_deg), _name(name), 1 if require_watertight else 0)
        return _body(native, self)

    @api_group("Import")
    def load_off(self, path: str, group_border_angle_deg: float,
                 name: EntityName | None = None, require_watertight: bool = True) -> Surface | Solid:
        """Import OFF as a Solid or, when allowed, an open Surface."""
        native = _require(self._n, "load_off_file")(
            path, float(group_border_angle_deg), _name(name), 1 if require_watertight else 0)
        return _body(native, self)

    @api_group("Import")
    def load_obj(self, path: str, group_border_angle_deg: float = -1,
                 name: EntityName | None = None, scale: float = 1.0) -> Surface | Solid:
        """Import Wavefront OBJ. ``group_border_angle_deg`` splits patches at sharp edges
        (viewer edges are drawn on patch borders; use e.g. 22 when the OBJ has no ``g`` tags).
        ``scale`` multiplies vertex positions (e.g. 0.01 for cm→m)."""
        native = _require(self._n, "load_wavefront_obj_file")(
            path, float(group_border_angle_deg), _name(name), float(scale))
        return _body(native, self)

    def show(self, title: str = "Camber") -> None:
        """Open the 3D viewer on all meshes in this part."""
        from .view import show as _show
        _show(self, title=title)

    def sketch_interactive(
            self, plane: str = "xy", name: EntityName | None = None,
            part_var: str = "part", sketch_var: str = "sk",
            frame: Frame | None = None, emit_frame: bool = False) -> tuple[Sketch | None, str | None]:
        """Draw a sketch in the viewer. Finish copies camber Python to the clipboard.

        Returns (Sketch, code) or (None, None) if cancelled. Requires pyglet + imgui.
        """
        from .sketch_ui import sketch_interactive as _run
        return _run(
            self, plane=plane, name=_name(name), part_var=part_var, sketch_var=sketch_var,
            frame=frame, emit_frame=emit_frame)

    def __repr__(self):
        return "Part(name={0!r})".format(self.name)


class Sketch(object):
    """2D sketch on a Part: plotter strips, named curves, and optional constraints.

    Points are ``vec2``/tuples or names like ``\"Line1@1.000\"``. Constraints
    return ``self`` for chaining. Call ``solve()`` after a batch of constraints
    unless ``solve_after_every_constraint`` is True (default).
    """

    def __init__(self, native: object, part: Part) -> None:
        self._n = native
        self._part = part

    @api_group("Build solids")
    def extrude(self, height: float, name: EntityName | None = None, both_sides: bool = False,
                max_deviation: float = -1, twist: float = 0,
                taper_angle: float = 0) -> Solid:
        """Extrude this sketch; positive taper narrows the far end (radians)."""
        return self._part._extrude(self, height, name, both_sides, max_deviation,
                                   twist, taper_angle)

    @api_group("Surface modeling")
    def extrude_surface(self, height: float, name: EntityName | None = None,
                        max_deviation: float = -1) -> Surface:
        """Extrude the sketch boundary into an uncapped side surface."""
        return Surface(_require(self._part._n, "extrude_surface")(
            self._n, float(height), float(max_deviation), _name(name)), self._part)

    @api_group("Build solids")
    def extrude_two_sides(self, plus_z: float, minus_z: float = 0.0,
                          name: EntityName | None = None, max_deviation: float = -1,
                          twist: float = 0) -> Solid:
        """Extrude along both sides of the sketch plane."""
        return self._part._extrude_two_sides(self, plus_z, minus_z, name, max_deviation, twist)

    @api_group("Build solids")
    def extrude_until_next(self, target: Solid, name: EntityName | None = None,
                           max_deviation: float = -1) -> Solid:
        """Extrude to the first intersection with a target solid."""
        return self._part._extrude_until_next(self, target, name, max_deviation)

    @api_group("Build solids")
    def extrude_until_surface(self, surface: Surface, name: EntityName | None = None,
                              max_deviation: float = -1) -> Solid:
        """Extrude to an open surface spanning the entire profile."""
        return self._part._extrude_until_surface(self, surface, name, max_deviation)

    @api_group("Build solids")
    def extrude_until_face(self, target: Solid, patch_name: EntityName,
                           name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Extrude to a named face of a target solid."""
        return self._part._extrude_until_face(self, target, patch_name, name, max_deviation)

    @api_group("Build solids")
    def revolve(self, angle: float, name: EntityName | None = None,
                max_deviation: float = -1) -> Solid:
        """Revolve this sketch around its X axis by an angle in radians."""
        return self._part._revolve(self, angle, name, max_deviation)

    @api_group("Surface modeling")
    def revolve_surface(self, angle: float, name: EntityName | None = None,
                        max_deviation: float = -1) -> Surface:
        """Revolve the sketch into an uncapped surface around its X axis."""
        return Surface(_require(self._part._n, "revolve_surface")(
            self._n, float(angle), float(max_deviation), _name(name)), self._part)

    @api_group("Build solids")
    def extrude_along_curve(self, curve: Curve3D, name: EntityName | None = None,
                            max_deviation: float = -1, twist: float = 0,
                            reference_direction: Point3Like | None = None) -> Solid:
        """Sweep this profile along a 3D curve."""
        return self._part._extrude_along_curve(self, curve, name, max_deviation,
                                               twist, reference_direction)

    @api_group("Surface modeling")
    def sweep_surface(self, guide: Curve3D | Sequence[Curve3D], name: EntityName | None = None,
                      max_deviation: float = -1, twist: float = 0,
                      reference_direction: Point3Like | None = None) -> Surface:
        """Sweep the sketch boundary along a 3D curve or connected curve list."""
        from .geom import _pack_points3
        reference = "" if reference_direction is None else _pack_points3([reference_direction])
        if isinstance(guide, Curve3D):
            native = _require(self._part._n, "sweep_surface")(
                self._n, guide._n, float(max_deviation), float(twist), _name(name), reference)
        else:
            native = _require(self._part._n, "sweep_surface_strip")(
                self._n, _curve_list(guide), float(max_deviation), float(twist), _name(name), reference)
        return Surface(native, self._part)

    @api_group("Build solids")
    def extrude_along_curve_strip(self, curves: Sequence[Curve3D], name: EntityName | None = None,
                                  max_deviation: float = -1, twist: float = 0,
                                  reference_direction: Point3Like | None = None) -> Solid:
        """Sweep a profile along connected curves; sharp joins use a miter section."""
        return self._part._extrude_along_curve_strip(self, curves, name, max_deviation,
                                                     twist, reference_direction)

    @api_group("Build solids")
    def extrude_along_sketch(self, guide: Sketch, name: EntityName | None = None,
                             max_deviation: float = -1) -> Solid:
        """Sweep this profile along a guide sketch."""
        return self._part._extrude_along_sketch(self, guide, name, max_deviation)

    @api_group("Project")
    def project_onto(self, solid: Solid, name: EntityName | None = None,
                     max_deviation: float = -1) -> ProjectedSketch:
        """Project this sketch onto a solid along its plane normal."""
        return self._part._project_sketch(self, solid, name, max_deviation)

    @property
    def name(self) -> str:
        """Kernel sketch name."""
        return self._n.name

    @property
    def solve_after_every_constraint(self) -> bool:
        """If True, the kernel solves after each constraint (default)."""
        flag = getattr(self._n, "solve_after_every_constraint", None)
        if flag is None:
            return True
        return bool(flag)

    @solve_after_every_constraint.setter
    def solve_after_every_constraint(self, value: bool) -> None:
        self._n.solve_after_every_constraint = 1 if value else 0

    @property
    def curve_count(self) -> int:
        """Number of stored curves in this sketch, including sampled curves."""
        n = getattr(self._n, "curve_count", None)
        if n is None:
            n = getattr(self._n, "constraint_curve_count", None)
        if n is None:
            return -1
        return int(n)

    @property
    def constraint_count(self) -> int:
        """Number of constraints, or -1 if this is a plotter-only sketch."""
        n = getattr(self._n, "constraint_count", None)
        if n is None:
            return -1
        return int(n)

    def remove_last_constraint(self) -> Sketch:
        """Undo the last constraint. Returns self."""
        _require(self._n, "remove_last_constraint")()
        return self

    def remove_last_curve(self) -> Sketch:
        """Undo the last constraint-solver curve. Returns self."""
        _require(self._n, "remove_last_constraint_curve")()
        return self

    @property
    def frame(self) -> Frame:
        """World Frame of this sketch plane."""
        return Frame._from_native(_require(self._n, "frame")())

    def add_line(self, start: Point2Like | str, end: Point2Like | str,
                 name: EntityName | None = None, construction: bool = False) -> SketchCurve:
        """Segment from start to end (XY or named points). Returns SketchCurve."""
        start_xy, start_name = self._resolved_endpoint(start)
        end_xy, end_name = self._resolved_endpoint(end)
        x0, y0 = start_xy
        x1, y1 = end_xy
        index = self.curve_count
        if name:
            _require(self._n, "add_line_named")(x0, y0, x1, y1, _name(name))
        else:
            _invoke(self._n, "add_line", x0, y0, x1, y1)
        curve_name = self._added_curve_name(name, "line", index)
        if construction:
            self.set_construction(curve_name, True)
        if start_name:
            self._snap_added_point(curve_name, "line", 0, start_name)
        if end_name:
            self._snap_added_point(curve_name, "line", 1, end_name)
        return SketchCurve(curve_name, "line")

    def add_ellipse(self, center: Point2Like | str, radii: Sequence[float],
                    rotation: float = 0, name: EntityName | None = None,
                    construction: bool = False) -> SketchCurve:
        """Ellipse with two semiaxes; rotation is in radians from sketch X.

        Dimension axes with ``distance(e @ "center", e @ 0, rx)`` and
        ``distance(e @ "center", e @ .25, ry)``. These points stay live as
        dimensions change. A construction line joining the center to ``e @ 0``
        can carry the usual horizontal or angle constraint.
        """
        center_xy, center_name = self._resolved_endpoint(center)
        rx, ry = radii
        values = (*center_xy, rx, ry, rotation)
        if not all(math.isfinite(v) for v in values) or rx <= 0 or ry <= 0:
            raise ValueError("ellipse radii must be positive and all parameters finite")
        index = self.curve_count
        _require(self._n, "add_ellipse")(*center_xy, rx, ry, rotation, _name(name))
        curve_name = self._added_curve_name(name, "ellipse", index)
        if construction:
            self.set_construction(curve_name, True)
        if center_name:
            self.coincident(curve_name + "@center", center_name)
        return SketchCurve(curve_name, "ellipse")

    def add_circle(self, center: Point2Like | str, radius: float,
                   name: EntityName | None = None, construction: bool = False) -> SketchCurve:
        """Circle. ``center`` is XY or a named point. Returns SketchCurve."""
        center_xy, center_name = self._resolved_endpoint(center)
        cx, cy = center_xy
        index = self.curve_count
        if name:
            _require(self._n, "add_circle_named")(cx, cy, radius, _name(name))
        else:
            _invoke(self._n, "add_circle", cx, cy, radius)
        curve_name = self._added_curve_name(name, "circle", index)
        if construction:
            self.set_construction(curve_name, True)
        if center_name:
            self._snap_added_point(curve_name, "circle", 2, center_name)
        return SketchCurve(curve_name, "circle")

    def add_arc(self, start: Point2Like | str, mid: Point2Like | str,
                end: Point2Like | EntityName, name: EntityName | None = None,
                construction: bool = False) -> SketchCurve:
        """Circular arc through three points. Returns SketchCurve."""
        start_xy, start_name = self._resolved_endpoint(start)
        mid_xy, mid_name = self._resolved_endpoint(mid)
        end_xy, end_name = self._resolved_endpoint(end)
        x0, y0 = start_xy
        xm, ym = mid_xy
        x1, y1 = end_xy
        index = self.curve_count
        if name:
            _require(self._n, "add_arc_named")(x0, y0, xm, ym, x1, y1, _name(name))
        else:
            _invoke(self._n, "add_arc", x0, y0, xm, ym, x1, y1)
        curve_name = self._added_curve_name(name, "arc", index)
        if construction:
            self.set_construction(curve_name, True)
        if start_name:
            self._snap_added_point(curve_name, "arc", 0, start_name)
        if mid_name:
            self._snap_added_point(curve_name, "arc", 3, mid_name)
        if end_name:
            self._snap_added_point(curve_name, "arc", 1, end_name)
        return SketchCurve(curve_name, "arc")

    def _added_curve_name(self, given, kind, index):
        if given:
            return given
        native_name = getattr(self._n, "last_curve_name", None)
        if native_name is not None:
            return str(native_name())
        dumped = self._solved_actions()
        from . import pick as _pick
        if 0 <= index < len(dumped):
            action = dumped[index]
            return (action.get("name") or "").strip() or _pick.sketch_curve_display_name(action, index)
        if dumped:
            action = dumped[-1]
            return (action.get("name") or "").strip() or _pick.sketch_curve_display_name(
                action, len(dumped) - 1)
        return _pick.sketch_curve_display_name({"kind": kind}, 0 if index < 0 else index)

    def _snap_added_point(self, curve_name, kind, role, other_name):
        from . import naming as _naming
        self.coincident(_naming.format_sketch_handle_address(curve_name, kind, role), other_name)

    def set_construction(self, curve: SketchCurve | str, construction: bool = True) -> Sketch:
        """Mark a sketch curve as construction geometry (excluded from extrude/loft)."""
        named = getattr(self._n, "set_curve_construction_named", None)
        if named is not None:
            if isinstance(curve, SketchCurve):
                name = curve.name
            else:
                name = str(curve).strip()
            try:
                named(name, 1 if construction else 0)
                return self
            except Exception:
                pass
        index = _curve_index(self, curve)
        fn = getattr(self._n, "set_curve_construction", None)
        if fn is None:
            return self
        fn(int(index), 1 if construction else 0)
        return self

    def _resolved_endpoint(self, point):
        kind, value = _endpoint_value(point)
        if kind == "name":
            return _eval_named_point(self, value), value
        if kind == "xy":
            return value, None
        if kind == "handle":
            raise ValueError("use a name or XY for add_line/add_circle/add_arc endpoints")
        return value, None

    def add_rectangle(self, corner_a: Point2Like, corner_b: Point2Like,
                      names: Sequence[str] | None = None) -> tuple[SketchCurve, ...]:
        """Axis-aligned rectangle, CCW from the lower-left corner.

        Returns (south, east, north, west) as SketchCurves. Optional `names` is
        those four side names (native `add_rectangle_named` when present).
        """
        x0, y0 = _xy(corner_a)
        x1, y1 = _xy(corner_b)
        if names:
            south, east, north, west = names
            named = getattr(self._n, "add_rectangle_named", None)
            if named is not None:
                _invoke(self._n, "add_rectangle_named", x0, y0, x1, y1, _name(south), _name(east), _name(north), _name(west))
                return (
                    SketchCurve(south, "line"),
                    SketchCurve(east, "line"),
                    SketchCurve(north, "line"),
                    SketchCurve(west, "line"),
                )
            raise RuntimeError("Named rectangles require a rebuilt Camber wheel")
        index = self.curve_count
        _invoke(self._n, "add_rectangle", x0, y0, x1, y1)
        dumped = self._solved_actions()
        sides = []
        for i, action in enumerate(dumped[index:]):
            name = (action.get("name") or "").strip()
            if not name:
                from . import pick as _pick
                name = _pick.sketch_curve_display_name(action, index + i)
            sides.append(SketchCurve(name, "line"))
        if len(sides) < 4:
            raise RuntimeError("add_rectangle expected 4 sides, got {0}".format(len(sides)))
        return tuple(sides[:4])

    def add_text(self, text: str, origin: Point2Like, family: str = "Arial",
                 em_size: float = 0.2, bold: bool = False, italic: bool = False) -> Sketch:
        """Add TrueType glyph outlines at the baseline origin (sketch XY)."""
        x, y = _xy(origin)
        flags = 0
        if bold:
            flags |= 1
        if italic:
            flags |= 2
        _require(self._n, "add_text")(str(text), float(x), float(y), family or "", float(em_size), int(flags))
        return self

    def polylines(self, max_deviation: float = -1) -> list[list[vec2]]:
        """Tessellated sketch strips as a list of ``vec2`` polylines."""
        from .geom import _unpack_loops2
        md = float(max_deviation)
        if md <= 0:
            md = float(self._part.max_deviation)
        return _unpack_loops2(_require(self._n, "tessellate_polylines")(md))

    def triangulate(self, max_deviation: float = -1, working_volume: Part | None = None,
                    slices: int | None = None) -> tuple[list[vec2], list[tuple[int, int, int]]]:
        """Fill closed sketch contours. Nested holes and islands are handled by the kernel polygon tree.

        Returns ``(points, triangles)`` as ``vec2`` and ``(i, j, k)``. Construction
        geometry is skipped. Default ``working_volume`` is this sketch's Part lattice.
        """
        from .geom import _pack_working_volume, _unpack_indexed2
        md = float(max_deviation)
        if md <= 0:
            md = float(self._part.max_deviation)
        vol = _pack_working_volume(
            self._part if working_volume is None else working_volume, slices)
        return _unpack_indexed2(_require(self._n, "triangulate")(md, vol))

    def surface(self, name: EntityName | None = None,
                max_deviation: float = -1) -> Surface:
        """Fill closed contours and register an open sheet on the part.

        Returns a planar ``Surface`` with ``is_volume`` False, not a solid.
        """
        pts2, tris = self.triangulate(max_deviation)
        if not tris:
            raise ValueError("sketch has no closed fill")
        fr = self.frame
        pts3 = [fr.origin + fr.x * p.x + fr.y * p.y for p in pts2]
        return self._part.solid_from_mesh(pts3, tris, name=name)

    def horizontal(self, curve: SketchCurve | str) -> Sketch:
        """Keep a line parallel to sketch X. Returns self."""
        _require(self._n, "set_horizontal")(_curve_index(self, curve))
        return self

    def vertical(self, curve: SketchCurve | str) -> Sketch:
        """Keep a line parallel to sketch Y. Returns self."""
        _require(self._n, "set_vertical")(_curve_index(self, curve))
        return self

    def coincident(self, point_a: str | Point2Like, point_b: str | Point2Like) -> Sketch:
        """Coincident two points. Each is a name (\"Line1@1.000\") or (\"xy\", x, y)."""
        self._lock_named_derived_radius(point_a)
        self._lock_named_derived_radius(point_b)
        _dispatch_two_points(
            self._n,
            point_a,
            point_b,
            name_name=("set_coincident_names", "set_coincident_named"),
            name_xy="set_coincident_name_xy",
            two_constants="coincident needs at least one sketch point",
        )
        return self

    def _lock_named_derived_radius(self, point):
        """Freeze circle/arc radius when coinciding a cardinal or arc mid (native handle path)."""
        named = _named_point_ref(point)
        if not named:
            return
        from . import naming as _naming
        from . import pick as _pick
        owner, local = _naming.parse_qualified(named)
        parsed = _naming.parse_sketch_curve_address(local or named)
        if parsed is None or parsed.get("center"):
            return
        try:
            dumped = self._solved_actions()
        except Exception:
            dumped = []
        curve = parsed["curve"]
        action = None
        for index, item in enumerate(dumped):
            action_name = (item.get("name") or "").strip()
            if action_name == curve or _pick.sketch_curve_display_name(item, index) == curve:
                action = item
                break
        if action is None:
            return
        kind = action.get("kind")
        if kind == "circle":
            self.radius(curve, action["radius"])
            return
        if kind == "arc" and abs(float(parsed.get("uniform") or 0.0) - 0.5) < 1e-9:
            center = _arc_center_xy(action)
            if center is None:
                return
            start = action["start"]
            self.radius(curve, math.hypot(start[0] - center[0], start[1] - center[1]))

    def coincident_on_curve(self, point: str, curve: SketchCurve | str,
                            uniform: float) -> Sketch:
        """Place a named point on another curve at uniform parameter u in [0, 1]."""
        kind, value = _classify_point(point)
        if kind == "name":
            _require(self._n, "set_coincident_on_curve_named")(
                _native_point_name(value), _curve_index(self, curve), float(uniform))
            return self
        raise ValueError("coincident_on_curve needs a named point")

    def parallel(self, curve_a: SketchCurve | str, curve_b: SketchCurve | str) -> Sketch:
        """Keep two lines parallel. Returns self."""
        _require(self._n, "set_parallel")(_curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def perpendicular(self, curve_a: SketchCurve | str, curve_b: SketchCurve | str) -> Sketch:
        """Keep two lines perpendicular. Returns self."""
        _require(self._n, "set_perpendicular")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def tangent(self, line_curve: SketchCurve | str,
                circular_curve: SketchCurve | str) -> Sketch:
        """Line tangent to a circle/arc. Returns self."""
        _require(self._n, "set_tangent")(
            _curve_index(self, line_curve), _curve_index(self, circular_curve))
        return self

    def equal(self, curve_a: SketchCurve | str, curve_b: SketchCurve | str) -> Sketch:
        """Equal length (lines) or equal radius (circles/arcs). Returns self."""
        _require(self._n, "set_equal")(_curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def midpoint(self, point: str, line_curve: SketchCurve | str) -> Sketch:
        """Named point at the midpoint of a line. Returns self."""
        kind, value = _classify_point(point)
        curve = _curve_index(self, line_curve)
        if kind == "name":
            _require(self._n, "set_midpoint_named")(_native_point_name(value), curve)
            return self
        raise ValueError("midpoint needs a named point")

    def point_on_line(self, point: str, line_curve: SketchCurve | str) -> Sketch:
        """Put a named point on a line (infinite line through the segment).

        ``line_curve`` may be a sketch curve or a sketch axis (``sk @ \"x\"`` / ``sk @ \"y\"``).
        """
        kind, value = _classify_point(point)
        if kind != "name":
            raise ValueError("point_on_line needs a named point")
        from . import naming as _naming
        axis = None
        if not isinstance(line_curve, SketchCurve):
            axis = _naming.sketch_axis_index(str(line_curve).strip())
        if axis is not None:
            _require(self._n, "set_point_on_sketch_axis_named")(_native_point_name(value), int(axis))
            return self
        curve = _curve_index(self, line_curve)
        _require(self._n, "set_point_on_line_named")(_native_point_name(value), curve)
        return self

    def point_on_circle(self, point: str, circle_curve: SketchCurve | str) -> Sketch:
        """Put a named point on a circle circumference."""
        kind, value = _classify_point(point)
        if kind != "name":
            raise ValueError("point_on_circle needs a named point")
        _require(self._n, "set_point_on_circle_named")(
            _native_point_name(value), _curve_index(self, circle_curve))
        return self

    def distance_to_line(self, point: str, line_curve: SketchCurve | str,
                         value: float) -> Sketch:
        """Perpendicular distance from a named point to a line."""
        kind, value_pt = _classify_point(point)
        if kind != "name":
            raise ValueError("distance_to_line needs a named point")
        _require(self._n, "set_distance_point_line_named")(
            _native_point_name(value_pt), _curve_index(self, line_curve), float(value))
        return self

    def tangent_circles(self, curve_a: SketchCurve | str,
                        curve_b: SketchCurve | str) -> Sketch:
        """Two circles/arcs tangent to each other. Returns self."""
        _require(self._n, "set_tangent_circles")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def vertical_distance(self, point_a: str, point_b: str, value: float) -> Sketch:
        """Signed vertical offset ``b.y - a.y = value``."""
        kind_a, a = _classify_point(point_a)
        kind_b, b = _classify_point(point_b)
        if kind_a != "name" or kind_b != "name":
            raise ValueError("vertical_distance needs two named points")
        _require(self._n, "set_vertical_distance_names")(
            _native_point_name(a), _native_point_name(b), float(value))
        return self

    def horizontal_distance(self, point_a: str, point_b: str, value: float) -> Sketch:
        """Signed horizontal offset ``b.x - a.x = value``."""
        kind_a, a = _classify_point(point_a)
        kind_b, b = _classify_point(point_b)
        if kind_a != "name" or kind_b != "name":
            raise ValueError("horizontal_distance needs two named points")
        _require(self._n, "set_horizontal_distance_names")(
            _native_point_name(a), _native_point_name(b), float(value))
        return self

    def concentric(self, curve_a: SketchCurve | str, curve_b: SketchCurve | str) -> Sketch:
        """Share a center (circles/arcs). Returns self."""
        _require(self._n, "set_concentric")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def fix(self, point: str | Point2Like) -> Sketch:
        """Pin a named point in the sketch plane. XY constants are ignored."""
        kind, value = _classify_point(point)
        if kind == "xy":
            return self
        if kind == "name":
            _require(self._n, "fix_point_named")(_native_point_name(value))
            return self
        raise ValueError("fix needs a named point")

    def length(self, curve: SketchCurve | str, value: float) -> Sketch:
        """Set line length. Returns self."""
        _require(self._n, "set_length")(_curve_index(self, curve), float(value))
        return self

    def radius(self, curve: SketchCurve | str, value: float) -> Sketch:
        """Set circle/arc radius. Returns self."""
        _require(self._n, "set_radius")(_curve_index(self, curve), float(value))
        return self

    def distance(self, point_a: str | Point2Like, point_b: str | Point2Like,
                 value: float) -> Sketch:
        """Distance between two points. Each is a name or (\"xy\", x, y)."""
        _dispatch_two_points(
            self._n,
            point_a,
            point_b,
            name_name=("set_distance_names", "set_distance_named"),
            name_xy="set_distance_name_xy",
            two_constants="distance needs at least one sketch point",
            extra=(float(value),),
        )
        return self

    def angle(self, curve_a: SketchCurve | str, curve_b: SketchCurve | str,
              degrees: float) -> Sketch:
        """Angle between two lines, or a line and an arc tangent, in degrees. Returns self."""
        _require(self._n, "set_angle_degrees")(
            _curve_index(self, curve_a), _curve_index(self, curve_b), float(degrees))
        return self

    def solve(self) -> Sketch:
        """Solve constraints, raising on nonconvergence. Returns self."""
        _require(self._n, "solve_constraints")()
        return self

    def eval_xy(self, point: str | Point2Like) -> vec2:
        """Sketch-plane XY of a named point (``line @ 1.000``, ``sk @ \"origin\"``)."""
        kind, value = _endpoint_value(point)
        if kind == "name":
            return _eval_named_point(self, value)
        if kind == "xy":
            return value
        raise ValueError("eval_xy needs a named point or XY pair")

    def offset(self, curves: EntitySelection, distance: float,
               side: Literal["both", "out", "in", "left", "right"] = "both",
               join: Literal["miter", "square", "round"] = "miter",
               end_cap: Literal["square", "butt", "round"] = "square",
               open_mode: Literal["network", "outline", "parallel"] | None = None) -> list[SketchCurve]:
        """Offset curves. Closed loops without forks use ``side='out'`` / ``'in'``.

        Returns sampled sketch curves named ``{source}@in_offset`` / ``{source}@out_offset``,
        outline end caps ``{source}@start_cap`` / ``{source}@end_cap``, and corner joins
        ``cap[a,b]`` from the two named curves they connect.

        ``side``:
          * ``both`` — thicken centerlines both ways (T-junctions union).
            ``distance`` is the half-width.
          * ``out`` / ``in`` — one-sided offset of a single closed loop (no forks).
            ``out`` grows the room (positive CAD convention for CCW).
          * ``left`` / ``right`` — one-sided offset of an open strip (left of travel).

        ``join`` is ``miter``, ``square``, or ``round``. ``end_cap`` is ``square``,
        ``butt``, or ``round`` (only for ``both`` / open outline).

        ``open_mode`` (optional): ``network`` (default for ``side='both'``),
        ``outline`` (both sides of a single strip plus end caps), or ``parallel``.
        """
        names = _curve_name_list(curves)
        joins = {"square": 0, "round": 1, "miter": 2}
        caps = {"round": 0, "butt": 1, "square": 2}
        sides = {
            "both": 0, "center": 0, "centre": 0,
            "out": 1, "outward": 1, "outside": 1,
            "in": 2, "inward": 2, "inside": 2,
            "left": 3,
            "right": 4,
        }
        modes = {"network": 1, "outline": 2, "parallel": 3}
        join_key = str(join).strip().lower()
        cap_key = str(end_cap).strip().lower()
        side_key = str(side).strip().lower()
        if join_key not in joins:
            raise ValueError("join must be miter, square, or round")
        if cap_key not in caps:
            raise ValueError("end_cap must be square, butt, or round")
        if side_key not in sides:
            raise ValueError("side must be both, out, in, left, or right")
        mode_code = 0
        if open_mode is not None:
            mode_key = str(open_mode).strip().lower()
            if mode_key not in modes:
                raise ValueError("open_mode must be network, outline, or parallel")
            mode_code = modes[mode_key]
        joined_names = "|".join(names)
        joined = ""
        styled = getattr(self._n, "offset_styled", None)
        if styled is not None and mode_code != 0:
            joined = styled(
                joined_names, float(distance), joins[join_key], caps[cap_key],
                sides[side_key], mode_code)
        else:
            fn = getattr(self._n, "offset_named", None)
            if fn is not None:
                joined = fn(joined_names, float(distance), joins[join_key], caps[cap_key], sides[side_key])
            elif sides[side_key] != 0:
                raise AttributeError(
                    "this camber wheel has no 'offset_named'; rebuild with Geo.Python/publish-wheel.ps1")
            else:
                joined = _require(self._n, "offset_network_named")(
                    joined_names, float(distance), joins[join_key], caps[cap_key])
        return _offset_result_curves(joined)

    def _try_drag_point(self, curve, point, xy):
        """Move a sketch handle (same roles as the overlay diamonds) and resolve constraints."""
        fn = getattr(self._n, "try_drag_sketch_point", None)
        if fn is None:
            return False
        x, y = _xy(xy)
        try:
            return int(fn(int(curve), int(point), float(x), float(y))) != 0
        except Exception:
            return False

    def _solved_actions(self):
        """Parsed 2D curve dump after solve: list of dicts (kind, name, points, …)."""
        actions = []
        try:
            text = _require(self._n, "dump_curves_2d")()
        except Exception:
            return actions
        return _parse_sketch_curve_dump(text)

    def add_rectangle_centered(self, center: Point2Like, size_x: float, size_y: float,
                               names: Sequence[EntityName] | None = None) -> Sketch:
        """Axis-aligned rectangle. `names` is (south, east, north, west) for the four sides."""
        cx, cy = _xy(center)
        sx, sy = float(size_x), float(size_y)
        if names:
            south, east, north, west = names
            named = getattr(self._n, "add_rectangle_centered_named", None)
            if named is not None:
                named(cx, cy, sx, sy, _name(south), _name(east), _name(north), _name(west))
                return self
            hx, hy = 0.5 * sx, 0.5 * sy
            bl = (cx - hx, cy - hy)
            br = (cx + hx, cy - hy)
            tr = (cx + hx, cy + hy)
            tl = (cx - hx, cy + hy)
            self.add_line(bl, br, name=south)
            self.add_line(br, tr, name=east)
            self.add_line(tr, tl, name=north)
            self.add_line(tl, bl, name=west)
            return self
        _require(self._n, "add_rectangle_centered")(cx, cy, sx, sy)
        return self

    def append_line(self, end: Point2Like) -> Sketch:
        """Plotter: line from the current pen to ``end``. Returns self."""
        x, y = _xy(end)
        _require(self._n, "append_line")(x, y)
        return self

    def set_start(self, point: Point2Like) -> Sketch:
        """Set the strip pen without drawing (plotter ``Append*`` methods)."""
        x, y = _xy(point)
        _require(self._n, "set_start_point")(x, y)
        return self

    def move_to(self, point: Point2Like) -> Sketch:
        """Start a new disconnected strip at ``point``."""
        x, y = _xy(point)
        _require(self._n, "move_to")(x, y)
        return self

    def append_line_horizontal(self, end_x: float) -> Sketch:
        """Plotter: horizontal line to sketch X = ``end_x``. Returns self."""
        _require(self._n, "append_line_horizontal")(float(end_x))
        return self

    def append_line_vertical(self, end_y: float) -> Sketch:
        """Plotter: vertical line to sketch Y = ``end_y``. Returns self."""
        _require(self._n, "append_line_vertical")(float(end_y))
        return self

    def append_arc_left(self, radius: float, angle: float | None = None) -> Sketch:
        """Tangential left turn of ``angle`` radians (default π/2) at ``radius``."""
        if angle is None:
            angle = 0.5 * math.pi
        _require(self._n, "append_arc_tangential_left")(float(radius), float(angle))
        return self

    def append_arc_right(self, radius: float, angle: float | None = None) -> Sketch:
        """Tangential right turn of ``angle`` radians (default π/2) at ``radius``."""
        if angle is None:
            angle = 0.5 * math.pi
        _require(self._n, "append_arc_tangential_right")(float(radius), float(angle))
        return self

    def add_spline(self, points: Sequence[Point2Like],
                   start_tangent: Point2Like | None = None,
                   end_tangent: Point2Like | None = None,
                   name: EntityName | None = None) -> SketchCurve:
        """Cubic Hermite spline through 2D points."""
        joined = _join_xy_points(points)
        stx, sty, etx, ety, has_s, has_e = _optional_tangents(start_tangent, end_tangent)
        index = self.curve_count
        _require(self._n, "add_cubic_hermite_spline")(joined, stx, sty, etx, ety, has_s, has_e)
        if name:
            self._try_name_last_curve(name)
        # Older native backends assign Hermite names and cannot rename them.
        # Return the real identifier, so downstream faces and edge references
        # never use a requested name that the kernel did not accept.
        curve_name = self._added_curve_name(None, "spline", index)
        return SketchCurve(curve_name, "spline")

    def add_sampled_curve(self, points: Sequence[Point2Like],
                          name: EntityName | None = None,
                          construction: bool = False) -> SketchCurve:
        """Add one curve represented by the supplied 2D samples."""
        joined = _join_xy_points(points)
        if len(joined.split("|")) < 2:
            raise ValueError("a sampled curve needs at least two points")
        index = self.curve_count
        _require(self._n, "add_sampled_curve")(joined, _name(name))
        curve_name = self._added_curve_name(name, "sampled", index)
        if construction:
            self.set_construction(curve_name, True)
        return SketchCurve(curve_name, "sampled")

    def add_involute(self, center: Point2Like, base_radius: float, t_start: float,
                     t_end: float, rotation: float = 0,
                     name: EntityName | None = None, max_deviation: float = -1) -> SketchCurve:
        """Circle involute as a cubic Hermite spline (same idea as NACA airfoils).

        ``t_start`` / ``t_end`` are unroll angles on the base circle. Knots follow
        the part ``max_deviation`` (or ``max_deviation=`` if given).
        """
        cx, cy = _xy(center)
        md = float(self._part.max_deviation) if max_deviation is None or float(max_deviation) <= 0 else float(max_deviation)
        index = self.curve_count
        _require(self._n, "add_involute")(
            cx, cy, float(base_radius), float(t_start), float(t_end), float(rotation), md)
        curve_name = self._added_curve_name(name, "involute", index)
        if name:
            self._try_name_last_curve(name)
        return SketchCurve(curve_name, "spline")

    def add_involute_gear(
            self, center: Point2Like, module: float, teeth: int,
            pressure_angle: float | None = None, addendum: float = 1.0,
            dedendum: float = 1.25, max_deviation: float = -1) -> Sketch:
        """External involute spur gear. Flanks are Hermite fits; tip and root are arcs.

        Center the gear on the sketch origin before a twist extrude. Cut the bore
        as a straight cylinder afterwards. ``pressure_angle`` is radians (default 20°).
        """
        cx, cy = _xy(center)
        if pressure_angle is None:
            pressure_angle = math.radians(20.0)
        md = float(self._part.max_deviation) if max_deviation is None or float(max_deviation) <= 0 else float(max_deviation)
        _require(self._n, "add_involute_gear")(
            cx, cy, float(module), int(teeth), float(pressure_angle),
            float(addendum), float(dedendum), md)
        return self

    def add_involute_internal_gear(
            self, center: Point2Like, module: float, teeth: int,
            pressure_angle: float | None = None, addendum: float = 1.0,
            dedendum: float = 1.25, max_deviation: float = -1) -> Sketch:
        """Inner hole of an internal ring gear (external involute, addendum/dedendum swapped).

        Extrude and subtract from a disc so the cut-outs match a mating pinion.
        ``pressure_angle`` is radians (default 20°).
        """
        cx, cy = _xy(center)
        if pressure_angle is None:
            pressure_angle = math.radians(20.0)
        md = float(self._part.max_deviation) if max_deviation is None or float(max_deviation) <= 0 else float(max_deviation)
        _require(self._n, "add_involute_internal_gear")(
            cx, cy, float(module), int(teeth), float(pressure_angle),
            float(addendum), float(dedendum), md)
        return self

    def append_spline(self, points: Sequence[Point2Like],
                      start_tangent: Point2Like | None = None,
                      end_tangent: Point2Like | None = None) -> Sketch:
        """Hermite spline from the current strip end through ``points``."""
        joined = _join_xy_points(points)
        stx, sty, etx, ety, has_s, has_e = _optional_tangents(start_tangent, end_tangent)
        _require(self._n, "append_cubic_hermite_spline")(joined, stx, sty, etx, ety, has_s, has_e)
        return self

    def _try_name_last_curve(self, name):
        dumped = self._solved_actions()
        if not dumped:
            return
        last = dumped[-1]
        if last.get("name"):
            return
        fn = getattr(self._n, "set_curve_name", None)
        if fn is None:
            return
        try:
            fn(len(dumped) - 1, _name(name))
        except Exception:
            pass

    def repeat_circular(self, curves: EntitySelection, center: Point2Like, count: int,
                        total_angle: float | None = None,
                        include_original: bool = False) -> list[SketchCurve]:
        """Rotational copies about ``center``. Skips the 0° instance unless ``include_original``."""
        names = _curve_name_list(curves)
        cx, cy = _xy(center)
        if total_angle is None:
            total_angle = 2.0 * math.pi
        joined = _require(self._n, "repeat_circular_named")(
            "|".join(names), cx, cy, int(count), float(total_angle), 1 if include_original else 0)
        return _offset_result_curves(joined)

    def repeat_grid(self, curves: EntitySelection, count_x: int, count_y: int,
                    step_x: Point2Like, step_y: Point2Like,
                    include_original: bool = False) -> list[SketchCurve]:
        """Grid copies. Skips the (0, 0) cell unless ``include_original``."""
        names = _curve_name_list(curves)
        sxx, sxy = _xy(step_x)
        syx, syy = _xy(step_y)
        joined = _require(self._n, "repeat_grid_named")(
            "|".join(names), int(count_x), int(count_y), sxx, sxy, syx, syy,
            1 if include_original else 0)
        return _offset_result_curves(joined)

    def add_naca4(self, code: str | int, leading_edge: Point2Like, chord_length: float,
                  chord_angle: float = 0.0, samples_per_side: int = 36,
                  analytic_end_tangents: bool = True,
                  te_trim: float = _NACA_TE_TRIM) -> Sketch:
        """Closed NACA 4-digit airfoil. ``code`` is ``2412`` or the integer 2412."""
        lx, ly = _xy(leading_edge)
        ae = 1 if analytic_end_tangents else 0
        if isinstance(code, str):
            fn = getattr(self._n, "add_naca_4_digit_airfoil_label", None) or getattr(
                self._n, "add_naca4_digit_airfoil_label", None)
            if fn is None:
                _require(self._n, "add_naca_4_digit_airfoil_label")
            fn(code, lx, ly, float(chord_length), float(chord_angle), int(samples_per_side), ae, float(te_trim))
        else:
            fn = getattr(self._n, "add_naca_4_digit_airfoil", None) or getattr(
                self._n, "add_naca4_digit_airfoil", None)
            if fn is None:
                _require(self._n, "add_naca_4_digit_airfoil")
            fn(int(code), lx, ly, float(chord_length), float(chord_angle), int(samples_per_side), ae, float(te_trim))
        return self

    def show(self, title: str = "Camber") -> Sketch:
        """Open the sketch viewer on this sketch's plane (default planes: xy, yz, zx)."""
        from .sketch_ui import show_sketch
        show_sketch(self, title=title)
        return self

    def __matmul__(self, param):
        """`sk @ \"origin\"` is the sketch origin; `sk @ \"x\"` / `sk @ \"y\"` are the axes."""
        from . import naming as _naming
        if not isinstance(param, str):
            raise TypeError("sketch @ expects \"origin\", \"x\", or \"y\"")
        key = param.strip().lower()
        if key == _naming.ORIGIN:
            local = _naming.ORIGIN
        elif _naming.sketch_axis_index(key) == 0:
            local = _naming.AXIS_X
        elif _naming.sketch_axis_index(key) == 1:
            local = _naming.AXIS_Y
        else:
            raise TypeError("sketch @ expects \"origin\", \"x\", or \"y\"")
        name = (self.name or "").strip()
        if not name:
            raise ValueError("sketch @ needs a named sketch")
        return _naming.qualify(name, local)

    def __repr__(self):
        return "Sketch(name={0!r})".format(self.name)


def _offset_result_curves(joined):
    if joined is None:
        return []
    if not isinstance(joined, str):
        joined = str(joined)
    result = []
    for name in joined.split("|"):
        name = name.strip()
        if name:
            result.append(SketchCurve(name, "sampled"))
    return result


def _curve_name_list(curves):
    if isinstance(curves, (str, SketchCurve)):
        curves = (curves,)
    names = []
    for curve in curves:
        if isinstance(curve, SketchCurve):
            names.append(curve.name)
        else:
            names.append(str(curve).strip())
    if not names:
        raise ValueError("needs at least one curve")
    return names


def _join_xy_points(points):
    parts = []
    for point in points:
        x, y = _xy(point)
        parts.append("{0},{1}".format(x, y))
    if not parts:
        raise ValueError("needs at least one point")
    return "|".join(parts)


def _optional_tangents(start_tangent, end_tangent):
    stx = sty = etx = ety = 0.0
    has_s = has_e = 0
    if start_tangent is not None:
        stx, sty = _xy(start_tangent)
        has_s = 1
    if end_tangent is not None:
        etx, ety = _xy(end_tangent)
        has_e = 1
    return stx, sty, etx, ety, has_s, has_e


def _parse_sketch_curve_dump(text):
    """Parse SketchCurveDump text into geometry actions. Accepts named and legacy rows."""
    actions = []
    if not text:
        return actions
    for raw in (text or "").splitlines():
        fields = raw.split("\t")
        kind = fields[0] if fields else ""
        if kind == "L":
            name, nums = _dump_name_and_nums(fields, 4)
            if nums is None:
                continue
            actions.append({
                "kind": "line",
                "name": name,
                "p0": (nums[0], nums[1]),
                "p1": (nums[2], nums[3]),
            })
        elif kind == "C":
            name, nums = _dump_name_and_nums(fields, 3)
            if nums is None:
                continue
            actions.append({
                "kind": "circle",
                "name": name,
                "center": (nums[0], nums[1]),
                "radius": nums[2],
            })
        elif kind == "A":
            name, nums = _dump_name_and_nums(fields, 6)
            if nums is None:
                continue
            actions.append({
                "kind": "arc",
                "name": name,
                "start": (nums[0], nums[1]),
                "mid": (nums[2], nums[3]),
                "end": (nums[4], nums[5]),
            })
    return actions


def _dump_name_and_nums(fields, count):
    if len(fields) == count + 2:
        try:
            return fields[1], [float(v) for v in fields[2:]]
        except ValueError:
            return "", None
    if len(fields) == count + 1:
        try:
            return "", [float(v) for v in fields[1:]]
        except ValueError:
            return "", None
    return "", None


class ProjectedSketch(object):
    """Result of Sketch.project_onto: named 3D polylines with surface normals."""

    def __init__(self, native: object, part: Part) -> None:
        self._n = native
        self._part = part

    @api_group("Build solids")
    def extrude(self, height: float, name: EntityName | None = None) -> Solid:
        """Extrude along stored surface normals; negative goes into the solid."""
        return self._part._extrude_projected(self, height, name)

    @property
    def name(self) -> EntityName:
        """Kernel name of the projected sketch."""
        return self._n.name

    @property
    def curve_count(self) -> int:
        """Number of projected 3D polylines."""
        return int(self._n.curve_count)

    def curve_name_at(self, index: int) -> EntityName:
        """Name of the polyline at ``index``."""
        return _invoke(self._n, "curve_name_at", int(index))

    def __repr__(self):
        return "ProjectedSketch(name={0!r}, curves={1})".format(self.name, self.curve_count)


class _MeshBody(object):
    """Shared native mesh handle for the public Solid and Surface types."""

    def __init__(self, native, part):
        self._n = native
        self._part = part

    @property
    def name(self) -> EntityName:
        """Kernel mesh name."""
        return self._n.name

    @property
    def triangle_count(self) -> int:
        """Number of triangles in the kernel mesh."""
        return self._n.triangle_count

    @property
    def is_volume(self) -> bool:
        """True if the kernel treats this mesh as a closed volume."""
        flag = getattr(self._n, "is_volume", None)
        if flag is None:
            return True
        return bool(flag)

    @property
    def patch_names(self) -> tuple[str, ...]:
        """Canonical patch names, identical to the renderer's face names."""
        return tuple(_require(self._n, "patch_name_at")(i) for i in range(int(self._n.patch_count)))

    def patch_frame_at(self, index: int) -> Frame:
        """Area-centroid frame of a tessellated patch (normal from its winding)."""
        return Frame._from_native(_require(self._n, "patch_frame_at")(int(index)))

    @property
    def curve_names(self) -> tuple[str, ...]:
        """Canonical mesh-edge curve names, accepted by fillet and chamfer."""
        return tuple(_require(self._n, "curve_name_at")(i) for i in range(int(self._n.curve_count)))

    @property
    def point_names(self) -> tuple[str, ...]:
        """Canonical displayed edge-anchor names (start, midpoint, end, and closed-loop quarters)."""
        return tuple(_require(self._n, "point_name_at")(i) for i in range(int(self._n.point_count)))
    def mesh(self) -> tuple[list[vec3], list[tuple[int, int, int]]]:
        """Raw kernel triangles: ``(points, triangles)`` as ``vec3`` and ``(i, j, k)``."""
        from .geom import _unpack_indexed3
        return _unpack_indexed3(_require(self._n, "pack_mesh")())

    def is_watertight(self) -> bool:
        """True if every edge is shared by exactly two triangles."""
        return bool(_require(self._n, "is_watertight")())

    def save_stl(self, path: str, binary: bool = True) -> None:
        """Write STL. ``binary=False`` is ASCII."""
        _invoke(self._part._n, "save_stl", self._n, path, 1 if binary else 0)

    def save_step(self, path: str) -> None:
        """Write STEP (faceted)."""
        _invoke(self._part._n, "save_step", self._n, path)

    def save_iges(self, path: str) -> None:
        """Write IGES (faceted)."""
        _invoke(self._part._n, "save_iges", self._n, path)

    def save_off(self, path: str) -> None:
        """Write OFF mesh."""
        _require(self._part._n, "save_off")(self._n, path)

    def save_obj(self, path: str) -> None:
        """Write Wavefront OBJ."""
        _require(self._part._n, "save_wavefront_obj")(self._n, path)

    def save_usda(self, path: str) -> None:
        """Write USDA triangle mesh (per-vertex normals, UVs, named face subsets)."""
        _require(self._part._n, "save_usda")(self._n, path)

    def show(self, title: str = "Camber") -> None:
        """Open the 3D viewer on this solid."""
        from .view import show as _show
        _show(self, title=title)

    def __repr__(self):
        return "MeshBody(name={0!r}, triangle_count={1})".format(self.name, self.triangle_count)


class Solid(_MeshBody):
    """Closed triangle volume owned by a Part.

    Operators are ``+`` union, ``-`` subtract, and ``&`` intersect.
    """

    def __init__(self, native, part):
        super(Solid, self).__init__(native, part)
        if not self.is_volume:
            raise ValueError("Solid requires a closed volume mesh")

    @api_group("Edge treatment")
    def fillet(self, edges: EntitySelection, radius: float,
               name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Round named edges."""
        return self._part._fillet(self, edges, radius, name, max_deviation)

    @api_group("Edge treatment")
    def chamfer(self, edges: EntitySelection, distance: float,
                name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Bevel named edges."""
        return self._part._chamfer(self, edges, distance, name, max_deviation)

    @api_group("Hollow and draft")
    def shell(self, thickness: float, faces: EntitySelection | None = None,
              name: EntityName | None = None, max_deviation: float = -1,
              outward: bool = False, join: Literal["sharp", "round"] = "sharp") -> Solid:
        """Hollow inward or outward; optionally remove named opening faces."""
        return self._part._shell(self, thickness, faces, name, max_deviation, outward, join)

    @api_group("Hollow and draft")
    def draft_faces(self, faces: EntitySelection, neutral_frame: Frame, angle: float, *,
                    pull_direction: Point3Like | None = None,
                    name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Draft named prism side faces about a neutral cap plane."""
        return self._part._draft_faces(self, faces, neutral_frame, angle,
                                       pull_direction=pull_direction, name=name,
                                       max_deviation=max_deviation)

    @api_group("Add material")
    def rib(self, path: Sequence[Point2Like], thickness: float, to: float, *,
            frame: Frame | None = None, name: EntityName | None = None,
            max_deviation: float = -1) -> Solid:
        """Add a constant-width polyline rib to this solid."""
        return self._part._rib(self, path, thickness, to, frame=frame,
                               name=name, max_deviation=max_deviation)

    @api_group("Holes")
    def hole(self, mouth: Frame, diameter: float, depth: float | None = None, *,
             name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Drill a flat-bottom hole; no depth means through."""
        return self._part._hole(self, mouth, diameter, depth, name=name,
                                max_deviation=max_deviation)

    @api_group("Holes")
    def counterbore_hole(self, mouth: Frame, diameter: float, counterbore_diameter: float,
                         counterbore_depth: float, depth: float | None = None, *,
                         name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Drill a hole with a flat-bottom counterbore."""
        return self._part._counterbore_hole(self, mouth, diameter, counterbore_diameter,
                                            counterbore_depth, depth, name=name,
                                            max_deviation=max_deviation)

    @api_group("Holes")
    def countersink_hole(self, mouth: Frame, diameter: float, countersink_diameter: float,
                         included_angle: float, depth: float | None = None, *,
                         name: EntityName | None = None, max_deviation: float = -1) -> Solid:
        """Drill a hole with a conical countersink; angle is in radians."""
        return self._part._countersink_hole(self, mouth, diameter, countersink_diameter,
                                            included_angle, depth, name=name,
                                            max_deviation=max_deviation)

    @api_group("Transforms")
    def pattern_linear(self, count: int, step: Point3Like,
                       name: EntityName | None = None) -> list[Solid]:
        """Return independent copies at equal XYZ steps, including this seed."""
        return self._part._pattern_linear(self, count, step, name)

    @api_group("Transforms")
    def pattern_circular(self, count: int,
                         axis: Curve3D | tuple[Point3Like, Point3Like] | None = None,
                         angle: float = 2*math.pi,
                         name: EntityName | None = None) -> list[Solid]:
        """Return copies rotated around an axis, including this seed."""
        return self._part._pattern_circular(self, count, axis, angle, name)

    @api_group("Transforms")
    def mirror(self, plane: Frame | None = None,
               name: EntityName | None = None) -> Solid:
        """Return a reflected copy across a Frame's XY plane."""
        return self._part._mirror(self, plane, name)

    @api_group("Inspect")
    def raycast(self, origin: Point3Like, direction: Point3Like) -> object | None:
        """Return the nearest mesh hit from origin along direction, or None."""
        return self._part._raycast(self, origin, direction)

    @api_group("Inspect")
    def face_surface(self, patch_name: EntityName,
                     name: EntityName | None = None) -> Surface:
        """Extract a named face as an oriented open surface."""
        return self._part._face_surface(self, patch_name, name)

    def __add__(self, other):
        """Boolean union. Result keeps this solid's name."""
        return self._part.union(self, other, name=self.name)

    def located(self, location: Point3Like) -> Solid:
        """Return this solid translated to a world-space location."""
        target = _xyz(location)
        copies = self.pattern_linear(2, target)
        return copies[1]

    def __sub__(self, other):
        """Boolean subtraction (this minus other). Result keeps this solid's name."""
        return self._part.subtract(self, other, name=self.name)

    def __and__(self, other):
        """Boolean intersection. Result keeps this solid's name."""
        return self._part.intersect(self, other, name=self.name)

    def trim(self, surface: Surface, *, side: Literal["normal", "opposite"] = "normal",
             name: EntityName | None = None) -> Solid:
        """Trim this closed solid by an open Surface.

        ``side`` is ``"normal"`` or ``"opposite"`` relative to the surface
        triangle normals.
        """
        return self._part._trim_by_surface(self, surface, side=side,
                                          name=self.name if name is None else name)

    def signed_volume(self) -> float:
        """Signed tetrahedron volume. Negative means inward orientation."""
        return float(_require(self._n, "signed_volume")())

    def volume(self) -> float:
        """Absolute volume of this closed solid."""
        return abs(self.signed_volume())

    def __repr__(self):
        return "Solid(name={0!r}, triangle_count={1})".format(self.name, self.triangle_count)


class Surface(_MeshBody):
    """Open triangle surface owned by a Part.

    Surfaces share mesh inspection, naming, display, and export with solids.
    Their triangle normals define the retained side when trimming a Solid.
    """

    def __init__(self, native, part):
        super(Surface, self).__init__(native, part)
        if self.is_volume:
            raise ValueError("Surface requires an open mesh")

    @api_group("Hollow and draft")
    def thicken(self, thickness: float, *, both_sides: bool = False,
                name: EntityName | None = None) -> Solid:
        """Create a solid from this surface.

        Positive thickness follows the authored triangle normals and negative
        thickness goes against them. ``both_sides=True`` applies the absolute
        distance on each side, so the total wall thickness is twice that value.
        """
        return self._part._thicken(self, thickness, both_sides=both_sides, name=name)

    @api_group("Surface modeling")
    def trim(self, cutter: Surface, *, side: Literal["normal", "opposite"] = "normal",
             name: EntityName | None = None) -> Surface:
        """Trim this sheet with another sheet using the exact CSG resolver."""
        if not isinstance(cutter, Surface) or cutter._part is not self._part:
            raise TypeError("cutter must be a Surface from this Part")
        if side == "normal":
            keep_normal = 1
        elif side == "opposite":
            keep_normal = 0
        else:
            raise ValueError("side must be 'normal' or 'opposite'")
        native = _require(self._part._n, "trim_surface")(
            self._n, cutter._n, keep_normal, _name(name))
        return Surface(native, self._part)

    @api_group("Surface modeling")
    def split(self, cutter: Surface, *, name: EntityName | None = None) -> list[Surface]:
        """Return the normal and opposite sides of an exact surface trim."""
        if not isinstance(cutter, Surface) or cutter._part is not self._part:
            raise TypeError("cutter must be a Surface from this Part")
        native = _require(self._part._n, "split_surface")(
            self._n, cutter._n, _name(name))
        return [Surface(native.get(i), self._part) for i in range(native.count)]

    @api_group("Surface modeling")
    def intersection_curves(self, other: Surface | Solid, *,
                            name: EntityName | None = None) -> list[Curve3D]:
        """Return connected 3D intersection curves with another surface or solid.

        Curves follow the current triangle meshes exactly. Curved source geometry
        is therefore limited by each body's tessellation tolerance. Coplanar
        overlap has no unique 1D intersection and is omitted.
        """
        if not isinstance(other, (Surface, Solid)) or other._part is not self._part:
            raise TypeError("other must be a Surface or Solid from this Part")
        native = _require(self._part._n, "intersection_curves")(
            self._n, other._n, _name(name))
        return [Curve3D(native.get(i), self._part) for i in range(int(native.count))]

    @api_group("Surface modeling")
    def cap_planar_boundaries(self, *, make_solid: bool = True,
                              name: EntityName | None = None) -> Surface | Solid:
        """Fill exact planar boundary loops, returning a Solid by default."""
        native = _require(self._part._n, "cap_planar_boundaries")(
            self._n, int(bool(make_solid)), _name(name))
        return _body(native, self._part)

    def __repr__(self):
        return "Surface(name={0!r}, triangle_count={1})".format(self.name, self.triangle_count)


def _body(native, part):
    """Wrap a native mesh in its public volume or surface type."""
    return Solid(native, part) if bool(native.is_volume) else Surface(native, part)
