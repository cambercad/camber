import math
import json
import os
import sys
from collections import namedtuple

from .vec import _xy, _xyz, vec2, vec3

BOOLEAN_UNION = 0  # a + b / Part.union
BOOLEAN_DIFFERENCE = 1  # a - b / Part.cut
BOOLEAN_INTERSECT = 2  # a & b / Part.intersect
BOOLEAN_RESOLVE = 3
BOOLEAN_NO_OP_INTERSECTION_CONTOUR_ONLY = 4

LOFT_STYLE_RULED = 0
LOFT_STYLE_SMOOTH_CATMULL_ROM = 1
LOFT_STYLE_HERMITE = 2

_NACA_TE_TRIM = 0.01
_TYPES = None


def _env_flag(name, default=True):
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


# One console line before each native call. Opt out with set_progress_log(False)
# or CAMBER_PROGRESS=0.
_progress_log = _env_flag("CAMBER_PROGRESS", True)


def set_progress_log(enabled):
    """Print ``camber: <call>`` before each native API call. On by default."""
    global _progress_log
    _progress_log = bool(enabled)


def progress_log_enabled():
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

    def __init__(self, origin=None, x=None, y=None, z=None):
        """World origin plus orthonormal axes. Defaults to identity at (0,0,0)."""
        self.origin = vec3(0, 0, 0) if origin is None else vec3(origin)
        self.x = vec3(1, 0, 0) if x is None else vec3(x)
        self.y = vec3(0, 1, 0) if y is None else vec3(y)
        self.z = vec3(0, 0, 1) if z is None else vec3(z)

    @staticmethod
    def from_plane(origin, normal, x, y):
        """Same pose as Curves.Plane3D(origin, normal, x, y).GetCoordinateSystem()."""
        return Frame(origin, x=x, y=y, z=normal)

    def offset(self, delta):
        """Translate origin by ``delta`` (vec3 or 3-tuple). Axes unchanged."""
        return Frame(self.origin + delta, self.x, self.y, self.z)

    def to_local(self, other):
        """Express ``other`` in this frame. Returns a Frame."""
        return Frame._from_native(self._native().to_local(_as_frame(other)._native()))

    def to_global(self, local):
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


def frame_from_axis(origin, axis="z"):
    """Pose whose local +Z is world +axis (CreateCylinder extrudes along local +Z)."""
    origin = vec3(origin)
    axis = (axis or "z").lower()
    if axis == "x":
        return Frame(origin, x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
    if axis == "y":
        return Frame(origin, x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0))
    return Frame(origin)


class Curve(object):
    """3D guide curve (Line3D / Helix3D / Circle3D / Arc3D / Spiral3D)."""

    def __init__(self, native):
        self._n = native

    @property
    def name(self):
        """Kernel entity name."""
        return self._n.name

    @staticmethod
    def line(start, end, name=None):
        """World-space line from ``start`` to ``end`` (vec3 or 3-tuple)."""
        x0, y0, z0 = _xyz(start)
        x1, y1, z1 = _xyz(end)
        return Curve(_require(_native_mod()["NativeCurve"], "line")(x0, y0, z0, x1, y1, z1, _name(name)))

    @staticmethod
    def hermite(points, tangent_directions, name=None):
        """Cubic guide through points, with one nonzero tangent direction per knot.

        Direction magnitudes are ignored; adjacent chord lengths set derivative
        magnitudes. Repeat the first point and direction to close the curve.
        The closed seam is C1, including unequal first/last chord lengths.
        Cubic interpolation does not guarantee C2 acceleration continuity.
        """
        from .geom import _pack_points3
        return Curve(_require(_native_mod()["NativeCurve"], "hermite")(
            _pack_points3(points), _pack_points3(tangent_directions), _name(name)))

    def point(self, u):
        """Point at normalized parameter u in [0,1], not normalized arc length."""
        return vec3(tuple(map(float, _require(self._n, "point")(float(u)).split())))

    def tangent(self, u):
        """Unit tangent at normalized parameter u in [0,1]."""
        return vec3(tuple(map(float, _require(self._n, "tangent")(float(u)).split())))

    @staticmethod
    def helix(origin, x_axis, y_axis, radius, pitch, turns, right_handed=True, name=None):
        """Helix in the plane of ``x_axis``/``y_axis``; axis is their cross. ``pitch`` is z per turn."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve(_require(_native_mod()["NativeCurve"], "helix")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(radius), float(pitch), float(turns), 1 if right_handed else 0, _name(name)))

    @staticmethod
    def circle(origin, x_axis, y_axis, radius, name=None):
        """Full circle in the x/y plane of the given axes."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve(_require(_native_mod()["NativeCurve"], "circle")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz, float(radius), _name(name)))

    @staticmethod
    def arc(origin, x_axis, y_axis, radius, start_angle, sweep_angle, name=None):
        """Circular arc. Angles in radians, from ``x_axis`` toward ``y_axis``."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve(_require(_native_mod()["NativeCurve"], "arc")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(radius), float(start_angle), float(sweep_angle), _name(name)))

    @staticmethod
    def spiral(origin, x_axis, y_axis, start_radius, end_radius, z_per_turn, turns, right_handed=True, name=None):
        """Planar spiral with optional z advance (``z_per_turn``). Radii in the x/y plane."""
        ox, oy, oz = _xyz(origin)
        xx, xy, xz = _xyz(x_axis)
        yx, yy, yz = _xyz(y_axis)
        return Curve(_require(_native_mod()["NativeCurve"], "spiral")(
            ox, oy, oz, xx, xy, xz, yx, yy, yz,
            float(start_radius), float(end_radius), float(z_per_turn), float(turns),
            1 if right_handed else 0, _name(name)))

    def __repr__(self):
        return "Curve(name={0!r})".format(self.name)


class LoftOptions(object):
    """GeoAPI LoftOptions. Enum fields are ints (see LOFT_STYLE_*)."""

    def __init__(self, native=None, *, correspondence=None):
        """Empty options, or wrap an existing native LoftOptions."""
        NativeLoftOptions = _native_mod()["NativeLoftOptions"]
        self._n = native if native is not None else NativeLoftOptions()
        if correspondence is not None:
            self.correspondence = correspondence

    @property
    def correspondence(self):
        """Section matching: 'arc_length', 'uniform', 'features', or 'vertices'.

        'vertices' matches ordered polygon corners; all sections must have the
        same vertex count. Sharp polygon corners retain separate face normals.
        """
        return ("arc_length", "uniform", "features", "vertices")[self._n.correspondence_mode]

    @correspondence.setter
    def correspondence(self, value):
        choices = ("arc_length", "uniform", "features", "vertices")
        if value not in choices:
            raise ValueError("correspondence must be one of " + ", ".join(choices))
        self._n.correspondence_mode = choices.index(value)

    @staticmethod
    def propeller_blade():
        """Preset loft options for a propeller blade (smooth + robust caps)."""
        return LoftOptions(_require(_native_mod()["NativeLoftOptions"], "propeller_blade")())

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


def _constraint_entities(*datums):
    names = []
    for datum in datums:
        if datum is None:
            continue
        name = getattr(datum, "entity", None) or ""
        if name:
            names.append(str(name))
    return names


def _entity_short(datum):
    ent = getattr(datum, "entity", None) or ""
    if ent.endswith(":"):
        return ent[:-1]
    i = ent.find(":")
    if i >= 0:
        return ent[i + 1:] or ent
    ref = getattr(datum, "reference", None) or ""
    if ref:
        return str(ref)
    part = getattr(datum, "_part", None)
    return getattr(part, "name", "") if part is not None else ""


class AssemblyPart(object):
    """An instance of a Solid in an Assembly (pose + named datums)."""
    def __init__(self, native):
        self._n = native

    @property
    def name(self):
        """Instance name."""
        return self._n.name

    @property
    def pose(self):
        """Translation in the owning assembly as ``(x, y, z)``."""
        return (self._n.pose_x, self._n.pose_y, self._n.pose_z)

    def axis(self, reference):
        """Axis datum from a named entity on this part (edge/axis string)."""
        return AssemblyAxisDatum(_invoke(self._n, "add_axis_datum", str(reference)), self, str(reference))

    def point(self, reference):
        """Point datum from a named entity on this part."""
        return AssemblyPointDatum(_invoke(self._n, "add_point_datum", str(reference)), self, str(reference))

    def plane(self, reference):
        """Plane datum from a named entity on this part."""
        return AssemblyPlaneDatum(_invoke(self._n, "add_plane_datum", str(reference)), self, str(reference))

    def axis_at(self, point, direction):
        """Axis through ``point`` along ``direction`` in part-local coordinates."""
        px, py, pz = _xyz(point)
        dx, dy, dz = _xyz(direction)
        return AssemblyAxisDatum(_invoke(self._n, "add_axis_datum_at", px, py, pz, dx, dy, dz), self)

    def point_at(self, point):
        """Point datum in part-local coordinates."""
        px, py, pz = _xyz(point)
        return AssemblyPointDatum(_invoke(self._n, "add_point_datum_at", px, py, pz), self)

    def plane_at(self, origin, normal):
        """Plane datum at ``origin`` with ``normal``, both in part-local coordinates."""
        px, py, pz = _xyz(origin)
        nx, ny, nz = _xyz(normal)
        return AssemblyPlaneDatum(_invoke(self._n, "add_plane_datum_at", px, py, pz, nx, ny, nz), self)


class AssemblyOccurrence(object):
    """Rigid instance of a nested Assembly inside a parent Assembly."""

    def __init__(self, native, parent=None):
        self._n = native
        self._parent = parent

    @property
    def name(self):
        """Nested assembly name."""
        return self._n.name

    @property
    def pose(self):
        """Translation of this occurrence in the parent assembly as ``(x, y, z)``."""
        return (self._n.pose_x, self._n.pose_y, self._n.pose_z)

    @property
    def assembly(self):
        """The nested Assembly (its parts, mates, and further sub-assemblies)."""
        part = self._parent._part if self._parent is not None else None
        return Assembly(_invoke(self._n, "get_child"), part)

    @property
    def parts(self):
        """Direct parts of the nested assembly."""
        count = int(self._n.part_count)
        return [AssemblyPart(_invoke(self._n, "get_part", i)) for i in range(count)]

    @property
    def subassemblies(self):
        """Direct sub-assemblies of the nested assembly."""
        count = int(self._n.sub_assembly_count)
        nested_parent = self.assembly
        return [
            AssemblyOccurrence(_invoke(self._n, "get_sub_assembly", i), nested_parent)
            for i in range(count)
        ]

    def __repr__(self):
        return "AssemblyOccurrence({0!r})".format(self.name)


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


class AssemblySolveResult(namedtuple("AssemblySolveResult", "converged sum_squared_error num_parameters num_equations message characteristic_length mates")):
    """Immutable solver outcome and per-mate errors at the resulting pose.

    Parameter/equation counts describe the solver system, not remaining degrees
    of freedom. Contact mates have their own regularized residual tolerance.
    ``converged`` is the solver outcome; ``unsatisfied`` separately checks every
    equality mate at its stricter tolerance, even in a regularized contact solve.
    """
    __slots__ = ()

    @property
    def unsatisfied(self):
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
    """Rigid-part assembly solved by C# Geo Assembly constraints.

    Parts and nested assemblies can be mixed: ``add_part`` places a Solid,
    ``add_subassembly`` places another Assembly (which may itself contain
    parts and sub-assemblies). Nested internals stay rigid; parent mates may
    still use datums on nested parts.
    """

    def __init__(self, native, part=None):
        self._n = native
        self._part = part
        self.constraints = []

    @property
    def name(self):
        """Assembly instance name."""
        return self._n.name

    @property
    def solve_after_every_constraint(self):
        """If True, the kernel solves after each constraint."""
        return bool(self._n.solve_after_every_constraint)

    @solve_after_every_constraint.setter
    def solve_after_every_constraint(self, value):
        self._n.solve_after_every_constraint = 1 if value else 0

    def interferences(self, *, min_volume=0):
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

    def add_part(self, solid, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        """Place ``solid`` at ``position`` with quaternion ``orientation`` (x,y,z,w). Returns AssemblyPart."""
        px, py, pz = _xyz(position)
        if len(orientation) != 4:
            raise ValueError("orientation must be quaternion (x, y, z, w)")
        qx, qy, qz, qw = orientation
        return AssemblyPart(_invoke(
            self._n, "add_part",
            solid._n, px, py, pz, float(qx), float(qy), float(qz), float(qw)))

    def pattern_linear(self, seed, count, step):
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

    def pattern_circular(self, seed, count, axis=None, angle=2*math.pi):
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

    def mirror(self, seed, plane=None, name=None):
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

    def add_subassembly(self, assembly, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        """Place ``assembly`` as a rigid child. Returns AssemblyOccurrence.

        The child may contain parts and further sub-assemblies. Solve the child
        first; parent mates may use datums created on nested parts.
        """
        px, py, pz = _xyz(position)
        if len(orientation) != 4:
            raise ValueError("orientation must be quaternion (x, y, z, w)")
        qx, qy, qz, qw = orientation
        return AssemblyOccurrence(_invoke(
            self._n, "add_sub_assembly",
            assembly._n, px, py, pz, float(qx), float(qy), float(qz), float(qw)), self)

    @property
    def parts(self):
        """Parts added with ``add_part`` on this assembly (not nested leaves)."""
        count = int(self._n.part_count)
        return [AssemblyPart(_invoke(self._n, "get_part", i)) for i in range(count)]

    @property
    def subassemblies(self):
        """Child assemblies added with ``add_subassembly``."""
        count = int(self._n.sub_assembly_count)
        return [
            AssemblyOccurrence(_invoke(self._n, "get_sub_assembly", i), self)
            for i in range(count)
        ]

    def world_pose(self, part):
        """Translation of a direct or nested part in this assembly's frame."""
        return (
            _invoke(self._n, "get_world_pose_x", part._n),
            _invoke(self._n, "get_world_pose_y", part._n),
            _invoke(self._n, "get_world_pose_z", part._n),
        )

    def _record(self, kind, label, entities):
        self.constraints.append({"kind": kind, "label": label, "entities": list(entities or [])})

    def fix(self, part):
        """Lock an AssemblyPart or AssemblyOccurrence in world (ground)."""
        if isinstance(part, AssemblyOccurrence):
            _invoke(self._n, "fix_sub_assembly", part._n)
            self._record("FixPart", "Fix " + part.name, [part.name + ":"])
            return
        _invoke(self._n, "fix_part", part._n)
        self._record("FixPart", "Fix " + part.name, [part.name + ":"])

    def coincident(self, a, b, opposite_normals=None):
        """Coincide two matching datums (point/point, axis/axis, plane/plane)."""
        if isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            if opposite_normals is None:
                _invoke(self._n, "set_coincident_planes", a._n, b._n)
                kind = "CoincidentPlanes"
                label = "Coincident"
            else:
                _invoke(self._n, "set_coincident_planes_oriented",
                    a._n, b._n, 1 if opposite_normals else 0)
                kind = "CoincidentPlanes"
                label = "Coincident oriented"
        elif isinstance(a, AssemblyAxisDatum) and isinstance(b, AssemblyAxisDatum):
            if opposite_normals is not None:
                raise TypeError("opposite_normals applies only to plane datums")
            _invoke(self._n, "set_coincident_axes", a._n, b._n)
            kind = "CoincidentAxes"
            label = "Coincident"
        elif isinstance(a, AssemblyPointDatum) and isinstance(b, AssemblyPointDatum):
            if opposite_normals is not None:
                raise TypeError("opposite_normals applies only to plane datums")
            _invoke(self._n, "set_coincident_points", a._n, b._n)
            kind = "CoincidentPoints"
            label = "Coincident"
        else:
            raise TypeError("coincident datums must have matching point, axis, or plane types")
        ents = _constraint_entities(a, b)
        shown = [_entity_short(a), _entity_short(b)]
        shown = [n for n in shown if n]
        self._record(kind, "{0}: {1}".format(label, " <> ".join(shown)), ents)

    def parallel(self, a, b):
        """Keep two axis datums parallel."""
        if not isinstance(a, AssemblyAxisDatum) or not isinstance(b, AssemblyAxisDatum):
            raise TypeError("parallel currently expects two axis datums")
        _invoke(self._n, "set_parallel_axes", a._n, b._n)
        ents = _constraint_entities(a, b)
        self._record("ParallelAxes", "Parallel: {0}".format(" <> ".join(
            n for n in [_entity_short(a), _entity_short(b)] if n)), ents)

    def concentric(self, a, b):
        """Concentric axes/cylinders (two datums)."""
        _invoke(self._n, "set_concentric", a._n, b._n)
        ents = _constraint_entities(a, b)
        self._record("Concentric", "Concentric: {0}".format(" <> ".join(
            n for n in [_entity_short(a), _entity_short(b)] if n)), ents)

    def perpendicular(self, a, b):
        """Keep two axis datums perpendicular."""
        if not isinstance(a, AssemblyAxisDatum) or not isinstance(b, AssemblyAxisDatum):
            raise TypeError("perpendicular currently expects two axis datums")
        _invoke(self._n, "set_perpendicular_axes", a._n, b._n)
        ents = _constraint_entities(a, b)
        self._record("PerpendicularAxes", "Perp: {0}".format(" <> ".join(
            n for n in [_entity_short(a), _entity_short(b)] if n)), ents)

    def angle(self, a, b, radians):
        """Angle between two axis datums, in radians."""
        if not isinstance(a, AssemblyAxisDatum) or not isinstance(b, AssemblyAxisDatum):
            raise TypeError("angle currently expects two axis datums")
        _invoke(self._n, "set_angle_axes", a._n, b._n, float(radians))
        ents = _constraint_entities(a, b)
        self._record("AngleAxes", "Angle: {0}".format(" <> ".join(
            n for n in [_entity_short(a), _entity_short(b)] if n)), ents)

    def distance(self, a, b, value):
        """Distance between two point datums, or a signed plane offset along B's normal."""
        if isinstance(a, AssemblyPointDatum) and isinstance(b, AssemblyPointDatum):
            _invoke(self._n, "set_distance_points", a._n, b._n, float(value))
            kind = "DistancePoints"
            label = "Distance"
        elif isinstance(a, AssemblyPlaneDatum) and isinstance(b, AssemblyPlaneDatum):
            _invoke(self._n, "set_distance_planes", a._n, b._n, float(value))
            kind = "DistancePlanes"
            label = "Offset"
        else:
            raise TypeError("distance expects two point datums or two plane datums")
        ents = _constraint_entities(a, b)
        self._record(kind, "{0}: {1}".format(label, " <> ".join(
            n for n in [_entity_short(a), _entity_short(b)] if n)), ents)

    def solve(self):
        """Solve assembly mates and return an immutable AssemblySolveResult.

        Inspect ``result.converged`` and ``result.unsatisfied`` for diagnostics.
        """
        return AssemblySolveResult._from_json(_invoke(self._n, "solve_constraints"))

    def plane_frame(self, reference):
        """World Frame of a named assembly plane, or None."""
        native = _require(self._n, "get_plane_frame")(str(reference))
        if native is None:
            return None
        return Frame._from_native(native)

    def show(self, title="Camber"):
        """Open the 3D viewer on this assembly."""
        from .view import show as _show
        _show(self, title=title)


class Part(object):
    """CAD session: operating box, sketches, solids, booleans, import/export.

    ``low`` / ``high`` are the world AABB (vec3, 3-tuple, or a scalar on all axes).
    ``tolerance`` is default tessellation when methods pass ``max_deviation=-1``.
    Solids support ``a + b`` (union), ``a - b`` (cut), ``a & b`` (intersect).
    """

    def __init__(self, low, high, tolerance=0.01):
        """Create an independent CAD session without resetting other sessions."""
        NativePart, _, _ = _native()
        lx, ly, lz = _xyz(low)
        hx, hy, hz = _xyz(high)
        _log_progress("Part")
        self._n = NativePart(lx, ly, lz, hx, hy, hz, float(tolerance))

    @property
    def name(self):
        """Part instance name."""
        return self._n.name

    @property
    def max_deviation(self):
        """Default tessellation tolerance (world units)."""
        return self._n.max_deviation

    def smallest_unit(self):
        """Lattice step: operating-box extent / slices (~1e-6 of the box)."""
        return _require(self._n, "smallest_unit")()

    def _generate_name(self, prefix):
        """Next unique kernel name with this prefix."""
        return _require(_native_mod()["NativePart"], "generate_name")(prefix)

    def assembly(self, name=None):
        """Create or get an Assembly attached to this part."""
        return Assembly(_require(self._n, "get_assembly")(_name(name)), self)

    def sketch(self, plane="xy", name=None, origin_name=None, frame=None, constrained=False):
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

    def _unregister_sketch(self, sketch):
        """Drop a sketch from the part (does not undo solids already built from it)."""
        _require(self._n, "unregister_sketch")(sketch._n)

    def extrude(self, sketch, height, name=None, both_sides=False, max_deviation=-1, twist=0):
        """Extrude along the sketch-plane normal. ``height`` world units; ``twist`` rad per unit length."""
        n = _name(name)
        md = float(max_deviation)
        tw = float(twist)
        if both_sides:
            return Solid(_require(self._n, "extrude_two_sides")(sketch._n, height, height, md, tw, n), self)
        return Solid(_require(self._n, "extrude")(sketch._n, height, md, tw, n), self)

    def extrude_two_sides(self, sketch, plus_z, minus_z=0.0, name=None, max_deviation=-1, twist=0):
        """Extrude ``plus_z`` along +normal and ``minus_z`` along −normal."""
        return Solid(_require(self._n, "extrude_two_sides")(
            sketch._n, plus_z, minus_z, float(max_deviation), float(twist), _name(name)), self)

    def project_sketch(self, sketch, solid, name=None, max_deviation=-1):
        """Tessellate sketch and project onto solid along the sketch-plane normal."""
        return ProjectedSketch(_require(self._n, "project_sketch_onto_mesh")(
            sketch._n, solid._n, float(max_deviation), _name(name)), self)

    def extrude_projected(self, projected, height, name=None):
        """Extrude a ProjectedSketch along stored surface normals (negative = into solid)."""
        return Solid(_require(self._n, "extrude_projected_sketch")(
            projected._n, float(height), _name(name)), self)

    def revolve(self, sketch, angle, name=None, max_deviation=-1):
        """Revolve the sketch about sketch X by ``angle`` radians."""
        return Solid(_require(self._n, "revolve")(sketch._n, float(angle), float(max_deviation), _name(name)), self)

    def cylinder(self, origin, radius, height, name=None, axis="z", max_deviation=-1):
        """Cylinder from a point or Frame. ``axis`` is ``x``/``y``/``z`` if origin is a point.

        Along ``pose.z`` when ``origin`` is a Frame; base at origin, height in +Z.
        """
        if isinstance(origin, Frame):
            pose = origin
        else:
            pose = frame_from_axis(origin, axis)
        return Solid(_require(self._n, "create_cylinder")(
            _as_frame(pose)._native(), float(radius), float(height), float(max_deviation), _name(name)), self)

    def cylinder_revolve(self, pose, radius, height, name=None, max_deviation=-1):
        """Cylinder as a revolve of a rectangle (same pose convention as cylinder)."""
        return Solid(_require(self._n, "create_cylinder_revolve")(
            _as_frame(pose)._native(), float(radius), float(height), float(max_deviation), _name(name)), self)

    def sphere(self, center, radius, name=None, max_deviation=-1):
        """Sphere at ``center`` (point or Frame)."""
        return Solid(_require(self._n, "create_sphere")(
            _as_frame(center)._native(), float(radius), float(max_deviation), _name(name)), self)

    def cube(self, pose, extent, name=None):
        """Axis-aligned cube of side ``extent`` in the pose (center at origin)."""
        return Solid(_require(self._n, "create_cube")(_as_frame(pose)._native(), float(extent), _name(name)), self)

    def cuboid(self, pose_or_min, extents_or_max, name=None):
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
            self, pose, major_diameter, pitch, length, name=None, max_deviation=-1,
            right_handed=True, outer_radius=-1):
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
            self, pose, major_diameter, pitch, length, name=None, max_deviation=-1,
            right_handed=True, include_bore_chamfers=True):
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
            self, pose, major_diameter, pitch, length, name=None, max_deviation=-1,
            right_handed=True, include_bore_chamfers=False):
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

    def add_line(self, start, end, name=None):
        """3D construction line (not a sketch). Returns Curve."""
        x0, y0, z0 = _xyz(start)
        x1, y1, z1 = _xyz(end)
        return Curve(_require(self._n, "add_line")(x0, y0, z0, x1, y1, z1, _name(name)))

    def add_line_by_names(self, start_name, end_name, name=None):
        """3D line between two named points already on the part."""
        return Curve(_require(self._n, "add_line_by_names")(start_name, end_name, _name(name)))

    def add_plane(self, plane_name, origin_anchor_name):
        """Named construction plane at a named origin point."""
        _require(self._n, "add_plane")(plane_name, origin_anchor_name)

    def plane_frame(self, plane):
        """Frame of a named part plane, or None."""
        native = _require(self._n, "get_plane_frame")(str(plane))
        if native is None:
            return None
        return Frame._from_native(native)

    def extrude_along_curve(self, sketch, curve, name=None, max_deviation=-1, twist=0,
                            reference_direction=None):
        """Sweep a profile along a 3D Curve.

        Optional reference_direction projects a fixed world direction onto
        each normal plane to control profile orientation. It must never be
        parallel to the path tangent. The initial sketch sets profile clocking.
        """
        from .geom import _pack_points3
        reference = "" if reference_direction is None else _pack_points3([reference_direction])
        return Solid(_require(self._n, "extrude_along_curve")(
            sketch._n, curve._n, float(max_deviation), float(twist), _name(name), reference), self)

    def extrude_along_curve_strip(self, sketch, curves, name=None, max_deviation=-1, twist=0):
        """Sweep the sketch along a sequence of Curves (G1 strip)."""
        return Solid(_require(self._n, "extrude_along_curve_strip")(
            sketch._n, _curve_list(curves), float(max_deviation), float(twist), _name(name)), self)

    def extrude_along_sketch(self, profile, guide, name=None, max_deviation=-1):
        """Sweep ``profile`` along a 3D path taken from ``guide`` sketch curves."""
        return Solid(_require(self._n, "extrude_along_sketch")(
            profile._n, guide._n, float(max_deviation), _name(name)), self)

    def loft(self, sketches, options=None, name=None, max_deviation=-1, *, first_curves=None):
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

    def loft_surface(self, sections, *, guides=None, start_tangent=None,
                     end_tangent=None, name=None, max_deviation=-1):
        """Create an uncapped NURBS sheet through sketch sections.

        Sections retain authored curve order; degrees and knots are matched exactly.
        Rational sections must share weights after this conversion.
        Optional ``guides`` control the two side boundaries: use lines or
        ``Curve.hermite`` curves with one point at each section endpoint, in order.
        Optional tangents are world-space derivative vectors (direction and magnitude)
        for the start/end of the loft. Both point in the direction of section order.
        The loft parameter runs from zero to one with equal intervals per section.
        Conflicting guides/tangents raise an error; inputs are never fitted or snapped.
        """
        from .geom import _pack_points3
        def tangent(value):
            return "" if value is None else _pack_points3([value])
        return Solid(_require(self._n, "loft_surface")(
            _sketch_list(sections), _curve_list(() if guides is None else guides),
            tangent(start_tangent), tangent(end_tangent), float(max_deviation), _name(name)), self)

    def boolean(self, a, b, operation, name=None):
        """CSG: ``operation`` is BOOLEAN_UNION / DIFFERENCE / INTERSECT."""
        return Solid(_require(self._n, "boolean")(a._n, b._n, int(operation), _name(name)), self)

    def union(self, a, b, name=None):
        """Boolean union. Same as ``a + b``."""
        return Solid(_invoke(self._n, "union", a._n, b._n, _name(name)), self)

    def cut(self, a, b, name=None):
        """Boolean difference ``a minus b``. Same as ``a - b``."""
        return Solid(_invoke(self._n, "cut", a._n, b._n, _name(name)), self)

    def intersect(self, a, b, name=None):
        """Boolean intersection. Same as ``a & b``."""
        return Solid(_invoke(self._n, "intersect", a._n, b._n, _name(name)), self)

    def batch_union(self, meshes):
        """Union many solids. Empty list → None; one item returned as-is."""
        if not meshes:
            return None
        if len(meshes) == 1:
            return meshes[0]
        return Solid(_require(self._n, "batch_union")(_solid_list(meshes)), self)

    def batch_boolean_chain(self, mesh_a, steps):
        """Apply ``steps`` as ``[(solid, BOOLEAN_*), ...]`` in order onto ``mesh_a``."""
        chain = _native_mod()["NativeBooleanChain"]()
        for solid, op in steps:
            chain.add(solid._n, int(op))
        return Solid(_require(self._n, "batch_boolean_chain")(mesh_a._n, chain), self)

    def solid_from_mesh(self, positions, triangles, name=None):
        """Build a solid from world-space vertices and ``(i, j, k)`` triangles."""
        from .geom import _pack_points3, _pack_triangles
        return Solid(_require(self._n, "solid_from_mesh")(
            _pack_points3(positions), _pack_triangles(triangles), _name(name)), self)

    def raycast(self, solid, origin, direction):
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

    def pattern_linear(self, solid, count, step, name=None):
        """Return independent solids at equal XYZ steps, including ``solid``.

        ``count`` includes the unchanged seed. ``name`` prefixes the new names.
        """
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("count must be a positive integer")
        x, y, z = _xyz(step)
        result = _invoke(self._n, "pattern_linear", solid._n, count,
                         float(x), float(y), float(z), _name(name))
        return [solid] + [Solid(_invoke(result, "get", i), self) for i in range(1, int(result.count))]

    def pattern_circular(self, solid, count, axis=None, angle=2*math.pi, name=None):
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

    def mirror(self, solid, plane=None, name=None):
        """Return a new solid reflected in the XY plane of ``plane`` (a Frame).

        The default is the world XY plane. Source geometry and names stay intact.
        """
        plane = Frame() if plane is None else _as_frame(plane)
        return Solid(_invoke(self._n, "mirror", solid._n, plane._native(), _name(name)), self)

    def copy_solid(self, source, name):
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

    def fillet(self, solid, edges, radius, name=None, max_deviation=-1):
        """Fillet named edges. ``edges`` is a string or list of names.

        Continues across supported planar/cylindrical tangent seams. Surviving
        original faces keep their names; sharp boundaries are preserved.
        """
        return Solid(_require(self._n, "fillet")(
            solid._n, _join_names(edges), float(radius), float(max_deviation), _name(name)), self)

    def chamfer(self, solid, edges, distance, name=None, max_deviation=-1):
        """Chamfer named edges. ``edges`` is a string or list of names."""
        return Solid(_require(self._n, "chamfer")(
            solid._n, _join_names(edges), float(distance), float(max_deviation), _name(name)), self)

    def solid(self, name):
        """Look up a Solid already registered on this part by name, or None."""
        found = _require(self._n, "get_mesh_from_name")(name)
        if found is None:
            return None
        return Solid(found, self)

    def load_stl(self, path, group_border_angle_deg, name=None, require_watertight=True):
        """Import STL. ``group_border_angle_deg`` splits patches at sharp edges."""
        return Solid(_require(self._n, "load_stl_file")(
            path, float(group_border_angle_deg), _name(name), 1 if require_watertight else 0), self)

    def load_off(self, path, group_border_angle_deg, name=None, require_watertight=True):
        """Import OFF mesh."""
        return Solid(_require(self._n, "load_off_file")(
            path, float(group_border_angle_deg), _name(name), 1 if require_watertight else 0), self)

    def load_obj(self, path, group_border_angle_deg=-1, name=None, scale=1.0):
        """Import Wavefront OBJ. ``group_border_angle_deg`` splits patches at sharp edges
        (viewer edges are drawn on patch borders; use e.g. 22 when the OBJ has no ``g`` tags).
        ``scale`` multiplies vertex positions (e.g. 0.01 for cm→m)."""
        return Solid(_require(self._n, "load_wavefront_obj_file")(
            path, float(group_border_angle_deg), _name(name), float(scale)), self)

    def show(self, title="Camber"):
        """Open the 3D viewer on all meshes in this part."""
        from .view import show as _show
        _show(self, title=title)

    def sketch_interactive(
            self, plane="xy", name=None, part_var="part", sketch_var="sk",
            frame=None, emit_frame=False):
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

    def __init__(self, native, part):
        self._n = native
        self._part = part

    @property
    def name(self):
        """Kernel sketch name."""
        return self._n.name

    @property
    def solve_after_every_constraint(self):
        """If True, the kernel solves after each constraint (default)."""
        flag = getattr(self._n, "solve_after_every_constraint", None)
        if flag is None:
            return True
        return bool(flag)

    @solve_after_every_constraint.setter
    def solve_after_every_constraint(self, value):
        self._n.solve_after_every_constraint = 1 if value else 0

    @property
    def curve_count(self):
        """Number of constraint-solver curves, or -1 if this is a plotter-only sketch."""
        n = getattr(self._n, "constraint_curve_count", None)
        if n is None:
            return -1
        return int(n)

    @property
    def constraint_count(self):
        """Number of constraints, or -1 if this is a plotter-only sketch."""
        n = getattr(self._n, "constraint_count", None)
        if n is None:
            return -1
        return int(n)

    def remove_last_constraint(self):
        """Undo the last constraint. Returns self."""
        _require(self._n, "remove_last_constraint")()
        return self

    def remove_last_curve(self):
        """Undo the last constraint-solver curve. Returns self."""
        _require(self._n, "remove_last_constraint_curve")()
        return self

    @property
    def frame(self):
        """World Frame of this sketch plane."""
        return Frame._from_native(_require(self._n, "frame")())

    def add_line(self, start, end, name=None, construction=False):
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

    def add_ellipse(self, center, radii, rotation=0, name=None, construction=False):
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

    def add_circle(self, center, radius, name=None, construction=False):
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

    def add_arc(self, start, mid, end, name=None, construction=False):
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

    def set_construction(self, curve, construction=True):
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

    def add_rectangle(self, corner_a, corner_b, names=None):
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

    def add_text(self, text, origin, family="Arial", em_size=0.2, bold=False, italic=False):
        """Add TrueType glyph outlines at the baseline origin (sketch XY)."""
        x, y = _xy(origin)
        flags = 0
        if bold:
            flags |= 1
        if italic:
            flags |= 2
        _require(self._n, "add_text")(str(text), float(x), float(y), family or "", float(em_size), int(flags))
        return self

    def polylines(self, max_deviation=-1):
        """Tessellated sketch strips as a list of ``vec2`` polylines."""
        from .geom import _unpack_loops2
        md = float(max_deviation)
        if md <= 0:
            md = float(self._part.max_deviation)
        return _unpack_loops2(_require(self._n, "tessellate_polylines")(md))

    def triangulate(self, max_deviation=-1, working_volume=None, slices=None):
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

    def surface(self, name=None, max_deviation=-1):
        """Fill closed contours and register an open sheet on the part.

        The result is a Solid with ``is_volume`` False (a planar fill, not a solid).
        """
        pts2, tris = self.triangulate(max_deviation)
        if not tris:
            raise ValueError("sketch has no closed fill")
        fr = self.frame
        pts3 = [fr.origin + fr.x * p.x + fr.y * p.y for p in pts2]
        return self._part.solid_from_mesh(pts3, tris, name=name)

    def horizontal(self, curve):
        """Keep a line parallel to sketch X. Returns self."""
        _require(self._n, "set_horizontal")(_curve_index(self, curve))
        return self

    def vertical(self, curve):
        """Keep a line parallel to sketch Y. Returns self."""
        _require(self._n, "set_vertical")(_curve_index(self, curve))
        return self

    def coincident(self, point_a, point_b):
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

    def coincident_on_curve(self, point, curve, uniform):
        """Place a named point on another curve at uniform parameter u in [0, 1]."""
        kind, value = _classify_point(point)
        if kind == "name":
            _require(self._n, "set_coincident_on_curve_named")(
                _native_point_name(value), _curve_index(self, curve), float(uniform))
            return self
        raise ValueError("coincident_on_curve needs a named point")

    def parallel(self, curve_a, curve_b):
        """Keep two lines parallel. Returns self."""
        _require(self._n, "set_parallel")(_curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def perpendicular(self, curve_a, curve_b):
        """Keep two lines perpendicular. Returns self."""
        _require(self._n, "set_perpendicular")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def tangent(self, line_curve, circular_curve):
        """Line tangent to a circle/arc. Returns self."""
        _require(self._n, "set_tangent")(
            _curve_index(self, line_curve), _curve_index(self, circular_curve))
        return self

    def equal(self, curve_a, curve_b):
        """Equal length (lines) or equal radius (circles/arcs). Returns self."""
        _require(self._n, "set_equal")(_curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def midpoint(self, point, line_curve):
        """Named point at the midpoint of a line. Returns self."""
        kind, value = _classify_point(point)
        curve = _curve_index(self, line_curve)
        if kind == "name":
            _require(self._n, "set_midpoint_named")(_native_point_name(value), curve)
            return self
        raise ValueError("midpoint needs a named point")

    def point_on_line(self, point, line_curve):
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

    def point_on_circle(self, point, circle_curve):
        """Put a named point on a circle circumference."""
        kind, value = _classify_point(point)
        if kind != "name":
            raise ValueError("point_on_circle needs a named point")
        _require(self._n, "set_point_on_circle_named")(
            _native_point_name(value), _curve_index(self, circle_curve))
        return self

    def distance_to_line(self, point, line_curve, value):
        """Perpendicular distance from a named point to a line."""
        kind, value_pt = _classify_point(point)
        if kind != "name":
            raise ValueError("distance_to_line needs a named point")
        _require(self._n, "set_distance_point_line_named")(
            _native_point_name(value_pt), _curve_index(self, line_curve), float(value))
        return self

    def tangent_circles(self, curve_a, curve_b):
        """Two circles/arcs tangent to each other. Returns self."""
        _require(self._n, "set_tangent_circles")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def vertical_distance(self, point_a, point_b, value):
        """Signed vertical offset ``b.y - a.y = value``."""
        kind_a, a = _classify_point(point_a)
        kind_b, b = _classify_point(point_b)
        if kind_a != "name" or kind_b != "name":
            raise ValueError("vertical_distance needs two named points")
        _require(self._n, "set_vertical_distance_names")(
            _native_point_name(a), _native_point_name(b), float(value))
        return self

    def horizontal_distance(self, point_a, point_b, value):
        """Signed horizontal offset ``b.x - a.x = value``."""
        kind_a, a = _classify_point(point_a)
        kind_b, b = _classify_point(point_b)
        if kind_a != "name" or kind_b != "name":
            raise ValueError("horizontal_distance needs two named points")
        _require(self._n, "set_horizontal_distance_names")(
            _native_point_name(a), _native_point_name(b), float(value))
        return self

    def concentric(self, curve_a, curve_b):
        """Share a center (circles/arcs). Returns self."""
        _require(self._n, "set_concentric")(
            _curve_index(self, curve_a), _curve_index(self, curve_b))
        return self

    def fix(self, point):
        """Pin a named point in the sketch plane. XY constants are ignored."""
        kind, value = _classify_point(point)
        if kind == "xy":
            return self
        if kind == "name":
            _require(self._n, "fix_point_named")(_native_point_name(value))
            return self
        raise ValueError("fix needs a named point")

    def length(self, curve, value):
        """Set line length. Returns self."""
        _require(self._n, "set_length")(_curve_index(self, curve), float(value))
        return self

    def radius(self, curve, value):
        """Set circle/arc radius. Returns self."""
        _require(self._n, "set_radius")(_curve_index(self, curve), float(value))
        return self

    def distance(self, point_a, point_b, value):
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

    def angle(self, curve_a, curve_b, degrees):
        """Angle between two lines, in degrees. Returns self."""
        _require(self._n, "set_angle_degrees")(
            _curve_index(self, curve_a), _curve_index(self, curve_b), float(degrees))
        return self

    def solve(self):
        """Solve constraints, raising on nonconvergence. Returns self."""
        _require(self._n, "solve_constraints")()
        return self

    def eval_xy(self, point):
        """Sketch-plane XY of a named point (``line @ 1.000``, ``sk @ \"origin\"``)."""
        kind, value = _endpoint_value(point)
        if kind == "name":
            return _eval_named_point(self, value)
        if kind == "xy":
            return value
        raise ValueError("eval_xy needs a named point or XY pair")

    def offset(self, curves, distance, side="both", join="miter", end_cap="square", open_mode=None):
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

    def add_rectangle_centered(self, center, size_x, size_y, names=None):
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

    def append_line(self, end):
        """Plotter: line from the current pen to ``end``. Returns self."""
        x, y = _xy(end)
        _require(self._n, "append_line")(x, y)
        return self

    def set_start(self, point):
        """Set the strip pen without drawing (plotter ``Append*`` methods)."""
        x, y = _xy(point)
        _require(self._n, "set_start_point")(x, y)
        return self

    def move_to(self, point):
        """Start a new disconnected strip at ``point``."""
        x, y = _xy(point)
        _require(self._n, "move_to")(x, y)
        return self

    def append_line_horizontal(self, end_x):
        """Plotter: horizontal line to sketch X = ``end_x``. Returns self."""
        _require(self._n, "append_line_horizontal")(float(end_x))
        return self

    def append_line_vertical(self, end_y):
        """Plotter: vertical line to sketch Y = ``end_y``. Returns self."""
        _require(self._n, "append_line_vertical")(float(end_y))
        return self

    def append_arc_left(self, radius, angle=None):
        """Tangential left turn of ``angle`` radians (default π/2) at ``radius``."""
        if angle is None:
            angle = 0.5 * math.pi
        _require(self._n, "append_arc_tangential_left")(float(radius), float(angle))
        return self

    def append_arc_right(self, radius, angle=None):
        """Tangential right turn of ``angle`` radians (default π/2) at ``radius``."""
        if angle is None:
            angle = 0.5 * math.pi
        _require(self._n, "append_arc_tangential_right")(float(radius), float(angle))
        return self

    def add_spline(self, points, start_tangent=None, end_tangent=None, name=None):
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

    def add_involute(self, center, base_radius, t_start, t_end, rotation=0, name=None, max_deviation=-1):
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
            self, center, module, teeth, pressure_angle=None, addendum=1.0, dedendum=1.25,
            max_deviation=-1):
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
            self, center, module, teeth, pressure_angle=None, addendum=1.0, dedendum=1.25,
            max_deviation=-1):
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

    def append_spline(self, points, start_tangent=None, end_tangent=None):
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

    def repeat_circular(self, curves, center, count, total_angle=None, include_original=False):
        """Rotational copies about ``center``. Skips the 0° instance unless ``include_original``."""
        names = _curve_name_list(curves)
        cx, cy = _xy(center)
        if total_angle is None:
            total_angle = 2.0 * math.pi
        joined = _require(self._n, "repeat_circular_named")(
            "|".join(names), cx, cy, int(count), float(total_angle), 1 if include_original else 0)
        return _offset_result_curves(joined)

    def repeat_grid(self, curves, count_x, count_y, step_x, step_y, include_original=False):
        """Grid copies. Skips the (0, 0) cell unless ``include_original``."""
        names = _curve_name_list(curves)
        sxx, sxy = _xy(step_x)
        syx, syy = _xy(step_y)
        joined = _require(self._n, "repeat_grid_named")(
            "|".join(names), int(count_x), int(count_y), sxx, sxy, syx, syy,
            1 if include_original else 0)
        return _offset_result_curves(joined)

    def add_naca4(self, code, leading_edge, chord_length, chord_angle=0.0, samples_per_side=36,
                  analytic_end_tangents=True, te_trim=_NACA_TE_TRIM):
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

    def show(self, title="Camber"):
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
    """Result of Part.project_sketch: named 3D polylines on a mesh with surface normals."""

    def __init__(self, native, part):
        self._n = native
        self._part = part

    @property
    def name(self):
        """Kernel name of the projected sketch."""
        return self._n.name

    @property
    def curve_count(self):
        """Number of projected 3D polylines."""
        return int(self._n.curve_count)

    def curve_name_at(self, index):
        """Name of the polyline at ``index``."""
        return _invoke(self._n, "curve_name_at", int(index))

    def __repr__(self):
        return "ProjectedSketch(name={0!r}, curves={1})".format(self.name, self.curve_count)


class Solid(object):
    """Triangle mesh owned by a Part. Operators: ``+`` union, ``-`` cut, ``&`` intersect.

    ``Solid.mesh()`` dumps this solid's triangles. ``Part.solid(name)`` looks up a
    named solid on the part.
    """

    def __init__(self, native, part):
        self._n = native
        self._part = part

    @property
    def name(self):
        """Kernel solid name (also used as the default boolean result name)."""
        return self._n.name

    @property
    def triangle_count(self):
        """Number of triangles in the kernel mesh."""
        return self._n.triangle_count

    @property
    def is_volume(self):
        """True if the kernel treats this mesh as a closed volume."""
        flag = getattr(self._n, "is_volume", None)
        if flag is None:
            return True
        return bool(flag)

    @property
    def edge_names(self):
        """Feature edge references accepted by fillet/chamfer, excluding diagonals.

        Unambiguous provenance names can be authored in scripts. When a face's
        ancestry is ambiguous, an enumerated name carries ``#current=...`` and
        selects this solid snapshot only; reacquire it after rebuilding or copying.
        """
        count = int(self._n.edge_count)
        return tuple(_require(self._n, "edge_name_at")(i) for i in range(count))

    def __add__(self, other):
        """Boolean union. Result keeps this solid's name."""
        return self._part.union(self, other, name=self.name)

    def __sub__(self, other):
        """Boolean difference (this minus other). Result keeps this solid's name."""
        return self._part.cut(self, other, name=self.name)

    def __and__(self, other):
        """Boolean intersection. Result keeps this solid's name."""
        return self._part.intersect(self, other, name=self.name)

    def mesh(self):
        """Raw kernel triangles: ``(points, triangles)`` as ``vec3`` and ``(i, j, k)``."""
        from .geom import _unpack_indexed3
        return _unpack_indexed3(_require(self._n, "pack_mesh")())

    def signed_volume(self):
        """Signed tetrahedron volume. Negative means inward orientation."""
        return float(_require(self._n, "signed_volume")())

    def volume(self):
        """Absolute volume of a watertight mesh."""
        return abs(self.signed_volume())

    def is_watertight(self):
        """True if every edge is shared by exactly two triangles."""
        return bool(_require(self._n, "is_watertight")())

    def save_stl(self, path, binary=True):
        """Write STL. ``binary=False`` is ASCII."""
        _invoke(self._part._n, "save_stl", self._n, path, 1 if binary else 0)

    def save_step(self, path):
        """Write STEP (faceted)."""
        _invoke(self._part._n, "save_step", self._n, path)

    def save_iges(self, path):
        """Write IGES (faceted)."""
        _invoke(self._part._n, "save_iges", self._n, path)

    def save_off(self, path):
        """Write OFF mesh."""
        _require(self._part._n, "save_off")(self._n, path)

    def save_obj(self, path):
        """Write Wavefront OBJ."""
        _require(self._part._n, "save_wavefront_obj")(self._n, path)

    def save_usda(self, path):
        """Write USDA triangle mesh (per-vertex normals, UVs, named face subsets)."""
        _require(self._part._n, "save_usda")(self._n, path)

    def show(self, title="Camber"):
        """Open the 3D viewer on this solid."""
        from .view import show as _show
        _show(self, title=title)

    def __repr__(self):
        return "Solid(name={0!r}, triangle_count={1})".format(self.name, self.triangle_count)
