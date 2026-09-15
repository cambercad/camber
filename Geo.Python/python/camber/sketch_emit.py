"""Emit camber sketch Python from recorded draw actions."""

from . import naming as _naming
from . import pick as _pick


def emit_sketch_python(
        actions, part_var="part", sketch_var="sk", plane="xy", name="", frame=None):
    name = name or "Sketch1"
    plane = plane or "xy"
    if plane.lower() in ("xy", "yz", "zx", "xz"):
        plane = plane.lower()
    if frame is None:
        lines = [
            '{sk} = {part}.sketch("{plane}", constrained=True, name="{name}")'.format(
                sk=sketch_var, part=part_var, plane=_escape(plane), name=_escape(name)
            )
        ]
    else:
        lines = [
            "{sk} = {part}.sketch(".format(sk=sketch_var, part=part_var),
            "    frame=Frame(origin={origin}, x={x}, y={y}, z={z}),".format(
                origin=_fmt_xyz(frame.origin),
                x=_fmt_xyz(frame.x),
                y=_fmt_xyz(frame.y),
                z=_fmt_xyz(frame.z)),
            '    constrained=True, name="{0}")'.format(_escape(name)),
        ]
    for a in actions:
        kind = a["kind"]
        if kind == "line":
            lines.append(_fmt_add_curve(
                sketch_var, "add_line",
                "{0}, {1}".format(
                    _fmt_endpoint(a, "p0", "p0_ref", actions, name, sketch_var),
                    _fmt_endpoint(a, "p1", "p1_ref", actions, name, sketch_var)),
                a, actions, name))
        elif kind == "circle":
            lines.append(_fmt_add_curve(
                sketch_var, "add_circle",
                "{0}, {1}".format(
                    _fmt_endpoint(a, "center", "center_ref", actions, name, sketch_var),
                    _fmt_num(a["radius"])),
                a, actions, name))
        elif kind == "arc":
            lines.append(_fmt_add_curve(
                sketch_var, "add_arc",
                "{0}, {1}, {2}".format(
                    _fmt_endpoint(a, "start", "start_ref", actions, name, sketch_var),
                    _fmt_endpoint(a, "mid", "mid_ref", actions, name, sketch_var),
                    _fmt_endpoint(a, "end", "end_ref", actions, name, sketch_var)),
                a, actions, name))
        elif kind == "rectangle":
            lines.append("{sk}.add_rectangle({a}, {b})".format(
                sk=sketch_var, a=_fmt_xy(a["p0"]), b=_fmt_xy(a["p1"])))
        elif kind in ("horizontal", "vertical"):
            lines.append("{0}.{1}({2})".format(
                sketch_var, kind, _fmt_curve_ref(a["curves"][0], actions, name)))
        elif kind == "coincident":
            lines.append("{0}.coincident({1}, {2})".format(
                sketch_var,
                _fmt_point_expr(a["points"][0], actions, name, sketch_var),
                _fmt_point_expr(a["points"][1], actions, name, sketch_var)))
        elif kind in ("parallel", "perpendicular", "equal", "concentric"):
            lines.append("{0}.{1}({2}, {3})".format(
                sketch_var, kind,
                _fmt_curve_ref(a["curves"][0], actions, name),
                _fmt_curve_ref(a["curves"][1], actions, name)))
        elif kind == "tangent":
            lines.append("{0}.tangent({1}, {2})".format(
                sketch_var,
                _fmt_curve_ref(a["curves"][0], actions, name),
                _fmt_curve_ref(a["curves"][1], actions, name)))
        elif kind == "midpoint":
            lines.append("{0}.coincident({1}, {2})".format(
                sketch_var,
                _fmt_point_expr(a["points"][0], actions, name, sketch_var),
                _fmt_curve_midpoint(a["curves"][0], actions, name)))
        elif kind == "fix":
            lines.append("{0}.fix({1})".format(
                sketch_var, _fmt_point_expr(a["points"][0], actions, name, sketch_var)))
        elif kind in ("length", "radius"):
            lines.append("{0}.{1}({2}, {3})".format(
                sketch_var, kind,
                _fmt_curve_ref(a["curves"][0], actions, name),
                _fmt_num(a["value"])))
        elif kind == "distance":
            lines.append("{0}.distance({1}, {2}, {3})".format(
                sketch_var,
                _fmt_point_expr(a["points"][0], actions, name, sketch_var),
                _fmt_point_expr(a["points"][1], actions, name, sketch_var),
                _fmt_num(a["value"])))
        elif kind == "angle":
            lines.append("{0}.angle({1}, {2}, {3})".format(
                sketch_var,
                _fmt_curve_ref(a["curves"][0], actions, name),
                _fmt_curve_ref(a["curves"][1], actions, name),
                _fmt_num(a["value"])))
    if any(a["kind"] not in ("line", "circle", "arc", "rectangle") for a in actions):
        lines.append("{0}.solve()".format(sketch_var))
    return "\n".join(lines) + "\n"


_GEOM_KINDS = ("line", "circle", "arc", "rectangle")


def _solve_after_every_constraint(sketch):
    try:
        return bool(sketch.solve_after_every_constraint)
    except Exception:
        native = getattr(sketch, "_n", None)
        if native is None:
            return None
        try:
            return bool(native.solve_after_every_constraint)
        except Exception:
            return None


def _set_solve_after_every_constraint(sketch, value):
    previous = _solve_after_every_constraint(sketch)
    try:
        sketch.solve_after_every_constraint = value
        return previous
    except Exception:
        native = getattr(sketch, "_n", None)
        if native is None:
            return previous
        try:
            native.solve_after_every_constraint = 1 if value else 0
        except Exception:
            return previous
    return previous


def apply_actions(sketch, actions, start=0, end=None):
    """Apply actions[start:end] onto an existing sketcher.

    `actions` is the full recorded list so derived-handle rewrite can see
    earlier curves. Pass start=0 to replay everything onto a new sketch.
    """
    if start < 0:
        start = 0
    if end is None or end > len(actions):
        end = len(actions)
    added = actions[start:end]
    previous = _set_solve_after_every_constraint(sketch, False)
    try:
        for a in added:
            kind = a["kind"]
            if kind == "line":
                _apply_line(sketch, a)
            elif kind == "circle":
                _apply_add(
                    sketch.add_circle,
                    (a.get("center_ref") or a["center"], a["radius"]),
                    a)
            elif kind == "arc":
                _apply_add(
                    sketch.add_arc,
                    (
                        a.get("start_ref") or a["start"],
                        a.get("mid_ref") or a["mid"],
                        a.get("end_ref") or a["end"],
                    ),
                    a)
            elif kind == "rectangle":
                sketch.add_rectangle(a["p0"], a["p1"])
        for a in added:
            kind = a["kind"]
            if kind in ("horizontal", "vertical"):
                getattr(sketch, kind)(a["curves"][0])
            elif kind == "coincident":
                sketch.coincident(
                    _public_point(actions, a["points"][0], sketch),
                    _public_point(actions, a["points"][1], sketch))
            elif kind in ("parallel", "perpendicular", "equal", "concentric"):
                getattr(sketch, kind)(a["curves"][0], a["curves"][1])
            elif kind == "tangent":
                sketch.tangent(a["curves"][0], a["curves"][1])
            elif kind == "midpoint":
                sketch.midpoint(_public_point(actions, a["points"][0], sketch), a["curves"][0])
            elif kind == "fix":
                sketch.fix(_public_point(actions, a["points"][0], sketch))
            elif kind in ("length", "radius"):
                getattr(sketch, kind)(a["curves"][0], a["value"])
            elif kind == "distance":
                sketch.distance(
                    _public_point(actions, a["points"][0], sketch),
                    _public_point(actions, a["points"][1], sketch),
                    a["value"])
            elif kind == "angle":
                sketch.angle(a["curves"][0], a["curves"][1], a["value"])
        if any(a["kind"] not in _GEOM_KINDS for a in added):
            solve = getattr(sketch, "solve", None)
            if solve is None:
                raise AttributeError("sketch cannot solve constraints")
            solve()
    finally:
        if previous is not None:
            _set_solve_after_every_constraint(sketch, previous)
    return sketch


def _fmt_xy(p):
    return "({0}, {1})".format(_fmt_num(p[0]), _fmt_num(p[1]))


def _fmt_xyz(p):
    try:
        values = (p.x, p.y, p.z)
    except AttributeError:
        values = p
    return "({0}, {1}, {2})".format(
        _fmt_num(values[0]), _fmt_num(values[1]), _fmt_num(values[2]))


_DERIVED_UNIFORMS = {
    ("line", 3): 0.5,
    ("circle", 3): 0.0,
    ("circle", 4): 0.25,
    ("circle", 5): 0.5,
    ("circle", 6): 0.75,
    ("arc", 3): 0.5,
}


def _geometry_list(actions):
    return [a for a in actions if a.get("kind") in ("line", "circle", "arc")]


def _curve_role(point):
    if _is_xy_ref(point) or point is None or not hasattr(point, "__len__") or len(point) < 2:
        return None
    try:
        return int(point[0]), int(point[1])
    except (TypeError, ValueError):
        return None


def derived_target(actions, point):
    """Return {curve, role, kind, uniform} for a derived handle, else None."""
    cr = _curve_role(point)
    if cr is None:
        return None
    curve_index, role = cr
    geoms = _geometry_list(actions)
    if curve_index < 0 or curve_index >= len(geoms):
        return None
    kind = geoms[curve_index].get("kind")
    uniform = _DERIVED_UNIFORMS.get((kind, role))
    if uniform is None:
        return None
    return {
        "curve": curve_index,
        "role": role,
        "kind": kind,
        "uniform": uniform,
    }


def apply_coincident(sketch, actions, point_a, point_b):
    """Coincident two recorded points after rewriting handles to names."""
    sketch.coincident(
        _public_point(actions, point_a, sketch),
        _public_point(actions, point_b, sketch),
    )


def _public_point(actions, point, sketch=None, sketch_name=""):
    """Name or (\"xy\", x, y) for a recorded point. (curve, role) handles become names."""
    if point is None:
        return point
    if isinstance(point, str):
        return _rewrite_point_name(actions, sketch, point, sketch_name)
    if _is_xy_ref(point):
        return point
    handle = _curve_role(point)
    if handle is not None:
        named = _handle_to_name(actions, sketch, handle[0], handle[1])
        if named:
            return named
        raise ValueError("cannot name sketch handle {0!r}".format(point))
    return point


def _live_curve_name(actions, sketch, index):
    recorded = _curve_name(actions, index)
    if sketch is None:
        return recorded
    try:
        dumped = sketch._solved_actions()
    except Exception:
        dumped = []
    geoms = _geometry_list(actions)
    if 0 <= index < len(dumped) and 0 <= index < len(geoms):
        if dumped[index].get("kind") == geoms[index].get("kind"):
            name = (dumped[index].get("name") or "").strip()
            if name:
                return name
    return recorded


def _handle_to_name(actions, sketch, index, role):
    geoms = _geometry_list(actions)
    kind = geoms[index].get("kind") if 0 <= index < len(geoms) else ""
    curve_name = _live_curve_name(actions, sketch, index)
    return _naming.format_sketch_handle_address(curve_name, kind, role)


def _rewrite_point_name(actions, sketch, name, sketch_name=""):
    local = local_point_name(name, sketch_name)
    parsed = _naming.parse_sketch_curve_address(local)
    if parsed is None:
        return local
    recorded = parsed["curve"]
    geoms = _geometry_list(actions)
    index = None
    for i, action in enumerate(geoms):
        action_name = (action.get("name") or "").strip()
        if action_name == recorded or _pick.sketch_curve_display_name(action, i) == recorded:
            index = i
            break
    if index is None:
        return local
    live_name = _live_curve_name(actions, sketch, index)
    if live_name == recorded:
        return local
    if parsed.get("center"):
        return _naming.format_sketch_curve_center(live_name)
    return _naming.format_sketch_curve_address(live_name, parsed.get("uniform") or 0.0)


def _apply_line(sketch, action):
    start = action.get("p0_ref") or action["p0"]
    end = action.get("p1_ref") or action["p1"]
    _apply_add(sketch.add_line, (start, end), action)


def _curve_kwargs(action):
    kwargs = {}
    if action.get("name"):
        kwargs["name"] = action["name"]
    if action.get("construction"):
        kwargs["construction"] = True
    return kwargs


def _apply_add(fn, args, action):
    kwargs = _curve_kwargs(action)
    try:
        fn(*args, **kwargs)
        return
    except TypeError:
        pass
    if "construction" in kwargs:
        kwargs = dict(kwargs)
        kwargs.pop("construction", None)
        try:
            fn(*args, **kwargs)
            return
        except TypeError:
            pass
    fn(*args)


def _fmt_add_curve(sketch_var, method, args, action, actions, sketch_name):
    extra = ""
    if action.get("construction"):
        extra = ", construction=True"
    return "{sk}.{method}({args}, name={name}{extra})".format(
        sk=sketch_var,
        method=method,
        args=args,
        name=_fmt_name(_curve_name(actions, _geometry_index(actions, action))),
        extra=extra,
    )


def _helper_xy(action, role):
    kind = action.get("kind")
    if kind == "line" and role == 3:
        p0, p1 = action["p0"], action["p1"]
        return ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5)
    if kind == "circle" and role in (3, 4, 5, 6):
        cx, cy = action["center"]
        r = float(action["radius"])
        if role == 3:
            return (cx + r, cy)
        if role == 4:
            return (cx, cy + r)
        if role == 5:
            return (cx - r, cy)
        return (cx, cy - r)
    if kind == "arc" and role == 3:
        return tuple(action["mid"])
    return None


def _materialize_point_ref(actions, point):
    """Convert derived handles (circle cardinals, midpoints) to fixed XY."""
    if isinstance(point, str) or _is_xy_ref(point) or point is None:
        return point
    if not hasattr(point, "__len__") or len(point) < 2:
        return point
    if point[0] == "xy":
        return point
    try:
        curve_index = int(point[0])
        role = int(point[1])
    except (TypeError, ValueError):
        return point
    geoms = _geometry_list(actions)
    if curve_index < 0 or curve_index >= len(geoms):
        return point
    xy = _helper_xy(geoms[curve_index], role)
    if xy is None:
        return point
    return ("xy", float(xy[0]), float(xy[1]))


def _is_xy_ref(point):
    return (
        point is not None
        and hasattr(point, "__len__")
        and len(point) >= 3
        and point[0] == "xy"
    )


def _fmt_origin(sketch_var="sk"):
    return '{0} @ "origin"'.format(sketch_var)


def _fmt_point_ref(point, actions=None, sketch_name="", sketch_var="sk"):
    return _fmt_point_expr(point, actions or [], sketch_name, sketch_var)


def _fmt_point_expr(point, actions, sketch_name="", sketch_var="sk"):
    if isinstance(point, str):
        local = local_point_name(point, sketch_name)
        if _naming.is_origin_name(local) or _naming.is_origin_name(point):
            return _fmt_origin(sketch_var)
        return _fmt_name(local)
    if _is_xy_ref(point):
        if abs(float(point[1])) < 1e-12 and abs(float(point[2])) < 1e-12:
            return _fmt_origin(sketch_var)
        return _fmt_xy((point[1], point[2]))
    handle = _curve_role(point)
    if handle is not None:
        named = handle_display_name(actions, handle[0], handle[1])
        if named:
            return _fmt_name(named)
        return "({0}, {1})".format(handle[0], handle[1])
    if point is not None and hasattr(point, "__len__") and len(point) >= 2:
        try:
            return _fmt_xy((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            pass
    return repr(point)


def _fmt_endpoint(action, xy_key, ref_key, actions, sketch_name, sketch_var="sk"):
    ref = action.get(ref_key)
    if ref:
        return _fmt_point_expr(ref, actions, sketch_name, sketch_var)
    xy = action[xy_key]
    if abs(float(xy[0])) < 1e-12 and abs(float(xy[1])) < 1e-12:
        return _fmt_origin(sketch_var)
    return _fmt_xy(xy)


def _fmt_curve_ref(curve, actions, sketch_name=""):
    if isinstance(curve, str):
        return _fmt_name(local_curve_name(curve, sketch_name))
    try:
        return _fmt_name(_curve_name(actions, int(curve)))
    except (TypeError, ValueError):
        return _fmt_name(str(curve))


def _fmt_curve_midpoint(curve, actions, sketch_name=""):
    if isinstance(curve, str):
        local = local_curve_name(curve, sketch_name)
        return _fmt_name(_naming.format_sketch_handle_address(local, "line", 3))
    try:
        index = int(curve)
    except (TypeError, ValueError):
        return _fmt_name(str(curve))
    return _fmt_name(handle_display_name(actions, index, 3) or (_curve_name(actions, index) + "@0.500"))


def _curve_name(actions, index):
    geoms = _geometry_list(actions)
    if 0 <= index < len(geoms):
        return _pick.sketch_curve_display_name(geoms[index], index)
    return "Curve{0}".format(int(index) + 1)


def handle_display_name(actions, curve_index, role):
    geoms = _geometry_list(actions)
    if curve_index < 0 or curve_index >= len(geoms):
        return ""
    action = geoms[curve_index]
    curve_name = _pick.sketch_curve_display_name(action, curve_index)
    return _naming.format_sketch_handle_address(curve_name, action.get("kind"), role)


def local_point_name(name, sketch_name=""):
    owner, local = _naming.parse_qualified(name)
    if _naming.is_origin_name(local or name):
        return _naming.ORIGIN
    if owner and sketch_name and owner == sketch_name:
        return local
    if not owner:
        return local or name
    return name


def local_curve_name(name, sketch_name=""):
    owner, local = _naming.parse_qualified(name)
    if "@" in (local or ""):
        local = local.rsplit("@", 1)[0]
    if owner and sketch_name and owner == sketch_name:
        return local
    return local or name


def _geometry_index(actions, action):
    geoms = _geometry_list(actions)
    for index, item in enumerate(geoms):
        if item is action:
            return index
    try:
        return geoms.index(action)
    except ValueError:
        return 0


def next_curve_name(actions, kind):
    prefix = {"line": "Line", "circle": "Circle", "arc": "Arc"}.get(kind, "Curve")
    used = set()
    geoms = _geometry_list(actions)
    for index, item in enumerate(geoms):
        name = (item.get("name") or "").strip()
        used.add(name or _pick.sketch_curve_display_name(item, index))
    n = 1
    while prefix + str(n) in used:
        n += 1
    return prefix + str(n)


def _fmt_name(name):
    return '"{0}"'.format(_escape(name))


def _fmt_num(v):
    v = float(v)
    r = round(v)
    if abs(v - r) < 1e-9:
        return str(int(r))
    s = "{0:.4f}".format(v).rstrip("0").rstrip(".")
    return s if s else "0"


def _escape(s):
    return (s or "").replace("\\", "\\\\").replace('"', '\\"')
