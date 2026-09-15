"""Named ray picking for the viewer and the sketch.

One catalog describes every pickable point and polyline. `closest` / `hit`
return a name. Sketch mode merges sketch handles with every named 3D-model
anchor (projected onto the sketch plane as an XY constant). The viewer uses
the same catalog for points and curves, then falls back to mesh picking for
faces.
"""

import math

from . import naming as _naming

PICK_PX = 5.0

KIND_POINT = "point"
KIND_CURVE = "curve"
KIND_SURFACE = "surface"

MODE_ANY = "any"
MODE_POINTS = "points"
MODE_LINES = "lines"
MODE_CIRCLES = "circles"
MODE_LINE_OR_CIRCLE = "line_or_circle"
MODE_POINT_AND_LINE = "point_and_line"

SOURCE_SKETCH = "sketch"
SOURCE_MODEL = "model"


def closest(origin, direction, targets, point_radius, curve_radius):
    """Return the winning target name, or None. Points win over curves."""
    if not targets:
        return None
    ox, oy, oz = origin
    dx, dy, dz = _unit(direction)
    if dx is None:
        return None
    if point_radius <= 0.0:
        point_radius = 1e-9
    if curve_radius <= 0.0:
        curve_radius = 1e-9

    best_point = None
    best_point_d = point_radius
    best_curve = None
    best_curve_d = curve_radius

    for target in targets:
        name = target.get("name")
        if not name:
            continue
        kind = target.get("kind")
        if kind == KIND_POINT:
            world = target.get("world")
            if world is None:
                continue
            distance = ray_point_distance((ox, oy, oz), (dx, dy, dz), world)
            if distance < best_point_d:
                best_point_d = distance
                best_point = name
            continue
        if kind != KIND_CURVE:
            continue
        poly = target.get("points") or ()
        if len(poly) < 2:
            continue
        distance = _ray_polyline_distance((ox, oy, oz), (dx, dy, dz), poly, bool(target.get("closed")))
        if distance < best_curve_d:
            best_curve_d = distance
            best_curve = name

    if best_point is not None:
        return best_point
    return best_curve


def prefer_selection(catalog_name, catalog_kind, mesh_name, mesh_kind):
    """Choose the selection name from a catalog hit and a mesh hit.

    Points win, then curves: a catalog hit means the click is within the
    pick radius of that overlay, so an edge stays pickable even when the
    ray also hits a face. Surface wins only when nothing in the catalog is
    close enough.
    """
    _ = mesh_kind
    if catalog_name and catalog_kind == KIND_POINT:
        return catalog_name
    if catalog_name and catalog_kind == KIND_CURVE:
        return catalog_name
    if catalog_name:
        return catalog_name
    return mesh_name


def hit(catalog, origin, direction, point_radius, curve_radius, mode=MODE_ANY, exclude=None):
    """Closest allowed name in `catalog`, or None."""
    return closest(
        origin,
        direction,
        filter_catalog(catalog, mode=mode, exclude=exclude),
        point_radius,
        curve_radius,
    )


def nearest_hit(
        catalog, origin, direction, point_radius, curve_radius,
        spatial_index=None):
    """Return the nearest catalog target inside its pick radius.

    The result is ``(name, kind, ray_depth)``. Depth is measured to the
    closest point on the target, not to the front of its tolerance volume.
    """
    d = _unit(direction)
    if d[0] is None:
        return None, None, float("inf")
    candidates = []
    targets = _spatial_candidates(
        spatial_index, catalog, origin, d, max(point_radius, curve_radius))
    for target in targets:
        name = target.get("name")
        kind = target.get("kind")
        if not name:
            continue
        if kind == KIND_POINT:
            world = target.get("world")
            if world is None:
                continue
            radius = max(float(point_radius), 1e-9)
            distance, depth = _ray_point_hit(origin, d, world)
            priority = 0
        elif kind == KIND_CURVE:
            polyline = target.get("points") or ()
            if len(polyline) < 2:
                continue
            radius = max(float(curve_radius), 1e-9)
            distance, depth = _ray_polyline_hit(
                origin, d, polyline, bool(target.get("closed")), radius)
            priority = 1
        else:
            continue
        if distance >= radius:
            continue
        candidates.append((depth, priority, distance, name, kind, radius))
    if not candidates:
        return None, None, float("inf")
    front_depth = min(candidate[0] for candidate in candidates)
    same_depth_layer = [
        candidate for candidate in candidates
        if candidate[0] <= front_depth + candidate[5]
    ]
    best = min(
        same_depth_layer,
        key=lambda candidate: (
            candidate[1], candidate[2], candidate[0], candidate[3]))
    return best[3], best[4], best[0]


def build_spatial_index(catalog, leaf_size=12):
    """Build an immutable BVH over point and curve bounds."""
    items = []
    for target in catalog or ():
        kind = target.get("kind")
        if kind == KIND_POINT:
            points = (target.get("world"),)
        elif kind == KIND_CURVE:
            points = target.get("points") or ()
        else:
            continue
        points = [point for point in points if point is not None]
        if not points:
            continue
        lo = tuple(min(point[axis] for point in points) for axis in range(3))
        hi = tuple(max(point[axis] for point in points) for axis in range(3))
        items.append((target, lo, hi))
    if not items:
        return None

    nodes = []

    def build(indices):
        lo = tuple(min(items[index][1][axis] for index in indices) for axis in range(3))
        hi = tuple(max(items[index][2][axis] for index in indices) for axis in range(3))
        node_index = len(nodes)
        nodes.append(None)
        if len(indices) <= leaf_size:
            nodes[node_index] = (lo, hi, -1, -1, tuple(indices))
            return node_index
        extents = tuple(hi[axis] - lo[axis] for axis in range(3))
        axis = max(range(3), key=lambda value: extents[value])
        indices.sort(
            key=lambda index: items[index][1][axis] + items[index][2][axis])
        middle = len(indices) // 2
        left = build(indices[:middle])
        right = build(indices[middle:])
        nodes[node_index] = (lo, hi, left, right, ())
        return node_index

    build(list(range(len(items))))
    return {"items": items, "nodes": nodes}


def _spatial_candidates(index, catalog, origin, direction, radius):
    if not index:
        return catalog or ()
    result = []
    stack = [0]
    items = index["items"]
    nodes = index["nodes"]
    while stack:
        lo, hi, left, right, indices = nodes[stack.pop()]
        if not _ray_box_hit(origin, direction, lo, hi, radius):
            continue
        if left < 0:
            result.extend(items[item_index][0] for item_index in indices)
        else:
            stack.append(right)
            stack.append(left)
    return result


def _ray_box_hit(origin, direction, lo, hi, radius):
    near = 0.0
    far = float("inf")
    radius = max(float(radius), 0.0)
    for axis in range(3):
        lower = lo[axis] - radius
        upper = hi[axis] + radius
        component = direction[axis]
        if abs(component) < 1e-15:
            if origin[axis] < lower or origin[axis] > upper:
                return False
            continue
        first = (lower - origin[axis]) / component
        second = (upper - origin[axis]) / component
        if first > second:
            first, second = second, first
        near = max(near, first)
        far = min(far, second)
        if near > far:
            return False
    return far >= 0.0


def filter_catalog(catalog, mode=MODE_ANY, exclude=None):
    blocked = set(exclude or ())
    out = []
    for target in catalog or ():
        name = target.get("name")
        if not name or name in blocked:
            continue
        if allowed(target, mode):
            out.append(target)
    return out


def allowed(target, mode):
    if mode is None or mode == MODE_ANY:
        return True
    kind = target.get("kind")
    curve_kind = target.get("curve_kind")
    if mode == MODE_POINTS:
        return kind == KIND_POINT
    if mode == MODE_LINES:
        return kind == KIND_CURVE and curve_kind == "line"
    if mode == MODE_CIRCLES:
        return kind == KIND_CURVE and curve_kind in ("circle", "arc")
    if mode == MODE_LINE_OR_CIRCLE:
        return kind == KIND_CURVE and curve_kind in ("line", "circle", "arc")
    if mode == MODE_POINT_AND_LINE:
        return kind == KIND_POINT or (kind == KIND_CURVE and curve_kind == "line")
    return True


def entry_by_name(catalog, name):
    if not name:
        return None
    for target in catalog or ():
        if target.get("name") == name:
            return target
    return None


def merge_catalogs(*catalogs):
    """Concatenate catalogs. The first entry for a name wins."""
    seen = set()
    out = []
    for catalog in catalogs:
        for item in catalog or ():
            name = item.get("name")
            if not name:
                continue
            key = _naming.canonicalize_point_name(name)
            if key in seen:
                continue
            seen.add(key)
            if key != name:
                item = dict(item)
                item["name"] = key
            out.append(item)
    return out


def point_entry(name, world, ref=None, source=SOURCE_MODEL):
    return {
        "name": _naming.canonicalize_point_name(name),
        "kind": KIND_POINT,
        "world": (float(world[0]), float(world[1]), float(world[2])),
        "ref": ref,
        "source": source,
    }


def curve_entry(name, points, closed=False, curve_kind=None, ref=None, source=SOURCE_MODEL):
    return {
        "name": name,
        "kind": KIND_CURVE,
        "points": list(points),
        "closed": bool(closed),
        "curve_kind": curve_kind,
        "ref": ref,
        "source": source,
    }


def catalog_from_scene(scene, frame=None, reserved_names=None):
    """Named model points and polylines from a DisplayScene.

    Every named point is included, including those off the sketch plane.
    When `frame` is set, each point gets an XY ref by projecting onto that
    plane. When `reserved_names` is a set, names are uniquified the same way
    the viewer uniquifies mesh names (patches first, then curves, then points).
    """
    if scene is None:
        return []
    uniquify = reserved_names is not None
    used = set(reserved_names or ())
    items = []
    for curve in getattr(scene, "curves", None) or ():
        if not isinstance(curve, dict):
            continue
        raw = (curve.get("name") or "").strip()
        pts = curve.get("points") or ()
        if not raw or len(pts) < 2:
            continue
        name = unique_name(used, raw) if uniquify else raw
        if not uniquify and name in used:
            continue
        used.add(name)
        items.append(curve_entry(
            name,
            [_as_xyz(p) for p in pts],
            closed=bool(curve.get("closed")),
            source=SOURCE_MODEL,
        ))
    for pt in getattr(scene, "points", None) or ():
        if not isinstance(pt, dict):
            continue
        raw = (pt.get("name") or "").strip()
        world = _as_xyz(pt.get("position"))
        if not raw or world is None:
            continue
        name = unique_name(used, raw) if uniquify else raw
        if not uniquify and name in used:
            continue
        used.add(name)
        ref = None
        if frame is not None:
            uv = project_to_uv(world, frame)
            ref = ("xy", uv[0], uv[1])
        items.append(point_entry(name, world, ref=ref, source=SOURCE_MODEL))
    return items


def catalog_from_part_points(part_points):
    """Legacy (uv, world, name) tuples used by older sketch tests."""
    items = []
    seen = set()
    for item in part_points or ():
        if item is None or len(item) < 3:
            continue
        uv, world, name = item[0], item[1], item[2]
        if not name or name in seen:
            continue
        seen.add(name)
        items.append(point_entry(name, world, ref=("xy", uv[0], uv[1]), source=SOURCE_MODEL))
    return items


def catalog_from_sketch(actions, frame, lift, point_lift, sketch_name):
    """Sketch curves, handles, and origin. Does not include model anchors."""
    catalog = []
    for index, action in enumerate(actions or ()):
        local = sketch_curve_display_name(action, index)
        curve_name = _naming.qualify(sketch_name, local)
        catalog.append(curve_entry(
            curve_name,
            sketch_curve_world_pts(action, frame, lift),
            closed=action.get("kind") == "circle",
            curve_kind=action.get("kind"),
            ref=("curve", index),
            source=SOURCE_SKETCH,
        ))
        for role, uv in sketch_handle_uvs(action):
            handle = _naming.format_sketch_handle_address(local, action.get("kind"), role)
            if not handle:
                continue
            catalog.append(point_entry(
                _naming.qualify(sketch_name, handle),
                uv_to_world(uv, frame, point_lift),
                ref=("point", index, role),
                source=SOURCE_SKETCH,
            ))
    catalog.append(point_entry(
        _naming.qualify(sketch_name, _naming.ORIGIN),
        uv_to_world((0.0, 0.0), frame, point_lift),
        ref=("xy", 0.0, 0.0),
        source=SOURCE_SKETCH,
    ))
    return catalog


def build_catalog(actions, frame, lift, point_lift, sketch_name, scene=None, part_points=None):
    """Sketch handles plus every named model point/curve (or explicit part_points)."""
    sketch = catalog_from_sketch(actions, frame, lift, point_lift, sketch_name)
    if scene is not None:
        model = catalog_from_scene(scene, frame)
    else:
        model = catalog_from_part_points(part_points)
    return merge_catalogs(sketch, model)


def build_sketch_pick_catalog(actions, part_points, frame, lift, point_lift, sketch_name, scene=None):
    """Back-compat wrapper used by sketch tests."""
    return build_catalog(
        actions, frame, lift, point_lift, sketch_name,
        scene=scene, part_points=part_points,
    )


def part_point_snaps(catalog):
    """(uv, world, name) for model points, used by sketch snap-to-anchor."""
    out = []
    for entry in catalog or ():
        if entry.get("kind") != KIND_POINT or entry.get("source") != SOURCE_MODEL:
            continue
        ref = entry.get("ref")
        if ref is None or ref[0] != "xy":
            continue
        out.append(((ref[1], ref[2]), entry.get("world"), entry.get("name")))
    return out


def sketch_curve_display_name(action, index):
    name = (action.get("name") or "").strip()
    if name:
        return name
    prefix = {"line": "Line", "circle": "Circle", "arc": "Arc"}.get(action.get("kind"), "Curve")
    return prefix + str(index + 1)


def sketch_handle_uvs(action):
    """(role, uv) for every pickable handle on a sketch action."""
    return _sketch_end_uvs(action) + _sketch_helper_uvs(action)


def sketch_curve_world_pts(action, frame, lift):
    kind = action.get("kind")
    if kind == "line":
        return [uv_to_world(action["p0"], frame, lift), uv_to_world(action["p1"], frame, lift)]
    if kind == "circle":
        return circle_polyline(action["center"], action["radius"], frame, lift)
    if kind == "arc":
        return arc_polyline(action["start"], action["mid"], action["end"], frame, lift)
    return []


def project_to_uv(world, frame):
    origin = _as_xyz(frame.origin)
    delta = (world[0] - origin[0], world[1] - origin[1], world[2] - origin[2])
    x_axis = _as_xyz(frame.x)
    y_axis = _as_xyz(frame.y)
    return (_dot(delta, x_axis), _dot(delta, y_axis))


def uv_to_world(uv, frame, lift=0.0):
    origin = _as_xyz(frame.origin)
    x_axis = _as_xyz(frame.x)
    y_axis = _as_xyz(frame.y)
    normal = _as_xyz(frame.z)
    return (
        origin[0] + uv[0] * x_axis[0] + uv[1] * y_axis[0] + lift * normal[0],
        origin[1] + uv[0] * x_axis[1] + uv[1] * y_axis[1] + lift * normal[1],
        origin[2] + uv[0] * x_axis[2] + uv[1] * y_axis[2] + lift * normal[2],
    )


def unique_name(used, name):
    base = name or "unnamed"
    candidate = base
    n = 2
    while candidate in used:
        candidate = "{0}_{1}".format(base, n)
        n += 1
    used.add(candidate)
    return candidate


def circle_polyline(center, radius, frame, lift=0.0, n=48):
    pts = []
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        uv = (center[0] + radius * math.cos(ang), center[1] + radius * math.sin(ang))
        pts.append(uv_to_world(uv, frame, lift))
    return pts


def arc_polyline(start, mid, end, frame, lift=0.0, n=32):
    d = 2.0 * (start[0] * (mid[1] - end[1]) + mid[0] * (end[1] - start[1]) + end[0] * (start[1] - mid[1]))
    if abs(d) < 1e-12:
        return [uv_to_world(p, frame, lift) for p in (start, mid, end)]
    ux = (
        (start[0] * start[0] + start[1] * start[1]) * (mid[1] - end[1])
        + (mid[0] * mid[0] + mid[1] * mid[1]) * (end[1] - start[1])
        + (end[0] * end[0] + end[1] * end[1]) * (start[1] - mid[1])
    ) / d
    uy = (
        (start[0] * start[0] + start[1] * start[1]) * (end[0] - mid[0])
        + (mid[0] * mid[0] + mid[1] * mid[1]) * (start[0] - end[0])
        + (end[0] * end[0] + end[1] * end[1]) * (mid[0] - start[0])
    ) / d
    center = (ux, uy)
    a0 = math.atan2(start[1] - uy, start[0] - ux)
    a1 = math.atan2(mid[1] - uy, mid[0] - ux)
    a2 = math.atan2(end[1] - uy, end[0] - ux)
    sweep = _arc_sweep(a0, a1, a2)
    radius = math.sqrt((start[0] - center[0]) ** 2 + (start[1] - center[1]) ** 2)
    out = []
    for i in range(n + 1):
        ang = a0 + sweep * i / n
        out.append(uv_to_world(
            (center[0] + radius * math.cos(ang), center[1] + radius * math.sin(ang)),
            frame,
            lift,
        ))
    return out


def ray_point_distance(origin, direction, point):
    return _ray_point_hit(origin, direction, point)[0]


def _ray_point_hit(origin, direction, point):
    ox, oy, oz = origin
    dx, dy, dz = direction
    px, py, pz = point
    t = (px - ox) * dx + (py - oy) * dy + (pz - oz) * dz
    if t < 0.0:
        return float("inf"), float("inf")
    return _dist(point, (ox + t * dx, oy + t * dy, oz + t * dz)), t


def _sketch_end_uvs(action):
    kind = action.get("kind")
    if kind == "line":
        return [(0, action["p0"]), (1, action["p1"])]
    if kind == "arc":
        return [(0, action["start"]), (1, action["end"])]
    if kind == "circle":
        return [(2, action["center"])]
    return []


def _sketch_helper_uvs(action):
    kind = action.get("kind")
    if kind == "line":
        p0, p1 = action["p0"], action["p1"]
        return [(3, ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5))]
    if kind == "circle":
        cx, cy = action["center"]
        r = action["radius"]
        return [
            (3, (cx + r, cy)),
            (4, (cx, cy + r)),
            (5, (cx - r, cy)),
            (6, (cx, cy - r)),
        ]
    if kind == "arc":
        return [(3, action["mid"])]
    return []


def _arc_sweep(a0, a1, a2):
    def wrap(a):
        while a <= -math.pi:
            a += 2.0 * math.pi
        while a > math.pi:
            a -= 2.0 * math.pi
        return a
    mid = wrap(a1 - a0)
    end = wrap(a2 - a0)
    if mid * end < 0:
        if end > 0:
            end -= 2.0 * math.pi
        else:
            end += 2.0 * math.pi
    return end


def _ray_polyline_distance(origin, direction, polyline, closed):
    return _ray_polyline_hit(origin, direction, polyline, closed)[0]


def _ray_polyline_hit(origin, direction, polyline, closed, radius=None):
    best = float("inf")
    best_depth = float("inf")
    last = len(polyline) - 1
    for i in range(last):
        distance, depth = _ray_segment_hit(origin, direction, polyline[i], polyline[i + 1])
        if ((radius is not None and distance < radius and depth < best_depth)
                or (radius is None and distance < best)):
            best = distance
            best_depth = depth
    if closed and len(polyline) > 2:
        distance, depth = _ray_segment_hit(origin, direction, polyline[last], polyline[0])
        if ((radius is not None and distance < radius and depth < best_depth)
                or (radius is None and distance < best)):
            best = distance
            best_depth = depth
    return best, best_depth


def _ray_segment_distance(origin, direction, a, b):
    return _ray_segment_hit(origin, direction, a, b)[0]


def _ray_segment_hit(origin, direction, a, b):
    ox, oy, oz = origin
    dx, dy, dz = direction
    ax, ay, az = a
    ex, ey, ez = b[0] - ax, b[1] - ay, b[2] - az
    wx, wy, wz = ox - ax, oy - ay, oz - az
    aa = ex * ex + ey * ey + ez * ez
    ad = ex * dx + ey * dy + ez * dz
    aw = ex * wx + ey * wy + ez * wz
    dw = dx * wx + dy * wy + dz * wz
    if aa < 1e-18:
        return _ray_point_hit(origin, direction, a)
    denom = aa - ad * ad
    if abs(denom) < 1e-18:
        return min(
            _ray_point_hit(origin, direction, a),
            _ray_point_hit(origin, direction, b),
        )
    s = (aw - ad * dw) / denom
    if s < 0.0:
        s = 0.0
    elif s > 1.0:
        s = 1.0
    t = s * ad - dw
    if t < 0.0:
        return float("inf"), float("inf")
    distance = _dist(
        (ax + s * ex, ay + s * ey, az + s * ez),
        (ox + t * dx, oy + t * dy, oz + t * dz))
    return distance, t


def _unit(direction):
    dx, dy, dz = direction
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-15:
        return None, None, None
    return dx / length, dy / length, dz / length


def _as_xyz(v):
    if v is None:
        return None
    try:
        if hasattr(v, "__len__") and len(v) >= 3:
            return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        pass
    try:
        return (float(v.x), float(v.y), float(v.z))
    except Exception:
        return None


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
