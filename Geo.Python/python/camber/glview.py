"""Lightweight OpenGL viewer (pyglet + imgui).

Phong meshes, joined screen-space curves, point sprites, and depth-peeled
transparency. Install: pip install pyglet imgui[pyglet] numpy
"""

import json
import math

from . import naming as _naming
from . import pick as _pick

# Screen-space styling, expressed as pixel radii.
_LINE_COLOR = (0.02, 0.02, 0.03)
_POINT_COLOR = (0.16, 0.16, 0.17)
_WIRE_COLOR = (0.16, 0.16, 0.18)
_EDGE_PX_PERSPECTIVE = 0.6375
_EDGE_PX_ORTHO = 1.5
_POINT_PX_PERSPECTIVE = 1.6875
_POINT_PX_ORTHO = 3.0
_WIRE_WIDTH_PX = 0.9375
_CURVE_DEPTH_PULL_PX = 1.0
_POINT_DEPTH_PULL_PX = 0.75
_SURFACE_ALPHA = 0.4
# Keep in sync with the 10.0 / 0.75 literals in _MESH_FRAG.
_CHECKER_CELLS = 10
_CHECKER_DARKEN = 0.75
# Polyscope Pretty default (options::transparencyRenderPasses).
_PEEL_PASSES = 8
_BG = (250.0 / 255.0, 250.0 / 255.0, 252.0 / 255.0)
_SELECT_SURFACE = (1.0, 0.48, 0.06)
_SELECT_CURVE = (0.82, 0.20, 0.05)
_SELECT_POINT = (1.0, 0.88, 0.0)
_WINDOW_W = 1920
_WINDOW_H = 1080
_FOV = 0.7853981633974483
# Resolve analytic line and point coverage from a 2x SSAA scene, with MSAA
# when the driver supports it.
_SSAA = 2
_MSAA = 8
DIAMOND_CORNERS = ((-1.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0))
DISC_CORNERS = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0))

# GeoCore.SurfaceType / EdgeCurveType (display blob v3). Unknown → no pick prefix.
_SURFACE_TYPE_TAGS = {
    1: "pln",
    2: "cyl",
    3: "con",
    4: "sph",
    5: "tor",
}
_EDGE_TYPE_TAGS = {
    1: "lin",
    2: "cir",
    3: "arc",
}


def _norm3(v):
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length < 1e-18:
        return None
    return (v[0] / length, v[1] / length, v[2] / length)


# Same vector as C# PhongPreviewLighting.LightDirection. That app spins the
# mesh and keeps this direction in world space; this viewer orbits the camera,
# so the shader treats it as a view-space ray (X right, Y up, −Z into scene).
_LIGHT_DIR = _norm3((0.5, -1.0, -1.0))


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _len(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _rodrigues(v, axis, angle):
    axis = _norm3(axis)
    if axis is None:
        return v
    c = math.cos(angle)
    s = math.sin(angle)
    return _add(_add(_mul(v, c), _mul(_cross(axis, v), s)), _mul(axis, _dot(axis, v) * (1.0 - c)))


def _color_from_hsv(h, s, v):
    def channel(n):
        k = (n + h * 6.0) % 6.0
        return v - v * s * max(0.0, min(min(k, 4.0 - k), 1.0))

    return (max(channel(5), 0.3), max(channel(3), 0.3), max(channel(1), 0.3))


def segment_color(index, count):
    """GeoScriptViewer ColorGenerator.SegmentColor (min RGB 0.3)."""
    if count <= 1:
        seed = 0
    else:
        seed = index * 255 // count
    return _color_from_hsv((seed / 255.0) + 0.55, 1.0, 1.0)


def checkerboard_scale(uv, cells=_CHECKER_CELLS, darken=_CHECKER_DARKEN):
    """Darken odd UV cells so each surface is a ``cells`` x ``cells`` checkerboard."""
    cell_u = int(math.floor(float(uv[0]) * cells))
    cell_v = int(math.floor(float(uv[1]) * cells))
    if (cell_u + cell_v) & 1:
        return darken
    return 1.0


def expand_point(position):
    """Six vertices for a screen-space point quad."""
    return [(position, corner) for corner in DIAMOND_CORNERS]


def expand_disc(position):
    """Six vertices covering an analytic screen-space disc."""
    return [(position, corner) for corner in DISC_CORNERS]


def expand_wireframe(verts, faces):
    """Duplicate triangle vertices with barycentrics for explicit wireframe."""
    positions = []
    barycentrics = []
    corners = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for face in faces:
        if len(face) < 3:
            continue
        for corner, vertex_index in zip(corners, face[:3]):
            positions.append(verts[int(vertex_index)])
            barycentrics.append(corner)
    return positions, barycentrics


def _append_polyline_mesh(points, closed, positions, prev_sides, nexts, indices):
    """Append one continuous strip with shared cross-sections at its joins."""
    pts = [tuple(p) for p in points]
    count = len(pts)
    if count < 2:
        return 0
    base = len(positions)
    for i, point in enumerate(pts):
        previous = pts[(i - 1) % count] if closed or i > 0 else point
        following = pts[(i + 1) % count] if closed or i + 1 < count else point
        for side in (-1.0, 1.0):
            positions.append(point)
            prev_sides.append((previous[0], previous[1], previous[2], side))
            nexts.append(following)
    segment_count = count if closed else count - 1
    for i in range(segment_count):
        a = base + 2 * i
        b = base + 2 * ((i + 1) % count)
        indices.extend((a, a + 1, b, b, a + 1, b + 1))
    return len(positions) - base


def _polyline_clean(pts, closed):
    """Remove only duplicate vertices; preserve the authoritative mesh edge."""
    if pts is None or len(pts) < 2:
        return list(pts or ())
    cleaned = [tuple(pts[0])]
    for p in pts[1:]:
        if _len(_sub(p, cleaned[-1])) > 1e-10:
            cleaned.append(tuple(p))
    if closed and len(cleaned) > 2 and _len(_sub(cleaned[0], cleaned[-1])) <= 1e-10:
        cleaned.pop()
    if len(cleaned) < 2:
        return [tuple(p) for p in pts]
    return cleaned


def build_triangle_cache(verts, faces):
    """Precompute immutable NumPy triangle data used by repeated ray picks."""
    if not verts or not faces:
        return None
    try:
        import numpy as np
    except ImportError:
        return None
    points = np.asarray(verts, dtype=np.float64)
    indices = np.asarray(faces, dtype=np.int64)
    if (points.ndim != 2 or indices.ndim != 2 or indices.shape[1] != 3
            or points.shape[0] < 3):
        return None
    v0 = points[indices[:, 0]]
    v1 = points[indices[:, 1]]
    v2 = points[indices[:, 2]]
    bounds_lo = np.minimum(np.minimum(v0, v1), v2)
    bounds_hi = np.maximum(np.maximum(v0, v1), v2)
    nodes = []

    def build(item_indices):
        lo = np.min(bounds_lo[item_indices], axis=0)
        hi = np.max(bounds_hi[item_indices], axis=0)
        node_index = len(nodes)
        nodes.append(None)
        if len(item_indices) <= 32:
            nodes[node_index] = (
                tuple(lo), tuple(hi), -1, -1, item_indices)
            return node_index
        axis = int(np.argmax(hi - lo))
        centers = bounds_lo[item_indices, axis] + bounds_hi[item_indices, axis]
        order = np.argsort(centers, kind="stable")
        middle = len(item_indices) // 2
        left = build(item_indices[order[:middle]])
        right = build(item_indices[order[middle:]])
        nodes[node_index] = (tuple(lo), tuple(hi), left, right, None)
        return node_index

    build(np.arange(len(indices), dtype=np.int64))
    return {
        "v0": v0,
        "e1": v1 - v0,
        "e2": v2 - v0,
        "nodes": nodes,
    }


def closest_triangle(origin, direction, verts, faces, cache=None):
    """Closest front-or-back triangle index along the ray, or -1."""
    if not verts or not faces:
        return -1
    d = _norm3(direction)
    if d is None:
        return -1
    try:
        import numpy as np
    except ImportError:
        return _closest_triangle_py(origin, d, verts, faces)
    if cache is None:
        cache = build_triangle_cache(verts, faces)
    if cache is None:
        return _closest_triangle_py(origin, d, verts, faces)
    o = np.asarray(origin, dtype=np.float64)
    dirv = np.asarray(d, dtype=np.float64)
    candidates = _triangle_bvh_candidates(cache, origin, d)
    if not candidates:
        return -1
    candidate_indices = np.asarray(candidates, dtype=np.int64)
    v0 = cache["v0"][candidate_indices]
    e1 = cache["e1"][candidate_indices]
    e2 = cache["e2"][candidate_indices]
    pvec = np.cross(dirv, e2)
    det = np.einsum("ij,ij->i", e1, pvec)
    valid = np.abs(det) > 1e-12
    inv = np.zeros_like(det)
    inv[valid] = 1.0 / det[valid]
    tvec = o - v0
    u = np.einsum("ij,ij->i", tvec, pvec) * inv
    qvec = np.cross(tvec, e1)
    vv = np.einsum("j,ij->i", dirv, qvec) * inv
    t = np.einsum("ij,ij->i", e2, qvec) * inv
    hit = valid & (u >= 0.0) & (vv >= 0.0) & ((u + vv) <= 1.0) & (t > 1e-8)
    if not np.any(hit):
        return -1
    ts = np.where(hit, t, np.inf)
    return int(candidate_indices[int(np.argmin(ts))])


def _triangle_bvh_candidates(cache, origin, direction):
    nodes = cache.get("nodes")
    if not nodes:
        return range(len(cache["v0"]))
    result = []
    stack = [0]
    while stack:
        lo, hi, left, right, indices = nodes[stack.pop()]
        if not _ray_box_hit(origin, direction, lo, hi):
            continue
        if left < 0:
            result.extend(indices.tolist())
        else:
            stack.append(right)
            stack.append(left)
    return result


def _ray_box_hit(origin, direction, lo, hi):
    near = 0.0
    far = float("inf")
    for axis in range(3):
        component = direction[axis]
        if abs(component) < 1e-15:
            if origin[axis] < lo[axis] or origin[axis] > hi[axis]:
                return False
            continue
        first = (lo[axis] - origin[axis]) / component
        second = (hi[axis] - origin[axis]) / component
        if first > second:
            first, second = second, first
        near = max(near, first)
        far = min(far, second)
        if near > far:
            return False
    return far >= 0.0


def _closest_triangle_py(origin, d, verts, faces):
    best = -1
    best_t = 1e300
    ox, oy, oz = origin
    for i, (ia, ib, ic) in enumerate(faces):
        v0 = verts[ia]
        v1 = verts[ib]
        v2 = verts[ic]
        e1 = _sub(v1, v0)
        e2 = _sub(v2, v0)
        pvec = _cross(d, e2)
        det = _dot(e1, pvec)
        if abs(det) < 1e-12:
            continue
        inv = 1.0 / det
        tvec = _sub((ox, oy, oz), v0)
        u = _dot(tvec, pvec) * inv
        if u < 0.0 or u > 1.0:
            continue
        qvec = _cross(tvec, e1)
        v = _dot(d, qvec) * inv
        if v < 0.0 or u + v > 1.0:
            continue
        t = _dot(e2, qvec) * inv
        if 1e-8 < t < best_t:
            best_t = t
            best = i
    return best


def triangle_hit_distance(origin, direction, verts, face):
    """Ray depth of a known triangle hit, or infinity."""
    d = _norm3(direction)
    if d is None:
        return float("inf")
    v0 = verts[int(face[0])]
    normal = _cross(
        _sub(verts[int(face[1])], v0),
        _sub(verts[int(face[2])], v0),
    )
    denominator = _dot(normal, d)
    if abs(denominator) < 1e-12:
        return float("inf")
    depth = _dot(normal, _sub(v0, origin)) / denominator
    return depth if depth > 1e-8 else float("inf")


def _grow_bounds(lo, hi, p):
    if lo is None:
        return list(p), list(p)
    for i in range(3):
        lo[i] = min(lo[i], p[i])
        hi[i] = max(hi[i], p[i])
    return lo, hi


def _face_normals(verts, faces):
    out = [(0.0, 0.0, 0.0)] * len(verts)
    acc = [[0.0, 0.0, 0.0] for _ in verts]
    for a, b, c in faces:
        n = _cross(_sub(verts[b], verts[a]), _sub(verts[c], verts[a]))
        for i in (a, b, c):
            acc[i][0] += n[0]
            acc[i][1] += n[1]
            acc[i][2] += n[2]
    for i, v in enumerate(acc):
        n = _norm3(v)
        if n is not None:
            out[i] = n
    return out


def pack_scene(scene):
    """Flatten a DisplayScene into CPU buffers (no GL)."""
    used = set()
    mesh_pos = []
    mesh_n = []
    mesh_uv = []
    mesh_color = []
    mesh_idx = []
    face_names = []
    vert_names = []
    lo = hi = None

    patches = getattr(scene, "patches", None) or ()
    curves = getattr(scene, "curves", None) or ()
    entity_types = {}
    order = []
    seen = set()
    for patch in patches:
        name = patch.get("name")
        if name and name not in seen:
            seen.add(name)
            order.append(name)
    n_unique = len(order)
    color_of = {name: segment_color(i, n_unique) for i, name in enumerate(order)}

    for patch in patches:
        tris = patch.get("faces") or []
        if not tris:
            continue
        name = _pick.unique_name(used, patch.get("name"))
        tag = _SURFACE_TYPE_TAGS.get(int(patch.get("surface_type", 0) or 0))
        if tag:
            entity_types[name] = tag
        base = len(mesh_pos)
        pv = patch["vertices"]
        pn = patch.get("normals")
        pu = patch.get("uvs")
        col = color_of.get(patch.get("name"), segment_color(0, 1))
        mesh_pos.extend(pv)
        if pn and len(pn) == len(pv):
            mesh_n.extend(pn)
        else:
            mesh_n.extend(_face_normals(pv, tris))
        if pu and len(pu) == len(pv):
            mesh_uv.extend(pu)
        else:
            mesh_uv.extend([(0.0, 0.0)] * len(pv))
        for v in pv:
            mesh_color.append(col)
            vert_names.append(name)
            lo, hi = _grow_bounds(lo, hi, v)
        for tri in tris:
            mesh_idx.append((tri[0] + base, tri[1] + base, tri[2] + base))
            face_names.append(name)

    for curve in curves:
        cname = curve.get("name")
        tag = _EDGE_TYPE_TAGS.get(int(curve.get("edge_type", 0) or 0))
        if cname and tag:
            entity_types[cname] = tag

    catalog = _pick.catalog_from_scene(scene, reserved_names=set(used))

    line_start = []
    line_end = []
    line_next = []
    line_idx = []
    line_names = []
    line_cap_pos = []
    line_cap_corner = []
    line_cap_names = []
    point_pos = []
    point_corner = []
    point_names = []
    for item in catalog:
        kind = item.get("kind")
        name = item.get("name")
        if kind == _pick.KIND_CURVE:
            closed = bool(item.get("closed"))
            pts = _polyline_clean(item.get("points") or (), closed)
            if len(pts) < 2:
                continue
            item["points"] = pts
            first = len(line_start)
            _append_polyline_mesh(
                pts, closed, line_start, line_end, line_next, line_idx)
            for point in pts:
                lo, hi = _grow_bounds(lo, hi, point)
            nverts = len(line_start) - first
            line_names.extend([name] * nverts)
            if not closed:
                for point in (pts[0], pts[-1]):
                    for pos, corner in expand_disc(point):
                        line_cap_pos.append(pos)
                        line_cap_corner.append(corner)
                    line_cap_names.extend([name] * 6)

    point_groups = {}
    for item in catalog:
        if item.get("kind") != _pick.KIND_POINT or item.get("world") is None:
            continue
        # Shared mesh vertices are bit-identical. Exact grouping avoids merging
        # distinct points merely because a model is very small.
        point_groups.setdefault(tuple(item["world"]), []).append(item)
    point_aliases = {}
    point_proxy_by_name = {}
    for key in sorted(point_groups):
        aliases = sorted(point_groups[key], key=lambda item: item["name"])
        proxy_name = aliases[0]["name"]
        alias_names = tuple(item["name"] for item in aliases)
        point_aliases[proxy_name] = alias_names
        for alias_name in alias_names:
            point_proxy_by_name[alias_name] = proxy_name
        p = aliases[0]["world"]
        for pos, corner in expand_point(p):
            point_pos.append(pos)
            point_corner.append(corner)
        point_names.extend([proxy_name] * 6)
        lo, hi = _grow_bounds(lo, hi, p)

    if lo is None:
        lo, hi = [-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]
    catalog_index = _pick.build_spatial_index(catalog)
    triangle_cache = build_triangle_cache(mesh_pos, mesh_idx)
    return {
        "mesh_pos": mesh_pos,
        "mesh_n": mesh_n,
        "mesh_uv": mesh_uv,
        "mesh_color": mesh_color,
        "mesh_idx": mesh_idx,
        "triangle_cache": triangle_cache,
        "face_names": face_names,
        "vert_names": vert_names,
        "line_start": line_start,
        "line_end": line_end,
        "line_next": line_next,
        "line_idx": line_idx,
        "line_names": line_names,
        "line_cap_pos": line_cap_pos,
        "line_cap_corner": line_cap_corner,
        "line_cap_names": line_cap_names,
        "point_pos": point_pos,
        "point_corner": point_corner,
        "point_names": point_names,
        "point_aliases": point_aliases,
        "point_proxy_by_name": point_proxy_by_name,
        "bounds": (tuple(lo), tuple(hi)),
        "catalog": catalog,
        "catalog_index": catalog_index,
        "entity_types": entity_types,
    }


def part_key(name):
    """Assembly instance token: the `mesh:` prefix of a qualified display name."""
    if not name:
        return ""
    i = name.find(":")
    if i < 0:
        return name
    return name[:i]


def packed_part_names(packed):
    """Unique assembly part names in first-seen order."""
    names = []
    seen = set()
    for key in ("face_names", "vert_names", "line_names", "point_names", "line_cap_names"):
        for name in packed.get(key) or ():
            part = part_key(name)
            if part and part not in seen:
                seen.add(part)
                names.append(part)
    for item in packed.get("catalog") or ():
        part = part_key(item.get("name"))
        if part and part not in seen:
            seen.add(part)
            names.append(part)
    return names


def split_packed_by_part(packed):
    """One packed scene per assembly part, in first-seen order.

    A single-part (or unnamed) scene is returned as-is. Line indices stay a
    flat integer list, matching pack_scene.
    """
    names = packed_part_names(packed)
    if len(names) <= 1:
        return [(names[0] if names else "", packed)]

    bounds = packed.get("bounds")
    buckets = {}

    def bucket(name):
        part = buckets.get(name)
        if part is None:
            part = {
                "mesh_pos": [],
                "mesh_n": [],
                "mesh_uv": [],
                "mesh_color": [],
                "mesh_idx": [],
                "face_names": [],
                "vert_names": [],
                "line_start": [],
                "line_end": [],
                "line_next": [],
                "line_idx": [],
                "line_names": [],
                "line_cap_pos": [],
                "line_cap_corner": [],
                "line_cap_names": [],
                "point_pos": [],
                "point_corner": [],
                "point_names": [],
                "point_aliases": {},
                "point_proxy_by_name": {},
                "bounds": bounds,
                "catalog": [],
                "_mesh_map": {},
                "_line_map": {},
            }
            buckets[name] = part
        return part

    def mesh_vert(part, index):
        mapped = part["_mesh_map"].get(index)
        if mapped is None:
            mapped = len(part["mesh_pos"])
            part["_mesh_map"][index] = mapped
            part["mesh_pos"].append(packed["mesh_pos"][index])
            part["mesh_n"].append(packed["mesh_n"][index])
            uvs = packed.get("mesh_uv") or []
            part["mesh_uv"].append(uvs[index] if index < len(uvs) else (0.0, 0.0))
            part["mesh_color"].append(packed["mesh_color"][index])
            names_v = packed.get("vert_names") or []
            part["vert_names"].append(names_v[index] if index < len(names_v) else "")
        return mapped

    def line_vert(part, index):
        mapped = part["_line_map"].get(index)
        if mapped is None:
            mapped = len(part["line_start"])
            part["_line_map"][index] = mapped
            part["line_start"].append(packed["line_start"][index])
            part["line_end"].append(packed["line_end"][index])
            part["line_next"].append(packed["line_next"][index])
            names_l = packed.get("line_names") or []
            part["line_names"].append(names_l[index] if index < len(names_l) else "")
        return mapped

    for tri, name in zip(packed.get("mesh_idx") or (), packed.get("face_names") or ()):
        part = bucket(part_key(name))
        a, b, c = tri[0], tri[1], tri[2]
        part["mesh_idx"].append((mesh_vert(part, a), mesh_vert(part, b), mesh_vert(part, c)))
        part["face_names"].append(name)

    line_names = packed.get("line_names") or []
    line_idx = packed.get("line_idx") or []
    i = 0
    while i < len(line_idx):
        item = line_idx[i]
        if isinstance(item, (tuple, list)):
            verts = list(item)
            i += 1
        else:
            if i + 2 >= len(line_idx):
                break
            verts = [line_idx[i], line_idx[i + 1], line_idx[i + 2]]
            i += 3
        if not verts:
            continue
        i0 = verts[0]
        name = line_names[i0] if 0 <= i0 < len(line_names) else ""
        part = bucket(part_key(name))
        part["line_idx"].extend(line_vert(part, v) for v in verts)

    for i, name in enumerate(packed.get("line_cap_names") or ()):
        part = bucket(part_key(name))
        part["line_cap_pos"].append(packed["line_cap_pos"][i])
        part["line_cap_corner"].append(packed["line_cap_corner"][i])
        part["line_cap_names"].append(name)

    for i, name in enumerate(packed.get("point_names") or ()):
        part = bucket(part_key(name))
        part["point_pos"].append(packed["point_pos"][i])
        part["point_corner"].append(packed["point_corner"][i])
        part["point_names"].append(name)

    for item in packed.get("catalog") or ():
        bucket(part_key(item.get("name")))["catalog"].append(item)

    aliases = packed.get("point_aliases") or {}
    proxy_by_name = packed.get("point_proxy_by_name") or {}
    out = []
    order = list(names)
    seen = set(names)
    for name in buckets:
        if name not in seen:
            order.append(name)
            seen.add(name)
    seen = set()
    for name in order:
        if name in seen or name not in buckets:
            continue
        seen.add(name)
        part = buckets[name]
        proxies = set(part["point_names"])
        for proxy in proxies:
            if proxy in aliases:
                part["point_aliases"][proxy] = aliases[proxy]
        for alias, proxy in proxy_by_name.items():
            if proxy in proxies:
                part["point_proxy_by_name"][alias] = proxy
        part.pop("_mesh_map", None)
        part.pop("_line_map", None)
        part["triangle_cache"] = build_triangle_cache(part["mesh_pos"], part["mesh_idx"])
        part["catalog_index"] = _pick.build_spatial_index(part["catalog"])
        parent_types = packed.get("entity_types") or {}
        if parent_types:
            part_types = {}
            for key in ("face_names", "line_names", "line_cap_names", "point_names"):
                for pick_name in part.get(key) or ():
                    if pick_name in parent_types:
                        part_types[pick_name] = parent_types[pick_name]
            for item in part.get("catalog") or ():
                pick_name = item.get("name")
                if pick_name in parent_types:
                    part_types[pick_name] = parent_types[pick_name]
            part["entity_types"] = part_types
        out.append((name, part))
    return out or [("", packed)]


class Camera:
    """Z-up orbit camera. Ortho zoom is ViewScale (world half-height)."""

    def __init__(self):
        self.eye = (2.0, -2.0, 1.5)
        self.center = (0.0, 0.0, 0.0)
        self.up = (0.0, 0.0, 1.0)
        self.ortho = True
        self.zoom = 1.0
        self.fov = _FOV
        self.near = 0.05
        self.far = 200.0
        self.width = _WINDOW_W
        self.height = _WINDOW_H

    @property
    def aspect(self):
        return float(self.width) / float(max(self.height, 1))

    @property
    def view_scale(self):
        if self.ortho:
            return self.zoom
        dist = max(_len(_sub(self.eye, self.center)), self.near)
        return dist * math.tan(self.fov * 0.5)

    def frame(self):
        look = _norm3(_sub(self.center, self.eye))
        if look is None:
            look = (0.0, 1.0, 0.0)
        right = _norm3(_cross(look, self.up))
        if right is None:
            right = _norm3(_cross(look, (0.0, 1.0, 0.0))) or (1.0, 0.0, 0.0)
        up = _norm3(_cross(right, look)) or (0.0, 0.0, 1.0)
        return look, up, right

    def fit(self, lo, hi):
        c = ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)
        radius = 0.5 * _len(_sub(hi, lo))
        if radius < 1e-9:
            radius = 1.0
        direction = _norm3((1.6, -1.8, 1.2))
        dist = max(radius * 2.8, radius * 1.15 / math.tan(self.fov * 0.5))
        self.center = c
        self.zoom = radius * 1.15
        self.eye = _add(c, _mul(direction, dist))
        self.up = (0.0, 0.0, 1.0)
        self._clip(radius, dist)

    def _clip(self, radius, dist):
        self.near = max(1e-4, dist * 0.01)
        self.far = max(self.near + 1e-2, dist + radius * 12.0)

    def orbit(self, ndx, ndy):
        look, up, right = self.frame()
        _ = look
        speed = 5.0
        rel = _sub(self.eye, self.center)
        rel = _rodrigues(rel, up, -ndx * speed)
        rel = _rodrigues(rel, right, -ndy * speed)
        self.up = _rodrigues(up, right, -ndy * speed)
        self.eye = _add(self.center, rel)

    def pan(self, ndx, ndy):
        look, up, right = self.frame()
        scale = self.view_scale
        offset = _add(_mul(right, -2.0 * ndx * scale), _mul(up, 2.0 * ndy * scale))
        self.eye = _add(self.eye, offset)
        self.center = _add(self.center, offset)
        _ = look

    def zoom_at(self, wheel, ndc_x, ndc_y):
        if abs(wheel) < 1e-12:
            return
        factor = math.pow(0.85, -wheel)
        origin, direction = self.ray(ndc_x, ndc_y)
        look, up, right = self.frame()
        _ = up, right
        denom = _dot(direction, look)
        if abs(denom) < 1e-8:
            hit = self.center
        else:
            t = _dot(_sub(self.center, origin), look) / denom
            hit = _add(origin, _mul(direction, t))
        if self.ortho:
            planar = _sub(hit, self.center)
            planar = _sub(planar, _mul(look, _dot(planar, look)))
            shift = _mul(planar, 1.0 - factor)
            self.eye = _add(self.eye, shift)
            self.center = _add(self.center, shift)
            self.zoom = max(self.zoom * factor, 1e-9)
            return
        self.eye = _add(hit, _mul(_sub(self.eye, hit), factor))
        self.center = _add(hit, _mul(_sub(self.center, hit), factor))

    def ray(self, ndc_x, ndc_y):
        look, up, right = self.frame()
        if self.ortho:
            origin = _add(
                self.eye,
                _add(_mul(right, ndc_x * self.aspect * self.zoom), _mul(up, ndc_y * self.zoom)),
            )
            return origin, look
        half = math.tan(self.fov * 0.5)
        direction = _norm3(
            _add(look, _add(_mul(right, half * ndc_x * self.aspect), _mul(up, half * ndc_y)))
        )
        return self.eye, direction or look

    def view_matrix(self):
        look, up, right = self.frame()
        z = _mul(look, -1.0)
        x, y = right, up
        e = self.eye
        return (
            (x[0], x[1], x[2], -_dot(x, e)),
            (y[0], y[1], y[2], -_dot(y, e)),
            (z[0], z[1], z[2], -_dot(z, e)),
            (0.0, 0.0, 0.0, 1.0),
        )

    def proj_matrix(self):
        asp = self.aspect
        n, f = self.near, self.far
        if self.ortho:
            z = self.zoom
            l, r, b, t = -asp * z, asp * z, -z, z
            return (
                (2.0 / (r - l), 0.0, 0.0, -(r + l) / (r - l)),
                (0.0, 2.0 / (t - b), 0.0, -(t + b) / (t - b)),
                (0.0, 0.0, -2.0 / (f - n), -(f + n) / (f - n)),
                (0.0, 0.0, 0.0, 1.0),
            )
        y_max = n * math.tan(0.5 * self.fov)
        x_max = y_max * asp
        return (
            (n / x_max, 0.0, 0.0, 0.0),
            (0.0, n / y_max, 0.0, 0.0),
            (0.0, 0.0, -(f + n) / (f - n), -(2.0 * f * n) / (f - n)),
            (0.0, 0.0, -1.0, 0.0),
        )


_PEEL_GLSL = """
uniform float u_peel;
uniform vec2 u_viewport;
uniform sampler2D t_opaque_depth;
uniform sampler2D t_peel_depth;

void peel_discard(float depth) {
    if (u_peel < 0.5)
        return;
    vec2 uv = gl_FragCoord.xy / max(u_viewport, vec2(1.0));
    float opaque_z = texture(t_opaque_depth, uv).r;
    float min_z = texture(t_peel_depth, uv).r;
    if (depth <= min_z + 1e-6 || depth >= opaque_z - 1e-6)
        discard;
}
"""

_MESH_VERT = """
#version 330
uniform mat4 u_view;
uniform mat4 u_proj;
in vec3 in_pos;
in vec3 in_n;
in vec3 in_color;
in vec2 in_uv;
out vec3 v_n;
out vec3 v_view;
out vec3 v_color;
out vec2 v_uv;
void main() {
    vec4 view = u_view * vec4(in_pos, 1.0);
    v_view = view.xyz;
    v_n = mat3(u_view) * in_n;
    v_color = in_color;
    v_uv = in_uv;
    gl_Position = u_proj * view;
}
"""

_MESH_FRAG = """
#version 330
uniform vec3 u_light_dir;
uniform float u_alpha;
""" + _PEEL_GLSL + """
in vec3 v_n;
in vec3 v_view;
in vec3 v_color;
in vec2 v_uv;
out vec4 frag;
void main() {
    peel_discard(gl_FragCoord.z);
    vec3 N = normalize(v_n);
    vec3 L = normalize(-u_light_dir);
    vec3 V = normalize(-v_view);
    float ndotl = abs(dot(N, L));
    vec3 H = normalize(V + L);
    float spec = 0.0;
    if (ndotl > 0.0)
        spec = 0.1 * pow(abs(dot(N, H)), 8.0);
    float a = clamp(u_alpha, 0.0, 1.0);
    vec3 albedo = v_color;
    if (a >= 0.999) {
        ivec2 cell = ivec2(floor(v_uv * 10.0));
        float checker = float((cell.x + cell.y) & 1);
        albedo *= mix(1.0, 0.75, checker);
    }
    vec3 lit = (0.3 * ndotl + 0.6 + spec) * albedo;
    frag = vec4(lit * a, a);
}
"""

_WIRE_VERT = """
#version 330
uniform mat4 u_view;
uniform mat4 u_proj;
in vec3 in_pos;
in vec3 in_barycentric;
noperspective out vec3 v_barycentric;
void main() {
    v_barycentric = in_barycentric;
    gl_Position = u_proj * u_view * vec4(in_pos, 1.0);
}
"""

_WIRE_FRAG = """
#version 330
uniform vec3 u_color;
uniform float u_width_px;
noperspective in vec3 v_barycentric;
out vec4 frag;
void main() {
    vec3 pixel_width = max(fwidth(v_barycentric), vec3(1e-6));
    vec3 interior = smoothstep(
        vec3(0.0), pixel_width * u_width_px, v_barycentric);
    float alpha = 1.0 - min(min(interior.x, interior.y), interior.z);
    if (alpha <= 0.0)
        discard;
    frag = vec4(u_color, alpha);
}
"""

_LINE_VERT = """
#version 330
uniform mat4 u_view;
uniform mat4 u_proj;
uniform float u_radius_px;
uniform float u_aa_px;
uniform vec2 u_viewport;
in vec3 in_start;
in vec4 in_end;
in vec3 in_next;
in vec3 in_color;
flat out vec3 v_color;
noperspective out float v_edge_px;
noperspective out float v_axis_depth;

vec2 clipToPixel(vec4 clip, vec2 viewport) {
    vec2 ndc = clip.xy / max(abs(clip.w), 1e-12);
    return (ndc * 0.5 + 0.5) * viewport;
}

void main() {
    vec2 viewport = max(u_viewport, vec2(1.0));
    vec3 previous = in_end.xyz;
    vec3 current = in_start;
    vec3 following = in_next;
    float side = in_end.w;
    vec4 previous_clip = u_proj * u_view * vec4(previous, 1.0);
    vec4 current_clip = u_proj * u_view * vec4(current, 1.0);
    vec4 next_clip = u_proj * u_view * vec4(following, 1.0);
    vec2 previous_px = clipToPixel(previous_clip, viewport);
    vec2 current_px = clipToPixel(current_clip, viewport);
    vec2 next_px = clipToPixel(next_clip, viewport);

    bool has_previous = length(current_px - previous_px) > 1e-5;
    bool has_next = length(next_px - current_px) > 1e-5;
    vec2 tangent;
    if (has_previous && has_next)
        tangent = normalize(next_px - previous_px);
    else if (has_next)
        tangent = normalize(next_px - current_px);
    else if (has_previous)
        tangent = normalize(current_px - previous_px);
    else
        tangent = vec2(1.0, 0.0);
    vec2 normal = vec2(-tangent.y, tangent.x);
    float extent = max(u_radius_px, 0.55) + 0.5 * max(u_aa_px, 0.75);
    vec4 clip = current_clip;
    vec2 offset_px = side * normal * extent;
    clip.xy += (2.0 * offset_px / viewport) * clip.w;
    v_color = in_color;
    v_edge_px = side * extent;
    v_axis_depth = 0.5 * (current_clip.z / max(abs(current_clip.w), 1e-12)) + 0.5;
    gl_Position = clip;
}
"""

_SCREEN_DEPTH_GLSL = """
uniform float u_near;
uniform float u_far;
uniform float u_ortho;
uniform float u_view_scale;
uniform float u_depth_pull_px;
uniform mat4 u_proj;
uniform vec2 u_viewport;

float linearizeDepth(float depth) {
    float n = max(u_near, 1e-6);
    float f = max(u_far, n + 1e-4);
    if (u_ortho > 0.5)
        return mix(n, f, clamp(depth, 0.0, 1.0));
    return (n * f) / max(f - clamp(depth, 0.0, 0.999999) * (f - n), 1e-8);
}

float nonlinearizeDepth(float view_z) {
    float n = max(u_near, 1e-6);
    float f = max(u_far, n + 1e-4);
    float z = max(view_z, n);
    if (u_ortho > 0.5)
        return clamp((z - n) / (f - n), 0.0, 1.0);
    return clamp((f / (f - n)) * (1.0 - n / z), 0.0, 1.0);
}

float pulledDepth(float axis_depth) {
    if (u_depth_pull_px <= 0.0)
        return clamp(axis_depth, 0.0, 1.0);
    float view_z = linearizeDepth(axis_depth);
    float pixel_world = u_ortho > 0.5
        ? 2.0 * u_view_scale / max(u_viewport.y, 1.0)
        : 2.0 * view_z / max(abs(u_proj[1][1]) * u_viewport.y, 1e-6);
    float pull = min(max(u_depth_pull_px, 0.0) * pixel_world,
                     max(view_z - u_near, 0.0) * 0.1);
    return nonlinearizeDepth(max(view_z - pull, u_near));
}
"""

_LINE_FRAG = """
#version 330
uniform float u_radius_px;
uniform float u_aa_px;
""" + _SCREEN_DEPTH_GLSL + """
flat in vec3 v_color;
noperspective in float v_edge_px;
noperspective in float v_axis_depth;
out vec4 frag;

void main() {
    float radius_px = max(u_radius_px, 0.55);
    float aa_px = max(u_aa_px, 0.75);
    float alpha = clamp((radius_px - abs(v_edge_px)) / aa_px + 0.5, 0.0, 1.0);
    if (alpha <= 0.0)
        discard;

    gl_FragDepth = pulledDepth(v_axis_depth);
    frag = vec4(v_color, alpha);
}
"""

_POINT_VERT = """
#version 330
uniform mat4 u_view;
uniform mat4 u_proj;
uniform float u_radius_px;
uniform float u_aa_px;
uniform vec2 u_viewport;
in vec3 in_pos;
in vec2 in_corner;
in vec3 in_color;
flat out vec2 v_center_xy;
flat out float v_axis_depth;
flat out vec3 v_color;
void main() {
    vec2 viewport = max(u_viewport, vec2(1.0));
    vec4 clip = u_proj * u_view * vec4(in_pos, 1.0);
    vec2 ndc = clip.xy / max(abs(clip.w), 1e-12);
    v_center_xy = (ndc * 0.5 + 0.5) * viewport;
    v_axis_depth = 0.5 * (clip.z / max(abs(clip.w), 1e-12)) + 0.5;
    v_color = in_color;
    float extent = max(u_radius_px, 0.55) + 0.5 * max(u_aa_px, 0.75);
    clip.xy += (2.0 * in_corner * extent / viewport) * clip.w;
    gl_Position = clip;
}
"""

_POINT_FRAG = """
#version 330
uniform float u_radius_px;
uniform float u_aa_px;
""" + _SCREEN_DEPTH_GLSL + """
flat in vec2 v_center_xy;
flat in float v_axis_depth;
flat in vec3 v_color;
out vec4 frag;
void main() {
    float dist = length(gl_FragCoord.xy - v_center_xy);
    float r = max(u_radius_px, 0.55);
    float aa = max(u_aa_px, 0.75);
    float alpha = clamp((r - dist) / aa + 0.5, 0.0, 1.0);
    if (alpha <= 0.0)
        discard;
    gl_FragDepth = pulledDepth(v_axis_depth);
    frag = vec4(v_color, alpha);
}
"""

_FLAT_VERT = """
#version 330
uniform mat4 u_view;
uniform mat4 u_proj;
in vec3 in_pos;
void main() {
    gl_Position = u_proj * u_view * vec4(in_pos, 1.0);
}
"""

_FLAT_FRAG = """
#version 330
uniform vec4 u_color;
""" + _PEEL_GLSL + """
out vec4 frag;
void main() {
    peel_discard(gl_FragCoord.z);
    float a = clamp(u_color.a, 0.0, 1.0);
    frag = vec4(u_color.rgb * a, a);
}
"""

_BLIT_VERT = """
#version 330
in vec3 in_pos;
out vec2 v_uv;
void main() {
    v_uv = in_pos.xy * 0.5 + 0.5;
    gl_Position = vec4(in_pos.xy, 0.0, 1.0);
}
"""

_BLIT_FRAG = """
#version 330
uniform sampler2D u_image;
in vec2 v_uv;
out vec4 frag;
void main() {
    frag = texture(u_image, v_uv);
}
"""

_DEPTH_COPY_FRAG = """
#version 330
uniform sampler2D u_depth;
in vec2 v_uv;
void main() {
    float d = texture(u_depth, v_uv).r;
    if (d >= 0.999)
        discard;
    gl_FragDepth = d;
}
"""

_RESOLVE_FRAG = """
#version 330
uniform sampler2D u_image;
uniform float u_ssaa;
in vec2 v_uv;
out vec4 frag;
void main() {
    ivec2 size = textureSize(u_image, 0);
    int s = max(int(u_ssaa + 0.5), 1);
    ivec2 dest = ivec2(floor(v_uv * vec2(size / s)));
    ivec2 last = max(size / s - 1, ivec2(0));
    dest = clamp(dest, ivec2(0), last);
    vec4 acc = vec4(0.0);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i < s && j < s)
                acc += texelFetch(u_image, dest * s + ivec2(i, j), 0);
        }
    }
    frag = acc / float(s * s);
}
"""


def _as_scene(obj):
    from .display import DisplayScene, decode

    if isinstance(obj, DisplayScene):
        return obj
    native = getattr(obj, "_n", obj)
    dump = getattr(native, "dump_display", None)
    if dump is None:
        raise TypeError("show() expects a Part, Solid, Assembly, or DisplayScene")
    return decode(dump())


def _import_gl():
    try:
        import numpy as np
        import pyglet

        from .imgui_compat import import_imgui
        imgui = import_imgui()
        from imgui.integrations.pyglet import create_renderer
    except ImportError:
        raise ImportError(
            "The OpenGL viewer needs pyglet, imgui, and numpy. Install with:\n"
            "  pip install pyglet imgui[pyglet] numpy"
        )
    return pyglet, np, imgui, create_renderer


def _selection_python(names):
    return ", ".join(json.dumps(name, ensure_ascii=False) for name in names)


def _quote_pick_name(name):
    return '"{0}"'.format(str(name).replace("\\", "\\\\").replace('"', '\\"'))


def _entity_type_tag(entity_types, name):
    if not entity_types or not name:
        return None
    if name in entity_types:
        return entity_types[name]
    base = str(name).rsplit("@", 1)[0]
    return entity_types.get(base)


def _selection_display(names, entity_types=None):
    """Footnote label with optional 3-letter type tags; clipboard uses _selection_python."""
    parts = []
    for name in names:
        quoted = _quote_pick_name(name)
        tag = _entity_type_tag(entity_types, name)
        if tag:
            parts.append("{0}:{1}".format(tag, quoted))
        else:
            parts.append(quoted)
    return ", ".join(parts)


def merged_entity_types(packeds):
    types = {}
    for packed in packeds or ():
        if packed:
            types.update(packed.get("entity_types") or {})
    return types


def _mat4(m, np):
    return tuple(np.asarray(m, dtype="f4").T.ravel().tolist())


def world_per_pixel(camera, height=None):
    """World size of one pixel at the look-at plane (same as the Polyscope overlays)."""
    h = float(height if height is not None else camera.height)
    return 2.0 * float(camera.view_scale) / float(max(int(h), 1))


def overlay_radii(camera, height=None):
    """World equivalents of the desired screen-space line and point radii."""
    wpp = world_per_pixel(camera, height)
    if camera.ortho:
        return wpp * _EDGE_PX_ORTHO, wpp * _POINT_PX_ORTHO, wpp
    return wpp * _EDGE_PX_PERSPECTIVE, wpp * _POINT_PX_PERSPECTIVE, wpp


def pick_radii(camera, height=None):
    """World pick radius for points and curves (screen pixels at the look-at plane)."""
    wpp = world_per_pixel(camera, height)
    radius = _pick.PICK_PX * wpp
    return radius, radius


def point_proxy_aliases(packed, name):
    """All named points represented by the same deterministic drawable proxy."""
    proxy = packed.get("point_proxy_by_name", {}).get(name, name)
    return packed.get("point_aliases", {}).get(proxy, ())


def _curve_adjacent_patches(name):
    """Qualified patch names encoded by a ``[patch_a,patch_b]`` edge name."""
    curve_name = (name or "").rsplit("@", 1)[0]
    owner, local = _naming.parse_qualified(curve_name)
    start = local.find("[")
    end = local.find("]", start + 1)
    if start < 0 or end < 0:
        return ()
    patches = [part.strip() for part in local[start + 1:end].split(",", 1)]
    if len(patches) != 2 or not all(patches):
        return ()
    return tuple(_naming.qualify(owner, patch) for patch in patches)


def pick_name(packed, origin, direction, point_r, curve_r, packeds=None):
    """Return the nearest visible named point, curve, or surface patch."""
    scenes = list(packeds) if packeds is not None else (packed,)
    cat_name = None
    cat_kind = None
    cat_depth = float("inf")
    cat_scene = None
    surf_name = None
    surf_depth = float("inf")
    for scene in scenes:
        if scene is None:
            continue
        name, kind, depth = _pick.nearest_hit(
            scene["catalog"], origin, direction, point_r, curve_r,
            scene.get("catalog_index"))
        if name:
            pri = 0 if kind == _pick.KIND_POINT else 1
            best_pri = 0 if cat_kind == _pick.KIND_POINT else 1
            if cat_name is None or (pri, depth) < (best_pri, cat_depth):
                cat_name, cat_kind, cat_depth, cat_scene = name, kind, depth, scene
        fi = closest_triangle(
            origin, direction, scene["mesh_pos"], scene["mesh_idx"],
            scene.get("triangle_cache"))
        if fi >= 0:
            dist = triangle_hit_distance(
                origin, direction, scene["mesh_pos"], scene["mesh_idx"][fi])
            if dist < surf_depth:
                surf_depth = dist
                surf_name = scene["face_names"][fi]
    picked_name = cat_name
    if cat_name and cat_kind == _pick.KIND_POINT and cat_scene is not None:
        cat_name = cat_scene.get("point_proxy_by_name", {}).get(cat_name, cat_name)
    tolerance = point_r if cat_kind == _pick.KIND_POINT else curve_r
    adjacent = surf_name in _curve_adjacent_patches(picked_name)
    if cat_name and (
            adjacent
            or cat_depth <= surf_depth + max(float(tolerance), 1e-9)):
        return cat_name
    return surf_name


def show(obj, title="Camber"):
    from .host import run_solid
    run_solid(obj, title)

