"""Viewer entry: pyglet + imgui solids and sketches."""

import math
import time

from .clipboard import copy_to_clipboard
from . import pick as _pick

# Screen-space overlays (world radius is updated each frame from camera distance).
_EDGE_PX_PERSPECTIVE = 0.6375
_EDGE_PX_ORTHO = 1.5
_POINT_PX_PERSPECTIVE = 1.6875
_POINT_PX_ORTHO = 3.0
_SURFACE_BLUE = (0.55, 0.70, 0.86)
_SURFACE_ALPHA = 0.4
_SELECT_SURFACE = (1.0, 0.48, 0.06)
_SELECT_CURVE = (0.82, 0.20, 0.05)
_SELECT_POINT = (1.0, 0.88, 0.0)
_POINT_COLOR = (0.16, 0.16, 0.17)
# Fully saturated gold, slightly orange (HSV ~32°, S=1, V=1).
_CONSTRAINT_GOLD = (1.0, 0.55, 0.0, 1.0)
_CONSTRAINT_GOLD_HOVER = (1.0, 0.68, 0.0, 1.0)
_CONSTRAINT_GOLD_ACTIVE = (1.0, 0.42, 0.0, 1.0)
_CONSTRAINT_SELECT = _CONSTRAINT_GOLD
_CONSTRAINT_SELECT_HOVER = _CONSTRAINT_GOLD_HOVER
_CONSTRAINT_SELECT_ACTIVE = _CONSTRAINT_GOLD_ACTIVE
_CONSTRAINT_REF_COLORS = (
    (0.90, 0.12, 0.12),
    (0.12, 0.70, 0.22),
    (0.95, 0.72, 0.10),
    (0.15, 0.50, 0.95),
    (0.80, 0.20, 0.80),
    (0.10, 0.78, 0.78),
)
_PICK_DRAG_PX = 5.0
_CAD = {"press": None, "orbiting": False}
_OVERLAY = {"height": 800.0, "wpp": None, "edge_r": None, "point_r": None}
_LOADED_MATERIALS = set()

_BATCH_SURF = "__camber_surfaces"
_BATCH_SURF_HL = "__camber_surf_hl"
_BATCH_CURVE = "__camber_curves"
_BATCH_CURVE_PICK = "__camber_curve_pick"
_BATCH_CURVE_HL = "__camber_curve_hl"
_BATCH_POINT = "__camber_points"


def _selection_python(names):
    """Comma-separated double-quoted names: copy-paste as a Python string or list items."""
    parts = []
    for name in names:
        parts.append('"{0}"'.format(name.replace("\\", "\\\\").replace('"', '\\"')))
    return ", ".join(parts)


def _is_sketch(obj):
    return callable(getattr(obj, "_solved_actions", None)) and callable(getattr(obj, "add_line", None))


def show(obj, title="Camber"):
    """Open a native 3D window: click to pick a name, Ctrl-click to multi-select, copies names."""
    if _is_sketch(obj):
        from .sketch_ui import show_sketch
        show_sketch(obj, title=title)
        return
    from .glview import show as _gl_show
    _gl_show(obj, title=title)


def _enter_sketch_mode(ps, imgui, plane, sketch_session, action_toast, selected, apply_colors):
    from .sketch_ui import begin_sketch_session

    def on_selection(names):
        selected[:] = list(names)
        apply_colors()

    try:
        session = begin_sketch_session(
            ps,
            imgui,
            plane["part"],
            plane=plane["name"],
            frame=plane["frame"],
            emit_frame=plane["emit_frame"],
            register_context=False,
            look_at=True,
            on_selection=on_selection,
        )
    except Exception as ex:
        action_toast[:] = ["Could not start sketch: {0}".format(ex), time.monotonic() + 4.0]
        return
    selected[:] = []
    apply_colors()
    sketch_session[0] = session


def _leave_sketch_mode(ps, session, sketch_session, action_toast, transparent):
    from .sketch_ui import finalize_sketch_session

    finished = session["state"].get("done") == "finish"
    finalize_sketch_session(ps, session)
    sketch_session[0] = None
    try:
        ps.set_transparency_mode("pretty" if transparent[0] else "none")
    except Exception:
        pass
    if finished:
        message = "Sketch Python copied to clipboard" if session.get("clipboard_copied") else "Sketch finished; clipboard copy failed"
        action_toast[:] = [message, time.monotonic() + 3.5]
    else:
        action_toast[:] = ["Sketch discarded", time.monotonic() + 2.5]


def init_viewer(ps, title="Camber", ground="shadow_only"):
    """Studio look: cool backdrop, soft contact shadows, contrasty matcap lighting."""
    try:
        ps.set_use_prefs_file(False)
    except Exception:
        pass
    window_title = "Camber"
    try:
        ps.set_program_name(window_title)
    except Exception:
        pass
    ps.init()
    _try_set(ps, "set_program_name", window_title)
    _try_set(ps, "set_window_size", 1920, 1080)
    _center_native_window(window_title)
    _try_set(ps, "set_up_dir", "z_up")
    # C# GeoScriptViewer orbits with unconstrained screen-axis rotation (not a turntable).
    _try_set(ps, "set_navigation_style", "none")
    _try_set(ps, "set_do_default_mouse_interaction", False)
    _CAD["press"] = None
    _CAD["orbiting"] = False
    _try_set(ps, "set_background_color", (250.0 / 255.0, 250.0 / 255.0, 252.0 / 255.0))
    _try_set(ps, "set_ground_plane_mode", ground)
    _try_set(ps, "set_shadow_darkness", 0.28)
    _try_set(ps, "set_shadow_blur_iters", 6)
    _try_set(ps, "set_SSAA_factor", 2)
    _try_set(ps, "set_build_default_gui_panels", False)
    _try_set(ps, "set_open_imgui_window_for_user_callback", False)
    _try_set(ps, "set_view_projection_mode", "orthographic")


def _center_native_window(title):
    """Center the native viewer in the Windows desktop work area."""
    import os

    if os.name != "nt":
        return
    try:
        import ctypes
        from ctypes import wintypes

        user32 = ctypes.windll.user32
        hwnd = user32.FindWindowW(None, str(title))
        if not hwnd:
            return

        window = wintypes.RECT()
        work = wintypes.RECT()
        if not user32.GetWindowRect(hwnd, ctypes.byref(window)):
            return
        if not user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(work), 0):
            return

        width = window.right - window.left
        height = window.bottom - window.top
        x = work.left + max(0, (work.right - work.left - width) // 2)
        y = work.top + max(0, (work.bottom - work.top - height) // 2)
        user32.SetWindowPos(hwnd, 0, x, y, 0, 0, 0x0001 | 0x0004 | 0x0010)
    except Exception:
        pass


def _float_to_rgbe(r, g, b):
    r = 0.0 if r < 0.0 else r
    g = 0.0 if g < 0.0 else g
    b = 0.0 if b < 0.0 else b
    v = r
    if g > v:
        v = g
    if b > v:
        v = b
    if v < 1e-32:
        return (0, 0, 0, 0)
    mantissa, exponent = math.frexp(v)
    scale = mantissa * 256.0 / v
    return (
        min(255, int(r * scale)),
        min(255, int(g * scale)),
        min(255, int(b * scale)),
        exponent + 128,
    )


def _write_hdr_rle_channel(stream, values):
    i = 0
    n = len(values)
    while i < n:
        dump = min(128, n - i)
        stream.write(bytes((dump,)))
        stream.write(bytes(values[i:i + dump]))
        i += dump


def _write_hdr(path, size, pixels):
    """Linear Radiance HDR. Polyscope inverse-tonemaps 8-bit images and that posters."""
    with open(path, "wb") as stream:
        stream.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
        stream.write("-Y {0} +X {0}\n".format(size).encode("ascii"))
        for row in range(size):
            start = row * size
            rgbe = [_float_to_rgbe(*pixels[start + col]) for col in range(size)]
            stream.write(bytes((2, 2, (size >> 8) & 255, size & 255)))
            for channel in range(4):
                _write_hdr_rle_channel(stream, [p[channel] for p in rgbe])


def _sat(value):
    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0
    return value


def _csharp_phong_shade(x, y, z):
    """Same BRDF as GeoRendering shadeDirectionalPhong / PhongPreviewLighting."""
    # C# LightDirection = normalize(0.5, -1, -1). Matcaps are view-locked, so the
    # same magnitudes are used in view space (right, up, toward camera).
    lx, ly, lz = 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0
    ndotl = abs(x * lx + y * ly + z * lz)
    hx, hy, hz = lx, ly, lz + 1.0
    hl = math.sqrt(hx * hx + hy * hy + hz * hz)
    ndoth = abs((x * hx + y * hy + z * hz) / hl)
    spec = 0.1 * (ndoth ** 8) if ndotl > 0.0 else 0.0
    return 0.6 + 0.3 * ndotl + spec


def _load_surface_material(ps):
    """C# Phong baked into a blendable matcap (albedo in RGB, no extra K highlight)."""
    import os
    import tempfile

    name = "camber-csharp-phong"
    if name in _LOADED_MATERIALS:
        return name
    size = 512
    root = os.path.join(tempfile.gettempdir(), "camber-polyscope")
    paths = [os.path.join(root, "csharp_phong_{0}.hdr".format(c)) for c in "rgbk"]
    try:
        os.makedirs(root, exist_ok=True)
        channels = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        muls = []
        for iy in range(size):
            y = 2.0 * iy / (size - 1) - 1.0
            for ix in range(size):
                x = 2.0 * ix / (size - 1) - 1.0
                z = math.sqrt(max(0.0, 1.0 - x * x - y * y))
                muls.append(min(1.0, _csharp_phong_shade(x, y, z)))
        rgb_hdr = []
        for channel in channels:
            rgb_hdr.append([(m * channel[0], m * channel[1], m * channel[2]) for m in muls])
        k_hdr = [(0.0, 0.0, 0.0)] * (size * size)
        for path, pixels in zip(paths[:3], rgb_hdr):
            _write_hdr(path, size, pixels)
        _write_hdr(paths[3], size, k_hdr)
        ps.load_blendable_material(name, filenames=paths)
        _LOADED_MATERIALS.add(name)
        return name
    except Exception:
        if name in _LOADED_MATERIALS:
            return name
        return "clay"


def _polyline_edges(pts, closed):
    edges = [[i, i + 1] for i in range(len(pts) - 1)]
    if closed:
        last = pts[-1]
        first = pts[0]
        dx = last[0] - first[0]
        dy = last[1] - first[1]
        dz = last[2] - first[2]
        if dx * dx + dy * dy + dz * dz > 1e-16:
            edges.append([len(pts) - 1, 0])
    return edges


def _concat_patches(patches, used_names):
    verts = []
    faces = []
    face_names = []
    normals = []
    have_normals = True
    for patch in patches:
        tris = patch.get("faces") or []
        if not tris:
            continue
        name = _unique(used_names, patch["name"])
        base = len(verts)
        pv = patch["vertices"]
        verts.extend(pv)
        pn = patch.get("normals")
        if have_normals and pn and len(pn) == len(pv):
            normals.extend(pn)
        else:
            have_normals = False
        for tri in tris:
            faces.append((tri[0] + base, tri[1] + base, tri[2] + base))
            face_names.append(name)
    return verts, faces, face_names, normals if have_normals and normals else None


def _hemisphere_dirs(count):
    dirs = []
    golden = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(count):
        z = (i + 0.5) / count
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        theta = golden * i
        dirs.append((math.cos(theta) * radius, math.sin(theta) * radius, z))
    return dirs


def _vertex_ambient_occlusion(verts, faces, vertex_normals=None):
    """World-space voxel AO. Polyscope has no SSAO; this darkens cavities and contact."""
    try:
        import numpy as np
    except ImportError:
        return None
    if not verts or not faces:
        return None
    points = np.asarray(verts, dtype=np.float64)
    tris = np.asarray(faces, dtype=np.int64)
    nvert = int(points.shape[0])
    if nvert < 4 or tris.size < 3:
        return [1.0] * nvert

    a = points[tris[:, 0]]
    b = points[tris[:, 1]]
    c = points[tris[:, 2]]
    if vertex_normals is not None and len(vertex_normals) == nvert:
        normals = np.asarray(vertex_normals, dtype=np.float64)
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(lengths, 1e-18)
    else:
        normals = np.zeros((nvert, 3), dtype=np.float64)
        face_n = np.cross(b - a, c - a)
        np.add.at(normals, tris[:, 0], face_n)
        np.add.at(normals, tris[:, 1], face_n)
        np.add.at(normals, tris[:, 2], face_n)
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(lengths, 1e-18)

    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    diag = float(np.linalg.norm(maxs - mins))
    if not math.isfinite(diag) or diag < 1e-12:
        return [1.0] * nvert

    edge = float(np.mean(np.linalg.norm(np.vstack((b - a, c - b, a - c)), axis=1)))
    radius = min(0.14 * diag, max(0.06 * diag, 4.5 * max(edge, 1e-12)))
    cell = max(diag / 88.0, radius / 8.0)
    dims = np.maximum(np.ceil((maxs - mins) / cell).astype(np.int32), 1)
    origin = mins - 0.5 * cell

    occ = np.zeros((int(dims[0]), int(dims[1]), int(dims[2])), dtype=np.uint8)
    subdiv = int(max(2, min(4, math.ceil(max(edge, cell) / cell))))
    samples = [points]
    for i in range(subdiv + 1):
        for j in range(subdiv + 1 - i):
            k = subdiv - i - j
            samples.append((i * a + j * b + k * c) / float(subdiv))
    samples = np.vstack(samples)
    keys = np.floor((samples - origin) / cell).astype(np.int32)
    keys[:, 0] = np.clip(keys[:, 0], 0, dims[0] - 1)
    keys[:, 1] = np.clip(keys[:, 1], 0, dims[1] - 1)
    keys[:, 2] = np.clip(keys[:, 2], 0, dims[2] - 1)
    occ[keys[:, 0], keys[:, 1], keys[:, 2]] = 1

    up = np.array((0.0, 0.0, 1.0))
    alt = np.array((0.0, 1.0, 0.0))
    tangent = np.cross(normals, up)
    fallback = np.linalg.norm(tangent, axis=1) < 1e-8
    tangent[fallback] = np.cross(normals[fallback], alt)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-18)
    bitangent = np.cross(normals, tangent)

    home = np.floor((points - origin) / cell).astype(np.int32)
    home[:, 0] = np.clip(home[:, 0], 0, dims[0] - 1)
    home[:, 1] = np.clip(home[:, 1], 0, dims[1] - 1)
    home[:, 2] = np.clip(home[:, 2], 0, dims[2] - 1)

    hits = np.zeros(nvert, dtype=np.float64)
    weights = np.zeros(nvert, dtype=np.float64)
    seen = np.zeros(nvert, dtype=bool)
    steps = 8
    local_dirs = _hemisphere_dirs(20)
    for lx, ly, lz in local_dirs:
        world = lx * tangent + ly * bitangent + lz * normals
        cosine = max(lz, 0.05)
        weights += cosine
        seen[:] = False
        for step in range(1, steps + 1):
            t = step / float(steps)
            probe = points + world * (t * radius)
            g = np.floor((probe - origin) / cell).astype(np.int32)
            inside = (
                (g[:, 0] >= 0) & (g[:, 0] < dims[0])
                & (g[:, 1] >= 0) & (g[:, 1] < dims[1])
                & (g[:, 2] >= 0) & (g[:, 2] < dims[2])
            )
            blocked = np.zeros(nvert, dtype=bool)
            if np.any(inside):
                gi = g[inside]
                blocked[inside] = occ[gi[:, 0], gi[:, 1], gi[:, 2]] > 0
            same = (
                (g[:, 0] == home[:, 0])
                & (g[:, 1] == home[:, 1])
                & (g[:, 2] == home[:, 2])
            )
            blocked &= ~same & ~seen
            hits += blocked.astype(np.float64) * ((1.0 - 0.65 * t) ** 2) * cosine
            seen |= blocked

    ao = 1.0 - 0.95 * (hits / np.maximum(weights, 1e-8))
    ao = np.clip(ao, 0.0, 1.0)
    ao = 0.30 + 0.70 * ao
    return ao.tolist()


def _face_ao_from_vertices(faces, vertex_ao):
    if not vertex_ao or not faces:
        return None
    out = []
    for a, b, c in faces:
        out.append((vertex_ao[a] + vertex_ao[b] + vertex_ao[c]) / 3.0)
    return out


def _concat_curves(curves, used_names):
    nodes = []
    edges = []
    edge_names = []
    node_names = []
    for curve in curves:
        pts = curve.get("points") or []
        if len(pts) < 2:
            continue
        name = _unique(used_names, curve["name"])
        base = len(nodes)
        nodes.extend(pts)
        node_names.extend([name] * len(pts))
        local = _polyline_edges(pts, curve.get("closed"))
        for a, b in local:
            edges.append((a + base, b + base))
            edge_names.append(name)
    return nodes, edges, edge_names, node_names


def _concat_points(points, used_names):
    positions = []
    names = []
    for pt in points:
        names.append(_unique(used_names, pt["name"]))
        pos = pt["position"]
        positions.append((float(pos[0]), float(pos[1]), float(pos[2])))
    return positions, names


def _repeat_each(items, times):
    out = []
    for item in items:
        for _ in range(times):
            out.append(item)
    return out


def _quad_batch_faces(count):
    faces = []
    for i in range(count):
        o = 4 * i
        faces.append((o, o + 1, o + 2))
        faces.append((o, o + 2, o + 3))
    return faces


def _diamond_batch_verts(positions, right, up, half, pull):
    verts = []
    for p in positions:
        verts.extend(_diamond_verts(_vadd(p, pull), right, up, half))
    return verts


def _rect_batch_verts(positions, right, up, half, pull):
    verts = []
    for p in positions:
        verts.extend(_rect_verts(_vadd(p, pull), right, up, half))
    return verts


def _curve_pick_verts(nodes, edges, look, up, half, pull):
    verts = []
    for a, b in edges:
        pa = _vadd(nodes[a], pull)
        pb = _vadd(nodes[b], pull)
        d = _vsub(pb, pa)
        side = _cross(d, look)
        sl = _vlen(side)
        if sl < 1e-18:
            side = _cross(d, up)
            sl = _vlen(side)
        if sl < 1e-18:
            side = (1.0, 0.0, 0.0)
            sl = 1.0
        side = _vmul(side, half / sl)
        verts.append(_vsub(pa, side))
        verts.append(_vadd(pa, side))
        verts.append(_vadd(pb, side))
        verts.append(_vsub(pb, side))
    return verts


def _curve_hl_frame(ps):
    pos, center, look, up, right = _cad_frame(ps)
    wpp = _OVERLAY.get("wpp")
    if wpp is None or not math.isfinite(wpp) or wpp <= 0.0:
        wpp = 1e-4
    if look is None:
        look = (0.0, 0.0, 1.0)
        up = (0.0, 1.0, 0.0)
    edge_r = wpp * (_EDGE_PX_ORTHO if _is_ortho(ps) else _EDGE_PX_PERSPECTIVE)
    half = max(edge_r * 2.6, wpp * 2.0)
    pull = _vmul(look, -max(4.5 * wpp, 1e-9))
    return look, up, half, pull




def _load_constraints(obj):
    native = getattr(obj, "_n", None)
    dump = getattr(native, "dump_constraints", None) if native is not None else None
    if dump is not None:
        try:
            parsed = _parse_constraint_dump(dump())
            if parsed:
                return parsed
        except UnicodeDecodeError:
            pass
    rec = getattr(obj, "constraints", None)
    if rec:
        return list(rec)
    return []


_CONSTRAINT_KIND_SHORT = {
    "FixPart": "Fix",
    "CoincidentPlanes": "Coincident",
    "CoincidentAxes": "Coincident",
    "CoincidentPoints": "Coincident",
    "ParallelAxes": "Parallel",
    "Concentric": "Concentric",
    "DistancePlanes": "Offset",
    "DistancePoints": "Distance",
}


def _entity_list_name(ref):
    query = _constraint_entity_query(ref)
    if not query:
        return ""
    if query.endswith(":"):
        return query[:-1]
    if ":" in query:
        return _entity_local(query) or query
    return query


def _constraint_row_text(constraint):
    kind = constraint.get("kind") or ""
    head = _CONSTRAINT_KIND_SHORT.get(kind, kind or "constraint")
    names = [_entity_list_name(e) for e in (constraint.get("entities") or [])]
    names = [n for n in names if n]
    if names:
        return "{0}: {1}".format(head, " <> ".join(names))
    raw = constraint.get("label") or head
    return raw.replace(" ↔ ", " <> ").replace("?", "<>")


def _parse_constraint_dump(text):
    constraints = []
    current = None
    if not text:
        return constraints
    for line in str(text).splitlines():
        if not line:
            continue
        parts = line.split("\t")
        if not parts:
            continue
        if parts[0] == "C" and len(parts) >= 3:
            current = {"kind": parts[1], "label": parts[2], "entities": [], "glyphs": []}
            constraints.append(current)
        elif parts[0] == "E" and current is not None and len(parts) >= 2:
            current["entities"].append(parts[1])
        elif parts[0] == "R" and current is not None and len(parts) >= 3:
            current["entities"].append(parts[2])
        elif parts[0] == "G" and current is not None and len(parts) >= 8:
            try:
                origin = (float(parts[2]), float(parts[3]), float(parts[4]))
                direction = (float(parts[5]), float(parts[6]), float(parts[7]))
            except (TypeError, ValueError):
                continue
            current.setdefault("glyphs", []).append({
                "kind": parts[1],
                "origin": origin,
                "direction": direction,
            })
    return constraints


def _constraint_entity_query(ref):
    if isinstance(ref, str):
        return ref
    if isinstance(ref, dict):
        return ref.get("name") or ref.get("entity") or ""
    return ""


def _entity_mesh(name):
    if not name:
        return ""
    i = name.find(":")
    if i < 0:
        return ""
    return name[:i]


def _entity_local(name):
    if not name:
        return ""
    i = name.find(":")
    if i < 0:
        return name
    return name[i + 1:]


def _part_query(query):
    """Bare 'pipe1' or 'pipe1:' means the whole part. Otherwise None."""
    if not query:
        return None
    if query.endswith(":"):
        part = query[:-1]
        return part or None
    if ":" not in query:
        return query
    return None


def _constraint_name_matches(display_name, ref):
    query = _constraint_entity_query(ref)
    if not query or not display_name:
        return False
    if display_name == query:
        return True
    part = _part_query(query)
    if part is not None:
        # DisplayPack names are '{mesh}:{local}'. Require that exact mesh token
        # (pipe1:… matches, pipe10:… and pipe2:pipe1-… do not).
        d_mesh = _entity_mesh(display_name)
        if d_mesh:
            return d_mesh == part
        return display_name == part or display_name.startswith(part + "-")
    q_mesh = _entity_mesh(query)
    d_mesh = _entity_mesh(display_name)
    if not q_mesh or d_mesh != q_mesh:
        return False
    return _local_names_match(_entity_local(display_name), _entity_local(query))


def _local_names_match(display_local, query_local):
    """Same patch, or a disconnected split of it (``Circle1_1``)."""
    if not display_local or not query_local:
        return False
    if display_local == query_local:
        return True
    prefix = query_local + "_"
    if display_local.startswith(prefix):
        return display_local[len(prefix):].isdigit()
    return False


def _is_part_entity(ref):
    return _part_query(_constraint_entity_query(ref)) is not None


def _name_is_selected(display_name, selected_set):
    if not display_name or not selected_set:
        return False
    if display_name in selected_set:
        return True
    for selected in selected_set:
        if _constraint_name_matches(display_name, selected):
            return True
    return False


def _constraint_ref_color(display_name, refs, aliases=None):
    """Mate-slot color (A red, B green, …) if this display name belongs to a ref."""
    if not refs:
        return None
    names = []
    if display_name:
        names.append(display_name)
    if aliases:
        for alias in aliases:
            if alias and alias not in names:
                names.append(alias)
    for j, ref in enumerate(refs):
        if _is_part_entity(ref):
            continue
        for name in names:
            if _constraint_name_matches(name, ref):
                return _CONSTRAINT_REF_COLORS[j % len(_CONSTRAINT_REF_COLORS)]
    return None


def _mate_triangle_indices(mesh_idx, vert_names, refs):
    """Triangles whose three vertices belong to a selected mate patch."""
    if not refs or not mesh_idx or not vert_names:
        return []
    n = len(vert_names)
    mate = [False] * n
    for i, name in enumerate(vert_names):
        if _constraint_ref_color(name, refs) is not None:
            mate[i] = True
    if not any(mate):
        return []
    kept = []
    for tri in mesh_idx:
        if tri is None or len(tri) < 3:
            continue
        a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        if a < 0 or b < 0 or c < 0 or a >= n or b >= n or c >= n:
            continue
        if mate[a] and mate[b] and mate[c]:
            kept.append((a, b, c))
    return kept


def _color_from_hsv(h, s, v):
    def channel(n):
        k = (n + h * 6.0) % 6.0
        return v - v * s * max(0.0, min(min(k, 4.0 - k), 1.0))

    r = max(channel(5), 0.3)
    g = max(channel(3), 0.3)
    b = max(channel(1), 0.3)
    return (r, g, b)


def _segment_color(index, count):
    """GeoScriptViewer ColorGenerator.SegmentColor."""
    if count <= 1:
        seed = 0
    else:
        seed = index * 255 // count
    return _color_from_hsv((seed / 255.0) + 0.55, 1.0, 1.0)


def _patch_face_colors(names):
    order = []
    seen = set()
    for name in names:
        if name not in seen:
            seen.add(name)
            order.append(name)
    count = len(order)
    index = {name: i for i, name in enumerate(order)}
    return [_segment_color(index[name], count) for name in names]


def _face_base_color(default, i):
    if not default:
        return _SURFACE_BLUE
    first = default[0]
    if isinstance(first, (int, float)):
        return default
    if i < len(default):
        return default[i]
    return default[0]


def _surface_face_colors(names, default, selected_set, selected_color, refs, face_ao=None):
    colors = []
    for i, name in enumerate(names):
        highlighted = _name_is_selected(name, selected_set)
        color = selected_color if highlighted else _face_base_color(default, i)
        for j, ref in enumerate(refs or []):
            if _is_part_entity(ref):
                continue
            if _constraint_name_matches(name, ref):
                color = _CONSTRAINT_REF_COLORS[j % len(_CONSTRAINT_REF_COLORS)]
                highlighted = True
                break
        if not highlighted and face_ao is not None and i < len(face_ao):
            color = _tint_rgb(color, face_ao[i])
        colors.append(color)
    return colors


def _tint_rgb(color, amount):
    amount = 0.0 if amount < 0.0 else (1.0 if amount > 1.0 else amount)
    return (color[0] * amount, color[1] * amount, color[2] * amount)


def _set_mesh_colors(mesh, colors, default, defined_on):
    if mesh is None:
        return
    for name in ("__surface_colors", "__surface_ao"):
        try:
            mesh.remove_quantity(name, False)
        except Exception:
            pass
    if not colors:
        try:
            mesh.set_color(default)
        except Exception:
            pass
        return
    quantity = "__surface_ao" if defined_on == "vertices" else "__surface_colors"
    try:
        mesh.add_color_quantity(quantity, colors, defined_on=defined_on, enabled=True)
    except Exception:
        try:
            mesh.set_color(default)
        except Exception:
            pass


def _set_face_colors(mesh, colors, default):
    _set_mesh_colors(mesh, colors, default, "faces")


def _selectable(imgui, label, selected):
    fn = getattr(imgui, "Selectable", None)
    if fn is None:
        return False
    try:
        clicked = fn(label, bool(selected))
    except TypeError:
        try:
            clicked = fn(label)
        except Exception:
            return False
    if isinstance(clicked, tuple):
        clicked = clicked[0]
    return bool(clicked)


def _push_style_colors(imgui, pairs):
    pushed = 0
    seen = set()
    for name, color in pairs:
        idx = getattr(imgui, name, None)
        if idx is None or idx in seen:
            continue
        seen.add(idx)
        try:
            imgui.PushStyleColor(int(idx), color)
            pushed += 1
            continue
        except TypeError:
            pass
        except Exception:
            continue
        try:
            imgui.PushStyleColor(int(idx), color[0], color[1], color[2], color[3])
            pushed += 1
            continue
        except Exception:
            pass
        try:
            vec4 = getattr(imgui, "ImVec4", None)
            if vec4 is not None:
                imgui.PushStyleColor(int(idx), vec4(color[0], color[1], color[2], color[3]))
                pushed += 1
        except Exception:
            pass
    return pushed


def _push_constraint_window_colors(imgui):
    gold = _CONSTRAINT_GOLD
    return _push_style_colors(imgui, (
        ("ImGuiCol_TitleBg", gold),
        ("ImGuiCol_TitleBgActive", gold),
        ("ImGuiCol_TitleBgCollapsed", gold),
        ("Col_TitleBg", gold),
        ("Col_TitleBgActive", gold),
        ("Col_TitleBgCollapsed", gold),
    ))


def _push_constraint_select_colors(imgui):
    return _push_style_colors(imgui, (
        ("ImGuiCol_Header", _CONSTRAINT_SELECT),
        ("ImGuiCol_HeaderHovered", _CONSTRAINT_SELECT_HOVER),
        ("ImGuiCol_HeaderActive", _CONSTRAINT_SELECT_ACTIVE),
        ("Col_Header", _CONSTRAINT_SELECT),
        ("Col_HeaderHovered", _CONSTRAINT_SELECT_HOVER),
        ("Col_HeaderActive", _CONSTRAINT_SELECT_ACTIVE),
    ))


def _pop_style_color(imgui, count):
    if not count:
        return
    pop = getattr(imgui, "PopStyleColor", None)
    if pop is None:
        return
    try:
        pop(int(count))
    except TypeError:
        for _ in range(int(count)):
            try:
                pop()
            except Exception:
                break
    except Exception:
        pass


def _imgui_flag(result, attr):
    """pyimgui 2 returns a struct with .selected / .opened, not a raw bool."""
    if result is None:
        return False
    flag = getattr(result, attr, None)
    if flag is not None:
        return bool(flag)
    if isinstance(result, tuple) and result:
        return bool(result[0])
    return bool(result)


def _begin_tab_bar(imgui, name):
    fn = getattr(imgui, "BeginTabBar", None)
    if fn is None:
        return False
    try:
        opened = fn(name)
    except Exception:
        return False
    return _imgui_flag(opened, "opened")


def _end_tab_bar(imgui):
    fn = getattr(imgui, "EndTabBar", None)
    if fn is not None:
        try:
            fn()
        except Exception:
            pass


def _begin_tab_item(imgui, label):
    fn = getattr(imgui, "BeginTabItem", None)
    if fn is None:
        return False
    try:
        opened = fn(label)
    except TypeError:
        try:
            opened = fn(label, None)
        except Exception:
            return False
    except Exception:
        return False
    return _imgui_flag(opened, "selected")


def _end_tab_item(imgui):
    fn = getattr(imgui, "EndTabItem", None)
    if fn is not None:
        try:
            fn()
        except Exception:
            pass


def _checkbox(imgui, label, value):
    fn = getattr(imgui, "Checkbox", None)
    if fn is None:
        return False, value
    try:
        result = fn(label, bool(value))
    except Exception:
        return False, value
    if isinstance(result, tuple):
        changed = bool(result[0])
        if len(result) > 1:
            return changed, bool(result[1])
        return changed, value
    return bool(result), value


def _draw_assembly_panel(imgui, constraints, selected_index, menu_open, part_names, hidden_parts):
    """Tabs for assembly parts (visibility) and mates. Returns (mate_changed, vis_changed)."""
    if (not constraints and not part_names) or not menu_open[0]:
        return False, False
    cond = int(getattr(imgui, "ImGuiCond_FirstUseEver", 4) or 4)
    flags = 0
    for name in ("ImGuiWindowFlags_NoCollapse", "ImGuiWindowFlags_NoFocusOnAppearing"):
        flags |= int(getattr(imgui, name, 0) or 0)
    try:
        imgui.SetNextWindowPos((12.0, 12.0), cond)
    except TypeError:
        try:
            imgui.SetNextWindowPos((12.0, 12.0))
        except Exception:
            pass
    try:
        imgui.SetNextWindowSize((560.0, 360.0), cond)
    except TypeError:
        pass
    title = "Assembly (M hide)"
    if not part_names:
        title = "Constraints (M hide, T transparency)"
    win_pushed = _push_constraint_window_colors(imgui)
    visible, still_open = _begin_closable(imgui, title, flags, menu_open[0])
    menu_open[0] = still_open
    if not visible:
        _end_overlay(imgui)
        _pop_style_color(imgui, win_pushed)
        return False, False
    mate_changed = False
    vis_changed = False
    try:
        use_tabs = bool(part_names) and bool(constraints)
        if use_tabs and _begin_tab_bar(imgui, "##asm_tabs"):
            if _begin_tab_item(imgui, "Parts"):
                vis_changed = _draw_parts_tab(imgui, part_names, hidden_parts)
                _end_tab_item(imgui)
            if _begin_tab_item(imgui, "Constraints"):
                mate_changed = _draw_constraints_tab(imgui, constraints, selected_index)
                _end_tab_item(imgui)
            _end_tab_bar(imgui)
        elif part_names and constraints:
            vis_changed = _draw_parts_tab(imgui, part_names, hidden_parts)
            try:
                imgui.Separator()
            except Exception:
                pass
            mate_changed = _draw_constraints_tab(imgui, constraints, selected_index)
        elif part_names:
            vis_changed = _draw_parts_tab(imgui, part_names, hidden_parts)
        else:
            mate_changed = _draw_constraints_tab(imgui, constraints, selected_index)
    finally:
        _end_overlay(imgui)
        _pop_style_color(imgui, win_pushed)
    return mate_changed, vis_changed


def _draw_parts_tab(imgui, part_names, hidden_parts):
    changed = False
    try:
        imgui.TextUnformatted("Visible parts")
    except Exception:
        try:
            imgui.Text("Visible parts")
        except Exception:
            pass
    if imgui.Button("Show all"):
        if hidden_parts:
            hidden_parts.clear()
            changed = True
    try:
        imgui.SameLine()
    except Exception:
        pass
    if imgui.Button("Hide all"):
        before = len(hidden_parts)
        hidden_parts.update(part_names)
        if len(hidden_parts) != before:
            changed = True
    try:
        imgui.Separator()
    except Exception:
        pass
    for i, name in enumerate(part_names):
        visible = name not in hidden_parts
        toggled, value = _checkbox(imgui, "{0}##part{1}".format(name, i), visible)
        if toggled:
            changed = True
            if value:
                hidden_parts.discard(name)
            else:
                hidden_parts.add(name)
    return changed


def _draw_constraints_tab(imgui, constraints, selected_index):
    changed = False
    try:
        imgui.TextUnformatted("A red   B green   extra yellow/blue")
    except Exception:
        try:
            imgui.Text("A red   B green   extra yellow/blue")
        except Exception:
            pass
    try:
        imgui.Separator()
    except Exception:
        pass
    pushed = _push_constraint_select_colors(imgui)
    try:
        for i, constraint in enumerate(constraints):
            label = "{0}. {1}##c{2}".format(i + 1, _constraint_row_text(constraint), i)
            if _selectable(imgui, label, selected_index[0] == i):
                if selected_index[0] == i:
                    selected_index[0] = -1
                else:
                    selected_index[0] = i
                changed = True
    finally:
        _pop_style_color(imgui, pushed)
    return changed


def _draw_constraints_panel(imgui, constraints, selected_index, menu_open):
    changed, _vis = _draw_assembly_panel(
        imgui, constraints, selected_index, menu_open, [], set())
    return changed


def _paint_named(obj, names, default, select_color, selected_set, defined_on):
    if not names:
        return
    if not selected_set:
        try:
            obj.remove_quantity("__sel", False)
        except Exception:
            pass
        try:
            obj.set_color(default)
        except Exception:
            pass
        return
    cols = [select_color if _name_is_selected(n, selected_set) else default for n in names]
    try:
        if defined_on is None:
            obj.add_color_quantity("__sel", cols, enabled=True)
        else:
            obj.add_color_quantity("__sel", cols, defined_on=defined_on, enabled=True)
    except Exception:
        try:
            obj.set_color(select_color)
        except Exception:
            pass


def _pick_name_at(ps, imgui, catalog, tables, screen):
    """Catalog points/curves plus GPU mesh pick. Faces keep the surface hit."""
    catalog_name = None
    catalog_kind = None
    origin, direction = camera_ray(ps, imgui, screen)
    if origin is not None and direction is not None:
        wpp = _world_per_pixel(ps, imgui)
        pixel = _pick.PICK_PX * wpp if wpp is not None and wpp > 0.0 else 0.0
        tolerance = max(pixel, 1e-6)
        catalog_name = _pick.hit(catalog, origin, direction, tolerance, tolerance)
        entry = _pick.entry_by_name(catalog, catalog_name)
        if entry is not None:
            catalog_kind = entry.get("kind")
    mesh_name, mesh_kind = _gpu_pick_hit(ps, tables, screen)
    return _pick.prefer_selection(catalog_name, catalog_kind, mesh_name, mesh_kind)


def _gpu_pick_hit(ps, tables, screen):
    result = ps.pick(screen_coords=screen)
    hit = getattr(result, "is_hit", None)
    if hit is None:
        hit = getattr(result, "isHit", False)
    if not hit:
        return None, None
    name = _resolve_pick_name(result, tables)
    if not name:
        return None, None
    sname = getattr(result, "structure_name", None) or getattr(result, "structureName", None)
    table = tables.get(str(sname)) or {}
    kind = table.get("pick_kind") or table.get("kind")
    if kind == "overlay":
        kind = _pick.KIND_POINT if str(sname) == _BATCH_POINT else _pick.KIND_CURVE
    if kind == "surface":
        kind = _pick.KIND_SURFACE
    return name, kind


def _resolve_pick_name(pick, tables):
    sname = getattr(pick, "structure_name", None) or getattr(pick, "structureName", None)
    if not sname:
        return None
    table = tables.get(str(sname))
    if table is None:
        if str(sname).startswith("__"):
            return None
        return str(sname)
    data = getattr(pick, "structure_data", None) or {}
    idx = data.get("index", getattr(pick, "local_index", None))
    try:
        idx = int(idx)
    except Exception:
        return None
    names = table.get("faces")
    if table.get("kind") == "curve":
        et = str(data.get("element_type", "") or "").lower()
        names = table["nodes"] if "node" in et else table["edges"]
    if table.get("kind") == "overlay":
        names = table.get("faces")
    if names is None or idx < 0 or idx >= len(names):
        return None
    return names[idx]


def _draw_footnote(imgui, text):
    if not text:
        return None
    screen_w, screen_h = _display_wh(imgui)
    margin = 14.0
    max_w = max(160.0, screen_w - 2.0 * margin)
    wrap_w = max_w - 20.0
    win_w, win_h = _wrapped_text_size(imgui, text, wrap_w)
    win_w = min(max_w, max(80.0, win_w + 16.0))
    win_h = min(screen_h * 0.4, max(24.0, win_h + 12.0))
    try:
        imgui.SetNextWindowPos((margin, screen_h - margin), 0, (0.0, 1.0))
    except TypeError:
        try:
            imgui.SetNextWindowPos((margin, screen_h - margin - win_h))
        except Exception:
            pass
    try:
        imgui.SetNextWindowSize((win_w, win_h))
    except Exception:
        pass
    try:
        imgui.SetNextWindowBgAlpha(0.78)
    except Exception:
        pass
    flags = _overlay_flags(imgui, scrollbar=win_h >= screen_h * 0.4 - 1.0)
    opened = _begin_overlay(imgui, "##camber_sel", flags)
    top = None
    if opened:
        _draw_wrapped_text(imgui, text, wrap_w)
        top = _window_top(imgui)
    _end_overlay(imgui)
    return top


def _draw_copy_toast(imgui, sel_top, text="Copied to clipboard"):
    screen_w, screen_h = _display_wh(imgui)
    margin = 14.0
    y = (sel_top - 6.0) if sel_top is not None else (screen_h - 52.0)
    try:
        imgui.SetNextWindowPos((margin, y), 0, (0.0, 1.0))
    except TypeError:
        try:
            imgui.SetNextWindowPos((margin, y - 28.0))
        except Exception:
            pass
    try:
        imgui.SetNextWindowBgAlpha(0.86)
    except Exception:
        pass
    flags = _overlay_flags(imgui, scrollbar=False)
    opened = _begin_overlay(imgui, "##camber_copied", flags)
    if opened:
        try:
            imgui.TextUnformatted(text)
        except Exception:
            imgui.Text(text)
    _end_overlay(imgui)


def _display_wh(imgui):
    w, h = 1920.0, 1080.0
    try:
        size = imgui.GetIO().DisplaySize
        w = float(size[0] if hasattr(size, "__len__") else size.x)
        h = float(size[1] if hasattr(size, "__len__") else size.y)
    except Exception:
        pass
    return w, h


def _overlay_flags(imgui, scrollbar):
    names = [
        "ImGuiWindowFlags_NoTitleBar",
        "ImGuiWindowFlags_NoResize",
        "ImGuiWindowFlags_NoMove",
        "ImGuiWindowFlags_NoSavedSettings",
        "ImGuiWindowFlags_NoFocusOnAppearing",
        "ImGuiWindowFlags_NoNav",
        "ImGuiWindowFlags_NoCollapse",
    ]
    if scrollbar:
        names.append("ImGuiWindowFlags_AlwaysVerticalScrollbar")
    else:
        names.append("ImGuiWindowFlags_NoScrollbar")
        names.append("ImGuiWindowFlags_AlwaysAutoResize")
        names.append("ImGuiWindowFlags_NoInputs")
        names.append("ImGuiWindowFlags_NoMouseInputs")
    flags = 0
    for name in names:
        flags |= int(getattr(imgui, name, 0) or 0)
    return flags


def _begin_overlay(imgui, name, flags):
    try:
        return imgui.Begin(name, flags)
    except TypeError:
        return imgui.Begin(name)


def _begin_closable(imgui, name, flags, opened):
    result = None
    try:
        result = imgui.Begin(name, opened, flags)
    except TypeError:
        try:
            result = imgui.Begin(name, opened)
        except TypeError:
            result = imgui.Begin(name, flags)
    if isinstance(result, tuple):
        visible = bool(result[0])
        still_open = bool(result[1]) if len(result) > 1 and result[1] is not None else opened
        return visible, still_open
    return bool(result), opened


def _end_overlay(imgui):
    try:
        imgui.End()
    except Exception:
        pass


def _window_top(imgui):
    try:
        p = imgui.GetWindowPos()
        return float(p[1] if hasattr(p, "__len__") else p.y)
    except Exception:
        return None


def _wrapped_text_size(imgui, text, wrap_w):
    try:
        sz = imgui.CalcTextSize(text, False, wrap_w)
        return float(sz[0] if hasattr(sz, "__len__") else sz.x), float(sz[1] if hasattr(sz, "__len__") else sz.y)
    except Exception:
        pass
    try:
        sz = imgui.CalcTextSize(text)
        w = float(sz[0] if hasattr(sz, "__len__") else sz.x)
        h = float(sz[1] if hasattr(sz, "__len__") else sz.y)
        if w <= wrap_w:
            return w, h
        lines = max(1.0, math.ceil(w / max(wrap_w, 1.0)))
        return wrap_w, h * lines
    except Exception:
        return wrap_w, 24.0


def _draw_wrapped_text(imgui, text, wrap_w):
    try:
        imgui.PushTextWrapPos(imgui.GetCursorPosX() + wrap_w)
    except Exception:
        try:
            imgui.PushTextWrapPos(wrap_w)
        except Exception:
            pass
    try:
        wrapped = getattr(imgui, "TextWrapped", None)
        if wrapped is not None:
            wrapped(text)
        else:
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


def _key_down(imgui, io, key):
    try:
        enum = getattr(imgui, "ImGuiKey_" + key, None)
        if enum is not None:
            return bool(imgui.IsKeyDown(enum))
    except Exception:
        pass
    try:
        return bool(io.KeysDown[ord(key)])
    except Exception:
        return False


def _try_set(ps, name, *args):
    fn = getattr(ps, name, None)
    if fn is None:
        return
    try:
        fn(*args)
    except Exception:
        pass


def _try_set_vertex_normals(mesh, normals):
    """Polyscope shades from connectivity; pass C# normals if this build accepts them."""
    if mesh is None or not normals:
        return
    for name in ("set_vertex_normals", "update_vertex_normals", "set_smooth_normals"):
        fn = getattr(mesh, name, None)
        if fn is None:
            continue
        try:
            fn(normals)
            return
        except Exception:
            pass


def tick_cad_camera(ps, imgui, io=None, want_mouse=False):
    """Middle-drag orbit (or L+R), Shift+middle / right-drag pan, wheel zoom at cursor.

    Matches GeoScriptViewer: unconstrained screen-axis orbit. Camera-orbit signs are the
    opposite of C# model-spin so the scene follows the mouse.
    """
    if io is None:
        io = imgui.GetIO()
    if want_mouse:
        _CAD["orbiting"] = False
        return
    height = _display_height(io)
    if height < 1.0:
        height = 1.0
    middle = _mouse_down(imgui, io, 2)
    left = _mouse_down(imgui, io, 0)
    right = _mouse_down(imgui, io, 1)
    shift = _io_flag(io, "KeyShift")
    orbit = middle or (left and right)
    pan = (orbit and shift) or (right and middle) or (right and not left and not middle)
    dx, dy = _mouse_delta(io)
    wheel = _mouse_wheel(io)
    moving = abs(dx) > 0.0 or abs(dy) > 0.0
    if (orbit or pan) and moving:
        _CAD["orbiting"] = True
        ndx = dx / height
        ndy = dy / height
        if pan:
            _cad_pan(ps, ndx, ndy)
        else:
            _cad_orbit(ps, ndx, ndy)
    if not orbit and not pan:
        _CAD["orbiting"] = False
    if abs(wheel) > 0.0:
        _cad_zoom(ps, imgui, io, wheel)


def _cad_commit(ps, pos, center, up):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        cam.eye = (float(pos[0]), float(pos[1]), float(pos[2]))
        cam.center = (float(center[0]), float(center[1]), float(center[2]))
        cam.up = (float(up[0]), float(up[1]), float(up[2]))
        return
    try:
        ps.look_at_dir(pos, center, up, False)
    except TypeError:
        try:
            ps.look_at_dir(pos, center, up)
        except Exception:
            return
    setter = getattr(ps, "set_view_center_raw", None)
    if setter is None:
        setter = getattr(ps, "set_view_center_and_look_at", None)
    if setter is not None:
        try:
            setter(center)
        except TypeError:
            try:
                setter(center, False)
            except Exception:
                pass


def _cad_orbit(ps, ndx, ndy):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        cam.orbit(ndx, ndy)
        return
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None:
        return
    speed = 5.0
    yaw = -ndx * speed
    pitch = -ndy * speed
    rel = _vsub(pos, center)
    rel = _rodrigues(rel, up, yaw)
    rel = _rodrigues(rel, right, pitch)
    new_up = _rodrigues(up, right, pitch)
    _cad_commit(ps, _vadd(center, rel), center, new_up)


def _cad_look_at(pos, center, look):
    """Pivot on the look ray so a stale view-center cannot change orientation."""
    dist = _dot(_vsub(center, pos), look)
    if dist < 1e-6:
        dist = max(_vlen(_vsub(pos, center)), 1e-6)
    return _vadd(pos, _vmul(look, dist)), dist


def _cad_pan(ps, ndx, ndy):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        cam.pan(ndx, ndy)
        return
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None:
        return
    look_at, dist = _cad_look_at(pos, center, look)
    scale = max(dist, 1e-6)
    if _is_ortho(ps):
        scale = scale * math.tan(0.5 * math.radians(_view_fov_deg(ps)))
    offset = _vadd(_vmul(right, -2.0 * ndx * scale), _vmul(up, 2.0 * ndy * scale))
    _cad_commit(ps, _vadd(pos, offset), _vadd(look_at, offset), up)


def _cad_zoom(ps, imgui, io, wheel):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        pos = _mouse_pos(io)
        width, height = _pick_viewport(ps, imgui)
        if pos is None or width < 2.0 or height < 2.0:
            cam.zoom_at(wheel, 0.0, 0.0)
            return
        ndc_x = (2.0 * pos[0] / width) - 1.0
        ndc_y = 1.0 - (2.0 * pos[1] / height)
        cam.zoom_at(wheel, ndc_x, ndc_y)
        return
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None:
        return
    factor = math.pow(0.85, -wheel)
    hit = _zoom_anchor(ps, io, pos, center, look)
    if _is_ortho(ps):
        ps.set_vertical_fov_degrees(min(170.0, max(1.0, _view_fov_deg(ps) * factor)))
        planar = _vsub(hit, center)
        planar = _vsub(planar, _vmul(look, _dot(planar, look)))
        shift = _vmul(planar, 1.0 - factor)
        _cad_commit(ps, _vadd(pos, shift), _vadd(center, shift), up)
        return
    _cad_commit(
        ps,
        _vadd(hit, _vmul(_vsub(pos, hit), factor)),
        _vadd(hit, _vmul(_vsub(center, hit), factor)),
        up,
    )


def _is_ortho(ps):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        return bool(cam.ortho)
    try:
        return "ortho" in str(ps.get_view_projection_mode()).lower()
    except Exception:
        return False


def _view_fov_deg(ps):
    try:
        fov = float(ps.get_vertical_fov_degrees())
        if math.isfinite(fov) and 1.0 <= fov <= 170.0:
            return fov
    except Exception:
        pass
    return 45.0


def _zoom_anchor(ps, io, pos, center, look):
    mp = _mouse_pos(io)
    if mp is None:
        return center
    try:
        pick = ps.pick(screen_coords=mp)
        hit = getattr(pick, "is_hit", None)
        if hit is None:
            hit = getattr(pick, "isHit", False)
        if hit:
            p = getattr(pick, "position", None)
            if p is not None:
                return _as3(p)
    except Exception:
        pass
    return center


def _set_world_radius(obj, radius):
    if radius is None or not math.isfinite(radius) or radius <= 0.0:
        return
    try:
        obj.set_radius(radius, False)
    except TypeError:
        obj.set_radius(radius)


def _view_aspect(ps):
    try:
        size = ps.get_window_size()
        w = float(size[0] if hasattr(size, "__len__") else size.x)
        h = float(size[1] if hasattr(size, "__len__") else size.y)
        if w > 1.0 and h > 1.0:
            return w / h
    except Exception:
        pass
    return 16.0 / 9.0


def _scene_aabb(ps):
    try:
        low, high = ps.get_bounding_box()
        low = _as3(low)
        high = _as3(high)
        if _vlen(_vsub(high, low)) > 1e-18:
            return low, high
    except Exception:
        pass
    return None, None


def _fit_view(ps):
    """Recenter and zoom to the scene AABB. Keep the current camera rotation."""
    pos, _, look, up, right = _cad_frame(ps)
    if pos is None or look is None or up is None or right is None:
        return
    low, high = _scene_aabb(ps)
    if low is None:
        return
    center = _vmul(_vadd(low, high), 0.5)
    corners = [
        (x, y, z)
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ]
    aspect = max(_view_aspect(ps), 1e-6)
    margin = 1.04
    half_h = 1e-9
    half_w = 1e-9
    for corner in corners:
        rel = _vsub(corner, center)
        half_w = max(half_w, abs(_dot(rel, right)))
        half_h = max(half_h, abs(_dot(rel, up)))
    need_h = max(half_h, half_w / aspect) * margin
    dist = max(_dot(_vsub(center, pos), look), 1e-6)
    cam = getattr(ps, "camera", None)
    if cam is not None and bool(getattr(cam, "ortho", False)):
        new_pos = _vsub(center, _vmul(look, dist))
        _cad_commit(ps, new_pos, center, up)
        cam.zoom = max(need_h, 1e-6)
        _OVERLAY["cam"] = None
        return
    if _is_ortho(ps):
        tan_half = need_h / dist
        max_tan = math.tan(math.radians(85.0))
        if tan_half > max_tan:
            dist = need_h / math.tan(math.radians(45.0))
            tan_half = math.tan(math.radians(45.0))
        fov = 2.0 * math.degrees(math.atan(tan_half))
        ps.set_vertical_fov_degrees(min(170.0, max(1.0, fov)))
    else:
        fov = _view_fov_deg(ps)
        t_v = math.tan(math.radians(0.5 * fov))
        t_h = t_v * aspect
        dist = 1e-6
        for corner in corners:
            rel = _vsub(corner, center)
            along = _dot(rel, look)
            dist = max(dist, abs(_dot(rel, up)) / t_v - along)
            dist = max(dist, abs(_dot(rel, right)) / t_h - along)
        dist = max(dist, 1e-6) * margin
    new_pos = _vsub(center, _vmul(look, dist))
    _cad_commit(ps, new_pos, center, up)
    _OVERLAY["cam"] = None


def _selected_plane_context(obj, selected):
    if len(selected) != 1:
        return None
    name = selected[0]
    part = obj if callable(getattr(obj, "sketch_interactive", None)) else getattr(obj, "_part", None)
    resolver = obj if callable(getattr(obj, "plane_frame", None)) else part
    if resolver is None:
        return None
    try:
        frame = resolver.plane_frame(name)
    except Exception:
        return None
    if frame is None:
        return None
    return {
        "name": name,
        "frame": frame,
        "part": part,
        "emit_frame": resolver is obj and part is not obj,
    }


def _orient_view_to_frame(ps, frame):
    origin = _as3(frame.origin)
    normal = _normalize(_as3(frame.z))
    up = _normalize(_as3(frame.y))
    pos, center, _, _, _ = _cad_frame(ps)
    distance = _vlen(_vsub(pos, center)) if pos is not None else 1.0
    distance = max(distance, 1e-6)
    _cad_commit(ps, _vadd(origin, _vmul(normal, distance)), origin, up)
    _fit_view(ps)


def _toggle_projection(ps):
    getter = getattr(ps, "get_view_projection_mode", None)
    setter = getattr(ps, "set_view_projection_mode", None)
    if setter is None:
        return
    mode = "perspective"
    if getter is not None:
        try:
            mode = str(getter()).lower()
        except Exception:
            pass
    if "ortho" in mode:
        setter("perspective")
    else:
        setter("orthographic")


def _window_height(ps, imgui):
    height = 0.0
    try:
        size = ps.get_window_size()
        height = float(size[1] if hasattr(size, "__len__") else size.y)
    except Exception:
        pass
    if height < 32.0:
        height = _display_height(imgui.GetIO())
    if height < 32.0:
        return _OVERLAY["height"]
    _OVERLAY["height"] = height
    return height


def _world_per_pixel(ps, imgui):
    """World units spanned by one pixel at the look-at plane (perspective and ortho)."""
    prev = _OVERLAY["wpp"]
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None:
        return prev if prev is not None else 1e-4
    dist = _vlen(_vsub(pos, center))
    if (not math.isfinite(dist)) or dist < 1e-9:
        return prev if prev is not None else 1e-4
    height = _window_height(ps, imgui)
    fov_deg = 45.0
    try:
        fov_deg = float(ps.get_view_camera_parameters().get_fov_vertical_deg())
    except Exception:
        getter = getattr(ps, "get_vertical_fov_degrees", None)
        if getter is not None:
            try:
                fov_deg = float(getter())
            except Exception:
                pass
    if (not math.isfinite(fov_deg)) or fov_deg < 1.0 or fov_deg > 170.0:
        fov_deg = 45.0
    tan_half = math.tan(0.5 * math.radians(fov_deg))
    if _is_ortho(ps):
        try:
            # Match Polyscope's orthographic projection bounds.
            half = 2.0 * float(ps.get_length_scale()) * tan_half
        except Exception:
            half = dist * tan_half
    else:
        half = dist * tan_half
    if (not math.isfinite(half)) or half <= 0.0:
        return prev if prev is not None else 1e-4
    wpp = half / (0.5 * height)
    if (not math.isfinite(wpp)) or wpp <= 0.0:
        return prev if prev is not None else 1e-4
    if prev is not None:
        if wpp > prev * 8.0:
            wpp = prev * 8.0
        elif wpp < prev / 8.0:
            wpp = prev / 8.0
    _OVERLAY["wpp"] = wpp
    return wpp


def _diamond_verts(center, right, up, half):
    """Camera-facing square rotated 45° (vertices along right/up)."""
    d = half * math.sqrt(2.0)
    return [
        _vadd(center, _vmul(up, d)),
        _vadd(center, _vmul(right, -d)),
        _vadd(center, _vmul(up, -d)),
        _vadd(center, _vmul(right, d)),
    ]


def _rect_verts(center, right, up, half):
    """Camera-facing axis-aligned square (sides along right/up)."""
    return [
        _vadd(center, _vadd(_vmul(right, half), _vmul(up, half))),
        _vadd(center, _vadd(_vmul(right, -half), _vmul(up, half))),
        _vadd(center, _vadd(_vmul(right, -half), _vmul(up, -half))),
        _vadd(center, _vadd(_vmul(right, half), _vmul(up, -half))),
    ]


def _update_screen_overlays(ps, imgui, registered, overlay):
    wpp = _world_per_pixel(ps, imgui)
    if wpp is None or not math.isfinite(wpp) or wpp <= 0.0:
        return
    pos, center, look, up, right = _cad_frame(ps)
    if look is None or up is None or right is None:
        return
    edge_r = wpp * (_EDGE_PX_ORTHO if _is_ortho(ps) else _EDGE_PX_PERSPECTIVE)
    point_r = wpp * (_POINT_PX_ORTHO if _is_ortho(ps) else _POINT_PX_PERSPECTIVE)
    curve_pull = _vmul(look, -max(3.0 * wpp, 1e-9))
    point_pull = _vmul(look, -max(5.0 * wpp, 1e-9))
    prev = _OVERLAY.get("cam")
    changed = prev is None
    if prev is not None:
        pl, pu, pr, pe, pp = prev
        changed = (
            _vlen(_vsub(look, pl)) > 1e-6
            or _vlen(_vsub(up, pu)) > 1e-6
            or _vlen(_vsub(right, pr)) > 1e-6
            or abs(edge_r - pe) > 0.02 * max(pe, 1e-12)
            or abs(point_r - pp) > 0.02 * max(pp, 1e-12)
        )
    if not changed:
        return
    _OVERLAY["cam"] = (look, up, right, edge_r, point_r)
    _OVERLAY["edge_r"] = edge_r
    _OVERLAY["point_r"] = point_r
    for kind, obj in registered:
        if kind == "curve" or kind == "normal":
            _set_world_radius(obj, edge_r)
    points = overlay.get("points")
    if points is not None and overlay.get("point_pos"):
        try:
            points.update_vertex_positions(
                _diamond_batch_verts(overlay["point_pos"], right, up, point_r, point_pull)
            )
        except Exception:
            pass
    curve_pick = overlay.get("curve_pick")
    if curve_pick is not None and overlay.get("curve_edges"):
        try:
            curve_pick.update_vertex_positions(
                _curve_pick_verts(
                    overlay["curve_nodes"], overlay["curve_edges"], look, up, edge_r, curve_pull
                )
            )
        except Exception:
            pass
    curve_hl = overlay.get("curve_hl")
    sel_edges = overlay.get("hl_curve_edges")
    if curve_hl is not None and sel_edges:
        try:
            look_hl, up_hl, half_hl, pull_hl = _curve_hl_frame(ps)
            curve_hl.update_vertex_positions(
                _curve_pick_verts(overlay["curve_nodes"], sel_edges, look_hl, up_hl, half_hl, pull_hl)
            )
        except Exception:
            pass


def _cad_frame(ps):
    cam = getattr(ps, "camera", None)
    if cam is not None:
        look, up, right = cam.frame()
        return cam.eye, cam.center, look, up, right
    try:
        params = ps.get_view_camera_parameters()
        pos = _as3(params.get_position())
        look = _normalize(_as3(params.get_look_dir()))
        up = _normalize(_as3(params.get_up_dir()))
        right = _normalize(_as3(params.get_right_dir()))
        center = _as3(ps.get_view_center())
        return pos, center, look, up, right
    except Exception:
        return None, None, None, None, None


def camera_ray(ps, imgui, screen=None):
    """World ray through a screen pixel. Shared by viewer and sketch picking."""
    if screen is None:
        screen = _mouse_pos(imgui.GetIO())
    if screen is None:
        return None, None
    mx, my = float(screen[0]), float(screen[1])
    pos, center, look, up, right = _cad_frame(ps)
    if pos is None or center is None or look is None or up is None or right is None:
        return None, None
    width, height = _pick_viewport(ps, imgui)
    if width < 2.0 or height < 2.0:
        return None, None
    dist = max(_vlen(_vsub(pos, center)), 1e-6)
    fov, aspect = _camera_fov_aspect(ps, width, height)
    tan_half = math.tan(0.5 * math.radians(max(fov, 1.0)))
    ndc_x = (2.0 * mx / width) - 1.0
    ndc_y = 1.0 - (2.0 * my / height)
    if _is_ortho(ps):
        try:
            half_h = 2.0 * tan_half * float(ps.get_length_scale())
        except Exception:
            half_h = dist * tan_half
        half_w = half_h * aspect
        offset = _vadd(_vmul(right, ndc_x * half_w), _vmul(up, ndc_y * half_h))
        return _vadd(pos, offset), look

    native_ray = getattr(ps, "screen_coords_to_world_ray", None)
    if native_ray is not None:
        try:
            return pos, _normalize(_as3(native_ray((mx, my))))
        except Exception:
            pass
    half_h = dist * tan_half
    half_w = half_h * aspect
    offset = _vadd(_vmul(right, ndc_x * half_w), _vmul(up, ndc_y * half_h))
    direction = _normalize(_vsub(_vadd(center, offset), pos))
    return pos, direction


def _camera_fov_aspect(ps, width, height):
    fov = 45.0
    aspect = width / height
    try:
        mode = str(ps.get_view_projection_mode()).lower()
    except Exception:
        mode = ""
    try:
        params = ps.get_view_camera_parameters()
        cam_aspect = float(params.get_aspect())
        if math.isfinite(cam_aspect) and cam_aspect > 1e-6:
            aspect = cam_aspect
        if "ortho" not in mode:
            cam_fov = float(params.get_fov_vertical_deg())
            if math.isfinite(cam_fov) and 1.0 <= cam_fov <= 170.0:
                fov = cam_fov
    except Exception:
        pass
    if "ortho" in mode:
        getter = getattr(ps, "get_vertical_fov_degrees", None)
        if getter is not None:
            try:
                cam_fov = float(getter())
                if math.isfinite(cam_fov) and 1.0 <= cam_fov <= 170.0:
                    fov = cam_fov
            except Exception:
                pass
    return fov, aspect


def _pick_viewport(ps, imgui):
    """Window dimensions used by Polyscope screen coordinates."""
    try:
        size = ps.get_window_size()
        win_w = float(size[0] if hasattr(size, "__len__") else size.x)
        win_h = float(size[1] if hasattr(size, "__len__") else size.y)
        if win_w >= 2.0 and win_h >= 2.0:
            return win_w, win_h
    except Exception:
        pass
    try:
        size = imgui.GetIO().DisplaySize
        disp_w = float(size[0] if hasattr(size, "__len__") else size.x)
        disp_h = float(size[1] if hasattr(size, "__len__") else size.y)
        if disp_w >= 2.0 and disp_h >= 2.0:
            return disp_w, disp_h
    except Exception:
        pass
    return 0.0, 0.0


def _left_click_released(imgui, io):
    """Pick only on a click that did not drag (orbit uses middle; left-drag must not select)."""
    if _CAD["orbiting"]:
        _CAD["press"] = None
        return None
    pos = _mouse_pos(io)
    if _mouse_clicked(imgui, io, 0):
        _CAD["press"] = pos
        return None
    if not _mouse_released(imgui, io, 0):
        return None
    press = _CAD["press"]
    _CAD["press"] = None
    if press is None or pos is None:
        return None
    dx = pos[0] - press[0]
    dy = pos[1] - press[1]
    if dx * dx + dy * dy > _PICK_DRAG_PX * _PICK_DRAG_PX:
        return None
    return pos


def _want_mouse(imgui, io):
    try:
        if bool(io.WantCaptureMouse):
            return True
    except Exception:
        pass
    try:
        flags = int(getattr(imgui, "ImGuiHoveredFlags_AnyWindow", 0) or 0)
        if flags and imgui.IsWindowHovered(flags):
            return True
    except Exception:
        pass
    try:
        return bool(imgui.IsAnyItemHovered())
    except Exception:
        return False


def _io_flag(io, name):
    try:
        return bool(getattr(io, name))
    except Exception:
        return False


def _mouse_down(imgui, io, button):
    try:
        return bool(imgui.IsMouseDown(button))
    except Exception:
        pass
    try:
        return bool(io.MouseDown[button])
    except Exception:
        return False


def _mouse_clicked(imgui, io, button):
    try:
        return bool(imgui.IsMouseClicked(button))
    except Exception:
        pass
    try:
        return bool(io.MouseClicked[button])
    except Exception:
        return False


def _mouse_released(imgui, io, button):
    try:
        return bool(imgui.IsMouseReleased(button))
    except Exception:
        pass
    try:
        return bool(io.MouseReleased[button])
    except Exception:
        return False


def _mouse_pos(io):
    try:
        p = io.MousePos
        return (float(p[0] if hasattr(p, "__len__") else p.x), float(p[1] if hasattr(p, "__len__") else p.y))
    except Exception:
        return None


def _mouse_delta(io):
    try:
        d = io.MouseDelta
        return (float(d[0] if hasattr(d, "__len__") else d.x), float(d[1] if hasattr(d, "__len__") else d.y))
    except Exception:
        return (0.0, 0.0)


def _mouse_wheel(io):
    try:
        return float(io.MouseWheel)
    except Exception:
        return 0.0


def _display_height(io):
    try:
        size = io.DisplaySize
        return float(size[1] if hasattr(size, "__len__") else size.y)
    except Exception:
        return 800.0


def _as3(v):
    return (float(v[0]), float(v[1]), float(v[2]))


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vlen(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _normalize(a):
    n = _vlen(a)
    if n < 1e-15:
        return (0.0, 0.0, 1.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _rodrigues(v, axis, angle):
    axis = _normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    return _vadd(_vadd(_vmul(v, c), _vmul(_cross(axis, v), s)), _vmul(axis, _dot(axis, v) * (1.0 - c)))


def _face_normal_segments(patches):
    """One segment per triangle: centroid to centroid + unit normal * 0.01 * AABB diagonal."""
    lo = [None, None, None]
    hi = [None, None, None]
    for patch in patches:
        for v in patch["vertices"]:
            for i in range(3):
                if lo[i] is None or v[i] < lo[i]:
                    lo[i] = v[i]
                if hi[i] is None or v[i] > hi[i]:
                    hi[i] = v[i]
    if lo[0] is None:
        return [], []
    diag = math.sqrt((hi[0] - lo[0]) ** 2 + (hi[1] - lo[1]) ** 2 + (hi[2] - lo[2]) ** 2)
    length = 0.01 * max(diag, 1e-12)
    pts = []
    edges = []
    for patch in patches:
        verts = patch["vertices"]
        normals = patch.get("normals")
        use_n = bool(normals) and len(normals) == len(verts)
        for a, b, c in patch["faces"]:
            va, vb, vc = verts[a], verts[b], verts[c]
            mx = (va[0] + vb[0] + vc[0]) / 3.0
            my = (va[1] + vb[1] + vc[1]) / 3.0
            mz = (va[2] + vb[2] + vc[2]) / 3.0
            if use_n:
                na, nb, nc = normals[a], normals[b], normals[c]
                nx = na[0] + nb[0] + nc[0]
                ny = na[1] + nb[1] + nc[1]
                nz = na[2] + nb[2] + nc[2]
            else:
                ux, uy, uz = vb[0] - va[0], vb[1] - va[1], vb[2] - va[2]
                vx, vy, vz = vc[0] - va[0], vc[1] - va[1], vc[2] - va[2]
                nx = uy * vz - uz * vy
                ny = uz * vx - ux * vz
                nz = ux * vy - uy * vx
            nlen = math.sqrt(nx * nx + ny * ny + nz * nz)
            if nlen < 1e-18:
                continue
            s = length / nlen
            i = len(pts)
            pts.append((mx, my, mz))
            pts.append((mx + nx * s, my + ny * s, mz + nz * s))
            edges.append((i, i + 1))
    return pts, edges


def _as_scene(obj):
    from .display import DisplayScene, decode

    if isinstance(obj, DisplayScene):
        return obj
    native = getattr(obj, "_n", obj)
    dump = getattr(native, "dump_display", None)
    if dump is None:
        raise TypeError("show() expects a Part, Solid, Assembly, or DisplayScene")
    return decode(dump())


def _unique(used, name):
    base = name or "unnamed"
    candidate = base
    n = 2
    while candidate in used:
        candidate = "{0}_{1}".format(base, n)
        n += 1
    used.add(candidate)
    return candidate


def _import_polyscope():
    raise ImportError("Polyscope was removed. Use camber.host.Viewer (pyglet + imgui).")
