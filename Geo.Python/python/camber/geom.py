"""Kernel geometry helpers.

Free functions take sequences of points (tuples, ``vec2`` / ``vec3``, or an
``(N, 2)`` / ``(N, 3)`` array) the same way numpy accepts array-likes. Solids
and sketches keep methods (``solid.mesh()``, ``sketch.polylines()``) so the
object API stays with the rest of camber.
"""

from .api import _native_mod, _require
from .vec import _xy, _xyz, vec2, vec3


class RayHit(object):
    """Nearest hit from ``Part.raycast``. ``t`` is 0 at the origin and 1 at the
    far end of the cast segment used by the kernel."""

    __slots__ = ("point", "normal", "t", "triangle_index", "group_id")

    def __init__(self, point, normal, t, triangle_index, group_id):
        self.point = point
        self.normal = normal
        self.t = float(t)
        self.triangle_index = int(triangle_index)
        self.group_id = int(group_id)

    def __repr__(self):
        return "RayHit(point={0}, t={1})".format(self.point, self.t)


def _G():
    t = _native_mod()
    if "NativeGeom" not in t:
        raise AttributeError(
            "this camber wheel has no NativeGeom; rebuild with Geo.Python/publish-wheel.ps1"
        )
    return t["NativeGeom"]


def _is_xy(value):
    if hasattr(value, "shape"):
        shape = getattr(value, "shape")
        if shape is not None and len(shape) >= 1:
            return False
    try:
        _xy(value)
        return True
    except (TypeError, IndexError, ValueError):
        return False


def _as_points2(seq):
    """Coerce an (N, 2)-like sequence to a list of ``vec2``."""
    if seq is None:
        return []
    shape = getattr(seq, "shape", None)
    if shape is not None and len(shape) == 2:
        return [vec2(float(row[0]), float(row[1])) for row in seq]
    return [vec2(*_xy(p)) for p in seq]


def _as_points3(seq):
    """Coerce an (N, 3)-like sequence to a list of ``vec3``."""
    if seq is None:
        return []
    shape = getattr(seq, "shape", None)
    if shape is not None and len(shape) == 2:
        return [vec3(float(row[0]), float(row[1]), float(row[2])) for row in seq]
    return [vec3(*_xyz(p)) for p in seq]


def _as_loops2(loops, *holes):
    """One loop, ``outer, hole, ...``, or a list of loops."""
    extra = [_as_points2(h) for h in holes]
    if loops is None:
        return extra
    shape = getattr(loops, "shape", None)
    if shape is not None and len(shape) == 2:
        return [_as_points2(loops)] + extra
    if extra:
        return [_as_points2(loops)] + extra
    seq = list(loops)
    if not seq:
        return []
    if _is_xy(seq[0]):
        return [_as_points2(seq)]
    return [_as_points2(loop) for loop in seq]


def _fmt2(p):
    return "{0:.17g} {1:.17g}".format(p.x, p.y)


def _fmt3(p):
    return "{0:.17g} {1:.17g} {2:.17g}".format(p.x, p.y, p.z)


def _pack_working_volume(working_volume=None, slices=None):
    """None → auto AABB; a Part → that part's lattice; ``(low, high)`` → explicit box.

    ``low`` / ``high`` may be 2D or 3D. ``slices`` is lattice divisions along the
    longest axis (default 1_000_000), ignored when ``working_volume`` is a Part.
    """
    sl = 0 if slices is None else int(slices)
    if working_volume is None:
        return "auto {0}\n".format(sl)
    native = getattr(working_volume, "_n", None)
    fn = getattr(native, "pack_working_volume", None) if native is not None else None
    if callable(fn):
        return fn()
    lo, hi = working_volume
    try:
        lx, ly, lz = _xyz(lo)
        hx, hy, hz = _xyz(hi)
    except (TypeError, IndexError, ValueError):
        lx, ly = _xy(lo)
        hx, hy = _xy(hi)
        lz = 0.0
        hz = 0.0
    n = sl if sl > 1 else 1000000
    return "{0}\n{1:.17g} {2:.17g} {3:.17g}\n{4:.17g} {5:.17g} {6:.17g}\n".format(
        n, lx, ly, lz, hx, hy, hz)


def _pack_loops2(loops, *holes):
    packed = _as_loops2(loops, *holes)
    lines = [str(len(packed))]
    for loop in packed:
        lines.append(str(len(loop)))
        for p in loop:
            lines.append(_fmt2(p))
    return "\n".join(lines) + "\n"


def _pack_points2(points):
    pts = _as_points2(points)
    lines = [str(len(pts))]
    for p in pts:
        lines.append(_fmt2(p))
    return "\n".join(lines) + "\n"


def _pack_points3(points):
    pts = _as_points3(points)
    lines = [str(len(pts))]
    for p in pts:
        lines.append(_fmt3(p))
    return "\n".join(lines) + "\n"


def _pack_triangles(triangles):
    tris = list(triangles)
    lines = [str(len(tris))]
    for tri in tris:
        lines.append("{0} {1} {2}".format(int(tri[0]), int(tri[1]), int(tri[2])))
    return "\n".join(lines) + "\n"


def _line_iter(packed):
    for line in packed.replace("\r", "\n").split("\n"):
        if line.strip():
            yield line.strip()


def _unpack_loops2(packed):
    it = _line_iter(packed)
    try:
        n_loops = int(next(it))
    except StopIteration:
        return []
    loops = []
    for _ in range(n_loops):
        n = int(next(it))
        loop = []
        for _k in range(n):
            x, y = next(it).split()
            loop.append(vec2(float(x), float(y)))
        loops.append(loop)
    return loops


def _unpack_indexed2(packed):
    it = _line_iter(packed)
    n = int(next(it))
    points = []
    for _ in range(n):
        x, y = next(it).split()
        points.append(vec2(float(x), float(y)))
    nt = int(next(it))
    tris = []
    for _ in range(nt):
        a, b, c = next(it).split()
        tris.append((int(a), int(b), int(c)))
    return points, tris


def _unpack_indexed3(packed):
    it = _line_iter(packed)
    n = int(next(it))
    points = []
    for _ in range(n):
        x, y, z = next(it).split()
        points.append(vec3(float(x), float(y), float(z)))
    nt = int(next(it))
    tris = []
    for _ in range(nt):
        a, b, c = next(it).split()
        tris.append((int(a), int(b), int(c)))
    return points, tris


def _parse_ray_hit(packed):
    if packed is None or str(packed).strip() == "":
        return None
    parts = str(packed).split()
    if len(parts) < 9:
        return None
    return RayHit(
        vec3(float(parts[0]), float(parts[1]), float(parts[2])),
        vec3(float(parts[3]), float(parts[4]), float(parts[5])),
        float(parts[6]),
        int(parts[7]),
        int(parts[8]),
    )


def triangulate(loops, *holes, working_volume=None, slices=None):
    """Fill polygons. Nested holes and islands are classified by a polygon tree.

    Pass one outer loop plus holes, or a list of closed loops (nesting is
    inferred). Returns ``(points, triangles)``. Predicates use the kernel
    lattice (Rat2 from Rat3). ``working_volume`` is auto-fit to the points,
    a ``(low, high)`` box, or a ``Part``. For a whole sketch use
    ``Sketch.triangulate()``.
    """
    vol = _pack_working_volume(working_volume, slices)
    packed = _require(_G(), "triangulate")(_pack_loops2(loops, *holes), vol)
    return _unpack_indexed2(packed)


def signed_area(loop, working_volume=None, slices=None):
    """Shoelace area on the working-volume lattice, converted back to world units.
    Positive is CCW. Pass the same ``working_volume`` as ``triangulate`` when comparing.
    """
    vol = _pack_working_volume(working_volume, slices)
    return float(_require(_G(), "signed_area")(_pack_loops2(loop), vol))


def is_ccw(loop, working_volume=None, slices=None):
    """True if ``signed_area(loop)`` is positive (CCW in sketch XY)."""
    return signed_area(loop, working_volume=working_volume, slices=slices) > 0.0


def point_in_polygon(loop, point, on_edge=True, working_volume=None, slices=None):
    """True if ``point`` is inside ``loop``. Boundary counts as inside unless
    ``on_edge=False``."""
    x, y = _xy(point)
    vol = _pack_working_volume(working_volume, slices)
    code = int(_require(_G(), "point_in_polygon")(_pack_loops2(loop), float(x), float(y), vol))
    if code == 0:
        return True
    if code == 2:
        return bool(on_edge)
    return False


def convex_hull(points, working_volume=None, slices=None):
    """2D convex hull as a CCW list of ``vec2`` (no closing duplicate)."""
    vol = _pack_working_volume(working_volume, slices)
    packed = _require(_G(), "convex_hull")(_pack_points2(points), vol)
    loops = _unpack_loops2(packed)
    if not loops:
        return []
    return loops[0]


def tessellate_bezier(controls, max_deviation=0.01):
    """Adaptive polyline for a 2D Bezier given control points."""
    packed = _require(_G(), "tessellate_bezier")(_pack_points2(controls), float(max_deviation))
    loops = _unpack_loops2(packed)
    if not loops:
        return []
    return loops[0]


def text_outlines(text, origin=(0, 0), family="Arial", em_size=0.2, bold=False, italic=False):
    """Tessellated glyph contours as a list of loops (``vec2``)."""
    x, y = _xy(origin)
    flags = 0
    if bold:
        flags |= 1
    if italic:
        flags |= 2
    packed = _require(_G(), "text_outlines")(
        str(text), float(x), float(y), family or "", float(em_size), int(flags)
    )
    return _unpack_loops2(packed)
