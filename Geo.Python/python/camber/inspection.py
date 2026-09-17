"""Capped section snapshots and point-to-point inspection in model units."""

from dataclasses import dataclass
import math

from .api import Assembly, Frame, Part, Solid, _require
from .geom import RayHit
from .vec import vec3


@dataclass(frozen=True)
class Measurement:
    """Straight distance between two world points, in model units.

    This is a point-to-point measurement, not a minimum surface gap or an
    inferred wall thickness. Endpoints are immutable coordinate tuples.
    """

    start: tuple
    end: tuple
    length: float
    label: str = "Distance"


def _point(value):
    point = tuple(float(c) for c in value)
    if len(point) != 3 or not all(math.isfinite(c) for c in point):
        raise ValueError("expected a finite world-space 3-vector")
    return point


class Section:
    """Immutable geometry snapshot with a mutable list of inspection dimensions.

    Create with :func:`section`; view with ``show`` or ``render_views``. Press
    M in its interactive view, then pick two surfaces, to append a measurement.
    All lengths use the model's coordinate units; no unit conversion is inferred.
    """

    def __init__(self, native, plane):
        self._n = native
        self._plane = Frame(plane.origin, plane.x, plane.y, plane.z)
        self._measurements = []

    @property
    def plane(self):
        """A copy of the section coordinate frame."""
        p = self._plane
        return Frame(p.origin, p.x, p.y, p.z)

    @property
    def measurements(self):
        return tuple(self._measurements)

    def raycast(self, origin, direction):
        """Nearest hit on the retained capped geometry, or None.

        Uses the existing kernel ray cast. ``RayHit.t`` retains that API's
        segment-parameter convention; use ``hit.point`` for measurements.
        """
        origin, direction = _point(origin), _point(direction)
        if not any(direction):
            raise ValueError("ray direction must be nonzero")
        packed = _require(self._n, "raycast")(*origin, *direction)
        if not packed:
            return None
        values = packed.split()
        return RayHit(vec3(tuple(map(float, values[:3]))), vec3(tuple(map(float, values[3:6]))),
                      float(values[6]), int(values[7]), int(values[8]))

    def measure(self, start, end, *, label="Distance"):
        """Add and return a straight point-to-point measurement.

        Accepts RayHit objects, world 3-vectors, or plane-local (x, y) pairs.
        Two arbitrary surface picks do not establish minimum wall thickness.
        """
        def world(value):
            if isinstance(value, RayHit):
                value = value.point
            value = tuple(value)
            if len(value) == 2:
                p = self._plane
                value = p.origin + p.x * float(value[0]) + p.y * float(value[1])
            return _point(value)
        a, b = world(start), world(end)
        result = Measurement(a, b, math.dist(a, b), str(label))
        self._measurements.append(result)
        return result


def section(obj, plane="XY", *, offset=0):
    """Return a capped snapshot retaining plane-local Z <= 0.

    ``obj`` is a Solid, Part or Assembly in its current pose. The source is
    neither solved nor changed. ``plane`` accepts a Frame or XY/XZ/YZ:
    XY has +Z normal, XZ has -Y normal, YZ has +X normal. ``offset`` shifts
    the plane along that normal, in model units. Occurrence names are kept;
    generated cut faces end in ``section_cap`` and accept normal color rules.

    Example::

        cut = section(bike, "XZ")
        cut.measure((0, 0), (25, 0), label="Reference")
        render_views(cut, "bike_section.png")
    """
    if isinstance(plane, str):
        frames = {"XY": Frame(), "XZ": Frame(x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)),
                  "YZ": Frame(x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))}
        try:
            plane = frames[plane.upper()]
        except KeyError:
            raise ValueError("plane must be XY, XZ, YZ or a Frame") from None
    if not isinstance(plane, Frame):
        raise TypeError("plane must be XY, XZ, YZ or a Frame")
    offset = float(offset)
    if not math.isfinite(offset):
        raise ValueError("offset must be finite")
    for vector in (plane.origin, plane.x, plane.y, plane.z):
        _point(vector)
    plane = plane.offset(plane.z * offset)
    if isinstance(obj, Solid):
        native = _require(obj._part._n, "section_solid")(obj._n, plane._native())
    elif isinstance(obj, (Part, Assembly)):
        native = _require(obj._n, "section")(plane._native())
    else:
        raise TypeError("section requires a Solid, Part or Assembly")
    return Section(native, plane)


def _scale_bar(span):
    """Readable 1/2/5 reference length no greater than one fifth of a view."""
    target = span / 5
    base = 10 ** math.floor(math.log10(target))
    return max(v * base for v in (1, 2, 5) if v * base <= target)


def _place_label(desired, size, occupied, viewport):
    """Place a screen label near its anchor without covering earlier labels.

    A finite set of positions above/below existing boxes keeps layout bounded.
    If the viewport cannot fit all labels, choose the least covered position.
    """
    width, height = size
    screen_width, screen_height = viewport
    x = max(4, min(desired[0], screen_width-width-4))
    top = max(4, screen_height-height-4)
    clamp = lambda value: max(4, min(value, top))
    candidates = [clamp(desired[1])]
    for _, y, _, h in occupied:
        candidates.extend((clamp(y+h+5), clamp(y-height-5)))

    def score(y):
        overlap = sum(max(0, min(x+width, bx+bw)-max(x, bx)) *
                      max(0, min(y+height, by+bh)-max(y, by))
                      for bx, by, bw, bh in occupied)
        return overlap, abs(y-desired[1])

    y = min(candidates, key=score)
    occupied.append((x, y, width, height))
    return x, y


def _draw_annotations(viewer):
    """Shared live/capture overlay; coordinates use logical window pixels."""
    inspection = getattr(viewer, "inspection", None)
    if inspection is None:
        return
    cam, g = viewer.camera, viewer._pyglet
    width, height = viewer.window.width, viewer.window.height
    look, up, right = cam.frame()
    np = viewer._np
    look, up, right = map(np.asarray, (look, up, right))
    eye = np.asarray(cam.eye)
    shapes = []
    color = (35, 60, 95, 255)
    occupied = [(0, height-42, width, 42), (0, 0, 200, 60)]

    def project(point):
        delta = np.asarray(point) - eye
        depth = float(delta @ look)
        if depth <= 0:
            return None
        scale = cam.zoom if cam.ortho else depth * math.tan(cam.fov / 2)
        return (width * (.5 + float(delta @ right) / (2 * scale * cam.aspect)),
                height * (.5 + float(delta @ up) / (2 * scale)))

    def line(a, b):
        shapes.append(g.shapes.Line(*a, *b, thickness=3.5, color=(255, 255, 255, 230)))
        shapes.append(g.shapes.Line(*a, *b, thickness=1.5, color=color))

    def label(text, x, y, anchor=None):
        caption = g.text.Label(text, x=x, y=y, font_size=11, color=color)
        w, h = caption.content_width+6, caption.content_height+4
        bx, by = x-3, y-3
        if anchor is not None:
            bx, by = _place_label((bx, by), (w, h), occupied, (width, height))
            if abs(bx-(x-3)) + abs(by-(y-3)) > 1:
                line(anchor, (max(bx, min(anchor[0], bx+w)),
                              max(by, min(anchor[1], by+h))))
            caption.x, caption.y = bx+3, by+3
        shapes.append(g.shapes.Rectangle(bx, by, w, h, color=(255, 255, 255, 230)))
        shapes.append(caption)

    viewer._gl.glDisable(viewer._gl.GL_DEPTH_TEST)
    for measurement in inspection.measurements:
        a, b = project(measurement.start), project(measurement.end)
        if a is None or b is None:
            continue
        line(a, b)
        for point in (a, b):
            shapes.append(g.shapes.Circle(*point, radius=3, color=color))
        label(f"{measurement.label}: {measurement.length:.6g} model units",
              (a[0]+b[0])/2+7, (a[1]+b[1])/2+8,
              anchor=((a[0]+b[0])/2, (a[1]+b[1])/2))
    if cam.ortho:
        length = _scale_bar(2 * cam.zoom * cam.aspect)
        pixels = width * length / (2 * cam.zoom * cam.aspect)
        line((24, 28), (24+pixels, 28))
        line((24, 24), (24, 32))
        line((24+pixels, 24), (24+pixels, 32))
        label(f"{length:.6g} model units", 24, 40)
    try:
        for drawable in shapes:
            drawable.draw()
    finally:
        for drawable in shapes:
            drawable.delete()
