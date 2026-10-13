"""Capped section snapshots and point-to-point inspection in model units."""

from dataclasses import dataclass
from itertools import product
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


class _ViewerInspection:
    """Viewer-owned measurement state and section controls."""

    def __init__(self, viewer, source=None):
        self.viewer = viewer
        self.measurements = source._measurements if isinstance(source, Section) else []
        self.open = not isinstance(source, Section)
        self.axis = "Z"
        self.normal = (0.0, 0.0, 1.0)
        self.offset = 0.0
        self.azimuth = 0.0
        self.elevation = 90.0
        self.reverse = False
        self.error = ""
        self.section_error = ""
        self._section_initialized = False
        self._geometry = {}
        self._visible = ()

    def measure_selected(self, names):
        from .measure import Geometry, closest_entities, named_geometry
        if len(names) != 2:
            raise ValueError("select exactly two points, edges or faces")
        scenes = self.viewer.visible_part_packeds()
        visible = tuple(id(scene) for scene in scenes)
        if visible != self._visible:
            self._geometry.clear()
            self._visible = visible
        for name in names:
            if name not in self._geometry:
                self._geometry[name] = Geometry(named_geometry(scenes, name))
        result = closest_entities(*(self._geometry[name] for name in names), *names)
        measurement = Measurement(result.first, result.second, result.length, "Minimum distance")
        self.measurements.append(measurement)
        return measurement

    def set_section(self, normal, offset, *, reverse=False):
        normal = _point(normal)
        length = math.hypot(*normal)
        if length == 0 or not math.isfinite(length) or not math.isfinite(offset):
            raise ValueError("section needs a nonzero normal and finite offset")
        self.normal = tuple(c/length for c in normal)
        self.offset, self.reverse = float(offset)/length, bool(reverse)
        horizontal = math.hypot(*self.normal[:2])
        if horizontal > 1e-15:
            self.azimuth = math.degrees(math.atan2(self.normal[1], self.normal[0]))
        self.elevation = math.degrees(math.atan2(self.normal[2], horizontal))
        self.axis = next((axis for i, axis in enumerate("XYZ")
                          if self.normal == tuple(float(j == i) for j in range(3))), "Custom")
        self._section_initialized = True
        self._update_plane()

    def _update_plane(self):
        sign = -1 if self.reverse else 1
        self.viewer.section_plane = (tuple(c*sign for c in self.normal), self.offset*sign)

    def _limits(self):
        lo, hi = self.viewer.get_bounding_box()
        values = [sum(self.normal[i]*p[i] for i in range(3))
                  for p in product(*zip(lo, hi))]
        return min(values), max(values)

    def _orient_section(self, normal, origin=None):
        """Rotate about the model center, retaining the signed section position."""
        lo, hi = self.viewer.get_bounding_box()
        center = tuple((a+b)*.5 for a, b in zip(lo, hi))
        position = self.offset-sum(a*b for a, b in zip(self.normal, center)) \
            if self.viewer.section_plane is not None else 0.0
        normal = _point(normal)
        length = math.hypot(*normal)
        if length == 0 or not math.isfinite(length):
            raise ValueError("section needs a nonzero normal")
        normal = tuple(c/length for c in normal)
        offset = sum(a*b for a, b in zip(normal, origin if origin is not None else center))
        self.set_section(normal, offset + (position if origin is None else 0), reverse=self.reverse)

    def _draw_section(self, ui, selected_plane):
        enabled = self.viewer.section_plane is not None
        changed, enabled = ui.Checkbox("Enable cross section", enabled)
        if changed:
            if enabled:
                if self._section_initialized:
                    self._update_plane()
                else:
                    self._orient_section(self.normal)
            else:
                self.viewer.section_plane = None
        if not enabled:
            ui.TextUnformatted("Enable to reveal the inside of the model.")
            return
        ui.TextUnformatted("Plane normal")
        changed, axis = ui.ComboBox("Normal axis", self.axis, ("X", "Y", "Z", "Custom"))
        if changed:
            if axis != "Custom":
                self._orient_section(tuple(float(i == "XYZ".index(axis)) for i in range(3)))
            self.axis = axis
        if ui.Button("Align to view"):
            self._orient_section(self.viewer.camera.frame()[0])
            self.section_error = ""
        ui.SameLine()
        if ui.Button("Use selected face"):
            context = selected_plane()
            if context is None:
                self.section_error = "Select one planar face first."
            else:
                frame = context["frame"]
                self._orient_section(tuple(frame.z), tuple(frame.origin))
                self.section_error = ""
        if self.section_error:
            ui.TextUnformatted(self.section_error)
        ui.TextUnformatted("Azimuth (degrees)")
        changed_a, azimuth = ui.SliderFloat("##section_azimuth", self.azimuth, -180, 180, width=190)
        ui.SameLine()
        ui.PushItemWidth(80)
        typed_a, azimuth = ui.InputDouble("##section_azimuth_value", azimuth, fmt="%.4g")
        ui.PopItemWidth()
        ui.TextUnformatted("Elevation (degrees)")
        changed_e, elevation = ui.SliderFloat("##section_elevation", self.elevation, -90, 90, width=190)
        ui.SameLine()
        ui.PushItemWidth(80)
        typed_e, elevation = ui.InputDouble("##section_elevation_value", elevation, fmt="%.4g")
        ui.PopItemWidth()
        if changed_a or changed_e or typed_a or typed_e:
            azimuth = (azimuth+180) % 360-180
            elevation = max(-90, min(90, elevation))
            a, e = math.radians(azimuth), math.radians(elevation)
            self._orient_section((math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e)))
            self.azimuth, self.elevation, self.axis = azimuth, elevation, "Custom"
        ui.Separator()
        lo, hi = self._limits()
        center = (lo+hi)*.5
        position = self.offset-center
        ui.TextUnformatted("Position (model units; center = 0)")
        if hi > lo:
            changed, position = ui.SliderFloat("##section_position", position,
                                               min(lo-center, position), max(hi-center, position), width=190)
            if changed:
                self.offset = position+center
                self._update_plane()
            ui.SameLine()
        ui.PushItemWidth(80)
        changed, position = ui.InputDouble("##section_position_value", position, fmt="%.6g")
        ui.PopItemWidth()
        if changed:
            self.offset = position+center
            self._update_plane()
        changed, self.reverse = ui.Checkbox("Keep opposite side", self.reverse)
        if changed:
            self._update_plane()
        _, self.viewer.section_fill = ui.Checkbox("Fill cut faces (closed solids)", self.viewer.section_fill)

    def draw(self, ui, selected, selected_plane):
        if not self.open:
            return
        width, height = self.viewer.window.get_size()
        ui.SetNextWindowPos((max(12, width-338), 45), 4)
        ui.SetNextWindowSize((326, min(530, height-65)), 4)
        expanded, self.open = ui.Begin("Inspect (I)", self.open)
        if expanded and ui.BeginTabBar("inspection"):
            if ui.BeginTabItem("Measure"):
                self._draw_measurements(ui, selected)
                ui.EndTabItem()
            if ui.BeginTabItem("Cross section"):
                self._draw_section(ui, selected_plane)
                ui.EndTabItem()
            ui.EndTabBar()
        ui.End()

    def _draw_measurements(self, ui, selected):
        ui.TextUnformatted("Ctrl-click two points, edges or faces")
        if ui.Button("Measure minimum distance", enabled=len(selected) == 2):
            try:
                self.measure_selected(selected)
                self.error = ""
            except ValueError as error:
                self.error = str(error)
        if self.error:
            ui.TextWrapped(self.error)
        if self.measurements:
            measurement = self.measurements[-1]
            ui.TextUnformatted(f"Distance: {measurement.length:.8g} model units")
            delta = tuple(b-a for a, b in zip(measurement.start, measurement.end))
            ui.TextUnformatted("ΔX / ΔY / ΔZ: " + " / ".join(f"{d:.6g}" for d in delta))
            if ui.Button("Clear measurements"):
                self.measurements.clear()
        ui.TextUnformatted("Curved geometry uses the model tolerance.")


def _draw_annotations(viewer):
    """Shared live/capture overlay; coordinates use logical window pixels."""
    inspection = getattr(viewer, "inspection", None)
    if inspection is None:
        return
    cam, g = viewer.camera, viewer._pyglet
    width, height = viewer.window.width, viewer.window.height
    vx, vy, vw, vh = getattr(viewer._ui, "viewport_rect", (0, 0, width, height))
    look, up, right = cam.frame()
    np = viewer._np
    look, up, right = map(np.asarray, (look, up, right))
    eye = np.asarray(cam.eye)
    shapes = []
    color = (35, 60, 95, 255)
    occupied = [(0, height-42, width, 42)]
    scale_bar = None
    if cam.ortho:
        length = _scale_bar(2 * cam.zoom * cam.aspect)
        pixels = vw * length / (2 * cam.zoom * cam.aspect)
        x = max(vx + 4, vx + vw - 24 - max(pixels, 140))
        y = height-vy-vh+28
        scale_bar = (length, pixels, x, y)
        occupied.append((x-8, y-8, max(pixels, 140)+16, 45))

    def project(point):
        delta = np.asarray(point) - eye
        depth = float(delta @ look)
        if depth <= 0:
            return None
        scale = cam.zoom if cam.ortho else depth * math.tan(cam.fov / 2)
        return (vx + vw * (.5 + float(delta @ right) / (2 * scale * cam.aspect)),
                height-vy-vh + vh * (.5 + float(delta @ up) / (2 * scale)))

    def line(a, b):
        shapes.append(g.shapes.Line(*a, *b, thickness=3.5, color=(255, 255, 255, 230)))
        shapes.append(g.shapes.Line(*a, *b, thickness=1.5, color=color))

    def label(text, x, y, anchor=None):
        caption = g.text.Label(text, x=x, y=y, font_size=11, color=color,
                               anchor_y="center")
        w, h = caption.content_width+6, caption.content_height+4
        bx, by = x-3, y-3
        if anchor is not None:
            bx, by = _place_label((bx, by), (w, h), occupied, (width, height))
            if abs(bx-(x-3)) + abs(by-(y-3)) > 1:
                line(anchor, (max(bx, min(anchor[0], bx+w)),
                              max(by, min(anchor[1], by+h))))
            caption.x, caption.y = bx+3, by+h/2
        else:
            caption.y = by+h/2
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
    if scale_bar is not None:
        length, pixels, x, y = scale_bar
        line((x, y), (x+pixels, y))
        line((x, y-4), (x, y+4))
        line((x+pixels, y-4), (x+pixels, y+4))
        label(f"{length:.6g} model units", x, y+12)
    try:
        for drawable in shapes:
            drawable.draw()
    finally:
        for drawable in shapes:
            drawable.delete()
