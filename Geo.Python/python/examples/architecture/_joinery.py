"""Shared geometry for window and door construction; no extra dependencies."""
from dataclasses import dataclass
import math
from numbers import Integral, Real

from camber import Assembly, Frame, Part

SEAL = (.12, .13, .14)
METAL = (.48, .51, .54)
GLASS = (.64, .79, .83)


def positive(name, value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number")


def layout(count, weights, angles, sides):
    if isinstance(count, bool) or not isinstance(count, Integral) or count < 1:
        raise ValueError("leaves must be a positive integer")
    weights = tuple(weights) if weights is not None else (1.0,) * count
    angles = tuple(angles) if angles is not None else (0.0,) * count
    sides = tuple(sides) if sides is not None else tuple("left" if i % 2 == 0 else "right" for i in range(count))
    if any(len(values) != count for values in (weights, angles, sides)):
        raise ValueError("leaf_weights, opening_angles and hinge_sides must have one entry per leaf")
    for weight in weights:
        positive("leaf weight", weight)
    for angle, side in zip(angles, sides):
        if isinstance(angle, bool) or not isinstance(angle, Real) or not math.isfinite(angle) or not 0 <= angle <= 120:
            raise ValueError("opening_angles must be finite degrees between 0 and 120")
        if side not in ("left", "right", "fixed"):
            raise ValueError("hinge_sides must contain 'left', 'right' or 'fixed'")
        if side == "fixed" and angle != 0:
            raise ValueError("a fixed leaf cannot have an opening angle")
    return weights, angles, sides


def color(value):
    if len(value) != 3 or any(not isinstance(c, Real) or not math.isfinite(c) or not 0 <= c <= 1 for c in value):
        raise ValueError("colors must be three finite RGB values between 0 and 1")


@dataclass(frozen=True)
class JoineryModel:
    assembly: Assembly
    colors: dict
    leaves: tuple
    solids: tuple
    glass: tuple


class Builder:
    def __init__(self, part, name, width, height, depth, *, margin=250):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("name must be a nonempty string")
        self.part = part if part is not None else Part(
            (-width-margin, -width, -margin), (2 * width+margin, 2 * width + depth, height + margin), tolerance=.025)
        self.name = name
        self.colors, self.solids, self.glass = {}, [], []
        self.assembly = self.group(name)

    def group(self, name):
        assembly = self.part.assembly(name=name)
        assembly.solve_after_every_constraint = False
        return assembly

    def add(self, assembly, solid, rgb):
        occurrence = assembly.add_part(solid)
        assembly.fix(occurrence)
        self.solids.append(solid)
        self.colors[f"*{solid.name}*"] = rgb
        return solid

    def box(self, assembly, name, low, high, rgb):
        return self.add(assembly, self.part.cuboid(low, high, name=name), rgb)

    def cylinder(self, assembly, name, origin, radius, length, *, axis="z", rgb=METAL):
        return self.add(assembly, self.part.cylinder(origin, radius, length,
                        axis=axis, name=name, max_deviation=.025), rgb)

    def polygon(self, points, front, back, name):
        sketch = self.part.sketch(frame=Frame((0, back, 0), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), name=name + "_section")
        for a, b in zip(points, points[1:] + points[:1]):
            sketch.add_line(a, b)
        return sketch.extrude(back - front, name=name)

    def rounded_box(self, assembly, name, x, z, width, height, front, back, radius, rgb):
        return self.add(assembly, self.rounded_solid(name, x, z, width, height, front, back, radius), rgb)

    def rounded_solid(self, name, x, z, width, height, front, back, radius):
        sketch = self.part.sketch(frame=Frame((x, back, z), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), name=name + "_section")
        r, w, h, q = radius, width, height, math.sqrt(.5)
        corners = [((r, 0), (w-r, 0), (w-r+r*q, r-r*q), (w, r)),
                   ((w, r), (w, h-r), (w-r+r*q, h-r+r*q), (w-r, h)),
                   ((w-r, h), (r, h), (r-r*q, h-r+r*q), (0, h-r)),
                   ((0, h-r), (0, r), (r-r*q, r-r*q), (r, 0))]
        for start, end, mid, finish in corners:
            if start != end:
                sketch.add_line(start, end)
            sketch.add_arc(end, mid, finish)
        return sketch.extrude(back-front, name=name, max_deviation=.025)

    def ring(self, assembly, name, x, z, width, height, border, front, back, rgb):
        sketch = self.part.sketch(frame=Frame((x, back, z), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), name=name + "_section")
        sketch.add_rectangle((0, 0), (width, height))
        sketch.add_rectangle((border, border), (width-border, height-border))
        return self.add(assembly, sketch.extrude(back-front, name=name), rgb)

    def frame(self, assembly, name, x, z, width, height, sections, rgb, *, closed=True, cutters=None):
        """Four mitred members, or three jamb/head members with square feet."""
        result = {}
        for side in ("bottom", "right", "top", "left") if closed else ("right", "top", "left"):
            pieces = []
            for i, (border, front, back) in enumerate(sections):
                w, h, b = width, height, border
                outlines = {
                    "bottom": [(0, 0), (w, 0), (w-b, b), (b, b)],
                    "right": [(w, 0), (w, h), (w-b, h-b), (w-b, b if closed else 0)],
                    "top": [(w, h), (0, h), (b, h-b), (w-b, h-b)],
                    "left": [(0, h), (0, 0), (b, b if closed else 0), (b, h-b)],
                }
                points = [(px+x, pz+z) for px, pz in outlines[side]]
                pieces.append(self.polygon(points, front, back, f"{name}_{side}_layer{i}"))
            solid = self.part.batch_union(pieces, name=f"{name}_{side}")
            for cutter in (cutters or {}).get(side, ()):
                solid = self.part.subtract(solid, cutter, name=f"{name}_{side}")
            result[side] = self.add(assembly, solid, rgb)
        return result

    def glazing(self, assembly, name, x, z, width, height, front, layers):
        y = front
        for i, thickness in enumerate(layers):
            if i % 2:
                self.ring(assembly, f"{name}_spacer{i//2+1}", x, z, width, height, 7, y, y+thickness, SEAL)
            else:
                pane = self.box(assembly, f"{name}_pane{i//2+1}",
                                (x, y, z), (x+width, y+thickness, z+height), GLASS)
                self.glass.append(pane)
            y += thickness

    def screw(self, assembly, name, x, y, z, *, direction=1):
        axis = Frame((x, y, z), x=(1, 0, 0), y=(0, 0, -direction), z=(0, direction, 0))
        head = self.part.cylinder(axis, 2.5, 1.5, name=name + "_blank", max_deviation=.025)
        low, high = sorted((y+.9*direction, y+2*direction))
        slot = self.part.cuboid((x-1.8, low, z-.4), (x+1.8, high, z+.4), name=name + "_slot")
        self.add(assembly, self.part.subtract(head, slot, name=name), METAL)

    def hinges(self, fixed, leaf, name, x, base_z, width, height, back, side, *, large=False):
        pivot_x = 0 if side == "left" else width
        pivot_y = back + 5
        r, length = (6, 72) if large else (4.5, 44)
        count = 3 if large or height > 1400 else 2
        margin = min(180, height*.18)
        for i in range(count):
            center = margin + i * (height - 2*margin)/(count-1)
            label = f"{name}_hinge{i+1}"
            for segment in range(3):
                bottom = center-length/2 + segment*length/3 + .6
                sketch = self.part.sketch(name=f"{label}_knuckle{segment}_section")
                sketch.add_circle((pivot_x, pivot_y), r)
                sketch.add_circle((pivot_x, pivot_y), 2.7)
                tube = sketch.extrude(length/3-1.2, name=f"{label}_knuckle{segment}")
                target = leaf if segment == 1 else fixed
                position = (0, 0, bottom) if segment == 1 else (x, 0, base_z+bottom)
                occurrence = target.add_part(tube, position=position)
                target.fix(occurrence)
                self.solids.append(tube)
                self.colors[f"*{tube.name}*"] = METAL
            self.cylinder(fixed, label + "_pin", (x+pivot_x, pivot_y, base_z+center-length/2), 2.4, length)
            for top in (False, True):
                self.cylinder(fixed, label + ("_top_cap" if top else "_bottom_cap"),
                              (x+pivot_x, pivot_y, base_z+center + (length/2 if top else -length/2-1)), r, 1)
            sign = 1 if side == "left" else -1
            for target, offset, origin_z, direction in ((leaf, 0, 0, sign), (fixed, x, base_z, -sign)):
                lo, hi = sorted((pivot_x+direction*3, pivot_x+direction*19))
                plate_name = label + ("_leaf_plate" if target is leaf else "_frame_plate")
                self.box(target, plate_name, (offset+lo, back+1, origin_z+center-length/2),
                         (offset+hi, back+4, origin_z+center+length/2), METAL)
                for dz in (-length*.32, length*.32):
                    self.screw(target, plate_name + ("_upper_screw" if dz > 0 else "_lower_screw"),
                               offset+pivot_x+direction*13, back+4, origin_z+center+dz)
        return pivot_x, pivot_y

    def place_leaf(self, leaf, x, z, pivot, angle, side):
        theta = math.radians(angle if side == "left" else -angle)
        c, s = math.cos(theta), math.sin(theta)
        px, py = pivot
        occurrence = self.assembly.add_subassembly(leaf,
            position=(x+px-c*px+s*py, py-s*px-c*py, z),
            orientation=(0, 0, math.sin(theta/2), math.cos(theta/2)))
        self.assembly.fix(occurrence)
        return occurrence

    def finish(self, leaves):
        result = self.assembly.solve()
        if not result.converged:
            raise RuntimeError(f"{self.name}: assembly placement failed: {result}")
        return JoineryModel(self.assembly, self.colors, tuple(leaves), tuple(self.solids), tuple(self.glass))
