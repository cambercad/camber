"""Small shared primitives for architectural component generators."""
from dataclasses import dataclass
import math
from numbers import Integral, Real

from camber import Assembly, Frame, Part
from examples.architecture._joinery import color, positive


def nonnegative(name, value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and nonnegative")


def integer(name, value, minimum=1):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(f"{name} must be an integer of at least {minimum}")


def vector(value):
    result = tuple(value)
    if len(result) != 3:
        raise ValueError("points must have three finite coordinates")
    for coordinate in result:
        if isinstance(coordinate, bool) or not isinstance(coordinate, Real) or not math.isfinite(coordinate):
            raise ValueError("points must have three finite coordinates")
    return result


def polygon(part, points, frame, name):
    sketch = part.sketch(frame=frame, name=name + "_profile")
    points = list(points)
    for a, b in zip(points, points[1:] + points[:1]):
        if a != b:
            sketch.add_line(a, b)
    return sketch


def rounded_rectangle(part, frame, width, height, radius, name):
    sketch = part.sketch(frame=frame, name=name+'_profile')
    r, w, h, q = radius, width, height, math.sqrt(.5)
    corners = [((r, 0), (w-r, 0), (w-r+r*q, r-r*q), (w, r)),
               ((w, r), (w, h-r), (w-r+r*q, h-r+r*q), (w-r, h)),
               ((w-r, h), (r, h), (r-r*q, h-r+r*q), (0, h-r)),
               ((0, h-r), (0, r), (r-r*q, r-r*q), (r, 0))]
    for start, end, mid, finish in corners:
        if start != end:
            sketch.add_line(start, end)
        sketch.add_arc(end, mid, finish)
    return sketch


@dataclass(frozen=True)
class BuildingComponent:
    part: Part
    assembly: Assembly
    solids: tuple
    colors: dict


class Builder:
    def __init__(self, part, name, low, high):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("name must be a nonempty string")
        self.part = part if part is not None else Part(low, high, tolerance=.025)
        self.name = name
        self.assembly = self.part.assembly(name=name)
        self.assembly.solve_after_every_constraint = False
        self.solids, self.colors = {}, {}

    def add(self, solid, rgb, *, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        self.assembly.fix(self.assembly.add_part(solid, position=position, orientation=orientation))
        self.solids[solid.name] = solid
        self.colors[f"*{solid.name}*"] = rgb
        return solid

    def box(self, name, low, high, rgb):
        return self.add(self.part.cuboid(low, high, name=name), rgb)

    def prism(self, name, points, frame, thickness, rgb):
        return self.add(polygon(self.part, points, frame, name).extrude(thickness, name=name), rgb)

    def finish(self):
        result = self.assembly.solve()
        if not result.converged:
            raise RuntimeError(f"{self.name}: assembly placement failed: {result}")
        return BuildingComponent(self.part, self.assembly, tuple(self.solids.values()), self.colors)


def show_component(build, parameters):
    """Common console entry point: inspect, render or check a default component."""
    import argparse
    parser = argparse.ArgumentParser(description=build.__doc__)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--render')
    args = parser.parse_args()
    model = build(parameters)
    print(f"{model.assembly.name}: {len(model.solids)} definitions, {len(model.assembly.leaves())} components")
    if args.render:
        from camber import render_views
        render_views(model.assembly, args.render, views=['iso'], colors=model.colors, checker=False)
    elif not args.no_show:
        model.assembly.show(colors=model.colors, checker=False)
