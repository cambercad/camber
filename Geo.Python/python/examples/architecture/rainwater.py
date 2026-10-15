"""Half-round gutters and hollow downpipes with rounded, three-dimensional routes."""
from dataclasses import dataclass
import math
from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Curve3D, Frame, vec3
from camber.modeling_parameters import register_generator
from examples.architecture._building import BuildingComponent, Builder, nonnegative, positive, show_component, vector

METAL = (.47, .50, .53)


@dataclass(frozen=True, kw_only=True)
class GutterParameters:
    length: float = 10000
    radius: float = 75
    wall_thickness: float = 1
    elevation: float = 3000
    fall: float = 30
    end_caps: bool = True
    outlet_offset: float = 100
    outlet_diameter: float = 80
    outlet_length: float = 60
    bead_radius: float = 8
    hangers: bool = True
    hanger_spacing: float = 600

    def __post_init__(self):
        for name in ('length', 'radius', 'wall_thickness', 'outlet_diameter', 'outlet_length', 'bead_radius', 'hanger_spacing'):
            positive(name, getattr(self, name))
        for name in ('elevation', 'fall', 'outlet_offset'):
            nonnegative(name, getattr(self, name))
        if self.wall_thickness >= self.radius or self.length <= 2*self.wall_thickness:
            raise ValueError('wall_thickness must fit inside the radius and length')
        if self.outlet_diameter >= 2*(self.radius-self.wall_thickness):
            raise ValueError('outlet_diameter must fit inside the gutter')
        if not self.outlet_diameter/2+self.wall_thickness < self.outlet_offset < self.length-self.outlet_diameter/2-self.wall_thickness:
            raise ValueError('outlet_offset must leave clearance from both end caps')
        if self.fall >= self.length:
            raise ValueError('fall must be less than the gutter length')
        if not isinstance(self.end_caps, bool):
            raise ValueError('end_caps must be True or False')
        if not isinstance(self.hangers, bool) or self.bead_radius <= self.wall_thickness or self.bead_radius >= self.radius/3:
            raise ValueError('hangers must be True/False; bead_radius must exceed wall_thickness and be below radius/3')


@dataclass(frozen=True)
class GutterModel(BuildingComponent):
    outlet: tuple


def _half_round(part, frame, radius, thickness, name, *, bead=0):
    sketch = part.sketch(frame=frame, name=name+'_profile')
    sketch.add_arc((-radius, 0), (0, -radius), (radius, 0))
    if thickness:
        inside = radius-thickness
        if bead:
            # Continuous sheet section: the front edge curls almost a full
            # turn, with an open underside, and the rear edge has a folded hem.
            c, s = math.cos(math.pi/3), math.sin(math.pi/3)
            center = radius+bead
            end = (center-bead*c, -bead*s)
            outside = bead+thickness
            outer_end = (center-outside*c, -outside*s)
            sketch.add_arc((radius, 0), (center+bead, 0), end)
            sketch.add_line(end, outer_end)
            sketch.add_arc(outer_end, (center+outside, 0), (inside, 0))
        else:
            sketch.add_line((radius, 0), (inside, 0))
        sketch.add_arc((inside, 0), (0, -inside), (-inside, 0))
        tail = [(-inside, 0)]
        if bead:
            tail += [(-inside, 12-thickness), (-radius+6, 12-thickness), (-radius+6, 8),
                     (-radius+6+thickness, 8), (-radius+6+thickness, 12), (-radius, 12)]
        tail += [(-radius, 0)]
        for start, end in zip(tail, tail[1:]):
            sketch.add_line(start, end)
    else:
        sketch.add_line((radius, 0), (-radius, 0))
    return sketch


def build_gutter(parameters=GutterParameters(), *, part=None, name='gutter'):
    """Build an open half-round gutter along +Y, falling towards its far end.

    The outlet_offset is measured back from the low end (Y=length). The result's outlet is the bottom
    centre of the vertical outlet neck, ready to start a downpipe route.
    """
    p = parameters
    if not isinstance(p, GutterParameters):
        raise TypeError('parameters must be GutterParameters')
    margin = p.radius+500
    b = Builder(part, name, (-margin, -margin, min(0, p.elevation-p.fall-p.radius-p.outlet_length)-500),
                (margin, p.length+margin, p.elevation+margin))
    length = math.hypot(p.length, p.fall)
    frame = Frame((0, p.length, p.elevation-p.fall), x=(1, 0, 0),
                  y=(0, p.fall/length, p.length/length), z=(0, -p.length/length, p.fall/length))
    body = _half_round(b.part, frame, p.radius, p.wall_thickness, name, bead=p.bead_radius).extrude(length, name=name+'_channel', max_deviation=.1)
    outlet_y = p.length-p.outlet_offset
    z = p.elevation-p.fall*outlet_y/p.length-p.radius*length/p.length
    outlet = (0, outlet_y, z-p.outlet_length)
    cutter = b.part.cylinder(outlet, p.outlet_diameter/2-p.wall_thickness,
                             p.outlet_length+2*p.radius, name=name+'_outlet_tool', max_deviation=.1)
    b.add(b.part.subtract(body, cutter, name=name+'_channel'), METAL)
    if p.end_caps:
        for label, offset in (('far', 0), ('near', length-p.wall_thickness)):
            section = _half_round(b.part, frame.offset(frame.z*offset), p.radius, 0, name+'_'+label)
            b.add(section.extrude(p.wall_thickness, name=name+'_'+label+'_cap'), METAL)
    neck = b.part.sketch(frame=Frame(outlet), name=name+'_outlet_profile')
    neck.add_circle((0, 0), p.outlet_diameter/2)
    neck.add_circle((0, 0), p.outlet_diameter/2-p.wall_thickness)
    # Trim the neck to the curved inside of the channel, rather than leaving
    # a flat pipe end hanging below its sides or projecting into the waterway.
    neck_solid = neck.extrude(p.outlet_length+p.radius, name=name+'_neck_blank')
    waterway = b.part.sketch(frame=frame.offset(frame.z*-1), name=name+'_waterway_profile')
    waterway.add_circle((0, 0), p.radius-p.wall_thickness)
    void = waterway.extrude(length+2, name=name+'_waterway_tool')
    b.add(b.part.subtract(neck_solid, void, name=name+'_outlet'), METAL)
    if p.hangers:
        strap = _half_round(b.part, frame, p.radius+5, 4, name+'_hanger').extrude(30, name=name+'_hanger_strap', max_deviation=.1)
        shank_frame = Frame(frame.origin-frame.x*(p.radius+5)-frame.y*10, x=frame.x, y=frame.y, z=frame.z)
        shank = b.part.cuboid(shank_frame, (5, 120, 30), name=name+'_hanger_shank')
        bracket = b.part.union(strap, shank, name=name+'_hanger_blank')
        holes = []
        for i, height in enumerate((55, 90)):
            pose = Frame(frame.origin-frame.x*(p.radius+6)+frame.y*height+frame.z*15,
                         x=-frame.z, y=frame.y, z=frame.x)
            profile = b.part.sketch(frame=pose, name=f'{name}_hanger_fixing{i}')
            profile.add_circle((0, 0), 3)
            holes.append(profile.extrude(8, name=f'{name}_hanger_hole{i}', max_deviation=.1))
        bracket = b.part.batch_subtract(bracket, holes, name=name+'_hanger')
        count = max(2, math.ceil(length/p.hanger_spacing)+1)
        margin = min(100, length/4)
        for i in range(count):
            distance = margin+(length-2*margin)*i/(count-1)-15
            b.add(bracket, METAL, position=tuple(frame.z*distance))
    model = b.finish()
    return GutterModel(model.part, model.assembly, model.solids, model.colors, outlet)


@dataclass(frozen=True, kw_only=True)
class DownpipeParameters:
    points: tuple = ((0, 0, 3000), (0, 0, 2600), (0, 300, 2300), (0, 300, 200), (0, 450, 50))
    diameter: float = 80
    wall_thickness: float = 1.2
    bend_radius: float = 100
    collars: bool = True
    collar_spacing: float = 1500

    def __post_init__(self):
        for name in ('diameter', 'wall_thickness', 'bend_radius', 'collar_spacing'):
            positive(name, getattr(self, name))
        object.__setattr__(self, 'points', tuple(vector(p) for p in self.points))
        if len(self.points) < 2:
            raise ValueError('points must contain at least two route points')
        if self.wall_thickness >= self.diameter/2 or self.bend_radius <= self.diameter/2:
            raise ValueError('wall_thickness must fit inside the pipe; bend_radius must exceed its outer radius')
        if not isinstance(self.collars, bool):
            raise ValueError('collars must be True or False')
        _route(self)


def _route(p):
    points = [vec3(point) for point in p.points]
    lengths = [(b-a).norm() for a, b in zip(points, points[1:])]
    if any(length <= 1e-6 for length in lengths):
        raise ValueError('adjacent route points must be distinct')
    directions = [(b-a)/length for a, b, length in zip(points, points[1:], lengths)]
    corners = [None]*len(points)
    offsets = [0.0]*len(points)
    for i in range(1, len(points)-1):
        before, after = directions[i-1], directions[i]
        cosine = max(-1, min(1, before.dot(after)))
        if cosine < -1+1e-10:
            raise ValueError('a downpipe route cannot reverse direction at a point')
        if cosine > 1-1e-10:
            continue
        angle = math.acos(cosine)
        offsets[i] = p.bend_radius*math.tan(angle/2)
        normal = (after-before*cosine).normalized()
        start = points[i]-before*offsets[i]
        corners[i] = (start, points[i]+after*offsets[i], start+normal*p.bend_radius, -normal, before, angle)
    if any(a+b >= length-1e-6 for a, b, length in zip(offsets, offsets[1:], lengths)):
        raise ValueError('route segments are too short for bend_radius; move the points or reduce the radius')
    return points, directions, corners


def _pipe_frame(origin, direction):
    reference = vec3(1, 0, 0) if abs(direction.x) < .9 else vec3(0, 1, 0)
    x = (reference-direction*reference.dot(direction)).normalized()
    return Frame(origin, x=x, y=direction.cross(x), z=direction)


def build_downpipe(parameters=DownpipeParameters(), *, part=None, name='downpipe'):
    """Sweep a hollow pipe through a 3D polyline with true circular elbow bends.

    Route points are centreline coordinates. Bends are tangent to neighbouring
    segments; short segments and reversals are rejected before creating geometry.
    Collars are separate rings on straight runs, not assumed wall fixings.
    """
    p = parameters
    if not isinstance(p, DownpipeParameters):
        raise TypeError('parameters must be DownpipeParameters')
    points, directions, corners = _route(p)
    margin = p.diameter+p.bend_radius
    low = tuple(min(point[i] for point in points)-margin for i in range(3))
    high = tuple(max(point[i] for point in points)+margin for i in range(3))
    b = Builder(part, name, low, high)
    curves, straight = [], []
    start = points[0]
    for i in range(1, len(points)):
        end = corners[i][0] if corners[i] else points[i]
        curves.append(Curve3D.line(start, end, name=f'{name}_run{i}'))
        straight.append((start, end, directions[i-1]))
        if corners[i]:
            _, start, center, x, y, angle = corners[i]
            curves.append(Curve3D.arc(center, x, y, p.bend_radius, 0, angle, name=f'{name}_bend{i}'))
        else:
            start = end
    profile = b.part.sketch(frame=_pipe_frame(points[0], directions[0]), name=name+'_profile')
    profile.add_circle((0, 0), p.diameter/2)
    profile.add_circle((0, 0), p.diameter/2-p.wall_thickness)
    b.add(profile.extrude_along_curve_strip(curves, name=name+'_pipe', max_deviation=.1), METAL)
    if p.collars:
        for i, (a, end, direction) in enumerate(straight):
            length = (end-a).norm()
            count = math.ceil(length/p.collar_spacing)
            if length < 50:
                continue
            for j in range(count):
                origin = a+direction*(length*(j+.5)/count-12.5)
                collar = b.part.sketch(frame=_pipe_frame(origin, direction), name=f'{name}_collar{i}_{j}_profile')
                collar.add_circle((0, 0), p.diameter/2+4)
                collar.add_circle((0, 0), p.diameter/2)
                b.add(collar.extrude(25, name=f'{name}_collar{i}_{j}'), METAL)
    return b.finish()


gutter_component = register_generator(build_gutter, GutterParameters, name='gutter_component')
downpipe_component = register_generator(build_downpipe, DownpipeParameters, name='downpipe_component')

if __name__ == '__main__':
    show_component(build_gutter, GutterParameters())
