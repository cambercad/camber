"""Physical clay tiles or folded standing-seam sheets fitted to roof planes."""
from dataclasses import dataclass
from itertools import combinations
import math
from pathlib import Path
import sys
from typing import Literal

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Frame
from camber.modeling_parameters import register_generator
from examples.architecture._building import Builder, color, nonnegative, polygon, positive, show_component
from examples.architecture.roofs import _RoofLayout, roof_bounds, roof_faces
from examples.architecture._roof_details import clay_tile, ridge_tile


@dataclass(frozen=True, kw_only=True)
class RoofCoveringParameters(_RoofLayout):
    material: Literal['tiles', 'metal'] = 'tiles'
    detail: Literal['individual', 'surface'] = 'individual'
    tile_profile: Literal['pantile', 'plain'] = 'pantile'
    tile_width: float | None = None
    tile_length: float | None = None
    tile_gap: float = 2
    headlap: float | None = None
    side_lap: float | None = None
    pantile_rise: float = 35
    tile_thickness: float = 10
    sheet_thickness: float = .7
    tile_camber: float = 4
    cross_camber: float = 2
    sheet_width: float = 500
    seam_height: float = 25
    clearance: float = 15
    ridge_caps: bool = True
    covering_color: tuple = (.55, .21, .12)

    def __post_init__(self):
        super().__post_init__()
        if self.tile_profile not in ('pantile', 'plain'):
            raise ValueError("tile_profile must be 'pantile' or 'plain'")
        defaults = (250, 375, 75, 26) if self.tile_profile == 'pantile' else (165, 265, 65, 0)
        for name, default in zip(('tile_width', 'tile_length', 'headlap', 'side_lap'), defaults):
            if getattr(self, name) is None:
                object.__setattr__(self, name, default)
        for name in ('tile_width', 'tile_length', 'tile_thickness', 'sheet_thickness', 'sheet_width', 'seam_height', 'tile_camber', 'cross_camber'):
            positive(name, getattr(self, name))
        for name in ('clearance', 'headlap', 'tile_gap', 'side_lap'):
            nonnegative(name, getattr(self, name))
        positive('pantile_rise', self.pantile_rise)
        if self.tile_profile == 'pantile' and not 8 < self.side_lap < self.tile_width/3:
            raise ValueError('pantile side_lap must exceed 8 mm and be below one third of tile_width')
        if self.headlap >= self.tile_length/2:
            raise ValueError('headlap must be less than half the tile length')
        if self.tile_gap >= self.tile_width/4:
            raise ValueError('tile_gap must be less than a quarter of tile_width')
        if self.material not in ('tiles', 'metal') or self.detail not in ('individual', 'surface'):
            raise ValueError("use material='tiles'/'metal' and detail='individual'/'surface'")
        minimum_pitch = 35 if self.tile_profile == 'plain' else 22.5
        if self.material == 'tiles' and (self.shape == 'flat' or self.pitch < minimum_pitch):
            raise ValueError(f'{self.tile_profile} tiles require a sloping roof of at least {minimum_pitch} degrees')
        if self.material == 'tiles' and (self.tile_width < 90 or self.tile_length < 100):
            raise ValueError('tile_width must be at least 90 mm and tile_length at least 100 mm for the nibs and fixings')
        if self.tile_camber >= self.tile_length/10 or self.cross_camber >= self.tile_width/10:
            raise ValueError('tile cambers must be less than one tenth of the corresponding dimension')
        if self.material == 'tiles' and self.course_lift >= self.gauge:
            raise ValueError('tile thickness and cambers must fit within the tile course spacing')
        if self.sheet_thickness >= 2 or self.seam_height <= 10:
            raise ValueError('sheet_thickness must be below 2 mm and seam_height must exceed 10 mm for the folded lock')
        if self.material == 'metal' and self.sheet_width <= 16:
            raise ValueError('sheet_width must exceed the 8 mm standing seam')
        if not isinstance(self.ridge_caps, bool):
            raise ValueError('ridge_caps must be True or False')
        object.__setattr__(self, 'covering_color', tuple(self.covering_color))
        color(self.covering_color)

    @property
    def thickness(self):
        return self.tile_thickness if self.material == 'tiles' else self.sheet_thickness

    @property
    def course_lift(self):
        # Allow the upper course to clear both cambers of the staggered tile
        # below, rather than interpenetrating the neighbouring curved body.
        return self.tile_thickness+self.tile_camber+(self.cross_camber if self.tile_profile == 'plain' else 0)

    @property
    def gauge(self):
        return (self.tile_length-self.headlap)/(2 if self.tile_profile == 'plain' else 1)


def _clip(subject, boundary):
    """Convex polygon clipping in the roof plane; no solid Boolean per tile."""
    result = list(subject)
    for a, b in zip(boundary, boundary[1:]+boundary[:1]):
        def side(p):
            return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
        source, result = result, []
        if not source:
            break
        for start, end in zip(source, source[1:]+source[:1]):
            ds, de = side(start), side(end)
            if (ds >= 0) != (de >= 0):
                t = ds/(ds-de)
                result.append((start[0]+t*(end[0]-start[0]), start[1]+t*(end[1]-start[1])))
            if de >= 0:
                result.append(end)
    clean = []
    for point in result:
        if not clean or math.dist(point, clean[-1]) > 1e-8:
            clean.append(point)
    if len(clean) > 1 and math.dist(clean[0], clean[-1]) <= 1e-8:
        clean.pop()
    area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(clean, clean[1:]+clean[:1]))
    return clean if len(clean) >= 3 and area > 1e-8 else []


def _caps(b, faces, p):
    junctions = []
    for first, second in combinations(faces, 2):
        def vertices(face):
            f = face.frame
            return [f.origin+f.x*x+f.y*y for x, y in face.outline]
        shared = [a for a in vertices(first) if any(math.dist(a, c) < 1e-6 for c in vertices(second))]
        if len(shared) != 2:
            continue
        a, end = shared
        direction = (end-a).normalized()
        normal = (first.frame.z+second.frame.z).normalized()
        across = normal.cross(direction).normalized()
        half = min(100, p.width/8, p.length/8)
        def height(x):
            return min(-x*f.frame.z.dot(across)/f.frame.z.dot(normal) for f in (first, second))
        t = p.thickness
        points = [(-half, height(-half)), (0, 0), (half, height(half)),
                  (half, height(half)+t), (0, t), (-half, height(-half)+t)]
        gauge = p.gauge
        lift = p.clearance+p.tile_length*t/math.sqrt(gauge*gauge-t*t)+t if p.material == 'tiles' else p.clearance+p.seam_height+t
        frame = Frame(a+normal*lift, x=across, y=normal, z=direction)
        label = b.name+'_cap_'+first.name+'_'+second.name
        length = (end-a).norm()
        if p.material == 'tiles' and p.detail == 'individual':
            # Individual half-round ridge/hip tiles, with visible mortar joints.
            radius = min(130, p.width/6, p.length/6)
            cap_frame = frame.offset(-normal*(radius*.65))
            for point in (a, end):
                entry = next((item for item in junctions if math.dist(item[0], point) < 1e-6), None)
                if entry is None:
                    entry = (point, [])
                    junctions.append(entry)
                entry[1].append(point+(cap_frame.origin-a)+normal*radius)
            count = math.ceil(length/305)
            module = length/count
            gap = min(3, module/10)
            tile = ridge_tile(b.part, cap_frame, radius, min(t, radius/4), module-gap, label)
            joint = ridge_tile(b.part, cap_frame.offset(direction*(module-gap)), radius*.99,
                               min(t, radius/4)*.8, gap, label+'_mortar') if count > 1 else None
            for index in range(count):
                b.add(tile, p.covering_color, position=tuple(direction*(index*module)))
                if joint is not None and index < count-1:
                    b.add(joint, (.58, .57, .53), position=tuple(direction*(index*module)))
        else:
            b.prism(label, points, frame, length, p.covering_color)
    for index, (_, crowns) in enumerate(junctions):
        if len(crowns) < 3:
            continue
        # A separate hollow crown covers the meeting of three/four ridge
        # branches. Ordinary straight ridge pieces leave an open apex here.
        t = p.tile_thickness
        x = sum(point.x for point in crowns)/len(crowns)
        y = sum(point.y for point in crowns)/len(crowns)
        z = min(point.z for point in crowns)-2*t
        radius = max(2*t, max(math.hypot(point.x-x, point.y-y) for point in crowns)+2*t)
        label = f'{b.name}_ridge_junction{index+1}'
        outer = b.part.sphere((x, y, z), radius, name=label+'_outer', max_deviation=.15)
        inner = b.part.sphere((x, y, z), radius-t, name=label+'_inner', max_deviation=.15)
        hood = b.part.subtract(outer, inner, name=label+'_shell')
        top = b.part.cuboid((x-radius-1, y-radius-1, z), (x+radius+1, y+radius+1, z+radius+1), name=label+'_limit')
        b.add(b.part.intersect(hood, top, name=label), p.covering_color)


def build_roof_covering(parameters=RoofCoveringParameters(), *, part=None, name='roof_covering'):
    """Fit interlocking pantiles, plain tiles or standing-seam sheets to a roof.

    Match the roof's width, length, shape, pitch, overhang and eave_height.
    Individual tiles have curved bodies, underside nibs and through fixings;
    plain tiles use staggered double courses, pantiles use aligned side laps.
    Hip/verge pieces are clipped to the roof. Surface detail uses one skin per
    face for whole-house work. Repeated full tiles share solid definitions.
    """
    p = parameters
    if not isinstance(p, RoofCoveringParameters):
        raise TypeError('parameters must be RoofCoveringParameters')
    b = Builder(part, name, *roof_bounds(p, margin=p.clearance+p.thickness+p.seam_height+500))
    faces = roof_faces(p)
    for face in faces:
        f, boundary = face.frame, list(face.outline)
        prefix = name+'_'+face.name
        if p.detail == 'surface':
            b.prism(prefix+'_skin', boundary, f.offset(f.z*p.clearance), p.thickness, p.covering_color)
            continue
        width, height = max(x for x, _ in boundary), max(y for _, y in boundary)
        if p.material == 'metal':
            # A female folded lock overlaps the next panel's male upstand.
            # These are thin sheet sections with air inside the seam.
            panel_frame = Frame(f.origin+f.y*height+f.z*p.clearance, x=f.x, y=f.z, z=-f.y)
            for column in range(math.ceil(width/p.sheet_width)):
                x = column*p.sheet_width
                w, h, t = p.sheet_width+3, p.seam_height, p.sheet_thickness
                points = [(0, 0), (w-6, 0), (w-6, h-t), (w-t, h-t), (w-t, h-8),
                          (w, h-8), (w, h), (w-6-t, h), (w-6-t, t), (t, t), (t, h-5), (0, h-5)]
                label = prefix+f'_sheet{column+1}'
                sheet = polygon(b.part, points, panel_frame, label).extrude(height, name=label+'_folded')
                outline = [(px-x, py) for px, py in boundary]
                limit = polygon(b.part, outline, f.offset(f.z*(p.clearance-1)), label+'_limit').extrude(h+2, name=label+'_limit')
                b.add(b.part.intersect(sheet, limit, name=label), p.covering_color, position=tuple(f.x*x))
        else:
            gauge = p.gauge
            tilt = math.asin(p.course_lift/gauge)
            c, s = math.cos(tilt), math.sin(tilt)
            tile_frame = Frame(f.origin+f.z*(p.clearance+p.tile_length*s/c),
                               x=f.x, y=f.y*c-f.z*s, z=f.y*s+f.z*c)
            full = clay_tile(b.part, tile_frame, p, prefix+'_tile')
            trimmed = {}
            module_width = p.tile_width-p.side_lap if p.tile_profile == 'pantile' else p.tile_width
            for row in range(math.ceil(height/gauge)):
                y = row*gauge
                offset = -p.tile_width/2 if row % 2 and p.tile_profile == 'plain' else 0
                for column in range(math.ceil((width-offset)/module_width)):
                    x = offset+column*module_width
                    gap = p.tile_gap/2 if p.tile_profile == 'plain' else 0
                    left, right = x+gap, x+p.tile_width-gap
                    rectangle = [(left, y), (right, y), (right, y+p.tile_length), (left, y+p.tile_length)]
                    outline = _clip(rectangle, boundary)
                    if not outline:
                        continue
                    complete = len(outline) == 4 and all(point in outline for point in rectangle)
                    solid = full
                    if not complete:
                        # Cut the curved body with a prism normal to the roof,
                        # keeping analytic camber and any surviving nibs/holes.
                        local = [(px-x, py-y) for px, py in outline]
                        key = tuple(sorted(local))
                        solid = trimmed.get(key)
                        if solid is None:
                            label = prefix+f'_trimmed_r{row+1}_c{column+1}'
                            depth = p.clearance+p.tile_length*s/c+p.tile_camber+p.cross_camber+p.tile_thickness+100
                            cutter = polygon(b.part, local, f.offset(f.z*-50), label+'_limit').extrude(depth, name=label+'_limit')
                            solid = b.part.intersect(full, cutter, name=label)
                            trimmed[key] = solid
                    b.add(solid, p.covering_color, position=tuple(f.x*x+f.y*y))
    if p.ridge_caps:
        _caps(b, faces, p)
    return b.finish()


roof_covering_component = register_generator(build_roof_covering, RoofCoveringParameters, name='roof_covering_component')

if __name__ == '__main__':
    show_component(build_roof_covering, RoofCoveringParameters())
