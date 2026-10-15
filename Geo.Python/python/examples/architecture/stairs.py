"""Configurable straight stairs: steel, timber or concrete, with modern guarding."""
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Literal

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Frame
from camber.modeling_parameters import register_generator
from examples.architecture._building import Builder, integer, nonnegative, polygon, positive, rounded_rectangle, show_component

WOOD = (.65, .53, .38)
METAL = (.16, .17, .18)


@dataclass(frozen=True, kw_only=True)
class StairParameters:
    construction: Literal['steel', 'timber', 'concrete'] = 'steel'
    stringer_layout: Literal['auto', 'central', 'sides'] = 'auto'
    railing_style: Literal['metal', 'glass', 'timber'] = 'metal'
    railing_sides: Literal['both', 'left', 'right'] = 'both'
    tread_material: Literal['timber', 'steel', 'stone', 'none'] = 'timber'
    width: float = 1000
    total_rise: float = 2800
    risers: int = 16
    going: float = 270
    tread_thickness: float = 50
    nosing: float = 10
    closed_risers: bool | None = None
    riser_thickness: float = 18
    stringer_width: float | None = None
    stringer_depth: float | None = None
    steel_wall_thickness: float = 6
    waist_thickness: float = 160
    landing_length: float = 1000
    railings: bool = True
    railing_height: float = 1000
    baluster_spacing: float = 110
    nosing_radius: float = 3
    newel_width: float = 90
    handrail_width: float = 60
    handrail_height: float = 55
    baluster_diameter: float = 14
    post_spacing: float = 1000
    glass_thickness: float = 12.76

    def __post_init__(self):
        for name, allowed in (('construction', ('steel', 'timber', 'concrete')),
                              ('stringer_layout', ('auto', 'central', 'sides')),
                              ('railing_style', ('metal', 'glass', 'timber')),
                              ('railing_sides', ('both', 'left', 'right')),
                              ('tread_material', ('timber', 'steel', 'stone', 'none'))):
            if getattr(self, name) not in allowed:
                raise ValueError(f'{name} must be one of {allowed}')
        if self.stringer_layout == 'auto':
            object.__setattr__(self, 'stringer_layout', 'central' if self.construction == 'steel' else 'sides')
        if self.stringer_width is None:
            object.__setattr__(self, 'stringer_width', 180 if self.construction == 'steel' and self.stringer_layout == 'central' else 45)
        if self.stringer_depth is None:
            object.__setattr__(self, 'stringer_depth', 200 if self.construction == 'steel' else 300)
        if self.closed_risers is None:
            object.__setattr__(self, 'closed_risers', self.construction != 'steel')
        integer('risers', self.risers, 2)
        for name in ('width', 'total_rise', 'going', 'tread_thickness', 'riser_thickness',
                     'stringer_width', 'stringer_depth', 'railing_height', 'baluster_spacing',
                     'nosing_radius', 'newel_width', 'handrail_width', 'handrail_height', 'baluster_diameter',
                     'steel_wall_thickness', 'waist_thickness', 'post_spacing', 'glass_thickness'):
            positive(name, getattr(self, name))
        for name in ('nosing', 'landing_length'):
            nonnegative(name, getattr(self, name))
        rise = self.total_rise/self.risers
        if self.tread_thickness >= rise:
            raise ValueError('tread_thickness must be less than one riser height')
        if self.width <= 2*self.stringer_width+80:
            raise ValueError('width must leave space between the two stringers')
        if self.stringer_depth <= rise or self.riser_thickness >= self.going or self.nosing >= self.going:
            raise ValueError('stringer_depth must exceed a riser; riser_thickness and nosing must be less than going')
        if self.railings and (self.railing_height <= 100 or self.baluster_spacing <= 20):
            raise ValueError('railing_height must exceed 100 mm and baluster_spacing must exceed 20 mm')
        if not isinstance(self.railings, bool) or not isinstance(self.closed_risers, bool):
            raise ValueError('railings and closed_risers must be True or False')
        if self.nosing_radius >= self.tread_thickness/2 or self.nosing_radius >= self.going/2:
            raise ValueError('nosing_radius must be less than half the tread thickness and going')
        if self.newel_width <= self.handrail_width+10 or 2*self.newel_width >= self.width:
            raise ValueError('newel_width must exceed handrail_width by 10 mm and leave a clear stair width')
        if min(self.handrail_width, self.handrail_height) <= 24 or self.baluster_diameter >= self.baluster_spacing:
            raise ValueError('handrail dimensions must exceed 24 mm; baluster_diameter must be below baluster_spacing')
        if self.steel_wall_thickness*2 >= min(self.stringer_width, self.stringer_depth):
            raise ValueError('steel_wall_thickness must leave a hollow stringer section')
        if self.construction != 'concrete' and self.tread_material == 'none':
            raise ValueError('steel and timber construction require a tread material')
        if self.construction == 'concrete' and not self.closed_risers:
            raise ValueError('concrete construction requires closed_risers=True')
        if self.construction == 'timber' and self.stringer_layout != 'sides':
            raise ValueError('timber construction requires side stringers')
        if self.going < 120 or self.width < 400:
            raise ValueError('going must be at least 120 mm and width at least 400 mm')
        if self.railings and self.railing_style != 'timber' and self.landing_length and self.landing_length < 120:
            raise ValueError('landing_length must be at least 120 mm for the handrail elbow')


def build_stairs(parameters=StairParameters(), *, part=None, name='stairs'):
    """Build a straight stair along +Y, starting at floor level, with width along X.

    Risers includes the final rise onto the upper floor/landing: tread count is
    risers-1. Set landing_length=0 for an existing upper floor.
    Steel uses hollow stringers and tread saddles; concrete uses a continuous
    waist. Choose slim metal infill, clamped glass or timber guarding.
    """
    p = parameters
    if not isinstance(p, StairParameters):
        raise TypeError('parameters must be StairParameters')
    rise, run = p.total_rise/p.risers, (p.risers-1)*p.going
    b = Builder(part, name, (-500, -p.nosing-500, -500),
                (p.width+500, run+p.landing_length+500, p.total_rise+p.railing_height+500))
    finish_color = {'timber': WOOD, 'steel': METAL, 'stone': (.64, .65, .64), 'none': (.64, .65, .64)}[p.tread_material]
    for i in range(p.risers-1) if p.tread_material != 'none' else ():
        y, z = i*p.going, (i+1)*rise
        frame = Frame((0, y, z), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
        profile = b.part.sketch(frame=frame, name=f'{name}_tread{i+1}_profile')
        r, front, t, q = p.nosing_radius, -p.nosing, p.tread_thickness, math.sqrt(.5)
        points = [(front+r, 0), (p.going, 0), (p.going, -t), (front+r, -t)]
        for a, end in zip(points, points[1:]):
            profile.add_line(a, end)
        profile.add_arc((front+r, -t), (front+r-r*q, -t+r-r*q), (front, -t+r))
        profile.add_line((front, -t+r), (front, -r))
        profile.add_arc((front, -r), (front+r-r*q, -r+r*q), (front+r, 0))
        label = f'{name}_tread{i+1}'
        tread = profile.extrude(p.width, name=label+'_blank' if p.construction == 'steel' else label, max_deviation=.1)
        if p.construction == 'steel':
            from examples.architecture._stair_structure import drill_tread
            tread = drill_tread(b.part, tread, p, i, label)
        b.add(tread, finish_color)
    if p.closed_risers and p.construction != 'concrete':
        for i in range(p.risers):
            y, z = i*p.going, i*rise
            b.box(f'{name}_riser{i+1}', (0, y, z), (p.width, y+p.riser_thickness, z+rise-p.tread_thickness), finish_color)
    top = [(0, rise-p.tread_thickness)]
    for i in range(p.risers-1):
        top.append(((i+1)*p.going, (i+1)*rise-p.tread_thickness))
        if i < p.risers-2:
            top.append(((i+1)*p.going, (i+2)*rise-p.tread_thickness))
    # The underside rises continuously; clip its lower end at the finished floor.
    bottom_end = max(0, p.total_rise-p.tread_thickness-p.stringer_depth)
    floor_run = max(0, min(run, (p.stringer_depth+p.tread_thickness-rise)*p.going/rise))
    outline = top+[(run, bottom_end), (floor_run, 0), (0, 0)]
    if p.construction == 'timber':
        for i, x in enumerate((0, p.width-p.stringer_width)):
            frame = Frame((x, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
            b.prism(f'{name}_stringer{i+1}', outline, frame, p.stringer_width, WOOD)
    else:
        from examples.architecture._stair_structure import build_structure
        build_structure(b, p, rise, run)
    if p.landing_length:
        if p.tread_material != 'none':
            b.box(name+'_landing', (0, run, p.total_rise-p.tread_thickness),
                  (p.width, run+p.landing_length, p.total_rise), finish_color)
        if p.construction == 'timber':
            for i, x in enumerate((0, p.width-p.stringer_width)):
                b.box(f'{name}_landing_joist{i+1}', (x, run, p.total_rise-p.tread_thickness-200),
                      (x+p.stringer_width, run+p.landing_length, p.total_rise-p.tread_thickness), WOOD)
    if p.railings and p.railing_style != 'timber':
        from examples.architecture._stair_guards import build_guard
        build_guard(b, p, rise, run)
    if p.railings and p.railing_style == 'timber':
        a = math.atan(rise/p.going)
        c, s = math.cos(a), math.sin(a)
        # Turned steel spindle shoes are a shared machined component.
        shoe_profile = b.part.sketch(name=name+'_shoe_profile')
        shoe_profile.add_circle((0, 0), p.baluster_diameter/2+9)
        shoe_profile.add_circle((0, 0), p.baluster_diameter/2)
        shoe = shoe_profile.extrude(3, name=name+'_shoe_flange', max_deviation=.1)
        collar = b.part.sketch(frame=Frame((0, 0, 3)), name=name+'_shoe_collar_profile')
        collar.add_circle((0, 0), p.baluster_diameter/2+3)
        collar.add_circle((0, 0), p.baluster_diameter/2)
        shoe = b.part.union(shoe, collar.extrude(10, name=name+'_shoe_collar', max_deviation=.1), name=name+'_spindle_shoe')
        for side, x in (('left', p.newel_width/2), ('right', p.width-p.newel_width/2)):
            if p.railing_sides not in ('both', side):
                continue
            rail = Frame((x+p.handrail_width/2, 0, rise+p.railing_height),
                         x=(-1, 0, 0), y=(0, -s, c), z=(0, c, s))
            grip = rounded_rectangle(b.part, rail, p.handrail_width, p.handrail_height, 12, f'{name}_{side}_handrail')
            sloping = b.add(grip.extrude(run/c, name=f'{name}_{side}_handrail', max_deviation=.1), WOOD)
            rails = [sloping]
            if p.landing_length:
                pose = Frame((x+p.handrail_width/2, run, p.total_rise+p.railing_height),
                             x=(-1, 0, 0), y=(0, 0, 1), z=(0, 1, 0))
                level_rail = rounded_rectangle(b.part, pose, p.handrail_width, p.handrail_height, 12, f'{name}_{side}_landing_rail')
                rails.append(b.add(level_rail.extrude(p.landing_length, name=f'{name}_{side}_landing_rail', max_deviation=.1), WOOD))
            posts = [(0, 0, rise+p.railing_height+p.handrail_height+60),
                     (run, p.total_rise-200, p.total_rise+p.railing_height+p.handrail_height+60)]
            if p.landing_length:
                posts.append((run+p.landing_length, p.total_rise-200, posts[-1][2]))
            for index, (y, bottom, top) in enumerate(posts):
                half = p.newel_width/2
                post_name = f'{name}_{side}_newel{index+1}'
                post = b.part.cuboid((x-half, y-half, bottom), (x+half, y+half, top), name=post_name+'_blank')
                # The rails enter actual receiving mortises in the newels.
                b.add(b.part.batch_subtract(post, rails, name=post_name), WOOD)
                lower = b.part.sketch(frame=Frame((x, y, top)), name=post_name+'_cap_lower')
                lower.add_rectangle((-half-6, -half-6), (half+6, half+6))
                upper = b.part.sketch(frame=Frame((x, y, top+8)), name=post_name+'_cap_upper')
                upper.add_rectangle((-half-2, -half-2), (half+2, half+2))
                b.add(b.part.loft([lower, upper], name=post_name+'_cap'), WOOD)
            count = math.ceil(run/p.baluster_spacing)
            for i in range(count+1):
                y = run*i/count
                if min(y, run-y) < p.newel_width/2+p.baluster_diameter:
                    continue
                level = min(p.risers, math.floor(y/p.going)+1)*rise
                height = rise+y*rise/p.going+p.railing_height
                rod = b.part.cylinder((x, y, level), p.baluster_diameter/2, height-level,
                                      name=f'{name}_{side}_baluster{i+1}', max_deviation=.1)
                b.add(rod, METAL)
                b.add(shoe, METAL, position=(x, y, level))
            if p.landing_length:
                count = math.ceil(p.landing_length/p.baluster_spacing)
                for i in range(1, count+1):
                    y = run+p.landing_length*i/count
                    if min(y-run, run+p.landing_length-y) < p.newel_width/2+p.baluster_diameter:
                        continue
                    rod = b.part.cylinder((x, y, p.total_rise), p.baluster_diameter/2, p.railing_height,
                                          name=f'{name}_{side}_landing_baluster{i}', max_deviation=.1)
                    b.add(rod, METAL)
                    b.add(shoe, METAL, position=(x, y, p.total_rise))
    return b.finish()


stair_component = register_generator(build_stairs, StairParameters, name='stair_component')

if __name__ == '__main__':
    show_component(build_stairs, StairParameters())
