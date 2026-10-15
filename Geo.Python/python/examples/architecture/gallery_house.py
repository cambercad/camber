"""A family house with a gallery, cross-gable, dormers and rooflights.

Millimetres; north is +Y. Run with --render PATH, --no-show or --cutaway.
The plan is a design study, not construction documentation.
"""
from dataclasses import dataclass
import argparse
import math
from pathlib import Path
import sys

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Frame
from camber.modeling_parameters import register_generator
from examples.architecture._building import BuildingComponent, Builder, polygon
from examples.architecture.windows import WindowParameters, build_window
from examples.architecture.doors import DoorParameters, build_door
from examples.architecture.stairs import StairParameters, build_stairs
from examples.architecture.roofs import RoofParameters, build_roof
from examples.architecture.rainwater import GutterParameters, DownpipeParameters, build_gutter, build_downpipe

PLASTER = (.89, .87, .81)
OAK = (.62, .49, .33)
STONE = (.55, .55, .52)
METAL = (.22, .24, .25)
ROOF = (.29, .31, .32)
STAIR_GOING = 270
STAIR_RUN = 17*STAIR_GOING
STAIR_TOP = 6800
STAIR_START = STAIR_TOP-STAIR_RUN


@dataclass(frozen=True)
class HouseWindow:
    name: str
    room: str
    parameters: WindowParameters

    @property
    def clear_glazing_area(self):
        """Visible rectangle area in m²; not a daylight performance calculation."""
        p = self.parameters
        available = p.width-2*p.frame_width-(p.leaves-1)*p.mullion_width
        height = p.height-2*p.frame_width-2*p.gap-2*p.sash_width
        return sum((available*w/sum(p.leaf_weights)-2*p.gap-2*p.sash_width)*height
                   for w in p.leaf_weights)/1e6


@dataclass(frozen=True)
class HouseModel(BuildingComponent):
    windows: tuple[HouseWindow, ...]
    plan_walls: tuple
    plan_joinery: tuple
    plan_boxes: tuple
    stair_parameters: StairParameters
    stair_position: tuple

    def daylight_schedule(self):
        """Room rectangle areas and visible glazing, without compliance claims."""
        return tuple({'level': level, 'room': room, 'plan_area': (x1-x0)*(y1-y0)/1e6,
                      'clear_glazing_area': sum(w.clear_glazing_area for w in self.windows if w.room == room),
                      'windows': tuple(w.name for w in self.windows if w.room == room)}
                     for level, room, (x0, y0, x1, y1) in ROOMS)


WINDOW_ROOMS = {
    'living_glazing': 'Living / double height', 'west_glazing': 'Living / double height',
    'upper_0_0': 'Living / double height', 'rooflight0': 'Living / double height',
    'kitchen_window': 'Kitchen', 'east_window': 'Kitchen',
    'dining_glazing': 'Dining', 'cross_gable_window': 'Dining',
    'study_window': 'Study / guest', 'utility_window': 'Utility / laundry',
    'wc_window': 'Guest WC / shower', 'entrance_window': 'Entrance / coats',
    'upper_180_0': 'Bedroom 2', 'west_dormer_window': 'Bedroom 2',
    'upper_180_1': 'Bedroom 3', 'bath_window': 'Family bathroom',
    'upper_0_1': 'Main bedroom', 'east_dormer_window': 'Main bedroom',
    'rooflight1': 'Gallery / reading', 'rooflight2': 'Landing',
}


@dataclass(frozen=True, kw_only=True)
class HouseParameters:
    cutaway: bool = False
    rainwater: bool = True
    terrace: bool = True

    def __post_init__(self):
        for name in ('cutaway', 'rainwater', 'terrace'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} must be True or False')


# Indicative room label zones, not wall geometry or measured usable floor areas.
ROOMS = (
    (0, 'Living / double height', (350, 350, 5500, 6200)),
    (0, 'Dining', (5650, 350, 8150, 4400)),
    (0, 'Kitchen', (8300, 350, 11650, 4500)),
    (0, 'Study / guest', (350, 6650, 3800, 9650)),
    (0, 'Utility / laundry', (3950, 6650, 6400, 9650)),
    (0, 'Guest WC / shower', (6550, 6650, 8000, 9650)),
    (0, 'Entrance / coats', (8150, 6650, 11650, 9650)),
    (1, 'Bedroom 2', (350, 6650, 4200, 9650)),
    (1, 'Family bathroom', (4350, 8050, 7850, 9650)),
    (1, 'Bedroom 3', (8300, 6650, 11650, 9650)),
    (1, 'Main bedroom', (8300, 350, 11650, 5200)),
    (1, 'Gallery / reading', (350, 5150, 5500, 6500)),
    (1, 'Landing', (5650, 6800, 7850, 7900)),
)


def build_house(parameters=HouseParameters(), *, part=None, name='house'):
    """Build a fixed 12 x 10 m layout with real openings and detailed components.

    Cutaway removes the roof and upper walls; it retains the upper floor and
    gallery guarding. The ground-floor plan is available separately via write_plans.
    """
    p = parameters
    if not isinstance(p, HouseParameters):
        raise TypeError('parameters must be HouseParameters')
    b = Builder(part, name, (-4000, -5000, -1200), (17000, 15000, 13000))
    windows, plan_walls, plan_joinery, plan_boxes = [], [], [], []

    def record_window(label, parameters):
        windows.append(HouseWindow(name+'_'+label, WINDOW_ROOMS[label], parameters))

    def box(label, low, high, rgb=PLASTER, *, plan_wall=False):
        solid = b.box(name+'_'+label, low, high, rgb)
        if plan_wall:
            plan_walls.append(solid)
        else:
            plan_boxes.append((label, low, high))
        return solid

    def component(model, *, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        occurrence = b.assembly.add_subassembly(model.assembly, position=position, orientation=orientation)
        b.assembly.fix(occurrence)
        b.colors.update(model.colors)
        b.solids.update((solid.name, solid) for solid in model.solids)

    def wall(label, low, high, openings=(), rgb=PLASTER, *, clip=None):
        solid = b.part.cuboid(low, high, name=name+'_'+label+'_blank')
        cutters = [b.part.cuboid(a, end, name=f'{name}_{label}_opening{i}')
                   for i, (a, end) in enumerate(openings)]
        if cutters:
            solid = b.part.batch_subtract(solid, cutters, name=name+'_'+label)
        if clip is not None:
            solid = b.part.intersect(solid, clip, name=name+'_'+label+'_clipped')
        b.add(solid, rgb)
        plan_walls.append(solid)

    def joinery(label, width, height, position, angle=0, *, door=False, leaves=2, opening=0):
        a = math.radians(angle)/2
        if door:
            parameters = DoorParameters(width=width, height=height, style='flush',
                               opening_angles=(opening,), leaf_color=OAK, frame_color=PLASTER)
            model = build_door(parameters, part=b.part, name=name+'_'+label)
        else:
            parameters = WindowParameters(width=width, height=height, leaves=leaves, frame_color=METAL)
            model = build_window(parameters, part=b.part, name=name+'_'+label)
            record_window(label, parameters)
        plan_joinery.append((label, parameters, position, angle))
        component(model, position=position, orientation=(0, 0, math.sin(a), math.cos(a)))

    box('foundation', (0, 0, -400), (12000, 10000, -120), STONE)
    box('ground_floor', (350, 350, -120), (11650, 9650, 0), OAK)
    box('bay_floor', (4200, -900, -200), (7800, 350, 0), OAK)
    if p.terrace:
        box('terrace', (-1000, -3400, -180), (13000, -950, -80), STONE)
        for i in range(24):
            box(f'terrace_joint{i}', (-1000+i*600, -3400, -80), (-997+i*600, -950, -79), METAL)

    # South-facing communal rooms; service rooms buffer the entrance to the north.
    wall('south_wall', (0, 0, 0), (12000, 350, 3200), (
        ((900, -1, 0), (3100, 351, 2550)), ((4200, -1, 0), (7800, 351, 3201)),
        ((8900, -1, 1050), (11100, 351, 2450))))
    joinery('living_glazing', 2200, 2550, (900, 95, 0))
    joinery('kitchen_window', 2200, 1400, (8900, 95, 1050))
    wall('bay_front', (4200, -900, 0), (7800, -550, 3200), (((4450, -901, 0), (7550, -549, 2650)),))
    joinery('dining_glazing', 3100, 2650, (4450, -805, 0), leaves=3)
    box('bay_left', (4200, -900, 0), (4550, 0, 3200), plan_wall=True)
    box('bay_right', (7450, -900, 0), (7800, 0, 3200), plan_wall=True)
    wall('north_wall', (0, 9650, 0), (12000, 10000, 3200), (
        ((900, 9649, 900), (3100, 10001, 2300)), ((4600, 9649, 1300), (5800, 10001, 2300)),
        ((6800, 9649, 1500), (7600, 10001, 2450)),
        ((8800, 9649, 0), (9900, 10001, 2300))))
    joinery('study_window', 2200, 1400, (3100, 9905, 900), 180)
    joinery('utility_window', 1200, 1000, (5800, 9905, 1300), 180)
    joinery('wc_window', 800, 950, (7600, 9905, 1500), 180, leaves=1)
    joinery('entry_door', 1100, 2300, (9900, 9905, 0), 180, door=True)
    wall('west_wall', (0, 350, 0), (350, 9650, 3200), (((-1, 1500, 0), (351, 4300, 2550)),))
    joinery('west_glazing', 2800, 2550, (95, 4300, 0), -90, leaves=3)
    wall('east_wall', (11650, 350, 0), (12000, 9650, 3200), (
        ((11649, 1800, 950), (12001, 4200, 2450)),
        ((11649, 7500, 650), (12001, 8700, 2450))))
    joinery('east_window', 2400, 1500, (11905, 1800, 950), 90, leaves=3)
    joinery('entrance_window', 1200, 1800, (11905, 7500, 650), 90)

    wall('service_partition', (350, 6500, 0), (11650, 6650, 2900), tuple(
        ((x, 6499, 0), (x+900, 6651, 2150)) for x in (2850, 5000, 6800, 9000)))
    for label, x in (('study', 2850), ('utility', 5000), ('wc', 6800), ('hall', 9000)):
        joinery(label+'_door', 900, 2150, (x, 6500, 0), door=True)
    for label, x in (('office_divider', 3800), ('utility_divider', 6400), ('entry_divider', 8000)):
        box(label, (x, 6650, 0), (x+150, 9650, 2900), plan_wall=True)
    box('entrance_storage', (8150, 6950, 0), (8750, 9200, 2400), OAK)
    # Kitchen is beside the service core; the island leaves 1.1 m working aisles.
    box('kitchen_base', (11050, 650, 0), (11650, 4450, 880), OAK)
    box('kitchen_worktop', (11020, 620, 880), (11670, 4480, 920), STONE)
    box('kitchen_island', (8700, 1600, 0), (9650, 3700, 880), OAK)
    box('island_worktop', (8650, 1550, 880), (9700, 3750, 920), STONE)

    # The floor plate deliberately excludes the living-room void and stair well.
    for i, (a, end) in enumerate((((350, 5150, 2900), (5650, 9650, 3200)),
                                ((5650, 350, 2900), (6800, 9650, 3200)),
                                ((6800, 350, 2900), (8000, STAIR_START-200, 3200)),
                                ((6800, 6800, 2900), (8000, 9650, 3200)),
                                ((8000, 350, 2900), (11650, 9650, 3200)))):
        box(f'upper_floor{i}', a, end, OAK)
    stair_parameters = StairParameters(width=1000, total_rise=3200, risers=18, going=STAIR_GOING,
                         landing_length=0, railing_sides='both', railing_style='metal')
    stair_position = (6900, STAIR_START, 0)
    stairs = build_stairs(stair_parameters, part=b.part, name=name+'_stairs')
    component(stairs, position=stair_position)
    # Guard the gallery edge and the opposite side of the stair opening.
    for label, a, end in (('gallery', (400, 5125, 3200), (5650, 5125, 3200)),
                          ('void_return', (5650, 350, 3200), (5650, 5125, 3200)),
                          ('stair_guard', (8000, STAIR_START-200, 3200), (8000, STAIR_TOP, 3200))):
        dx, dy = end[0]-a[0], end[1]-a[1]
        length = math.hypot(dx, dy)
        count = math.ceil(length/110)
        for i in range(count+1):
            x, y = a[0]+dx*i/count, a[1]+dy*i/count
            b.add(b.part.cylinder((x, y, 3200), 7, 1000, name=f'{name}_{label}_rod{i}', max_deviation=.2), METAL)
        low = (min(a[0], end[0])-18, min(a[1], end[1])-18, 4200)
        high = (max(a[0], end[0])+18, max(a[1], end[1])+18, 4235)
        box(label+'_grip', low, high, OAK)

    if not p.cutaway:
        _upper_enclosure(b, wall, joinery, record_window, plan_walls)
        if p.rainwater:
            for side, x in (('west', -380), ('east', 12380)):
                gutter = build_gutter(GutterParameters(length=10900, radius=90, elevation=4250,
                                     outlet_diameter=90, outlet_offset=250, fall=35), part=b.part, name=name+'_'+side+'_gutter')
                component(gutter, position=(x, -450, 0))
                gx, gy, gz = gutter.outlet
                inset = 530 if side == 'west' else -530
                pipe = build_downpipe(DownpipeParameters(points=((gx, gy, gz), (gx, gy, gz-350),
                      (gx+inset, gy, gz-900), (gx+inset, gy, 100)), diameter=90, bend_radius=120),
                      part=b.part, name=name+'_'+side+'_downpipe')
                component(pipe, position=(x, -450, 0))
    model = b.finish()
    return HouseModel(model.part, model.assembly, model.solids, model.colors, tuple(windows),
                      tuple(plan_walls), tuple(plan_joinery), tuple(plan_boxes), stair_parameters, stair_position)


def _upper_enclosure(b, wall, joinery, record_window, plan_walls):
    """Intersecting roof envelopes form open valleys, rather than overlapping lids."""
    name, part = b.name, b.part
    slope = math.tan(math.radians(35))
    def envelope(label, x0, x1, y0, y1, eave, pitch, base, *, across=False):
        width = (y1-y0) if across else (x1-x0)
        peak = eave+width/2*math.tan(math.radians(pitch))
        outline = [(0, base), (width, base), (width, eave), (width/2, peak), (0, eave)]
        frame = (Frame((x0, y0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0)) if across else
                 Frame((x0, y1, 0), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)))
        return polygon(part, outline, frame, name+'_'+label).extrude(
            x1-x0 if across else y1-y0, name=name+'_'+label)

    # Base roof builder is used directly; penetrations are cut through both slopes.
    main = build_roof(RoofParameters(width=12000, length=10000, pitch=35, eave_height=4700,
                                    overhang=450, thickness=220), part=part, name=name+'_main_roof')
    additions = [('cross_gable', 4200, 7800, -900, 3300, 6500, 35, False),
                 ('east_dormer', 9600, 12500, 1100, 4700, 6600, 12, True),
                 ('west_dormer', -500, 2500, 6800, 9200, 6600, 18, True)]
    inners, outers = [], []
    for label, x0, x1, y0, y1, eave, pitch, across in additions:
        outers.append(envelope(label+'_outer', x0, x1, y0, y1, eave, pitch, 3200, across=across))
        inners.append(envelope(label+'_inner', x0+180, x1-180, y0+180, y1-180,
                     eave-220+180*math.tan(math.radians(pitch)), pitch, 3190, across=across))
    main_void = envelope('main_void', 350, 11650, 350, 9650, 4480+350*slope, 35, 3190)
    voids = [main_void]+inners
    upper = envelope('upper_walls_blank', 0, 12000, 0, 10000, 4480, 35, 3200)
    upper = part.batch_subtract(upper, voids, name=name+'_upper_walls')
    # Upper floor windows are also real wall cuts.
    window_cuts = [part.cuboid((x, -1, 3700), (x+2200, 351, 4900), name=f'{name}_gable_cut{i}')
                   for i, x in enumerate((900, 8900))]
    window_cuts += [part.cuboid((x, 9649, 3700), (x+2200, 10001, 4900), name=f'{name}_rear_cut{i}')
                    for i, x in enumerate((900, 8900))]
    window_cuts.append(part.cuboid((4600, 9649, 4400), (5800, 10001, 5300), name=name+'_bath_window_cut'))
    enclosure = part.batch_subtract(upper, window_cuts, name=name+'_upper_enclosure')
    b.add(enclosure, PLASTER)
    plan_walls.append(enclosure)
    for y, angle in ((95, 0), (9905, 180)):
        for i, x in enumerate((900, 8900)):
            joinery(f'upper_{angle}_{i}', 2200, 1200, (x if angle == 0 else x+2200, y, 3700), angle)
    joinery('bath_window', 1200, 900, (5800, 9905, 4400), 180)
    cuts = inners[:]
    # Rooflights: a proper pitched window frame with the same detailed sash builder.
    a = math.radians(35)
    c, s = math.cos(a), math.sin(a)
    rooflights = ((Frame((2400, 3400, 4700+2400*slope), x=(0, -1, 0), y=(c, 0, s), z=(-s, 0, c)), 1000, 1500, -1),
                  (Frame((2400, 6400, 4700+2400*slope), x=(0, -1, 0), y=(c, 0, s), z=(-s, 0, c)), 1000, 1500, -1),
                  (Frame((7600, 7050, 4700+4400*slope), x=(0, 1, 0), y=(-c, 0, s), z=(s, 0, c)), 800, 1200, 1))
    for i, (frame, width, height, direction) in enumerate(rooflights):
        opening = polygon(part, [(0, 0), (width, 0), (width, height), (0, height)],
                          frame.offset(-frame.z*400), f'{name}_rooflight_cut{i}')
        cuts.append(opening.extrude(800, name=f'{name}_rooflight_opening{i}'))
        parameters = WindowParameters(width=width, height=height, leaves=1, hinge_sides=('fixed',),
                                      sill=False, frame_color=METAL)
        model = build_window(parameters, part=part, name=f'{name}_rooflight{i}')
        record_window(f'rooflight{i}', parameters)
        ax, az = (a-math.pi/2)/2, direction*math.pi/4
        q = (math.cos(az)*math.sin(ax), math.sin(az)*math.sin(ax), math.sin(az)*math.cos(ax), math.cos(az)*math.cos(ax))
        b.assembly.fix(b.assembly.add_subassembly(model.assembly, position=tuple(frame.origin), orientation=q))
        b.colors.update(model.colors)
        b.solids.update((s.name, s) for s in model.solids)
    for i, solid in enumerate(main.solids):
        b.add(part.batch_subtract(solid, cuts, name=f'{name}_roof_slope{i}'), ROOF)
    # Remove the main interior from each addition so there are no hidden roof lids.
    main_outer = envelope('main_outer', 0, 12000, 0, 10000, 4700, 35, 3190)
    for i, (outer, inner) in enumerate(zip(outers, inners)):
        shell = part.batch_subtract(outer, [inner, main_outer], name=f'{name}_addition{i}')
        if i == 0:
            hole = part.cuboid((4450, -901, 3500), (7550, -719, 6100), name=name+'_cross_window_cut')
            joinery('cross_gable_window', 3100, 2600, (4450, -805, 3500), leaves=3)
        elif i == 1:
            hole = part.cuboid((12319, 1350, 4050), (12501, 4450, 6350), name=name+'_east_dormer_cut')
            joinery('east_dormer_window', 3100, 2300, (12405, 1350, 4050), 90, leaves=3)
        else:
            hole = part.cuboid((-501, 7050, 4050), (-319, 8950, 6350), name=name+'_west_dormer_cut')
            joinery('west_dormer_window', 1900, 2300, (-405, 8950, 4050), -90)
        shell = part.subtract(shell, hole, name=f'{name}_dormer_shell{i}')
        b.add(shell, METAL)
        plan_walls.append(shell)
    # Upper partitions leave the double-height volume open to the gallery.
    ceiling = part.batch_union(voids, name=name+'_ceiling_limit')
    wall('upper_north_partition', (350, 6500, 3200), (11650, 6650, 10000), tuple(
        ((x, 6499, 3200), (x+900, 6651, 5350)) for x in (3000, 8500))+
        (((4200, 6499, 3199), (8300, 6651, 10001)),), clip=ceiling)
    for label, x in (('bedroom2', 3000), ('bedroom3', 8500)):
        joinery(label+'_door', 900, 2150, (x, 6500, 3200), door=True)
    wall('bedroom2_side', (4200, 6650, 3200), (4350, 9650, 10000), clip=ceiling)
    wall('bath_side', (7850, 8050, 3200), (8150, 9650, 10000), clip=ceiling)
    wall('bedroom3_side', (8150, 6650, 3200), (8300, 9650, 10000),
         (((8149, 6850, 3200), (8301, 7750, 5350)),), clip=ceiling)
    joinery('bedroom3_landing_door', 900, 2150, (8150, 7750, 3200), -90, door=True)
    wall('bath_front', (4350, 7900, 3200), (7850, 8050, 10000), (((4700, 7899, 3200), (5600, 8051, 5350)),), clip=ceiling)
    joinery('bathroom_door', 900, 2150, (4700, 7900, 3200), door=True)
    wall('main_bedroom_side', (8150, 350, 3200), (8300, 6650, 10000), (((8149, 5400, 3200), (8301, 6300, 5350)),), clip=ceiling)
    joinery('main_bedroom_door', 900, 2150, (8150, 6300, 3200), -90, door=True)
    wall('main_bedroom_rear', (8300, 5200, 3200), (11650, 5350, 10000),
         (((9000, 5199, 3200), (9900, 5351, 5350)),), clip=ceiling)
    joinery('bedroom_vestibule_door', 900, 2150, (9900, 5350, 3200), 180, door=True)


house_component = register_generator(build_house, HouseParameters, name='house_component')


def write_plans(path, *, model=None):
    """Regenerate architectural plans from the built model and its component inputs."""
    from examples.architecture.floor_plans import write_house_plans
    write_house_plans(path, model if model is not None else build_house(), rooms=ROOMS)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cutaway', action='store_true')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--render')
    parser.add_argument('--plans')
    args = parser.parse_args()
    model = build_house(HouseParameters(cutaway=args.cutaway))
    if args.plans:
        write_plans(args.plans, model=model)
    print(f'{len(model.assembly.leaves())} components')
    if args.render:
        from camber import render_views, section
        render_views(model.assembly, args.render, views=['iso', 'rear iso', '+Z'], colors=model.colors, checker=False, tile_size=(1400, 1000), individual=True)
        target = Path(args.render)
        for level, height in (('ground', 1500), ('upper', 4800)):
            render_views(section(model.assembly, offset=height), target.with_name(target.stem+'_'+level+target.suffix),
                         views=['+Z'], colors=model.colors, checker=False, tile_size=(1400, 1200))
        render_views(section(model.assembly, 'YZ', offset=7400), target.with_name(target.stem+'_stairs'+target.suffix),
                     views=['+X'], colors=model.colors, checker=False, tile_size=(1400, 1200))
    elif not args.no_show:
        model.assembly.show(colors=model.colors, checker=False)
