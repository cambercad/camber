"""Hollow steel stringers, tread connections and continuous concrete waists."""
import math
from camber import Frame
from examples.architecture._building import polygon

STEEL = (.16, .17, .18)
HARDWARE = (.50, .52, .54)
CONCRETE = (.67, .68, .67)
PLATE = 8


def support_centers(p):
    if p.stringer_layout == 'central':
        return (p.width/2,)
    return (30+p.stringer_width/2, p.width-30-p.stringer_width/2)


def fixing_positions(p, index):
    width = p.width-160 if p.stringer_layout == 'central' else max(110, p.stringer_width+30)
    depth = p.going*.65
    y = (index+.5)*p.going
    return [(x+dx, y+dy) for x in support_centers(p)
            for dx in (-width/2+22, width/2-22) for dy in (-depth/2+22, depth/2-22)]


def drill_tread(part, tread, p, index, name):
    cutters = []
    top = (index+1)*p.total_rise/p.risers
    for j, (x, y) in enumerate(fixing_positions(p, index)):
        cutters.append(part.cylinder((x, y, top-p.tread_thickness-1), 4.4, p.tread_thickness+2,
                                     name=f'{name}_fixing{j}', max_deviation=.1))
        cutters.append(part.cylinder((x, y, top-6), 7, 7, name=f'{name}_counterbore{j}', max_deviation=.1))
    return part.batch_subtract(tread, cutters, name=name)


def _fasteners(part, p, name):
    def hexagon(radius):
        return [(radius*math.cos(i*math.pi/3), radius*math.sin(i*math.pi/3)) for i in range(6)]
    shank = part.cylinder((0, 0, -p.tread_thickness-PLATE-11), 4, p.tread_thickness+PLATE+5,
                          name=name+'_shank', max_deviation=.1)
    head = part.cylinder((0, 0, -6), 6.5, 6, name=name+'_head', max_deviation=.1)
    socket = polygon(part, hexagon(3.5), Frame((0, 0, -3)), name+'_socket').extrude(4, name=name+'_socket')
    bolt = part.subtract(part.union(shank, head, name=name+'_bolt_blank'), socket, name=name+'_bolt')
    z = -p.tread_thickness-PLATE-1.6
    washer_profile = part.sketch(frame=Frame((0, 0, z)), name=name+'_washer_profile')
    washer_profile.add_circle((0, 0), 8)
    washer_profile.add_circle((0, 0), 4.4)
    washer = washer_profile.extrude(1.6, name=name+'_washer', max_deviation=.1)
    nut_profile = polygon(part, hexagon(7.5), Frame((0, 0, z-6.5)), name+'_nut')
    nut_profile.add_circle((0, 0), 4)
    nut = nut_profile.extrude(6.5, name=name+'_nut', max_deviation=.1)
    return bolt, washer, nut


def build_structure(b, p, rise, run):
    slope = rise/p.going
    c = 1/math.sqrt(1+slope*slope)
    s = slope*c
    if p.construction == 'concrete':
        finish = p.tread_thickness if p.tread_material != 'none' else 0
        depth = rise+p.waist_thickness/c
        points = [(0, rise-finish)]
        for i in range(p.risers-1):
            points.append(((i+1)*p.going, (i+1)*rise-finish))
            if i < p.risers-2:
                points.append(((i+1)*p.going, (i+2)*rise-finish))
        if p.landing_length:
            points += [(run, p.total_rise-finish), (run+p.landing_length, p.total_rise-finish),
                       (run+p.landing_length, p.total_rise-finish-depth), (run, p.total_rise-finish-depth)]
        else:
            points.append((run, p.total_rise-finish-depth))
        points += [(min(run, (depth+finish-rise)/slope), 0), (0, 0)]
        frame = Frame(x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
        b.prism(b.name+'_concrete_waist', points, frame, p.width, CONCRETE)
        return

    top_at_zero = rise/2-p.tread_thickness-PLATE-80
    for i, x in enumerate(support_centers(p)):
        label = f'{b.name}_stringer{i+1}'
        frame = Frame((x, -p.going, top_at_zero-rise), x=(1, 0, 0), y=(0, s, -c), z=(0, c, s))
        profile = b.part.sketch(frame=frame, name=label+'_profile')
        w, h, wall = p.stringer_width, p.stringer_depth, p.steel_wall_thickness
        profile.add_rectangle((-w/2, 0), (w/2, h))
        profile.add_rectangle((-w/2+wall, wall), (w/2-wall, h-wall))
        raw = profile.extrude((run+2*p.going)/c, name=label+'_blank')
        limit = b.part.cuboid((x-w, 0, 0), (x+w, run, p.total_rise), name=label+'_limit')
        b.add(b.part.intersect(raw, limit, name=label), STEEL)
        floor_end = min(run, max(100, (h/c-top_at_zero)/slope))
        base = b.part.sketch(frame=Frame((x, 0, -10)), name=label+'_base_profile')
        base.add_rectangle((-w/2-50, 0), (w/2+50, floor_end+40))
        for dx in (-w/2-25, w/2+25):
            for y in (25, floor_end+15):
                base.add_circle((dx, y), 7)
        b.add(base.extrude(10, name=label+'_baseplate', max_deviation=.1), STEEL)
        if p.landing_length:
            frame = Frame((x, run, p.total_rise-p.tread_thickness-PLATE),
                          x=(1, 0, 0), y=(0, 0, -1), z=(0, 1, 0))
            profile = b.part.sketch(frame=frame, name=label+'_landing_profile')
            profile.add_rectangle((-w/2, 0), (w/2, h))
            profile.add_rectangle((-w/2+wall, wall), (w/2-wall, h-wall))
            b.add(profile.extrude(p.landing_length, name=label+'_landing_beam'), STEEL)
    hardware = _fasteners(b.part, p, b.name+'_saddle_fixing')
    plate_width = p.width-160 if p.stringer_layout == 'central' else max(110, p.stringer_width+30)
    for i in range(p.risers-1):
        y, top = (i+.5)*p.going, (i+1)*rise-p.tread_thickness
        for j, x in enumerate(support_centers(p)):
            label = f'{b.name}_saddle{i+1}_{j+1}'
            profile = b.part.sketch(frame=Frame((x, y, top-PLATE)), name=label+'_profile')
            profile.add_rectangle((-plate_width/2, -p.going*.325), (plate_width/2, p.going*.325))
            for dx in (-plate_width/2+22, plate_width/2-22):
                for dy in (-p.going*.325+22, p.going*.325-22):
                    profile.add_circle((dx, dy), 4.4)
            b.add(profile.extrude(PLATE, name=label, max_deviation=.1), STEEL)
            for k, offset in enumerate((-p.stringer_width/4, p.stringer_width/4)):
                front, back = y-p.going*.18, y+p.going*.18
                points = [(front, top_at_zero+front*slope), (back, top_at_zero+back*slope), (back, top-PLATE), (front, top-PLATE)]
                frame = Frame((x+offset-4, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
                b.prism(label+f'_web{k+1}', points, frame, 8, STEEL)
        for x, fy in fixing_positions(p, i):
            for solid in hardware:
                b.add(solid, HARDWARE, position=(x, fy, (i+1)*rise))
