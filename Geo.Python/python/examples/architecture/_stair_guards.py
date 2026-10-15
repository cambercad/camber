"""Slim tubular guarding with vertical infill or mechanically clamped glass."""
import math
from camber import Frame
from examples.architecture._building import polygon
from examples.architecture.rainwater import DownpipeParameters, build_downpipe

METAL = (.16, .17, .18)
GLASS = (.70, .80, .82)


def build_guard(b, p, rise, run):
    radius = 21.2
    slope = rise/p.going
    # Shared drilled mounting plates; posts and panels follow the actual treads.
    plate = b.part.sketch(name=b.name+'_guard_base_profile')
    plate.add_rectangle((-40, -40), (40, 40))
    for x in (-27, 27):
        for y in (-27, 27):
            plate.add_circle((x, y), 4.4)
    base = plate.extrude(8, name=b.name+'_guard_baseplate', max_deviation=.1)
    clamp = None
    if p.railing_style == 'glass':
        t = p.glass_thickness
        blank = b.part.cuboid((-t/2-8, 0, 0), (t/2+8, 30, 40), name=b.name+'_clamp_blank')
        slot = b.part.cuboid((-t/2-.5, 8, -1), (t/2+.5, 31, 41), name=b.name+'_clamp_slot')
        clamp = b.part.subtract(blank, slot, name=b.name+'_glass_clamp')
    for side, x in (('left', 50), ('right', p.width-50)):
        if p.railing_sides not in ('both', side):
            continue
        label = f'{b.name}_{side}'
        def top(y):
            return rise+min(y, run)*slope+p.railing_height-radius
        def floor(y):
            return min(p.risers, math.floor((y+1e-7)/p.going)+1)*rise
        route = [(x, 0, top(0)), (x, run, top(run))]
        if p.landing_length:
            route.append((x, run+p.landing_length, top(run)))
        handrail = build_downpipe(DownpipeParameters(points=tuple(route), diameter=2*radius,
                                  wall_thickness=2, bend_radius=60, collars=False),
                                  part=b.part, name=label+'_handrail').solids[0]
        b.add(handrail, METAL)
        count = max(1, math.ceil(run/p.post_spacing))
        # Place intermediate posts on tread centres, end posts on the first/last tread.
        stations = [40]+[round((p.risers-1)*i/count)*p.going+40 for i in range(1, count)]+[run+40 if p.landing_length else run-40]
        if p.landing_length:
            count = max(1, math.ceil(p.landing_length/p.post_spacing))
            stations += [run+40+(p.landing_length-80)*i/count for i in range(1, count+1)]
        stations = sorted(set(stations))
        for i, y in enumerate(stations):
            z = floor(y)
            b.add(base, METAL, position=(x, y, z))
            profile = b.part.sketch(frame=Frame((x, y, z+8)), name=f'{label}_post{i}_profile')
            profile.add_circle((0, 0), radius)
            profile.add_circle((0, 0), radius-3)
            post = profile.extrude(top(y)-z-8, name=f'{label}_post{i}_blank', max_deviation=.1)
            b.add(b.part.subtract(post, handrail, name=f'{label}_post{i}'), METAL)
        for i, (a, end) in enumerate(zip(stations, stations[1:])):
            if p.railing_style == 'glass':
                left, right = a+25, end-25
                if right <= left:
                    continue
                def lower(y):
                    return rise+min(y, run)*slope+100
                points = [(left, lower(left)), (right, lower(right)),
                          (right, top(right)-35), (left, top(left)-35)]
                frame = Frame((x-p.glass_thickness/2, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
                b.add(polygon(b.part, points, frame, f'{label}_glass{i}').extrude(
                      p.glass_thickness, name=f'{label}_glass{i}'), GLASS)
                for y, direction in ((a+16, 1), (end-16, -1)):
                    for fraction in (.25, .75):
                        z = lower(y)+(top(y)-35-lower(y))*fraction-20
                        b.add(clamp, METAL, position=(x, y, z),
                              orientation=(0, 0, 0, 1) if direction == 1 else (0, 0, 1, 0))
            else:
                count = max(1, math.ceil((end-a)/p.baluster_spacing))
                for j in range(1, count):
                    y = a+(end-a)*j/count
                    if min(y-a, end-y) <= radius+p.baluster_diameter/2:
                        continue
                    z = floor(y)
                    rod = b.part.cylinder((x, y, z), p.baluster_diameter/2, top(y)-z,
                                          name=f'{label}_infill{i}_{j}', max_deviation=.1)
                    b.add(b.part.subtract(rod, handrail, name=f'{label}_baluster{i}_{j}'), METAL)
