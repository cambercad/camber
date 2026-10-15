"""Physical tile bodies and curved ridge sections, shared by roof layouts."""
import math
from camber import Frame, LoftOptions


def clay_tile(part, frame, p, name):
    """Cambered plain or interlocking S-profile body, nibs and fixing holes."""
    width = p.tile_width-p.tile_gap
    left, right = p.tile_gap/2, p.tile_gap/2+width
    sections, first_curves = [], []
    for i in range(5):
        u = i/4
        height = 4*p.tile_camber*u*(1-u)
        section_frame = Frame(frame.origin+frame.y*(u*p.tile_length)+frame.z*height,
                              x=frame.x, y=frame.z, z=-frame.y)
        profile = part.sketch(frame=section_frame, name=f'{name}_section{i}')
        if p.tile_profile == 'plain':
            first_curve = profile.add_arc((left, 0), ((left+right)/2, p.cross_camber), (right, 0), name='bottom').name
            profile.add_line((right, 0), (right, p.tile_thickness))
            profile.add_arc((right, p.tile_thickness), ((left+right)/2, p.cross_camber+p.tile_thickness),
                            (left, p.tile_thickness))
            profile.add_line((left, p.tile_thickness), (left, 0))
        else:
            first_curve = _pantile_profile(profile, p)
        sections.append(profile)
        first_curves.append(first_curve)
    solid = part.loft(sections, options=LoftOptions(correspondence='vertices'), first_curves=first_curves,
                      name=name+'_body', max_deviation=.1)
    nibs, holes = [], []
    for index, x in enumerate((p.tile_width*.28, p.tile_width*.72)):
        origin = frame.origin+frame.x*(x-9)+frame.y*(p.tile_length-28)-frame.z*8
        nib_frame = Frame(origin, x=frame.x, y=frame.y, z=frame.z)
        rib_height = p.cross_camber if p.tile_profile == 'plain' else p.pantile_rise
        def underside(px, py):
            if p.tile_profile == 'pantile':
                cross = _pantile_height(p, px)
            else:
                radius = width*width/(8*p.cross_camber)+p.cross_camber/2
                cross = math.sqrt(radius*radius-(px-(left+right)/2)**2)-radius+p.cross_camber
            u = py/p.tile_length
            return cross+4*p.tile_camber*u*(1-u)
        root = min(underside(px, py) for px in (x-9, x, x+9)
                   for py in (p.tile_length-28, p.tile_length-21, p.tile_length-14))+p.tile_thickness/3
        nibs.append(part.cuboid(nib_frame, (18, 14, 8+root), name=f'{name}_nib{index}'))
        hole_frame = frame.offset(frame.x*x+frame.y*(p.tile_length-38)-frame.z*10)
        hole = part.sketch(frame=hole_frame, name=f'{name}_fixing{index}')
        hole.add_circle((0, 0), 2.5)
        holes.append(hole.extrude(20+p.tile_thickness+p.tile_camber+rib_height, name=f'{name}_hole{index}', max_deviation=.1))
    body = part.batch_union([solid]+nibs, name=name+'_with_nibs')
    return part.batch_subtract(body, holes, name=name)


def _pantile_height(p, x):
    return p.pantile_rise*.5*(1-math.sin(2*math.pi*(x-p.side_lap)/(p.tile_width-p.side_lap)))


def _pantile_profile(profile, p):
    """S-shaped water channel and a recessed side lap, not a wavy solid block."""
    def shape(x):
        return _pantile_height(p, x)
    t = p.tile_thickness
    def curve(start, end, shift, name=None):
        points = [(x, shape(x)+shift) for x in (start+(end-start)*i/16 for i in range(17))]
        curve = profile.add_spline(points, name=name)
        return points[0], points[-1], curve.name
    low, high = p.side_lap-2, p.side_lap+2
    first, end, first_name = curve(0, low, -t-1, 'bottom')
    profile.add_line(end, (high, shape(high)))
    _, end, _ = curve(high, p.tile_width, 0)
    profile.add_line(end, (p.tile_width, shape(p.tile_width)+t))
    _, end, _ = curve(p.tile_width, high, t)
    profile.add_line(end, (low, shape(low)-1))
    _, end, _ = curve(low, 0, -1)
    profile.add_line(end, first)
    return first_name


def ridge_tile(part, frame, radius, thickness, length, name):
    profile = part.sketch(frame=frame, name=name+'_profile')
    r, inside = radius, radius-thickness
    profile.add_arc((-r, 0), (0, r), (r, 0))
    profile.add_line((r, 0), (inside, 0))
    profile.add_arc((inside, 0), (0, inside), (-inside, 0))
    profile.add_line((-inside, 0), (-r, 0))
    return profile.extrude(length, name=name, max_deviation=.15)
