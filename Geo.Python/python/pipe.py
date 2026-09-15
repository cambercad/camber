"""Reusable 90-degree flanged pipe elbow.

`create_pipe()` returns one solid and does not open a window. Run this file
directly to build and display a single pipe.
"""
import math
import weakref

from camber import Part, vec3

_PIPE_CACHE = weakref.WeakKeyDictionary()


def create_pipe(part=None, part_name="pipe"):
    horizontal_len = 150.0
    arc_radius = 260.0
    pipe_radius = 145.0
    pipe_inner_radius = 105.0
    pipe_mean_radius = 125.0
    plate_size = 360.0
    plate_thickness = 50.0
    fillet_radius = 20.0
    end_chamfer = 10.0
    bolt_inset = 130.0
    bolt_radius = 30.0
    through_bolt_radius = 15.0
    bolt_depth = 10.0

    if part is None:
        part = Part(vec3(-1000), vec3(1000), tolerance=0.1)

    cached = _PIPE_CACHE.get(part)
    if cached is not None:
        if cached.name == part_name:
            return cached
        return part.copy_solid(cached, part_name)

    p0 = (0.0, 0.0)
    p1 = (-horizontal_len, 0.0)
    arc_center = (-horizontal_len, arc_radius)
    p2 = (-horizontal_len - arc_radius, arc_radius)
    p3 = (-horizontal_len - arc_radius, arc_radius + horizontal_len)
    arc_mid_angle = 5.0 * math.pi / 4.0
    arc_mid = (
        arc_center[0] + arc_radius * math.cos(arc_mid_angle),
        arc_center[1] + arc_radius * math.sin(arc_mid_angle),
    )

    def named(suffix):
        return part_name + "-" + suffix

    def add_bolt_circles(sketch, radius, prefix):
        sketch.add_circle((-bolt_inset, -bolt_inset), radius, name=prefix + "_sw")
        sketch.add_circle((bolt_inset, -bolt_inset), radius, name=prefix + "_se")
        sketch.add_circle((bolt_inset, bolt_inset), radius, name=prefix + "_ne")
        sketch.add_circle((-bolt_inset, bolt_inset), radius, name=prefix + "_nw")

    guide = part.sketch("xy", name=named("guide"))
    guide.add_line(p0, p1, name="h_line")
    guide.add_arc(p1, arc_mid, p2, name="bend")
    guide.add_line(p2, p3, name="v_line")

    profile = part.sketch("yz", name=named("annulus"))
    profile.add_circle((0.0, 0.0), pipe_radius, name="outer")
    profile.add_circle((0.0, 0.0), pipe_inner_radius, name="inner")

    pipe = part.extrude_along_sketch(profile, guide, name=part_name)

    # Capture both terminal frames before booleans replace the virgin sweep.
    bottom_sk = part.sketch(part_name + "-ExtrudeBottom", name=named("bottom_flange_profile"))
    bottom_sk.add_rectangle_centered(
        (0.0, 0.0), plate_size, plate_size, names=("south", "east", "north", "west")
    )
    bottom_sk.add_circle((0.0, 0.0), pipe_mean_radius, name="clearance")
    bottom_frame = bottom_sk.frame

    top_sk = part.sketch(part_name + "-ExtrudeTop", name=named("top_flange_profile"))
    top_sk.add_rectangle_centered(
        (0.0, 0.0), plate_size, plate_size, names=("south", "east", "north", "west")
    )
    top_sk.add_circle((0.0, 0.0), pipe_mean_radius, name="clearance")
    top_frame = top_sk.frame

    def flange_corners(flange):
        return [
            "[{0}-south,{0}-west]".format(flange),
            "[{0}-south,{0}-east]".format(flange),
            "[{0}-east,{0}-north]".format(flange),
            "[{0}-north,{0}-west]".format(flange),
        ]

    bottom_name = named("bottom_flange")
    top_name = named("top_flange")
    bottom_ext = part.extrude_two_sides(
        bottom_sk, 0.0, plate_thickness, name=bottom_name
    )
    top_ext = part.extrude_two_sides(
        top_sk, plate_thickness, 0.0, name=top_name
    )

    # Union both plates first, then fillet all eight vertical corners. Filleting
    # one flange and then unioning the other re-runs coplanar fusion and can
    # wipe the first set of blends (the C# template fillets each plate right
    # after its own union because nothing else is merged in between).
    pipe = pipe + bottom_ext
    pipe = pipe + top_ext
    pipe = part.fillet(pipe, flange_corners(bottom_name) + flange_corners(top_name), fillet_radius)

    bolt_sk = part.sketch(frame=bottom_frame, name=named("bottom_counterbores"))
    add_bolt_circles(bolt_sk, bolt_radius, "counterbore")
    pipe = pipe - part.extrude_two_sides(
        bolt_sk, 0.0, bolt_depth, name=named("bottom_counterbore_cutter")
    )

    through_sk = part.sketch(frame=bottom_frame, name=named("bottom_bolt_holes"))
    add_bolt_circles(through_sk, through_bolt_radius, "bolt_hole")
    pipe = pipe - part.extrude_two_sides(
        through_sk, 0.0, plate_thickness, name=named("bottom_hole_cutter")
    )

    top_bolt_sk = part.sketch(frame=top_frame, name=named("top_counterbores"))
    add_bolt_circles(top_bolt_sk, bolt_radius, "counterbore")
    pipe = pipe - part.extrude_two_sides(
        top_bolt_sk, bolt_depth, 0.0, name=named("top_counterbore_cutter")
    )

    top_through_sk = part.sketch(frame=top_frame, name=named("top_bolt_holes"))
    add_bolt_circles(top_through_sk, through_bolt_radius, "bolt_hole")
    pipe = pipe - part.extrude_two_sides(
        top_through_sk, plate_thickness, 0.0, name=named("top_hole_cutter")
    )

    # Sweep side = {solid}-{profile curve}-{guide curve}.
    pipe = part.chamfer(
        pipe,
        [
            "[{0}-outer-h_line,{1}-ExtrudeBottom]".format(part_name, bottom_name),
            "[{0}-outer-v_line,{1}-ExtrudeTop]".format(part_name, top_name),
        ],
        end_chamfer,
    )
    _PIPE_CACHE[part] = pipe
    return pipe


if __name__ == "__main__":
    single_pipe = create_pipe(part_name="pipe")
    print(single_pipe)
    single_pipe.show(title="camber 90deg pipe")
