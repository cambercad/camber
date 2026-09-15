"""700c / 29er rear wheel — clincher rim, 2.25 in MTB tire, 3-cross J-bend spokes, 160 mm RT56-style disc.

Hub flange and 6-bolt mount numbers come from bike_cassette.

From Geo.Python, with the venv active:
    python python\\bike_wheel.py
"""
import math

from camber import Frame, vec2, vec3

from bike_cassette import (
    DISC_BOLT_BCD,
    DISC_BOLT_HOLE_R,
    DISC_BOLT_N,
    angle_mid,
    hub_spoke_layout,
    new_part,
    polar,
    xy,
)

TWO_PI = 2.0 * math.pi
BSD = 622.0
# 29 x 2.25 (57-622). Same bead seat as 700c; the carcass is a square MTB section.
TIRE_SECTION = 57.0
TREAD_CROWN = 40.5
CENTER_KNOBS = 36
SHOULDER_KNOBS = 22
SPOKE_R = 1.0
SPOKE_HEAD_R = 1.7
THROUGH_LEN = 7.2
# Centerline radius of a 14g J-bend. The old sphere-at-the-corner was only SPOKE_R.
BEND_R = 2.6
CROSS = 3

ROTOR_OD = 160.0
ROTOR_T = 1.8
ROTOR_TRACK = 16.5
ROTOR_HUB_R = 29.0
ROTOR_SCALLOPS = 20
ROTOR_SCALLOP_AMP = 2.3
ROTOR_ARMS = 6
ROTOR_ARM_SWEEP = math.radians(46.0)
ROTOR_ARM_HALF = 6.0
ROTOR_FLOWER_MIN = 11.0
ROTOR_FLOWER_MAX = 17.2
ROTOR_HOLE_R = 1.55
ROTOR_ARM_HOLE_R = 2.25
BOLT_R = 2.3
BOLT_H = 8.0


def meridian():
    """Sketch X = hub Z, sketch Y = radius. Revolve around the hub axis."""
    return Frame(vec3(0.0, 0.0, 0.0), x=(0.0, 0.0, 1.0), y=(1.0, 0.0, 0.0), z=(0.0, 1.0, 0.0))


def spoke_bed_r():
    return 0.5 * BSD - 19.0


def add_poly(sk, points, name):
    sk.add_line(points[0], points[1], name=name)
    for point in points[2:]:
        sk.append_line(point)
    sk.append_line(points[0])


def create_rim(part, layout, name="rim", max_deviation=0.1):
    """Hooked clincher section: bead hooks, well, sidewalls, spoke bed."""
    z = 0.5 * (layout["left_z"] + layout["right_z"])
    bsd = 0.5 * BSD
    pts = [
        (z - 5.0, bsd - 19.0),
        (z - 9.0, bsd - 15.5),
        (z - 10.6, bsd - 7.0),
        (z - 10.6, bsd + 0.4),
        (z - 9.6, bsd + 3.4),
        (z - 8.0, bsd + 1.1),
        (z - 4.2, bsd - 3.6),
        (z, bsd - 5.2),
        (z + 4.2, bsd - 3.6),
        (z + 8.0, bsd + 1.1),
        (z + 9.6, bsd + 3.4),
        (z + 10.6, bsd + 0.4),
        (z + 10.6, bsd - 7.0),
        (z + 9.0, bsd - 15.5),
        (z + 5.0, bsd - 19.0),
    ]
    sk = part.sketch(frame=meridian(), name=name + "_section")
    add_poly(sk, pts, "od")
    body = part.revolve(sk, TWO_PI, name=name + "_blank", max_deviation=max_deviation)
    return punch_spoke_holes(part, body, layout, name, max_deviation)


def punch_spoke_holes(part, rim, layout, name, max_deviation):
    """One radial hole in the spoke bed for each nipple."""
    z_mid = 0.5 * (layout["left_z"] + layout["right_z"])
    count = 2 * int(layout["holes"])
    r0 = spoke_bed_r() - 4.0
    r1 = spoke_bed_r() + 7.0
    cutter = None
    for k in range(count):
        a = TWO_PI * k / count
        start = vec3(r0 * math.cos(a), r0 * math.sin(a), z_mid)
        end = vec3(r1 * math.cos(a), r1 * math.sin(a), z_mid)
        pose, length = frame_along(start, end)
        hole = part.cylinder(
            pose, 2.1, length, name=name + "_hole{0}".format(k), max_deviation=max_deviation)
        cutter = hole if cutter is None else part.union(cutter, hole, name=name + "_holes")
    return part.cut(rim, cutter, name=name)


def tire_carcass_points(z_mid):
    """Open-sidewall 2.25 in section. Sketch X = hub Z, Y = radius."""
    bsd = 0.5 * BSD
    crown = bsd + TREAD_CROWN
    return [
        (z_mid - 9.0, bsd + 0.8),
        (z_mid - 11.0, bsd + 6.5),
        (z_mid - 26.5, bsd + 16.0),
        (z_mid - 28.0, bsd + 28.0),
        (z_mid - 21.0, bsd + 36.0),
        (z_mid - 12.0, bsd + 39.4),
        (z_mid, crown),
        (z_mid + 12.0, bsd + 39.4),
        (z_mid + 21.0, bsd + 36.0),
        (z_mid + 28.0, bsd + 28.0),
        (z_mid + 26.5, bsd + 16.0),
        (z_mid + 11.0, bsd + 6.5),
        (z_mid + 9.0, bsd + 0.8),
    ]


def _knob_box(part, origin, axis_x, axis_y, axis_z, hx, hy, hz, inset, name):
    """Block knob: pose +Z is outward. Cuboid grows from the inner corner."""
    corner = origin - axis_x * hx - axis_y * hy - axis_z * inset
    return part.cuboid(
        Frame(corner, x=axis_x, y=axis_y, z=axis_z),
        (2.0 * hx, 2.0 * hy, hz + inset),
        name=name)


def _tread_axes(angle, tilt=0.0):
    """Outward, circumferential, across-tread. tilt tips the knob toward a sidewall."""
    radial = vec3(math.cos(angle), math.sin(angle), 0.0)
    across = vec3(0.0, 0.0, 1.0)
    if abs(tilt) > 1e-6:
        outward = (radial * math.cos(tilt) + across * math.sin(tilt)).normalized()
    else:
        outward = radial
    circ = across.cross(outward).normalized()
    across = outward.cross(circ).normalized()
    return circ, across, outward


def make_tread_knobs(part, z_mid, name):
    """XC-style blocks: staggered center paddles and taller shoulder knobs.

    No 3D circular pattern in the API, so each knob is posed and then batch-unioned.
    """
    bsd = 0.5 * BSD
    crown_r = bsd + TREAD_CROWN - 1.2
    knobs = []

    for i in range(CENTER_KNOBS):
        a = TWO_PI * i / CENTER_KNOBS
        side = 1.0 if i % 2 == 0 else -1.0
        circ, across, outward = _tread_axes(a)
        origin = vec3(crown_r * math.cos(a), crown_r * math.sin(a), z_mid + side * 5.6)
        knobs.append(_knob_box(
            part, origin, circ, across, outward,
            3.6, 4.1, 3.8, 1.4,
            "{0}_c{1}".format(name, i)))

    shoulder_r = bsd + 33.5
    tilt = math.radians(38.0)
    for side in (-1.0, 1.0):
        for i in range(SHOULDER_KNOBS):
            a = TWO_PI * (i + 0.5) / SHOULDER_KNOBS
            circ, across, outward = _tread_axes(a, side * tilt)
            origin = vec3(
                shoulder_r * math.cos(a),
                shoulder_r * math.sin(a),
                z_mid + side * 20.5)
            label = "l" if side < 0.0 else "r"
            knobs.append(_knob_box(
                part, origin, circ, across, outward,
                5.2, 4.4, 5.4, 1.6,
                "{0}_s{1}{2}".format(name, label, i)))
    return knobs


def cut_tread_channels(part, carcass, z_mid, name, max_deviation):
    """Two circumferential grooves between the center and shoulder rows."""
    bsd = 0.5 * BSD
    for i, dz in enumerate((-12.2, 12.2)):
        sk = part.sketch(frame=meridian(), name=name + "_groove{0}".format(i))
        sk.add_circle((z_mid + dz, bsd + TREAD_CROWN), 1.7, name="g")
        cutter = part.revolve(sk, TWO_PI, name=name + "_groove{0}c".format(i), max_deviation=max_deviation)
        carcass = part.cut(carcass, cutter, name=name + "_chan{0}".format(i))
    return carcass


def create_tire(part, layout, name="tire", max_deviation=0.1):
    """29 x 2.25 knobby: revolved carcass, two sipes, then discrete tread blocks."""
    z_mid = 0.5 * (layout["left_z"] + layout["right_z"])
    sk = part.sketch(frame=meridian(), name=name + "_section")
    add_poly(sk, tire_carcass_points(z_mid), "carcass")
    body = part.revolve(sk, TWO_PI, name=name + "_carcass", max_deviation=max_deviation)
    body = cut_tread_channels(part, body, z_mid, name, max_deviation)
    knobs = make_tread_knobs(part, z_mid, name)
    tread = part.batch_union(knobs)
    return part.union(body, tread, name=name)


def sketch_rotor_rim(sk, r_outer):
    """Scalloped OD and flower bore as tangent-ready circular arcs (two per period)."""
    sk.solve_after_every_constraint = False
    r_peak = r_outer
    r_dip = r_outer - ROTOR_SCALLOP_AMP
    r_mean = 0.5 * (r_peak + r_dip)
    step = TWO_PI / ROTOR_SCALLOPS
    hill0 = None
    valley0 = None
    for i in range(ROTOR_SCALLOPS):
        a0 = step * i
        h0 = polar(r_mean, a0 - 0.25 * step)
        hm = polar(r_peak, a0)
        h1 = polar(r_mean, a0 + 0.25 * step)
        vm = polar(r_dip, a0 + 0.5 * step)
        v1 = polar(r_mean, a0 + 0.75 * step)
        if i == 0:
            hill0 = sk.add_arc(h0, hm, h1, name="hill0")
            valley0 = sk.add_arc(h1, vm, v1, name="scallop0")
            sk.coincident(hill0 @ 1.000, valley0 @ 0.000)
            try:
                sk.tangent_circles(hill0, valley0)
            except Exception:
                pass
        else:
            sk.add_arc(h0, hm, h1)
            sk.add_arc(h1, vm, v1)

    for i in range(ROTOR_ARMS):
        a0 = -TWO_PI * i / ROTOR_ARMS
        a1 = -TWO_PI * (i + 1) / ROTOR_ARMS
        am = 0.5 * (a0 + a1)
        if i == 0:
            sk.add_arc(
                polar(ROTOR_FLOWER_MIN, a0),
                polar(ROTOR_FLOWER_MAX, am),
                polar(ROTOR_FLOWER_MIN, a1),
                name="petal0")
        else:
            sk.add_arc(
                polar(ROTOR_FLOWER_MIN, a0),
                polar(ROTOR_FLOWER_MAX, am),
                polar(ROTOR_FLOWER_MIN, a1))


def rotor_arm_edge(i, leading, r_in, r_out):
    """Swept banana edge of one spider arm. leading=True is the convex front."""
    a0 = TWO_PI * i / ROTOR_ARMS
    da_in = ROTOR_ARM_HALF / r_in
    da_out = ROTOR_ARM_HALF / r_out
    if leading:
        a_in = a0 - da_in
        a_out = a0 + ROTOR_ARM_SWEEP - da_out
        mid_a = a0 + 0.50 * ROTOR_ARM_SWEEP - 0.5 * (da_in + da_out) + 0.14
        mid_r = 0.48 * r_in + 0.52 * r_out + 3.5
    else:
        a_in = a0 + da_in
        a_out = a0 + ROTOR_ARM_SWEEP + da_out
        mid_a = a0 + 0.50 * ROTOR_ARM_SWEEP + 0.5 * (da_in + da_out) - 0.05
        mid_r = 0.52 * r_in + 0.48 * r_out - 1.2
    return polar(r_in, a_in), polar(mid_r, mid_a), polar(r_out, a_out), a_in, a_out


def sketch_rotor_window(sk, r_in, r_out, k=0):
    """Petal cutout between arm k (trailing) and arm k+1 (leading)."""
    nxt = (k + 1) % ROTOR_ARMS
    _t0, t_mid, t_out, a_t_in, a_t_out = rotor_arm_edge(k, False, r_in, r_out)
    t_in = polar(r_in, a_t_in)
    l_in, l_mid, l_out, a_l_in, a_l_out = rotor_arm_edge(nxt, True, r_in, r_out)
    hub_mid = polar(r_in, angle_mid(a_t_in, a_l_in))
    ring_mid = polar(r_out, angle_mid(a_t_out, a_l_out))
    sk.add_arc(t_in, hub_mid, l_in, name="win{0}_hub".format(k))
    sk.add_arc(l_in, l_mid, l_out, name="win{0}_lead".format(k))
    sk.add_arc(l_out, ring_mid, t_out, name="win{0}_ring".format(k))
    sk.add_arc(t_out, t_mid, t_in, name="win{0}_trail".format(k))


def add_hole_ring(sk, radius, count, hole_r, name, a0=0.0):
    seed = sk.add_circle(
        (radius * math.cos(a0), radius * math.sin(a0)), hole_r, name=name)
    if count > 1:
        sk.repeat_circular([seed], (0.0, 0.0), count)
    return seed


def create_rotor(part, layout, name="rotor", max_deviation=0.1):
    """160 mm 6-bolt rotor in the SM-RT56 style: scalloped ring, swept arms, hole rows."""
    z = layout["disc_z"] - ROTOR_T - 0.2
    r_outer = 0.5 * ROTOR_OD
    r_track = r_outer - ROTOR_TRACK

    blank_sk = part.sketch(frame=xy(z), constrained=True, name=name + "_blank")
    sketch_rotor_rim(blank_sk, r_outer)
    body = part.extrude(blank_sk, ROTOR_T, name=name + "_blank", max_deviation=max_deviation)

    windows = None
    for k in range(ROTOR_ARMS):
        win_sk = part.sketch(frame=xy(z - 0.2), name=name + "_win{0}".format(k))
        sketch_rotor_window(win_sk, ROTOR_HUB_R, r_track + 0.6, k)
        piece = part.extrude(
            win_sk, ROTOR_T + 0.4, name=name + "_win{0}e".format(k), max_deviation=max_deviation)
        windows = piece if windows is None else part.union(
            windows, piece, name=name + "_windows")
    body = part.cut(body, windows, name=name + "_spider")

    holes = part.sketch(frame=xy(z - 0.2), name=name + "_holes")
    add_hole_ring(holes, 0.5 * DISC_BOLT_BCD, DISC_BOLT_N, DISC_BOLT_HOLE_R, "bolt0")
    add_hole_ring(holes, 33.0, ROTOR_ARMS, ROTOR_ARM_HOLE_R, "armh0", 0.10)
    add_hole_ring(holes, r_outer - 3.3, 40, ROTOR_HOLE_R, "out0")
    add_hole_ring(holes, r_outer - 8.1, 40, ROTOR_HOLE_R, "mid0", math.pi / 40.0)
    add_hole_ring(holes, r_outer - 13.2, 20, ROTOR_HOLE_R, "in0")
    cutter = part.extrude(holes, ROTOR_T + 0.4, name=name + "_cut", max_deviation=max_deviation)
    return part.cut(body, cutter, name=name)


def create_rotor_bolts(part, layout, name="rotor_bolt", max_deviation=0.1):
    z = layout["disc_z"] - ROTOR_T - 0.4
    bolts = []
    for i in range(DISC_BOLT_N):
        a = TWO_PI * i / DISC_BOLT_N
        origin = vec3(
            0.5 * DISC_BOLT_BCD * math.cos(a),
            0.5 * DISC_BOLT_BCD * math.sin(a),
            z)
        bolts.append(part.cylinder(
            Frame(origin), BOLT_R, BOLT_H, name="{0}{1}".format(name, i),
            max_deviation=max_deviation))
    return bolts


def frame_along(start, end):
    z = (end - start).normalized()
    hint = vec3(0.0, 0.0, 1.0) if abs(z.z) < 0.85 else vec3(1.0, 0.0, 0.0)
    x = hint.cross(z).normalized()
    y = z.cross(x)
    return Frame(start, x=x, y=y, z=z), (end - start).norm()


def dot3(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z


def flange_points(z, pcd, count, angle0=0.0):
    r = 0.5 * pcd
    return [
        vec3(
            r * math.cos(angle0 + TWO_PI * i / count),
            r * math.sin(angle0 + TWO_PI * i / count),
            z)
        for i in range(count)
    ]


def rim_nipple(k, z_mid, holes):
    a = TWO_PI * k / holes
    r = spoke_bed_r()
    return vec3(r * math.cos(a), r * math.sin(a), z_mid)


def laced_rim_index(flange_i, trailing, side, rim_n):
    """3-cross: even flange holes trail, odd holes lead. side 0 = DS, 1 = NDS."""
    dest = flange_i + CROSS if trailing else flange_i - CROSS
    return (2 * dest + side) % rim_n


def create_spoke(part, head, through, nipple, name, max_deviation):
    """J-bend: circle swept along pin + bend arc + shaft, then a nail head."""
    inward = through.normalized()
    corner = head + inward * THROUGH_LEN
    out = (nipple - corner).normalized()
    pin_start = head - inward * 0.3

    # Fillet the pin/shaft corner in the J plane (same construction as a pipe elbow).
    n1 = inward * -1.0
    n2 = out
    theta = math.acos(max(-1.0, min(1.0, dot3(n1, n2))))
    half = max(0.25, 0.5 * theta)
    tan_half = math.tan(half)
    inset = BEND_R / tan_half
    inset = min(inset, THROUGH_LEN - 1.4)
    bend_r = inset * tan_half
    pin_tan = corner + n1 * inset
    shaft_tan = corner + n2 * inset
    center = corner + (n1 + n2).normalized() * (bend_r / math.sin(half))
    arc_mid = center + ((pin_tan - center) + (shaft_tan - center)).normalized() * bend_r

    x_axis = inward
    z_axis = inward.cross(out)
    if z_axis.norm() < 1e-9:
        hint = vec3(1.0, 0.0, 0.0) if abs(inward.x) < 0.9 else vec3(0.0, 1.0, 0.0)
        z_axis = inward.cross(hint)
    z_axis = z_axis.normalized()
    y_axis = z_axis.cross(x_axis).normalized()
    guide_frame = Frame(pin_start, x=x_axis, y=y_axis, z=z_axis)

    def to2(point):
        delta = point - pin_start
        return (dot3(delta, x_axis), dot3(delta, y_axis))

    guide = part.sketch(frame=guide_frame, name=name + "_path")
    guide.add_line(to2(pin_start), to2(pin_tan), name="pin")
    guide.add_arc(to2(pin_tan), to2(arc_mid), to2(shaft_tan), name="j")
    guide.add_line(to2(shaft_tan), to2(nipple), name="shaft")

    profile = part.sketch(frame=
        Frame(pin_start, x=y_axis, y=z_axis, z=x_axis), name=name + "_section")
    profile.add_circle((0.0, 0.0), SPOKE_R, name="wire")
    body = part.extrude_along_sketch(
        profile, guide, name=name + "_wire", max_deviation=max_deviation)

    head_pose, _ = frame_along(pin_start, pin_start + inward * 1.8)
    cap = part.cylinder(
        head_pose, SPOKE_HEAD_R, 1.8, name=name + "_head", max_deviation=max_deviation)
    return part.union(body, cap, name=name)


def create_spokes(part, layout, name="spoke", max_deviation=0.1):
    """3-cross: each flange pairs a trailing spoke with the next leading spoke."""
    z_mid = 0.5 * (layout["left_z"] + layout["right_z"])
    n = int(layout["holes"])
    rim_n = 2 * n
    half = math.pi / n
    ds_heads = flange_points(layout["right_outboard"] + 1.2, layout["pcd"], n, 0.0)
    nds_heads = flange_points(layout["left_outboard"] - 1.2, layout["pcd"], n, half)
    spokes = []
    for i, head in enumerate(ds_heads):
        rim = rim_nipple(laced_rim_index(i, i % 2 == 0, 0, rim_n), z_mid, rim_n)
        spokes.append(create_spoke(
            part, head, vec3(0.0, 0.0, -1.0), rim,
            "{0}_ds{1}".format(name, i), max_deviation))
    for i, head in enumerate(nds_heads):
        rim = rim_nipple(laced_rim_index(i, i % 2 == 0, 1, rim_n), z_mid, rim_n)
        spokes.append(create_spoke(
            part, head, vec3(0.0, 0.0, 1.0), rim,
            "{0}_nds{1}".format(name, i), max_deviation))
    return spokes


def add_wheel(asm, part, layout=None, max_deviation=0.1):
    """Rim, tire, 3-cross spokes, rotor, and bolts on an assembly that has the hub."""
    if layout is None:
        layout = hub_spoke_layout()
    asm.add_part(create_rim(part, layout, max_deviation=max_deviation))
    asm.add_part(create_tire(part, layout, max_deviation=max_deviation))
    asm.add_part(create_rotor(part, layout, max_deviation=max_deviation))
    for bolt in create_rotor_bolts(part, layout, max_deviation=max_deviation):
        asm.add_part(bolt)
    for spoke in create_spokes(part, layout, max_deviation=max_deviation):
        asm.add_part(spoke)
    return asm


def build_wheel(part=None, max_deviation=0.1):
    """Rim + tire + spokes + disc + the cassette hub (no cogs)."""
    from bike_cassette import create_hub

    if part is None:
        part = new_part(extent=360.0, tolerance=max_deviation)
    hub = create_hub(part, "hub", max_deviation=max_deviation)
    asm = part.assembly("wheel")
    ground = asm.add_part(hub)
    asm.fix(ground)
    add_wheel(asm, part, max_deviation=max_deviation)
    return asm


if __name__ == "__main__":
    wheel = build_wheel()
    print(wheel)
    wheel.show(title="camber 700c rear wheel")
