"""Compact, readable solid builders for the PTO visual showcase."""
import math

from camber import Frame
from assumptions import SPLINE_MAJOR_DIAMETER, SPLINE_ROOT_DIAMETER, SPLINE_TEETH


def rounded_triangle_sketch(part, name, across_corners, corner_radius, z=0.0):
    """A three-lobe profile made from tangent lines and true circular arcs."""
    circumradius = across_corners / 2
    center_radius = circumradius - corner_radius
    centers = [
        (center_radius * math.cos(math.pi / 2 + i * 2 * math.pi / 3),
         center_radius * math.sin(math.pi / 2 + i * 2 * math.pi / 3))
        for i in range(3)
    ]
    incoming, outgoing = [], []
    for i, center in enumerate(centers):
        previous, following = centers[(i - 1) % 3], centers[(i + 1) % 3]

        def outward(a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            return dy / length, -dx / length

        n_in = outward(previous, center)
        n_out = outward(center, following)
        incoming.append((center[0] + corner_radius * n_in[0],
                         center[1] + corner_radius * n_in[1]))
        outgoing.append((center[0] + corner_radius * n_out[0],
                         center[1] + corner_radius * n_out[1]))

    sketch = part.sketch(frame=Frame((0, 0, z)), name=name)
    for i, center in enumerate(centers):
        start, end = incoming[i], outgoing[i]
        a0 = math.atan2(start[1] - center[1], start[0] - center[0])
        a1 = math.atan2(end[1] - center[1], end[0] - center[0])
        while a1 <= a0:
            a1 += 2 * math.pi
        mid_angle = (a0 + a1) / 2
        middle = (center[0] + corner_radius * math.cos(mid_angle),
                  center[1] + corner_radius * math.sin(mid_angle))
        sketch.add_arc(start, middle, end, name="corner_%d" % i)
        next_start = incoming[(i + 1) % 3]
        sketch.add_line(end, next_start, name="side_%d" % i)
    return sketch


def build_tube(part, name, length, across_corners, wall):
    outer_r = across_corners / 2
    corner = min(5.0, outer_r * .17)
    outer = rounded_triangle_sketch(part, name + "_outer", across_corners, corner)
    body = outer.extrude(length, name=name + "_outer_skin")
    inner_size = across_corners - 2 * wall
    inner = rounded_triangle_sketch(
        part, name + "_bore", inner_size, corner - wall, z=-.2)
    cutter = inner.extrude(length + .4, name=name + "_bore_tool")
    body = body - cutter
    # A welded end plug joins the tube wall to its cylindrical yoke spigot.
    plug = part.cylinder((0, 0, 0), outer_r, 4.0, name=name + "_plug")
    pilot = part.cylinder((0, 0, -12.0), 13.5, 13.0,
                          name=name + "_pilot")
    return part.batch_union([body, plug, pilot])


def _rounded_yoke_arm(part, name, x0, hub_side):
    frame = Frame((x0, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0))
    sketch = part.sketch(frame=frame, name=name + "_outline")
    sign = 1 if hub_side < 0 else -1
    def p(y, z):
        return (y, sign * z)
    sketch.add_line(p(-17, -37), p(17, -37), name="root")
    sketch.add_arc(p(17, -37), p(20, -30), p(18, -20), name="right_shoulder")
    sketch.add_line(p(18, -20), p(14, 0), name="right_cheek")
    sketch.add_arc(p(14, 0), p(0, 14), p(-14, 0), name="bearing_crown")
    sketch.add_line(p(-14, 0), p(-18, -20), name="left_cheek")
    sketch.add_arc(p(-18, -20), p(-20, -30), p(-17, -37), name="left_shoulder")
    return sketch.extrude(14.0, name=name)


def build_yoke(part, name, hub_side, connector):
    """Forked yoke around the origin; its shaft hub points to hub_side Z."""
    hub_start = -75.0 if hub_side < 0 else 30.0
    hub_length = 45.0 if connector == "female" else 13.9
    if connector == "tube" and hub_side < 0:
        hub_start = -43.9
    hub = part.cylinder((0, 0, hub_start), 24.0, hub_length,
                        name=name + "_hub")
    ears = [
        _rounded_yoke_arm(part, name + "_left_fork", -29.0, hub_side),
        _rounded_yoke_arm(part, name + "_right_fork", 15.0, hub_side),
        part.cylinder((-34, 0, 0), 13.0, 20.0, name=name + "_left_boss", axis="x"),
        part.cylinder((14, 0, 0), 13.0, 20.0, name=name + "_right_boss", axis="x"),
    ]
    yoke = part.batch_union([hub] + ears, name=name + "_forging")

    left_hole = part.cylinder((-35, 0, 0), 12.1, 21.0,
                              name=name + "_left_bearing_seat", axis="x")
    right_hole = part.cylinder((14, 0, 0), 12.1, 21.0,
                               name=name + "_right_bearing_seat", axis="x")
    yoke = part.batch_subtract(yoke, [left_hole, right_hole], name=name)

    if connector == "female":
        start = hub_start - 1.0 if hub_side < 0 else hub_start + 14.0
        bore = part.cylinder((0, 0, start), SPLINE_ROOT_DIAMETER / 2, 32.0,
                             name=name + "_spline_root")
        slots = []
        for i in range(SPLINE_TEETH):
            angle = i * 2 * math.pi / SPLINE_TEETH
            radial = (math.cos(angle), math.sin(angle), 0)
            tangent = (-math.sin(angle), math.cos(angle), 0)
            frame = Frame((14.0 * radial[0] - 3.4 * tangent[0],
                           14.0 * radial[1] - 3.4 * tangent[1], start),
                          x=radial, y=tangent, z=(0, 0, 1))
            slot = part.cuboid(frame, (SPLINE_MAJOR_DIAMETER / 2 - 14.0, 6.8, 32.0),
                               name=name + "_spline_space_%d" % i)
            slots.append(slot)
        yoke = part.batch_subtract(yoke, [bore] + slots, name=name)
        # Transverse spring-button bore lies tangent to the spline socket.
        button_z = hub_start + (14 if hub_side < 0 else 31)
        button_boss = part.cylinder((-26, 17, button_z), 7, 52,
                                    name=name + "_button_boss", axis="x")
        button_hole = part.cylinder((-27, 17, button_z), 4, 54,
                                    name=name + "_button_hole", axis="x")
        yoke = (yoke + button_boss) - button_hole
    elif connector == "tube":
        seat_start = hub_start - .5
        seat = part.cylinder((0, 0, seat_start), 14.2, 31.0,
                             name=name + "_tube_seat")
        yoke = yoke - seat
    else:
        raise ValueError("connector must be 'female' or 'tube'")
    return yoke


def build_cross(part, name):
    center = part.sphere((0, 0, 0), 11.5, name=name + "_center")
    journals = [
        part.cylinder((-30, 0, 0), 7.7, 60, name=name + "_journal_x", axis="x"),
        part.cylinder((0, -30, 0), 7.7, 60, name=name + "_journal_y", axis="y"),
    ]
    cross = part.batch_union([center] + journals, name=name)
    seat = part.cylinder((0, 0, 5), 3.1, 8, name=name + "_grease_thread")
    spotface = part.cuboid((-5, -5, 9), (5, 5, 15), name=name + "_grease_spotface")
    return part.batch_subtract(cross, [seat, spotface])


def build_bearing_cup(part, name):
    outer = part.cylinder((0, 0, 0), 12.0, 21.1, name=name + "_outer")
    flange = part.cylinder((0, 0, 20.1), 13.2, 1.0, name=name + "_lip")
    cup = outer + flange
    bore = part.cylinder((0, 0, -.2), 9.9, 18.2, name=name + "_bore")
    return cup - bore


def build_guard_tube(part, name, length, radius, wall=2.0, cutaway=False):
    outer = part.cylinder((0, 0, 0), radius, length, name=name + "_outer")
    inner = part.cylinder((0, 0, -.2), radius - wall, length + .4,
                          name=name + "_inner")
    guard = outer - inner
    if cutaway:
        opening = part.cuboid((-radius - 1, -radius - 1, -.2),
                              (radius + 1, -radius * .12, length + .2),
                              name=name + "_view_window")
        guard = guard - opening
    return guard


def build_guard_bell(part, name):
    """Stepped shield bell with open ends, revolved from a clear section sketch."""
    sk = part.sketch(frame=Frame((0, 0, 0), x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0)),
                    name=name + "_section")
    # Section coordinates are (axial Z, radius), not (radius, Z).
    # The open mouth covers the joint; the neck points toward the shaft centre.
    points = [(-50, 59), (-47, 60), (5, 60), (9, 58), (12, 54),
              (43, 35), (48, 33), (74, 33), (76, 35), (80, 35),
              (80, 30), (47, 30), (41, 32), (9, 51), (5, 57),
              (-47, 57), (-50, 57)]
    for i, point in enumerate(points):
        sk.add_line(point, points[(i + 1) % len(points)], name="wall_%d" % i)
    shell = sk.revolve(2 * math.pi, name=name)
    ribs = []
    for i in range(18):
        a = i * 2 * math.pi / 18
        radial = (math.cos(a), math.sin(a), 0)
        tangent = (-math.sin(a), math.cos(a), 0)
        frame = Frame(tuple(-1.1 * v for v in tangent),
                      x=(0, 0, 1), y=radial, z=tangent)
        rib = part.sketch(frame=frame, name=name + "_rib_%02d" % i)
        outline = [(11, 51), (11, 57), (42, 39), (46, 33), (40, 32)]
        for j, p in enumerate(outline):
            rib.add_line(p, outline[(j + 1) % len(outline)])
        ribs.append(rib.extrude(2.2, name=name + "_rib_solid_%02d" % i))
    eye = part.cylinder((38, 0, 66), 6, 2, name=name + "_chain_eye", axis="x")
    root = part.cuboid((31, -3, 58), (39, 3, 62), name=name + "_eye_root")
    hole = part.cylinder((30, 0, 66), 3.4, 11, name=name + "_chain_eye_hole", axis="x")
    bell = part.batch_union([shell] + ribs)
    lug = (eye + root) - hole
    return bell + lug


def build_grease_nipple(part, name):
    base = part.cylinder((0, 0, 0), 3.0, 4.0, name=name + "_base")
    collar = part.cylinder((0, 0, 3.1), 4.0, 1.9, name=name + "_collar")
    stem = part.cylinder((0, 0, 4.0), 2.5, 3.0, name=name + "_stem")
    head = part.sphere((0, 0, 7.0), 3.3, name=name + "_head")
    return part.batch_union([base, collar, stem, head], name=name)


def build_chain_link(part, name):
    # Round-wire obround, 24 x 10 overall, not an extruded flat washer.
    guide = part.sketch(name=name + "_centreline")
    guide.add_line((-7, -4), (7, -4))
    guide.add_arc((7, -4), (11, 0), (7, 4))
    guide.add_line((7, 4), (-7, 4))
    guide.add_arc((-7, 4), (-11, 0), (-7, -4))
    section = part.sketch(name=name + "_wire",
                          frame=Frame((-7, -4, 0), x=(0, 1, 0),
                                      y=(0, 0, 1), z=(1, 0, 0)))
    section.add_circle((0, 0), 1.0)
    return section.extrude_along_sketch(guide, name=name)


def build_guard_label(part, name, length, outer_radius):
    """Thin full-wrap safety label, kept as a distinct replaceable component."""
    return build_guard_tube(part, name, length, outer_radius, .2)
