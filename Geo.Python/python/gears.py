"""Involute gear solids used by v_gear.py and planetary_gearbox.py.

Flanks are Hermite fits of the circle involute. An internal ring is a disc
minus an external involute hole (addendum/dedendum swapped). Bores are cut
after extrude so a twist-extrude cannot helix the hole.
"""
import math

from camber import Frame

TWO_PI = 2.0 * math.pi


def pitch_radius(module, teeth):
    return 0.5 * module * teeth


def tip_radius(module, teeth, addendum=1.0):
    return pitch_radius(module, teeth) + addendum * module


def root_radius(module, teeth, dedendum=1.25):
    return max(0.15 * module, pitch_radius(module, teeth) - dedendum * module)


def internal_tip_radius(module, teeth, addendum=1.0):
    return pitch_radius(module, teeth) - addendum * module


def internal_root_radius(module, teeth, dedendum=1.25):
    return pitch_radius(module, teeth) + dedendum * module


def quat_z(radians):
    """Quaternion (x, y, z, w) for a rotation about +Z."""
    h = 0.5 * radians
    return (0.0, 0.0, math.sin(h), math.cos(h))


def yaw_frame(z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return Frame((0.0, 0.0, z), x=(c, s, 0.0), y=(-s, c, 0.0), z=(0.0, 0.0, 1.0))


def _cut_bore(part, body, radius, height, name):
    cutter = part.cylinder((0.0, 0.0, -0.5), radius, height, name=name)
    return body - cutter


def external_spur(
        part, name, module, teeth, face, bore=0.0, pressure=None,
        addendum=1.0, dedendum=1.25, max_deviation=-1):
    """Origin-centered external spur, midplane at z = 0."""
    if pressure is None:
        pressure = math.radians(20.0)
    sk = part.sketch(name=name + "_sk")
    sk.add_involute_gear(
        (0.0, 0.0), module, teeth, pressure_angle=pressure,
        addendum=addendum, dedendum=dedendum, max_deviation=max_deviation)
    body = part.extrude(sk, 0.5 * face, both_sides=True, name=name)
    if bore > 0.0:
        body = _cut_bore(part, body, bore, face + 2.0, name + "_bore")
    return body


def internal_ring(
        part, name, module, teeth, face, outer_radius, pressure=None,
        addendum=1.0, dedendum=1.25, max_deviation=-1):
    """Internal ring: disc minus the involute hole. Midplane at z = 0."""
    if pressure is None:
        pressure = math.radians(20.0)
    sk = part.sketch(name=name + "_hole")
    sk.add_involute_internal_gear(
        (0.0, 0.0), module, teeth, pressure_angle=pressure,
        addendum=addendum, dedendum=dedendum, max_deviation=max_deviation)
    extra = 0.5
    cutter = part.extrude(sk, 0.5 * face + extra, both_sides=True, name=name + "_cut")
    blank = part.cylinder((0.0, 0.0, -0.5 * face), outer_radius, face, name=name + "_blank")
    return blank - cutter


def herringbone(
        part, name, module, teeth, half_width, helix, bore=0.0, pressure=None,
        max_deviation=-1):
    """Citroën chevron: two opposite twist-extrudes, then a straight bore."""
    if pressure is None:
        pressure = math.radians(20.0)
    rp = pitch_radius(module, teeth)
    twist = math.tan(helix) / rp

    def involute_sketch(sk_name, frame):
        sk = part.sketch(frame=frame, name=sk_name)
        sk.add_involute_gear((0.0, 0.0), module, teeth, pressure_angle=pressure,
                             max_deviation=max_deviation)
        return sk

    lower = part.extrude(
        involute_sketch(name + "_lo_sk", Frame()), half_width, name=name + "_lo", twist=twist)
    upper = part.extrude(
        involute_sketch(name + "_hi_sk", yaw_frame(half_width, twist * half_width)),
        half_width, name=name + "_hi", twist=-twist)
    body = lower + upper
    if bore > 0.0:
        body = body - part.cylinder(
            (0.0, 0.0, -1.0), bore, 2.0 * half_width + 2.0, name=name + "_bore")
    return body


def grooved_shaft(part, name, radius, z0, z1, grooves=()):
    """Cylinder on +Z from z0 to z1. Each groove is (z, width, depth)."""
    h = z1 - z0
    body = part.cylinder((0.0, 0.0, z0), radius, h, name=name)
    for i, (gz, gw, gd) in enumerate(grooves):
        tube = part.cylinder((0.0, 0.0, gz), radius + 0.2, gw, name=name + "_g{0}a".format(i))
        core = part.cylinder((0.0, 0.0, gz - 0.05), radius - gd, gw + 0.1, name=name + "_g{0}b".format(i))
        body = body - (tube - core)
    return body


def deep_groove_bearing(part, name, inner_d, outer_d, width):
    """ISO-sized races only (no balls). Midplane at z = 0."""
    ir = 0.5 * inner_d
    or_ = 0.5 * outer_d
    wall = max(1.2, 0.12 * (or_ - ir))
    inner = part.cylinder((0.0, 0.0, -0.5 * width), ir + wall, width, name=name + "_ir")
    inner = inner - part.cylinder((0.0, 0.0, -0.5 * width - 0.2), ir, width + 0.4, name=name + "_irb")
    outer = part.cylinder((0.0, 0.0, -0.5 * width), or_, width, name=name + "_or")
    outer = outer - part.cylinder(
        (0.0, 0.0, -0.5 * width - 0.2), or_ - wall, width + 0.4, name=name + "_orb")
    return inner + outer


def hex_cap_screw(part, name, thread_d, length, head_af, head_h):
    """Unthreaded hex-head screw. Origin at the bearing face; head +Z, shank −Z."""
    r_head = head_af / math.sqrt(3.0)
    sk = part.sketch(name=name + "_hex")
    pts = []
    for i in range(6):
        a = math.radians(30.0) + i * TWO_PI / 6.0
        pts.append((r_head * math.cos(a), r_head * math.sin(a)))
    for i in range(6):
        sk.add_line(pts[i], pts[(i + 1) % 6])
    head = part.extrude(sk, head_h, name=name + "_head")
    shank = part.cylinder((0.0, 0.0, -length), 0.5 * thread_d, length, name=name + "_shank")
    return head + shank


def add_bolt_circle(sk, center, radius, hole_r, count, start_angle=0.0, name="bcd"):
    cx, cy = center
    for i in range(count):
        a = start_angle + i * TWO_PI / count
        sk.add_circle((cx + radius * math.cos(a), cy + radius * math.sin(a)), hole_r, name=name + str(i))
    return sk


def plate_with_holes(part, name, outer_r, thickness, holes, z0=0.0):
    """Disc from z0 along +Z. holes is a list of (x, y, r)."""
    body = part.cylinder((0.0, 0.0, z0), outer_r, thickness, name=name)
    for i, (x, y, r) in enumerate(holes):
        body = body - part.cylinder(
            (x, y, z0 - 0.2), r, thickness + 0.4, name=name + "_h{0}".format(i))
    return body
