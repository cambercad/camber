"""9-speed road cassette 12-27 (mm) on a rear hub — Shimano 105-style stack.

How a human models this:
  1. Sketch one ISO 606 tooth (roller seat, flank radius, ANSI topping) and walk it N times.
  2. Extrude the cog and cut the HG spline (or a clearance bore for the spider cluster).
  3. Build spacers, the 21-24-27 spider, lockring, and hub as separate solids.
  4. Slide everything onto the freehub along +Z (wide spline already on +X).

From Geo.Python, with the venv active:
    python python\\bike_cassette.py
Cassette only. For hub + rim + tire + spokes:
    python python\\bike_cassette_assembly.py
"""
import math

from camber import Frame, Part, vec2, vec3

# --- 9-speed HG road numbers (Sheldon / Shimano) ---
CHAIN_PITCH = 12.7
ROLLER_DIA = 7.77
COG_THICKNESS = 1.78
COG_PITCH = 4.34
SPACER_THICK = 2.56
STACK_WIDTH = 36.5

CLUSTER = (27, 24, 21)
INDIVIDUALS = (19, 17, 15, 14, 13, 12)

HG_MAJOR = 34.95
HG_MINOR = 30.0
HG_NARROW = math.radians(16.0)
HG_WIDE = math.radians(32.0)
HG_CLEARANCE = 0.10

SPACER_OD = 39.0
SPIDER_BOSS_OD = 42.0
# Stay inside the 21T root so the carrier never sits in a tooth gap.
SPIDER_ARM_R = 35.0
SPIDER_ARM_HALF = 6.5
SPIDER_ARM_THICK = 2.0
CLUSTER_H = 2.0 * COG_PITCH + COG_THICKNESS

LOCK_OD = 38.5
LOCK_ID = 33.2
LOCK_H = 6.5
LOCK_NOTCHES = 12
LOCK_NOTCH_W = 3.4
LOCK_NOTCH_D = 2.2
LOCK_JOURNAL_L = 8.0
LOCK_JOURNAL_R = 16.4

FLANGE_OD = 62.0
FLANGE_T = 2.8
FLANGE_SPAN = 50.0
SHELL_OD = 28.0
SHOULDER_OD = 42.0
SHOULDER_T = 2.5
AXLE_R = 5.1
SPOKE_N = 16
SPOKE_PCD = 52.0
SPOKE_R = 1.3

# ISO 6-bolt disc mount on the left (non-drive) flange.
DISC_MOUNT_H = 5.0
DISC_MOUNT_OD = 54.0
DISC_BOLT_BCD = 44.0
DISC_BOLT_N = 6
DISC_BOLT_HOLE_R = 2.6

TWO_PI = 2.0 * math.pi


def polar(radius, angle):
    return vec2(radius * math.cos(angle), radius * math.sin(angle))


def unit(v):
    v = vec2(v)
    n = v.norm()
    return v * (1.0 / n) if n > 1e-12 else vec2(1.0, 0.0)


def angle_mid(a0, a1):
    delta = (a1 - a0) % TWO_PI
    return a0 + 0.5 * delta


def xy(z):
    return Frame(vec3(0.0, 0.0, z))


def cross2(a, b):
    a, b = vec2(a), vec2(b)
    return a.x * b.y - a.y * b.x


def circle_hits(c1, r1, c2, r2):
    """Intersection points of two circles. Empty if they miss."""
    c1, c2 = vec2(c1), vec2(c2)
    delta = c2 - c1
    dist = delta.norm()
    if dist < 1e-12 or dist > r1 + r2 + 1e-8 or dist < abs(r1 - r2) - 1e-8:
        return []
    along = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist)
    height2 = r1 * r1 - along * along
    if height2 < -1e-8:
        return []
    height = math.sqrt(max(0.0, height2))
    mid = c1 + delta * (along / dist)
    if height < 1e-10:
        return [mid]
    perp = vec2(-delta.y, delta.x) * (height / dist)
    return [mid + perp, mid - perp]


def minor_arc_mid(center, radius, p0, p1):
    """Midpoint of the shorter arc from p0 to p1 on the circle."""
    center = vec2(center)
    a0 = math.atan2(p0.y - center.y, p0.x - center.x)
    a1 = math.atan2(p1.y - center.y, p1.x - center.x)
    span = (a1 - a0) % TWO_PI
    if span > math.pi:
        span -= TWO_PI
    mid = a0 + 0.5 * span
    return vec2(center.x + radius * math.cos(mid), center.y + radius * math.sin(mid))


def pick_outer_hit(hits, tooth_ray, want_ccw):
    """The outer intersection on the requested side of the tooth centreline."""
    scored = []
    for point in hits:
        on_ccw = cross2(tooth_ray, point) > 0.0
        if on_ccw == want_ccw:
            scored.append(point)
    pool = scored if scored else list(hits)
    return max(pool, key=lambda point: point.norm())


def sprocket_dims(teeth):
    """ISO 606 pitch, roller seat, flank radius, and a production topping radius."""
    n = int(teeth)
    d1 = ROLLER_DIA
    pitch = CHAIN_PITCH
    pitch_r = pitch / (2.0 * math.sin(math.pi / n))
    # ISO 606 5.4 — roller seat, mid-range of the allowed band.
    seat_r = 0.505 * d1 + 0.35 * 0.069 * (d1 ** (1.0 / 3.0))
    # ISO 606 seating angle, toward the fatter (minimum-gap) side.
    alpha = math.radians(136.0 - 90.0 / n)
    # ISO 606 flank radius re, between 0.12 d1 (z+2) and 0.008 d1 (z^2+180).
    re_lo = 0.12 * d1 * (n + 2)
    re_hi = 0.008 * d1 * (n * n + 180)
    flank_r = 0.55 * re_lo + 0.45 * re_hi
    # ISO 606 5.5 — tip envelope near da_max. The topping then blunts the crest.
    addendum = 0.625 * pitch - 0.5 * d1 + 0.8 / n
    tip_r = pitch_r + addendum
    # ANSI B29.1 topping F is ~6.5 mm for the full ANSI working curve. On an
    # ISO flank that already reaches da, 3 mm is enough to cut the sharp point
    # the way stamped cassette cogs do (no Hyperglide ramps).
    tip_fillet = 3.0
    root_r = pitch_r - seat_r
    return n, pitch_r, seat_r, tip_r, root_r, tip_fillet, alpha, flank_r


def tooth_outline(n, i, pitch_r, seat_r, tip_r, tip_fillet, alpha, flank_r):
    """ISO 606 seat + flank radius, then an ANSI-style topping on the crest."""
    valley = TWO_PI * i / n
    nxt = TWO_PI * ((i + 1) % n) / n
    tooth = valley + math.pi / n
    tooth_ray = polar(1.0, tooth)

    seat_c = polar(pitch_r, valley)
    next_c = polar(pitch_r, nxt)
    enter = seat_c + polar(seat_r, valley + math.pi + 0.5 * alpha)
    leave = seat_c + polar(seat_r, valley + math.pi - 0.5 * alpha)
    bottom = polar(pitch_r - seat_r, valley)
    next_enter = next_c + polar(seat_r, nxt + math.pi + 0.5 * alpha)

    cf_a = leave - unit(leave - seat_c) * flank_r
    cf_b = next_enter - unit(next_enter - next_c) * flank_r

    fillet = tip_fillet
    tip_a = tip_b = tip_c = None
    for trial in (fillet, 2.6, 3.4, 2.3, 3.8, 2.2):
        tip_c = polar(tip_r - trial, tooth)
        hits_a = circle_hits(cf_a, flank_r, tip_c, trial)
        hits_b = circle_hits(cf_b, flank_r, tip_c, trial)
        if not hits_a or not hits_b:
            continue
        tip_a = pick_outer_hit(hits_a, tooth_ray, False)
        tip_b = pick_outer_hit(hits_b, tooth_ray, True)
        fillet = trial
        break
    if tip_a is None:
        tip_c = polar(tip_r - 0.6, tooth)
        tip_a = polar(tip_r, tooth - 0.22 * math.pi / n)
        tip_b = polar(tip_r, tooth + 0.22 * math.pi / n)
        fillet = 0.6

    tip_mid = polar(tip_r, tooth)
    flank_a_mid = minor_arc_mid(cf_a, flank_r, leave, tip_a)
    flank_b_mid = minor_arc_mid(cf_b, flank_r, tip_b, next_enter)
    return (enter, bottom, leave, flank_a_mid, tip_a, tip_mid, tip_b,
            flank_b_mid, next_enter, fillet)


def sketch_teeth(sk, teeth):
    """ISO 606 tooth: seat, flank radius, topped tip. First tooth constrained."""
    n, pitch_r, seat_r, tip_r, root_r, tip_fillet, alpha, flank_r = sprocket_dims(teeth)
    sk.solve_after_every_constraint = False

    pitch = sk.add_circle((0.0, 0.0), pitch_r, name="pitch", construction=True)
    sk.coincident(pitch @ "center", sk @ "origin")
    sk.radius(pitch, pitch_r)

    for i in range(n):
        (enter, bottom, leave, flank_a_mid, tip_a, tip_mid, tip_b,
         flank_b_mid, next_enter, fillet) = tooth_outline(
            n, i, pitch_r, seat_r, tip_r, tip_fillet, alpha, flank_r)
        if i == 0:
            seat = sk.add_arc(enter, bottom, leave, name="seat0")
            flank_a = sk.add_arc(leave, flank_a_mid, tip_a, name="flank_a0")
            tip = sk.add_arc(tip_a, tip_mid, tip_b, name="tip0")
            flank_b = sk.add_arc(tip_b, flank_b_mid, next_enter, name="flank_b0")
            sk.coincident(seat @ 1.000, flank_a @ 0.000)
            sk.coincident(flank_a @ 1.000, tip @ 0.000)
            sk.coincident(tip @ 1.000, flank_b @ 0.000)
            sk.radius(seat, seat_r)
            sk.radius(flank_a, flank_r)
            sk.radius(tip, fillet)
            sk.radius(flank_b, flank_r)
        else:
            sk.add_arc(enter, bottom, leave, name="seat{0}".format(i))
            sk.add_arc(leave, flank_a_mid, tip_a, name="flank_a{0}".format(i))
            sk.add_arc(tip_a, tip_mid, tip_b, name="tip{0}".format(i))
            sk.add_arc(tip_b, flank_b_mid, next_enter, name="flank_b{0}".format(i))
    return root_r, tip_r


def hg_lobe_angles():
    """9 lobes, wide one on +X (clocks the cassette)."""
    gap = (TWO_PI - 8.0 * HG_NARROW - HG_WIDE) / 9.0
    spans = []
    a = -0.5 * HG_WIDE
    spans.append((a, a + HG_WIDE))
    a += HG_WIDE
    for _ in range(8):
        a += gap
        spans.append((a, a + HG_NARROW))
        a += HG_NARROW
    return spans


def sketch_hg_spline(sk, hole=False, clearance=0.0, name="hg"):
    r_maj = 0.5 * HG_MAJOR + clearance
    r_min = 0.5 * HG_MINOR + clearance
    spans = hg_lobe_angles()
    segs = []
    for i, (a0, a1) in enumerate(spans):
        a_next = spans[(i + 1) % len(spans)][0]
        segs.append(("line", polar(r_min, a0), polar(r_maj, a0), None))
        segs.append(("arc", polar(r_maj, a0), polar(r_maj, angle_mid(a0, a1)), polar(r_maj, a1)))
        segs.append(("line", polar(r_maj, a1), polar(r_min, a1), None))
        segs.append(("arc", polar(r_min, a1), polar(r_min, angle_mid(a1, a_next)), polar(r_min, a_next)))
    if hole:
        segs = [("line", b, a, None) if kind == "line" else ("arc", c, b, a)
                for kind, a, b, c in reversed(segs)]

    first = True
    for kind, a, b, c in segs:
        if kind == "line":
            if first:
                sk.add_line(a, b, name=name)
                first = False
            else:
                sk.append_line(b)
        elif first:
            sk.add_arc(a, b, c, name=name)
            first = False
        else:
            sk.add_arc(a, b, c)


def cut_lightening(part, body, name, teeth, root_r, inner_r, thickness, z0, max_deviation):
    web = root_r - inner_r
    if web < 8.0:
        return body
    hole_r = min(3.6, 0.26 * web)
    pcd = inner_r + 0.52 * web
    count = 5 if teeth >= 21 else 4
    sk = part.sketch(frame=xy(z0 - 0.2), name=name + "_light")
    seed = sk.add_circle((pcd, 0.0), hole_r, name="light0")
    sk.repeat_circular([seed], (0.0, 0.0), count)
    cutter = part.extrude(sk, thickness + 0.4, name=name + "_light_cut", max_deviation=max_deviation)
    return part.cut(body, cutter, name=name)


def create_sprocket(part, teeth, name, max_deviation=0.1,
                   bore="spline", built_in_spacer=False, lightening=True):
    """One steel cog. bore='spline' for HG, 'clearance' for the spider cluster."""
    spacer_h = SPACER_THICK if built_in_spacer else 0.0
    z0 = spacer_h

    sk = part.sketch(frame=xy(z0), constrained=True, name=name + "_teeth")
    root_r, tip_r = sketch_teeth(sk, teeth)
    if bore == "spline":
        sketch_hg_spline(sk, hole=True, clearance=HG_CLEARANCE, name=name + "_hg")
        inner_r = 0.5 * HG_MAJOR + 3.0
    else:
        sk.add_circle((0.0, 0.0), 0.5 * SPIDER_BOSS_OD + 0.4, name="bore")
        inner_r = 0.5 * SPIDER_BOSS_OD + 1.5

    blank = name + "_blank" if (lightening or built_in_spacer) else name
    body = part.extrude(sk, COG_THICKNESS, name=blank, max_deviation=max_deviation)
    if lightening:
        body = cut_lightening(
            part, body, name, teeth, root_r, inner_r, COG_THICKNESS, z0, max_deviation)

    if not built_in_spacer:
        return body

    ring = part.sketch(frame=xy(0.0), name=name + "_collar")
    ring.add_circle((0.0, 0.0), 0.5 * SPACER_OD, name="od")
    sketch_hg_spline(ring, hole=True, clearance=HG_CLEARANCE, name=name + "_collar_hg")
    collar = part.extrude(ring, spacer_h, name=name + "_collar_body", max_deviation=max_deviation)
    return part.union(body, collar, name=name)


def create_spacer(part, name, max_deviation=0.1):
    sk = part.sketch("xy", name=name + "_profile")
    sk.add_circle((0.0, 0.0), 0.5 * SPACER_OD, name="od")
    sk.add_circle((0.0, 0.0), 0.5 * HG_MAJOR + 0.25, name="bore")
    return part.extrude(sk, SPACER_THICK, name=name, max_deviation=max_deviation)


def sketch_spider_arms(sk, boss_r, tip_r, half_w):
    for i in range(4):
        axis = 0.5 * math.pi * i
        side = vec2(-math.sin(axis), math.cos(axis))
        half = math.asin(min(0.95, half_w / boss_r))
        a0, a1 = axis - half, axis + half
        next_a0 = 0.5 * math.pi * ((i + 1) % 4) - half
        tip_c = polar(tip_r - half_w, axis)
        p0 = polar(boss_r, a0)
        p1 = tip_c - side * half_w
        p2 = polar(tip_r, axis)
        p3 = tip_c + side * half_w
        p4 = polar(boss_r, a1)
        p5 = polar(boss_r, next_a0)
        p_mid = polar(boss_r, angle_mid(a1, next_a0))
        if i == 0:
            sk.add_line(p0, p1, name="arm0")
        else:
            sk.append_line(p1)
        sk.add_arc(p1, p2, p3)
        sk.append_line(p4)
        sk.add_arc(p4, p_mid, p5)


def create_spider(part, name, max_deviation=0.1):
    """Aluminium carrier: spline boss + two 4-arm plates in the cog gaps."""
    boss_sk = part.sketch("xy", name=name + "_boss")
    boss_sk.add_circle((0.0, 0.0), 0.5 * SPIDER_BOSS_OD, name="od")
    sketch_hg_spline(boss_sk, hole=True, clearance=HG_CLEARANCE, name=name + "_hg")
    boss = part.extrude(boss_sk, CLUSTER_H, name=name + "_boss_body", max_deviation=max_deviation)

    def arms(suffix, z):
        sk = part.sketch(frame=xy(z), name=name + "_" + suffix)
        sketch_spider_arms(sk, 0.5 * SPIDER_BOSS_OD - 0.2, SPIDER_ARM_R, SPIDER_ARM_HALF)
        sk.add_circle((0.0, 0.0), 0.5 * SPIDER_BOSS_OD - 0.3, name="clear")
        return part.extrude(sk, SPIDER_ARM_THICK, name=name + "_" + suffix, max_deviation=max_deviation)

    plates = part.union(
        arms("arms0", COG_THICKNESS + 0.15),
        arms("arms1", COG_PITCH + COG_THICKNESS + 0.15),
        name=name + "_arms")
    return part.union(boss, plates, name=name)


def create_lockring(part, name, max_deviation=0.1):
    """12-notch TL-LR15 style lockring (smooth bore — not an ISO thread)."""
    sk = part.sketch("xy", name=name + "_profile")
    od = 0.5 * LOCK_OD
    half = 0.5 * LOCK_NOTCH_W / od
    step = TWO_PI / LOCK_NOTCHES
    for i in range(LOCK_NOTCHES):
        notch = (i + 1) * step
        a0 = i * step + half
        a1 = notch - half
        if i == 0:
            sk.add_arc(polar(od, a0), polar(od, angle_mid(a0, a1)), polar(od, a1), name="od0")
        else:
            sk.add_arc(polar(od, a0), polar(od, angle_mid(a0, a1)), polar(od, a1))
        sk.append_line(polar(od - LOCK_NOTCH_D, a1))
        sk.append_line(polar(od - LOCK_NOTCH_D, notch + half))
        sk.append_line(polar(od, notch + half))
    sk.add_circle((0.0, 0.0), 0.5 * LOCK_ID, name="bore")
    return part.extrude(sk, LOCK_H, name=name, max_deviation=max_deviation)


def create_hub(part, name, max_deviation=0.1):
    """Rear hub: HG freehub on +Z, cassette shoulder at z=0, flanges to -Z."""
    right_flange_z = -(SHOULDER_T + FLANGE_T)
    left_flange_z = right_flange_z - FLANGE_SPAN - FLANGE_T
    shell_z = left_flange_z + FLANGE_T
    shell_h = -right_flange_z - shell_z

    def flange(suffix, z, hole_angle=0.0):
        disc_sk = part.sketch(frame=xy(z), name=name + "_" + suffix)
        disc_sk.add_circle((0.0, 0.0), 0.5 * FLANGE_OD, name="od")
        disc_sk.add_circle((0.0, 0.0), AXLE_R, name="bore")
        disc = part.extrude(disc_sk, FLANGE_T, name=name + "_" + suffix + "_disc", max_deviation=max_deviation)
        holes = part.sketch(frame=xy(z - 0.2), name=name + "_" + suffix + "_spokes")
        seed = holes.add_circle((
            0.5 * SPOKE_PCD * math.cos(hole_angle),
            0.5 * SPOKE_PCD * math.sin(hole_angle),
        ), SPOKE_R, name="spoke0")
        holes.repeat_circular([seed], (0.0, 0.0), SPOKE_N)
        cutter = part.extrude(holes, FLANGE_T + 0.4, name=name + "_" + suffix + "_cut", max_deviation=max_deviation)
        return part.cut(disc, cutter, name=name + "_" + suffix)

    freehub_sk = part.sketch("xy", name=name + "_freehub")
    sketch_hg_spline(freehub_sk, hole=False, name=name + "_hg")
    freehub_sk.add_circle((0.0, 0.0), AXLE_R, name="bore")
    freehub = part.extrude(freehub_sk, STACK_WIDTH, name=name + "_freehub", max_deviation=max_deviation)

    journal_sk = part.sketch(frame=xy(STACK_WIDTH), name=name + "_journal")
    journal_sk.add_circle((0.0, 0.0), LOCK_JOURNAL_R, name="od")
    journal_sk.add_circle((0.0, 0.0), AXLE_R, name="bore")
    journal = part.extrude(journal_sk, LOCK_JOURNAL_L, name=name + "_journal", max_deviation=max_deviation)

    shoulder_sk = part.sketch(frame=xy(-SHOULDER_T), name=name + "_shoulder")
    shoulder_sk.add_circle((0.0, 0.0), 0.5 * SHOULDER_OD, name="od")
    shoulder_sk.add_circle((0.0, 0.0), AXLE_R, name="bore")
    shoulder = part.extrude(shoulder_sk, SHOULDER_T, name=name + "_shoulder", max_deviation=max_deviation)

    shell = part.cylinder(xy(shell_z), 0.5 * SHELL_OD, shell_h, name=name + "_shell", max_deviation=max_deviation)
    axle = part.cylinder(xy(shell_z - 1.0), AXLE_R, shell_h + 2.0, name=name + "_axle", max_deviation=max_deviation)
    shell = part.cut(shell, axle, name=name + "_shell_bored")

    mount_sk = part.sketch(frame=xy(left_flange_z - DISC_MOUNT_H), name=name + "_disc_mount")
    mount_sk.add_circle((0.0, 0.0), 0.5 * DISC_MOUNT_OD, name="od")
    mount_sk.add_circle((0.0, 0.0), AXLE_R, name="bore")
    mount = part.extrude(
        mount_sk, DISC_MOUNT_H, name=name + "_disc_mount_blank", max_deviation=max_deviation)
    bolt_sk = part.sketch(frame=xy(left_flange_z - DISC_MOUNT_H - 0.2), name=name + "_disc_bolts")
    bolt = bolt_sk.add_circle((0.5 * DISC_BOLT_BCD, 0.0), DISC_BOLT_HOLE_R, name="bolt0")
    bolt_sk.repeat_circular([bolt], (0.0, 0.0), DISC_BOLT_N)
    bolt_cut = part.extrude(
        bolt_sk, DISC_MOUNT_H + 0.4, name=name + "_disc_bolts", max_deviation=max_deviation)
    mount = part.cut(mount, bolt_cut, name=name + "_disc_mount")

    body = part.union(freehub, journal, name=name + "_u0")
    body = part.union(body, shoulder, name=name + "_u1")
    body = part.union(body, flange("flange_r", right_flange_z), name=name + "_u2")
    body = part.union(body, shell, name=name + "_u3")
    body = part.union(body, flange("flange_l", left_flange_z, math.pi / SPOKE_N), name=name + "_u4")
    return part.union(body, mount, name=name)


def hub_spoke_layout():
    """Flange faces, spoke circle, and disc plane used by bike_wheel.py."""
    right_face = -(SHOULDER_T + FLANGE_T)
    left_face = right_face - FLANGE_SPAN - FLANGE_T
    return {
        "left_z": left_face + 0.5 * FLANGE_T,
        "right_z": right_face + 0.5 * FLANGE_T,
        "left_outboard": left_face,
        "right_outboard": right_face + FLANGE_T,
        "left_inboard": left_face + FLANGE_T,
        "right_inboard": right_face,
        "pcd": SPOKE_PCD,
        "holes": SPOKE_N,
        "hole_r": SPOKE_R,
        "flange_t": FLANGE_T,
        "disc_z": left_face - DISC_MOUNT_H,
    }


def new_part(extent=90.0, tolerance=0.1):
    return Part(vec3(-extent, -extent, -110.0), vec3(extent, extent, 60.0), tolerance=tolerance)


def _on_axis(z):
    return (0.0, 0.0, z)


def build_cassette(part=None, max_deviation=0.1):
    """Make every part and slide them onto the freehub along +Z.

    Every spline is already clocked with the wide lobe on +X, so the stack
    is just a known 9-speed pitch: 27-24-21 on the spider, then spacer+cog
    out to the 12T (collar built in) and the lockring.
    """
    if part is None:
        part = new_part(tolerance=max_deviation)

    hub = create_hub(part, "hub", max_deviation=max_deviation)
    spider = create_spider(part, "spider", max_deviation=max_deviation)
    lockring = create_lockring(part, "lockring", max_deviation=max_deviation)
    cluster = {
        n: create_sprocket(
            part, n, "cog{0}".format(n), max_deviation=max_deviation, bore="clearance")
        for n in CLUSTER
    }
    singles = {}
    for n in INDIVIDUALS:
        singles[n] = create_sprocket(
            part, n, "cog{0}".format(n),
            max_deviation=max_deviation,
            bore="spline",
            built_in_spacer=(n == 12),
            lightening=(n >= 17),
        )
    spacers = [create_spacer(part, "spacer0", max_deviation=max_deviation)]
    for i in range(1, 5):
        spacers.append(part.copy_solid(spacers[0], "spacer{0}".format(i)))

    asm = part.assembly("cassette")
    ground = asm.add_part(hub)
    asm.fix(ground)

    # Shoulder is z=0. Spider and 27T sit there; 24 and 21 step out by cog pitch.
    asm.add_part(spider, _on_axis(0.0))
    for i, n in enumerate(CLUSTER):
        asm.add_part(cluster[n], _on_axis(i * COG_PITCH))

    # Outboard of the 21T: washer, cog, washer, cog … then the 12T collar.
    z = 2.0 * COG_PITCH + COG_THICKNESS
    for i, n in enumerate(INDIVIDUALS):
        if n == 12:
            asm.add_part(singles[n], _on_axis(z))
            z += SPACER_THICK + COG_THICKNESS
        else:
            asm.add_part(spacers[i], _on_axis(z))
            z += SPACER_THICK
            asm.add_part(singles[n], _on_axis(z))
            z += COG_THICKNESS

    asm.add_part(lockring, _on_axis(z))
    return asm


if __name__ == "__main__":
    cassette = build_cassette()
    print(cassette)
    cassette.show(title="camber 9-speed 12-27 cassette")
