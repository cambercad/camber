"""3:1 planetary gearbox (ring fixed, carrier output).

Ratio i = 1 + Zr/Zs = 3, so Zr = 2 Zs. Standard planetary also needs
Zr = Zs + 2 Zp → Zp = Zs/2. Three equally spaced planets require
(Zs + Zr) / 3 integer, which follows automatically.

Tooth counts 42 / 21 / 84 (sun / planet / ring):
  * 20° involute, module 1.5 mm (hob / gear-shaper sizes)
  * planet z = 21 is above the 17-tooth undercut limit and odd, so a space
    faces the sun while a tooth faces the ring (even planets cannot do both)
  * theoretical centre distance a = m/2 (Zs + Zp); planet-to-ring matches
  * face 8×module; DIN-ish root fillet lives in the involute helper
  * spur rather than herringbone so planets assemble axially through the ring

Housing is grounded. Ring, cover, bearings, carrier, sun, pins, planets, and
screws are placed roughly then mated on named cylinder walls (Zylindermantel)
and those cylinders' end caps, then solved.

From Geo.Python, venv active (rebuild the wheel once for internal gears):

  python python\\planetary_gearbox.py
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from camber import Frame, Part, vec3

from gears import (
    deep_groove_bearing,
    external_spur,
    grooved_shaft,
    hex_cap_screw,
    internal_ring,
    internal_root_radius,
    pitch_radius,
    plate_with_holes,
    quat_z,
    tip_radius,
)

TWO_PI = 2.0 * math.pi

# --- mesh (mm) ---
MODULE = 1.5
PRESSURE = math.radians(20.0)
ZS = 42
ZP = 21
ZR = 84
N_PLANETS = 3
FACE = 12.0
ADDENDUM = 1.0
DEDENDUM = 1.25

# Fits / hardware
SUN_SHAFT_D = 12.0
PLANET_PIN_D = 8.0
PLANET_BORE_D = 8.1
PIN_HEAD_D = 11.0
CIRCLIP_W = 0.9
CIRCLIP_DEPTH = 0.4
CARRIER_PLATE = 4.0
AXIAL_PLAY = 0.4
RING_RIM = 8.0
COVER_T = 8.0
HOUSING_WALL = 6.0
N_BOLTS = 6
BOLT_D = 4.0
BOLT_AF = 7.0
BOLT_HEAD = 2.8
BOLT_LEN = 16.0

# Bearings (ISO 15)
BRG_IN = (12.0, 28.0, 8.0)
BRG_OUT = (20.0, 42.0, 12.0)

TOLERANCE = 0.08

assert ZR == 2 * ZS, "3:1 ring/sun"
assert ZS == 2 * ZP, "standard planetary: Zr = Zs + 2 Zp"
assert (ZS + ZR) % N_PLANETS == 0, "equal planet spacing"
assert ZP % 2 == 1, "odd planet teeth so sun-space and ring-tooth oppose"

RP_S = pitch_radius(MODULE, ZS)
RP_P = pitch_radius(MODULE, ZP)
RP_R = pitch_radius(MODULE, ZR)
CD = 0.5 * MODULE * (ZS + ZP)
assert abs(CD - (RP_R - RP_P)) < 1e-9
assert abs(CD - (RP_S + RP_P)) < 1e-9

SUN_TIP = tip_radius(MODULE, ZS, ADDENDUM)
RING_ROOT_R = internal_root_radius(MODULE, ZR, DEDENDUM)
RING_R = RING_ROOT_R + RING_RIM
FLANGE_R = RING_R + 7.0
BCD_R = 0.5 * (RING_R + FLANGE_R)
HOUSING_R = FLANGE_R + HOUSING_WALL
CARRIER_R = CD + 0.5 * PIN_HEAD_D + 4.0
SUN_CLEAR = SUN_TIP + 1.2
OUTPUT_HUB_D = BRG_OUT[0]
OUTPUT_HUB_L = 22.0

# z = 0 is the mesh midplane
Z_GEAR0 = -0.5 * FACE
Z_GEAR1 = 0.5 * FACE
Z_PLATE_R0 = Z_GEAR0 - AXIAL_PLAY - CARRIER_PLATE
Z_PLATE_F0 = Z_GEAR1 + AXIAL_PLAY
Z_PLATE_F1 = Z_PLATE_F0 + CARRIER_PLATE
Z_RING0 = Z_GEAR0 - 1.0
Z_RING1 = Z_GEAR1 + 1.0
Z_COVER0 = Z_PLATE_F1 + 0.5
Z_COVER1 = Z_COVER0 + COVER_T
Z_HOUSING0 = Z_PLATE_R0 - BRG_OUT[2] - 4.0
Z_HOUSING1 = Z_COVER0
PIN_Z0 = Z_PLATE_R0 - 1.0
PIN_Z1 = Z_PLATE_F1 + CIRCLIP_W + 1.2


def planet_phi(k):
    return TWO_PI * k / N_PLANETS


def planet_spin(k):
    phi = planet_phi(k)
    return math.pi - math.pi / ZP - phi * ZS / ZP


def planet_pos(k):
    phi = planet_phi(k)
    return (CD * math.cos(phi), CD * math.sin(phi), 0.0)


def build_sun(part):
    pinion = external_spur(
        part, "sun", MODULE, ZS, FACE, bore=0.0, pressure=PRESSURE,
        addendum=ADDENDUM, dedendum=DEDENDUM, max_deviation=TOLERANCE)
    shaft = grooved_shaft(
        part, "sun_shaft", 0.5 * SUN_SHAFT_D,
        Z_GEAR0 - 2.0, Z_COVER1 + BRG_IN[2] + 8.0,
        grooves=((Z_COVER1 + BRG_IN[2] + 1.0, CIRCLIP_W, CIRCLIP_DEPTH),))
    key = part.cuboid(
        Frame((0.5 * SUN_SHAFT_D, -2.0, Z_COVER1 + 2.0)),
        (2.2, 4.0, 14.0), name="sun_keyway")
    return (pinion + shaft) - key


def build_planet(part):
    return external_spur(
        part, "planet", MODULE, ZP, FACE, bore=0.5 * PLANET_BORE_D,
        pressure=PRESSURE, addendum=ADDENDUM, dedendum=DEDENDUM,
        max_deviation=TOLERANCE)


def build_ring(part):
    ring = internal_ring(
        part, "ring", MODULE, ZR, Z_RING1 - Z_RING0, RING_R,
        pressure=PRESSURE, addendum=ADDENDUM, dedendum=DEDENDUM,
        max_deviation=TOLERANCE)
    flange_t = 5.0
    flange = part.cylinder((0.0, 0.0, Z_RING1), FLANGE_R, flange_t, name="ring_flange")
    flange = flange - part.cylinder(
        (0.0, 0.0, Z_RING1 - 0.2), RING_R - 0.2, flange_t + 0.4, name="ring_flange_id")
    for i in range(N_BOLTS):
        a = i * TWO_PI / N_BOLTS
        x, y = BCD_R * math.cos(a), BCD_R * math.sin(a)
        flange = flange - part.cylinder(
            (x, y, Z_RING1 - 0.2), 0.5 * BOLT_D + 0.1, flange_t + 0.4,
            name="ring_flange_h{0}".format(i))
    return ring + flange


def build_carrier(part):
    holes = [(0.0, 0.0, SUN_CLEAR)]
    for k in range(N_PLANETS):
        x, y, _ = planet_pos(k)
        holes.append((x, y, 0.5 * PLANET_PIN_D))
    rear = plate_with_holes(
        part, "carrier_rear", CARRIER_R, CARRIER_PLATE, holes, z0=Z_PLATE_R0)
    front = plate_with_holes(
        part, "carrier_front", CARRIER_R, CARRIER_PLATE, holes, z0=Z_PLATE_F0)
    hub = part.cylinder(
        (0.0, 0.0, Z_HOUSING0 + 2.0), 0.5 * OUTPUT_HUB_D, OUTPUT_HUB_L, name="carrier_hub")
    hub_bore = part.cylinder(
        (0.0, 0.0, Z_HOUSING0 + 1.5), 0.5 * OUTPUT_HUB_D - 3.0, OUTPUT_HUB_L + 1.0,
        name="carrier_hub_bore")
    shoulder = part.cylinder(
        (0.0, 0.0, Z_PLATE_R0 - 2.0), 0.5 * BRG_OUT[1] - 2.0, 2.0, name="carrier_shoulder")
    return (rear + front + hub + shoulder) - hub_bore


def build_housing(part):
    h = Z_HOUSING1 - Z_HOUSING0
    cup = part.cylinder((0.0, 0.0, Z_HOUSING0), HOUSING_R, h, name="housing_od")
    ring_pocket = part.cylinder(
        (0.0, 0.0, Z_RING0 - 0.2), RING_R + 0.05, (Z_RING1 - Z_RING0) + 0.4,
        name="housing_ring_pocket")
    cup = cup - ring_pocket
    flange_pocket = part.cylinder(
        (0.0, 0.0, Z_RING1), FLANGE_R + 0.2, Z_HOUSING1 - Z_RING1 + 0.2,
        name="housing_flange_pocket")
    cup = cup - flange_pocket
    seat = part.cylinder(
        (0.0, 0.0, Z_HOUSING0 + 1.0), 0.5 * BRG_OUT[1], BRG_OUT[2] + 0.2, name="housing_brg")
    cup = cup - seat
    through = part.cylinder(
        (0.0, 0.0, Z_HOUSING0 - 1.0), 0.5 * OUTPUT_HUB_D + 0.3, 8.0, name="housing_out")
    cup = cup - through
    for i in range(N_BOLTS):
        a = i * TWO_PI / N_BOLTS
        x, y = BCD_R * math.cos(a), BCD_R * math.sin(a)
        cup = cup - part.cylinder(
            (x, y, Z_COVER0 - 18.0), 0.5 * BOLT_D, 20.0, name="housing_tap{0}".format(i))
    return cup


def build_cover(part):
    cover = part.cylinder((0.0, 0.0, Z_COVER0), HOUSING_R, COVER_T, name="cover")
    cover = cover - part.cylinder(
        (0.0, 0.0, Z_COVER0 - 0.2), 0.5 * BRG_IN[1], COVER_T + 0.4, name="cover_id")
    for i in range(N_BOLTS):
        a = i * TWO_PI / N_BOLTS
        x, y = BCD_R * math.cos(a), BCD_R * math.sin(a)
        cover = cover - part.cylinder(
            (x, y, Z_COVER0 - 0.2), 0.5 * BOLT_D + 0.1, COVER_T + 0.4,
            name="cover_h{0}".format(i))
    seat = part.cylinder(
        (0.0, 0.0, Z_COVER0 - 0.1), 0.5 * BRG_IN[1], BRG_IN[2] + 0.2, name="cover_brg")
    return cover - seat


def build_pin(part, name="pin"):
    """Planet pin on the part Z axis. Head at PIN_Z0; instance XY comes from mates."""
    shank = part.cylinder(
        (0.0, 0.0, PIN_Z0 + 1.5), 0.5 * PLANET_PIN_D, PIN_Z1 - PIN_Z0 - 1.5,
        name=name)
    head = part.cylinder(
        (0.0, 0.0, PIN_Z0), 0.5 * PIN_HEAD_D, 1.6, name=name + "-head")
    groove_z = Z_PLATE_F1 + 0.35
    tube = part.cylinder(
        (0.0, 0.0, groove_z), 0.5 * PLANET_PIN_D + 0.2, CIRCLIP_W, name=name + "_ga")
    core = part.cylinder(
        (0.0, 0.0, groove_z - 0.05), 0.5 * PLANET_PIN_D - CIRCLIP_DEPTH,
        CIRCLIP_W + 0.1, name=name + "_gb")
    return (shank + head) - (tube - core)


def _axis(inst, op):
    """Cylindrical wall of an extruded circle (the pickable side patch)."""
    return inst.axis(op + "-Circle1")


def _bottom(inst, op):
    return inst.plane(op + "-ExtrudeBottom")


def _top(inst, op):
    return inst.plane(op + "-ExtrudeTop")


def mate_gearbox(asm, housing, ring, sun, planets, pins, carrier, cover, brg_in, brg_out, screws):
    """Ground the housing, then mate cylinder walls (Zylindermantel) and their end caps."""
    asm.fix(housing)

    asm.concentric(_axis(ring, "ring_blank"), _axis(housing, "housing_ring_pocket"))
    asm.coincident(_bottom(ring, "ring_flange"), _bottom(housing, "housing_flange_pocket"))
    asm.concentric(_axis(ring, "ring_flange_h0"), _axis(housing, "housing_tap0"))

    asm.concentric(_axis(cover, "cover"), _axis(housing, "housing_od"))
    asm.coincident(_bottom(cover, "cover"), _top(housing, "housing_od"))
    asm.concentric(_axis(cover, "cover_h0"), _axis(housing, "housing_tap0"))

    asm.concentric(_axis(brg_out, "brg_out_or"), _axis(housing, "housing_brg"))
    asm.coincident(_bottom(brg_out, "brg_out_or"), _bottom(housing, "housing_brg"))

    asm.concentric(_axis(brg_in, "brg_in_or"), _axis(cover, "cover_brg"))
    asm.coincident(_bottom(brg_in, "brg_in_or"), _bottom(cover, "cover"))

    asm.concentric(_axis(carrier, "carrier_hub"), _axis(brg_out, "brg_out_irb"))
    asm.distance(
        _bottom(carrier, "carrier_rear"), _bottom(housing, "housing_od"),
        Z_PLATE_R0 - Z_HOUSING0)

    asm.concentric(_axis(sun, "sun_shaft"), _axis(brg_in, "brg_in_irb"))
    asm.distance(
        _bottom(sun, "sun"), _bottom(housing, "housing_od"),
        Z_GEAR0 - Z_HOUSING0)

    for k, pin in enumerate(pins):
        asm.concentric(
            _axis(pin, pin.name),
            _axis(carrier, "carrier_rear_h{0}".format(k + 1)))
        asm.distance(
            _bottom(pin, pin.name + "-head"), _bottom(carrier, "carrier_rear"),
            PIN_Z0 - Z_PLATE_R0)

    for k, planet in enumerate(planets):
        asm.concentric(_axis(planet, "planet_bore"), _axis(pins[k], pins[k].name))
        asm.coincident(_bottom(planet, planet.name), _bottom(sun, "sun"))

    for i, screw in enumerate(screws):
        asm.concentric(_axis(screw, "m4_shank"), _axis(cover, "cover_h{0}".format(i)))
        asm.coincident(_bottom(screw, screw.name), _top(cover, "cover"))


def build_gearbox():
    extent = HOUSING_R + 20.0
    part = Part(vec3(-extent), vec3(extent), tolerance=TOLERANCE)

    sun = build_sun(part)
    planet0 = build_planet(part)
    planets = [planet0]
    for k in range(1, N_PLANETS):
        planets.append(part.copy_solid(planet0, "planet{0}".format(k)))
    ring = build_ring(part)
    carrier = build_carrier(part)
    housing = build_housing(part)
    cover = build_cover(part)
    brg_in = deep_groove_bearing(part, "brg_in", *BRG_IN)
    brg_out = deep_groove_bearing(part, "brg_out", *BRG_OUT)
    screw0 = hex_cap_screw(part, "m4", BOLT_D, BOLT_LEN, BOLT_AF, BOLT_HEAD)
    screws = [screw0]
    for i in range(1, N_BOLTS):
        screws.append(part.copy_solid(screw0, "m4_{0}".format(i)))
    pin0 = build_pin(part, "pin0")
    pins = [pin0]
    for k in range(1, N_PLANETS):
        pins.append(part.copy_solid(pin0, "pin{0}".format(k)))

    asm = part.assembly("planetary_3to1")
    asm.solve_after_every_constraint = False

    hsg = asm.add_part(housing)
    rng = asm.add_part(ring)
    sun_i = asm.add_part(sun)
    planet_i = []
    pin_i = []
    for k in range(N_PLANETS):
        x, y, _ = planet_pos(k)
        planet_i.append(asm.add_part(planets[k], (x, y, 0.0), quat_z(planet_spin(k))))
        pin_i.append(asm.add_part(pins[k], (x, y, 0.0)))
    car = asm.add_part(carrier)
    cvr = asm.add_part(cover)
    bin_i = asm.add_part(brg_in, (0.0, 0.0, Z_COVER0 + 0.5 * BRG_IN[2]))
    bout_i = asm.add_part(brg_out, (0.0, 0.0, Z_HOUSING0 + 1.0 + 0.5 * BRG_OUT[2]))
    screw_i = []
    for i in range(N_BOLTS):
        a = i * TWO_PI / N_BOLTS
        x, y = BCD_R * math.cos(a), BCD_R * math.sin(a)
        screw_i.append(asm.add_part(screws[i], (x, y, Z_COVER1), quat_z(a)))

    mate_gearbox(
        asm, hsg, rng, sun_i, planet_i, pin_i, car, cvr, bin_i, bout_i, screw_i)
    asm.solve()

    print(
        "3:1 planetary  z={0}/{1}/{2}  m={3}  a={4:.2f} mm  i={5:.3f}".format(
            ZS, ZP, ZR, MODULE, CD, 1.0 + ZR / float(ZS)))
    return asm


if __name__ == "__main__":
    gearbox = build_gearbox()
    print(gearbox)
    gearbox.show(title="3:1 planetary gearbox")
