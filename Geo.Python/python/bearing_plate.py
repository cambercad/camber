"""Bearing-link assembly — CAD mates on Assembly.

Parts: horizontal rod (A), L-leg (B), bearing pin, support post on A.
Same layout as GeoScriptViewer/TestScriptBearingPlateAssembly.py.
"""
import math

from camber import Frame, Part, vec3

plate_w = 200.0
plate_h = 40.0
plate_thickness = 10.0
hole_radius = 5.0
hole_inset = 20.0

part = Part(vec3(-300), vec3(300), tolerance=0.01)


def build_holed_plate(profile_name, hole_sketch_name, solid_name, cutter_name, result_name):
    profile_sk = part.sketch("xy", name=profile_name)
    profile_sk.add_rectangle((0, 0), (plate_w, plate_h))
    solid = part.extrude_two_sides(
        profile_sk, plate_thickness * 0.5, plate_thickness * 0.5, name=solid_name)

    holes_sk = part.sketch("xy", name=hole_sketch_name)
    hole_y = plate_h * 0.5
    holes_sk.add_circle((hole_inset, hole_y), hole_radius, name="Circle1")
    holes_sk.add_circle((plate_w - hole_inset, hole_y), hole_radius, name="Circle2")
    cutter = part.extrude_two_sides(
        holes_sk, plate_thickness * 0.5, plate_thickness * 0.5, name=cutter_name)
    return part.cut(solid, cutter, name=result_name)


def build_square_post(prefix, size, height, result_name):
    sk = part.sketch("xy", name=prefix + "_profile")
    sk.add_rectangle((0, 0), (size, size))
    return part.extrude(sk, height, name=result_name)


plate_a = build_holed_plate(
    "plate_a_profile", "plate_a_holes", "plate_a_solid", "plate_a_cutter", "plateA")
plate_b = build_holed_plate(
    "plate_b_profile", "plate_b_holes", "plate_b_solid", "plate_b_cutter", "plateB")

post_size = 20.0
post_height = 50.0
support_post = build_square_post("support_post", post_size, post_height, "supportPost")

pin_radius = hole_radius - 0.5
pin_height = plate_thickness + 2.0
pin = part.cylinder(Frame(vec3(0, 0, 0)), pin_radius, pin_height, name="bearingPin")

asm = part.assembly("bearing_link")
asm.solve_after_every_constraint = False

body_a = asm.add_part(plate_a)
asm.fix(body_a)

body_b = asm.add_part(
    plate_b,
    (40, 0, 0),
    (0, 0, 0.70710678, 0.70710678),
)

elbow_a = body_a.axis("Circle1")
elbow_b = body_b.axis("Circle1")
leg_a = body_a.axis_at((plate_w * 0.5, plate_h * 0.5, 0), (1, 0, 0))
leg_b = body_b.axis_at((plate_w * 0.5, plate_h * 0.5, 0), (1, 0, 0))

asm.coincident(elbow_a, elbow_b)
asm.parallel(elbow_a, elbow_b)
asm.angle(leg_a, leg_b, math.pi * 0.5)
asm.coincident(body_a.point_at((0, 0, 0)), body_b.point_at((0, plate_h, 0)))

body_pin = asm.add_part(pin, (hole_inset, plate_h * 0.5, -plate_thickness * 0.5 - 1))
pin_axis = body_pin.axis_at((0, 0, pin_height * 0.5), (0, 0, 1))
asm.concentric(pin_axis, elbow_a)

body_post = asm.add_part(
    support_post, (plate_w - hole_inset, plate_h * 0.5, plate_thickness * 0.5))

top_a = body_a.plane_at((plate_w * 0.5, plate_h * 0.5, plate_thickness * 0.5), (0, 0, 1))
bottom_post = body_post.plane_at((post_size * 0.5, post_size * 0.5, 0), (0, 0, -1))
post_axis = body_post.axis_at((post_size * 0.5, post_size * 0.5, post_height * 0.5), (0, 0, 1))

asm.coincident(top_a, bottom_post)
asm.parallel(elbow_a, post_axis)
asm.perpendicular(leg_a, post_axis)
asm.distance(
    body_a.point_at((plate_w - hole_inset, plate_h * 0.5, plate_thickness * 0.5)),
    body_post.point_at((post_size * 0.5, post_size * 0.5, 0)),
    0,
)

asm.solve()
print(asm)
asm.show(title="bearing link")
