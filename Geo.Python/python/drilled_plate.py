"""Rectangular plate with a circular hole ring and a rectangular hole grid.

RepeatCircular / RepeatGrid copies track the seed curve. Same layout as
GeoScriptViewer/TestScriptDrilledPlate.py.
"""
from camber import Part, vec3

plate_w = 180.0
plate_h = 120.0
plate_thickness = 10.0
hole_radius = 3.0

part = Part(vec3(-120, -80, -10), vec3(120, 80, 15), tolerance=0.01)

plate_sk = part.sketch("xy", name="plate")
plate_sk.add_rectangle((0, 0), (plate_w, plate_h))
plate = part.extrude_two_sides(plate_sk, plate_thickness * 0.5, plate_thickness * 0.5, name="plate")

holes_sk = part.sketch("xy", name="holes")
ring_center = (-50.0, 0.0)
ring_radius = 32.0
seed_ring = holes_sk.add_circle((ring_center[0] + ring_radius, ring_center[1]), hole_radius, name="ring0")
holes_sk.repeat_circular([seed_ring], ring_center, 8)

grid_origin = (25.0, -22.5)
pitch_x = 18.0
pitch_y = 15.0
seed_grid = holes_sk.add_circle(grid_origin, hole_radius, name="grid0")
holes_sk.repeat_grid([seed_grid], 4, 4, (pitch_x, 0), (0, pitch_y))

hole_cutter = part.extrude_two_sides(
    holes_sk, plate_thickness * 0.5, plate_thickness * 0.5, name="hole_cutter")
plate = plate - hole_cutter
print(plate)
plate.show(title="drilled plate")
