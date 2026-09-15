"""Plate with an outward offset rim — GeoScriptViewer/TestScriptOffsetPlate.py.

Offset must run on the same sketch that owns the source curves. A helper
rectangle drives the rim; the plate base comes from a separate outline sketch.
"""
from camber import Part, vec3

plate_w = 100.0
plate_h = 60.0
plate_thickness = 8.0
rim_height = 3.0
rim_offset = 4.0

part = Part(vec3(-80, -50, -10), vec3(80, 50, 20), tolerance=0.01)

outline_sk = part.sketch("xy", name="outline")
outline_sk.add_rectangle((0, 0), (plate_w, plate_h))

rim_sk = part.sketch("xy", name="rim")
south = rim_sk.add_line((0, 0), (plate_w, 0), name="south", construction=True)
east = rim_sk.add_line((plate_w, 0), (plate_w, plate_h), name="east", construction=True)
north = rim_sk.add_line((plate_w, plate_h), (0, plate_h), name="north", construction=True)
west = rim_sk.add_line((0, plate_h), (0, 0), name="west", construction=True)
rim_sk.offset([south, east, north, west], rim_offset, side="out", join="round")

base = part.extrude_two_sides(
    outline_sk, plate_thickness * 0.5, plate_thickness * 0.5, name="plate_base")
rim_solid = part.extrude_two_sides(
    rim_sk, plate_thickness * 0.5 + rim_height, plate_thickness * 0.5, name="rim")
plate = base + rim_solid
print(plate)
plate.show(title="offset plate")
