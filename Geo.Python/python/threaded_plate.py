"""Rectangular plate with a centred metric tapped hole.

Geometry is in metres (M10 through an 8 mm plate), same units as hex_bolt.py.
The bore is tap-drill size (major − pitch); create_metric_thread_for_hole_negative
then carves the ISO 60° internal thread.

Set include_bore_chamfers True to add the nut-style lead-in/out countersinks
(those two CSG unions dominate the cutter build).
"""
from camber import BOOLEAN_DIFFERENCE, Frame, Part, vec3
from hex_bolt import MM_TO_M, clamp, metric_thread_m, parse_metric_designation

metric_designation = "M10"
plate_w_m = 0.040
plate_h_m = 0.040
plate_thickness_m = 0.008
max_dev = 1e-6
include_bore_chamfers = False


def create_metric_threaded_plate(
        part, designation, plate_w_m, plate_h_m, thickness_m, max_deviation,
        name=None, include_bore_chamfers=False):
    _, thread_key = parse_metric_designation(designation)
    if name is None:
        name = thread_key + "_threaded_plate"

    d_maj, pitch = metric_thread_m(thread_key)
    tap_diameter = d_maj - pitch
    hole_radius = 0.5 * tap_diameter
    w = float(plate_w_m)
    h = float(plate_h_m)
    t = float(thickness_m)

    part_overlap = clamp(10.0 * max_deviation, 0.05 * MM_TO_M, 0.2 * MM_TO_M)

    plate_sk = part.sketch(frame=Frame(vec3(0, 0, 0)), name=name + "_profile")
    plate_sk.add_rectangle((-0.5 * w, -0.5 * h), (0.5 * w, 0.5 * h))
    plate = part.extrude(plate_sk, t, name=name + "_blank", max_deviation=max_deviation)

    hole = part.cylinder(
        Frame(vec3(0, 0, -part_overlap)),
        hole_radius,
        t + 2.0 * part_overlap,
        name=name + "_tapHole",
        max_deviation=max_deviation,
    )
    holed = part.boolean(plate, hole, BOOLEAN_DIFFERENCE, name=name + "_holed")

    thread_axis = Frame(vec3(0, 0, 0))
    thread_negative = part.create_metric_thread_for_hole_negative(
        thread_axis, d_maj, pitch, t, name=name + "_threadNeg",
        max_deviation=max_deviation, right_handed=True,
        include_bore_chamfers=include_bore_chamfers,
    )
    return part.boolean(holed, thread_negative, BOOLEAN_DIFFERENCE, name=name)


if __name__ == "__main__":
    part = Part(vec3(-0.5), vec3(0.5), tolerance=max_dev)
    plate = create_metric_threaded_plate(
        part, metric_designation, plate_w_m, plate_h_m, plate_thickness_m, max_dev,
        include_bore_chamfers=include_bore_chamfers)
    print(metric_designation, "chamfers={0}".format(include_bore_chamfers), plate)
    plate.show(title="camber {0} threaded plate".format(metric_designation))
