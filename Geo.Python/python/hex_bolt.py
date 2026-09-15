"""ISO 4017 hex-head screw — same construction and units as TestScriptMetricHexBolt.py.

Geometry is in metres (M10 × 35 mm shank). Thread cutter sizes (0.002, 0.001, …)
are those metre constants, not millimetres.
"""
import math

from camber import BOOLEAN_DIFFERENCE, BOOLEAN_UNION, Frame, Part, vec3

metric_designation = "M10"
shank_length_m = 0.035
max_dev = 1e-6

MM_TO_M = 1e-3
SHANK_OVERSIZE_D_M = 1.0 * MM_TO_M
TWO_PI = 2.0 * math.pi

ISO_HEAD_ANCHORS = [
    (2, 4.0, 1.4), (3, 5.5, 2.0), (4, 7.0, 2.8), (5, 8.0, 3.5), (6, 10.0, 4.0),
    (8, 13.0, 5.3), (10, 16.0, 6.4), (12, 18.0, 7.5), (14, 22.0, 8.8), (16, 24.0, 10.0),
    (18, 27.0, 11.5), (20, 30.0, 12.5),
]

METRIC_THREADS_MM = {
    "M2": (2.0, 0.4), "M3": (3.0, 0.5), "M4": (4.0, 0.7), "M5": (5.0, 0.8),
    "M6": (6.0, 1.0), "M7": (7.0, 1.0), "M8": (8.0, 1.25), "M9": (9.0, 1.25),
    "M10": (10.0, 1.5), "M11": (11.0, 1.5), "M12": (12.0, 1.75), "M13": (13.0, 1.75),
    "M14": (14.0, 2.0), "M15": (15.0, 2.0), "M16": (16.0, 2.0), "M17": (17.0, 2.0),
    "M18": (18.0, 2.5), "M19": (19.0, 2.5), "M20": (20.0, 2.5),
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_metric_designation(designation):
    d = float(designation.strip()[1:])
    nominal_mm = int(round(d))
    return nominal_mm, "M" + str(nominal_mm)


def iso4017_head(nominal_m):
    for m, s, k in ISO_HEAD_ANCHORS:
        if m == nominal_m:
            return s, k
    for i in range(len(ISO_HEAD_ANCHORS) - 1):
        a = ISO_HEAD_ANCHORS[i]
        b = ISO_HEAD_ANCHORS[i + 1]
        if a[0] < nominal_m < b[0]:
            t = (nominal_m - a[0]) / float(b[0] - a[0])
            return a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])
    raise ValueError("unsupported designation {0}".format(nominal_m))


def metric_thread_m(thread_key):
    d_mm, p_mm = METRIC_THREADS_MM[thread_key.upper()]
    return d_mm * MM_TO_M, p_mm * MM_TO_M


def bolt_revolve_meridian_frame(origin_world):
    return Frame(origin_world, x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0))


def create_hex_head_bolt(part, designation, shank_length_m, max_deviation, name=None):
    nominal_mm, thread_key = parse_metric_designation(designation)
    s_mm, k_mm = iso4017_head(nominal_mm)
    if name is None:
        name = thread_key + "_hex_bolt"

    d_m = nominal_mm * MM_TO_M
    shank_radius = 0.5 * d_m + 0.5 * SHANK_OVERSIZE_D_M
    L = float(shank_length_m)
    k = k_mm * MM_TO_M
    s = s_mm * MM_TO_M
    z_top = L + k
    circum_radius = s / math.sqrt(3.0)

    part_overlap = clamp(10.0 * max_deviation, 0.05 * MM_TO_M, 0.2 * MM_TO_M)
    overlap_cap = 0.025 * min(L, k)
    if overlap_cap > 0 and part_overlap > overlap_cap:
        part_overlap = overlap_cap

    axis = Frame(vec3(0, 0, 0))
    shank = part.cylinder(axis, shank_radius, L + part_overlap, name=name + "_shank", max_deviation=max_deviation)

    washer_h = max(0.22 * d_m, 0.55 * MM_TO_M)
    washer = part.cylinder(
        Frame(vec3(0, 0, L - washer_h)), 0.25 * s, washer_h + part_overlap,
        name=name + "_washer", max_deviation=max_deviation,
    )

    hex_sketch = part.sketch(frame=Frame(vec3(0, 0, L - part_overlap)), name=name + "_head_profile")
    for i in range(6):
        t0 = i * (math.pi / 3.0)
        t1 = ((i + 1) % 6) * (math.pi / 3.0)
        hex_sketch.add_line(
            (circum_radius * math.cos(t0), circum_radius * math.sin(t0)),
            (circum_radius * math.cos(t1), circum_radius * math.sin(t1)),
        )
    head = part.extrude(hex_sketch, k + part_overlap, name=name + "_head", max_deviation=max_deviation)

    body = part.boolean(
        part.boolean(shank, washer, BOOLEAN_UNION, name=name + "_u1"),
        head,
        BOOLEAN_UNION,
        name=name + "_u2",
    )

    r_flat_top = 0.5 * s * 0.96
    chamfer_depth = clamp(0.32 * k, 0.12 * k, 0.85 * k)
    r_outer = circum_radius + max(0.6 * MM_TO_M, 0.04 * s)
    z_cutter_top = z_top + max(0.35 * k, 0.5 * MM_TO_M)
    meridian = bolt_revolve_meridian_frame(vec3(0, 0, 0))
    chamfer_sketch = part.sketch(frame=meridian, name=name + "_top_chamfer_cutter")
    p_a = (z_cutter_top, 0)
    p_b = (z_cutter_top, r_outer)
    p_c = (z_top - chamfer_depth, r_outer)
    p_d = (z_top, r_flat_top)
    p_e = (z_top, 0)
    chamfer_sketch.add_line(p_a, p_b)
    chamfer_sketch.add_line(p_b, p_c)
    chamfer_sketch.add_line(p_c, p_d)
    chamfer_sketch.add_line(p_d, p_e)
    chamfer_sketch.add_line(p_e, p_a)
    with_chamfer = part.boolean(
        body,
        part.revolve(chamfer_sketch, TWO_PI, name=name + "_topChamferCut", max_deviation=max_deviation),
        BOOLEAN_DIFFERENCE,
        name=name + "_chamfer",
    )

    tip_chamfer_axial = clamp(0.55 * d_m, 0.6 * MM_TO_M, 2.5 * MM_TO_M)
    if tip_chamfer_axial >= shank_radius:
        tip_chamfer_axial = 0.45 * shank_radius
    r_tip_clear = shank_radius + max(0.5 * MM_TO_M, 0.05 * d_m)
    z_tip_lo = -max(0.15 * MM_TO_M, 0.04 * tip_chamfer_axial)
    tip_chamfer_sketch = part.sketch(frame=bolt_revolve_meridian_frame(vec3(0, 0, 0)), name=name + "_tip_chamfer_cutter")
    t_a = (z_tip_lo, 0)
    t_b = (z_tip_lo, r_tip_clear)
    t_c = (tip_chamfer_axial, r_tip_clear)
    t_d = (tip_chamfer_axial, shank_radius)
    t_e = (0, shank_radius - tip_chamfer_axial)
    t_f = (0, 0)
    tip_chamfer_sketch.add_line(t_a, t_b)
    tip_chamfer_sketch.add_line(t_b, t_c)
    tip_chamfer_sketch.add_line(t_c, t_d)
    tip_chamfer_sketch.add_line(t_d, t_e)
    tip_chamfer_sketch.add_line(t_e, t_f)
    tip_chamfer_sketch.add_line(t_f, t_a)
    with_tip = part.boolean(
        with_chamfer,
        part.revolve(tip_chamfer_sketch, TWO_PI, name=name + "_tipChamferCut", max_deviation=max_deviation),
        BOOLEAN_DIFFERENCE,
        name=name + "_tipChamfer",
    )

    thread_axis_cs = Frame(vec3(0, 0, L), x=(0, 1, 0), y=(1, 0, 0), z=(0, 0, -1))
    d_maj, pitch = metric_thread_m(thread_key)
    thread_negative = part.create_metric_thread_for_bolt_negative(
        thread_axis_cs, d_maj, pitch, L, name=name + "_threadNeg",
        max_deviation=max_deviation, right_handed=True, outer_radius=shank_radius + 0.002,
    )
    return part.boolean(with_tip, thread_negative, BOOLEAN_DIFFERENCE, name=name)


if __name__ == "__main__":
    part = Part(vec3(-0.5), vec3(0.5), tolerance=max_dev)
    bolt = create_hex_head_bolt(part, metric_designation, shank_length_m, max_dev)
    print(metric_designation, bolt)
    bolt.show(title="camber {0} hex bolt".format(metric_designation))
