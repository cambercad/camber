"""Matched lofts, Boolean features, and named edge treatments (millimetres).

The gallery shows successful cases. --probe runs a focused kernel reproduction
and lets any error propagate; see the companion
Markdown file for the distinction between supported examples and open defects.
"""

import argparse
import json
import math
import time
from pathlib import Path

from camber import Frame, Part, render_views, set_progress_log, show
from loft_transition import section

COUPONS = (
    "convex_edges", "concave_notch", "mixed_selection", "corner_network",
    "different_radii", "chamfer_edges", "round_and_chamfer", "meeting_treatments",
)
SCULPTURES = ("curved_rounds", "curved_chamfers", "swept_vane", "twisted_duct", "crossed_fins", "helical_crown", "hollow_rounds", "hollow_chamfers")
CASES = SCULPTURES + COUPONS
PROBES = ("curved_round", "curved_chamfer", "curved_four_edges", "inward_chamfer_network", "enlarged_bounds_corner",
    "tapered_round_corner", "tapered_chamfer_corner", "pocket_round",
    "flange_chamfer_corner", "mixed_corner", "duct_rim_rounds", "duct_rim_chamfers",
)


def rectangle(part, name, z, width, height, center=(0, 0)):
    """A solved rectangular sketch; reuse the loft example's dimension scheme."""
    return section(part, name, (center[0], center[1], z), width, height, 0)


def housing(part):
    """Three directed, four-curve sections define a mildly tapered enclosure."""
    profiles = [
        rectangle(part, "foot_section", 0, 32, 24),
        rectangle(part, "middle_section", 12, 30, 22),
        rectangle(part, "top_section", 24, 28, 20),
    ]
    return part.loft(profiles, first_curves=["bottom"] * len(profiles), name="housing")


def flange(part, body):
    profile = rectangle(part, "flange_profile", -6, 42, 34)
    blank = part.extrude(profile, 7, name="flange")
    return part.union(body, blank, name="flanged_housing")


def notch(part, body):
    # A through cut exposes an accessible inside corner, with planar end trims.
    profile = rectangle(part, "notch_profile", -10, 30, 30, center=(15, 15))
    cutter = part.extrude(profile, 45, name="notch")
    return part.cut(body, cutter, name="notched_housing")


def edge(first, second):
    return f"[{first},{second}]"


LONG_EDGES = [edge("housing-Side-" + first, "housing-Side-" + second)
              for first, second in (("bottom", "right"), ("right", "top"),
                                    ("top", "left"), ("left", "bottom"))]
FLANGE_CORNER = [edge("flange-bottom", "flange-right"),
                  edge("flange-bottom", "flange-ExtrudeTop"),
                  edge("flange-right", "flange-ExtrudeTop")]
TAPERED_CORNER = [LONG_EDGES[0], edge("housing-Side-bottom", "housing-EndCap"),
                  edge("housing-Side-right", "housing-EndCap")]
INSIDE_EDGE = edge("notch-bottom", "notch-left")


def rounded_section(part, name, origin, width, height, angle):
    """Two straight flanks joined tangentially to semicircular ends."""
    angle = math.radians(angle)
    frame = Frame(origin, x=(math.cos(angle), math.sin(angle), 0),
                  y=(-math.sin(angle), math.cos(angle), 0), z=(0, 0, 1))
    sketch = part.sketch(name=name, frame=frame, constrained=True)
    sketch.solve_after_every_constraint = False
    radius = height / 2
    half_flank = (width - height) / 2
    sketch.add_line((-half_flank, -radius), (half_flank, -radius), name="bottom")
    sketch.add_arc((half_flank, -radius), (half_flank + radius, 0),
                   (half_flank, radius), name="nose")
    sketch.add_line((half_flank, radius), (-half_flank, radius), name="top")
    sketch.add_arc((-half_flank, radius), (-half_flank - radius, 0),
                   (-half_flank, -radius), name="tail")
    sketch.fix("bottom@0")
    sketch.horizontal("bottom").horizontal("top").length("bottom", 2 * half_flank)
    sketch.radius("nose", radius).radius("tail", radius)
    sketch.tangent("bottom", "nose").tangent("top", "nose")
    sketch.tangent("top", "tail").tangent("bottom", "tail")
    sketch.solve()
    return sketch


def sculpted_loft(part, name, stations, *, rounded=False):
    """Dimensioned sections are the design table; the loft interpolates them."""
    make_section = rounded_section if rounded else section
    profiles = [make_section(part, f"{name}_station_{i + 1}", *station)
                for i, station in enumerate(stations)]
    # The first authored curve defines correspondence for both arc/line and
    # polygon sections; every authored boundary curve produces its own face.
    return part.loft(profiles, first_curves=["bottom"] * len(profiles), name=name)


VANE_STATIONS = (
    ((0, 0, 0), 30, 10, 0),
    ((0, 0, 14), 30, 10, 0),
    ((10, 2, 40), 27, 8, 18),
    ((-5, 5, 70), 23, 6, 42),
    ((0, 7, 96), 18, 4, 65),
)
DUCT_STATIONS = (
    ((0, 0, -1), 32, 26, 0),
    ((0, 0, 12), 32, 26, 0),
    ((7, 0, 38), 26, 22, 25),
    ((-6, 4, 64), 22, 26, 55),
    ((0, 6, 90), 30, 24, 85),
)


def longitudinal_edges(feature, *, loft=False):
    """Four authored rectangle corners, retained by both extrusions and lofts."""
    prefix = feature + ("-Side-" if loft else "-")
    return [edge(prefix + first, prefix + second)
            for first, second in (("bottom", "right"), ("right", "top"),
                                  ("top", "left"), ("left", "bottom"))]


def hollow_duct(part, shoe):
    outer = sculpted_loft(part, "duct", DUCT_STATIONS)
    inner_stations = [((x, y, z - 10 if i == 0 else z + 1 if i == 4 else z),
                       width - 7, height - 7, angle)
                      for i, ((x, y, z), width, height, angle)
                      in enumerate(DUCT_STATIONS)]
    inner = sculpted_loft(part, "passage", inner_stations)
    return part.cut(part.union(shoe, outer), inner, name="open_duct")


def opening_rim(solid, cap):
    """The complete opening boundary, including arcs from previous rounds."""
    return [name for name in solid.edge_names
            if cap in name and ("passage-" in name or "BlendStrip" in name)]


def build_sculpture(part, case):
    """Curved lofts, machined shoes, and explicit long-edge treatment studies."""
    curved_removed = 0.0
    inner_added = 0.0
    rim_removed = 0.0
    rim_edges = []
    shoe = part.extrude(rectangle(part, "shoe_profile", -9, 76, 68), 10,
                        name="shoe")
    if case == "twisted_duct":
        body = hollow_duct(part, shoe)
        untreated_volume = body.volume()
        body = part.fillet(body, longitudinal_edges("passage", loft=True), .6,
                           name="inner_upright_rounds")
        inner_added = body.volume() - untreated_volume
    elif case in ("hollow_rounds", "hollow_chamfers"):
        shell = part.extrude(rectangle(part, "shell_profile", 0, 40, 32), 60,
                             name="shell")
        passage = part.extrude(rectangle(part, "passage_profile", -10, 30, 22), 71,
                               name="passage")
        body = part.cut(part.union(shoe, shell), passage, name="open_extrusion")
        untreated_volume = body.volume()
        body = part.fillet(body, longitudinal_edges("passage"), 2,
                           name="inner_upright_rounds")
        inner_added = body.volume() - untreated_volume
        # Separate machining features allow the concave upright rounds to meet
        # the convex lip. The new arc edges belong to the full opening rim too.
        rim_edges = opening_rim(body, "shell-ExtrudeTop")
        before_rim = body.volume()
        operation = part.fillet if case == "hollow_rounds" else part.chamfer
        body = operation(body, rim_edges, .8, name="opening_rim_treatment")
        rim_removed = before_rim - body.volume()
    elif case == "helical_crown":
        stations = (
            ((22, 0, 0), 16, 5, 90),
            ((22, 0, 12), 16, 5, 90),
            ((18, 8, 34), 18, 4, 115),
            ((8, 16, 56), 15, 3, 145),
            ((0, 18, 78), 10, 2, 175),
        )
        seed = sculpted_loft(part, "crown_vane", stations)
        vanes = part.pattern_circular(seed, 6, name="crown_vanes")
        body = part.batch_union([shoe, *vanes])
    else:
        stations = [((x + 17, y, z), width * .7, height, angle)
                    for (x, y, z), width, height, angle in VANE_STATIONS] if case == "crossed_fins" else VANE_STATIONS
        vane = sculpted_loft(part, "vane", stations, rounded=case == "swept_vane")
        if case in ("curved_rounds", "curved_chamfers"):
            untreated_volume = vane.volume()
            operation = part.fillet if case == "curved_rounds" else part.chamfer
            long_edges = longitudinal_edges("vane", loft=True)
            vane = operation(vane, long_edges, .8, name="long_edge_treatment")
            curved_removed = untreated_volume - vane.volume()
            if curved_removed <= 0 or not vane.is_watertight():
                raise ValueError(f"{case}: curved treatment must remove material from a closed vane")
        vanes = part.pattern_circular(vane, 2, name="opposed_fins") if case == "crossed_fins" else [vane]
        body = part.batch_union([shoe, *vanes])

    # A generous corner relief leaves its concave vertical edge fully exposed.
    # Its offset locates the cut outside the loft root, preserving the wall.
    relief = part.extrude(rectangle(part, "relief_profile", -12, 18, 20,
                                    center=(36, 32)), 18, name="relief")
    body = part.cut(body, relief, name="relieved_shoe")
    before = body.volume()
    inside = edge("relief-bottom", "relief-left")
    inside_before = body.volume()
    body = part.chamfer(body, inside, 2, name="inward_chamfer")
    inward_added = body.volume() - inside_before
    # The three connected convex edges meet at the opposite shoe corner.
    corner = [edge("shoe-bottom", "shoe-left"),
              edge("shoe-bottom", "shoe-ExtrudeTop"),
              edge("shoe-left", "shoe-ExtrudeTop")]
    body = part.fillet(body, corner, 2.5, name="shoe_corner_network")
    body = part.chamfer(body, edge("shoe-top", "shoe-ExtrudeTop"), .8,
                        name="meeting_rim_chamfer")
    if not body.is_watertight() or body.volume() <= 0 or inward_added <= 0:
        raise ValueError(f"{case}: invalid solid or inward chamfer failed to add material")
    return body, {"case": case, "description": {
        "curved_rounds": "R0.8 on all four curved and twisted longitudinal loft edges",
        "curved_chamfers": "Distance-0.8 chamfers on all four curved longitudinal loft edges",
        "swept_vane": "Five-section swept vane with tangent arc/line sections on a machined mounting shoe",
        "twisted_duct": "Twisted hollow transition with R0.6 on all four inner upright edges",
        "hollow_rounds": "Hollow extrusion with four R2 inner uprights and R0.8 around the full opening rim",
        "hollow_chamfers": "Hollow extrusion with four R2 inner uprights and D0.8 around the full opening rim",
        "crossed_fins": "Opposed swept fins joined by Boolean union",
        "helical_crown": "Six curved loft vanes patterned around a common mounting shoe",
    }[case], "before_volume_mm3": before, "volume_mm3": body.volume(),
        "volume_change_mm3": body.volume() - before, "watertight": True,
        "inward_chamfer_added_mm3": inward_added,
        "curved_edge_removed_mm3": curved_removed,
        "inner_upright_added_mm3": inner_added,
        "opening_rim_removed_mm3": rim_removed,
        "opening_rim_edges": len(rim_edges),
        "treatments": "D2 inward relief chamfer; R2.5 three-edge corner; D0.8 joining rim chamfer"}


def build_case(part, case):
    if case in SCULPTURES:
        return build_sculpture(part, case)
    body = housing(part)
    if case in ("concave_notch", "mixed_selection"):
        body = notch(part, body)
    elif case in ("corner_network", "round_and_chamfer", "meeting_treatments"):
        body = flange(part, body)
    before = body.volume()

    if case == "convex_edges":
        result = part.fillet(body, LONG_EDGES, 1.5, name="four_convex_rounds")
        description = "Four convex longitudinal edges, R1.5"
    elif case == "concave_notch":
        result = part.fillet(body, INSIDE_EDGE, 1, name="concave_round")
        description = "Through-notch inside edge, R1; adds material"
    elif case == "mixed_selection":
        result = part.fillet(body, [INSIDE_EDGE, LONG_EDGES[3]], 1, name="mixed_rounds")
        description = "Separate convex and concave edges in one R1 operation"
    elif case == "corner_network":
        result = part.fillet(body, FLANGE_CORNER, 2, name="three_edge_corner")
        description = "Three connected convex flange edges meeting at one corner, R2"
    elif case == "different_radii":
        large = part.fillet(body, LONG_EDGES[0], 2, name="large_round")
        result = part.fillet(large, LONG_EDGES[2], .7, name="small_round")
        description = "Opposite edges, R2 and R0.7 in successive features"
    elif case == "chamfer_edges":
        result = part.chamfer(body, LONG_EDGES, 1, name="four_chamfers")
        description = "Four longitudinal edges with flat distance-1 chamfers"
    elif case == "round_and_chamfer":
        rounded = part.fillet(body, FLANGE_CORNER, 2, name="rounded_corner")
        result = part.chamfer(rounded, edge("flange-top", "flange-left"), .8,
                              name="round_and_chamfer")
        description = "R2 corner network plus an opposite distance-0.8 chamfer"
    elif case == "meeting_treatments":
        rounded = part.fillet(body, FLANGE_CORNER, 2, name="rounded_corner")
        result = part.chamfer(rounded, edge("flange-left", "flange-ExtrudeTop"), .7,
                              name="meeting_treatments")
        description = "Distance-0.7 top-rim chamfer terminating into the R2 corner network"
    else:
        raise ValueError(f"Unknown gallery case: {case}")

    volume = result.volume()
    if not result.is_watertight() or volume <= 0:
        raise ValueError(f"{case}: expected a watertight, positive-volume solid")
    return result, {"case": case, "description": description,
                    "before_volume_mm3": before, "volume_mm3": volume,
                    "volume_change_mm3": volume - before, "watertight": True}


def build_gallery(cases=SCULPTURES):
    set_progress_log(False)
    started = time.perf_counter()
    bounds = (350, 300, 130) if any(case in SCULPTURES for case in cases) else (210, 150, 70)
    part = Part((-50, -50, -15), bounds, tolerance=.04)
    gallery = part.assembly("Loft edge treatments")
    reports = []
    solids = []
    for index, case in enumerate(cases):
        # Each specimen owns its feature tree and its human-readable face names.
        specimen = Part((-50, -50, -15),
                        (350, 300, 130) if case in SCULPTURES else (210, 150, 70),
                        tolerance=.04)
        print(f"Building {case}", flush=True)
        case_started = time.perf_counter()
        solid, report = build_case(specimen, case)
        solid = part.copy_solid(solid, name="specimen_" + case)
        gallery.add_part(solid, position=((index % 2) * 88, (index // 2) * 78, 0))
        solids.append(solid)
        report["build_seconds"] = time.perf_counter() - case_started
        report["triangles"] = solid.triangle_count
        reports.append(report)
    return gallery, solids, {"build_seconds": time.perf_counter() - started,
                             "cases": reports}


def probe_corner(case):
    """Minimal uncaught reproductions: failures are neither skipped nor repaired."""
    set_progress_log(False)
    part = Part((-50, -50, -15), (350, 300, 130), tolerance=.04)
    if case in ("duct_rim_rounds", "duct_rim_chamfers"):
        shoe = part.extrude(rectangle(part, "shoe_profile", -9, 76, 68), 10, name="shoe")
        body = hollow_duct(part, shoe)
        operation = part.fillet if case == "duct_rim_rounds" else part.chamfer
        return operation(body, opening_rim(body, "duct-EndCap"), .6)
    if case == "enlarged_bounds_corner":
        return build_case(part, "meeting_treatments")[0]
    if case == "curved_four_edges":
        body = sculpted_loft(part, "vane", VANE_STATIONS)
        edges = longitudinal_edges("vane", loft=True)
        return part.fillet(body, edges, .8)
    if case in ("curved_round", "curved_chamfer"):
        body = sculpted_loft(part, "vane", VANE_STATIONS)
        operation = part.fillet if case == "curved_round" else part.chamfer
        return operation(body, edge("vane-Side-bottom", "vane-Side-right"), .6)
    if case == "inward_chamfer_network":
        body = part.extrude(rectangle(part, "block_profile", -10, 60, 44), 14, name="block")
        cutter = part.extrude(rectangle(part, "pocket_profile", -5, 16, 12), 15, name="pocket")
        body = part.cut(body, cutter)
        return part.chamfer(body, [edge("pocket-bottom", "pocket-ExtrudeBottom"),
                                   edge("pocket-left", "pocket-ExtrudeBottom"),
                                   edge("pocket-bottom", "pocket-left")], .6)
    body = housing(part)
    if case == "tapered_round_corner":
        return part.fillet(body, TAPERED_CORNER, 1)
    if case == "tapered_chamfer_corner":
        return part.chamfer(body, TAPERED_CORNER, 1)
    if case == "pocket_round":
        cutter = part.extrude(rectangle(part, "pocket_profile", 15, 14, 10), 20,
                              name="pocket")
        body = part.cut(body, cutter)
        return part.fillet(body, edge("pocket-bottom", "pocket-ExtrudeBottom"), 1)
    if case == "flange_chamfer_corner":
        return part.chamfer(flange(part, body), FLANGE_CORNER, .7)
    if case == "mixed_corner":
        body = notch(part, body)
        return part.fillet(body, [INSIDE_EDGE, edge("notch-bottom", "housing-EndCap"),
                                  edge("notch-left", "housing-EndCap")], 1)
    raise ValueError(f"Unknown probe: {case}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coupons", action="store_true", help="show the original small diagnostic specimens")
    parser.add_argument("--case", choices=CASES, help="show just one specimen")
    parser.add_argument("--probe", choices=PROBES, help="run an open corner reproduction")
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--render", type=Path, help="save existing multiview inspection output")
    parser.add_argument("--checker", action=argparse.BooleanOptionalAction, default=True,
                        help="show UV checkerboard (on by default)")
    args = parser.parse_args()
    if args.probe:
        probe_corner(args.probe)
        print(f"The {args.probe} probe completed successfully.")
        return
    gallery, _, report = build_gallery((args.case,) if args.case else COUPONS if args.coupons else SCULPTURES)
    print(json.dumps(report, indent=2))
    print("Gallery order: left to right, then the next row in +Y.")
    print("Connected mixed-convexity junctions and complete twisted-rim treatments remain unsupported; see the companion .md.")
    hues = ((.12, .32, .95), (.90, .035, .025), (.36, .16, .80),
            (.90, .035, .025), (.98, .78, .04), (.95, .40, .025),
            (.15, .60, .08), (.12, .32, .95))
    palette = {"*": (.45, .65, .95),
               **{f"*specimen_{case}:*": hues[i % len(hues)] for i, case in enumerate(CASES)},
               "*Chamfer*": (.98, .85, .22), "*Blend*": (.16, .95, .40)}
    if args.render:
        render_views(gallery, args.render, individual=True, colors=palette, checker=args.checker, views={
            "Isometric": ((1, -1, .85), (0, 0, 1)),
            "Top": ((0, 0, 1), (0, 1, 0)),
            "Front": ((0, -1, 0), (0, 0, 1)),
        })
    if args.show if args.show is not None else not args.render:
        show(gallery, title="Sculpted lofts and edge treatments", colors=palette, checker=args.checker)


if __name__ == "__main__":
    main()
