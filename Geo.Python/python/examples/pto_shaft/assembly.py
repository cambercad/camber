"""Photo-referenced DEMA 67991 PTO assembly; run to open Camber's viewer."""
import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import camber
from camber import Frame, Part, set_progress_log

from assumptions import (
    COLLAPSED_LENGTH,
    CROSS_SPACING,
    EXTENDED_LENGTH,
    INNER_PROFILE_ACROSS_CORNERS,
    INNER_PROFILE_LENGTH,
    MIN_VISIBLE_ENGAGEMENT,
    PROFILE_ACROSS_CORNERS,
    PROFILE_CONNECTION_OFFSET,
    PROFILE_WALL,
    TELESCOPIC_TRAVEL,
    OUTER_PROFILE_LENGTH,
)
from parts import (
    build_bearing_cup,
    build_chain_link,
    build_cross,
    build_grease_nipple,
    build_guard_bell,
    build_guard_label,
    build_guard_tube,
    build_tube,
    build_yoke,
)

set_progress_log(False)

_SQRT_HALF = math.sqrt(.5)
IDENTITY = (0, 0, 0, 1)
FORK_CLOCK = (0, 0, _SQRT_HALF, _SQRT_HALF)
TO_REVERSED_AXIS = (0, 1, 0, 0)
TO_X = (0, _SQRT_HALF, 0, _SQRT_HALF)
TO_NEG_X = (0, -_SQRT_HALF, 0, _SQRT_HALF)
TO_Y = (-_SQRT_HALF, 0, 0, _SQRT_HALF)
TO_NEG_Y = (_SQRT_HALF, 0, 0, _SQRT_HALF)

COLORS = {
    "*yoke*": (.56, .48, .25),
    "*cross*": (.78, .56, .22),
    "*profile*": (.36, .38, .40),
    "*guard*": (.96, .65, .025),
    "*bell*": (.96, .65, .025),
    "*cup*": (.70, .73, .75),
    "*grease*": (.68, .70, .72),
    "*chain*": (.65, .69, .72),
    "*button*": (.65, .69, .72),
    "*needle*": (.64, .67, .69),
    "*seal*": (.12, .13, .14),
    "*warning_label*": (.94, .94, .85),
    "*warning_header*": (.77, .09, .07),
}


def _axis(occurrence, solid, feature):
    # A journal split by a union has several coaxial faces. Select its current
    # named cylindrical face, not the now-ambiguous pre-boolean ancestor.
    names = [p for p in solid.patch_names if p.split(":")[-1].startswith(feature)]
    if not names:
        raise ValueError("Missing cylindrical mating face: " + feature)
    return occurrence.axis(names[0])


def _tube_shoulder(occurrence, solid, profile):
    # Depending on tube size, the plug covers all or only some of the tube's
    # original end face. Resolve the surviving named annular shoulder.
    for feature in (profile + "_outer_skin-ExtrudeBottom",
                    profile + "_plug-ExtrudeBottom"):
        names = [p for p in solid.patch_names if p.split(":")[-1].startswith(feature)]
        if names:
            return occurrence.plane(names[0])
    raise ValueError("Missing tube shoulder on " + profile)


def _bearing_assembly(part, solids, name="needle_bearing"):
    """One purchased bearing cartridge, reused at all eight journal ends."""
    bearing = part.assembly(name)
    bearing.solve_after_every_constraint = False
    cup = bearing.add_part(solids["bearing_cup"])
    bearing.fix(cup)
    seed = bearing.add_part(solids["needle"], (8.8, 0, 2))
    bearing.concentric(seed.axis("bearing_needle-Circle1"), cup.axis_at((8.8, 0, 0), (0, 0, 1)))
    bearing.coincident(seed.point_at((0, 0, 0)), cup.point_at((8.8, 0, 2)))
    bearing.pattern_circular(seed, 24)
    seal = bearing.add_part(solids["bearing_seal"], (0, 0, .1))
    bearing.concentric(seal.axis("bearing_seal_outer-Circle1"), cup.axis("bearing_cup_outer-Circle1"))
    bearing.coincident(seal.point_at((0, 0, 0)), cup.point_at((0, 0, .1)))
    result = bearing.solve()
    if not result.converged or result.unsatisfied:
        raise RuntimeError("Bearing cartridge mates failed")
    return bearing


def _add_joint_hardware(assembly, solids, occurrences, prefix, center_z, bearing):
    cross = occurrences[prefix + "_cross"]
    cup_directions = (
        ((14, 0, center_z), TO_X), ((-14, 0, center_z), TO_NEG_X),
        ((0, 14, center_z), TO_Y), ((0, -14, center_z), TO_NEG_Y),
    )
    for index, (position, orientation) in enumerate(cup_directions):
        cartridge = assembly.add_subassembly(bearing, position, orientation)
        cup = occurrences["%s_cup_%d" % (prefix, index)] = cartridge.parts[0]
        journal = "universal_cross_journal_" + ("x" if index < 2 else "y") + "-Circle1"
        assembly.concentric(_axis(cross, solids["cross"], journal),
                            _axis(cup, solids["bearing_cup"], "bearing_cup_outer-Circle1"))
        local = (position[0], position[1], 0)
        assembly.coincident(cup.point_at((0, 0, 0)), cross.point_at(local))

    # Grease fittings are visible service details on each cross center.
    occurrences[prefix + "_grease"] = assembly.add_part(
        solids["grease_nipple"], (0, 0, center_z + 6))


def _add_chain(assembly, solid, occurrences, output_center):
    # Equal arc-length stations on a hanging parabola; alternating link planes.
    # One restraint chain hangs between the two bell attachments.
    sag = 180.0
    start = 72.0
    for _ in range(5):
        slope = -4 * sag / (output_center - 2 * start)
        start = 66 + 11 / math.sqrt(1 + slope * slope)
    end = output_center - start
    first_y = 11 * slope / math.sqrt(1 + slope * slope)
    samples, distances = [], [0.0]
    for i in range(1001):
        t = i / 1000
        point = (39, first_y - 4 * sag * t * (1 - t), start + (end - start) * t)
        if samples:
            distances.append(distances[-1] + math.dist(point, samples[-1]))
        samples.append(point)
    count = round(distances[-1] / 18) + 1
    if count % 2 == 0:
        count += 1
    station = 0
    for i in range(count):
        distance = distances[-1] * i / (count - 1)
        while station < 999 and distances[station + 1] < distance:
            station += 1
        f = (distance - distances[station]) / (distances[station + 1] - distances[station])
        point = tuple(a + f * (b - a) for a, b in zip(samples[station], samples[station + 1]))
        dy = samples[station + 1][1] - samples[station][1]
        dz = samples[station + 1][2] - samples[station][2]
        length = math.hypot(dy, dz)
        phi = math.atan2(dz, dy)
        s, c = math.sin(phi / 2) * _SQRT_HALF, math.cos(phi / 2) * _SQRT_HALF
        q = (s, -s, c, c)
        if i % 2:
            x, y, z, w = q
            q = ((x + w) * _SQRT_HALF, (y + z) * _SQRT_HALF,
                 (z - y) * _SQRT_HALF, (w - x) * _SQRT_HALF)
        occurrences["chain_link_%02d" % i] = assembly.add_part(solid, point, q)


def build(extended=False, cutaway=False):
    """Return the assembled showcase, named part definitions, and solve result."""
    travel = TELESCOPIC_TRAVEL if extended else 0.0
    output_center = CROSS_SPACING + travel
    part = Part((-150, -250, -100), (150, 150, output_center + 100), tolerance=.025)

    solids = {
        "input_yoke": build_yoke(part, "input_spline_yoke", -1, "female"),
        "input_tube_yoke": build_yoke(part, "input_tube_yoke", 1, "tube"),
        "output_tube_yoke": build_yoke(part, "output_tube_yoke", -1, "tube"),
        "output_yoke": build_yoke(part, "output_spline_yoke", 1, "female"),
        "cross": build_cross(part, "universal_cross"),
        "outer_profile": build_tube(part, "outer_profile", OUTER_PROFILE_LENGTH,
                                     PROFILE_ACROSS_CORNERS, PROFILE_WALL),
        "inner_profile": build_tube(part, "inner_profile", INNER_PROFILE_LENGTH,
                                     INNER_PROFILE_ACROSS_CORNERS, PROFILE_WALL),
        "bearing_cup": build_bearing_cup(part, "bearing_cup"),
        "guard_outer": build_guard_tube(part, "guard_outer", 450, 30, 2, cutaway),
        "guard_inner": build_guard_tube(part, "guard_inner", 450, 27.5, 2, cutaway),
        "guard_bell": build_guard_bell(part, "guard_bell"),
        "grease_nipple": build_grease_nipple(part, "grease_nipple"),
        "chain_link": build_chain_link(part, "safety_chain_link"),
        "needle": part.cylinder((0, 0, 0), 1, 14, name="bearing_needle"),
        "bearing_seal": build_guard_tube(part, "bearing_seal", 1.2, 9.75, 1.9),
        "warning_label": build_guard_label(part, "warning_label", 78, 30.25),
        "warning_header": build_guard_label(part, "warning_header", 12, 30.55),
    }
    pin = part.cylinder((-27, 0, 0), 3.8, 54, name="locking_button_pin", axis="x")
    head = part.cylinder((-30, 0, 0), 6, 3, name="locking_button_head", axis="x")
    solids["locking_button"] = pin + head
    for component, solid in list(solids.items()):
        if solid.name.startswith("Boolean"):
            solids[component] = part.copy_solid(solid, component)
    for name, solid in solids.items():
        try:
            if not solid.is_watertight() or solid.volume() <= 0:
                raise ValueError("Not a closed positive-volume solid")
        except Exception as error:
            raise RuntimeError("Invalid PTO component: " + name) from error

    assembly = part.assembly("DEMA_67991_reconstruction")
    assembly.solve_after_every_constraint = False
    occurrences = {}

    # The four yokes face toward their respective cross; the central profiles
    # telescope between their tube-side hubs.
    occurrences["input_yoke"] = assembly.add_part(solids["input_yoke"])
    occurrences["input_tube_yoke"] = assembly.add_part(
        solids["input_tube_yoke"], (0, 0, 0), FORK_CLOCK)
    occurrences["input_cross"] = assembly.add_part(solids["cross"])
    occurrences["output_tube_yoke"] = assembly.add_part(
        solids["output_tube_yoke"], (0, 0, output_center), FORK_CLOCK)
    occurrences["output_yoke"] = assembly.add_part(
        solids["output_yoke"], (0, 0, output_center), IDENTITY)
    occurrences["output_cross"] = assembly.add_part(
        solids["cross"], (0, 0, output_center))

    occurrences["outer_profile"] = assembly.add_part(
        solids["outer_profile"], (0, 0, PROFILE_CONNECTION_OFFSET))
    occurrences["inner_profile"] = assembly.add_part(
        solids["inner_profile"], (0, 0, output_center - PROFILE_CONNECTION_OFFSET),
        TO_REVERSED_AXIS)

    # Full shields by default. Inspection windows are explicitly optional.
    occurrences["guard_outer"] = assembly.add_part(solids["guard_outer"],
                                                    (0, 0, 70))
    occurrences["guard_inner"] = assembly.add_part(
        solids["guard_inner"], (0, 0, output_center - 70),
        TO_REVERSED_AXIS)
    if not cutaway:
        occurrences["warning_label"] = assembly.add_part(solids["warning_label"], (0, 0, 280))
        occurrences["warning_header"] = assembly.add_part(solids["warning_header"], (0, 0, 280))
    occurrences["input_bell"] = assembly.add_part(solids["guard_bell"])
    occurrences["output_bell"] = assembly.add_part(
        solids["guard_bell"], (0, 0, output_center),
        (1, 0, 0, 0))
    occurrences["input_button"] = assembly.add_part(solids["locking_button"], (0, 17, -61))
    occurrences["output_button"] = assembly.add_part(solids["locking_button"], (0, 17, output_center + 61))

    bearing = _bearing_assembly(part, solids)
    _add_joint_hardware(assembly, solids, occurrences, "input", 0, bearing)
    _add_joint_hardware(assembly, solids, occurrences, "output", output_center, bearing)
    _add_chain(assembly, solids["chain_link"], occurrences, output_center)

    assembly.fix(occurrences["input_yoke"])
    for prefix in ("input", "output"):
        for key, journal in ((prefix + "_yoke", "x"), (prefix + "_tube_yoke", "y")):
            source = "input_spline_yoke" if key == "input_yoke" else (
                "output_spline_yoke" if key == "output_yoke" else key)
            assembly.concentric(_axis(occurrences[key], solids[key], source + "_right_bearing_seat-Circle1"),
                                _axis(occurrences[prefix + "_cross"], solids["cross"], "universal_cross_journal_" + journal + "-Circle1"))
            assembly.coincident(occurrences[key].point_at((0, 0, 0)),
                                occurrences[prefix + "_cross"].point_at((0, 0, 0)))
    # Mate the actual circular pilot/seat edges, as in a conventional shaft fit.
    assembly.concentric(
        occurrences["input_tube_yoke"].axis("input_tube_yoke_tube_seat-Circle1"),
        occurrences["outer_profile"].axis("outer_profile_pilot-Circle1"))
    assembly.concentric(
        occurrences["output_tube_yoke"].axis("output_tube_yoke_tube_seat-Circle1"),
        occurrences["inner_profile"].axis("inner_profile_pilot-Circle1"))
    assembly.distance(_tube_shoulder(occurrences["outer_profile"], solids["outer_profile"], "outer_profile"),
                      occurrences["input_tube_yoke"].plane("input_tube_yoke_hub-ExtrudeTop"), .1)
    assembly.distance(_tube_shoulder(occurrences["inner_profile"], solids["inner_profile"], "inner_profile"),
                      occurrences["output_tube_yoke"].plane("output_tube_yoke_hub-ExtrudeBottom"), .1)
    assembly.concentric(occurrences["outer_profile"].axis("outer_profile_pilot-Circle1"),
                        occurrences["inner_profile"].axis("inner_profile_pilot-Circle1"))
    assembly.distance(occurrences["input_cross"].point_at((0, 0, 0)),
                      occurrences["output_cross"].point_at((0, 0, 0)), output_center)

    # The protective sleeve floats around the drive steel, but its end is
    # retained by the bell. Mate its cylindrical support and axial location.
    for side, sleeve, profile in (("input", "guard_outer", "outer_profile"),
                                  ("output", "guard_inner", "inner_profile")):
        assembly.concentric(_axis(occurrences[sleeve], solids[sleeve], sleeve + "_outer-Circle1"),
                            _axis(occurrences[profile], solids[profile], profile + "_pilot-Circle1"))
        assembly.coincident(occurrences[sleeve].point_at((0, 0, 0)),
                            occurrences[profile].point_at((0, 0, 26)))
        bell = side + "_bell"
        assembly.concentric(_axis(occurrences[bell], solids["guard_bell"], "guard_bell-wall_6"),
                            _axis(occurrences[sleeve], solids[sleeve], sleeve + "_outer-Circle1"))
        assembly.coincident(occurrences[bell].point_at((0, 0, 80)),
                            occurrences[sleeve].point_at((0, 0, 10)))

    if not cutaway:
        for label in ("warning_label", "warning_header"):
            assembly.concentric(_axis(occurrences[label], solids[label], label + "_outer-Circle1"),
                                _axis(occurrences["guard_outer"], solids["guard_outer"],
                                      "guard_outer_outer-Circle1"))
            assembly.coincident(occurrences[label].point_at((0, 0, 0)),
                                occurrences["guard_outer"].point_at((0, 0, 210)))

    solution = assembly.solve()
    if not solution.converged or solution.unsatisfied:
        raise RuntimeError("PTO showcase mates did not converge: %r" % (solution,))
    return assembly, solids, occurrences, solution


def build_joint_detail(solids):
    """Show the actual cup and cross hardware with the protective bell removed."""
    # The cartridge's circular pattern needs its solids registered in this
    # source Part. A second Part cannot pattern them across kernel sessions.
    detail_part = solids["cross"]._part
    detail = detail_part.assembly("PTO_universal_joint_detail")
    detail.solve_after_every_constraint = False
    joints = {
        "input_yoke": detail.add_part(solids["input_yoke"]),
        "input_tube_yoke": detail.add_part(solids["input_tube_yoke"], (0, 0, 0), FORK_CLOCK),
        "input_cross": detail.add_part(solids["cross"]),
        "input_button": detail.add_part(solids["locking_button"], (0, 17, -61)),
    }
    bearing = _bearing_assembly(detail_part, solids, "needle_bearing_detail")
    _add_joint_hardware(detail, solids, joints, "input", 0, bearing)
    detail.fix(joints["input_yoke"])
    for key, direction in (("input_yoke", "x"), ("input_tube_yoke", "y")):
        source = "input_spline_yoke" if key == "input_yoke" else key
        detail.concentric(_axis(joints[key], solids[key], source + "_right_bearing_seat-Circle1"),
                           _axis(joints["input_cross"], solids["cross"],
                                 "universal_cross_journal_" + direction + "-Circle1"))
        detail.coincident(joints[key].point_at((0, 0, 0)),
                          joints["input_cross"].point_at((0, 0, 0)))
    result = detail.solve()
    if not result.converged or result.unsatisfied:
        raise RuntimeError("Joint detail mates did not converge")
    return detail


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extended", action="store_true", help="show the extended 1100 mm pose")
    parser.add_argument("--cutaway", action="store_true", help="open the sleeve inspection windows")
    parser.add_argument("--no-show", action="store_true", help="build and render without opening a window")
    parser.add_argument("--no-render", action="store_true", help="skip the image capture")
    parser.add_argument("--detail", action="store_true", help="open the bare universal joint")
    parser.add_argument("--obj", action="store_true", help="save the complete assembly as one named-object OBJ")
    args = parser.parse_args()

    start = time.perf_counter()
    assembly, solids, occurrences, solution = build(args.extended, args.cutaway)
    length = EXTENDED_LENGTH if args.extended else COLLAPSED_LENGTH
    print("PTO showcase: %d mm, %d part definitions, %d occurrences, %d mates (%.1f s)" %
          (length, len(solids), len(assembly.leaves()), len(assembly.constraints),
           time.perf_counter() - start))

    if args.obj:
        obj_path = os.path.join(_HERE, "pto_shaft.obj")
        assembly.save_obj(obj_path)
        print("Assembly OBJ: " + obj_path)

    if not args.no_render:
        image_path = os.path.join(_HERE, "pto_shaft_showcase.png")
        camber.render_views(assembly, image_path,
                            views={"Hero": ((1, .3, .28), (0, 1, 0))},
                            tile_size=(1800, 1100),
                            colors=COLORS, checker=False)
        print("Showcase image: " + image_path)
        detail = build_joint_detail(solids)
        detail_path = os.path.join(_HERE, "pto_joint_detail.png")
        camber.render_views(detail, detail_path,
                            views={"Joint": ((1, -1, .8), (0, 0, 1)),
                                   "Socket": ((0, 0, -1), (0, 1, 0))},
                            tile_size=(950, 850), columns=2,
                            colors=COLORS, checker=False)
        print("Joint detail: " + detail_path)
    if not args.no_show:
        camber.show(build_joint_detail(solids) if args.detail else assembly,
                    title="Camber — DEMA 67991 PTO reconstruction",
                    colors=COLORS, checker=False)


if __name__ == "__main__":
    main()
