"""Drafted open-top electronics enclosure with internal standoffs and ports.

The housing and its matching cavity use tapered extrudes with opposite signed
draft angles. Remaining details use ordinary sketches, cylinders, and batched
CSG so the example also demonstrates a practical feature-based workflow.
All dimensions are millimetres; draft angles are radians in CamberCAD.
"""

import math

from camber import Frame, Part


def _octagon(sketch, width, depth, corner):
    """Draw a closed, clipped-rectangle profile in counter-clockwise order."""
    points = [
        (-width / 2 + corner, -depth / 2),
        ( width / 2 - corner, -depth / 2),
        ( width / 2, -depth / 2 + corner),
        ( width / 2,  depth / 2 - corner),
        ( width / 2 - corner,  depth / 2),
        (-width / 2 + corner,  depth / 2),
        (-width / 2,  depth / 2 - corner),
        (-width / 2, -depth / 2 + corner),
    ]
    for index, start in enumerate(points):
        sketch.add_line(start, points[(index + 1) % len(points)])


def build():
    """Return the finished enclosure as one watertight solid."""
    # Primary envelope and wall design parameters.
    base_width, base_depth, base_height = 92.0, 60.0, 2.1
    body_width, body_depth = 86.0, 56.0
    body_bottom_z, body_height = 2.0, 34.0
    wall, floor = 2.0, 4.0
    draft = math.radians(3.0)
    corner_clip = 7.0

    part = Part((-60, -45, -8), (60, 45, 45), tolerance=.02)

    # The small overlap between flange and body gives the union a proper
    # intersection instead of relying on two solids that only touch at a face.
    flange_profile = part.sketch("xy", frame=Frame((0, 0, 0)), name="base_flange_profile")
    _octagon(flange_profile, base_width, base_depth, corner_clip)
    flange = flange_profile.extrude(base_height, name="base_flange")

    body_profile = part.sketch("xy", frame=Frame((0, 0, body_bottom_z)), name="housing_profile")
    _octagon(body_profile, body_width, body_depth, corner_clip)
    body = body_profile.extrude(body_height, name="drafted_housing", taper_angle=draft)
    housing = part.batch_union((flange, body))

    # Follow the outer draft with an expanding downward cavity, leaving a
    # constant nominal side wall and a 4 mm floor. The slight top overrun makes
    # the opening unambiguous to CSG.
    top_z = body_bottom_z + body_height
    top_overrun = .1
    cavity_width = body_width - 2 * (body_height + top_overrun) * math.tan(draft) - 2 * wall
    cavity_depth = body_depth - 2 * (body_height + top_overrun) * math.tan(draft) - 2 * wall
    cavity_profile = part.sketch("xy", frame=Frame((0, 0, top_z + .1)), name="cavity_profile")
    # Both the draft and the wall inset are perpendicular offsets of each
    # polygon edge. For a 45-degree clipped corner, a mitered inward offset
    # changes the clip length by (sqrt(2) - 2) times the offset distance.
    cavity_offset = (body_height + top_overrun) * math.tan(draft) + wall
    cavity_clip = corner_clip + (math.sqrt(2) - 2) * cavity_offset
    _octagon(cavity_profile, cavity_width, cavity_depth, cavity_clip)
    cavity = cavity_profile.extrude(
        -(body_height - floor + top_overrun), taper_angle=-draft,
        name="drafted_cavity_tool")
    housing = part.subtract(housing, cavity, name="hollow_housing")

    # Four integral screw standoffs stand on the cavity floor. Their modest
    # height leaves room for a board below the opening and avoids tall posts.
    boss_centres = [(-31, -18), (-31, 18), (31, -18), (31, 18)]
    boss_top_z = body_bottom_z + floor + 12.0
    bosses = [
        part.cylinder((x, y, body_bottom_z + floor - .1), 4.0,
                      boss_top_z - (body_bottom_z + floor - .1),
                      name="screw_boss_{}".format(index + 1))
        for index, (x, y) in enumerate(boss_centres)
    ]
    housing = part.batch_union([housing, *bosses])

    cutters = []
    # Clearance bores pass through the flange, floor, and bosses. Short wider
    # bores at the boss tops are counterbores for the screw heads.
    for index, (x, y) in enumerate(boss_centres):
        cutters.append(part.cylinder((x, y, -.1), 1.7, boss_top_z + .2,
                                     name="M3_clearance_{}".format(index + 1)))
        cutters.append(part.cylinder((x, y, boss_top_z - 1.6), 3.0, 1.8,
                                     name="screw_counterbore_{}".format(index + 1)))

    # Shallow board-side details are cut into the inside floor, where they
    # remain visible in this open-top example.
    floor_top_z = body_bottom_z + floor
    display = part.sketch("xy", frame=Frame((0, 0, floor_top_z)), name="display_recess_profile")
    display.add_rectangle((-25, -10), (5, 10))
    cutters.append(display.extrude(-.8, name="display_recess_tool"))

    encoder = part.sketch("xy", frame=Frame((0, 0, floor_top_z)), name="encoder_profile")
    encoder.add_circle((22, 0), 5.5)
    cutters.append(encoder.extrude(-.8, name="encoder_seat_tool"))

    # Small through-floor vents, kept clear of the display recess and bosses.
    for index, (x, y) in enumerate((
            (x, y) for y in (-21.5, 21.5) for x in (-8, -2, 4, 10))):
        cutters.append(part.cylinder((x, y, body_bottom_z - .1), .65,
                                     floor + .2, name="floor_vent_{}".format(index + 1)))

    # A blind side-entry cable gland opens from the left wall into the cavity.
    cutters.append(part.cylinder((-44, 0, 15), 4.2, 9.0, axis="x", name="cable_entry_tool"))

    result = part.batch_subtract(housing, cutters, name="finished_enclosure")
    assert result.is_watertight()
    assert result.volume() > 0
    return result


if __name__ == "__main__":
    build().show(title="Drafted instrument enclosure")
