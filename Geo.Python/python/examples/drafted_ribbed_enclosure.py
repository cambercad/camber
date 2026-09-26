"""Drafted sensor enclosure with ribs and real fastener seats.

This compact part combines the new face-draft, rib, drilled-hole,
counterbore, and countersink operations. Dimensions are in millimetres;
draft and countersink angles are in radians.
"""
import math

from camber import Frame, Part


def build():
    """Return a finished, watertight enclosure ready for display."""
    draft_angle = math.radians(3)
    part = Part((-42, -32, -6), (42, 32, 38), tolerance=.01)

    # The base flange and raised housing overlap slightly so their union has
    # a genuine shared volume. Draft the four walls from the fixed bottom rim.
    flange = part.cuboid((-35, -25, 0), (35, 25, 4), name="mounting_flange")
    blank = part.cuboid((-29, -19, 3.8), (29, 19, 31.8), name="housing_blank")
    side_faces = [name for name in blank.patch_names if "-Line" in name]
    housing = blank.draft_faces(
        side_faces, Frame(origin=(0, 0, 3.8)), draft_angle,
        name="drafted_housing")

    # A straight pocket opens the top while leaving a 2.2 mm floor and robust
    # walls. Its smaller footprint accounts for the drafted outside walls.
    cavity = part.cuboid((-26, -16, 6), (26, 16, 32.2), name="cavity_tool")
    housing = part.subtract(housing, cavity, name="hollow_housing")
    housing = part.union(housing, flange, name="flanged_housing")

    # Two intersecting stiffeners rise from the cavity floor. Starting 0.1 mm
    # below the floor gives the union a small, deliberate overlap.
    rib_frame = Frame(origin=(0, 0, 5.9))
    housing = housing.rib([(-21, 0), (21, 0)], 1.4, 17,
                       frame=rib_frame, name="longitudinal_rib")
    housing = housing.rib([(0, -11), (0, 11)], 1.4, 13,
                       frame=rib_frame, name="cross_rib")

    # Four hollow screw bosses support the electronics board.
    boss_centres = [(-19, -9), (-19, 9), (19, -9), (19, 9)]
    for index, (x, y) in enumerate(boss_centres, 1):
        boss = part.cylinder((x, y, 5.9), 3.2, 7.1,
                             name="board_boss_{}".format(index))
        housing = part.union(housing, boss,
                             name="housing_with_bosses_{}".format(index))

    # Alternate plain clearance bores and counterbores in the bosses.
    down = (0, 0, -1)
    for index, (x, y) in enumerate(boss_centres):
        mouth = Frame(origin=(x, y, 13), x=(1, 0, 0), y=(0, -1, 0), z=down)
        if index < 2:
            housing = housing.hole(mouth, 2.2, depth=7.2,
                                name="board_clearance_{}".format(index + 1))
        else:
            housing = housing.counterbore_hole(
                mouth, 2.2, 4.6, 1.2, depth=7.2,
                name="board_counterbore_{}".format(index + 1))

    # Countersunk through holes in the flange accept flat-head mounting screws.
    for index, (x, y) in enumerate(((-31.5, -21.5), (-31.5, 21.5),
                                    (31.5, -21.5), (31.5, 21.5)), 1):
        mouth = Frame(origin=(x, y, 4), x=(1, 0, 0), y=(0, -1, 0), z=down)
        housing = housing.countersink_hole(
            mouth, 3.2, 6.0, math.radians(90),
            name="mount_countersink_{}".format(index))

    assert housing.is_watertight()
    assert housing.volume() > 0
    return housing


if __name__ == "__main__":
    build().show(title="Drafted, ribbed sensor enclosure")
