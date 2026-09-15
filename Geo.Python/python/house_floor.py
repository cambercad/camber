"""L-shaped EFH with an Erker — wall centerlines, then offset.

    y=10.5  +----------------+
            |    Schlafen    |
    y=6.6   +--------+-------+
            | Wohnen |  Bad  |
    y=4.2   |        +--+----+----+
            |        |Diele | Kü |
    y=0     +--Erker-+------+----+
           x=0  2  5.2     8   13.2

Heights (metres), typical Swiss residential:
  * 2.50 m lichte Raumhöhe — SIA 2024 Wohnen (MFH/EFH)
  * 0.25 m Stahlbetondecke — usual massive slab (20–30 cm)
Cantonal minima are often 2.40 m clear; new-build practice is 2.50–2.60 m.
"""
from camber import Frame, Part, vec3

wall = 0.30
half = 0.5 * wall
room_height = 2.50
slab = 0.25

bay_x0 = 2.0
bay_x1 = 5.2
bay_depth = 1.4
kitchen_x = 13.2
notch_x = 8.0
kitchen_y = 4.2
sleep_y = 6.6
north_y = 10.5

part = Part(vec3(-4, -4, -2), vec3(18, 14, 5), tolerance=0.001)


def extrude_walls(name, segments):
    sk = part.sketch("xy", constrained=True, name=name)
    sk.solve_after_every_constraint = False
    curves = [
        sk.add_line(a, b, name=n, construction=True) for n, a, b in segments
    ]
    sk.offset(curves, half, side="both", join="miter", end_cap="square")
    return part.extrude(sk, room_height, name=name)


sk = part.sketch("xy", constrained=True, name="shell")
sk.solve_after_every_constraint = False

# Rough clicks — solver snaps these to the dimensions below.
sw = sk.add_line((0.3, -0.2), (1.8, 0.2), name="sw", construction=True)
bay_w = sk.add_line((2.1, -0.1), (1.8, -1.2), name="bay_w", construction=True)
bay_s = sk.add_line((2.2, -1.5), (5.0, -1.3), name="bay_s", construction=True)
bay_e = sk.add_line((5.3, -1.3), (5.1, 0.2), name="bay_e", construction=True)
se = sk.add_line((5.4, 0.2), (12.9, -0.2), name="se", construction=True)
east = sk.add_line((13.4, 0.1), (13.1, 4.0), name="east", construction=True)
notch = sk.add_line((13.0, 4.4), (8.2, 4.0), name="notch", construction=True)
stem = sk.add_line((8.2, 4.3), (7.8, 10.3), name="stem", construction=True)
north = sk.add_line((7.8, 10.7), (0.2, 10.3), name="north", construction=True)
west = sk.add_line((-0.2, 10.4), (0.3, 0.2), name="west", construction=True)

sk.horizontal(sw)
sk.horizontal(bay_s)
sk.horizontal(se)
sk.horizontal(notch)
sk.horizontal(north)
sk.vertical(west)
sk.vertical(east)
sk.vertical(bay_w)
sk.vertical(bay_e)
sk.vertical(stem)

sk.coincident(sw @ 1.000, bay_w @ 0.000)
sk.coincident(bay_w @ 1.000, bay_s @ 0.000)
sk.coincident(bay_s @ 1.000, bay_e @ 0.000)
sk.coincident(bay_e @ 1.000, se @ 0.000)
sk.coincident(se @ 1.000, east @ 0.000)
sk.coincident(east @ 1.000, notch @ 0.000)
sk.coincident(notch @ 1.000, stem @ 0.000)
sk.coincident(stem @ 1.000, north @ 0.000)
sk.coincident(north @ 1.000, west @ 0.000)
sk.coincident(west @ 1.000, sw @ 0.000)

sk.coincident(sw @ 0.000, sk @ "origin")
sk.length(west, north_y)
sk.length(north, notch_x)
sk.length(sw, bay_x0)
sk.length(bay_s, bay_x1 - bay_x0)
sk.length(bay_w, bay_depth)
sk.length(bay_e, bay_depth)
sk.length(se, kitchen_x - bay_x1)
sk.length(east, kitchen_y)
sk.solve()

sk.offset(
    [sw, bay_w, bay_s, bay_e, se, east, notch, stem, north, west],
    half,
    side="both",
    join="miter",
    end_cap="square",
)
shell = part.extrude(sk, room_height, name="shell")

partitions = extrude_walls("partitions", [
    ("kit", (notch_x, 0.0), (notch_x, kitchen_y)),
    ("sleep", (0.0, sleep_y), (notch_x, sleep_y)),
    ("diele", (bay_x1, 0.0), (bay_x1, sleep_y)),
    ("bath", (bay_x1, kitchen_y), (notch_x, kitchen_y)),
])

# Bodenplatte: closed centerline, offset out by half the wall to the outer face.
slab_pts = (
    (0.0, 0.0),
    (bay_x0, 0.0),
    (bay_x0, -bay_depth),
    (bay_x1, -bay_depth),
    (bay_x1, 0.0),
    (kitchen_x, 0.0),
    (kitchen_x, kitchen_y),
    (notch_x, kitchen_y),
    (notch_x, north_y),
    (0.0, north_y),
    (0.0, 0.0),
)
slab_sk = part.sketch(
    constrained=True, name="slab", frame=Frame(origin=(0, 0, -slab)))
slab_sk.solve_after_every_constraint = False
slab_edges = []
for i in range(len(slab_pts) - 1):
    slab_edges.append(slab_sk.add_line(
        slab_pts[i], slab_pts[i + 1], name="slab{0}".format(i), construction=True))
slab_sk.offset(slab_edges, half, side="out", join="miter")
ground = part.extrude(slab_sk, slab, name="ground")

# Shell+slab first; a pre-union of all walls then fails against the slab.
base = part.union(shell, ground, name="base")
house = part.union(base, partitions, name="house")
print(house)
house.show(title="Swiss EFH")
