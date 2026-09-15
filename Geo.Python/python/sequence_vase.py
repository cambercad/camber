"""Dimensioned profile: two horizontals, a vertical on the Y-axis, origin-centered arc.

Uses the constrained sketcher the way a person would: draw the curves roughly,
then add horizontal / vertical / coincident / radius / distance and solve.

https://www.youtube.com/watch?v=GVW1zkMf8T8
"""
from camber import Part, vec3

top_height = 25.0
bottom_depth = 35.0
arc_radius = 40.0

part = Part(vec3(-80), vec3(80), tolerance=0.1)
sk = part.sketch("xy", constrained=True, name="profile")
sk.solve_after_every_constraint = False

# Rough clicks — the solver moves these to the dimensions below.
top = sk.add_line((2, 20), (28, 22), name="top")
rim = sk.add_arc((28, 22), (36, 2), (16, -28), name="rim")
bottom = sk.add_line((16, -28), (3, -30), name="bottom")
left = sk.add_line((3, -30), (2, 20), name="left")

sk.horizontal(top)
sk.horizontal(bottom)
sk.vertical(left)

sk.coincident(top @ 1.000, rim @ 0.000)
sk.coincident(rim @ 1.000, bottom @ 0.000)
sk.coincident(bottom @ 1.000, left @ 0.000)
sk.coincident(left @ 1.000, top @ 0.000)

sk.coincident(rim @ "center", sk @ "origin")
sk.radius(rim, arc_radius)

# Left edge through the origin (the Y-axis), then the two vertical dimensions.
sk.point_on_line(sk @ "origin", left)
sk.distance(top @ 0.000, sk @ "origin", top_height)
sk.distance(bottom @ 1.000, sk @ "origin", bottom_depth)

sk.solve()

print(sk)
sk.show(title="sequence vase")
