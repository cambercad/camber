"""Under-defined construction rectangle — GeoScriptViewer/TestScriptOrig.py.

Horizontal/vertical helper lines with a length on two sides and the origin on
the left edge. Opens the sketch viewer.
"""
from camber import Part, vec3

part = Part(vec3(-0.5), vec3(60), tolerance=1e-4)
sk = part.sketch("xy", constrained=True, name="rect")
sk.solve_after_every_constraint = False

line_a = sk.add_line((0, 0), (1, 0), name="lineA", construction=True)
line_b = sk.add_line((1, 0), (1, 1), name="lineB", construction=True)
line_c = sk.add_line((1, 1), (0, 1), name="lineC", construction=True)
line_d = sk.add_line((0, 1), (0, 0), name="lineD", construction=True)

sk.coincident(line_a @ 1.000, line_b @ 0.000)
sk.coincident(line_b @ 1.000, line_c @ 0.000)
sk.coincident(line_c @ 1.000, line_d @ 0.000)
sk.coincident(line_d @ 1.000, line_a @ 0.000)

sk.horizontal(line_a)
sk.length(line_a, 27.45)
sk.vertical(line_b)
sk.length(line_b, 50.75)
sk.horizontal(line_c)
sk.vertical(line_d)

sk.point_on_line(sk @ "origin", line_d)
sk.distance(sk @ "origin", line_a @ 0.000, 9)
sk.solve()

print(sk)
sk.show(title="construction rectangle")
