"""Short constrained polyline — GeoScriptViewer/BikeA.py.

Three construction segments with perpendicular / angle constraints.
"""
from camber import Part, vec3

part = Part(vec3(-100, -80, -20), vec3(100, 80, 40), tolerance=0.01)

sk = part.sketch("xy", constrained=True, name="Sketch1")
sk.solve_after_every_constraint = False
line1 = sk.add_line((-0.4497, -0.2755), (-0.4496, 0.5326), name="line1", construction=True)
line2 = sk.add_line((-0.4496, 0.5326), (0.3355, 0.5326), name="line2", construction=True)
sk.horizontal(line2)
sk.coincident(line1 @ 1.000, line2 @ 0.000)
line3 = sk.add_line((0.3355, 0.5326), (0.6002, -0.232), name="line3", construction=True)
sk.coincident(line2 @ 1.000, line3 @ 0.000)
line4 = sk.add_line((0.6002, -0.232), (-0.3607, -0.5647), name="line4", construction=True)
sk.coincident(line3 @ 1.000, line4 @ 0.000)
sk.perpendicular(line3, line4)
sk.angle(line1, line2, 92)
sk.solve()

print(sk)
sk.show(title="bike A")
