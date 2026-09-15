"""Constrained profile swept along a circular section, then a rectangular pocket.

Same construction as GeoScriptViewer/TestScript.py: a helper rectangle, filleted
arcs, a construction circle, then ExtrudeAlongSketch and a through-cut.
"""
import math

from camber import Part, vec3

part = Part(vec3(-0.5), vec3(60), tolerance=1e-4)

sk = part.sketch("xy", constrained=True, name="profile")

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

arc1 = sk.add_arc((0.0955, 35.2428), (2.4737, 39.2999), (6.6705, 41.8181), name="arc1")
sk.point_on_line(arc1 @ 0.000, line_d)
sk.point_on_line(arc1 @ 1.000, line_c)
sk.tangent(line_d, arc1)
sk.tangent(line_c, arc1)
sk.radius(arc1, 9.5)
sk.solve()

arc2 = sk.add_arc((0.0654, -0.8970), (9.0932, -8.9771), (19.6993, -2.4436), name="arc2")
sk.point_on_line(arc2 @ 0.000, line_d)
sk.tangent(line_a, arc2)
sk.tangent(line_d, arc2)
sk.coincident(arc2 @ 0.000, sk @ "origin")
sk.solve()

line1 = sk.add_line((0.9363, 2.5162), (1.0904, 30.5633), name="line1")
sk.coincident(line1 @ 0.000, arc2 @ 0.000)
sk.coincident(arc1 @ 0.000, line1 @ 1.000)
sk.solve()

circle1 = sk.add_circle((21.9092, 28.1571), 5.0, name="circle1", construction=True)
sk.distance_to_line(circle1 @ "center", line_b, 6.35)
sk.distance_to_line(circle1 @ "center", line_c, 12.85)
sk.tangent(line_b, circle1)
sk.solve()

line2 = sk.add_line(arc2 @ 1.000, (25.7499, 25.3673), name="line2")
sk.coincident(line2 @ 0.000, arc2 @ 1.000)
sk.tangent(line2, circle1)
sk.point_on_circle(line2 @ 1.000, circle1)
sk.solve()

arc3 = sk.add_arc((10.4586, 41.4931), (19.3024, 38.7611), (24.0021, 34.9367), name="arc3")
sk.coincident(arc1 @ 1.000, arc3 @ 0.000)
sk.point_on_circle(arc3 @ 1.000, circle1)
sk.tangent(line_c, arc3)
sk.tangent_circles(arc3, circle1)
sk.solve()

arc4 = sk.add_arc((27.0126, 32.9397), (29.0334, 30.0767), (28.2475, 27.4382), name="arc4")
sk.coincident(arc3 @ 1.000, arc4 @ 0.000)
sk.coincident(arc4 @ 1.000, line2 @ 1.000)
sk.equal(arc4, circle1)
sk.solve()

radius = 0.5 * 4.75
s2 = part.sketch("zx", constrained=True, name="section")
circ = s2.add_circle((1.3136, 3.5581), radius, name="circ")
s2.point_on_line(circ @ "center", s2 @ "y")
s2.distance(s2 @ "origin", circ @ "center", radius)
s2.solve()

# Plotter copy of the solved profile (helpers stay on the constraint sketch).
# Arc4 is rebuilt from the construction circle so the guide is one connected strip.
def _pt(curve, u):
    return sk.eval_xy(curve @ u)


def _arc_mid_on_circle(start, end, center):
    cx, cy = center
    a0 = math.atan2(start[1] - cy, start[0] - cx)
    a1 = math.atan2(end[1] - cy, end[0] - cx)
    delta = a1 - a0
    while delta <= -math.pi:
        delta += 2.0 * math.pi
    while delta > math.pi:
        delta -= 2.0 * math.pi
    mid = a0 + 0.5 * delta
    r = math.hypot(start[0] - cx, start[1] - cy)
    return (cx + r * math.cos(mid), cy + r * math.sin(mid))


center = sk.eval_xy(circle1 @ "center")
guide = part.sketch("xy", name="sweep_guide")
guide.add_line(_pt(line1, 0.000), _pt(line1, 1.000), name="line1")
guide.add_arc(_pt(arc1, 0.000), _pt(arc1, 0.500), _pt(arc1, 1.000), name="arc1")
guide.add_arc(_pt(arc3, 0.000), _pt(arc3, 0.500), _pt(arc3, 1.000), name="arc3")
a4_start = _pt(arc3, 1.000)
a4_end = _pt(line2, 1.000)
guide.add_arc(a4_start, _arc_mid_on_circle(a4_start, a4_end, center), a4_end, name="arc4")
guide.add_line(a4_end, _pt(line2, 0.000), name="line2")
guide.add_arc(_pt(arc2, 1.000), _pt(arc2, 0.500), _pt(arc2, 0.000), name="arc2")

body = part.extrude_along_sketch(s2, guide, name="profile", max_deviation=0.003)

s_cut = part.sketch("xy", constrained=True, name="pocket")
s_cut.solve_after_every_constraint = False
cut_guide = s_cut.add_line(sk.eval_xy(line2 @ 0.000), sk.eval_xy(line2 @ 1.000), name="guide", construction=True)
s_cut.fix(cut_guide @ 0.000)
s_cut.fix(cut_guide @ 1.000)

south, east, north, west = s_cut.add_rectangle((12.5274, 9.5557), (17.5002, 17.3704))
s_cut.point_on_line(south @ 1.000, cut_guide)
s_cut.point_on_line(east @ 1.000, cut_guide)
s_cut.length(north, 2 * radius)
s_cut.vertical_distance(s_cut @ "origin", south @ 0.000, 7.5)
s_cut.length(east, 14.5)
s_cut.solve()

cutter = part.extrude_two_sides(s_cut, 20, 20, name="pocket", max_deviation=0.003)
result = body - cutter
print(result)
result.show(title="profile sweep")
