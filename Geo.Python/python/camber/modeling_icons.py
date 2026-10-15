"""Small vector icons for the modeling commands, drawn in the GUI batch."""

import math

from . import sketch_icons as s

# Shared geometric symbols keep sketch and assembly constraints consistent.
fix = s.draw_fix
coincident = s.draw_coincident
concentric = s.draw_concentric
parallel = s.draw_parallel
perpendicular = s.draw_perpendicular
angle = s.draw_angle
distance = s.draw_distance


def _edge(dl, x, y, a, b, ink, thickness=1.6):
    s._line(dl, s._pt(x, y, *a), s._pt(x, y, *b), ink, thickness)


def file_new(dl, x, y, ink, accent):
    for a, b in (((5, 3), (16, 3)), ((16, 3), (20, 7)), ((20, 7), (20, 21)),
                 ((20, 21), (5, 21)), ((5, 21), (5, 3))):
        _edge(dl, x, y, a, b, ink)
    _edge(dl, x, y, (9, 12), (17, 12), accent, 2)
    _edge(dl, x, y, (13, 8), (13, 16), accent, 2)


def file_open(dl, x, y, ink, accent):
    for a, b in (((3, 19), (3, 5)), ((3, 5), (9, 5)), ((9, 5), (12, 8)),
                 ((12, 8), (20, 8)), ((20, 8), (20, 11))):
        _edge(dl, x, y, a, b, ink)
    for a, b in (((3, 19), (7, 11)), ((7, 11), (22, 11)),
                 ((22, 11), (18, 19)), ((18, 19), (3, 19))):
        _edge(dl, x, y, a, b, accent)


def file_save(dl, x, y, ink, accent):
    for a, b in (((4, 3), (17, 3)), ((17, 3), (21, 7)), ((21, 7), (21, 21)),
                 ((21, 21), (4, 21)), ((4, 21), (4, 3))):
        _edge(dl, x, y, a, b, ink)
    _edge(dl, x, y, (8, 3), (8, 10), accent)
    _edge(dl, x, y, (8, 10), (17, 10), accent)
    _edge(dl, x, y, (17, 10), (17, 3), accent)
    _edge(dl, x, y, (8, 17), (17, 17), ink)


def redo(dl, x, y, ink, accent):
    s.draw_redo(dl, x, y, ink, accent)


def undo(dl, x, y, ink, accent):
    s.draw_undo(dl, x, y, ink, accent)


def block(dl, x, y, ink, accent):
    points = ((3, 8), (11, 4), (21, 8), (13, 12), (3, 8))
    for a, b in zip(points, points[1:]):
        _edge(dl, x, y, a, b, ink)
    for a, b in (((3, 8), (3, 18)), ((13, 12), (13, 22)), ((21, 8), (21, 18)),
                 ((3, 18), (13, 22)), ((13, 22), (21, 18))):
        _edge(dl, x, y, a, b, accent)


def extrude(dl, x, y, ink, accent):
    # Keep the direction arrow clear of the profile and its projected edges.
    for a, b in (((3, 14), (11, 10)), ((11, 10), (19, 14)),
                 ((19, 14), (11, 18)), ((11, 18), (3, 14)),
                 ((3, 14), (3, 18)), ((3, 18), (11, 22)),
                 ((11, 18), (11, 22)), ((11, 22), (19, 18)),
                 ((19, 18), (19, 14))):
        _edge(dl, x, y, a, b, ink)
    for a, b in (((19, 10), (19, 2)), ((16, 5), (19, 2)), ((19, 2), (22, 5))):
        _edge(dl, x, y, a, b, accent)


def offset_sketch(dl, x, y, ink, accent):
    s.draw_rectangle(dl, x, y, ink, accent)
    _edge(dl, x, y, (18, 18), (18, 3), accent, 2)
    _edge(dl, x, y, (15, 6), (18, 3), accent, 2)
    _edge(dl, x, y, (21, 6), (18, 3), accent, 2)


def loft(dl, x, y, ink, accent):
    s._arc(dl, x, y, 12, 5, 5, 2, 0, 360, ink)
    s._arc(dl, x, y, 12, 19, 9, 3, 0, 180, ink)
    _edge(dl, x, y, (7, 5), (3, 19), accent)
    _edge(dl, x, y, (17, 5), (21, 19), accent)


def revolve(dl, x, y, ink, accent):
    s._arc(dl, x, y, 12, 13, 8, 8, -125, 290, accent, 1.8)
    end = math.radians(165)
    tip = (12 + 8*math.cos(end), 13 + 8*math.sin(end))
    direction = (-math.sin(end), math.cos(end))
    for side in (-1, 1):
        tail = (tip[0]-direction[0]*3-direction[1]*side*2,
                tip[1]-direction[1]*3+direction[0]*side*2)
        _edge(dl, x, y, tip, tail, accent, 1.8)
    _edge(dl, x, y, (10, 8), (10, 19), ink)
    _edge(dl, x, y, (10, 19), (15, 19), ink)


def cylinder(dl, x, y, ink, accent):
    s._arc(dl, x, y, 12, 6, 8, 3, 0, 360, ink)
    s._arc(dl, x, y, 12, 18, 8, 3, 0, 180, accent)
    _edge(dl, x, y, (4, 6), (4, 18), accent)
    _edge(dl, x, y, (20, 6), (20, 18), accent)


def sphere(dl, x, y, ink, accent):
    s._circ(dl, s._pt(x, y, 12, 12), 9, ink)
    s._arc(dl, x, y, 12, 12, 4, 9, 0, 360, accent)
    s._arc(dl, x, y, 12, 12, 9, 3, 0, 360, accent)


def boolean(dl, x, y, ink, accent):
    s._circ(dl, s._pt(x, y, 9, 12), 7, ink)
    s._circ(dl, s._pt(x, y, 15, 12), 7, accent)


def cut(dl, x, y, ink, accent):
    boolean(dl, x, y, ink, accent)
    _edge(dl, x, y, (11, 5), (18, 19), accent, 2)


def common(dl, x, y, ink, accent):
    boolean(dl, x, y, ink, accent)
    s._dot(dl, s._pt(x, y, 12, 12), accent, 2.5)


def fillet(dl, x, y, ink, accent):
    _edge(dl, x, y, (4, 3), (4, 12), ink)
    s._arc(dl, x, y, 12, 12, 8, 8, 180, -90, accent)
    _edge(dl, x, y, (12, 20), (21, 20), ink)


def chamfer(dl, x, y, ink, accent):
    _edge(dl, x, y, (4, 3), (4, 12), ink)
    _edge(dl, x, y, (4, 12), (12, 20), accent)
    _edge(dl, x, y, (12, 20), (21, 20), ink)


def draft(dl, x, y, ink, accent):
    for a, b in (((3, 20), (21, 20)), ((7, 4), (17, 4))):
        _edge(dl, x, y, a, b, ink)
    _edge(dl, x, y, (7, 4), (3, 20), accent)
    _edge(dl, x, y, (17, 4), (21, 20), accent)


def shell(dl, x, y, ink, accent):
    # Open section: the wall thickness, rather than two unrelated rectangles.
    for a, b in (((4, 4), (4, 20)), ((4, 20), (20, 20)), ((20, 20), (20, 4))):
        _edge(dl, x, y, a, b, ink)
    for a, b in (((4, 4), (8, 4)), ((8, 4), (8, 16)), ((8, 16), (16, 16)),
                 ((16, 16), (16, 4)), ((16, 4), (20, 4))):
        _edge(dl, x, y, a, b, accent)


def hole(dl, x, y, ink, accent):
    s.draw_rectangle(dl, x, y, ink, accent)
    s._circ(dl, s._pt(x,y,12,12), 4, accent)


def counterbore(dl, x, y, ink, accent):
    _hole_section(dl, x, y, ink, accent, tapered=False)


def countersink(dl, x, y, ink, accent):
    _hole_section(dl, x, y, ink, accent, tapered=True)


def _hole_section(dl, x, y, ink, accent, *, tapered):
    for a, b in (((2, 5), (6, 5)), ((18, 5), (22, 5)),
                 ((2, 20), (9, 20)), ((15, 20), (22, 20))):
        _edge(dl, x, y, a, b, ink)
    for outer, inner in ((6, 9), (18, 15)):
        points = ((outer, 5), (inner, 11), (inner, 20)) if tapered else (
            (outer, 5), (outer, 11), (inner, 11), (inner, 20))
        for a, b in zip(points, points[1:]):
            _edge(dl, x, y, a, b, accent)


def pattern(dl, x, y, ink, accent):
    for row in range(3):
        for column in range(3):
            px, py = 3 + column * 7, 3 + row * 7
            color = ink if row == column == 0 else accent
            dl.AddRectFilled(s._pt(x, y, px, py), s._pt(x, y, px + 4, py + 4), color)


def assembly(dl, x, y, ink, accent):
    # Two joined components, with the interface kept clear at small sizes.
    for cx, cy, color in ((7, 9, ink), (17, 15, accent)):
        points = ((cx - 5, cy - 3), (cx, cy - 6), (cx + 5, cy - 3),
                  (cx + 5, cy + 3), (cx, cy + 6), (cx - 5, cy + 3),
                  (cx - 5, cy - 3))
        for a, b in zip(points, points[1:]):
            _edge(dl, x, y, a, b, color, 1.4)
        for a, b in (((cx - 5, cy - 3), (cx, cy)),
                     ((cx + 5, cy - 3), (cx, cy)), ((cx, cy), (cx, cy + 6))):
            _edge(dl, x, y, a, b, color, 1.4)


def circular_pattern(dl, x, y, ink, accent):
    s._circ(dl,s._pt(x,y,12,12),8,ink)
    for px,py in ((12,4),(20,12),(12,20),(4,12)):
        s._dot(dl,s._pt(x,y,px,py),accent,2.5)


def mirror(dl, x, y, ink, accent):
    _edge(dl,x,y,(12,2),(12,22),ink)
    for a,b,c,color in (((3,6),(9,18),(3,18),ink),((21,6),(15,18),(21,18),accent)):
        dl.AddTriangleFilled(s._pt(x,y,*a),s._pt(x,y,*b),s._pt(x,y,*c),color)


def section_view(dl, x, y, ink, accent):
    block(dl, x, y, ink, accent)
    for a, b in (((2, 14), (15, 4)), ((15, 4), (22, 9)),
                 ((22, 9), (9, 20)), ((9, 20), (2, 14))):
        _edge(dl, x, y, a, b, accent, 2)
    _edge(dl, x, y, (6, 13), (12, 17), accent)
    _edge(dl, x, y, (10, 10), (16, 14), accent)
    _edge(dl, x, y, (14, 7), (20, 11), accent)


def surface(dl, x, y, ink, accent):
    for a,b in (((3,8),(15,3)),((15,3),(22,16)),((22,16),(8,21)),((8,21),(3,8))):
        _edge(dl,x,y,a,b,ink)
    for a,b in (((6,13),(18,8)),((8,17),(20,12)),((8,6),(13,19)),((12,4),(17,17))):
        _edge(dl,x,y,a,b,accent,1)


def sweep(dl, x, y, ink, accent):
    s._arc(dl, x, y, 17, 17, 9, 9, 180, 90, accent, 1.6)
    s._arc(dl, x, y, 17, 17, 15, 15, 180, 90, accent, 1.6)
    s._arc(dl, x, y, 5, 17, 3, 2, 0, 360, ink, 1.6)
    s._arc(dl, x, y, 17, 5, 2, 3, 0, 360, ink, 1.6)


def check(dl, x, y, ink, accent):
    _edge(dl, x, y, (4, 12), (10, 18), accent, 2)
    _edge(dl, x, y, (10, 18), (21, 5), accent, 2)
