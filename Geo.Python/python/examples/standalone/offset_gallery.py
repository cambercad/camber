"""OffsetStrip / OffsetNetwork gallery — open/closed sources, join & end-cap styles.

Layout (XY), left → right within each row:
  Row 0 (y≈0):    closed square — outside Round / Miter / Square, inside Round
  Row 1 (y≈-70):  open L — Parallel Round / Miter / Square, Parallel negative
  Row 2 (y≈-140): open L — Outline Round / Butt / Square end caps
  Row 3 (y≈-210): open plus (4 rays) — Outline Round / Butt / Square end caps
"""
from camber import Part, vec3

size = 30.0
gap = 55.0
offset_out = 4.0
offset_in = 4.0

part = Part(vec3(-20, -270, -5), vec3(280, 60, 5), tolerance=0.02)


def closed_square(sk, origin, prefix):
    x0, y0 = origin
    a = sk.add_line((x0, y0), (x0 + size, y0), name=prefix + "_s", construction=True)
    b = sk.add_line((x0 + size, y0), (x0 + size, y0 + size), name=prefix + "_e", construction=True)
    c = sk.add_line((x0 + size, y0 + size), (x0, y0 + size), name=prefix + "_n", construction=True)
    d = sk.add_line((x0, y0 + size), (x0, y0), name=prefix + "_w", construction=True)
    return [a, b, c, d]


def open_l(sk, origin, prefix):
    x0, y0 = origin
    a = (x0, y0)
    b = (x0 + size, y0)
    c = (x0 + size, y0 + size * 0.7)
    line0 = sk.add_line(a, b, name=prefix + "_h", construction=True)
    line1 = sk.add_line(b, c, name=prefix + "_v", construction=True)
    return [line0, line1]


def open_plus(sk, origin, prefix):
    x0, y0 = origin
    half = size * 0.5
    center = (x0 + half, y0 + half)
    return [
        sk.add_line(center, (x0 + size, y0 + half), name=prefix + "_e", construction=True),
        sk.add_line(center, (x0 + half, y0 + size), name=prefix + "_n", construction=True),
        sk.add_line(center, (x0, y0 + half), name=prefix + "_w", construction=True),
        sk.add_line(center, (x0 + half, y0), name=prefix + "_s", construction=True),
    ]


sk = part.sketch("xy", name="offset_gallery")
sk.offset(closed_square(sk, (0, 0), "c0"), offset_out, side="out", join="round")
sk.offset(closed_square(sk, (gap, 0), "c1"), offset_out, side="out", join="miter")
sk.offset(closed_square(sk, (2 * gap, 0), "c2"), offset_out, side="out", join="square")
sk.offset(closed_square(sk, (3 * gap, 0), "c3"), offset_in, side="in", join="round")

sk.offset(open_l(sk, (0, -70), "p0"), offset_out, side="left", join="round")
sk.offset(open_l(sk, (gap, -70), "p1"), offset_out, side="left", join="miter")
sk.offset(open_l(sk, (2 * gap, -70), "p2"), offset_out, side="left", join="square")
sk.offset(open_l(sk, (3 * gap, -70), "p3"), offset_out, side="right", join="round")

sk.offset(
    open_l(sk, (0, -140), "o0"), offset_out, side="both", join="round",
    end_cap="round", open_mode="outline")
sk.offset(
    open_l(sk, (gap, -140), "o1"), offset_out, side="both", join="miter",
    end_cap="butt", open_mode="outline")
sk.offset(
    open_l(sk, (2 * gap, -140), "o2"), offset_out, side="both", join="square",
    end_cap="square", open_mode="outline")

sk.offset(
    open_plus(sk, (0, -210), "n0"), offset_out, side="both", join="round",
    end_cap="round", open_mode="network")
sk.offset(
    open_plus(sk, (gap, -210), "n1"), offset_out, side="both", join="miter",
    end_cap="butt", open_mode="network")
sk.offset(
    open_plus(sk, (2 * gap, -210), "n2"), offset_out, side="both", join="square",
    end_cap="square", open_mode="network")

print(sk)
sk.show(title="offset gallery")
