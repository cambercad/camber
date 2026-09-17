"""A sheet from constrained sections, two boundary guides and optional tangents."""

import argparse
from camber import Curve, Frame, Part, show


def build_surface():
    part = Part((-10, -20, -10), (50, 20, 50), tolerance=0.02)
    sections = []
    for name, height in (("inlet", 0), ("middle", 20), ("outlet", 40)):
        section = part.sketch(name=name, frame=Frame((0, 0, height)), constrained=True)
        section.solve_after_every_constraint = False
        section.add_line((0, 0), (30, 0), name="span")
        section.fix("span@0")
        section.horizontal("span")
        section.length("span", 30)
        section.solve()
        sections.append(section)

    left = Curve.hermite([(0, 0, 0), (0, 0, 20), (0, 0, 40)],
                         [(0, 1, 2), (0, -1, 2), (0, 1, 2)], name="left_boundary")
    right = Curve.hermite([(30, 0, 0), (30, 0, 20), (30, 0, 40)],
                          [(0, -1, 2), (0, 1, 2), (0, -1, 2)], name="right_boundary")
    sheet = part.loft_surface(sections, guides=[left, right], name="guided_sheet")
    # Without guides: part.loft_surface(sections)
    # Optional end derivatives, pointing from inlet toward outlet:
    # part.loft_surface(sections, start_tangent=(0, 20, 40), end_tangent=(0, -20, 40))
    return part, sheet


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    part, sheet = build_surface()
    print(f"Surface loft: {sheet.triangle_count} triangles, is_volume={sheet.is_volume}")
    if args.show:
        show(sheet, title="Guided surface loft")
