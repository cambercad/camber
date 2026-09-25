"""CadQuery gallery: Offsetting wires in 2D with two corner join styles."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    original = Workplane().polygon(5, 10).extrude(0.1).translate((0, 0, 2))
    arc = Workplane().polygon(5, 10).offset2D(1, "arc").extrude(0.1).translate((0, 0, 1))
    intersection = Workplane().polygon(5, 10).offset2D(1, "intersection").extrude(0.1)
    result = original.union(arc).union(intersection)

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 27 — 2D wire offsets')
