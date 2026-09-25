"""CadQuery sketch tutorial: place a reusable profile on another plane."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane, Sketch
    from _common import show

    s = Sketch().trapezoid(3, 1, 110).vertices().fillet(0.2)
    result = (
        Workplane()
        .box(5, 5, 5)
        .faces(">X")
        .workplane()
        .transformed((0, 0, -90))
        .placeSketch(s)
        .cutThruAll()
    )
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 07 - place sketch')
