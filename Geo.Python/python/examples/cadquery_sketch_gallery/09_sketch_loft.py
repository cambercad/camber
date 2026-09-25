"""CadQuery sketch tutorial: loft between two moved sketches."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane, Sketch
    from _common import show

    s1 = Sketch().trapezoid(3, 1, 110).vertices().fillet(0.2)
    s2 = Sketch().rect(2, 1).vertices().fillet(0.2)
    result = Workplane().placeSketch(s1, s2.moved(z=3)).loft()
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 09 - loft')
