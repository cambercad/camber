"""CadQuery gallery: Splitting an Object."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    body = (Workplane("XY").box(1, 1, 1).faces(">Z").workplane()
            .circle(0.25).cutThruAll())
    result = body.faces(">Y").workplane(-0.5).split(keepTop=True)
    assert result.val().is_watertight()
    assert abs(result.val().volume() - body.val().volume() / 2) < 0.01

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 33 — split object')
