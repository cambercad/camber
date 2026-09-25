"""CadQuery gallery: Rotated Workplanes."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane, Vector

    result = (
        Workplane("front").box(4, 4, 0.25)
        .faces(">Z").workplane()
        .transformed(offset=Vector(0, -1.5, 1), rotate=Vector(60, 0, 0))
        .rect(1.5, 1.5, forConstruction=True).vertices().hole(0.25)
    )
    assert result.val().is_watertight()
    assert 3.8 < result.val().volume() < 4.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 17 — rotated workplanes')
