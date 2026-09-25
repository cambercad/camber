"""CadQuery gallery: Locating a Workplane on a vertex."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("front").box(3, 2, 0.5)
        .faces(">Z").vertices("<XY").workplane(centerOption="CenterOfMass")
        .circle(1).cutThruAll()
    )
    assert result.val().is_watertight()
    assert 2.0 < result.val().volume() < 3.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 14 — workplane on vertex')
