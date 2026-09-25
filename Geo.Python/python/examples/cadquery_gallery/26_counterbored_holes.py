"""CadQuery gallery: Making Counter-bored and Counter-sunk Holes."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("XY").box(4, 2, 0.5).faces(">Z").workplane()
        .rect(3.5, 1.5, forConstruction=True).vertices()
        .cboreHole(0.125, 0.25, 0.125, depth=None)
    )
    assert result.val().is_watertight()
    assert 3.94 < result.val().volume() < 3.97  # Four through holes and four counterbores.

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 26 — counterbored holes')
