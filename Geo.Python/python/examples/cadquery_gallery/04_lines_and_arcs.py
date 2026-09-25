"""CadQuery gallery: Building Profiles using lines and arcs."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("front")
        .lineTo(2, 0).lineTo(2, 1)
        .threePointArc((1, 1.5), (0, 1))
        .close().extrude(0.25)
    )
    assert result.val().is_watertight()
    assert 0.5 < result.val().volume() < 1.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 04 — lines and arcs')
