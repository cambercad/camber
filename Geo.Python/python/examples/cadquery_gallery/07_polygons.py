"""CadQuery gallery: Polygons (two hexagonal through cuts)."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("front").box(3.0, 4.0, 0.25)
        .pushPoints([(0, 0.75), (0, -0.75)])
        .polygon(6, 1.0).cutThruAll()
    )
    assert result.val().is_watertight()
    assert 2.5 < result.val().volume() < 3.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 07 — polygons')
