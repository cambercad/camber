"""CadQuery gallery: Rounding Corners with Fillet."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("XY").box(3, 3, 0.5).edges("|Z").fillet(0.125)
    assert result.val().is_watertight()
    assert result.val().volume() > 4.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 29 — filleted corners')
