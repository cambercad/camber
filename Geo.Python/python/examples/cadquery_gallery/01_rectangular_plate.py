"""CadQuery gallery: Simple Rectangular Plate."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("front").box(2.0, 2.0, 0.5)
    assert result.val().is_watertight()
    assert abs(result.val().volume() - 2.0) < 1e-5

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 01 — rectangular plate')
