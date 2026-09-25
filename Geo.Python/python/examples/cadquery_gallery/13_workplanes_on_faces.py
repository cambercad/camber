"""CadQuery gallery: Creating Workplanes on Faces."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("front").box(2, 3, 0.5).faces(">Z").workplane().hole(0.5)
    assert result.val().is_watertight()
    assert result.val().volume() < 3.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 13 — workplanes on faces')
