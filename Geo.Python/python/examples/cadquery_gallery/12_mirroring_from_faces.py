"""CadQuery gallery: Mirroring From Faces."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("XY").line(0, 1).line(1, 0).line(0, -0.5).close().extrude(1)
    result = result.mirror(result.faces(">X"), union=True)
    assert result.val().is_watertight()
    assert result.val().volume() > 1.0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 12 — mirroring from faces')
