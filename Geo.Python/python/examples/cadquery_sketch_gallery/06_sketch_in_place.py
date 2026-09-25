"""CadQuery sketch tutorial: sketching directly on a selected face."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane
    from _common import show

    result = (
        Workplane()
        .box(5, 5, 1)
        .faces(">Z")
        .sketch()
        .regularPolygon(2, 3, tag="outer")
        .regularPolygon(1.5, 3, mode="s")
        .vertices(tag="outer")
        .fillet(0.2)
        .finalize()
        .extrude(0.5)
    )
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 06 - in place')
