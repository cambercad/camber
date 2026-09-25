"""CadQuery gallery: Using tags to select faces from an earlier solid."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("XY")
        .polygon(3, 5)
        .extrude(4)
        .tag("prism")
        .sphere(10)
        .faces("<X", tag="prism")
        .workplane()
        .circle(1)
        .cutThruAll()
        .faces(">X", tag="prism")
        .faces(">Y")
        .workplane()
        .circle(1)
        .cutThruAll()
    )
    assert result.val().is_watertight()
    assert result.val().volume() > 0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 31 — tagged face selectors')
