"""CadQuery gallery: Copying Workplanes."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("front").circle(1).extrude(10)
        .copyWorkplane(Workplane("right", origin=(-5, 0, 0)))
        .circle(1).extrude(10)
    )
    assert result.val().is_watertight()
    assert result.val().volume() > 30

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 16 — copying workplanes')
