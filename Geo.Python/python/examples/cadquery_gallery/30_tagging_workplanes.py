"""CadQuery gallery: Tagging a workplane for later reuse."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane("XY").box(10, 10, 10).faces(">Z").workplane().tag("baseplane")
        .center(-3, 0).circle(1).extrude(3)
        .workplaneFromTagged("baseplane").center(3, 0).circle(1).extrude(2)
    )
    assert result.val().is_watertight()

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 30 — tagged workplanes')
