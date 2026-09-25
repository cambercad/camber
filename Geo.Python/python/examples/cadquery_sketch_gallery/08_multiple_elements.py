"""CadQuery sketch tutorial: distribute holes over a selected face."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane
    from _common import show

    result = (
        Workplane()
        .box(5, 5, 1)
        .faces(">Z")
        .workplane()
        .rarray(2, 2, 2, 2)
        .rect(1.5, 1.5)
        .extrude(0.5)
        .faces(">Z")
        .sketch()
        .circle(0.4)
        .wires()
        .distribute(6)
        .circle(0.1, mode="a")
        .clean()
        .finalize()
        .cutBlind(-0.5, taper=10)
    )
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 08 - multiple elements')
