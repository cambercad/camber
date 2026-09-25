"""CadQuery sketch tutorial: face-based profile construction."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    # Same sketch operations as the CadQuery tutorial. A short extrusion is added
    # only to give Camber's 3D viewer a displayable solid.
    sketch = (
        Sketch()
        .trapezoid(4, 3, 90)
        .vertices()
        .circle(0.5, mode="s")
        .reset()
        .vertices()
        .fillet(0.25)
        .reset()
        .rarray(0.6, 1, 5, 1)
        .slot(1.5, 0.4, mode="s", angle=90)
    )
    result = sketch.finalize().extrude(0.15)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 01 - face based')
