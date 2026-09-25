"""CadQuery sketch tutorial: operators combine selection-aware sketches."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    s1 = Sketch().rect(2, 2).vertices().fillet(0.25).reset()
    s2 = Sketch().rect(1, 1, angle=45).vertices().chamfer(0.1).reset()
    sketch = s1 - s2
    result = sketch.finalize().extrude(0.5)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 11 - boolean sketches')
