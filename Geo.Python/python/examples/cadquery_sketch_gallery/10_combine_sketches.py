"""CadQuery sketch tutorial: combine planar regions with face()."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    s1 = Sketch().rect(2, 2)
    s2 = Sketch().circle(0.5)
    sketch = s1.face(s2, mode="s")
    result = sketch.finalize().extrude(0.5)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 10 - combine sketches')
