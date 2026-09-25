"""CadQuery sketch tutorial: convex hull of circles and segments."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    sketch = (
        Sketch()
        .arc((0, 0), 1.0, 0.0, 360.0)
        .arc((1, 1.5), 0.5, 0.0, 360.0)
        .segment((0.0, 2), (-1, 3.0))
        .hull()
    )
    result = sketch.finalize().extrude(0.15)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 04 - convex hull')
