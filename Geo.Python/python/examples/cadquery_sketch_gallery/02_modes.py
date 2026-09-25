"""CadQuery sketch tutorial: construction tags and boolean modes."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    sketch = (
        Sketch()
        .rect(1, 2, mode="c", tag="base")
        .vertices(tag="base")
        .circle(0.7)
        .reset()
        .edges("|Y", tag="base")
        .ellipse(1.2, 1, mode="i")
        .reset()
        .rect(2, 2, mode="i")
        .clean()
    )
    result = sketch.finalize().extrude(0.15)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 02 - modes')
