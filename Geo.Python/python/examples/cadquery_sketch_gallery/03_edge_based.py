"""CadQuery sketch tutorial: edge-based geometry and face operations."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    sketch = (
        Sketch()
        .segment((0.0, 0), (0.0, 2.0))
        .segment((2.0, 0))
        .close()
        .arc((0.6, 0.6), 0.4, 0.0, 360.0)
        .assemble(tag="face")
        .edges("%LINE", tag="face")
        .vertices()
        .chamfer(0.2)
    )
    result = sketch.finalize().extrude(0.15)
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 03 - edge based')
