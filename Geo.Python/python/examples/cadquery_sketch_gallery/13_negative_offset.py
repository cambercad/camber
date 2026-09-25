"""CadQuery sketch tutorial: replace source contours with an inward offset."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane, Sketch
    from _common import show

    sketch = Sketch().rect(1.0, 4.0).circle(1.0).clean()
    sketch_offset = sketch.copy().wires().offset(-0.25, mode="r")
    body = Workplane("front").placeSketch(sketch).extrude(1.0)
    result = (body.faces(">Z").workplane().placeSketch(sketch_offset)
              .cutBlind(-0.50))
    assert result.val().is_watertight()
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 13 - negative offset')
