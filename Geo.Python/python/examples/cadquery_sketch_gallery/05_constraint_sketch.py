"""CadQuery sketch tutorial: named line/arc constraints."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Sketch
    from _common import show

    sketch = (
        Sketch()
        .segment((0, 0), (0, 3.0), "s1")
        .arc((0.0, 3.0), (1.5, 1.5), (0.0, 0.0), "a1")
        .constrain("s1", "Fixed", None)
        .constrain("s1", "a1", "Coincident", None)
        .constrain("a1", "s1", "Coincident", None)
        .constrain("s1", "a1", "Angle", 45)
        .solve()
        .assemble()
    )
    result = sketch.finalize().extrude(0.15)
    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery sketch 05 - constraints')
