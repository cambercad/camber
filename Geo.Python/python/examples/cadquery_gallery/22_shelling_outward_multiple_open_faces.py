"""CadQuery gallery: outward shell with multiple selected opening faces."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("front").box(2, 2, 2).faces("+Z or -X or +X").shell(0.1)
    assert result.val().is_watertight()

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 22 — outward shell with multiple openings')
