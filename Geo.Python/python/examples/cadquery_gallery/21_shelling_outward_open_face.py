"""CadQuery gallery: outward shell with one selected opening face."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = Workplane("front").box(2, 2, 2).faces("+Z").shell(0.1)
    assert result.val().is_watertight()

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 21 — outward shell with opening')
