"""CadQuery gallery: Extruding until a given face (the `next` variant)."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane

    result = (
        Workplane(origin=(20, 0, 0)).circle(2)
        .revolve(180, (-20, 0, 0), (-20, -1, 0))
        .center(-20, 0).workplane().rect(20, 4).extrude("next")
    )
    assert result.val().is_watertight()
    _points, _ = result.val().mesh()
    assert all(any(abs(point.x - x) < 0.01 and abs(point.y - y) < 0.01
                   and abs(point.z) < 0.01 for point in _points)
               for x, y in ((-10, -2), (10, 2)))

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 24 — extrude until next face')
