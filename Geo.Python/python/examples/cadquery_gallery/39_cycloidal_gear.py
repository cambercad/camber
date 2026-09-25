"""CadQuery gallery: the cycloidal gear, using a tolerance-sampled curve."""

def build():
    """Build and return this CadQuery gallery sample."""
    from math import cos, floor, pi, sin
    from camber.cqcompat import Workplane


    def hypocycloid(t, r1, r2):
        return (
            (r1 - r2) * cos(t) + r2 * cos(r1 / r2 * t - t),
            (r1 - r2) * sin(t) + r2 * sin(-(r1 / r2 * t - t)),
        )


    def epicycloid(t, r1, r2):
        return (
            (r1 + r2) * cos(t) - r2 * cos(r1 / r2 * t + t),
            (r1 + r2) * sin(t) - r2 * sin(r1 / r2 * t + t),
        )


    def gear(t, r1=4, r2=1):
        if (-1) ** (1 + floor(t / (2 * pi) * (r1 / r2))) < 0:
            return epicycloid(t, r1, r2)
        return hypocycloid(t, r1, r2)


    result = (
        Workplane("XY").parametricCurve(lambda t: gear(t * 2 * pi, 6, 1))
        .twistExtrude(15, 90)
        .faces(">Z").workplane().circle(2).cutThruAll()
    )
    assert result.val().is_watertight()
    assert result.val().volume() > 0

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 39 — cycloidal gear')
