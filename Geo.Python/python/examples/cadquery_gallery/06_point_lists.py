"""CadQuery gallery: Using Point Lists."""
import math
from camber.cqcompat import Workplane

result = (
    Workplane("front").circle(2.0)
    .pushPoints([(1.5, 0), (0, 1.5), (-1.5, 0), (0, -1.5)])
    .circle(0.25).extrude(0.125)
)
assert result.val().is_watertight()
assert abs(result.val().volume() - math.pi * (2**2 - 4 * 0.25**2) * 0.125) < 0.01
