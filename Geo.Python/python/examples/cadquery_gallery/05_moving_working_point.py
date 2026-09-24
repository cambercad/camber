"""CadQuery gallery: Moving The Current working point."""
import math
from camber.cqcompat import Workplane

result = Workplane("front").circle(3.0)
result = result.center(1.5, 0).rect(0.5, 0.5)
result = result.center(-1.5, 1.5).circle(0.25).extrude(0.25)
assert result.val().is_watertight()
expected = (math.pi * 3**2 - 0.5**2 - math.pi * 0.25**2) * 0.25
assert abs(result.val().volume() - expected) < 0.05
