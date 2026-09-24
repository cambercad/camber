"""CadQuery gallery: Simple Rectangular Plate."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2.0, 2.0, 0.5)
assert result.val().is_watertight()
assert abs(result.val().volume() - 2.0) < 1e-5
