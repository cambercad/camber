"""CadQuery gallery: Rounding Corners with Fillet."""
from camber.cqcompat import Workplane

result = Workplane("XY").box(3, 3, 0.5).edges("|Z").fillet(0.125)
assert result.val().is_watertight()
assert result.val().volume() > 4.0
