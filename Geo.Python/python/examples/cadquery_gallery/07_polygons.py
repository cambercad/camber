"""CadQuery gallery: Polygons (two hexagonal through cuts)."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").box(3.0, 4.0, 0.25)
    .pushPoints([(0, 0.75), (0, -0.75)])
    .polygon(6, 1.0).cutThruAll()
)
assert result.val().is_watertight()
assert 2.5 < result.val().volume() < 3.0
