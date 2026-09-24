"""CadQuery gallery: A Parametric Bearing Pillow Block."""
from camber.cqcompat import Workplane

length, height, bearing_diameter, thickness, padding = 30, 40, 22, 10, 8
result = (
    Workplane("XY").box(length, height, thickness)
    .faces(">Z").workplane().hole(bearing_diameter)
    .faces(">Z").workplane()
    .rect(length - padding, height - padding, forConstruction=True).vertices()
    .cboreHole(2.4, 4.4, 2.1)
)
assert result.val().is_watertight()
assert result.val().volume() > 0
