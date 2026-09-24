"""CadQuery gallery: Braille Example (raised cylindrical tactile dots)."""
from camber.cqcompat import Workplane

# One 6-dot cell, laid out at the CadQuery example's inter-dot pitch.
points = [(0, 5), (0, 2.5), (0, 0), (2.5, 5), (2.5, 2.5), (2.5, 0)]
result = (
    Workplane("XY").box(12, 12, 1.5, centered=(False, False, False))
    .faces(">Z").workplane().pushPoints(points).circle(0.65).extrude(0.5)
)
assert result.val().is_watertight()
