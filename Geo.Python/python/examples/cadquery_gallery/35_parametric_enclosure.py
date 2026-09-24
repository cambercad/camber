"""CadQuery gallery: A Parametric Enclosure (compact feature-path adaptation)."""
from camber.cqcompat import Workplane

width, length, height, wall = 40.0, 60.0, 24.0, 2.0
result = (
    Workplane("XY").box(width, length, height)
    .edges("|Z").fillet(3.0)
    .faces(">Z").shell(-wall)
)
assert result.val().is_watertight()
assert result.val().volume() > 0
