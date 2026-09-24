"""CadQuery gallery: Using construction Geometry."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").box(2, 2, 0.5)
    .faces(">Z").workplane()
    .rect(1.5, 1.5, forConstruction=True).vertices().hole(0.125)
)
assert result.val().is_watertight()
assert 1.9 < result.val().volume() < 2.0
