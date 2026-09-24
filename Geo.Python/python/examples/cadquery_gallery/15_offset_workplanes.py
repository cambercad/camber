"""CadQuery gallery: Offset Workplanes."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").box(3, 2, 0.5)
    .faces("<X").workplane(offset=0.75)
    .circle(1).extrude(0.5)
)
assert result.val().is_watertight()
assert result.val().volume() > 3.0
