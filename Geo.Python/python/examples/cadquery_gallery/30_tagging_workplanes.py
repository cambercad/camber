"""CadQuery gallery: Tagging a workplane for later reuse."""
from camber.cqcompat import Workplane

result = (
    Workplane("XY").box(10, 10, 10).faces(">Z").workplane().tag("baseplane")
    .center(-3, 0).circle(1).extrude(3)
    .workplaneFromTagged("baseplane").center(3, 0).circle(1).extrude(2)
)
assert result.val().is_watertight()
