"""CadQuery gallery: Mirroring Symmetric Geometry."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").hLine(1).vLine(0.5)
    .hLine(-0.25).vLine(-0.25).hLineTo(0)
    .mirrorY().extrude(0.25)
)
assert result.val().is_watertight()
assert result.val().volume() > 0.1
