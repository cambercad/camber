"""CadQuery gallery: Mirroring From Faces."""
from camber.cqcompat import Workplane

result = Workplane("XY").line(0, 1).line(1, 0).line(0, -0.5).close().extrude(1)
result = result.mirror(result.faces(">X"), union=True)
assert result.val().is_watertight()
assert result.val().volume() > 1.0
