"""CadQuery gallery: Creating Workplanes on Faces."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2, 3, 0.5).faces(">Z").workplane().hole(0.5)
assert result.val().is_watertight()
assert result.val().volume() < 3.0
