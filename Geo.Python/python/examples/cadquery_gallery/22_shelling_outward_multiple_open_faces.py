"""CadQuery gallery: outward shell with multiple selected opening faces."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2, 2, 2).faces("+Z or -X or +X").shell(0.1)
assert result.val().is_watertight()
