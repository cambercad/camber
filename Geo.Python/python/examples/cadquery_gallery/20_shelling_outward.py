"""CadQuery gallery: outward shell, the next shelling variant on the page."""
from camber.cqcompat import Workplane

# CadQuery grows a rounded outer envelope and leaves the original box as a
# closed internal void. Camber currently supports inward offsets only.
result = Workplane("front").box(2, 2, 2).shell(0.1)
assert result.val().is_watertight()
assert result.val().volume() > 0
