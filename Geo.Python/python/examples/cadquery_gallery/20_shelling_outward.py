"""CadQuery gallery: outward shell, the next shelling variant on the page."""
from camber.cqcompat import Workplane

# The original box becomes the enclosed void; the exterior expands by 0.1.
result = Workplane("front").box(2, 2, 2).shell(0.1)
assert result.val().is_watertight()
assert result.val().volume() > 0
