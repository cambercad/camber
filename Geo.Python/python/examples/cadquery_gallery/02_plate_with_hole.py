"""CadQuery gallery: Plate with Hole."""
import math
from camber.cqcompat import Workplane

length, width, thickness, bore = 80.0, 60.0, 10.0, 22.0
result = Workplane("XY").box(length, width, thickness).faces(">Z").workplane().hole(bore)
assert result.val().is_watertight()
assert abs(result.val().volume() - (length * width - math.pi * (bore / 2) ** 2) * thickness) < 10.0

if __name__ == "__main__":
    result.show(title="CadQuery gallery 02 — plate with hole")
