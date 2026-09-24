"""CadQuery gallery: An extruded prismatic solid (rectangle within a disk)."""
import math
from camber.cqcompat import Workplane

result = Workplane("front").circle(2.0).rect(0.5, 0.75).extrude(0.5)
assert result.val().is_watertight()
assert abs(result.val().volume() - (math.pi * 4 - 0.5 * 0.75) * 0.5) < 0.05

if __name__ == "__main__":
    result.show(title="CadQuery gallery 03 — extruded prism")
