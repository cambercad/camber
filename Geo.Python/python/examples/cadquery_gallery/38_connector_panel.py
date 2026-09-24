"""CadQuery gallery: Panel With Various Connector Holes (repeated rounded slots)."""
from camber.cqcompat import Workplane

result = Workplane("XY").box(120, 100, 2)
for y in (30, 10, -10, -30):
    result = (
        result.faces(">Z").workplane()
        .center(30, y).moveTo(-15, -4).threePointArc((-19, 0), (-15, 4))
        .lineTo(15, 4).threePointArc((19, 0), (15, -4)).close()
        .cutThruAll()
    )
assert result.val().is_watertight()

if __name__ == "__main__":
    result.show(title="CadQuery gallery 38 — connector panel")
