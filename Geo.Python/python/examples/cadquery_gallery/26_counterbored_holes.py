"""CadQuery gallery: Making Counter-bored and Counter-sunk Holes."""
from camber.cqcompat import Workplane

result = (
    Workplane("XY").box(4, 2, 0.5).faces(">Z").workplane()
    .rect(3.5, 1.5, forConstruction=True).vertices()
    .cboreHole(0.125, 0.25, 0.125, depth=None)
)
assert result.val().is_watertight()
assert result.val().volume() < 4

if __name__ == "__main__":
    result.show(title="CadQuery gallery 26 — counterbored holes")
