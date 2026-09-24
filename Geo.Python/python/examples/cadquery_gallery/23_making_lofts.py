"""CadQuery gallery: Making Lofts (circle profile to rectangular section)."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").box(4, 4, 0.25).faces(">Z")
    .circle(1.5).workplane(offset=3).rect(0.75, 0.5).loft(combine=True)
)
assert result.val().is_watertight()
assert result.val().volume() > 4.0

if __name__ == "__main__":
    result.show(title="CadQuery gallery 23 — lofts")
