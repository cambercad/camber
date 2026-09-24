"""CadQuery gallery: Shelling To Create Thin features, first (closed) variant."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2, 2, 2).shell(-0.1)
assert result.val().is_watertight()
assert 0 < result.val().volume() < 8

if __name__ == "__main__":
    result.show(title="CadQuery gallery 19 — closed shell")
