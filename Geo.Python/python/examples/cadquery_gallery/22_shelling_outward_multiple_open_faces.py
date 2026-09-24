"""CadQuery gallery: outward shell with multiple selected opening faces."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2, 2, 2).faces("+Z or -X or +X").shell(0.1)
assert result.val().is_watertight()

if __name__ == "__main__":
    result.show(title="CadQuery gallery 22 — outward shell with multiple openings")
