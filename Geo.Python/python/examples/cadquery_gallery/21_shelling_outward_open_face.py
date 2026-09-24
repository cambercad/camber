"""CadQuery gallery: outward shell with one selected opening face."""
from camber.cqcompat import Workplane

result = Workplane("front").box(2, 2, 2).faces("+Z").shell(0.1)
assert result.val().is_watertight()

if __name__ == "__main__":
    result.show(title="CadQuery gallery 21 — outward shell with opening")
