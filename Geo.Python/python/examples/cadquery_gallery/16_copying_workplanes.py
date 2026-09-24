"""CadQuery gallery: Copying Workplanes."""
from camber.cqcompat import Workplane

result = (
    Workplane("front").circle(1).extrude(10)
    .copyWorkplane(Workplane("right", origin=(-5, 0, 0)))
    .circle(1).extrude(10)
)
assert result.val().is_watertight()
assert result.val().volume() > 30

if __name__ == "__main__":
    result.show(title="CadQuery gallery 16 — copying workplanes")
