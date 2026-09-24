"""CadQuery gallery: Defining an Edge with a Spline."""
from camber.cqcompat import Workplane

points = [(2.75, 1.5), (2.5, 1.75), (2, 1.5), (1.5, 1),
          (1, 1.25), (0.5, 1), (0, 1)]
result = (
    Workplane("XY").lineTo(3, 0).lineTo(3, 1)
    .spline(points, includeCurrent=True).close().extrude(0.5)
)
assert result.val().is_watertight()
assert result.val().volume() > 0.5

if __name__ == "__main__":
    result.show(title="CadQuery gallery 09 — spline edge")
