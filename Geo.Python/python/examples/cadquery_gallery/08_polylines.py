"""CadQuery gallery: Polylines (symmetric I-section)."""
from camber.cqcompat import Workplane

length, height, width, wall = 100.0, 20.0, 20.0, 1.0
half_profile = [
    (0, height / 2), (width / 2, height / 2),
    (width / 2, height / 2 - wall), (wall / 2, height / 2 - wall),
    (wall / 2, wall - height / 2), (width / 2, wall - height / 2),
    (width / 2, -height / 2), (0, -height / 2),
]
result = Workplane("front").polyline(half_profile).mirrorY().extrude(length)
assert result.val().is_watertight()
assert abs(result.val().volume() - (2 * width * wall + (height - 2 * wall) * wall) * length) < 1e-3

if __name__ == "__main__":
    result.show(title="CadQuery gallery 08 — polylines")
