"""CadQuery gallery: Mirroring 3D Objects (a notched rail and four copies)."""
from camber.cqcompat import Workplane

# A smaller notched rail keeps the gallery's 3D transform/mirror/union workflow.
profile = [(0, 0), (12, 0), (12, 2), (9, 2), (9, 5),
           (12, 5), (12, 8), (0, 8), (0, 0)]
result = Workplane("XY").polyline(profile).close().extrude(30)
result = result.rotate((0, 0, 0), (1, 0, 0), 90)
points, _ = result.val().mesh()
center = tuple((min(p[i] for p in points) + max(p[i] for p in points)) / 2 for i in range(3))
result = result.translate(tuple(-v for v in center))
mirrors = [
    result.mirror("XY", (0, 0, -20)),
    result.mirror("XY", (0, 0, 20)),
    result.mirror("ZY", (-20, 0, 0)),
    result.mirror("ZY", (20, 0, 0)),
]
for copy in mirrors:
    result = result.union(copy)
assert result.val().is_watertight()
assert result.val().volume() > 1000
