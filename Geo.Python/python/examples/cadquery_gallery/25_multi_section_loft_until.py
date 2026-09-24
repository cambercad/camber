"""CadQuery gallery: multiple lofted sections followed by an until cut."""
from camber.cqcompat import Workplane

locations = [(-16, 1), (-8, 0), (7, 0.2), (17, -1.2)]
angles = iter([15, 0, -8, 10])
result = Workplane().pushPoints(locations).eachpoint(
    lambda loc: Workplane().rect(5, 16).workplane(offset=10)
    .ellipse(3, 8).workplane(offset=10).slot2D(20, 5, 90).loft()
)
