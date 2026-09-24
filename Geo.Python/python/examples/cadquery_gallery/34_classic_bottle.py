"""CadQuery gallery: The Classic OCC Bottle (profile and neck)."""
from camber.cqcompat import Workplane

length, width, wall = 20.0, 6.0, 3.0
body = (
    Workplane("XY").center(-length / 2, 0).vLine(width / 2)
    .threePointArc((length / 2, width / 2 + wall), (length, width / 2))
    .vLine(-width / 2).mirrorX().extrude(30, both=True)
)
result = body.faces(">Z").workplane(centerOption="CenterOfMass").circle(3).extrude(2, both=True)
result = result.faces(">Z").shell(0.3)
