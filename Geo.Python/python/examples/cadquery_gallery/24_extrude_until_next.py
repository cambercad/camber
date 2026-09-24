"""CadQuery gallery: Extruding until a given face (the `next` variant)."""
from camber.cqcompat import Workplane

result = (
    Workplane(origin=(20, 0, 0)).circle(2)
    .revolve(180, (-20, 0, 0), (-20, -1, 0))
    .center(-20, 0).workplane().rect(20, 4).extrude("next")
)
