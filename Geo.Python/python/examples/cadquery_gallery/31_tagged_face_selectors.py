"""CadQuery gallery: Using tags to select faces from an earlier solid."""
from camber.cqcompat import Workplane

result = (
    Workplane("XY").polygon(3, 5).extrude(4).tag("prism")
    .sphere(10)
    .faces("<X", tag="prism").workplane().circle(1).cutThruAll()
)
