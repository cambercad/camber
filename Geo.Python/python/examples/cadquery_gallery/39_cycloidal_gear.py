"""CadQuery gallery: Cycloidal gear profile from a parametric curve."""
from math import cos, sin
from camber.cqcompat import Workplane

result = (
    Workplane("XY").parametricCurve(lambda t: (5 * cos(t), 5 * sin(t)))
    .twistExtrude(15, 90)
)
