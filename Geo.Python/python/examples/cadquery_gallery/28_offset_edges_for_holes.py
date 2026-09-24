"""CadQuery gallery: Edge-offset construction geometry for counterbored holes."""
from camber.cqcompat import Workplane

result = (
    Workplane().box(4, 2, 0.5).faces(">Z").edges().toPending()
    .offset2D(-0.25, forConstruction=True).vertices()
    .cboreHole(0.125, 0.25, 0.125, depth=None)
)

if __name__ == "__main__":
    result.show(title="CadQuery gallery 28 — offset edges for holes")
