"""CadQuery assembly docs: two line mates locate a sphere at a box corner."""

from camber.cqcompat import Assembly, Color, Location, Workplane
from _common import part


def build():
    model = part()
    box = Workplane(part=model).box(1, 1, 1)
    sphere = Workplane(part=model).sphere(0.15)
    result = Assembly(part=model)
    result.add(box, name="b1")
    result.add(sphere, name="b2", loc=Location((0, 0, 4)), color=Color("red"))
    result.constrain("b1", "Fixed")
    result.constrain("b2", "b1@edges@>>Z and >>Y", "PointOnLine")
    result.constrain("b2", "b1@edges@>>Z and >>X", "PointOnLine")
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — point on line")
