"""CadQuery assembly docs: center and vertex fixed to the same world point."""

from camber.cqcompat import Assembly, Color, Location, Workplane
from _common import part


def build():
    model = part()
    box = Workplane(part=model).box(1, 1, 1)
    sphere = Workplane(part=model).sphere(0.15)
    result = Assembly(part=model)
    result.add(box, name="b1")
    result.add(sphere, name="b2", loc=Location((0, 0, 4)), color=Color("red"))
    result.add(box, name="b3", loc=Location((-2, 0, 0)), color=Color("red"))
    result.constrain("b1", "Fixed")
    result.constrain("b2", "FixedPoint", (0.5, 0.5, 0.5))
    result.constrain("b3@vertices@<X and <Y and <Z", "FixedPoint", (0.5, 0.5, 0.5))
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — fixed point")
