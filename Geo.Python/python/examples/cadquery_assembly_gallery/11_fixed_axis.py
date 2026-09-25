"""CadQuery assembly docs: drafted post with one fixed face direction."""

from camber.cqcompat import Assembly, Color, Location, Workplane
from _common import part


def build():
    model = part()
    box = Workplane(part=model).box(1, 1, 1)
    post = Workplane(part=model).rect(0.1, 0.1).extrude(1, taper=-15)
    result = Assembly(part=model)
    result.add(box, name="b1")
    result.add(post, name="b2", loc=Location((0, 0, 4)), color=Color("red"))
    result.constrain("b1", "Fixed")
    result.constrain("b2@faces@<Z", "FixedPoint", (0, 0, 0.5))
    result.constrain("b2@faces@>Z", "FixedAxis", (1, 0, 2))
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — fixed axis")
