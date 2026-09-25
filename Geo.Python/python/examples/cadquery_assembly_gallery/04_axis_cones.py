"""CadQuery assembly docs: opposed cone base normals."""

from camber.cqcompat import Assembly, Color
from _common import cone, part


def build():
    model = part()
    shape = cone(model)
    result = Assembly(part=model)
    result.add(shape, name="cone0", color=Color("green"))
    result.add(shape, name="cone1", color=Color("blue"))
    result.constrain("cone0@faces@<Z", "cone1@faces@<Z", "Axis")
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — axis cones")
