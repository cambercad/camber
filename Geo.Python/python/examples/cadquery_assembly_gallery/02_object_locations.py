"""CadQuery assembly docs: two cones with explicit instance locations."""

from camber.cqcompat import Assembly, Color, Location
from _common import cone, part


def build():
    model = part()
    shape = cone(model).val()
    result = Assembly(part=model)
    result.add(shape, loc=Location((0, 0, 0), (1, 0, 0), 180),
               name="cone0", color=Color("green"))
    result.add(shape, name="cone1", color=Color("blue"))
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — object locations")
