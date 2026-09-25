"""CadQuery assembly docs: point plus directed axis puts a pin in a plate."""

from camber.cqcompat import Assembly, Color, Workplane
from _common import cone, part


def build():
    model = part()
    plate = Workplane(part=model).box(10, 10, 1).faces(">Z").workplane().hole(2)
    pin = cone(model, 0.8, 0, 4)
    result = Assembly(part=model)
    result.add(plate, name="plate", color=Color("green"))
    result.add(pin, name="cone", color=Color("blue"))
    result.constrain("plate@faces@>Z", "cone@faces@<Z", "Point")
    result.constrain("plate@faces@>Z", "cone@faces@<Z", "Axis", param=0)
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — plate and pin")
