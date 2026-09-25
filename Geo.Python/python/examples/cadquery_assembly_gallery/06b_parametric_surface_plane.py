"""CadQuery's sinusoidal-surface Plane constraint refactor."""

from camber.cqcompat import Assembly, Color
from _common import cone, part, sinusoidal_surface


def build():
    model = part(25)
    surface = sinusoidal_surface(model)
    pin = cone(model, bottom=1, top=0.1, height=2)
    result = Assembly(part=model)
    result.add(surface, name="surf", color=Color("lightgray"))
    result.add(pin, name="cone", color=Color("green"))
    result.constrain("surf", "cone@faces@>Z", "Plane")
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — parametric surface plane")
