"""CadQuery assembly docs: L bracket located by offset PointInPlane mates."""

from camber.cqcompat import Assembly, Color, Workplane
from _common import part


def build():
    model = part()
    bracket = (Workplane("YZ", part=model).hLine(1).vLine(0.1)
               .hLineTo(0.2).vLineTo(1).hLineTo(0).close().extrude(1))
    box = Workplane(part=model).box(0.5, 0.5, 0.5)
    result = Assembly(part=model)
    result.add(bracket, name="bracket", color=Color("gray"))
    result.add(box, name="box", color=Color("green"))
    result.constrain("bracket@faces@>Z", "box@faces@>Z", "Axis", param=0)
    result.constrain("bracket@faces@>X", "box@faces@>X", "Axis", param=0)
    result.constrain("box@faces@<Z", "bracket@faces@Line3", "PointInPlane")
    result.constrain("box@faces@<Y", "bracket@faces@Line4", "PointInPlane", param=0.2)
    result.constrain("box@faces@>X", "bracket@faces@>X", "PointInPlane", param=-0.1)
    result.solve()
    return result


if __name__ == "__main__":
    build().show(title="CadQuery assembly — point in plane")
