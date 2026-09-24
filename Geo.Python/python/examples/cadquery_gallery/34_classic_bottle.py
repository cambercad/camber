"""CadQuery gallery: The Classic OCC Bottle (profile and neck)."""
from camber.cqcompat import Workplane

length, width, wall = 20.0, 6.0, 3.0
body = (
    Workplane("XY").center(-length / 2, 0).vLine(width / 2)
    .threePointArc((length / 2, width / 2 + wall), (length, width / 2))
    .vLine(-width / 2).mirrorX().extrude(30.0, True)
)
result = body.faces(">Z").workplane(centerOption="CenterOfMass").circle(3.0).extrude(2.0, True)
result = result.faces(">Z").shell(0.3, kind="arc")
assert result.val().is_watertight()
assert result.val().volume() > 0
assert any("ShellRim_" in name for name in result.val().patch_names)

if __name__ == "__main__":
    result.show(title="CadQuery gallery 34 — classic bottle")
