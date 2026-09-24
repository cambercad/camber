"""CadQuery gallery: Lego Brick (thin 6 by 2 brick with underside posts)."""
from camber.cqcompat import Workplane

length_bumps, width_bumps = 6, 2
pitch, clearance, stud_diameter, stud_height = 8.0, 0.1, 4.8, 1.8
height = 3.2
wall = (pitch - 2 * clearance - stud_diameter) / 2
post_diameter = pitch - wall
length = length_bumps * pitch - 2 * clearance
width = width_bumps * pitch - 2 * clearance
result = (
    Workplane("XY").box(length, width, height)
    .faces("<Z").shell(-wall)
    .faces(">Z").workplane().rarray(pitch, pitch, length_bumps, width_bumps)
    .circle(stud_diameter / 2).extrude(stud_height)
)
assert result.val().is_watertight()

if __name__ == "__main__":
    result.show(title="CadQuery gallery 36 — Lego brick")
