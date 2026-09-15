"""Stepped profile from a SolidWorks-style drawing (mm).

Origin at bottom-left; profile starts on +Y at y_top, ends on the same level at x=250.
Left/right chamfer steps: 60° from vertical (20 horizontal, 12 vertical each), 4 mm shelves.
"""
from camber import Part, vec3

y_top = 241.30
step_dx = 20.0
step_dy = 12.0
shelf = 4.0
floor_len = 80.0
total_width = 250.0

part = Part(vec3(-20, -20, -10), vec3(270, 260, 10), tolerance=0.01)
sk = part.sketch("xy", name="stepped")

sk.add_line((0, -40), (0, 280), construction=True)
sk.add_line((-40, 0), (290, 0), construction=True)

y_floor = y_top - 2.0 * step_dy
sk.set_start((0, y_top))
sk.append_line((step_dx, y_top - step_dy))
sk.append_line_horizontal(step_dx + shelf)
sk.append_line((2.0 * step_dx + shelf, y_floor))
sk.append_line_horizontal(2.0 * step_dx + shelf + floor_len)
sk.append_line((3.0 * step_dx + shelf + floor_len, y_top - step_dy))
sk.append_line_horizontal(3.0 * step_dx + 2.0 * shelf + floor_len)
sk.append_line((4.0 * step_dx + 2.0 * shelf + floor_len, y_top))
sk.append_line_horizontal(total_width)

print(sk)
sk.show(title="stepped profile")
