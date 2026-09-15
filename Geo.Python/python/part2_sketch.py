"""SolidWorks \"Sketch1 of Part2\": origin → 150 mm left → quarter arc → vertical up.

Arc radius matches the dimension in the SW property manager (42.09666306 mm).
The vertical segment is under-defined in SW; adjust vertical_len if needed.
"""
import math

from camber import Part, vec3

horizontal_len = 150.0
arc_radius = 42.09666306
vertical_len = 50.0

part = Part(vec3(-220, -10, -10), vec3(20, 120, 10), tolerance=0.01)
sk = part.sketch("xy", name="Sketch1")
sk.set_start((0, 0))
sk.add_line((0, 0), (-horizontal_len, 0))
# West-going strip: Right(π/2) turns toward +Y, then vertical.
sk.append_arc_right(arc_radius, math.pi / 2)
sk.append_line_vertical(arc_radius + vertical_len)
print(sk)
sk.show(title="part2 sketch1")
