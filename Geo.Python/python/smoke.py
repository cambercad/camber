from camber import Part, vec2, vec3
import os
import tempfile

part = Part(vec3(-10), vec3(10), tolerance=0.05)
sk = part.sketch("xy", name="box")
sk.add_line((0, 0), (2, 0))
sk.add_line((2, 0), (2, 2))
sk.add_line((2, 2), (0, 2))
sk.add_line((0, 2), (0, 0))
cube = part.extrude(sk, 2.0, name="cube")
hole = part.cylinder(origin=(1, 1, -0.5), radius=0.4, height=3.0, name="cyl")
cut = cube - hole
out = os.path.join(tempfile.gettempdir(), "camber_smoke.stl")
cut.save_stl(out)
print("ok", cut.triangle_count, "triangles", out)
print("vec", vec2(1, 0) + (0, 2))
