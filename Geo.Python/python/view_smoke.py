from camber import Part, vec3

part = Part(vec3(-10), vec3(10), tolerance=0.05)
sk = part.sketch("xy", name="box")
sk.add_line((0, 0), (2, 0))
sk.add_line((2, 0), (2, 2))
sk.add_line((2, 2), (0, 2))
sk.add_line((0, 2), (0, 0))
cube = part.extrude(sk, 2.0, name="cube")
hole = part.cylinder(origin=(1, 1, -0.5), radius=0.4, height=3.0, name="cyl")
(cube - hole).show()
