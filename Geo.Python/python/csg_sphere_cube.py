"""Classic CSG demo: (sphere & cube) minus three through-holes. Opens Polyscope.

Needs a wheel with sphere + cylinder(axis=...). From Geo.Python, venv active:

  .\\publish-wheel.ps1
  python -m pip install --force-reinstall (Get-ChildItem dist\\camber*.whl | Select-Object -Last 1).FullName
  python python\\csg_sphere_cube.py
"""
from camber import Part, vec3

cube_half = 9.0
sphere_r = 10.0
hole_r = 5.0
hole_h = 24.0

part = Part(vec3(-25), vec3(25), tolerance=0.01)

sk = part.sketch("xy", name="cube_sk")
sk.add_rectangle((-cube_half, -cube_half), (cube_half, cube_half))
cube = part.extrude(sk, cube_half, name="cube", both_sides=True)

sphere = part.sphere((0, 0, 0), sphere_r, name="sphere")
body = sphere & cube
body = body - part.cylinder((0, 0, -hole_h / 2), hole_r, hole_h, axis="z", name="hole_z")
body = body - part.cylinder((-hole_h / 2, 0, 0), hole_r, hole_h, axis="x", name="hole_x")
body = body - part.cylinder((0, -hole_h / 2, 0), hole_r, hole_h, axis="y", name="hole_y")

print(body)
body.show(title="camber CSG")
