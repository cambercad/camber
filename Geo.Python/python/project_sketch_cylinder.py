"""Project a sketch onto a cylinder, extrude along surface normals, subtract.

Steps:
  1. Build a cylinder (host solid).
  2. Sketch a rounded rectangle on a plane facing the barrel.
  3. Project the sketch onto the cylinder along the sketch-plane normal.
  4. Extrude the projected curves along stored surface normals (into the solid)
     and subtract the cutter from the host.

From Geo.Python, venv active:

  .\\publish-wheel.ps1
  python -m pip install --force-reinstall (Get-ChildItem dist\\camber*.whl | Select-Object -Last 1).FullName
  python python\\project_sketch_cylinder.py
"""
from camber import Frame, Part, vec3

part = Part(vec3(-40), vec3(40), tolerance=0.02)

# Host cylinder along Z.
cyl_r = 12.0
cyl_h = 40.0
host = part.cylinder((0, 0, -cyl_h / 2), cyl_r, cyl_h, axis="z", name="host")

# Sketch plane facing the barrel: local +Z points toward the axis (-world X).
sketch_plane = Frame(
    origin=(cyl_r + 8.0, 0, 0),
    x=(0, 1, 0),
    y=(0, 0, -1),
    z=(-1, 0, 0),
)
sk = part.sketch(frame=sketch_plane, name="feature_sk")
# Rectangle in sketch XY (Y = world Z, X = world Y).
half_w = 4.0
half_h = 8.0
sk.add_rectangle((-half_w, -half_h), (half_w, half_h), names=("south", "east", "north", "west"))

projected = part.project_sketch(sk, host, name="on_cyl")
print(projected)

# Negative height follows normals into the solid (pocket).
cutter = part.extrude_projected(projected, height=-2.5, name="cutter")
body = host - cutter

print(body)
body.show(title="project sketch onto cylinder")
