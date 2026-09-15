"""Triangulate a sketch into an open sheet and show it next to a volume.

Closed sketch loops become a planar fill (``is_volume`` is False). The cube is
a watertight solid so the viewer has to draw both kinds of mesh.

From Geo.Python, venv active:

  .\\publish-wheel.ps1
  python -m pip install --force-reinstall (Get-ChildItem dist\\camber*.whl | Select-Object -Last 1).FullName
  python python\\sketch_triangulate.py
"""
from camber import Frame, Part, vec3

part = Part(vec3(-12), vec3(12), tolerance=0.01)

sk = part.sketch("xy", name="washer")
sk.add_rectangle((-4, -4), (4, 4))
sk.add_circle((0, 0), 1.6)
sheet = sk.surface(name="washer")

# Standing sheet on XZ (not a volume either).
wall = part.sketch("xz", name="wall")
wall.add_rectangle((2, 0), (6, 4))
wall_sheet = wall.surface(name="wall")

block = part.cube(Frame((8, 0, 1)), 2.0, name="block")

print(sheet, "watertight" if sheet.is_volume else "open sheet")
print(wall_sheet, "watertight" if wall_sheet.is_volume else "open sheet")
print(block, "watertight" if block.is_volume else "open sheet")
part.show(title="sketch triangulate")
