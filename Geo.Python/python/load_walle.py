"""Load and display the WALL-E assembly reference mesh (Thingiverse 3703555, part 1 of 3).

Wall-E_Assembly_NotForPrinting.stl is a visualisation-only assembly (overlapping parts),
so require_watertight=False. Coordinates are millimetres.
"""
from camber import Part, vec3

stl_path = r"C:\Users\Besitzer\Documents\Meshes\WallE\WALL-E Robot Replica - 3703555 - part 1 of 3\files\Wall-E_Assembly_NotForPrinting.stl"

bbox_min = vec3(-278.0, 7.0, -82.0)
bbox_max = vec3(72.0, 298.0, 314.0)
part = Part(bbox_min, bbox_max, tolerance=0.01)

wall_e = part.load_stl(stl_path, 25.0, require_watertight=False)
print(wall_e)
wall_e.show(title="WALL-E")
