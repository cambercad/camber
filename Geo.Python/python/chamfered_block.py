"""Mounting block with symmetric edge chamfers — GeoScriptViewer/TestScriptChamferedBlock.py.

Edge names match Fillet: \"[patchA,patchB]\" between two adjacent surface patches.
"""
from camber import Part, vec3

base_w = 80.0
base_d = 60.0
base_h = 20.0
lip_h = 8.0
chamfer_d = 4.0

part = Part(vec3(-100, -80, -20), vec3(100, 80, 40), tolerance=0.01)

base_sk = part.sketch("xy", name="base_profile")
base_sk.add_rectangle((0, 0), (base_w, base_d))
base = part.extrude_two_sides(base_sk, base_h * 0.5, base_h * 0.5, name="base")

lip_sk = part.sketch("base-ExtrudeTop", name="lip_profile")
lip_sk.add_rectangle((8, 8), (base_w - 8, base_d - 8))
lip = part.extrude_two_sides(lip_sk, lip_h, 0.0, name="lip")
block = part.union(base, lip, name="block")

lip_top_edges = [
    "[lip-Line1,lip-ExtrudeTop]",
    "[lip-Line2,lip-ExtrudeTop]",
    "[lip-Line3,lip-ExtrudeTop]",
    "[lip-Line4,lip-ExtrudeTop]",
]
block = part.chamfer(block, lip_top_edges, chamfer_d)
block = part.chamfer(block, ["[base-Line3,base-Line4]"], chamfer_d * 0.75)
print(block)
block.show(title="chamfered block")
