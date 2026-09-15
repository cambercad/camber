"""Citroën-style herringbone (double helical) involute gear.

Tooth flanks are circle involutes, fitted to cubic Hermite splines at the part
tolerance (same idea as NACA airfoils — not a separate curve type).

Writes OBJ, STL, STEP, IGES, and USDA next to this script. From Geo.Python, venv
active:

  python python\\v_gear.py
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from camber import Part, vec3
from gears import herringbone, pitch_radius, tip_radius

module = 2.0
teeth = 24
pressure = math.radians(20.0)
helix = math.radians(30.0)
half_width = 12.0
bore_r = 8.0
outer = tip_radius(module, teeth) + 8.0

part = Part(vec3(-outer), vec3(outer), tolerance=0.04)
body = herringbone(
    part, "gear", module, teeth, half_width, helix, bore=bore_r, pressure=pressure)
print(body, "volume", body.volume(), "pitch r", pitch_radius(module, teeth))

stem = os.path.join(_HERE, "v_gear")
exports = (
    ("stl", body.save_stl),
    ("obj", body.save_obj),
    ("step", body.save_step),
    ("iges", body.save_iges),
    ("usda", body.save_usda),
)
for ext, save in exports:
    path = stem + "." + ext
    save(path)
    print("wrote", path)

body.show(title="herringbone gear")
