"""Hydrofoil wing: extruded NACA 4412 in metres. Saves STL next to the script.

Geometry matches GeoScriptViewer/TestScriptNacaWingExtrude.py. CFD / VTK
streamlines stay in the viewer script (GeoAPIEx is not part of camber).
"""
import math
import os

from camber import Frame, Part, vec3

naca = "4412"
chord = 0.12
span = 2.5
aoa_deg = 5.0
max_dev = 0.0025

wing_margin = 0.02
part = Part(
    vec3(-wing_margin, -0.03, -wing_margin),
    vec3(chord + wing_margin, 0.03, span + wing_margin),
    tolerance=max_dev,
)

# CSG ToSketch uses CCW chordAngle; positive AoA (nose up, freestream +X) needs TE below LE.
chord_angle = -aoa_deg * math.pi / 180.0
sketch = part.sketch(frame=Frame.from_plane((0, 0, 0), (0, 0, 1), (1, 0, 0), (0, 1, 0)), name="naca_section")
sketch.add_naca4(naca, (0, 0), chord, chord_angle, samples_per_side=36)

wing = part.extrude(sketch, span, name="hydrofoil_4412", max_deviation=max_dev)
print("Created extruded NACA", naca, "wing:", wing)
print("chord=", chord, "m  span=", span, "m  AoA=", aoa_deg, "deg  max_dev=", max_dev, "m")

stl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hydrofoil_4412.stl")
wing.save_stl(stl_path)
print("Exported STL:", stl_path)
wing.show(title="NACA {0} hydrofoil".format(naca))
