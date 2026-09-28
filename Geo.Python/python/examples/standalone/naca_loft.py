"""Three-section NACA 2412 loft — GeoScriptViewer/TestScriptNaca.py.

XY-aligned foils at z = 0, 35, 70 with a mid-span chord bump (1.5× base chord).
"""
from camber import Frame, LoftOptions, Part, vec3

part = Part(vec3(-20, -30, -10), vec3(200, 30, 100), tolerance=0.01)

spacing_z = 35.0
chord_base = 100.0
chord_mid = 1.5 * chord_base
max_dev = 0.03


def make_naca_sketch(z_world, chord_length, sketch_name):
    sk = part.sketch(frame=Frame.from_plane((0, 0, z_world), (0, 0, 1), (1, 0, 0), (0, 1, 0)), name=sketch_name)
    sk.add_naca4("2412", (0, 0), chord_length, samples_per_side=40)
    return sk


sketches = [
    make_naca_sketch(0.0, chord_base, "naca_loft_bottom"),
    make_naca_sketch(spacing_z, chord_mid, "naca_loft_middle"),
    make_naca_sketch(2.0 * spacing_z, chord_base, "naca_loft_top"),
]

wing = part.loft(sketches, LoftOptions(), name="naca_three_section_loft", max_deviation=max_dev)
print(wing)
wing.show(title="NACA 2412 loft")
