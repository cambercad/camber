"""TrueType glyph outlines in a sketch, extruded into a nameplate.

Letters are sketched on OriginXY (baseline at origin) and fused onto a plate
so they stand proud of the face. Family falls back to Arial / Liberation /
DejaVu if the requested face is missing. CFF-only .otf fonts are skipped.

From Geo.Python, venv active (rebuild if add_text is missing):

  .\\publish-wheel.ps1
  python -m pip install --force-reinstall (Get-ChildItem dist\\camber*.whl | Select-Object -Last 1).FullName
  python python\\sketch_text.py
"""
from camber import Part, vec3

label = "CAMBER"
family = "Segoe UI"
em_size = 10.0
plate_half_x = 40.0
plate_half_y = 12.0
plate_h = 3.0
letter_h = 4.5
baseline = (-32.0, -4.0)

part = Part(vec3(-80), vec3(80), tolerance=0.05)

plate_sk = part.sketch("xy", name="plate")
plate_sk.add_rectangle((-plate_half_x, -plate_half_y), (plate_half_x, plate_half_y))
plate = part.extrude(plate_sk, plate_h, name="plate")

text_sk = part.sketch("xy", name="text")
text_sk.add_text(label, origin=baseline, family=family, em_size=em_size)
letters = part.extrude(text_sk, letter_h, name="letters")

body = plate + letters
print(body)
body.show(title="sketch text")
