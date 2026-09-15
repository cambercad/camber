"""Cubic Hermite spline plus a construction line — GeoScriptViewer/SplineTest.py."""
from camber import Part, vec3

part = Part(vec3(-0.5), vec3(0.5), tolerance=1e-4)
sk = part.sketch("xy", constrained=True, name="Sketch1")
sk.add_spline([
    (-0.6666, -0.1268),
    (-0.5252, 0.5354),
    (0.1504, 0.5421),
    (0.4871, 0.349),
    (0.4018, -0.2458),
    (-0.3412, -0.6566),
])
sk.add_line((-0.2245, -0.0415), (-0.211, 0.4164), name="line1", construction=True)
sk.solve()
print(sk)
sk.show(title="spline test")
