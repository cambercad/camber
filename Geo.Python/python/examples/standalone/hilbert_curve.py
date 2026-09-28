"""3D Hilbert curve as a thickened path of axis-aligned cuboids.

Same Skilling transpose / Gray decode as Geo.HilbertCurve3D. Geometry is
CreateCuboid + BatchUnion. Opens Polyscope.

level=2 -> 63 segments (fast); 3 -> 511 segments; 4 -> 4095.
"""
from camber import Part, vec3

level = 3
size = 10.0
short_edge = 0.5


def hilbert_axes(index, bits):
    mask = (1 << bits) - 1
    x = y = z = 0
    for j in range(bits):
        x |= ((index >> (j * 3 + 2)) & 1) << j
        y |= ((index >> (j * 3 + 1)) & 1) << j
        z |= ((index >> (j * 3 + 0)) & 1) << j
    x &= mask
    y &= mask
    z &= mask

    gray = z >> 1
    z ^= y
    y ^= x
    x ^= gray

    n = 1 << bits
    q = 1
    while q != n:
        p = q - 1
        for dim in (2, 1, 0):
            d = z if dim == 2 else (y if dim == 1 else x)
            if d & q:
                x ^= p
            else:
                t = (x ^ d) & p
                x ^= t
                if dim == 2:
                    z ^= t
                elif dim == 1:
                    y ^= t
        q *= 2
    return x, y, z


def generate_hilbert_points(level, size):
    bits = level
    axis_count = 1 << bits
    total_points = axis_count * axis_count * axis_count
    spacing = 0.0 if axis_count == 1 else size / float(axis_count - 1)
    points = []
    for idx in range(total_points):
        x, y, z = hilbert_axes(idx, bits)
        points.append(vec3(x * spacing, y * spacing, z * spacing))
    return points


def thickened_segment(part, a, b, short_edge_len, name):
    half = 0.5 * short_edge_len
    mn = vec3(min(a.x, b.x) - half, min(a.y, b.y) - half, min(a.z, b.z) - half)
    mx = vec3(max(a.x, b.x) + half, max(a.y, b.y) + half, max(a.z, b.z) + half)
    return part.cuboid(mn, mx, name=name)


margin = max(2.0, 0.5 * short_edge + 1.0)
part = Part(vec3(-margin), vec3(size + margin), tolerance=1e-4)

points = generate_hilbert_points(level, size)
segments = []
for i in range(1, len(points)):
    segments.append(thickened_segment(part, points[i - 1], points[i], short_edge, "seg{0}".format(i)))

hilbert = part.batch_union(segments)
print("hilbert L{0} segments={1} {2}".format(level, len(segments), hilbert))
hilbert.show(title="camber hilbert curve")
