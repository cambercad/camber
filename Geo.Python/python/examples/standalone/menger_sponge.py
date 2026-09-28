"""Level-n Menger sponge as a union of leaf cuboids. Opens Polyscope.

Fractal rule: subdivide into 3x3x3 and keep the 20 sub-cubes that are not
face-centre or body-centre holes (at most one axis index is the middle slot).

iterations=1 -> 20 cuboids (fast); 2 -> 400; 3 -> 8000 (very slow).
"""
from camber import Part, vec3

iterations = 3
size = 10.0


def collect_menger_leaves(x, y, z, cube_size, depth, max_depth, out):
    if depth == max_depth:
        out.append((x, y, z, cube_size))
        return

    sub = cube_size / 3.0
    for i in range(3):
        for j in range(3):
            for k in range(3):
                mid_count = (1 if i == 1 else 0) + (1 if j == 1 else 0) + (1 if k == 1 else 0)
                if mid_count > 1:
                    continue
                collect_menger_leaves(x + i * sub, y + j * sub, z + k * sub, sub, depth + 1, max_depth, out)


margin = max(1.0, 0.05 * size)
part = Part(vec3(-margin), vec3(size + margin), tolerance=1e-4)

leaves = []
collect_menger_leaves(0.0, 0.0, 0.0, size, 0, iterations, leaves)

cuboids = []
for idx, (x, y, z, cube_size) in enumerate(leaves):
    cuboids.append(
        part.cuboid(vec3(x, y, z), vec3(x + cube_size, y + cube_size, z + cube_size), name="menger_{0}".format(idx))
    )

sponge = part.batch_union(cuboids)
print("menger L{0} leaves={1} {2}".format(iterations, len(leaves), sponge))
sponge.show(title="camber menger sponge")
