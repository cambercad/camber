"""Level-n Menger sponge — sequential union variant (for debugging).

Same leaf cuboids as menger_sponge.py, but unions one cuboid at a time via
part.union instead of batch_union. Each step is named (union_1, union_2, …).

iterations=1 -> 20 cuboids (fast); 2 -> 400; 3 -> 8000 (very slow).
"""
from camber import Part, vec3

iterations = 2
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
                collect_menger_leaves(
                    x + i * sub, y + j * sub, z + k * sub, sub, depth + 1, max_depth, out)


margin = max(1.0, 0.05 * size)
part = Part(vec3(-margin), vec3(size + margin), tolerance=1e-4)

leaves = []
collect_menger_leaves(0.0, 0.0, 0.0, size, 0, iterations, leaves)

leaf0 = leaves[0]
sponge = part.cuboid(
    vec3(leaf0[0], leaf0[1], leaf0[2]),
    vec3(leaf0[0] + leaf0[3], leaf0[1] + leaf0[3], leaf0[2] + leaf0[3]),
    name="menger_0",
)
for step in range(1, len(leaves)):
    leaf = leaves[step]
    cube = part.cuboid(
        vec3(leaf[0], leaf[1], leaf[2]),
        vec3(leaf[0] + leaf[3], leaf[1] + leaf[3], leaf[2] + leaf[3]),
        name="menger_{0}".format(step),
    )
    sponge = part.union(sponge, cube, name="union_{0}".format(step))

print("menger L{0} sequential leaves={1} {2}".format(iterations, len(leaves), sponge))
sponge.show(title="camber menger sequential")
