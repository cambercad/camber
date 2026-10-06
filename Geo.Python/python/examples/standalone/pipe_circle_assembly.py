"""90-degree pipes mated from stacked identity (no pose seeds).

Pipe 1 is the only ground. Each next pipe is added at the origin, mated, and
solved; previous flange mates keep the seated chain in place.

Each joint mates the named pipe end faces and two flange side faces. The end
faces seat the flanges; the north and east faces align their bolt patterns.
"""
from camber import Part, vec3
from pipe import create_pipe

PIPE_COUNT = 4

# Keep the same operating lattice as pipe_90deg.py/create_pipe(). Geometry
# construction is quantized to the Part box, so changing this box changes the
# final triangulation even when all nominal dimensions are identical.
part = Part(vec3(-1000), vec3(1000), tolerance=0.1)
pipes = [create_pipe(part, "pipe{0}".format(i)) for i in range(1, PIPE_COUNT + 1)]

assembly = part.assembly("pipe_circle")
assembly.solve_after_every_constraint = False

bodies = [assembly.add_part(pipes[0])]
assembly.fix(bodies[0])


def mate_flanged_ends(top_index, bottom_index):
    top = bodies[top_index]
    bottom = bodies[bottom_index]
    top_name = pipes[top_index].name
    bottom_name = pipes[bottom_index].name

    assembly.coincident(
        top.plane(top_name + "-ExtrudeTop"),
        bottom.plane(bottom_name + "-ExtrudeBottom"),
        opposite_normals=True,
    )
    assembly.coincident(
        top.plane(top_name + "-top_flange-north"),
        bottom.plane(bottom_name + "-bottom_flange-north"),
        opposite_normals=False,
    )
    assembly.coincident(
        top.plane(top_name + "-top_flange-east"),
        bottom.plane(bottom_name + "-bottom_flange-east"),
        opposite_normals=False,
    )


for i in range(1, PIPE_COUNT):
    bodies.append(assembly.add_part(pipes[i]))
    mate_flanged_ends(i - 1, i)
    result = assembly.solve()
    if not result.converged:
        raise RuntimeError(result)
    print("after joint {0}->{1}: {2}".format(i, i + 1, [b.pose for b in bodies]))

if PIPE_COUNT == 4:
    mate_flanged_ends(3, 0)
    result = assembly.solve()
    if not result.converged:
        raise RuntimeError(result)
    print("after close: {0}".format([b.pose for b in bodies]))

assembly.show(title="Camber")
