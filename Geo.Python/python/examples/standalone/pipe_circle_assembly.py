"""90-degree pipes mated from stacked identity (no pose seeds).

Pipe 1 is the only ground. Each next pipe is added at the origin, mated, and
solved; previous flange mates keep the seated chain in place.

Each joint uses end frames retained from the sweep's terminal sketch planes. The
boolean-fused flanges do not preserve the original terminal face names, so the
assembly uses explicit plane/axis datums at those same design references.
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
    top_frame = pipes[top_index]._pipe_end_frames[1]
    bottom_frame = pipes[bottom_index]._pipe_end_frames[0]

    assembly.coincident(
        top.plane_at(top_frame.origin, top_frame.z),
        bottom.plane_at(bottom_frame.origin, bottom_frame.z),
        opposite_normals=True,
    )
    assembly.coincident(
        top.plane_at(top_frame.origin + top_frame.y * 180.0, top_frame.y),
        bottom.plane_at(bottom_frame.origin + bottom_frame.y * 180.0, bottom_frame.y),
        opposite_normals=False,
    )
    assembly.concentric(
        top.axis_at(top_frame.origin, top_frame.z),
        bottom.axis_at(bottom_frame.origin, bottom_frame.z),
    )


for i in range(1, PIPE_COUNT):
    bodies.append(assembly.add_part(pipes[i]))
    mate_flanged_ends(i - 1, i)
    assembly.solve()
    print("after joint {0}->{1}: {2}".format(i, i + 1, [b.pose for b in bodies]))

if PIPE_COUNT == 4:
    mate_flanged_ends(3, 0)
    assembly.solve()
    print("after close: {0}".format([b.pose for b in bodies]))

assembly.show(title="Camber")
