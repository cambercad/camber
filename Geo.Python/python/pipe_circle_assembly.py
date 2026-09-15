"""90-degree pipes mated from stacked identity (no pose seeds).

Pipe 1 is the only ground. Each next pipe is added at the origin, mated, and
solved; previous flange mates keep the seated chain in place.

Each joint uses named geometry only:
  * coincidence of the pipe terminal ExtrudeTop / ExtrudeBottom faces
  * same-normal coincidence of one pair of outer flange sides (clocks the square)
  * concentric inner-pipe cylinders (horizontal end vs vertical end) so the joint cannot slide
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
    top_name = "pipe{0}".format(top_index + 1)
    bottom_name = "pipe{0}".format(bottom_index + 1)
    top = bodies[top_index]
    bottom = bodies[bottom_index]

    assembly.coincident(
        top.plane(top_name + "-ExtrudeTop"),
        bottom.plane(bottom_name + "-ExtrudeBottom"),
        opposite_normals=False,
    )
    assembly.coincident(
        top.plane(top_name + "-top_flange-north"),
        bottom.plane(bottom_name + "-bottom_flange-north"),
        opposite_normals=False,
    )
    assembly.concentric(
        top.axis(top_name + "-inner-v_line"),
        bottom.axis(bottom_name + "-inner-h_line"),
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
