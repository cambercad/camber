"""Intersect a diagonally tilted square beam with a centered cube.

Run directly to inspect the resulting solid in the CamberCAD viewer.
Dimensions are in millimetres.
"""

import math

from camber import Frame, Part


def build(tolerance=0.01):
    beam_length = 40.0
    beam_side = 8.0
    cube_side = beam_length / 2.0
    roll = math.radians(45.0)
    tilt = math.radians(45.0)
    cr, sr = math.cos(roll), math.sin(roll)
    ct, st = math.cos(tilt), math.sin(tilt)

    # The beam starts along global X and is centered at the origin. First roll
    # about its long axis (global X), then rotate about global Y: Ry(tilt)*Rx(roll).
    x_axis = (ct, 0.0, -st)
    y_axis = (st * sr, cr, ct * sr)
    z_axis = (st * cr, -sr, ct * cr)
    half_l = beam_length / 2.0
    half_s = beam_side / 2.0
    origin = tuple(-half_l * x_axis[i] - half_s * y_axis[i] - half_s * z_axis[i]
                   for i in range(3))
    beam_frame = Frame(origin, x=x_axis, y=y_axis, z=z_axis)
    part = Part((-25, -25, -25), (25, 25, 25), tolerance=tolerance)
    beam = part.cuboid(beam_frame, (beam_length, beam_side, beam_side), name="tilted_beam")
    half_cube = cube_side / 2.0
    cube = part.cube((-half_cube, -half_cube, -half_cube), cube_side,
                     name="clipping_cube")
    result = part.intersect(beam, cube, name="beam_cube_intersection")
    result = result.fillet(
        "beam_cube_intersection:[tilted_beam-Line1,tilted_beam-ExtrudeTop]",
        0.5,
        name="rounded_beam_cube_intersection",
    )

    assert result.is_watertight()
    assert 0 < result.volume() < beam.volume()
    return result


if __name__ == "__main__":
    build().show(title="Rolled beam intersection with one rounded edge")
