"""Three rolled beams with their upper ridges rounded into a six-arm star.

Run this file directly to open the finished part in the CamberCAD viewer.
"""
import math

from camber import Frame, Part


def _beam_frame(angle: float, roll: float, length: float,
                width: float, height: float) -> Frame:
    """Frame for a centered beam, rolled locally then spaced about world Y."""
    c, s = math.cos(angle), math.sin(angle)
    cr, sr = math.cos(roll), math.sin(roll)
    x = (c, 0.0, s)
    y = (-s * sr, cr, c * sr)
    z = (-s * cr, -sr, c * cr)
    origin = tuple(-length / 2 * x[i] - width / 2 * y[i] - height / 2 * z[i]
                   for i in range(3))
    return Frame(origin, x=x, y=y, z=z)


def build():
    """Create the three-beam star with all three highest ridges filleted."""
    length, width, height = 40.0, 8.0, 8.0
    roll = math.radians(45)
    part = Part((-30, -30, -30), (30, 30, 30), tolerance=.02)

    star = None
    for index in range(3):
        beam_index = index + 1
        frame = _beam_frame(index * 2 * math.pi / 3, roll,
                            length, width, height)
        beam = part.cuboid(frame, (length, width, height),
                           name="beam_{}".format(beam_index))

        # For this fixed 45° section orientation, the maximum-Y ridge is the
        # Line3 / ExtrudeBottom boundary. The long edge appears as two visible
        # arms after the three beams are united.
        ridge = "beam_{}:[beam_{}-Line3,beam_{}-ExtrudeBottom]".format(
            beam_index, beam_index, beam_index)
        beam = beam.fillet(ridge, .8, name="rounded_beam_{}".format(beam_index))
        star = beam if star is None else part.union(
            star, beam, name="three_beam_star_{}".format(beam_index))

    assert star is not None and star.is_watertight()
    assert star.volume() > 0
    return star


if __name__ == "__main__":
    build().show(title="Three rolled beams — rounded six-arm star")
