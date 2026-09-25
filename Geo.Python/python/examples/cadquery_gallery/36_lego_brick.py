"""CadQuery gallery: parametric rectangular Lego-style brick."""
from camber.cqcompat import Workplane


def make_brick(length_bumps=6, width_bumps=2, thin=True):
    """Build a brick with studs and the matching underside supports."""
    pitch, clearance = 8.0, 0.1
    stud_diameter, stud_height = 4.8, 1.8
    height = 3.2 if thin else 9.6
    wall = (pitch - 2 * clearance - stud_diameter) / 2
    post_diameter = pitch - wall
    length = length_bumps * pitch - 2 * clearance
    width = width_bumps * pitch - 2 * clearance

    result = (
        Workplane("XY").box(length, width, height)
        .faces("<Z").shell(-wall)
        .faces(">Z").workplane().rarray(
            pitch, pitch, length_bumps, width_bumps, center=True
        )
        .circle(stud_diameter / 2).extrude(stud_height)
    )

    # Multi-row bricks use hollow tubes; a single row uses thin ribs.
    # A 1x1 brick needs neither and stays as the shelled body with one stud.
    underside = result.faces("<Z").workplane(invert=True)
    if length_bumps > 1 and width_bumps > 1:
        result = (
            underside.rarray(
                pitch, pitch, length_bumps - 1, width_bumps - 1, center=True
            )
            .circle(post_diameter / 2)
            .circle(stud_diameter / 2)
            .extrude(height - wall)
        )
    elif length_bumps > 1:
        result = (
            underside.rarray(pitch, pitch, length_bumps - 1, 1, center=True)
            .circle(wall)
            .extrude(height - wall)
        )
    elif width_bumps > 1:
        result = (
            underside.rarray(pitch, pitch, 1, width_bumps - 1, center=True)
            .circle(wall)
            .extrude(height - wall)
        )

    assert result.val().is_watertight()
    return result


def build():
    """Build the default Lego brick sample."""
    return make_brick()


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 36 — Lego brick')
