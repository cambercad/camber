"""CadQuery gallery: Panel With Various Connector Holes.

All dimensions, connector profiles, and placements match CadQuery's example.
The tool solids are subtracted together because Camber's cap triangulator
currently expects a single outer loop per extrusion sketch.
"""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber.cqcompat import Workplane


    width, height, thickness = 400.0, 500.0, 2.0
    h_sep = 60.0
    h_sep4DB9 = 30.0
    panel = Workplane("front").box(width, height, thickness)
    cutters = []


    def add_through_tool(profile):
        """Extrude one closed profile past both panel faces and queue the cutter."""
        cutters.append(profile.extrude(thickness + 0.1, both=True, combine=False).val())


    def add_d_sub_rows(x, first_y, count, pitch, pin_half_span, side_x, taper_x):
        """Add repeated D-sub apertures and their two mounting holes."""
        for idx in range(count):
            y = first_y - idx * pitch
            for pin_x in (-pin_half_span, pin_half_span):
                add_through_tool(
                    panel.workplane(offset=thickness / 2, centerOption="CenterOfBoundBox")
                    .center(x + pin_x, y).circle(1.6)
                )

            add_through_tool(
                panel.workplane(offset=thickness / 2, centerOption="CenterOfBoundBox")
                .center(x, y)
                .moveTo(-side_x + 3.4, -5.7)
                .threePointArc((-side_x + 0.995836, -4.70416), (-side_x, -2.3))
                .lineTo(-taper_x, 2.3)
                .threePointArc((-taper_x + 0.99584, 4.70416), (-taper_x + 3.4, 5.7))
                .lineTo(taper_x - 3.4, 5.7)
                .threePointArc((taper_x - 0.99584, 4.70416), (taper_x, 2.3))
                .lineTo(side_x, -2.3)
                .threePointArc((side_x - 0.995836, -4.70416), (side_x - 3.4, -5.7))
                .close()
            )


    # 12 wide-pin, 12 narrow-pin, and 24 DB9 connector outlines.
    for x in (157, 25, -107):
        add_d_sub_rows(x, 210, 4, h_sep, 23.5, 20.438896, 21.25)
        add_d_sub_rows(x, -30, 4, h_sep, 16.65, 13.5889, 14.4)

    for x in (91, -41, -173):
        add_d_sub_rows(x, 225, 8, h_sep4DB9, 12.5, 9.438896, 10.25)

    # Four circular four-pin connectors with a square pattern of mounting holes.
    for idx in range(4):
        x, y = -107, -30 - idx * h_sep
        add_through_tool(
            panel.workplane(offset=thickness / 2, centerOption="CenterOfBoundBox")
            .center(x, y).circle(14)
        )
        for dx in (-12.37435, 12.37435):
            for dy in (-12.37435, 12.37435):
                add_through_tool(
                    panel.workplane(offset=thickness / 2, centerOption="CenterOfBoundBox")
                    .center(x + dx, y + dy).circle(1.6)
                )

    # Four final compact connector openings, each bounded by two circular arcs.
    for idx in range(4):
        add_through_tool(
            panel.workplane(offset=thickness / 2, centerOption="CenterOfBoundBox")
            .center(-173, -30 - idx * h_sep)
            .moveTo(-2.9176, -5.3)
            .threePointArc((-6.05, 0), (-2.9176, 5.3))
            .lineTo(2.9176, 5.3)
            .threePointArc((6.05, 0), (2.9176, -5.3))
            .close()
        )

    result = Workplane(panel, obj=panel.part.batch_subtract(panel.val(), cutters))
    assert len(cutters) == 168  # all connector profiles, pins, and mounting holes
    assert result.val().is_watertight()
    assert result.val().volume() < width * height * thickness

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 38 - connector panel')
