"""CadQuery gallery: A Parametric Enclosure (CamberCAD bridge)."""

def _build_components():
    """Build enclosure pieces and intermediate stages."""
    from camber.cqcompat import Workplane

    # Parameter definitions (same default dimensions as CadQuery's example).
    p_outerWidth = 100.0
    p_outerLength = 150.0
    p_outerHeight = 50.0
    p_thickness = 3.0
    p_sideRadius = 10.0
    p_topAndBottomRadius = 2.0
    p_screwpostInset = 12.0
    p_screwpostID = 4.0
    p_screwpostOD = 10.0
    p_boreDiameter = 8.0
    p_boreDepth = 1.0
    p_countersinkDiameter = 0.0
    p_countersinkAngle = 90.0
    p_flipLid = True
    p_lipHeight = 1.0

    # Outer rounded shell. Fillet order matters when the vertical radius is larger.
    oshell = (
        Workplane("XY", size=100)
        .rect(p_outerWidth, p_outerLength)
        .extrude(p_outerHeight + p_lipHeight)
    )
    if p_sideRadius > p_topAndBottomRadius:
        oshell = oshell.edges("|Z").fillet(p_sideRadius)
        oshell = oshell.edges("#Z").fillet(p_topAndBottomRadius)
    else:
        oshell = oshell.edges("#Z").fillet(p_topAndBottomRadius)
        oshell = oshell.edges("|Z").fillet(p_sideRadius)

    # Hollow out the enclosure, retaining its bottom and top wall.
    ishell = (
        oshell.faces("<Z").workplane(p_thickness, True)
        .rect(p_outerWidth - 2 * p_thickness, p_outerLength - 2 * p_thickness)
        .extrude(p_outerHeight - 2 * p_thickness, combine=False)
    )
    ishell = ishell.edges("|Z").fillet(p_sideRadius - p_thickness)
    shellBody = oshell.cut(ishell)
    box = shellBody

    # Four hollow screw posts run from the enclosure floor to the lid split plane.
    postWidth = p_outerWidth - 2 * p_screwpostInset
    postLength = p_outerLength - 2 * p_screwpostInset
    box = (
        box.faces(">Z").workplane(-p_thickness)
        .rect(postWidth, postLength, forConstruction=True)
        .vertices()
        .circle(p_screwpostOD / 2)
        .circle(p_screwpostID / 2)
        .extrude(-(p_outerHeight + p_lipHeight - p_thickness), True)
    )

    # Split the top cover from the body, form its locating lip, and move it beside
    # the enclosure so both manufactured pieces can be inspected together.
    lid, bottom = (
        box.faces(">Z").workplane(-p_thickness - p_lipHeight)
        .split(keepTop=True, keepBottom=True)
        .all()
    )
    lowerLid = lid.translate((0, 0, -p_lipHeight))
    cutlip = lowerLid.cut(bottom).translate((p_outerWidth + p_thickness, 0,
                                              p_thickness - p_outerHeight + p_lipHeight))

    # Matching screw locations and counterbores in the cover.
    topOfLidCenters = (
        cutlip.faces(">Z").workplane()
        .rect(postWidth, postLength, forConstruction=True)
        .vertices()
    )
    if p_boreDiameter > 0 and p_boreDepth > 0:
        topOfLid = topOfLidCenters.cboreHole(
            p_screwpostID, p_boreDiameter, p_boreDepth, 2 * p_thickness)
    elif p_countersinkDiameter > 0 and p_countersinkAngle > 0:
        topOfLid = topOfLidCenters.cskHole(
            p_screwpostID, p_countersinkDiameter, p_countersinkAngle, 2 * p_thickness)
    else:
        topOfLid = topOfLidCenters.hole(p_screwpostID, 2 * p_thickness)

    if p_flipLid:
        topOfLid = topOfLid.rotateAboutCenter((1, 0, 0), 180)

    # The result contains the separate lid and bottom solids, as in CadQuery.
    result = topOfLid.union(bottom)
    assert box.val().volume() > shellBody.val().volume()  # Screw posts add material.
    assert bottom.val().is_watertight() and bottom.val().volume() > 0
    assert topOfLid.val().is_watertight() and topOfLid.val().volume() > 0
    assert result.val().is_watertight() and result.val().volume() > 0

    return {"bottom": bottom, "topOfLid": topOfLid, "shellBody": shellBody, "box": box, "cutlip": cutlip, "result": result}


def build():
    """Build and return the complete two-piece enclosure."""
    return _build_components()["result"]


def build_components():
    """Return the enclosure pieces and intermediate design stages."""
    return _build_components()


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 35 - parametric enclosure')
