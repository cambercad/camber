"""CadQuery gallery: third 'Extruding until a given face' example."""

def build():
    """Build and return this CadQuery gallery sample."""
    from camber import cqcompat as cq

    sphere = cq.Workplane().sphere(5)
    base = cq.Workplane(origin=(0, 0, -2)).box(12, 12, 10).cut(sphere).edges("|Z").fillet(2)
    sphere_face = base.faces(">>X[2] and (not |Z) and (not |Y)").val()
    base = base.faces("<Z").workplane().circle(2).extrude(10)

    shaft = cq.Workplane().sphere(4.5).circle(1.5).extrude(20)
    spherical_joint = (
        base.union(shaft)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .move(0, 4)
        .slot2D(10, 2, 90)
        .cutBlind(sphere_face)
        .workplane(offset=10)
        .move(0, 2)
        .circle(0.9)
        .extrude("next")
    )

    result = spherical_joint
    assert result.val().is_watertight()

    return result


if __name__ == "__main__":
    result = build()
    result.show(title='CadQuery gallery 40 - spherical joint until face')
