from camber import Part, Frame, Curve3D, LoftOptions


def build_model():
    # camber:begin setup
    part = Part(low=(-50, -50, -50), high=(50, 50, 50), tolerance=0.001)
    # camber:end setup

    # camber:begin sketch1
    sketch1 = part.sketch(plane='xy', name='sketch1', origin_name=None, frame=None, constrained=True)
    sketch1.add_line(sketch1 @ "origin", (30, 0), name="Line1")
    sketch1.add_line((30, 0), (30, 20), name="Line2")
    sketch1.add_line((30, 20), (0, 20), name="Line3")
    sketch1.add_line((0, 20), sketch1 @ "origin", name="Line4")
    sketch1.coincident("Line1@1.000", "Line2@0.000")
    sketch1.coincident("Line2@1.000", "Line3@0.000")
    sketch1.coincident("Line3@1.000", "Line4@0.000")
    sketch1.coincident("Line4@1.000", "Line1@0.000")
    sketch1.parallel("Line1", "Line3")
    sketch1.parallel("Line2", "Line4")
    sketch1.perpendicular("Line1", "Line2")
    sketch1.coincident("Line1@0.000", sketch1 @ "origin")
    sketch1.horizontal("Line1")
    sketch1.length("Line2", 20)
    sketch1.length("Line1", 30)
    sketch1.solve()
    # camber:end sketch1

    # camber:begin extrude1
    extrude1 = sketch1.extrude(height=15, name='extrude1', both_sides=False, max_deviation=-1, twist=0.017453292519943295, taper_angle=0)
    # camber:end extrude1

    # camber:begin fillet1
    fillet1 = extrude1.fillet(edges=('extrude1:[extrude1-Line1,extrude1-ExtrudeTop]', 'extrude1:[extrude1-Line4,extrude1-ExtrudeTop]', 'extrude1:[extrude1-Line3,extrude1-ExtrudeTop]', 'extrude1:[extrude1-Line2,extrude1-ExtrudeTop]'), radius=5, name='fillet1', max_deviation=-1)
    # camber:end fillet1

    return part, fillet1
