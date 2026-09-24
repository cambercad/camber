r"""Shell feature showcase: cup, bowl, twisted loft, and concave L-shaped prism.

Run from ``Geo.Python`` after rebuilding/installing the wheel::

    python python\shell_showcase.py

Use ``--no-show`` for a headless construction check.  In the viewer, click the
``ShellInner_*`` and ``ShellRim_*`` patches to inspect shell topology and names.
"""

import argparse
import math
import time

from camber import Frame, Part


def twisted_rectangle(part, name, center_x, z, width, depth, angle_degrees):
    angle = math.radians(angle_degrees)
    frame = Frame((center_x, 0, z),
                  x=(math.cos(angle), math.sin(angle), 0),
                  y=(-math.sin(angle), math.cos(angle), 0),
                  z=(0, 0, 1))
    sketch = part.sketch(frame=frame, name=name)
    sketch.add_rectangle((-width / 2, -depth / 2), (width / 2, depth / 2),
                         names=("south", "east", "north", "west"))
    return sketch


def build_showcase():
    part = Part((-32, -34, -3), (32, 18, 24), tolerance=.015)

    # Exact cylinder/plane adapters. Round the cavity-side rim after shelling to
    # demonstrate that shell rims are ordinary selectable feature edges.
    cup = part.cylinder((-18, 0, 0), 6.5, 13, name="cup", max_deviation=.02)
    cup = part.shell(cup, .8, faces="cup-ExtrudeTop", name="cup_shell")
    inner_rims = [name for name in cup.curve_names
                  if "ShellRim_" in name and "ShellInner_" in name]
    if not inner_rims:
        raise RuntimeError("The cup shell did not publish a cavity-side rim edge")
    cup = part.fillet(cup, inner_rims[0], .35, name="cup_finished", max_deviation=.02)

    # Exact spherical/plane junction after a CSG intersection.
    sphere = part.sphere((0, 0, 6), 7, name="bowl_sphere", max_deviation=.02)
    upper = part.cuboid((-8, -8, 6), (8, 8, 16), name="bowl_clip")
    hemisphere = part.intersect(sphere, upper, name="hemisphere")
    bowl = part.shell(hemisphere, .65, faces="bowl_clip-ExtrudeBottom", name="sphere_bowl")

    # Four rotated sections form NURBS/mesh-backed sides. The shell fallback
    # offsets their existing triangulation and miters all shared support tangents.
    stations = (
        (0, 10.0, 7.0, 0),
        (5, 12.0, 8.0, 12),
        (10, 8.5, 6.0, 31),
        (16, 10.5, 7.5, 52),
    )
    sections = [twisted_rectangle(part, "vessel_section_{0}".format(i + 1),
                                   18, z, width, depth, angle)
                for i, (z, width, depth, angle) in enumerate(stations)]
    vessel = part.loft(sections, first_curves=["south"] * len(sections),
                       name="twisted_vessel", max_deviation=.025)
    vessel = part.shell(vessel, .15, faces="twisted_vessel-EndCap",
                        name="twisted_vessel_shell", max_deviation=.025)

    # One sketched L profile gives the shell both convex corners and a concave
    # re-entrant corner; remove its single top face to leave an L-shaped cup.
    l_sketch = part.sketch(name="l_profile")
    l_points = ((-27, -31), (-13, -31), (-13, -27),
                (-23, -27), (-23, -17), (-27, -17))
    for i, start in enumerate(l_points):
        l_sketch.add_line(start, l_points[(i + 1) % len(l_points)],
                          name="l_edge_{0}".format(i + 1))
    l_prism = part.extrude(l_sketch, 10, name="l_prism")
    l_shell = part.shell(l_prism, .6, faces="l_prism-ExtrudeTop",
                         name="l_shaped_shell")

    for label, solid in (("filleted analytic cup", cup),
                         ("spherical CSG bowl", bowl),
                         ("twisted mesh-offset vessel", vessel),
                         ("concave L-shaped shell", l_shell)):
        if not solid.is_watertight() or solid.volume() <= 0:
            raise RuntimeError("{0} is not a positive watertight solid".format(label))
        shell_patches = [name for name in solid.patch_names
                         if "ShellInner_" in name or "ShellRim_" in name]
        print("{0}: {1} triangles, volume {2:.3f}".format(
            label, solid.triangle_count, solid.volume()))
        print("  shell patches: " + ", ".join(shell_patches))

    gallery = part.batch_union([cup, bowl, vessel, l_shell])
    if not gallery.is_watertight():
        raise RuntimeError("Combined shell gallery is not watertight")
    return gallery


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-show", action="store_true", help="build and validate without opening the viewer")
    args = parser.parse_args()
    started = time.perf_counter()
    gallery = build_showcase()
    print("Shell showcase built in {0:.3f}s; final triangles: {1}".format(
        time.perf_counter() - started, gallery.triangle_count))
    if not args.no_show:
        gallery.show(title="Camber shell showcase - analytic, CSG, and mesh fallback")


if __name__ == "__main__":
    main()
