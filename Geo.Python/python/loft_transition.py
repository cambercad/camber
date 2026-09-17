"""Four constrained rectangular sections lofted into a twisted transition.

Expected topology: four named side faces and two end caps. Click individual
faces in the viewer. Use --sections to also display the authored sketches.
"""

import argparse
import math
import time
from pathlib import Path

from camber import Frame, Part, render_views, set_progress_log, show

# name, origin, width, height, rotation about the section normal (degrees)
STATIONS = (
    ("inlet", (0, 0, 0), 30, 18, 0),
    ("shoulder", (8, 0, 30), 38, 24, 15),
    ("waist", (-4, 6, 65), 22, 16, 40),
    ("outlet", (4, 8, 100), 14, 10, 65),
)
SIDE_NAMES = ("bottom", "right", "top", "left")


def section(part, name, origin, width, height, angle):
    angle = math.radians(angle)
    frame = Frame(origin, x=(math.cos(angle), math.sin(angle), 0),
                  y=(-math.sin(angle), math.cos(angle), 0), z=(0, 0, 1))
    sketch = part.sketch(name=name, frame=frame, constrained=True)
    sketch.solve_after_every_constraint = False
    sketch.add_rectangle((-width / 2, -height / 2), (width / 2, height / 2), names=SIDE_NAMES)
    sketch.fix("bottom@0")
    sketch.horizontal("bottom").horizontal("top")
    sketch.vertical("right").vertical("left")
    sketch.length("bottom", width).length("right", height)
    sketch.solve()
    return sketch


def build_transition():
    set_progress_log(False)
    part = Part((-40, -40, -10), (40, 40, 110), tolerance=.04)
    sections = [section(part, *station) for station in STATIONS]
    # Named first curves define both correspondence and traversal direction.
    solid = part.loft(sections, first_curves=["bottom"] * len(sections), name="transition")
    if not solid.is_watertight() or solid.volume() <= 0:
        raise ValueError("The transition must be a closed solid with positive volume.")
    return part, solid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--sections", action="store_true", help="also show the four section sketches")
    parser.add_argument("--render", type=Path, help="write a multiview image instead of opening the viewer")
    args = parser.parse_args()
    started = time.perf_counter()
    part, solid = build_transition()
    print(f"Built in {time.perf_counter() - started:.3f} s; watertight; volume {solid.volume():.1f} mm³")
    print("Expected side faces: " + ", ".join("transition-Side-" + name for name in SIDE_NAMES))
    print("Expected end faces: transition-StartCap, transition-EndCap")
    model = part if args.sections else solid
    if args.render:
        render_views(model, args.render, views={
            "Isometric": ((1, -1, .65), (0, 0, 1)),
            "Front": ((0, -1, 0), (0, 0, 1)),
            "Top": ((0, 0, 1), (0, 1, 0)),
        })
    if args.show if args.show is not None else not args.render:
        show(model, title="Loft transition — four side faces and two caps")


if __name__ == "__main__":
    main()
