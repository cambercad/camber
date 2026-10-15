"""Detailed side-hung windows; millimetres, lower-left origin, +Y indoors.

Run: python Geo.Python/python/examples/architecture/windows.py --leaves 3 --open 35
Import: from examples.architecture.windows import WindowParameters, build_window
"""
from dataclasses import dataclass
import argparse
from pathlib import Path
import sys
from numbers import Real

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from examples.architecture._joinery import Builder, SEAL, METAL, color, layout, positive
from camber import Frame


@dataclass(frozen=True, kw_only=True)
class WindowParameters:
    width: float = 1400
    height: float = 1400
    leaves: int = 2
    leaf_weights: tuple | None = None
    opening_angles: tuple | None = None
    hinge_sides: tuple | None = None
    frame_width: float = 62
    frame_depth: float = 82
    sash_width: float = 62
    sash_depth: float = 68
    mullion_width: float = 48
    transom_height: float = 0
    gap: float = 3
    glazing: tuple = (4, 14, 4, 14, 4)
    sill: bool = True
    frame_color: tuple = (.91, .92, .93)

    def __post_init__(self):
        for name in ("width", "height", "frame_width", "frame_depth", "sash_width", "sash_depth", "gap"):
            positive(name, getattr(self, name))
        for name in ("mullion_width", "transom_height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not 0 <= value < float("inf"):
                raise ValueError(f"{name} must be finite and nonnegative")
        weights, angles, sides = layout(self.leaves, self.leaf_weights, self.opening_angles, self.hinge_sides)
        for field, value in (("leaf_weights", weights), ("opening_angles", angles), ("hinge_sides", sides),
                             ("glazing", tuple(self.glazing)), ("frame_color", tuple(self.frame_color))):
            object.__setattr__(self, field, value)
        if len(self.glazing) < 3 or len(self.glazing) % 2 != 1:
            raise ValueError("glazing must alternate glass/gap/glass thicknesses, e.g. (4, 16, 4)")
        for thickness in self.glazing:
            positive("glazing thickness", thickness)
        if self.sash_width <= 19 or self.sash_depth < sum(self.glazing)+12:
            raise ValueError("sash must accommodate the glazing, beads and 12 mm edge bite")
        if self.frame_width < 24:
            raise ValueError("frame_width must be at least 24 mm for the rebate and drainage")
        if self.frame_depth < self.sash_depth+13:
            raise ValueError("frame_depth must accommodate sash_depth plus the stop and seals (13 mm)")
        available = self.width-2*self.frame_width-(self.leaves-1)*self.mullion_width
        if min(available*w/sum(weights)-2*self.gap for w in weights) <= 2*self.sash_width+40:
            raise ValueError("each leaf must leave at least 40 mm of visible glass after frames and gaps")
        opening_height = self.height-2*self.frame_width-self.transom_height-(self.mullion_width if self.transom_height else 0)
        if opening_height-2*self.gap <= 2*self.sash_width+80:
            raise ValueError("height is too small for the sash profiles and transom")
        if self.transom_height and self.transom_height <= 2*self.sash_width+40:
            raise ValueError("transom_height must leave at least 40 mm of visible glass")
        if self.transom_height and not self.mullion_width:
            raise ValueError("a transom requires a positive mullion_width")
        if not self.mullion_width and (self.leaves > 2 or
                self.leaves == 2 and (sides[0] not in ("left", "fixed") or sides[1] not in ("right", "fixed"))):
            raise ValueError("without mullions, use one leaf or two leaves hinged at the outer jambs")
        if not isinstance(self.sill, bool):
            raise ValueError("sill must be True or False")
        color(self.frame_color)


def _sash(builder, fixed, p, name, x, z, width, height, side):
    leaf = builder.group(name)
    front, back = 11, 11+p.sash_depth
    structural = p.sash_width-12
    glass_front = front+(p.sash_depth-sum(p.glazing))/2
    builder.frame(leaf, name + "_profile", 0, 0, width, height,
                  [(structural, front, back), (p.sash_width, front, glass_front)], p.frame_color)
    builder.frame(leaf, name + "_glazing_bead", structural, structural,
                  width-2*structural, height-2*structural,
                  [(12, glass_front+sum(p.glazing), back)], p.frame_color)
    builder.glazing(leaf, name + "_glass", structural+1, structural+1,
                    width-2*structural-2, height-2*structural-2, glass_front, p.glazing)
    for label, y in (("exterior_glazing_seal", glass_front-1), ("interior_glazing_seal", glass_front+sum(p.glazing))):
        builder.ring(leaf, name + "_" + label, p.sash_width-1, p.sash_width-1,
                     width-2*p.sash_width+2, height-2*p.sash_width+2, 2, y, y+1, SEAL)
    builder.ring(fixed, name + "_weather_seal", x-1, z-1, width+2, height+2, 3, 8, 10.5, SEAL)
    if side == "fixed":
        return leaf, (0, 0)
    pivot = builder.hinges(fixed, leaf, name, x, z, width, height, back, side)
    handle_x = width-p.sash_width/2 if side == "left" else p.sash_width/2
    handle_z = min(height*.52, 1100)
    builder.rounded_box(leaf, name + "_handle_rosette", handle_x-13, handle_z-33,
                        26, 66, back+1, back+6, 5, METAL)
    builder.cylinder(leaf, name + "_handle_spindle", (handle_x, back+6, handle_z), 6, 18, axis="y")
    builder.rounded_box(leaf, name + "_handle_lever", handle_x-7, handle_z-88,
                        14, 98, back+20, back+32, 6, METAL)
    for offset in (-24, 24):
        builder.screw(leaf, name + f"_handle_screw{offset}", handle_x, back+6, handle_z+offset)
    return leaf, pivot


def build_window(parameters=WindowParameters(), *, part=None, name="window"):
    """Build a detailed assembly. Width/height are the outside frame dimensions.

    Leaf weights divide the clear bays. Angles are degrees inward; hinge sides
    are left/right as viewed from outside. A zero-width mullion gives meeting
    sashes. The optional top transom is a fixed glazed light. Frame dimensions
    exclude the projecting sill. Pass an existing Part to build into a house.
    """
    p = parameters
    if not isinstance(p, WindowParameters):
        raise TypeError("parameters must be WindowParameters")
    b = Builder(part, name, p.width, p.height, p.frame_depth)
    fixed = b.group(name + "_fixed_frame")
    # Exterior drainage mouths and rain hoods are separate from the glazing.
    drains = []
    drain_z = p.frame_width*.4
    for i, x in enumerate((p.width*.25, p.width*.75)):
        drains.append(b.rounded_solid(name + f"_drain_cutter{i}", x-12, drain_z,
                                     24, 5, -1, 13, 2))
    b.frame(fixed, name + "_frame", 0, 0, p.width, p.height,
            [(p.frame_width, 0, p.frame_depth), (p.frame_width+12, 0, 8)],
            p.frame_color, cutters={"bottom": drains})
    for i, x in enumerate((p.width*.25, p.width*.75)):
        b.rounded_box(fixed, name + f"_drain_hood{i}", x-16, drain_z+4, 32, 5, -3, 0, 2, p.frame_color)
    weights, angles, sides = layout(p.leaves, p.leaf_weights, p.opening_angles, p.hinge_sides)
    available = p.width-2*p.frame_width-(p.leaves-1)*p.mullion_width
    height = p.height-2*p.frame_width-p.transom_height-(p.mullion_width if p.transom_height else 0)
    leaves, x = [], p.frame_width
    for i, (weight, angle, side) in enumerate(zip(weights, angles, sides)):
        bay = available*weight/sum(weights)
        leaf, pivot = _sash(b, fixed, p, f"{name}_leaf{i+1}", x+p.gap, p.frame_width+p.gap,
                            bay-2*p.gap, height-2*p.gap, side)
        leaves.append(b.place_leaf(leaf, x+p.gap, p.frame_width+p.gap, pivot, angle, side))
        if i > 0 and not p.mullion_width:
            b.box(leaf, f"{name}_meeting_cover{i}", (-p.gap-10, 11+p.sash_depth, 0),
                  (10, 15+p.sash_depth, height-2*p.gap), p.frame_color)
        x += bay
        if i < p.leaves-1 and p.mullion_width:
            b.box(fixed, f"{name}_mullion{i+1}", (x, 0, p.frame_width),
                  (x+p.mullion_width, p.frame_depth, p.frame_width+height), p.frame_color)
            x += p.mullion_width
    if p.transom_height:
        z = p.frame_width+height
        b.box(fixed, name + "_transom", (p.frame_width, 0, z),
              (p.width-p.frame_width, p.frame_depth, z+p.mullion_width), p.frame_color)
        top, _ = _sash(b, fixed, p, name + "_fixed_light", p.frame_width+p.gap, z+p.mullion_width+p.gap,
                       p.width-2*p.frame_width-2*p.gap, p.transom_height-2*p.gap, "fixed")
        occurrence = b.assembly.add_subassembly(top, position=(p.frame_width+p.gap, 0, z+p.mullion_width+p.gap))
        b.assembly.fix(occurrence)
    if p.sill:
        sketch = b.part.sketch(frame=Frame((-15, 0, 0), x=(0, 1, 0), y=(0, 0, 1), z=(1, 0, 0)), name=name + "_sill_section")
        points = [(-40, -12), (p.frame_depth*.25, -6), (p.frame_depth*.25, -1), (-40, -7)]
        for a, end in zip(points, points[1:]+points[:1]):
            sketch.add_line(a, end)
        b.add(fixed, sketch.extrude(p.width+30, name=name + "_sloped_sill"), METAL)
    occurrence = b.assembly.add_subassembly(fixed)
    b.assembly.fix(occurrence)
    return b.finish(leaves)


from camber.modeling_parameters import register_generator
window_component = register_generator(build_window, WindowParameters, name="window_component")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, default=1400)
    parser.add_argument("--height", type=float, default=1400)
    parser.add_argument("--leaves", type=int, default=2)
    parser.add_argument("--open", type=float, default=0, dest="angle")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--render")
    args = parser.parse_args()
    model = build_window(WindowParameters(width=args.width, height=args.height, leaves=args.leaves,
                         opening_angles=(args.angle,)*args.leaves))
    print(f"Window: {len(model.leaves)} leaves, {len(model.glass)} glass panes, {len(model.solids)} components")
    if args.render:
        from camber import render_views
        render_views(model.assembly, args.render, views=["-Y", "+Y", "rear iso"], colors=model.colors, checker=False)
    if not args.no_show and not args.render:
        model.assembly.show(colors=model.colors, checker=False, title="Parametric window")
