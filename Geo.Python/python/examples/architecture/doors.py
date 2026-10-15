"""Detailed framed doors; millimetres, floor-level origin, +Y indoors.

Run: python Geo.Python/python/examples/architecture/doors.py --style glazed --open 45
Import: from examples.architecture.doors import DoorParameters, build_door
"""
from dataclasses import dataclass
import argparse
from pathlib import Path
import sys
from numbers import Integral, Real
from typing import Literal

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Frame
from examples.architecture._joinery import Builder, SEAL, METAL, color, layout, positive


@dataclass(frozen=True, kw_only=True)
class DoorParameters:
    width: float = 1000
    height: float = 2150
    leaves: int = 1
    leaf_weights: tuple | None = None
    opening_angles: tuple | None = None
    hinge_sides: tuple | None = None
    active_leaf: int = 0
    frame_depth: float = 160
    jamb_width: float = 32
    leaf_thickness: float = 44
    stile_width: float = 100
    rail_width: float = 100
    panel_rows: int = 3
    style: Literal['flush', 'panel', 'glazed'] = "panel"
    gap: float = 3
    floor_gap: float = 8
    threshold_height: float = 0
    casing_width: float = 65
    leaf_color: tuple = (.48, .28, .15)
    frame_color: tuple = (.89, .87, .81)

    def __post_init__(self):
        for name in ("width", "height", "frame_depth", "jamb_width", "leaf_thickness", "stile_width", "rail_width", "gap", "floor_gap", "casing_width"):
            positive(name, getattr(self, name))
        if self.leaves not in (1, 2) or isinstance(self.leaves, bool):
            raise ValueError("doors support one or two leaves")
        if isinstance(self.active_leaf, bool) or not isinstance(self.active_leaf, Integral) or not 0 <= self.active_leaf < self.leaves:
            raise ValueError("active_leaf must identify an existing leaf (zero-based)")
        weights, angles, sides = layout(self.leaves, self.leaf_weights, self.opening_angles, self.hinge_sides)
        for field, value in (("leaf_weights", weights), ("opening_angles", angles), ("hinge_sides", sides),
                             ("leaf_color", tuple(self.leaf_color)), ("frame_color", tuple(self.frame_color))):
            object.__setattr__(self, field, value)
        if "fixed" in sides:
            raise ValueError("door hinge_sides must be 'left' or 'right'")
        if self.leaves == 2 and sides != ("left", "right"):
            raise ValueError("double-door leaves must hinge at the outer jambs: ('left', 'right')")
        if isinstance(self.panel_rows, bool) or not isinstance(self.panel_rows, Integral) or self.panel_rows < 1:
            raise ValueError("panel_rows must be a positive integer")
        if self.style not in ("flush", "panel", "glazed"):
            raise ValueError("style must be 'flush', 'panel' or 'glazed'")
        if isinstance(self.threshold_height, bool) or not isinstance(self.threshold_height, Real) or not 0 <= self.threshold_height < float("inf"):
            raise ValueError("threshold_height must be finite and nonnegative")
        if self.frame_depth <= self.leaf_thickness+30 or self.leaf_thickness < 30:
            raise ValueError("frame_depth must exceed leaf_thickness by 30 mm, and leaf_thickness must be at least 30 mm")
        if self.style == "glazed" and self.leaf_thickness < 34:
            raise ValueError("glazed doors require a leaf_thickness of at least 34 mm for glazing beads")
        if self.stile_width < 85:
            raise ValueError("stile_width must be at least 85 mm to contain the lock mortise")
        if self.jamb_width < 18 or self.casing_width <= self.jamb_width:
            raise ValueError("jamb_width must be at least 18 mm, with a wider casing")
        clear = self.width-2*self.jamb_width-(self.leaves+1)*self.gap
        if min(clear*w/sum(weights) for w in weights) <= 2*self.stile_width+80:
            raise ValueError("each leaf must accommodate the stiles and an 80 mm panel opening")
        leaf_height = self.height-self.jamb_width-self.gap-self.floor_gap-self.threshold_height
        if leaf_height-(self.panel_rows+1)*self.rail_width < self.panel_rows*80:
            raise ValueError("height is too small for the rails and panel_rows")
        color(self.leaf_color)
        color(self.frame_color)


def _raised_panel(b, leaf, name, x, z, width, height, front, back, rgb):
    inset = min(18, width*.08, height*.08)
    core = b.part.cuboid((x, front+6, z), (x+width, back-6, z+height), name=name + "_core")
    caps = []
    for face, outer_y, inner_y in (("front", front+6, front), ("back", back-6, back)):
        sections = []
        for i, (y, margin) in enumerate(((outer_y, 0), (inner_y, inset))):
            sketch = b.part.sketch(frame=Frame((0, y, 0), x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)), name=f"{name}_{face}_section{i}")
            sketch.add_rectangle((x+margin, z+margin), (x+width-margin, z+height-margin))
            sections.append(sketch)
        caps.append(b.part.loft(sections, name=name + "_" + face, max_deviation=.05))
    b.add(leaf, b.part.batch_union([core]+caps, name=name), rgb)


def _hardware(b, fixed, leaf, p, name, x, base_z, width, height, front, back, side, *, active=True, strike=None):
    pivot = b.hinges(fixed, leaf, name, x, base_z, width, height, back, side, large=True)
    if not active:
        bolt_x = width-18 if side == "left" else 18
        for top in (False, True):
            z = height-150 if top else 50
            label = name + ("_upper_flush_bolt" if top else "_lower_flush_bolt")
            b.rounded_box(leaf, label + "_plate", bolt_x-9, z, 18, 100, back, back+2, 3, METAL)
            b.cylinder(leaf, label + "_bolt", (bolt_x, back+4, height-55 if top else 1), 2.5, 54)
            for offset in (8, 92):
                b.screw(leaf, label + f"_screw{offset}", bolt_x, back+2, z+offset)
        return pivot
    lock_x = width-55 if side == "left" else 55
    handle_z = min(1050-base_z, height*.55)
    for face, y, outward in (("front", front, -1), ("back", back, 1)):
        plate_front = y-4 if outward < 0 else y
        b.cylinder(leaf, name + f"_{face}_handle_rosette", (lock_x, plate_front, handle_z), 22, 4, axis="y")
        b.cylinder(leaf, name + f"_{face}_handle_neck", (lock_x, y-32 if outward < 0 else y, handle_z), 7, 32, axis="y")
        length = min(110, width*.28)
        lever_x = lock_x-length+7 if side == "left" else lock_x-7
        b.rounded_box(leaf, name + f"_{face}_lever", lever_x, handle_z-7,
                      length, 14, y-38 if outward < 0 else y+26, y-26 if outward < 0 else y+38, 6, METAL)
        key_z = handle_z-65
        escutcheon = b.part.cylinder((lock_x, plate_front, key_z), 17, 4, axis="y", name=name + f"_{face}_key_plate", max_deviation=.025)
        key_round = b.part.cylinder((lock_x, plate_front-1, key_z+3), 5, 6, axis="y", name=name + f"_{face}_key_round")
        key_stem = b.part.cuboid((lock_x-3, plate_front-1, key_z-8), (lock_x+3, plate_front+5, key_z+3), name=name + f"_{face}_key_stem")
        opening = b.part.batch_union([key_round, key_stem])
        b.add(leaf, b.part.subtract(escutcheon, opening, name=name + f"_{face}_escutcheon"), METAL)
        for dz in (-14, 14):
            b.screw(leaf, name + f"_{face}_rosette_screw{dz}", lock_x,
                    plate_front if outward < 0 else plate_front+4, handle_z+dz, direction=outward)
    edge = width if side == "left" else 0
    case_lo, case_hi = (width-83, width-3) if side == "left" else (3, 83)
    lock_case = b.part.cuboid((case_lo, front+11, handle_z-87), (case_hi, back-11, handle_z+52), name=name + "_lock_case_blank")
    spindle = b.part.cylinder((lock_x, front-1, handle_z), 6, p.leaf_thickness+2, axis="y", name=name + "_case_spindle")
    key = b.part.cylinder((lock_x, front-1, handle_z-65), 7, p.leaf_thickness+2, axis="y", name=name + "_case_key")
    b.add(leaf, b.part.subtract(lock_case, b.part.batch_union([spindle, key]), name=name + "_mortise_lock_case"), METAL)
    lo, hi = (width-2, width+.5) if side == "left" else (-.5, 2)
    b.box(leaf, name + "_lock_faceplate", (lo, front+8, handle_z-92), (hi, back-8, handle_z+65), METAL)
    b.box(leaf, name + "_latch", (edge-3, front+p.leaf_thickness*.4, handle_z-5),
          (edge+3, front+p.leaf_thickness*.6, handle_z+5), METAL)
    target, strike_x, strike_z = strike if strike is not None else (fixed, x+edge, base_z)
    plate = b.part.cuboid((strike_x-1 if side == "left" else strike_x-p.gap, front+6, strike_z+handle_z-45),
                         (strike_x+p.gap if side == "left" else strike_x+1, back-6, strike_z+handle_z+20), name=name + "_strike_blank")
    opening = b.part.cuboid((strike_x-p.gap-2, front+p.leaf_thickness*.35, strike_z+handle_z-7),
                            (strike_x+p.gap+2, front+p.leaf_thickness*.65, strike_z+handle_z+7), name=name + "_strike_opening")
    b.add(target, b.part.subtract(plate, opening, name=name + "_strike_plate"), METAL)
    return pivot


def build_door(parameters=DoorParameters(), *, part=None, name="door"):
    """Build a framed door with casings, stop, seals, panels, hinges and lock trim.

    Width/height describe the outside jamb, excluding projecting casings.
    Angles are degrees inward; left/right handing is viewed from outside.
    'glazed' replaces the highest panel by safety-glass-style double glazing.
    A positive threshold_height adds a metal threshold. Pass an existing Part
    to build into a house; the returned assembly can also be instanced there.
    """
    p = parameters
    if not isinstance(p, DoorParameters):
        raise TypeError("parameters must be DoorParameters")
    b = Builder(part, name, p.width, p.height, p.frame_depth, margin=max(250, p.casing_width+10))
    fixed = b.group(name + "_jamb")
    back = p.frame_depth-10
    front = back-p.leaf_thickness
    b.frame(fixed, name + "_frame", 0, 0, p.width, p.height,
            [(p.jamb_width, 0, p.frame_depth), (p.jamb_width+12, front-12, front-p.gap)], p.frame_color, closed=False)
    extension = p.casing_width-p.jamb_width
    for face, start, end in (("outside", -12, 0), ("inside", p.frame_depth, p.frame_depth+12)):
        b.frame(fixed, name + f"_{face}_casing", -extension, 0, p.width+2*extension, p.height+extension,
                [(p.casing_width, start, end)], p.frame_color, closed=False)
    if p.threshold_height:
        b.box(fixed, name + "_threshold", (p.jamb_width, -6, 0),
              (p.width-p.jamb_width, p.frame_depth+6, p.threshold_height), METAL)
    base_z = p.threshold_height+p.floor_gap
    height = p.height-p.jamb_width-p.gap-base_z
    weights, angles, sides = layout(p.leaves, p.leaf_weights, p.opening_angles, p.hinge_sides)
    available = p.width-2*p.jamb_width-(p.leaves+1)*p.gap
    widths = [available*weight/sum(weights) for weight in weights]
    definitions = [b.group(f"{name}_leaf{i+1}") for i in range(p.leaves)]
    leaves, x = [], p.jamb_width+p.gap
    for i, (width, angle, side, leaf) in enumerate(zip(widths, angles, sides, definitions)):
        label = f"{name}_leaf{i+1}"
        active = i == p.active_leaf
        lock_x = width-55 if side == "left" else 55
        handle_z = min(1050-base_z, height*.55)
        cutters = []
        if active:
            a, end = (width-85, width+1) if side == "left" else (-1, 85)
            face_a, face_end = (width-2.1, width+1) if side == "left" else (-1, 2.1)
            cutters = [b.part.cuboid((a, front+10, handle_z-90), (end, back-10, handle_z+55), name=label + "_mortise_tool"),
                       b.part.cuboid((face_a, front+7.5, handle_z-92.5), (face_end, back-7.5, handle_z+65.5), name=label + "_faceplate_tool")]
            for feature, z, radius in (("spindle", handle_z, 6), ("key", handle_z-65, 7)):
                cutters.append(b.part.cylinder((lock_x, front-1, z), radius, p.leaf_thickness+2,
                                               axis="y", name=label + f"_{feature}_tool"))
        if p.style == "flush":
            slab = b.part.cuboid((0, front, 0), (width, back, height), name=label + "_slab_blank")
            for cutter in cutters:
                slab = b.part.subtract(slab, cutter, name=label + "_flush_slab")
            b.add(leaf, slab, p.leaf_color)
        else:
            for hand, a, end in (("left", 0, p.stile_width), ("right", width-p.stile_width, width)):
                stile = b.part.cuboid((a, front, 0), (end, back, height), name=label + f"_{hand}_stile")
                if hand != side:
                    for cutter in cutters:
                        stile = b.part.subtract(stile, cutter, name=label + f"_{hand}_stile")
                b.add(leaf, stile, p.leaf_color)
            panel_height = (height-(p.panel_rows+1)*p.rail_width)/p.panel_rows
            for row in range(p.panel_rows+1):
                z = row*(panel_height+p.rail_width)
                b.box(leaf, label + f"_rail{row}", (p.stile_width, front, z),
                      (width-p.stile_width, back, z+p.rail_width), p.leaf_color)
            for row in range(p.panel_rows):
                z = p.rail_width+row*(panel_height+p.rail_width)
                panel_name = label + f"_panel{row+1}"
                if p.style == "glazed" and row == p.panel_rows-1:
                    w = width-2*p.stile_width
                    layers = (6, 12, 6)
                    glass_front = (front+back-sum(layers))/2
                    b.glazing(leaf, panel_name + "_glass", p.stile_width+1, z+1, w-2, panel_height-2, glass_front, layers)
                    for face, start, end in (("front", front+4, glass_front), ("back", glass_front+24, back-4)):
                        b.frame(leaf, panel_name + f"_{face}_bead", p.stile_width, z, w, panel_height,
                                [(15, start, end)], p.leaf_color)
                else:
                    _raised_panel(b, leaf, panel_name, p.stile_width+1, z+1, width-2*p.stile_width-2,
                                  panel_height-2, front+6, back-6, p.leaf_color)
        # Three-sided jamb seal; the floor gap intentionally stays open.
        b.frame(fixed, label + "_jamb_seal", x-1, base_z, width+2, height+1,
                [(3, front-p.gap, front-.5)], SEAL, closed=False)
        strike = None
        if p.leaves == 2 and active:
            other = 1-i
            strike = (definitions[other], -p.gap if i == 0 else widths[other]+p.gap, 0)
        pivot = _hardware(b, fixed, leaf, p, label, x, base_z, width, height, front, back, side, active=active, strike=strike)
        leaves.append(b.place_leaf(leaf, x, base_z, pivot, angle, side))
        x += width+p.gap
    if p.leaves == 2:
        # The passive meeting cover follows the second leaf, not the jamb.
        passive_index = 1-p.active_leaf
        passive = definitions[passive_index]
        start, end = (-p.gap-12, 12) if passive_index == 1 else (widths[0]-12, widths[0]+p.gap+12)
        b.box(passive, name + "_meeting_cover", (start, back, 0), (end, back+5, height), p.frame_color)
    occurrence = b.assembly.add_subassembly(fixed)
    b.assembly.fix(occurrence)
    return b.finish(leaves)


from camber.modeling_parameters import register_generator
door_component = register_generator(build_door, DoorParameters, name="door_component")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, default=1000)
    parser.add_argument("--height", type=float, default=2150)
    parser.add_argument("--leaves", type=int, default=1)
    parser.add_argument("--style", choices=("flush", "panel", "glazed"), default="panel")
    parser.add_argument("--open", type=float, default=0, dest="angle")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--render")
    args = parser.parse_args()
    model = build_door(DoorParameters(width=args.width, height=args.height, leaves=args.leaves,
                      style=args.style, opening_angles=(args.angle,)*args.leaves))
    print(f"Door: {len(model.leaves)} leaves, {len(model.glass)} glass panes, {len(model.solids)} components")
    if args.render:
        from camber import render_views
        render_views(model.assembly, args.render, views=["-Y", "+Y", "rear iso"], colors=model.colors, checker=False)
    if not args.no_show and not args.render:
        model.assembly.show(colors=model.colors, checker=False, title="Parametric door")
