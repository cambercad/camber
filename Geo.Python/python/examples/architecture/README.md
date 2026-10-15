# House components

Use ordinary sketches/extrudes for walls, floors and foundations. This toolbox
adds components whose construction details are useful to generate:

- `windows.py`: framed windows, glazing, sashes, hardware and sills.
- `doors.py`: framed flush/panel/glazed doors, hardware and thresholds.
- `roofs.py`: rectangular gable, hip and flat roofs with vertical thickness and overhangs.
- `roof_coverings.py`: S-profile pantiles with recessed side laps, or double-cambered
  double-lapped plain tiles; hanging nibs, fixing holes, cut hip/verge pieces and
  individual curved ridge/hip tiles. Metal panels have thin folded male/female locks.
  `detail='surface'` is an explicitly simplified skin.
- `stairs.py`: open timber treads on hollow steel centre or side stringers,
  drilled saddles and fasteners; slim tubular rails with vertical infill or clamped
  glass. Concrete waists and traditional timber construction are also available.
  Risers, landing, materials and railing sides are configurable.
- `rainwater.py`: sloping half-round gutters with rolled front beads, rear hems,
  drilled hanger brackets, end caps and a fitted outlet,
  plus hollow downpipes routed through 3D points with circular elbow bends.

The same frozen parameter dataclasses and `part=`/`name=` conventions apply to all
builders. They return `assembly`, `solids` and `colors`; new building components
also expose their owning `part`. Repeated full roof tiles share solid definitions.
No new pip dependencies are required.

## Complete house study

`gallery_house.py` combines the builders in a 12 × 10 m family-house design:
double-height living, a reading gallery, three bedrooms, study/guest room,
utility/laundry, entrance storage, guest WC/shower and an upstairs bathroom.
The 35° gable roof includes a cross-gable, two generous dormers and three rooflights.
Windows, doors, stairs, gutters and downpipes use the detailed component builders.
Room dimensions and circulation are intentional; this is a design study rather
than construction documentation. Roof finishes are represented as continuous
panels here, rather than thousands of individual tiles. Kitchen masses indicate
working space; appliances, sanitary fixtures, services and structural design are
not included.

```powershell
.venv/Scripts/python.exe Geo.Python/python/examples/architecture/gallery_house.py
.venv/Scripts/python.exe Geo.Python/python/examples/architecture/gallery_house.py --render house.png --plans house-plans.svg
.venv/Scripts/python.exe Geo.Python/python/examples/architecture/parametric_house.py
```

The render command writes exterior views, horizontal sections of both levels and
a longitudinal section through the stair. `HouseModel.daylight_schedule()` links
each room to its actual window definitions, indicative room rectangle area and visible glazing area
(m²). These are geometric coordination quantities, not a daylight simulation or
compliance calculation. The separate SVG labels room uses, indicative areas and
visible glazing areas. Privacy windows serve the guest WC and family bathroom;
the entrance has side glazing and the landing has its own rooflight. Upstairs
partitions extend to the pitched roof underside. The straight stair uses 18
risers over 3200 mm with 270 mm goings.
Rooflights are fixed glazed units; operable roof-window hinges, flashing kits
and roof-access hardware remain to be developed. Vertical opening windows
already include handles, hinges, seals and glazing beads.

The GUI document exposes
the house as a registered component with cutaway, terrace and rainwater options;
individual rooms are currently edited in the source layout, not as GUI parameters.
Use the modeler's section tool for interactive inspection. The native viewer uses
the component colors; the modeler keeps its normal workpiece display scheme.

```python
from dataclasses import asdict
from examples.architecture.roofs import RoofParameters, build_roof
from examples.architecture.roof_coverings import RoofCoveringParameters, build_roof_covering
from examples.architecture.stairs import StairParameters, build_stairs
from examples.architecture.rainwater import GutterParameters, DownpipeParameters, build_gutter, build_downpipe

roof_parameters = RoofParameters(width=8000, length=10000, shape='hip', pitch=35)
roof = build_roof(roof_parameters)
roof_layout = asdict(roof_parameters)
roof_layout.pop('thickness')  # Structural thickness is independent of tile/sheet thickness.
covering = build_roof_covering(RoofCoveringParameters(**roof_layout), part=roof.part)
stairs = build_stairs(StairParameters(total_rise=2800, risers=16, going=270))
gutter = build_gutter(GutterParameters(length=10000, fall=30))
x, y, z = gutter.outlet
pipe = build_downpipe(DownpipeParameters(points=((x, y, z), (x, y, 2300),
                                              (x-150, y, 2100), (x-150, y, 100))))
```

Roof width/length refer to exterior wall lines. `eave_height` is the roof top at
those lines, and pitch uses degrees. Gutter `outlet_offset` is measured back from
its low end. Stair `risers` includes the final rise onto the landing/upper floor,
so there is one fewer tread; `landing_length=0` connects to an existing upper floor.
Downpipe collars represent pipe rings, without assuming a wall or its bracket placement.

Run any of `roofs.py`, `roof_coverings.py`, `stairs.py` or `rainwater.py` like the
window/door commands below; `--no-show` checks construction and `--render file.png`
saves a view. For **GUI editing**, run `parametric_roof.py` or `parametric_stairs.py`.
They use the existing feature tree, parameter table, validation and undo/redo.
`parametric_roof.py` starts with individual pantiles. Set Detail to `surface` only
when a simplified representation is wanted. Imported `*_component` functions also become
available through the modeler's Operations catalog.

These are representative architectural components. The current scope is
rectangular roofs, pantiles/plain tiles/standing-seam sheets and straight stairs; it does
not infer roof framing, waterproofing systems, building-code compliance or load capacity.

Tile dimensions left as `None` use the selected profile's defaults: pantiles
250 × 375 mm, 26 mm side lap and 75 mm headlap; plain tiles 165 × 265 mm with
65 mm headlap. Plain tiles use staggered double courses; pantiles use aligned
channels and single courses. `tile_thickness` and `sheet_thickness` are separate.

Geometry references (representative profiles, not exact proprietary product copies):
[pantile proportions and laying dimensions](https://www.marley.co.uk/roof-tiles/clay-roof-tiles/lincoln-single-pantile),
[plain-tile guide](https://www.wienerberger.co.uk/content/dam/wienerberger/united-kingdom/marketing/documents-magazines/brochures/UK_MKT_DOC_Clay_Plain_Tile_Finder.pdf),
[ridge fittings](https://www.marley.co.uk/fittings/ridge-and-hip-tiles/clay-305mm-half-round-ridge),
[rolled gutter section](https://www.kmsheetmetal.com/wp-content/uploads/gutters/submittals/6-inch-euro-bead-half-round-gutter.pdf).

## Windows and doors

Reusable assemblies for house modelling. All lengths are in **millimetres**.
The origin is at the lower-left outside-frame corner (at floor level for doors).
X runs right, Z runs up, and +Y points indoors. Handing is viewed from outside;
positive opening angles swing inward. Width and height exclude projecting
window sills and door casings.

From the repository root, using the existing virtual environment:

```powershell
.venv\Scripts\python.exe Geo.Python\python\examples\architecture\windows.py --width 1800 --height 1400 --leaves 3 --open 30
.venv\Scripts\python.exe Geo.Python\python\examples\architecture\doors.py --width 1500 --leaves 2 --style glazed --open 40
```

Add `--no-show` for a construction check, or `--render output.png` to capture
exterior, interior and isometric views with Camber's existing renderer.

Import with `Geo.Python/python` on the Python path:

```python
from camber import Part
from examples.architecture.windows import WindowParameters, build_window
from examples.architecture.doors import DoorParameters, build_door

window = build_window(WindowParameters(
    width=2100, height=1650, leaves=3,
    leaf_weights=(1, 1.5, 1),
    hinge_sides=("left", "fixed", "right"),
    opening_angles=(30, 0, 45),
    transom_height=280,
), name="living_room_window")

door = build_door(DoorParameters(
    width=1500, height=2300, leaves=2,
    leaf_weights=(1.4, 1), active_leaf=0,
    style="glazed", opening_angles=(45, 0), threshold_height=18,
), name="entrance_door")

house_part = Part((-10000, -10000, -1000), (10000, 10000, 10000), tolerance=.025)
house = house_part.assembly(name="house")
house.solve_after_every_constraint = False
house.fix(house.add_subassembly(window.assembly, position=(2500, 0, 900)))
house.fix(house.add_subassembly(door.assembly, position=(0, 0, 0)))
house.solve()
house.show(colors={**window.colors, **door.colors}, checker=False)
```

Alternatively, pass `part=house_part` to either builder. Parameters are frozen
dataclasses; use `dataclasses.replace(parameters, width=...)` for variants.
Each result exposes `assembly`, `leaves`, `solids`, `glass`, and `colors`.
Use distinct names when combining several components. Leaves, panes, seals,
spacers, hinges, screws and trim retain separate, readable component names.

Windows have stepped, mitred frame/sash members, glazing beads and seals,
individual insulated-glass panes and hollow spacers, weather seals, drainage
mouths/rain hoods, hinges, handles and an optional sloped metal sill. Set
`glazing=(4, 16, 4)` for double glazing; the default is triple glazing.
Set `mullion_width=0` for two meeting sashes hinged at the outer jambs. The
optional top transom is fixed glazing; a `"fixed"` entry in `hinge_sides`
(for example, `hinge_sides=("left", "fixed")`) suppresses that leaf's operating hardware.

Doors have rebated jambs, casings on both sides, seals, three hinges per leaf,
raised panels or a flush slab, an optional upper glazed panel, mortise-lock
pockets, handles/escutcheons, latch and strike plate. Double doors have one
active leaf and a passive leaf with flush bolts and a moving meeting cover.
`threshold_height=0` gives an interior door with a floor gap; a positive value
adds a metal threshold.

These are representative architectural models, not certified manufacturer's
sections. Hidden extrusion chambers, reinforcement, internal lock mechanisms,
tilt-turn gearing and fire/thermal/security ratings are outside their scope.
Camber's current color overrides are RGB; the panes have glass-colored solids
and retain their physical thicknesses and gas cavities.

## Editable GUI document

Run `.venv\Scripts\python.exe Geo.Python\python\examples\architecture\parametric_openings.py`.
The Features panel contains a named parameter table. Select a parameter to edit
its value/expression, unit label and description, then Apply. Numeric feature
inputs accept arithmetic expressions referencing earlier parameters. Units are
labels, not automatic conversions; these generators use millimetres.
Use Delete parameter in the table or parameter editor to remove it. Referenced
parameters require confirmation: replace references with the current numeric
value, or cancel. Replacement preserves the current dimensions and is undoable.
References in custom Python scopes must first be edited in Python.

Select Window or Door to edit its dimensions and options. Parameter references
remain references after editing; changing a shared dimension regenerates the
component and its assembly occurrences. Invalid combinations block Apply, and
a failed rebuild preserves the previously displayed model. Save/reopen uses
ordinary Python source, including the parameter expressions.

The example is a module-level script: no `build_model()` function is required.
Module-level documents assign `part` and `displayed`; existing `build_model()`
documents remain supported. GUI-owned steps use the existing feature markers.

For other generators, use `register_generator(builder, ParameterDataclass,
name="example_component")` from `camber.modeling_parameters`, assigning its
return value to `example_component` in the builder's module. Explicitly import
that callable in a document to register its editor. Builders accept the dataclass
plus `part`/`name` keywords and return an object exposing an `assembly`. GUI calls
expose the dataclass fields as keyword arguments; the original dataclass builder
remains available for Python callers. No additional dependencies are needed.
