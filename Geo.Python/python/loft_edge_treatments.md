# Sculpted lofts and edge treatments

```bash
.venv/bin/python Geo.Python/python/loft_edge_treatments.py
.venv/bin/python Geo.Python/python/loft_edge_treatments.py --case helical_crown
.venv/bin/python Geo.Python/python/loft_edge_treatments.py --render /tmp/sculpted-gallery.png
```

UV checkerboard is on by default. The palette uses blue, clear red, violet, yellow, orange and green; chamfers are yellow and rounds bright green.
Use `--no-checker` for plain shading. Both `show(..., checker=True)` and
`render_views(..., checker=True)` support UV inspection with custom colors.

The default viewer contains eight sculpted engineering studies. They occupy four rows
spaced 78 mm apart, with X increasing left to right. All dimensions are millimetres.

| Case | Shape and feature combination |
| --- | --- |
| `curved_rounds` | R0.8 on all four complete 96-mm-high twisted longitudinal loft edges |
| `curved_chamfers` | Distance-0.8 chamfers on the same four long curved edges for direct comparison |
| `swept_vane` | Five tangent arc/line sections, an S-shaped centreline and 65° section twist |
| `twisted_duct` | An 85° twisted transition with R0.6 on all four curved inner upright edges |
| `crossed_fins` | Two opposed swept fins, patterned and united with their mounting shoe |
| `helical_crown` | Six curved vanes patterned around a common shoe and united together |
| `hollow_rounds` | Hollow rectangular extrusion: four concave upright R2 rounds followed by R0.8 around the complete top opening |
| `hollow_chamfers` | Same four concave upright rounds followed by D0.8 around the complete top opening |

The section tables state station locations, widths, thicknesses and angles.
The `swept_vane` sections have two straight flanks joined to two semicircles, with
radius and tangent constraints. These mixed-curve sketches and the other
specimens' four-line sketches use named `bottom` curves to establish
correspondence and direction. Each authored curve produces a separately
selectable side face, including the two rounded ends of the vane. Curved edges arise
from loft interpolation through the stations. The rectangular section helper
is shared with `loft_transition.py`.
These are engineering shape studies, not performance-validated turbine designs.

Each shoe has a Boolean-cut corner relief with a **distance-2 inward chamfer**.
It adds material at the concave edge, unlike the outside chamfers. A separate
**R2.5 three-edge connected corner** meets a **distance-0.8 rim chamfer** at the
end of the rounded rim. These are actual edge-processing operations with named
face intersections, not replacement meshes. The eight pieces form one assembly.
The cut relief is easiest to inspect from above or the rear-right corner.

The first two specimens apply actual fillet/chamfer operations to all four
longitudinal intersections of the loft faces. Their R0.8 rounds and distance-0.8
chamfers run through the complete sweep and twist. The previous disconnected
contours were caused by isolated reversed loft normals corrupting the offset
supports; correcting the normal orientation also fixed these edge treatments.
The `curved_four_edges` probe now succeeds and has a native regression covering
both rounds and chamfers.

The hollow extrusion pair shows the complete inner boundary: four upright
concave edges are rounded in one R2 feature, then all eight top-rim segments
(four straight edges and the four new corner arcs) receive a second treatment.
The upright rounds add material; the top-lip round or chamfer removes material.
The passage remains open through the shoe. These are successive features because
the upright edges and opening lip have opposite convexity. The twisted duct also
has all four inner upright edges rounded. Its top rim is a separate failing
kernel reproduction below, rather than an omitted operation disguised as success.

## Small diagnostic specimens

Use `--coupons` for the previous eight compact specimens, or select one directly:

| Case | Treatment |
| --- | --- |
| `convex_edges` | Four convex longitudinal edges rounded together, R1.5 |
| `concave_notch` | One concave through-notch edge, R1; adds material |
| `mixed_selection` | Separate convex and concave edges selected together, R1 |
| `corner_network` | Three connected convex flange edges meeting at a corner, R2 |
| `different_radii` | Successive features on opposite edges, R2 and R0.7 |
| `chamfer_edges` | Four flat longitudinal chamfers, distance 1 |
| `round_and_chamfer` | R2 corner network plus an opposite distance-0.8 chamfer |
| `meeting_treatments` | Distance-0.7 rim chamfer terminating into an R2 network |

A chamfer is the flat edge treatment. Different radii means separate constant
radii, not a continuously varying radius. Disconnected convex and concave
selections do not imply support for a connected mixed-convexity graph.

## Challenging kernel reproductions

`--probe NAME` runs a small reproduction without catching exceptions. Some
previous failures now succeed; the table distinguishes those successes from
remaining limitations. Failed treatments are never silently replaced.

| Probe | Current result |
| --- | --- |
| `curved_round`, `curved_chamfer` | **Now succeed:** R0.6 / distance-0.6 on the bottom/right curved edge |
| `curved_four_edges` | **Now succeeds:** four longitudinal curved edges rounded together at R0.8 |
| `inward_chamfer_network` | **Now succeeds:** three connected concave pocket edges, distance 0.6 |
| `tapered_round_corner`, `tapered_chamfer_corner` | **Now succeed:** connected tapered three-edge corners, R1 / distance 1 |
| `pocket_round` | **Now succeeds:** blind pocket bottom edge, R1 |
| `flange_chamfer_corner` | **Now succeeds:** three-edge flange chamfer corner, distance 0.7 |
| `enlarged_bounds_corner` | **Now succeeds:** meeting treatments in enlarged bounds, including explicit previous loft sampling |
| `mixed_corner` | Connected concave and convex edges: explicitly rejected by the kernel |
| `duct_rim_rounds`, `duct_rim_chamfers` | Twisted inner opening rim at R0.6 / D0.6: partial cut in the blend Boolean; the straight extruded opening succeeds |

For example:

```bash
.venv/bin/python Geo.Python/python/loft_edge_treatments.py --probe curved_round
.venv/bin/python Geo.Python/python/loft_edge_treatments.py --probe inward_chamfer_network
```

The repaired cases have native regressions. Pocket trims retain the source-edge
side of their supporting face, and open-end closures are kept separate from corner
arcs. Corner winding follows exact shared boundaries. Planar matched loft sides
retain their authored planes; offsets preserve distinct rational source vertices.

Individual curved endpoint closures also preserve their supporting face and inherit
its normals and UVs. Curved extension closures are not falsely labelled planar;
where no analytic extension exists, their metadata remains unspecified. A complete
twisted inner rim still requires a valid transition through its reentrant junctions.
Rounding its uprights first currently exposes a disconnected offset contour instead.

The remaining failures are **not evidence of impossible geometry**. They expose current
kernel limitations needing diagnosis. Oversized radii can also remove supporting
faces or cause neighbouring features to collide, but that is a different issue.

## Validation

```bash
.venv/bin/python -m unittest discover -s Geo.Python/python -p test_loft_edge_treatments.py -v
```

Tests build the eight sculptures and eight coupons, checking closed positive
volumes, convex material removal, concave material addition, generated corner
patches, open passages, material addition at all four inner upright rounds, complete
eight-edge opening treatments and distinct material removal by long curved rounds/chamfers. The runtime
report printed by the script measures construction, excluding rendering and
viewer startup, and includes each specimen’s build time and triangle count. Multiview capture also saves full-resolution individual images.
