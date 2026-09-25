# CadQuery sketch tutorial ports

Source: [CadQuery sketch tutorial](https://cadquery.readthedocs.io/en/latest/sketch.html).
The numbered files follow the online examples. Where an example returns a 2D
sketch, the script adds a short extrusion only so the Camber viewer can display
it. Running a file directly opens the viewer; the same files can be run
headlessly for checks.

The face-based, edge-based, hull, in-place, placement, loft, sketch-boolean,
and offset examples now use their corresponding fluent calls in the bridge.
The sketch bridge preserves face modes and performs their add/subtract/intersect
operations through Camber's existing solid CSG when the profile is consumed by
an extrusion or cut. Offset contours and convex hulls are still polygonal
approximations at the Part tolerance.

Multiple-elements example 08 uses a tapered blind cut. The bridge creates a
drafted extrusion for each closed profile and subtracts the cutters with Camber
CSG. Surface-terminated cuts do not yet accept taper.

All other scripts in this folder use the tutorial's sketch API sequence and
finish with a displayable Camber solid. Camber's mesh kernel can differ from
CadQuery's B-rep output in tessellation and in the experimental convex-hull and
offset approximations.
