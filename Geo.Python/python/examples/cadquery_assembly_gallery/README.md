# CadQuery assembly documentation ports

These correspond to the examples in [CadQuery's Assemblies page](https://cadquery.readthedocs.io/en/latest/assy.html). Each script exposes `build()`; supported scripts open Camber's viewer when run directly. The model components use one shared `Part`, required by Camber's native assembly registry. `cone()` uses a revolved profile equivalent to CadQuery's `Solid.makeCone`.

| Script | Status | Difference or gap |
| --- | --- | --- |
| 01 enclosure door | Blocked | DXF wire import, face/edge tag persistence with `end()`, and round-transition sweep are missing. Plain bars would misrepresent the V-slot assembly. |
| 02 object locations | Builds | Axis-angle `Location`; color alpha is not rendered. |
| 03 point on arc | Blocked | Open `Edge` assembly children, `Edge.positionAt`, and point datums on edges are missing. |
| 04 axis cones | Builds | Cone made by equivalent profile revolve. |
| 05 plate and pin | Builds | Cone made by equivalent profile revolve. |
| 06 surface Axis + Point | Builds | `parametricSurface` is a sampled triangle patch; the assembly datum uses its center normal from the callback. |
| 06b surface Plane | Builds | Same sampled-patch approximation; no analytic/NURBS surface metadata. |
| 07 PointInPlane | Builds | Uses source sketch-line face names instead of CadQuery's tags; offset is an offset native plane datum. |
| 08 PointOnLine | Builds | Exactly decomposed into two perpendicular point-on-plane mates. |
| 09 FixedPoint | Builds | World point represented as a datum on an already fixed component. |
| 10 FixedRotation | Builds | Directed X/Y axis mates to the fixed component lock rotation. |
| 11 FixedAxis | Builds | Directed axis mate to the fixed component. |

The bridge currently supports a small named-color palette and RGB display only; CadQuery's full named-color list and per-component transparency are not yet supported. For whole-solid point references it uses the bounding-box midpoint, which is exact for the boxes and spheres here but is not a general center-of-mass replacement. Assembly STEP/XML export and the object-form `constrain()` overload are also not exposed. Unsupported samples deliberately fail with a specific reason; they are not substituted with visually similar geometry.
