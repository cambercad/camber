# Known fillet limitations

## Endpoint branch and shared-boundary invariants

`CurvedEndpointBlendTests` covers a rib on a truncated sphere and on a block with
a spherical pocket, plus a beam entering a sphere. Both fillet and chamfer run
with metadata present/absent; the pocket also runs posed at scales 0.1 and 10.
An extended strip can cross the pocket twice. Rail contact alone then identifies
two components, not the one belonging to the selected edge. Endpoint trimming
uses the exact intersection contours and retains the split component containing
the closest strip point to an interior point of that source edge. Only contours
bounding that component produce caps. An anchor on a cut remains an error.

Splitting a cap against the two supports can insert additional vertices into its
strip contact contour. Those vertices must also subdivide the strip boundary.
Both sides use the exact splitter and existing rational positions; no snapping,
coordinate movement or normal/UV reconstruction from rounded positions is allowed.
The non-conforming T-junction guard remains enabled before volume trimming.

## History-free shell support extension

`CurvedShellRobustnessTests.HistoryFreeNestedCurvedIntersectionsHaveStableCavityNames`
is now an active regression: two perpendicular cylinders intersected with a sphere,
then inward-shelled without construction history, including absent metadata,
different thicknesses and a rigid pose. The previous open-patch trim failed at
`vertical-Circle1_2`: a split curved patch and tangential collar need not separate
the entire volume. Exact CSG correctly rejected conflicting cap-edge windings.

Shell now subtracts closed layers between source patches and their inward offsets.
Their sides close on the exact source coordinates, so ordinary volume CSG can
construct the shared seams without an open cutter having to span the solid.
Artificial layer sides and unchanged source walls must not survive in the cavity;
the final group coverage guard rejects them. Curved offsets remain sampled, but
closure and intersections are exact. This construction is currently selected for
convex inter-patch junctions; it does not establish support for all reentrant
junctions or topology-changing offsets. The recursive intersection-history path
was removed; inward shelling uses the authoritative current mesh.

### Reentrant curved shell junctions

`CurvedShellRobustnessTests.ReentrantNeckCavityFollowsOffsetSupports` checks a
sphere/cylinder union at wall thicknesses 0.2, 0.6 and 0.9, with and without
metadata. A one-shot intersection of tangents at the original seam produced
watertight cavities but missed the curved offset support by about 0.04 at 0.9.
The common tangent solver now re-evaluates analytic supports at each candidate
and requires converged support residuals. Its plane intersections and rank
tests still use rational arithmetic; curved evaluation is floating point.

The metadata-free 0.6 and 0.9 cases now pass, including rigid poses and scales
0.1 and 10. Two distinct causes needed fixing. Revolve previously spent the
whole deviation budget on both profile and angular sampling, allowing nearly
twice the requested surface error. It now shares that budget between them;
`ShellSourceFidelityTests` checks source triangles and Boolean seam vertices.
For missing support domains, mesh supports may use one bounded implicit
polynomial guide per patch, fitted to positions and normal directions and
checked against every referenced source vertex. `ImplicitSurfaceContinuationTests`
also exercises a trimmed bicubic NURBS patch without stored metadata or UVs.
The guide does not replace source triangles or supply invented analytic metadata.
Failed guide queries cannot switch to another approximation during a solve.
Coincident or roundoff-tangent named seams retain their original constraints,
so independently fitted guides cannot turn a smooth seam into conflicting
constraints; exact plane-rank and intersection predicates remain unchanged.

Sample validation is not a certified bound on unknown geometry between samples
or beyond a trim. Higher-degree/poorly conditioned patches may still have no
usable continuation; this does not establish support for every valid curved
shell. Local focal-collapse checks and exact triangle-orientation/volume checks
also do not prove absence of all global offset self-intersections. Watertightness
must not be presented as that proof.
A nearest-offset-triangle iteration was also tested and removed: independent
facet selection failed convergence at the neck seam and opening rim. Preserve
the connected support/intersection branch instead of independently selecting
nearest facets for successive vertex solves.
`ReentrantOffsetIntersectionRequiresMissingSupportDomain` verifies that, at all
three thicknesses, the analytic seam height is above the highest retained
sphere-offset triangle and below the lowest neck-offset triangle. The needed
support domain was removed by the Boolean, so intersecting the available
patches cannot supply this seam. Tangential collars alone cannot establish a
curvature/deviation guarantee for that missing region without a faithful
continuation. Exact arithmetic protects intersection and winding decisions,
but cannot reconstruct missing geometry.

One intended-success regression case is explicitly skipped in the normal
suite. This skip records missing kernel support; it does not mean that the
geometry is fixed. No production rejection or test assertion was removed.

## Impeller root

`ImpellerLoftSupportTests.OriginalRootFilletHasAConnectedSpine` requests a
1.2 mm round at the blade/hub neck intersection. `Intersector.IntersectSurfaces`
used to flatten multiple intersection strips into paired segment lists. The
open-edge path in `BlendEdge.ComputeSpineAndBoundaries` then treated those lists
as one spine. The exact continuity check rejected segment 84 of 96 because it
belonged to a disconnected branch. Intersections now retain their strips, and
open edges select the unique contour with the smallest exact bidirectional
vertex-to-polyline distance from its projected contact curves to that source
edge. A compact pair of
connected graph patches verifies this correspondence independently.

The later endpoint failure is now classified explicitly. The extended bell
support intersects the blend strip in unrelated closed loops and partial
contours, but none spans both fillet contact rails. The old global surface-side
classification combined those contours and reported contradictory evidence.
Endpoint trimming now uses only exact rail-to-rail contours, splits the strip
topologically, and retains the component connected to its opposite longitudinal
anchor. A compact sphere-trimmed beam regression verifies a single selected edge
with both endpoints on the same curved face. The impeller's partial-edge
termination across a real/extended support boundary remains unsupported. A
separate rolling-ball solver has not been established as necessary; offset
intersection and exact trimming remain the construction foundation.

The rotated curved-leg cap-rim regression is enabled. Its trim cutter was an
exactly joined oriented disk with a closed intersection contour. Comparing A
fragments against different curved-cutter facet planes gave contradictory local
sides; the resolver now identifies the retained cap first and classifies each
connected A patch by exact winding along the shared cut edges. Incomplete cuts
still fail, without a tolerance or fallback mesh.

The impeller fillet regression remains skipped until that endpoint closure is
supported. No lossy predicate or fallback mesh is introduced.

`CurvedShoulderFinFilletTests.ReducedFinRootReproducesPartialTerminationLimitation`
is a smaller active reproducer: one revolved curved shoulder and one five-section
fin (rather than the full impeller's nine sections), with only the root edge
treated. Exact endpoint splitting now rejects incomplete cuts for both treatments
before cap construction. For fillet, three disconnected contours are found, none
spanning both contact rails. Two
start on one rail but end away from either rail. Do not join their endpoints by
tolerance or interpret them as a complete cut. Source-edge matching succeeds;
the remaining work involves the endpoint support domain and closure construction.
The fix must keep one shared exact boundary through trimming and cap construction;
merely accepting the original-face segments is insufficient. Splitting the full
extended face exposes intersecting triangles in the tangential extension collar.
Restoring the construction domain before extension avoids that conflict in the
finer-resolution reproducer, but still leaves inconsistent closure-region sides;
the coarse reproducer also fails the support-fidelity check. Neither experiment
is enabled in production, and neither establishes that a different blend
construction algorithm is required. Metadata-free geometry must remain supported.
