# Known fillet limitations

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
with both endpoints on the same curved face. The impeller request instead needs
a rolling-ball partial-edge termination surface, which is not implemented.

The rotated curved-leg cap-rim regression is enabled. Its trim cutter was an
exactly joined oriented disk with a closed intersection contour. Comparing A
fragments against different curved-cutter facet planes gave contradictory local
sides; the resolver now identifies the retained cap first and classifies each
connected A patch by exact winding along the shared cut edges. Incomplete cuts
still fail, without a tolerance or fallback mesh.

The impeller fillet regression remains skipped until rolling-ball partial-edge
termination is supported. No lossy predicate or fallback mesh is introduced.

`CurvedShoulderFinFilletTests.ReducedFinRootReproducesPartialTerminationLimitation`
is a smaller active reproducer: one revolved curved shoulder and one five-section
fin (rather than the full impeller's nine sections), with only the root edge
rounded. Its exact intersection contour crosses from the artificial support
extension onto the real terminating face, where current closure construction
rejects it. This narrows the missing work: the limitation is in endpoint-cap
construction across the real/extended support seam, not source-edge matching.
The fix must keep one shared exact boundary through trimming and cap construction;
merely accepting the original-face segments would create a non-watertight patch.
