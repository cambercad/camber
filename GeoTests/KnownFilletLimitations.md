# Known fillet limitations

Two intended-success regression cases are explicitly skipped in the normal
suite. The user authorized excluding known failing tests or fixing their
underlying issues. These skips record missing kernel support; they do not mean
that these geometries are fixed. No production rejection or test assertion was
removed, and all other cases in both test classes remain enabled.

## Impeller root

`ImpellerLoftSupportTests.OriginalRootFilletHasAConnectedSpine` requests a
1.2 mm round at the blade/hub neck intersection. `Intersector.IntersectSurfaces`
flattens multiple intersection strips into paired segment lists. The open-edge
path in `BlendEdge.ComputeSpineAndBoundaries` treats those lists as one spine.
The exact continuity check in `ExtractCurveFromSegments` rejects segment 84 of
96 because it belongs to a disconnected branch. The closed-edge path already
has contour separation, but this open-edge case needs safe correspondence to
the source edge, not arbitrary nearest/longest-branch selection or a bridge
across the gap. Retained-support and loft-deviation regressions remain active.

## Rotated curved leg

`FigureShellRobustnessTests.RotatedCurvedLegProfileAcceptsBothCapRimFillets`
requests 0.1 mm cap-rim rounds in the oblique frame defined in that fixture.
The generated blend does not provide a complete oriented cut:
`Resolver.DetectPartialCut` finds contradictory exact side classifications in
one connected cluster. This is not an open-loop or T-junction failure: the
blend patches join exactly and intersect the solid in one closed contour. The
conflicting evidence comes from different triangles of the curved cutter, so
the remaining question is whether the cutter geometry or the partial-cut
classification is wrong. The check must not be bypassed or replaced with a
tolerance. The unrotated 0.1/0.2 mm cases and transformed stepped profile remain
enabled.

To re-enable either regression, remove its `Skip` argument, retain all existing
watertightness/support/volume assertions, and run the focused tests followed by
the complete native suite. No lossy predicate or fallback mesh is introduced.
