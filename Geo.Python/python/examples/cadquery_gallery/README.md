# CadQuery examples compatibility walk

Source: [CadQuery Examples](https://cadquery.readthedocs.io/en/latest/examples.html),
in page order. The numbered scripts were exercised independently with
`camber.cqcompat.Workplane`.

Status through the final gallery example:

| Result | Examples | Notes |
| --- | --- | --- |
| Runs | 01-19, 23, 25-33, 35-38 | Examples 35-38 are compact demonstrations of the same feature paths, not verbatim reproductions of the larger gallery models. |
| Runs, partial CadQuery parity | 20-22, 34, 39 | Outward shelling is watertight but has sharp intersection joins; example 39's callback curve is sampled into planar line segments rather than represented as a fitted spline. |
| Cannot run yet | 24 | `extrude("next")` is not implemented. |

These scripts use Camber's compatibility import rather than running the original
CadQuery files unchanged. A script running is not proof of identical geometry.
When a numbered script is run directly, it opens the Camber viewer on the
finished result after its checks. Example 24 currently stops at its unsupported
`extrude("next")` call, so it has no finished result to display.

Estimated order to unblock the remaining examples (easiest to hardest). Times
are active engineer-days for one developer familiar with the codebase, including
focused tests and running the affected gallery scripts; they are estimates, not
commitments.

| Rank | Examples | Missing capability | Estimate | Work needed |
| --- | --- | --- | --- | --- |
| Done | 27-28, 31 | 2D profile offset, selected-edge conversion, tagged-face selection | Complete | Implemented in the compatibility layer using Camber's native sketch offset operation. `toPending()` currently handles straight feature edges represented by mesh vertices. |
| Done | 33 | `split()` | Complete | Uses CSG with a cutter plane footprint covering the live solid's AABB. Surface trim was checked first but produced a non-watertight half for the holed gallery body. Both retained halves are available via `vals()`/`all()`. |
| Partial | 20-22 | Outward shell | Rounded joins missing | Positive thickness expands the exterior and keeps the original solid as the cavity. Closed, single-opening, and multiple-opening boxes run, but corners are sharp. |
| Partial | 34 | Union-seam outward shell | Rounded joins missing | At the bottle's topology-changing neck seam, each closed union operand is expanded and re-united with CSG before cutting the cavity. The script runs, but its outward joins are sharp. |
| Done, partial parity | 39 | `parametricCurve()` | Implemented | Evaluates the Python callback and adaptively samples planar points using the Part tolerance or explicit `tol`; current sketch geometry is a polyline, not a native fitted spline. |
| Done | 25 | `eachpoint()` | Complete | Creates a solid per pushed point, supports local workplane placement, and shows the full stack in the viewer. |
| 1 | 24 | `extrude("next")` | 3-8 days | The custom-axis revolve is fixed and tested. Determine the next limiting face along the extrusion direction, stop the extrusion at that face, and test the resulting trim against curved as well as planar targets. |

The estimates assume support sufficient for these gallery cases, not complete
CadQuery parity. `parametricCurve()` currently supports planar callback output
and sketch wires; it does not fit a native spline or support 3D curves and
CadQuery's spline-fitting controls. Total: 38 scripts run (5 with known
geometry/parity differences) and 1 currently stops at a missing feature.

The remaining cases remain as failing reproducers. The ledger covers the gallery through its final
cycloidal-gear example.

From `Geo.Python/python`, run a script with the repository Python environment
and `PYTHONPATH` pointing to that directory. Each numbered file is an
independent check; files marked blocked are expected to stop at the listed
limitation.
