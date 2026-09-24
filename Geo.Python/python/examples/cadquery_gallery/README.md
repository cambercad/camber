# CadQuery examples compatibility walk

Source: [CadQuery Examples](https://cadquery.readthedocs.io/en/latest/examples.html),
in page order. The numbered scripts were exercised independently with
`camber.cqcompat.Workplane`.

Status through the final gallery example:

| Result | Examples | Notes |
| --- | --- | --- |
| Pass | 01-19, 23, 26-33, 35-38 | Examples 35-38 are compact demonstrations of the same feature paths, not verbatim reproductions of the larger gallery models. |
| Blocked | 20-22, 24-25, 34, 39 | Ranked below by estimated implementation effort, easiest first. |

Estimated order to unblock the remaining examples (easiest to hardest). Times
are active engineer-days for one developer familiar with the codebase, including
focused tests and running the affected gallery scripts; they are estimates, not
commitments.

| Rank | Examples | Missing capability | Estimate | Work needed |
| --- | --- | --- | --- | --- |
| Done | 27-28, 31 | 2D profile offset, selected-edge conversion, tagged-face selection | Complete | Implemented in the compatibility layer using Camber's native sketch offset operation. `toPending()` currently handles straight feature edges represented by mesh vertices. |
| Done | 33 | `split()` | Complete | Uses CSG with a cutter plane footprint covering the live solid's AABB. Surface trim was checked first but produced a non-watertight half for the holed gallery body. Both retained halves are available via `vals()`/`all()`. |
| 1 | 39 | `parametricCurve()` | 2-5 days | Provide a clear parameter interval and sampling tolerance; evaluate/validate callback output; create a closed sampled sketch profile for the cycloidal gear; test closure, self-intersections/failures, and twist extrusion. |
| 2 | 25 | `eachpoint()` | 3-6 days | Apply each pushed location to the callback-produced profiles/features, respect local versus world coordinates, and combine the resulting lofts into a usable result. Test multiple locations, placement, and invalid callback results. |
| 3 | 24 | `extrude("next")` | 3-8 days | The custom-axis revolve is fixed and tested. Determine the next limiting face along the extrusion direction, stop the extrusion at that face, and test the resulting trim against curved as well as planar targets. |
| 4 | 20-22, 34 | Outward shell | 10-20 days | Implement outward support offsets, trim adjacent supports, create openings/rims, and validate watertightness and self-intersections for unselected and selected faces. Exercise boxes, multiple openings, and the bottle's curved profile. |

The estimates assume support sufficient for these gallery cases, not complete
CadQuery parity. The `extrude("next")` range needs refinement once the target
geometry in this example is investigated. Total: 32 passing scripts and 7 blocked scripts.

The outward-shell and missing-method cases remain as
failing reproducers. The ledger covers the gallery through its final
cycloidal-gear example.

From `Geo.Python/python`, run a script with the repository Python environment
and `PYTHONPATH` pointing to that directory. Each numbered file is an
independent check; files marked blocked are expected to stop at the listed
limitation.
