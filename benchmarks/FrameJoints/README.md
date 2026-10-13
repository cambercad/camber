# Racing-bike frame profiling

`Program.cs` reproduces the hollow down tube, top/head and down/head rounds,
and top/seat round directly in C#. Each construction step reports elapsed time
and triangle count; the final mesh must be exactly watertight. It uses the
bike's original dimensions, 0.2 mm model deviation and 0.1 mm fillet deviation.
The seat tube stops at the nominal seat datum; the complete bike also extends
the tube through the seat collar.

From the repository root, with the .NET 10 SDK on `PATH`:

```bash
dotnet run --project benchmarks/FrameJoints -c Release
```

Build separately before collecting a sampling trace:

```bash
dotnet build benchmarks/FrameJoints -c Release
dotnet-trace collect --profile dotnet-sampled-thread-time --format Speedscope \
  -o /tmp/frame.nettrace --show-child-io -- \
  dotnet benchmarks/FrameJoints/bin/Release/net10.0/FrameJoints.dll
```

The Python profiler runs the complete frame using the **installed wheel**,
without rendering. It writes a cProfile file and prints cumulative timings,
watertightness and volume:

```bash
python benchmarks/FrameJoints/profile_frame.py --output /tmp/frame.prof
```

For a full build followed by the interactive viewer:

```bash
python Geo.Python/python/examples/engineering_models/racing_bike.py
```

This prints construction-and-solve time before opening the viewer. Install the
local wheel with its `[view]` extra to use this mode.

cProfile measures native calls as a whole; use the C# sampling trace to see
inside those calls. A Speedscope thread-time trace includes waiting threads,
so inspect its CPU_TIME samples rather than summing all thread durations.

## Measurements, 2026-10-06

Linux x86_64, Intel i9-14900K, Python 3.13.9, .NET SDK 10.0.401. The full
bike uses four workers and 0.2 mm deviation. These are individual workstation
runs, not controlled statistical benchmarks. Build timing excludes imports,
rendering and the subsequent verification pass.

The initial PyPI 0.1.7 frame-only profile took 141.04 seconds:

| Work | Seconds |
| --- | ---: |
| Five native fillets | 85.09 |
| Subtractions | 34.00 |
| Unions | 14.11 |
| Batched unions | 6.69 |
| Lofting | 0.94 |

The C# trace identified costly exact rational comparisons in
`ValidateBlendPatchBoundaries`: every unshared patch boundary scanned all
volume triangles, and T-junction detection compared every boundary pair.
Both searches now use the existing BVH with conservative integer enclosures
and outward-rounded floating-point boxes. Exact rational bounds, coplanarity,
clipping, coverage and T-junction predicates remain the final decisions.

After updating to main `c1b2838`, before/after measurements for this change:

| Work | Baseline seconds | Indexed seconds | Reduction |
| --- | ---: | ---: | ---: |
| C# three-junction benchmark | 43.29 | 34.24 | 21% |
| Full bike construction and solve | 179.44 | 134.33 | 25% |
| Frame subsystem | 172.67 | 128.00 | 26% |
| Fork subsystem | 60.99 | 42.06 | 31% |

Subsystem times overlap and must not be added. The earlier full PyPI run was
174.19 seconds. The original checkout's C# benchmark improved from 40.22 to
28.31 seconds; those numbers should not be mixed with the updated-main results.
An additional optimized full-bike run with the interactive viewer enabled took
113.01 seconds for construction and solve (107.70 seconds for the frame).
That repeat is recorded separately because workstation load varied. Viewer
startup and the time spent inspecting the model are outside this measurement.

The updated-main C# benchmark retains identical triangle counts at every
stage (51,710 at completion) and exact watertightness. The optimized bike has
832 watertight solids, frame volume 1,913,427.454343625 mm³, and a converged
assembly solve with no unsatisfied constraints. The 25 focused C# regression
tests pass, including coverage and rejection of gaps of 2^-80 at large signed
coordinates, empty inputs, T-junction rejection, frame joints and closed caps.

The change reduces boundary validation work; exact Boolean construction and
the remaining fillet calculations still dominate runtime.

## Follow-up experiments

With the indexed kernel, two additional baseline runs took 29.24 and 29.38
seconds. Replacing plane-distance construction with the shared exact orientation
predicate took 28.53 seconds. Combining it with projected exact 2D clipping took
29.81 seconds. Returning immediately when a single triangle covers the complete
edge took 29.56 seconds. These small differences do not establish a reliable
additional win, so all three experiments were removed. The retained kernel
change is the spatial index described above.

Coverage tests now also span all three projection axes, oblique planes, both
triangle windings, and partial coverage across a triangle seam (33 focused C#
tests passed). The separate Ctrl-state fix passes 35 X11 viewer GUI tests.
The full-bike verification run with that fix took 120.25 seconds for
construction and solve (115.31 seconds for the frame), again with 832 watertight
solids and no unsatisfied assembly constraints.
