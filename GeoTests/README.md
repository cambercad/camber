# Native tests

Run the suite normally; parallel execution is enabled by default:

```sh
dotnet test GeoTests/GeoTests.csproj -c Release
```

`xunit.runner.json` runs up to four independent test classes concurrently with
xUnit's conservative scheduler. Tests within a class run sequentially. CAD
operations also use native worker threads, so increasing the test-worker count
does not necessarily improve throughput. The configuration is copied into the
test output directory automatically; no special command-line flags are needed.

## State isolation

Geometry tests create their own `GeoAPI` instances. Their cleanup releases the
static registry references while preserving global name counters, so concurrent
builders keep unique generated names and their own meshes stay usable.
The internal cleanup overload does not change the public `GeoAPI.Clear()` API.

Only tests that deliberately reset process-wide counters or exercise global
registry reset behavior belong to the exclusive `GlobalCadState` collection.
Do not put ordinary geometry tests in that collection or reset global counters
from a parallel test. Use unique temporary filenames for file-based tests.

See [xUnit's parallelism documentation](https://xunit.net/docs/running-tests-in-parallel)
for collection scheduling and [runner configuration](https://xunit.net/docs/config-xunit-runner-json)
for worker-count overrides.

## Verification on the development machine

The final four-class run passed 838 tests with two existing skips in 2 min 16 s.
The prior broadly serialized run took 3 min 31 s; this is roughly 35% less wall
time. An initial eight-class run took 2 min 19 s. These are individual runs,
not a statistical performance benchmark; workload and hardware affect timings.
