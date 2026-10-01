# Exact rational hashing performance

The rational equality fix removes the old Debug-only precondition that callers
must normalize before calling `Equals`. `Equals` now agrees with exact `==`.
`GetHashCode` also supports unreduced fractions, so `1/2` and `2/4` hash alike,
and rational integers hash like their `long` representation. Unequal values may
collide; dictionary equality still distinguishes them exactly.

Hashing does not mutate or cache normalization. **An unsimplified value pays for
GCD reduction on every hash call.** Normalize once before repeatedly using a key.
Existing geometry deduplication paths already do this. Normalized hybrid keys
use a direct hash path without GCD. Plain `BigRational` has no normalized flag
and canonicalizes a local value for hashing.

Ordinary and explicit hybrid copies share their backing wrapper until
`Simplify` replaces it with a private normalized copy. This prevents another
copy, a dictionary key, or a concurrent reader from observing a partially
updated fraction. Hashing and arithmetic do not write the shared wrapper.

## Reproducing the benchmark

```sh
dotnet build benchmarks/RationalHashing/RationalHashing.csproj -c Release
DOTNET_TieredCompilation=0 dotnet benchmarks/RationalHashing/bin/Release/net10.0/RationalHashing.dll
```

The optional argument `micro` runs hashing and deduplication only;
`construction` runs the cylinder Boolean workload only. Output is JSON lines,
with five measured samples after two warmups, total managed allocated bytes,
and a checksum. Hash checksums vary between processes because .NET salts hashes.

`hex_bolt` builds the M10 x 35 mm scene from
`Geo.Python/python/examples/standalone/hex_bolt.py` using the same geometry
recipe through the C# API. It uses two warmups and five timed builds.
`hex_bolt_once` reports one build's triangle count, peak and current process
working set, and live managed bytes. `hex_bolt_verify` builds three times and
reports triangle count, exact-position watertightness, and double-precision
signed volume plus order-independent triangle-coordinate signatures for each
result. The `bytes` field in timed output is cumulative
allocation during one build, not peak memory.

### FastBigInteger and deterministic Boolean follow-up

Windows, .NET SDK 10.0.401, Release, tiered compilation disabled; detached
baseline at `3c5c6c8` and the same C# hex-bolt harness in both builds. Five
timed builds followed two warmups. Median hex-bolt time fell from 15.24 s to
11.86 s; median cumulative allocation fell from 18.00 GB to 9.31 GB.
The baseline's triangle count varied across builds (80,402 to 80,406).
The candidate sorted intersection segments only for triangles written by
multiple overlap workers. Its five timed builds and six verification builds
all produced 80,410 triangles; verification builds also matched in both
order-independent mesh signatures and were watertight. The signed volume was
3.649095588218151e-6 in those verification runs.

A subsequent small geometry change shares exact rational orientation cache
entries across point permutations, rejects nonintersecting segments before
running two more orientation tests, and reuses trim segment directions and
coordinate comparisons. Endpoint collinearity uses the existing exact
cross-product sign predicate. Floating-point orientation keeps its original
evaluation order.

The preserved pre-change build measured 12.12 s median over five timed builds;
the final build measured 11.11 s across ten timed builds in two processes
(8.3% less time). Median cumulative allocation fell from 9.31 GB to 8.78 GB
(5.7% less). All timed builds produced 80,410 triangles, and three verification
builds matched the previous volume and both mesh signatures and were watertight.
The Release suite passed with 1,233 passed, 3 skipped, and no failures. Raw
samples and profiler notes are in `results/geometry-reuse-windows-net10.json`.

The older comparison below concerns a different rational-hashing change and
is retained as historical benchmark data.

Use independent source/output directories for baseline and candidate. The
recorded baseline is `bda1ede`; the candidate differs **only in
`RationalNumbers/BigRational.cs` and `BigRationalHybrid.cs`**. The other sweep/cap
fixes are deliberately excluded from this comparison. The same harness was
built against both snapshots.

## Recorded comparison

### Camber-owned BigRational replacement (2026-10-01)

Windows/.NET 10.0.401, Release with tiered compilation disabled, compared
against `8261851`. Each process ran two warmups and five measured samples;
small workloads have 15 samples per variant, and the threaded bolt has five.
Benchmark processes ran sequentially, without concurrent tests or builds.
The final implementation restores small-number addition and same-denominator
equality fast paths, hashes already-normalized internal values directly, and
caches the large constants used by the unchanged rational-to-double conversion.

Median cylinder Boolean time decreased from 59.86 ms to 56.17 ms (6.2%),
with 0.5% more cumulative allocation. The threaded bolt decreased from 8.34 s
to 7.99 s (4.1%) and allocated 1.0% less; every build had 80,410 triangles.
Integer, normalized, and unreduced hashing were within 1.6% of the baseline;
point deduplication was 3.0% slower and point insertion 8.7% slower, with
unchanged allocation. These workloads do not establish performance for all
modeling operations. Raw samples, including the initial unoptimized candidate,
are in `results/rational-replacement-windows-net10.json`.

### Earlier rational-hashing comparison

Linux, .NET SDK 10.0.401, Release, tiered compilation disabled. Three processes
per variant in baseline/candidate/candidate/baseline/baseline/candidate order;
15 timed samples per workload per variant. Tests and other benchmark processes
were not running concurrently. Values below are medians.

| Workload per sample | Baseline ms | Candidate ms | Change |
| --- | ---: | ---: | ---: |
| 2,048,000 integer hashes | 0.7754 | 1.3288 | +0.5534 ms |
| 1,024,000 normalized rational hashes | 32.6453 | 22.9311 | -29.8% |
| 102,400 unreduced rational hashes* | 3.1614 | 29.5443 | +26.3829 ms |
| Deduplicate 4,096 points, 30 times | 21.0021 | 21.4638 | +2.2% |
| Insert 4,096 points, 10 times | 7.5033 | 7.3692 | -1.8% |
| Construct and union two cylinders, 8 times | 87.9006 | 85.4714 | -2.8% |

*The old raw hash is not correct for arbitrary unsimplified keys: equivalent
fractions can hash differently. This row measures the cost of supplying the
missing correctness guarantee, not two equivalent implementations. The new
fallback costs approximately 0.289 microseconds and 85 allocated bytes per call
for these 128-bit numerators and 96-bit denominators. Other sizes differ.

The integer-only loop regressed by about 0.27 ns per call. Do not interpret
small construction differences as a demonstrated general speedup. The measured
point deduplication regression is 2.2%; this benchmark does not establish the
performance of the entire bike or LEGO gallery.

Deduplication allocation remained 35,935,240 bytes/sample. Point creation fell
from 15,920,584 to 13,954,504 bytes/sample (12.3% less). Cylinder construction
allocation rose from 62,820,568 to 63,315,416 bytes/sample (0.8%). Unreduced
hashing allocated 8,739,112 bytes/sample; normalized and integer hash loops had
no per-hash allocations.

A separate instrumented construction run counted integer, normalized-rational,
and unsimplified-rational hash calls. It was excluded from all timing results.
The counts and all timing samples are retained in `results/`.

## Investigation and regression coverage

The first safe implementation allocated an extra wrapper when copying then
normalizing. Copying the private wrapper reference and detaching on mutation
removes that duplicate allocation. A whole-struct copy (`this = source`) caused
a repeatable .NET 10 deduplication slowdown; explicit field assignments retain
the same value semantics without that measured regression. A larger hash body
also hurt hot paths, so canonical hashes are direct and the GCD fallback is
separate. `dotnet-trace` was used to distinguish explicit normalization from
hashing costs.

`GeoTests/RationalEqualityTests.cs` checks scaled and signed fractions, zero,
both `long` boundaries, 2,048-bit integers, sub-double differences, intentional
hash collisions, dictionary lookup after simplification, actual geometry point
deduplication, ordinary/explicit copy isolation, and concurrent normalization
of independent copies. All predicates and normalization use exact integers.

## Full-suite validation of the final implementation

`dotnet test GeoTests/GeoTests.csproj -c Debug --no-restore` and the corresponding
Release command each completed with **1,111 passed, 0 failed, 4 skipped** on
Linux/.NET 10. The four existing skips were unchanged. The suite now has 41
additional cases compared with the pulled baseline. Debug and Release were run
concurrently for correctness validation; their elapsed times are not benchmark
results. Windows was not available for a direct run.
