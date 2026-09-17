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

Use independent source/output directories for baseline and candidate. The
recorded baseline is `bda1ede`; the candidate differs **only in
`RationalNumbers/BigRational.cs` and `BigRationalHybrid.cs`**. The other sweep/cap
fixes are deliberately excluded from this comparison. The same harness was
built against both snapshots.

## Recorded comparison

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
