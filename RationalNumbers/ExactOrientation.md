# Exact orientation with an integer-only bound

`BigRationalHybrid.SignOfOrient3D` returns the exact sign of the determinant
of the represented rational coordinates. It first uses an exact Int128 path
when all coordinates use Int32 storage, then tries an integer bound, and
otherwise evaluates the full determinant with arbitrary-precision integers.
No coordinate is replaced or modified. No floating-point arithmetic is used
in any of these orientation paths. The obsolete `TrySignOfOrient3DDoubleFilter`
entry point delegates to the integer bound for compatibility.

## Proof of the bounded path

Let S = 2^20. For each coordinate x, compute q = trunc(S*x) by exact
BigInteger division, or an exact shift for an in-range integer. Therefore
|S*x - q| < 1, for negative values as well as positive ones. Accept the
conversion only when |q| <= 2^40.

Subtract the integer coordinates of the fourth point from each of the first
three. Each resulting integer difference a differs from its exact scaled
rational difference by less than 2. Let M be the largest absolute integer
difference. The exact scaled determinant has six signed cubic monomials.
For any one monomial, with |a|, |b|, |c| <= M and errors |e|, |f|, |g| < 2,
expansion and the triangle inequality give

    |(a+e)(b+f)(c+g) - abc| <= 6*M^2 + 12*M + 8.

Thus E = 36*M^2 + 72*M + 48 encloses the total determinant error. If the
integer determinant D is strictly greater than E, the exact determinant is
positive. If D is strictly less than -E, it is negative. Every other result,
including D = +/-E and exact zero, falls back. Scaling by positive S^3 does
not change the sign.

This deliberately loose bound remains valid under cancellation, unreduced
fractions and arbitrarily large numerators or denominators. It is not an
empirical tolerance. Input values beyond the fixed-width bound are handled
by the arbitrary-precision path.

## Overflow bounds

The checked conversion domain gives M <= 2^41. Each product has magnitude
at most M^3. The sum of magnitudes of all six monomials is at most
6*2^123 < 2^126, safely below the signed Int128 limit 2^127. This also bounds
every intermediate determinant operation, including the grouped differences.
E is smaller still. Coordinate subtraction and absolute value fit in Int64.
The quotient is compared as a BigInteger before conversion to Int64. Integer
coordinates are range-checked before shifting.

For the separate all-Int32 fast path, differences have magnitude below
2^32, hence the same six-product argument bounds every intermediate by
6*2^96, safely within Int128. Arbitrary Int64 inputs must not use this path.

## Regression coverage

`GeoTests/IntegerOrientationBoundsTests.cs` compares both the dispatcher and
any certified sign against a separate rational determinant calculation.
Coverage includes D=E-1,E,E+1 and their negative counterparts, positive and
negative coordinate range endpoints and adjacent scaled integers, fractional
remainders straddling the conversion boundary, all 4096 coordinate-limit
corner combinations, all point permutations of threshold cases, mixed
Int32 boundary inputs, full Int64 endpoints, exact coplanarity, tiny signed
heights, thousand-bit translations, unreduced/negative-denominator inputs,
and seeded rational/cancellation cases. Tests supplement the bounds above;
they are not the correctness argument on their own.

## Performance scope

The bound is useful when rational determinant evaluation needs large integer
products and common denominators. It is not universally faster: a synthetic
small shared-denominator workload and exact coplanar inputs paid extra filter
overhead, whereas 200-bit rationals showed a large gain. The full-bike timing
and watertightness/interference checks are recorded in `output/bike_exact_perf`
and summarized in `Geo.Python/python/racing_bike_audit.md`. No additional
parallelism is introduced by this change; the predicate has only local state
and remains safe for the existing parallel CSG processing.
