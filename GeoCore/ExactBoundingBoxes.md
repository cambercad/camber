# Exact conservative bounding boxes

The bounding-box optimization uses the same rational coordinates and preserves
integer enclosures. It introduces no tolerance or approximate geometry test.

## Point enclosure

For an exact rational x=n/d with d>0, integer division gives n=q*d+r,
where q truncates toward zero and |r|<d. Therefore:

- for n>=0: q <= x < q+1;
- for n<0: q-1 < x <= q.

`Rat3HybridExtensions.GetBox` computes this quotient and remainder directly
with BigInteger.DivRem. Canonical Int32 coordinates use an exact integer fast
path. This avoids allocating an absolute-value rational and comparing the
rounded integer with the rational again. It retains the previous one-unit
padding at exact integers, so normal in-domain results are unchanged.

If q is outside Int32, conversion throws. At q=Int32.MinValue or MaxValue,
a remainder pointing outside the domain also throws. An exact endpoint is
representable: its outward padding is clamped to that endpoint, avoiding
integer overflow. No finite integer box can enclose an out-of-domain rational;
throwing is necessary instead of manufacturing a wrapped box.

## Triangle enclosure

The lower and upper enclosure functions are monotone throughout their valid
domain, including at zero, integer boundaries and clamped endpoints. Hence
min(L(x_i)) = L(min(x_i)) and max(U(x_i)) = U(max(x_i)).

`ResolverTriangle.GetBounds` can therefore enclose the three points separately
and combine their integer boxes. It returns exactly the same box as selecting
rational extrema first, while avoiding twelve rational cross-product
comparisons. Geometry and exact predicates are unchanged.

## Existing floating-point BVH representation

The BVH uses binary32 boxes only to select potential intersections. The exact
narrow phase decides geometry. This representation must enclose its integer
input; missed candidates would invalidate exactness downstream.

A plain Int32-to-float conversion may round inward. A fixed one-unit margin
is insufficient at large coordinates because the float spacing exceeds one.
`Box3I.GetBoundsF` now moves each converted lower endpoint to the immediately
preceding float and each upper endpoint to the immediately succeeding float.
This guarantees enclosure: conversion selects a neighboring representable
float, and one step outward reaches or passes the exact integer. All Int32
values are finite and well within the float exponent range.

The existing extra positive padding is retained, but is not relied on for
correctness. Box spans are computed in Int64 to avoid Int32 subtraction
overflow. The validation comparison widens the binary32 endpoint to binary64:
every binary32 value and every Int32 value is exactly representable there,
so that comparison itself is exact. Comparing float directly with int would
round the integer to float and could conceal an invalid enclosure.

Regression tests reproduced four old point-box failures and six old float-box
failures before these fixes. Tests cover signed integer endpoints, immediately
outside and inside the domain, exact padding, 1600-bit fractional offsets,
float precision boundaries, full-width spans, random narrow large-coordinate
boxes, and equivalence with the previous triangle-extrema algorithm. The
mathematical enclosure argument, rather than test coverage alone, establishes
why the optimization cannot reject a true intersection.
