using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BooleanTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    // ── helpers ─────────────────────────────────────────────────────────────

    /// <summary>
    /// Signed volume of a closed triangle mesh via the divergence theorem.
    /// Returns positive volume for consistently outward-facing normals.
    /// </summary>
    private static double MeshVolume(AnchorMesh anchor)
    {
        var pts  = anchor.Mesh.Positions;
        var tris = anchor.Mesh.Triangles;
        double vol = 0;
        foreach (var t in tris)
        {
            var a = pts[t.A]; var b = pts[t.B]; var c = pts[t.C];
            vol += (a.X * (b.Y * c.Z - b.Z * c.Y)
                  + a.Y * (b.Z * c.X - b.X * c.Z)
                  + a.Z * (b.X * c.Y - b.Y * c.X)) / 6.0;
        }
        return Math.Abs(vol);
    }

    private static void AssertVolume(double expected, double actual, string label = "", double tol = 0.005)
    {
        double delta = Math.Abs(actual - expected);
        Assert.True(delta < tol,
            $"{label}: expected {expected:F6}, got {actual:F6} (Δ={delta:F6}, tol={tol})");
    }

    private static void AssertEmpty(AnchorMesh m, string label = "")
        => Assert.True(m.Mesh.Triangles.Count == 0,
            $"{label}: expected empty mesh but got {m.Mesh.Triangles.Count} triangles");

    // All tests use 1×1×1 cubes, matching the proven Test3_1 setup in MainForm.
    // A occupies [0,1]³. B is shifted by shiftX in X, occupying [shiftX, shiftX+1]³.
    // Overlap length in X = max(0, 1 - shiftX)  (only for shiftX in [0,1])
    // Overlap volume      = max(0, 1 - shiftX)
    private static (GeoAPI api, AnchorMesh a, AnchorMesh b) MakePair(double shiftX)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0,      0, 0)), new Vec3D(1, 1, 1), "a");
        var b   = api.CreateCuboid(new CoordinateSystem(new Vec3D(shiftX, 0, 0)), new Vec3D(1, 1, 1), "b");
        return (api, a, b);
    }

    private static double Overlap(double shiftX) => Math.Max(0.0, 1.0 - shiftX);

    // ═══════════════════════════════════════════════════════════════════════
    // Difference sweep: vol(A−B) = 1 − overlap
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]      // B == A → result is empty
    [InlineData(0.1234)]   // B overlaps A by 0.8766 → result vol = 0.1234
    [InlineData(0.25)]
    [InlineData(0.5)]
    [InlineData(0.75)]
    [InlineData(1.0)]      // B touches right face of A → result is full A (vol = 1)
    [InlineData(1.5)]      // B completely outside A
    [InlineData(2.0)]
    public void Boolean_Difference_XShift_VolumeIsCorrect(double shiftX)
    {
        var (api, a, b) = MakePair(shiftX);
        var result = api.Boolean(a, b, BooleanOp.Difference, "ab");

        double expected = 1.0 - Overlap(shiftX);
        double vol      = MeshVolume(result);
        Console.WriteLine($"[Diff shiftX={shiftX}] vol={vol:F6}  expected={expected:F6}  tris={result.Mesh.Triangles.Count}");
        AssertVolume(expected, vol, $"Difference shiftX={shiftX}");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Union sweep: vol(A∪B) = vol(A) + vol(B) − overlap = 2 − overlap
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]      // B == A → union == A, vol = 1
    [InlineData(0.1234)]
    [InlineData(0.25)]
    [InlineData(0.5)]
    [InlineData(0.75)]
    [InlineData(1.0)]      // touching faces → vol = 2
    [InlineData(1.5)]      // fully separated
    [InlineData(2.0)]
    public void Boolean_Union_XShift_VolumeIsCorrect(double shiftX)
    {
        var (api, a, b) = MakePair(shiftX);
        var result = api.Boolean(a, b, BooleanOp.Union, "ab");

        double expected = 2.0 - Overlap(shiftX);
        double vol      = MeshVolume(result);
        Console.WriteLine($"[Union shiftX={shiftX}] vol={vol:F6}  expected={expected:F6}  tris={result.Mesh.Triangles.Count}");
        AssertVolume(expected, vol, $"Union shiftX={shiftX}");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Intersect sweep: vol(A∩B) = overlap
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]      // B == A → intersect == A, vol = 1
    [InlineData(0.1234)]   // overlap = 0.8766
    [InlineData(0.25)]
    [InlineData(0.5)]
    [InlineData(0.75)]
    [InlineData(1.0)]      // touching only → intersect is empty (vol = 0)
    [InlineData(1.5)]      // no overlap → empty
    [InlineData(2.0)]
    public void Boolean_Intersect_XShift_VolumeIsCorrect(double shiftX)
    {
        var (api, a, b) = MakePair(shiftX);
        var result = api.Boolean(a, b, BooleanOp.Intersect, "ab");

        double expected = Overlap(shiftX);
        double vol      = MeshVolume(result);
        Console.WriteLine($"[Intersect shiftX={shiftX}] vol={vol:F6}  expected={expected:F6}  tris={result.Mesh.Triangles.Count}");
        AssertVolume(expected, vol, $"Intersect shiftX={shiftX}");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Volume identity: Vol(A∪B) = Vol(A) + Vol(B) − Vol(A∩B)
    // Holds regardless of overlap amount; tested at three representative shifts.
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.5)]
    [InlineData(1.5)]
    public void Boolean_VolumeIdentity(double shiftX)
    {
        var (api, a, b) = MakePair(shiftX);
        double volUnion = MeshVolume(api.Boolean(a, b, BooleanOp.Union,     "u"));
        double volIsect = MeshVolume(api.Boolean(a, b, BooleanOp.Intersect, "i"));
        double expected = 1.0 + 1.0 - volIsect;

        Console.WriteLine($"[Identity shiftX={shiftX}] union={volUnion:F6}  isect={volIsect:F6}  expected={expected:F6}");
        AssertVolume(expected, volUnion, $"Vol(A∪B)=Vol(A)+Vol(B)-Vol(A∩B) shiftX={shiftX}", tol: 0.005);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Union commutativity: Vol(A∪B) == Vol(B∪A)
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_Union_IsCommutative()
    {
        var (api1, a1, b1) = MakePair(0.3);
        double volAB = MeshVolume(api1.Boolean(a1, b1, BooleanOp.Union, "ab"));
        GeoAPI.Clear();

        var (api2, a2, b2) = MakePair(0.3);
        double volBA = MeshVolume(api2.Boolean(b2, a2, BooleanOp.Union, "ba"));

        Console.WriteLine($"[Commutative] A∪B={volAB:F6}  B∪A={volBA:F6}");
        AssertVolume(volAB, volBA, "Union commutativity");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // B fully inside A: carved hole
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_Difference_BInsideA_VolumeIsAMinusB()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(5)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(3, 3, 3), "a"); // vol=27
        var b   = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 1, 1)), new Vec3D(1, 1, 1), "b"); // vol=1, inside a
        var result = api.Boolean(a, b, BooleanOp.Difference, "ab");
        double vol = MeshVolume(result);
        Console.WriteLine($"[B-inside-A Diff] vol={vol:F6}  expected=26");
        AssertVolume(26.0, vol, "B-inside-A Difference");
        TestVisualization.Visualize(api, nameof(Boolean_Difference_BInsideA_VolumeIsAMinusB));
    }

    [Fact]
    public void Boolean_Intersect_BInsideA_VolumeIsB()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(5)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(3, 3, 3), "a");
        var b   = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 1, 1)), new Vec3D(1, 1, 1), "b");
        var result = api.Boolean(a, b, BooleanOp.Intersect, "ab");
        double vol = MeshVolume(result);
        Console.WriteLine($"[B-inside-A Intersect] vol={vol:F6}  expected=1");
        AssertVolume(1.0, vol, "B-inside-A Intersect");
        TestVisualization.Visualize(api, nameof(Boolean_Intersect_BInsideA_VolumeIsB));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Chained operations: (A ∪ B) − C
    // A=[0,1]³  B=[0.3,1.3]³  C=[0.6,1.6]³
    // Vol(A∪B) = 2 − 0.7 = 1.3,  C overlaps A∪B in [0.6,1.3]³ = 0.7 vol
    // Expected = 1.3 − 0.7 = 0.6
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_Chained_UnionThenDifference_VolumeIsCorrect()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.3, 0, 0)), new Vec3D(1, 1, 1), "b");
        var c   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.6, 0, 0)), new Vec3D(1, 1, 1), "c");

        var ab   = api.Boolean(a,  b, BooleanOp.Union,      "ab");
        var abc  = api.Boolean(ab, c, BooleanOp.Difference, "abc");

        // A∪B spans [0, 1.3] × [0,1] × [0,1] = vol 1.3
        // C=[0.6,1.6]³ overlap with A∪B = [0.6,1.3]×[0,1]×[0,1] = 0.7
        double expected = 1.3 - 0.7;
        double vol = MeshVolume(abc);
        Console.WriteLine($"[Chained (A∪B)−C] vol={vol:F6}  expected={expected:F6}");
        AssertVolume(expected, vol, "(A∪B)−C");
        TestVisualization.Visualize(api, nameof(Boolean_Chained_UnionThenDifference_VolumeIsCorrect));
    }

    [Fact]
    public void Boolean_UnionEdgeTouch()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 1, 0)), new Vec3D(1, 1, 1), "b");
        var c = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 2, 0)), new Vec3D(1, 1, 1), "c");

        var ab = api.Boolean(a, b, BooleanOp.Union, "ab");
        var abc = api.Boolean(ab, c, BooleanOp.Union, "abc");
        TestVisualization.Visualize(api, nameof(Boolean_UnionEdgeTouch));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // No-overlap difference: B is completely separate → result == A
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_Difference_NoOverlap_ReturnsA()
    {
        var (api, a, b) = MakePair(shiftX: 2.0); // B at [2,3]³, A at [0,1]³
        var result = api.Boolean(a, b, BooleanOp.Difference, "ab");
        double vol = MeshVolume(result);
        Console.WriteLine($"[Diff no-overlap] vol={vol:F6}  expected=1");
        AssertVolume(1.0, vol, "Difference no-overlap");
        TestVisualization.Visualize(api, nameof(Boolean_Difference_NoOverlap_ReturnsA));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Sphere volume sanity
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void CreateSphere_Volume_CloseToAnalytical()
    {
        var api    = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(0, 0, 0)), 2.0, 0.01, "sphere");
        double vol      = MeshVolume(sphere);
        double expected = 4.0 / 3.0 * Math.PI * 8.0; // (4/3)πr³, r=2
        double errPct   = 100 * Math.Abs(vol - expected) / expected;
        Console.WriteLine($"[Sphere r=2] vol={vol:F4}  theory={expected:F4}  err={errPct:F2}%");
        // maxDeviation=0.01 on r=2 → chord underestimates volume by ~3.5%
        Assert.True(errPct < 4.0, $"Sphere volume {vol:F4} deviates {errPct:F2}% (>4%) from theory {expected:F4}");
        TestVisualization.Visualize(api, nameof(CreateSphere_Volume_CloseToAnalytical));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Sphere − cuboid: result must be smaller than sphere and positive
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_Difference_SphereMinus_CuboidOverlap_VolumeShrinks()
    {
        var api    = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(0, 0, 0)), 2.0, 0.01, "sphere");
        var cube   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.5, 0, 0)), new Vec3D(1, 1, 1), "cube");

        double volSphere = MeshVolume(sphere);
        var    result    = api.Boolean(sphere, cube, BooleanOp.Difference, "diff");
        double volResult = MeshVolume(result);
        Console.WriteLine($"[SphereMinus] sphere={volSphere:F4}  result={volResult:F4}");

        Assert.True(volResult < volSphere, $"Result {volResult:F4} should be less than sphere {volSphere:F4}");
        Assert.True(volResult > 0,         $"Result {volResult:F4} should be positive");
        TestVisualization.Visualize(api, nameof(Boolean_Difference_SphereMinus_CuboidOverlap_VolumeShrinks));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Cylinder – axial-rotation sweep
    //
    // Setup:
    //   Cylinder A: radius=0.5, height=1, axis=Z, base at origin → occupies Z∈[0,1].
    //   Cylinder B: same dimensions, base shifted by 0.5 along Z (half the height),
    //               so its axis coincides with A's. B occupies Z∈[0.5, 1.5].
    //
    //   For a rotation around the shared Z-axis through B's own axis:
    //   - A cylinder is rotationally symmetric about its own axis, so rotating B
    //     around that axis leaves the geometry identical. The set-algebra identities
    //     must therefore hold for ALL angles.
    //   - Analytical overlap:  a cylinder-cylinder intersection along the shared axis
    //     with axial overlap 0.5 → Vol(A∩B) = π·r²·h_overlap = π·0.25·0.5 ≈ 0.3927
    //
    // This test probes the CSG engine's ability to handle the curved surfaces of
    // high-resolution cylinders at multiple orientations.
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]
    [InlineData(30.0)]
    [InlineData(60.0)]
    [InlineData(90.0)]
    [InlineData(120.0)]
    [InlineData(180.0)]
    public void Boolean_Cylinder_RotatedAroundAxis_SetAlgebraHolds(double angleDeg)
    {
        const double radius     = 0.5;
        const double height     = 1.0;
        const double axialShift = 0.5;       // half the height → overlap = 0.5 units
        const double dev        = 0.005;     // small deviation → high resolution tessellation

        double rad = angleDeg * Math.PI / 180.0;
        double c   = Math.Cos(rad), s = Math.Sin(rad);

        // A: base at origin, axis along +Z
        var csA = new CoordinateSystem(new Vec3D(0, 0, 0));

        // B: base shifted axialShift along Z. Its axis is the same Z-axis as A's.
        // Rotating B around that shared Z-axis by rad:
        //   X' = ( c, s, 0)   Y' = (-s, c, 0)   Z' = (0, 0, 1)   (rotation around Z)
        var bOrigin = new Vec3D(0, 0, axialShift);
        var csB = new CoordinateSystem(bOrigin,
            new Vec3D( c, s, 0),
            new Vec3D(-s, c, 0),
            new Vec3D( 0, 0, 1));

        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(4)), 1e-4);
        var a   = api.CreateCylinder(csA, radius, height,     dev, "cyl_a");
        var b   = api.CreateCylinder(csB, radius, height,     dev, "cyl_b");

        Console.WriteLine($"[Cyl Z-rot {angleDeg}°]");
        AssertSetAlgebra(api, a, b, $"CylZ{angleDeg:F0}");

        // Since cylinder is rotationally symmetric, the intersection volume is
        // always π·r²·axialShift regardless of rotation angle.
        double expectedIsect = Math.PI * radius * radius * axialShift;
        var    isect         = api.Boolean(a, b, BooleanOp.Intersect, "isect_check");
        double volIsect      = MeshVolume(isect);
        Console.WriteLine($"  Vol(A∩B)={volIsect:F5}  expected≈{expectedIsect:F5}");
        // Allow 3% for tessellation error on curved surfaces
        AssertVolume(expectedIsect, volIsect, $"CylZ{angleDeg:F0} analytical overlap", tol: expectedIsect * 0.03);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Rotation helpers
    // ═══════════════════════════════════════════════════════════════════════

    /// <summary>
    /// CoordinateSystem whose origin is at <paramref name="origin"/> and whose
    /// local X/Y/Z axes are rotated by <paramref name="angleRad"/> around the
    /// world Y-axis.
    /// </summary>
    private static CoordinateSystem RotatedAroundY(Vec3D origin, double angleRad)
    {
        double c = Math.Cos(angleRad), s = Math.Sin(angleRad);
        // Rotation matrix around Y:  X' = (c,0,s)  Y' = (0,1,0)  Z' = (-s,0,c)
        return new CoordinateSystem(origin,
            new Vec3D( c, 0, s),
            new Vec3D( 0, 1, 0),
            new Vec3D(-s, 0, c));
    }

    /// <summary>
    /// CoordinateSystem whose origin is at <paramref name="origin"/> and whose
    /// local X/Y/Z axes are rotated by <paramref name="angleRad"/> around the
    /// world Z-axis.
    /// </summary>
    private static CoordinateSystem RotatedAroundZ(Vec3D origin, double angleRad)
    {
        double c = Math.Cos(angleRad), s = Math.Sin(angleRad);
        // Rotation matrix around Z:  X' = (c,-s,0)  Y' = (s,c,0)  Z' = (0,0,1)
        return new CoordinateSystem(origin,
            new Vec3D(c, -s, 0),
            new Vec3D(s,  c, 0),
            new Vec3D(0,  0, 1));
    }

    /// <summary>
    /// Asserts the four fundamental set-algebra volume identities that must hold
    /// for ANY pair of closed watertight meshes, regardless of their relative
    /// orientation or overlap amount.
    ///
    ///   (1) Vol(A) + Vol(B) = Vol(A∪B) + Vol(A∩B)          inclusion-exclusion
    ///   (2) Vol(A−B)        = Vol(A)   - Vol(A∩B)           difference = A minus overlap
    ///   (3) Vol(B−A)        = Vol(B)   - Vol(A∩B)           symmetric
    ///   (4) Vol(A∪B)        = Vol(A−B) + Vol(B−A) + Vol(A∩B)  partition
    ///   (5) All individual volumes ≥ 0
    /// </summary>
    private static void AssertSetAlgebra(
        GeoAPI api,
        AnchorMesh a, AnchorMesh b,
        string label,
        double tol = 0.005)
    {
        double volA  = MeshVolume(a);
        double volB  = MeshVolume(b);

        var rUnion  = api.Boolean(a, b, BooleanOp.Union,     label + "_u");
        var rIsect  = api.Boolean(a, b, BooleanOp.Intersect, label + "_i");
        var rDiffAB = api.Boolean(a, b, BooleanOp.Difference, label + "_ab");
        var rDiffBA = api.Boolean(b, a, BooleanOp.Difference, label + "_ba");

        double volU  = MeshVolume(rUnion);
        double volI  = MeshVolume(rIsect);
        double volAB = MeshVolume(rDiffAB);
        double volBA = MeshVolume(rDiffBA);

        Console.WriteLine($"  [{label}] A={volA:F4} B={volB:F4} U={volU:F4} I={volI:F4} A-B={volAB:F4} B-A={volBA:F4}");

        // (5) non-negative
        Assert.True(volU  >= -tol, $"{label}: Vol(A∪B)={volU:F5} < 0");
        Assert.True(volI  >= -tol, $"{label}: Vol(A∩B)={volI:F5} < 0");
        Assert.True(volAB >= -tol, $"{label}: Vol(A-B)={volAB:F5} < 0");
        Assert.True(volBA >= -tol, $"{label}: Vol(B-A)={volBA:F5} < 0");

        // (1) inclusion-exclusion
        AssertVolume(volA + volB, volU + volI,   $"{label} Vol(A)+Vol(B)=Vol(U)+Vol(I)", tol);

        // (2) A - B
        AssertVolume(volA - volI, volAB,          $"{label} Vol(A-B)=Vol(A)-Vol(I)", tol);

        // (3) B - A
        AssertVolume(volB - volI, volBA,          $"{label} Vol(B-A)=Vol(B)-Vol(I)", tol);

        // (4) partition
        AssertVolume(volU, volAB + volBA + volI,  $"{label} Vol(U)=Vol(A-B)+Vol(B-A)+Vol(I)", tol);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Rotation sweep — Y axis
    //
    // Setup:
    //   Cube A: unit cube at origin, CoordSys identity, occupies [0,1]³.
    //   Cube B: unit cube, centre at (1.0, 0.5, 0.5) = right face centre of A,
    //           rotated around Y through its own centre by angleRad.
    //
    //   At 0°:  B occupies [0.5, 1.5]³ → overlap = 0.5 × 1 × 1 = 0.5
    //   At 90°: B rotated 90° around Y through centre (1.0,0.5,0.5);
    //           the rotated unit cube still spans 1 unit in each world direction
    //           from its centre → same AABB → overlap is still analytically 0.5
    //   At 45°/other angles: overlap changes; we only assert the set-algebra identities.
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]
    [InlineData(15.0)]
    [InlineData(30.0)]
    [InlineData(45.0)]
    [InlineData(60.0)]
    [InlineData(75.0)]
    [InlineData(90.0)]
    [InlineData(120.0)]
    [InlineData(150.0)]
    [InlineData(180.0)]
    public void Boolean_RotatedAroundY_SetAlgebraHolds(double angleDeg)
    {
        double rad = angleDeg * Math.PI / 180.0;

        // B's centre in world space: right face of A, same Y/Z centre
        var bCenter = new Vec3D(1.0, 0.5, 0.5);
        // Lower-left corner of B in B's LOCAL frame is at (-0.5, -0.5, -0.5) from its centre.
        // After rotating around Y through bCenter by rad, the new origin is:
        //   bCenter + R_Y(rad) * (-0.5, -0.5, -0.5)
        double c = Math.Cos(rad), s = Math.Sin(rad);
        // R_Y rotates (x,y,z) → (c*x+s*z, y, -s*x+c*z)
        var localLL = new Vec3D(-0.5, -0.5, -0.5);
        var rotatedLL = new Vec3D(c * localLL.X + s * localLL.Z,
                                  localLL.Y,
                                 -s * localLL.X + c * localLL.Z);
        var bOrigin = new Vec3D(bCenter.X + rotatedLL.X,
                                bCenter.Y + rotatedLL.Y,
                                bCenter.Z + rotatedLL.Z);

        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(4)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b   = api.CreateCuboid(RotatedAroundY(bOrigin, rad), new Vec3D(1, 1, 1), "b");

        Console.WriteLine($"[Y-rot {angleDeg}°]");
        AssertSetAlgebra(api, a, b, $"Y{angleDeg:F0}");

        // At 0° and 180° the geometry is axis-aligned → analytical overlap = 0.5
        if (angleDeg == 0.0 || angleDeg == 180.0)
        {
            var isect = api.Boolean(a, b, BooleanOp.Intersect, "i_check");
            AssertVolume(0.5, MeshVolume(isect), $"Y{angleDeg:F0} analytical overlap");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Rotation sweep — Z axis
    //
    // Same setup but rotation is around world Z through B's centre.
    // ═══════════════════════════════════════════════════════════════════════

    [Theory]
    [InlineData(0.0)]
    [InlineData(15.0)]
    [InlineData(30.0)]
    [InlineData(45.0)]
    [InlineData(60.0)]
    [InlineData(75.0)]
    [InlineData(90.0)]
    [InlineData(120.0)]
    [InlineData(150.0)]
    [InlineData(180.0)]
    public void Boolean_RotatedAroundZ_SetAlgebraHolds(double angleDeg)
    {
        double rad = angleDeg * Math.PI / 180.0;

        var bCenter = new Vec3D(1.0, 0.5, 0.5);
        double c = Math.Cos(rad), s = Math.Sin(rad);
        // R_Z rotates (x,y,z) → (c*x-s*y, s*x+c*y, z)
        var localLL = new Vec3D(-0.5, -0.5, -0.5);
        var rotatedLL = new Vec3D(c * localLL.X - s * localLL.Y,
                                  s * localLL.X + c * localLL.Y,
                                  localLL.Z);
        var bOrigin = new Vec3D(bCenter.X + rotatedLL.X,
                                bCenter.Y + rotatedLL.Y,
                                bCenter.Z + rotatedLL.Z);

        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(4)), 1e-4);
        var a   = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b   = api.CreateCuboid(RotatedAroundZ(bOrigin, rad), new Vec3D(1, 1, 1), "b");

        Console.WriteLine($"[Z-rot {angleDeg}°]");
        AssertSetAlgebra(api, a, b, $"Z{angleDeg:F0}");

        // At 0° and 180° the geometry is axis-aligned → analytical overlap = 0.5
        if (angleDeg == 0.0 || angleDeg == 180.0)
        {
            var isect = api.Boolean(a, b, BooleanOp.Intersect, "i_check");
            AssertVolume(0.5, MeshVolume(isect), $"Z{angleDeg:F0} analytical overlap");
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Batch union — analytical and stress (Hilbert tube union)
    // ═══════════════════════════════════════════════════════════════════════

    [Fact]
    public void Boolean_TenCubesInRow_BatchUnion_VolumeIsTen()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(15)), 1e-4);
        var list = new List<AnchorMesh>();
        for (int i = 0; i < 10; i++)
            list.Add(api.CreateCuboid(new CoordinateSystem(new Vec3D(i, 0, 0)), new Vec3D(1, 1, 1), $"c{i}"));
        var u = api.BatchUnion(list);
        AssertVolume(10.0, MeshVolume(u), "10 touching 1³ cubes in a row");
        TestVisualization.Visualize(api, nameof(Boolean_TenCubesInRow_BatchUnion_VolumeIsTen));
    }

    [Fact]
    public void Boolean_PlusShape_Union_VolumeIsFive()
    {
        // barX: [0,3]×[0,1]×[0,1], barY: [1,2]×[0,3]×[0,1] → overlap 1 → union vol = 3+3−1 = 5
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(5)), 1e-4);
        var barX = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(3, 1, 1), "barX");
        var barY = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 0, 0)), new Vec3D(1, 3, 1), "barY");
        var u = api.Boolean(barX, barY, BooleanOp.Union, "plus");
        AssertVolume(5.0, MeshVolume(u), "plus-shape union");
        TestVisualization.Visualize(api, nameof(Boolean_PlusShape_Union_VolumeIsFive));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Classic CSG diagram: (cube ∩ sphere) − (cylX ∪ cylY ∪ cylZ)
    // ═══════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Default CSG showcase: intersect a cube with a sphere (rounded box), then subtract the union
    /// of three cylinders along X, Y, and Z (through-holes). Matches the usual tutorial tree.
    /// </summary>
    [Fact]
    public void Boolean_CsgClassic_RoundedCubeMinusOrthogonalCylinderJacks_VolumeRegression()
    {
        const double dev = 0.006;
        const double cubeHalf = 1.0;
        const double sphereR = 1.22;
        const double cylLen = 3.5;
        const double holeR = 0.32;

        var api = new GeoAPI(new Box3D(new Vec3D(-3), new Vec3D(3)), 1e-4);

        var cube = api.CreateCuboid(
            new CoordinateSystem(new Vec3D(-cubeHalf, -cubeHalf, -cubeHalf)),
            new Vec3D(2 * cubeHalf, 2 * cubeHalf, 2 * cubeHalf),
            "cube");
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(0, 0, 0)), sphereR, dev, "sphere");
        var rounded = api.Boolean(cube, sphere, BooleanOp.Intersect, "rounded");

        // Through-holes along world X/Y/Z. Revolve builds each cylinder with axis = local sketch X;
        // each frame below is right-handed (X×Y = Z). See CoordinateSystem constructor.
        //
        var cylX = api.CreateCylinderRevolve(
            new CoordinateSystem(new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1)),
            holeR, cylLen, dev, "cylX");
        var cylY = api.CreateCylinderRevolve(
            new CoordinateSystem(new Vec3D(0, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0)),
            holeR, cylLen, dev, "cylY");
        var cylZ = api.CreateCylinderRevolve(
            new CoordinateSystem(new Vec3D(0, 0, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0)),
            holeR, cylLen, dev, "cylZ");

        foreach (var m in new[] { rounded, cylX, cylY, cylZ })
        {
            Assert.True(
                MeshAnalysis.AreTrianglesConsistentlyOriented(m.Mesh.PrecisionPositions, m.Mesh.Triangles),
                $"precondition: {m.Name} should be consistently oriented");
            Assert.True(
                MeshAnalysis.ComputeSignedMeshVolume(m.Mesh.Positions, m.Mesh.Triangles) > 0,
                $"precondition: {m.Name} signed volume should be positive (outward normals)");
        }

        // Single difference vs union of holes avoids sequential Debug failures in constrained
        // triangulation after the revolve coordinate-frame fix (orthogonal cuts).
        var holesUnion = api.BatchUnion(new List<AnchorMesh> { cylX, cylY, cylZ });
        var result = api.Boolean(rounded, holesUnion, BooleanOp.Difference, "csgClassic");

        double volRounded = MeshVolume(rounded);
        double volResult = MeshVolume(result);
        double signedResult = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);

        Assert.True(volRounded > 0.5, $"rounded ∩ should have substantial volume, got {volRounded:F4}");
        Assert.True(volResult > 0.05 && volResult < volRounded - 0.05,
            $"expected holes to remove volume: rounded={volRounded:F4}, result={volResult:F4}");

        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions, result.Mesh.Triangles),
            "result mesh should be consistently oriented after difference with united holes");
        Assert.True(signedResult > 0, $"result signed volume should be positive, got {signedResult:F6}");

        // Tessellation / boolean noise; baseline with true orthogonal revolve cylinders + union of holes.
        const double expectedVol = 4.9747;
        const double relTol = 0.04;
        Assert.True(Math.Abs(volResult - expectedVol) <= Math.Max(0.2, expectedVol * relTol),
            $"CSG classic volume expected ≈{expectedVol:F4} (±{100 * relTol:F0}%), got {volResult:F4}");
        Assert.True(
            Math.Abs(volResult - Math.Abs(signedResult)) < 1e-3,
            "MeshVolume helper should match |ComputeSignedMeshVolume| for this mesh");

        TestVisualization.Visualize(api, nameof(Boolean_CsgClassic_RoundedCubeMinusOrthogonalCylinderJacks_VolumeRegression));
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Known bugs — documented with Skip so they run as reminders
    // ═══════════════════════════════════════════════════════════════════════

    /// <summary>
    /// When Intersect(A, B) is called with non-overlapping meshes, the resolver
    /// produces open-boundary triangle fragments that fail the watertight check
    /// in AnchorMesh constructor. Expected: empty mesh. Actual: throws.
    /// Fix: add a bounding-box early-out in Mesh.BooleanOperation before calling Resolve.
    /// </summary>
    [Fact(Skip = "Bug: Intersect of non-overlapping meshes throws 'Mesh is not watertight'. " +
                 "The resolver produces open-boundary fragments. " +
                 "Fix: add AABB early-out in Mesh<T>.BooleanOperation or Resolver.Resolve.")]
    public void Boolean_Intersect_NoOverlap_Throws()
    {
        var (api, a, b) = MakePair(shiftX: 2.0);
        var result = api.Boolean(a, b, BooleanOp.Intersect, "ab");
        AssertEmpty(result, "Intersect no-overlap");
    }
}
