using Geo;
using GeoCore;

namespace GeoTests;

public class ExtruderTests
{
    [Fact]
    public void SimpleSquareExtrusion_IsWatertightAndConsistent()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var location = CoordinateSystem.Default;
        var contour = MeshTestHelpers.MakeSquareContourCCW(1.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        int numGroups = Extruder.GenerateExtrudedMesh(
            converter, location, contour,
            angleThresholdRadian: 0.5,
            height: 2.0,
            triangles, vertices, normals, uv, groups, precise, baseGroupIndex: 0);

        Assert.True(numGroups >= 3, "Should have at least side + top + bottom groups");
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-0.5, -0.5, 0), new Vec3D(0.5, 0.5, 2.0), 1e-6);
    }

    [Fact]
    public void SquareWithHole_IsWatertightAndConsistent()
    {
        var converter = MeshTestHelpers.MakeConverter();

        var outer = MeshTestHelpers.MakeSquareContourCCW(2.0);

        // Inner hole (CW when viewed from +Z) -- smaller square
        double h = 0.3;
        var inner = new List<Vec2D>
        {
            new Vec2D(-h, -h),
            new Vec2D(-h,  h),
            new Vec2D( h,  h),
            new Vec2D( h, -h),
            new Vec2D(-h, -h)
        };

        var contours = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { outer },
            new List<List<Vec2D>> { inner }
        };
        var contourNormals = new List<List<List<Vec2D>>>();
        foreach (var loop in contours)
        {
            var loopNormals = new List<List<Vec2D>>();
            foreach (var seg in loop)
                loopNormals.Add(Extruder.WeightedNormals2D(seg));
            contourNormals.Add(loopNormals);
        }

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        var names = new List<List<string>>
        {
            new List<string> { "OuterEdge" },
            new List<string> { "InnerEdge" }
        };

        int numGroups = Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, contours, contourNormals,
            height: 1.0,
            triangles, vertices, normals, uv, groups, precise,
            names, "TestExtrude", out var groupToName, baseGroupIndex: 0);

        Assert.True(numGroups >= 4);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        Assert.Contains("TestExtrude-ExtrudeBottom", groupToName.Values);
        Assert.Contains("TestExtrude-ExtrudeTop", groupToName.Values);
    }

    [Fact]
    public void BidirectionalExtrusion_SpansBothDirections()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(1.0);

        var contours = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { contour }
        };
        var contourNormals = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { Extruder.WeightedNormals2D(contour) }
        };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        var names = new List<List<string>>
        {
            new List<string> { "Edge" }
        };

        int numGroups = Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, contours, contourNormals,
            heightPositive: 3.0, heightNegative: 2.0,
            triangles, vertices, normals, uv, groups, precise,
            names, "BiDir", out var groupToName, baseGroupIndex: 0);

        Assert.True(numGroups >= 3);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-0.5, -0.5, -2.0), new Vec3D(0.5, 0.5, 3.0), 1e-6);
    }

    [Fact]
    public void DegenerateContour_ReturnsZeroGroups()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var tooFew = new List<Vec2D> { new Vec2D(0, 0), new Vec2D(1, 0) };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        int numGroups = Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, tooFew,
            angleThresholdRadian: 0.5, height: 1.0,
            triangles, vertices, normals, uv, groups, precise, baseGroupIndex: 0);

        Assert.Equal(0, numGroups);
        Assert.Empty(triangles);
    }

    [Fact]
    public void WeightedNormals2D_ProducesUnitLengthOutwardNormals()
    {
        var square = MeshTestHelpers.MakeSquareContourCCW(2.0);
        var normals = Extruder.WeightedNormals2D(square);

        Assert.Equal(square.Count, normals.Count);
        foreach (var n in normals)
        {
            double len = Math.Sqrt(n.X * n.X + n.Y * n.Y);
            Assert.InRange(len, 0.99, 1.01);
        }

        // Adjacent normals should differ at sharp corners (45-degree blending at a 90-degree corner)
        var n0 = normals[0];
        var n1 = normals[1];
        double dot = n0.X * n1.X + n0.Y * n1.Y;
        Assert.InRange(dot, -0.1, 0.9);
    }

    [Fact]
    public void WeightedNormals2D_TooFewPoints_ReturnsEmpty()
    {
        var result = Extruder.WeightedNormals2D(new List<Vec2D> { new Vec2D(0, 0) });
        Assert.Empty(result);

        var result2 = Extruder.WeightedNormals2D(null!);
        Assert.Empty(result2);
    }

    [Fact]
    public void TriangleExtrusion_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var triangle = new List<Vec2D>
        {
            new Vec2D(0, 0),
            new Vec2D(1, 0),
            new Vec2D(0.5, 1),
            new Vec2D(0, 0)
        };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        int numGroups = Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, triangle,
            angleThresholdRadian: 0.1, height: 1.0,
            triangles, vertices, normals, uv, groups, precise, baseGroupIndex: 0);

        Assert.True(numGroups >= 3);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    // --- Curve extrusion tests ---

    private static List<List<List<Vec2D>>> WrapContour(List<Vec2D> contour) =>
        new List<List<List<Vec2D>>> { new List<List<Vec2D>> { contour } };

    private static List<List<List<Vec2D>>> WrapContourNormals(List<Vec2D> contour) =>
        new List<List<List<Vec2D>>> { new List<List<Vec2D>> { Extruder.WeightedNormals2D(contour) } };

    private static List<CoordinateSystem> MakeStraightGuide(Vec3D start, Vec3D end, int steps)
    {
        var guide = new List<CoordinateSystem>();
        Vec3D dir = (end - start).Normalized();
        Vec3D up = Math.Abs(dir.Z) < 0.9 ? new Vec3D(0, 0, 1) : new Vec3D(0, 1, 0);
        Vec3D right = Vec3DOps.Cross(dir, up).Normalized();
        up = Vec3DOps.Cross(right, dir).Normalized();
        // right = dir×up ⇒ right×up = −dir (left-handed if X=right, Y=up, Z=dir). Use X=up, Y=right so X×Y=dir.
        for (int i = 0; i <= steps; i++)
        {
            double t = (double)i / steps;
            Vec3D origin = start + (end - start) * t;
            guide.Add(new CoordinateSystem(origin, up, right, dir));
        }
        return guide;
    }

    private static List<CoordinateSystem> MakeArcGuide(double radius, double angleRad, int steps)
    {
        var guide = new List<CoordinateSystem>();
        for (int i = 0; i <= steps; i++)
        {
            double t = angleRad * i / steps;
            double cosT = Math.Cos(t);
            double sinT = Math.Sin(t);
            Vec3D origin = new Vec3D(radius * sinT, 0, radius * (1 - cosT));
            Vec3D tangent = new Vec3D(cosT, 0, sinT);
            Vec3D normal = new Vec3D(-sinT, 0, cosT);
            Vec3D binormal = new Vec3D(0, 1, 0);
            guide.Add(new CoordinateSystem(origin, binormal, normal, tangent));
        }
        return guide;
    }

    [Fact]
    public void ExtrudeAlongStraightLine_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.5);
        var guide = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 3), 4);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "StraightExtrude", out var groupToName);

        Assert.True(numGroups >= 3);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-0.25, -0.25, 0), new Vec3D(0.25, 0.25, 3), 0.05);
    }

    [Fact]
    public void ExtrudeAlongArc_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.3);
        var guide = MakeArcGuide(radius: 2.0, angleRad: Math.PI / 2, steps: 8);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "ArcExtrude", out var groupToName);

        Assert.True(numGroups >= 3);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void ExtrudeAlongClosedLoop_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.2);
        // Full circle guide (closed loop: last point == first point)
        var guide = MakeArcGuide(radius: 2.0, angleRad: 2 * Math.PI, steps: 16);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "LoopExtrude", out var groupToName);

        Assert.True(numGroups >= 1);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void ExtrudeAlongCurve_NamedGroups()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.3);
        var guide = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 2), 3);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "MyEdge" } };

        Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "Pipe", out var groupToName);

        Assert.Contains("Pipe-ExtrudeBottom", groupToName.Values);
        Assert.Contains("Pipe-ExtrudeTop", groupToName.Values);
        Assert.Contains("Pipe-MyEdge", groupToName.Values);
    }

    [Fact]
    public void ExtrudeAlongCurve_MultiSegmentGuide_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.3);
        // Two connected segments
        var seg1 = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 2), 3);
        var seg2 = MakeStraightGuide(new Vec3D(0, 0, 2), new Vec3D(2, 0, 2), 3);
        var multiGuide = new List<List<CoordinateSystem>> { seg1, seg2 };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            multiGuide, multiGuide[0][0], multiGuide[0][0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "LShapeExtrude", out var groupToName);

        Assert.True(numGroups >= 3);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void ExtrudeAlongCurve_DegenerateGuide_ReturnsZero()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.3);
        // Single-point guide (too few points)
        var guide = new List<CoordinateSystem> { CoordinateSystem.Default };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            guide, CoordinateSystem.Default, new Vec3D(0, 0, 1), triangles, vertices, normals, uv, groups, precise,
            names, "Degen", out var groupToName);

        Assert.Equal(0, numGroups);
        Assert.Empty(triangles);
    }

    // --- Curve strip (multi-segment, sectioned) tests ---

    [Fact]
    public void ExtrudeAlongCurveStripSectioned_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.3);
        var seg1 = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 2), 3);
        var seg2 = MakeStraightGuide(new Vec3D(0, 0, 2), new Vec3D(2, 0, 2), 3);
        var multiGuide = new List<List<CoordinateSystem>> { seg1, seg2 };
        var guideNames = new List<string> { "Seg1", "Seg2" };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurveStripSectioned(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            multiGuide, guideNames, multiGuide[0][0], multiGuide[0][0].Z,
            triangles, vertices, normals, uv, groups, precise,
            names, "StripExtrude", out var groupToName);

        Assert.True(numGroups >= 4);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        Assert.Contains("StripExtrude-Edge-Seg1", groupToName.Values);
        Assert.Contains("StripExtrude-Edge-Seg2", groupToName.Values);
        Assert.Contains("StripExtrude-ExtrudeBottom", groupToName.Values);
        Assert.Contains("StripExtrude-ExtrudeTop", groupToName.Values);
    }

    [Fact]
    public void ExtrudeAlongCurveStripSectioned_ArcGuide_IsWatertight()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.2);
        var seg1 = MakeArcGuide(radius: 2.0, angleRad: Math.PI / 4, steps: 4);
        // Second segment starting where first ends
        var lastCS = seg1[seg1.Count - 1];
        var seg2 = MakeStraightGuide(lastCS.Origin, lastCS.Origin + new Vec3D(0, 0, 2), 4);
        var multiGuide = new List<List<CoordinateSystem>> { seg1, seg2 };
        var guideNames = new List<string> { "Arc", "Straight" };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int numGroups = Extruder.GenerateExtrudeAlongCurveStripSectioned(
            converter, CoordinateSystem.Default,
            WrapContour(contour), WrapContourNormals(contour),
            multiGuide, guideNames, multiGuide[0][0], multiGuide[0][0].Z,
            triangles, vertices, normals, uv, groups, precise,
            names, "ArcStrip", out var groupToName);

        Assert.True(numGroups >= 4);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    /// <summary>
    /// Twist + finite maxDeviation refines the tessellated guide polyline (extra cross-sections), same sagitta rule as linear twist.
    /// </summary>
    [Fact]
    public void CurveStrip_Twist_TightMaxDeviation_InsertsMoreCrossSectionsThanInfinity()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(0.5);
        // Coarse guide: single segment along +Z (two samples only)
        var guide = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 4), 1);
        var multiGuide = new List<List<CoordinateSystem>> { guide };
        var guideNames = new List<string> { "Line" };
        double twistRate = Math.PI / 4.0;
        const double maxDev = 0.02;
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int VertexCount(double maxDeviation)
        {
            var triangles = new List<Tri>();
            var vertices = new List<Vec3D>();
            var normals = new List<Vec3D>();
            var uv = new List<Vec2D>();
            var groups = new List<int>();
            var precise = new List<Rat3Hybrid>();
            Extruder.GenerateExtrudeAlongCurveStripSectioned(
                converter, CoordinateSystem.Default,
                WrapContour(contour), WrapContourNormals(contour),
                multiGuide, guideNames, multiGuide[0][0], multiGuide[0][0].Z,
                triangles, vertices, normals, uv, groups, precise,
                names, "RefineTwist", out _,
                twistRatePerExtrudeDistance: twistRate, maxDeviation: maxDeviation);
            MeshTestHelpers.AssertValidMesh(vertices, triangles);
            return vertices.Count;
        }

        int nInf = VertexCount(double.PositiveInfinity);
        int nFine = VertexCount(maxDev);
        Assert.True(nFine > nInf, $"expected refinement to add vertices: inf={nInf} fine={nFine}");
    }

    [Fact]
    public void LinearExtrude_WithTwistQuarterTurnOverHeight_TopCornerAndVolume()
    {
        const double H = 2.0;
        double twistRate = Math.PI / (2.0 * H);
        var converter = MeshTestHelpers.MakeConverter();
        // One segment per side, matching sketch tessellation: each wall keeps its own normal.
        var square = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var edges = new List<List<Vec2D>>();
        var edgeNormals = new List<List<Vec2D>>();
        var edgeNames = new List<string>();
        for (int i = 0; i < square.Count - 1; i++)
        {
            var edge = new List<Vec2D> { square[i], square[i + 1] };
            edges.Add(edge);
            edgeNormals.Add(Extruder.WeightedNormals2D(edge));
            edgeNames.Add("E" + i);
        }
        var contours = new List<List<List<Vec2D>>> { edges };
        var contourNormals = new List<List<List<Vec2D>>> { edgeNormals };
        var names = new List<List<string>> { edgeNames };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, contours, contourNormals,
            height: H, triangles, vertices, normals, uv, groups, precise,
            names, "TwistLinear", out _, twistRatePerExtrudeDistance: twistRate);

        MeshTestHelpers.AssertValidMesh(vertices, triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(vertices, triangles);
        Assert.InRange(vol, 1.99, 2.01);

        // Bottom corner (-0.5,-0.5) rotated by π/2 about origin → (0.5,-0.5) at z = H
        bool foundTopTwistedCorner = false;
        foreach (var v in vertices)
        {
            if (Math.Abs(v.Z - H) < 1e-5 && Math.Abs(v.X - 0.5) < 1e-5 && Math.Abs(v.Y + 0.5) < 1e-5)
            {
                foundTopTwistedCorner = true;
                break;
            }
        }
        Assert.True(foundTopTwistedCorner);

        // Corner (0.5, -0.5) is shared by the bottom edge (outward -Y, z = twistRate * x)
        // and the right edge (outward +X, z = twistRate * y). Each wall keeps its own normal.
        double k = twistRate * 0.5;
        double scale = 1.0 / Math.Sqrt(1.0 + k * k);
        bool foundBottom = false, foundRight = false;
        for (int i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            if (Math.Abs(v.Z) > 1e-5 || Math.Abs(v.X - 0.5) > 1e-5 || Math.Abs(v.Y + 0.5) > 1e-5)
                continue;
            var n = normals[i];
            if (Math.Abs(n.X) < 1e-5 && Math.Abs(n.Y + scale) < 1e-5 && Math.Abs(n.Z - k * scale) < 1e-5)
                foundBottom = true;
            if (Math.Abs(n.X - scale) < 1e-5 && Math.Abs(n.Y) < 1e-5 && Math.Abs(n.Z + k * scale) < 1e-5)
                foundRight = true;
        }
        Assert.True(foundBottom);
        Assert.True(foundRight);
    }

    [Fact]
    public void LinearExtrude_TwistAboutSketchCenter_OrbitsThatPoint()
    {
        const double H = 2.0;
        double twistRate = Math.PI / (2.0 * H);
        var center = new Vec2D(1.0, 0.0);
        var converter = MeshTestHelpers.MakeConverter();
        var square = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var edges = new List<List<Vec2D>>();
        var edgeNormals = new List<List<Vec2D>>();
        var edgeNames = new List<string>();
        for (int i = 0; i < square.Count - 1; i++)
        {
            var edge = new List<Vec2D> { square[i], square[i + 1] };
            edges.Add(edge);
            edgeNormals.Add(Extruder.WeightedNormals2D(edge));
            edgeNames.Add("E" + i);
        }
        var contours = new List<List<List<Vec2D>>> { edges };
        var contourNormals = new List<List<List<Vec2D>>> { edgeNormals };
        var names = new List<List<string>> { edgeNames };
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, contours, contourNormals,
            height: H, triangles, vertices, normals, uv, groups, precise,
            names, "TwistCenter", out _, twistRatePerExtrudeDistance: twistRate,
            twistCenter: center);

        // (0.5, -0.5) relative to (1, 0) is (-0.5, -0.5); a +90° turn lands at (1.5, -0.5).
        bool found = false;
        foreach (var v in vertices)
        {
            if (Math.Abs(v.Z - H) < 1e-5 && Math.Abs(v.X - 1.5) < 1e-5 && Math.Abs(v.Y + 0.5) < 1e-5)
            {
                found = true;
                break;
            }
        }
        Assert.True(found);

        // Bottom edge at (0.5, -0.5) relative to center (1, 0) has edge · (p − c) = -0.5.
        double k = twistRate * 0.5;
        double scale = 1.0 / Math.Sqrt(1.0 + k * k);
        bool foundTilt = false;
        for (int i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            if (Math.Abs(v.Z) > 1e-5 || Math.Abs(v.X - 0.5) > 1e-5 || Math.Abs(v.Y + 0.5) > 1e-5)
                continue;
            var n = normals[i];
            if (Math.Abs(n.X) < 1e-5 && Math.Abs(n.Y + scale) < 1e-5 && Math.Abs(n.Z + k * scale) < 1e-5)
                foundTilt = true;
        }
        Assert.True(foundTilt);
    }

    [Fact]
    public void LinearExtrude_Twist_TightMaxDeviation_SubdividesSideWalls()
    {
        const double H = 2.0;
        double twistRate = Math.PI / (2.0 * H);
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var contours = new List<List<List<Vec2D>>> { new List<List<Vec2D>> { contour } };
        var contourNormals = new List<List<List<Vec2D>>> { new List<List<Vec2D>> { Extruder.WeightedNormals2D(contour) } };
        var names = new List<List<string>> { new List<string> { "Edge" } };

        int Build(double maxDev)
        {
            var triangles = new List<Tri>();
            var vertices = new List<Vec3D>();
            var normals = new List<Vec3D>();
            var uv = new List<Vec2D>();
            var groups = new List<int>();
            var precise = new List<Rat3Hybrid>();
            Extruder.GenerateExtrudedMesh(converter, CoordinateSystem.Default, contours, contourNormals,
                height: H, triangles, vertices, normals, uv, groups, precise,
                names, "Subdiv", out _, twistRatePerExtrudeDistance: twistRate, maxDeviation: maxDev);
            MeshTestHelpers.AssertValidMesh(vertices, triangles);
            return vertices.Count;
        }

        int nCoarse = Build(double.PositiveInfinity);
        int nFine = Build(0.002);
        Assert.True(nFine > nCoarse);
    }

    [Fact]
    public void ExtrudeAlongCurve_WithTwist_VolumeMatchesUntwistedPrism()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var contour = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var guide = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, 3.0), 8);
        var names = new List<List<string>> { new List<string> { "Edge" } };
        const double L = 3.0;
        double twistRate = Math.PI / (4.0 * L);

        double RunVolume(bool twist)
        {
            var triangles = new List<Tri>();
            var vertices = new List<Vec3D>();
            var normals = new List<Vec3D>();
            var uv = new List<Vec2D>();
            var groups = new List<int>();
            var precise = new List<Rat3Hybrid>();
            Extruder.GenerateExtrudeAlongCurve(
                converter, CoordinateSystem.Default,
                WrapContour(contour), WrapContourNormals(contour),
                guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
                names, "TwistCmp", out _,
                twistRatePerExtrudeDistance: twist ? twistRate : 0);
            MeshTestHelpers.AssertValidMesh(vertices, triangles);
            return MeshAnalysis.ComputeSignedMeshVolume(vertices, triangles);
        }

        double v0 = RunVolume(false);
        double v1 = RunVolume(true);
        Assert.InRange(v0, L * 0.95, L * 1.08);
        Assert.InRange(v1, L * 0.95, L * 1.08);
        Assert.InRange(Math.Abs(v1 - v0), 0, 0.15);
    }

    [Fact]
    public void ExtrudeAlongStraightGuide_TwistTiltsSideNormalsAlongTheTangent()
    {
        const double L = 3.0;
        double twistRate = Math.PI / (2.0 * L);
        var converter = MeshTestHelpers.MakeConverter();
        var guide = MakeStraightGuide(new Vec3D(0, 0, 0), new Vec3D(0, 0, L), 2);
        var (contours, contourNormals, names) = SquareEdges();
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default, contours, contourNormals,
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "TwistGuide", out _, twistRatePerExtrudeDistance: twistRate);

        // Guide frame is X=(0,1,0), Y=(-1,0,0), Z=(0,0,1). Profile corner (0.5,-0.5)
        // lands at (0.5, 0.5, 0). Bottom edge normal is normalize(1, 0, twistRate * x);
        // right edge normal is normalize(0, 1, twistRate * y).
        double k = twistRate * 0.5;
        double scale = 1.0 / Math.Sqrt(1.0 + k * k);
        bool foundBottom = false, foundRight = false;
        for (int i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            if (Math.Abs(v.X - 0.5) > 1e-5 || Math.Abs(v.Y - 0.5) > 1e-5 || Math.Abs(v.Z) > 1e-5)
                continue;
            var n = normals[i];
            if (Math.Abs(n.X - scale) < 1e-5 && Math.Abs(n.Y) < 1e-5 && Math.Abs(n.Z - k * scale) < 1e-5)
                foundBottom = true;
            if (Math.Abs(n.X) < 1e-5 && Math.Abs(n.Y - scale) < 1e-5 && Math.Abs(n.Z + k * scale) < 1e-5)
                foundRight = true;
        }
        Assert.True(foundBottom);
        Assert.True(foundRight);
    }

    [Fact]
    public void ExtrudeAlongArc_TwistScalesNormalByCurvature()
    {
        const double radius = 5.0;
        const double twistRate = 0.25;
        var converter = MeshTestHelpers.MakeConverter();
        var guide = MakeArcGuide(radius, Math.PI, 64);
        var (contours, contourNormals, names) = SquareEdges();
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default, contours, contourNormals,
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "ArcTwist", out _, twistRatePerExtrudeDistance: twistRate);

        // Start station, twist angle 0. Frame Y points at the center, so profile point
        // (0.5, -0.5) is at (0, 0.5, -0.5) and σ = 1 − py/radius.
        // Bottom-edge normal is σ(-Y) + twistRate * 0.5 * T.
        double sigma = 1.0 - (-0.5) / radius;
        var raw = new Vec3D(twistRate * 0.5, 0, -sigma);
        var expected = raw / raw.Length();
        bool found = false;
        for (int i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            if (Math.Abs(v.X) > 1e-4 || Math.Abs(v.Y - 0.5) > 1e-4 || Math.Abs(v.Z + 0.5) > 1e-4)
                continue;
            var n = normals[i];
            if (Math.Abs(n.X - expected.X) < 2e-3 && Math.Abs(n.Y) < 2e-3 && Math.Abs(n.Z - expected.Z) < 2e-3)
                found = true;
        }
        Assert.True(found);
    }

    [Fact]
    public void LinearExtrude_Twist_TopStationRotatesTheInPlaneNormalAndKeepsTheAxialPart()
    {
        const double H = 2.0;
        double twistRate = Math.PI / (2.0 * H);
        var mesh = ExtrudeSquareEdges(twistRate, H, default);
        // θ = π/2 sends (0.5, -0.5) to (0.5, 0.5). The bottom-edge normal (0, -1)
        // rotates to +X; its axial part stays twistRate * x and is not rotated away.
        double k = twistRate * 0.5;
        double scale = 1.0 / Math.Sqrt(1.0 + k * k);
        bool foundBottom = false, foundRight = false, foundCap = false;
        for (int i = 0; i < mesh.vertices.Count; i++)
        {
            var v = mesh.vertices[i];
            var n = mesh.normals[i];
            if (Math.Abs(v.Z - H) < 1e-5 && Math.Abs(v.X - 0.5) < 1e-5 && Math.Abs(v.Y - 0.5) < 1e-5)
            {
                if (Math.Abs(n.X - scale) < 1e-5 && Math.Abs(n.Y) < 1e-5 && Math.Abs(n.Z - k * scale) < 1e-5)
                    foundBottom = true;
                if (Math.Abs(n.X) < 1e-5 && Math.Abs(n.Y - scale) < 1e-5 && Math.Abs(n.Z + k * scale) < 1e-5)
                    foundRight = true;
            }
            if (Math.Abs(v.Z) < 1e-5 && Math.Abs(v.X - 0.5) < 1e-5 && Math.Abs(v.Y + 0.5) < 1e-5 &&
                Math.Abs(n.X) < 1e-6 && Math.Abs(n.Y) < 1e-6 && Math.Abs(n.Z + 1) < 1e-6)
                foundCap = true;
        }
        Assert.True(foundBottom);
        Assert.True(foundRight);
        Assert.True(foundCap);
    }

    [Fact]
    public void LinearExtrude_WithoutTwist_SideNormalsStayPerpendicularToTheAxis()
    {
        var mesh = ExtrudeSquareEdges(0, 2.0, default);
        bool foundBottom = false, foundRight = false;
        for (int i = 0; i < mesh.vertices.Count; i++)
        {
            var v = mesh.vertices[i];
            if (Math.Abs(v.Z) > 1e-5 || Math.Abs(v.X - 0.5) > 1e-5 || Math.Abs(v.Y + 0.5) > 1e-5)
                continue;
            var n = mesh.normals[i];
            if (Math.Abs(n.Z) > 1e-8 && Math.Abs(Math.Abs(n.Z) - 1) > 1e-6)
                Assert.Fail("Untwisted side normal picked up an axial component: " + n);
            if (Math.Abs(n.X) < 1e-6 && Math.Abs(n.Y + 1) < 1e-6 && Math.Abs(n.Z) < 1e-8)
                foundBottom = true;
            if (Math.Abs(n.X - 1) < 1e-6 && Math.Abs(n.Y) < 1e-6 && Math.Abs(n.Z) < 1e-8)
                foundRight = true;
        }
        Assert.True(foundBottom);
        Assert.True(foundRight);
    }

    [Fact]
    public void ExtrudeAlongHelix_FrameTorsionTiltsSideNormalWithoutAuthoredTwist()
    {
        const double radius = 2.0;
        const double pitch = 0.5;
        double length = Math.Sqrt(radius * radius + pitch * pitch);
        var guide = new List<CoordinateSystem>();
        const int steps = 60;
        const double phi0 = -0.3;
        for (int i = 0; i <= steps; i++)
        {
            double phi = phi0 + 0.6 * i / steps;
            var origin = new Vec3D(radius * Math.Cos(phi), radius * Math.Sin(phi), pitch * phi);
            var tangent = new Vec3D(-radius * Math.Sin(phi), radius * Math.Cos(phi), pitch).Normalized();
            var up = Vec3DOps.Cross(tangent, new Vec3D(0, 0, 1)).Normalized();
            guide.Add(new CoordinateSystem(origin, Vec3DOps.Cross(up, tangent), up, tangent));
        }

        var converter = MeshTestHelpers.MakeConverter();
        var (contours, contourNormals, names) = SquareEdges();
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        Extruder.GenerateExtrudeAlongCurve(
            converter, CoordinateSystem.Default, contours, contourNormals,
            guide, guide[0], guide[0].Z, triangles, vertices, normals, uv, groups, precise,
            names, "HelixRoll", out _, twistRatePerExtrudeDistance: 0);

        double lengthSquared = length * length;
        double sigma = 1.0 - 0.5 * radius / lengthSquared;
        var tangent0 = new Vec3D(0, radius / length, pitch / length);
        var raw = sigma * new Vec3D(-1, 0, 0) + (pitch / lengthSquared) * 0.5 * tangent0;
        var expected = raw / raw.Length();
        var at = new Vec3D(radius - 0.5, -0.5 * pitch / length, 0.5 * radius / length);
        bool found = false;
        for (int i = 0; i < vertices.Count; i++)
        {
            if ((vertices[i] - at).Length() > 1e-4)
                continue;
            var n = normals[i];
            if ((n - expected).Length() < 1e-3)
                found = true;
        }
        Assert.True(found);
    }

    static (List<Vec3D> vertices, List<Vec3D> normals) ExtrudeSquareEdges(double twistRate, double height, Vec2D center)
    {
        var converter = MeshTestHelpers.MakeConverter();
        var (contours, contourNormals, names) = SquareEdges();
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();
        Extruder.GenerateExtrudedMesh(
            converter, CoordinateSystem.Default, contours, contourNormals,
            height: height, triangles, vertices, normals, uv, groups, precise,
            names, "Square", out _, twistRatePerExtrudeDistance: twistRate, twistCenter: center);
        return (vertices, normals);
    }

    static (List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> normals, List<List<string>> names) SquareEdges()
    {
        var square = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var edges = new List<List<Vec2D>>();
        var edgeNormals = new List<List<Vec2D>>();
        var edgeNames = new List<string>();
        for (int i = 0; i < square.Count - 1; i++)
        {
            var edge = new List<Vec2D> { square[i], square[i + 1] };
            edges.Add(edge);
            edgeNormals.Add(Extruder.WeightedNormals2D(edge));
            edgeNames.Add("E" + i);
        }
        return (
            new List<List<List<Vec2D>>> { edges },
            new List<List<List<Vec2D>>> { edgeNormals },
            new List<List<string>> { edgeNames });
    }
}
