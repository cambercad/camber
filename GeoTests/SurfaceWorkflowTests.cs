using Curves;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public sealed class SurfaceWorkflowTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void SewExactFacesReconstructsAClosedSolid()
    {
        var api = Api();
        var box = api.CreateCuboid(new Vec3D(0), new Vec3D(4, 3, 2), "box");
        var faces = new List<AnchorMesh>();
        foreach (string patch in box.groupIdToExtendedName.Values)
            faces.Add(api.ExtractFaceSurface(box, patch, "surface_" + faces.Count));

        var solid = api.Sew(faces, makeSolid: true, name: "sewn_box");

        Assert.True(solid.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(solid.Mesh.PrecisionPositions,
            solid.Mesh.Triangles));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles),
            23.999, 24.001);
        Assert.Equal(6, solid.groupIdToExtendedName.Count);
    }

    [Fact]
    public void SewKeepsOpenSheetsOpenAndSolidModeRejectsThem()
    {
        var api = Api();
        var box = api.CreateCuboid(new Vec3D(0), new Vec3D(4, 3, 2), "box");
        string first = box.groupIdToExtendedName.Values.First();
        string second = box.groupIdToExtendedName.Values.Skip(1).First();
        var sheets = new[] {
            api.ExtractFaceSurface(box, first, "first"),
            api.ExtractFaceSurface(box, second, "second")
        };

        var open = api.Sew(sheets, name: "open");

        Assert.False(open.IsVolume);
        Assert.Throws<InvalidOperationException>(() => api.Sew(sheets, makeSolid: true));
    }

    [Fact]
    public void ExtrudeRevolveAndSweepExposeUncappedSurfaceVariants()
    {
        var api = Api();
        var rectangle = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "rectangle");
        rectangle.AddRectangle(new Vec2D(0, 0), 3, 2);
        var extruded = api.ExtrudeSurface(rectangle, 5, name: "extruded_surface");

        var meridian = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "meridian");
        meridian.AddRectangle(new Vec2D(0, 2), 4, 1);
        var revolved = api.RevolveSurface(meridian, Math.PI, name: "revolved_surface");

        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginYZ, "profile");
        profile.AddCircle(new Vec2D(0, 0), .5);
        var guide = api.AddLine(new Vec3D(0), new Vec3D(6, 0, 0), "guide");
        var swept = api.SweepSurface(profile, guide, name: "swept_surface");

        foreach (var surface in new[] { extruded, revolved, swept })
        {
            Assert.False(surface.IsVolume);
            Assert.NotEmpty(surface.Mesh.Triangles);
            Assert.False(MeshAnalysis.IsWatertightMesh(surface.Mesh.PrecisionPositions,
                surface.Mesh.Triangles));
        }
        Assert.DoesNotContain(extruded.groupIdToExtendedName.Values,
            patch => patch.Contains("ExtrudeTop", StringComparison.Ordinal));
        Assert.DoesNotContain(revolved.groupIdToExtendedName.Values,
            patch => patch.Contains("StartCap", StringComparison.Ordinal));
    }

    [Fact]
    public void SurfaceSplitReturnsBothExactSides()
    {
        var api = Api();
        var sourceBox = api.CreateCuboid(new Vec3D(-2, -2, -1), new Vec3D(2, 2, 1), "source_box");
        var cutterBox = api.CreateCuboid(new Vec3D(0, -3, -2), new Vec3D(1, 6, 4), "cutter_box");
        string sourceTop = FindPatchOnCoordinate(sourceBox, 2, 1);
        string cutterSide = FindPatchOnCoordinate(cutterBox, 0, 0);
        var source = api.ExtractFaceSurface(sourceBox, sourceTop, "source");
        var cutter = api.ExtractFaceSurface(cutterBox, cutterSide, "cutter");

        var halves = api.SplitSurface(source, cutter, "halves");

        Assert.Equal(2, halves.Count);
        Assert.All(halves, half => {
            Assert.False(half.IsVolume);
            Assert.NotEmpty(half.Mesh.Triangles);
        });
        (double firstMin, double firstMax) = UsedXBounds(halves[0]);
        (double secondMin, double secondMax) = UsedXBounds(halves[1]);
        const double trimTolerance = 1e-3;
        Assert.True(firstMax <= trimTolerance || firstMin >= -trimTolerance);
        Assert.True(secondMax <= trimTolerance || secondMin >= -trimTolerance);
        Assert.True(firstMax <= trimTolerance
            ? secondMin >= -trimTolerance
            : secondMax <= trimTolerance);

        var sewnHalf = api.Sew(new[] { halves[0] }, name: "sewn_half");
        var usedGroups = halves[0].Mesh.TrianglesEx.Select(corner => corner.GroupId).ToHashSet();
        Assert.Equal(usedGroups.Count, sewnHalf.groupIdToExtendedName.Count);
    }

    [Fact]
    public void SurfaceSplitRecognizesAClosedIntersectionLoopOnACurvedSheet()
    {
        const double cutZ = 7.123;
        var api = Api();
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 5, 12, .01, "cylinder");
        string sideName = cylinder.surfaceMetaData.Single(pair =>
            pair.Value.SurfaceType == SurfaceType.Cylindrical).Key;
        var wall = api.ExtractFaceSurface(cylinder, sideName, "wall");

        var planeBlank = api.CreateCuboid(new Vec3D(-6, -6, cutZ - 1),
            new Vec3D(6, 6, cutZ), "plane_blank");
        var plane = api.ExtractFaceSurface(planeBlank,
            FindPatchOnCoordinate(planeBlank, 2, cutZ), "cut_plane");

        var halves = api.SplitSurface(wall, plane, "cylinder_halves");

        Assert.Equal(2, halves.Count);
        Assert.All(halves, half => Assert.NotEmpty(half.Mesh.Triangles));
        var bounds = new List<(double Min, double Max)>();
        foreach (var half in halves) bounds.Add(UsedZBounds(half));
        Assert.Contains(bounds, range => Math.Abs(range.Min) < 1e-3 && Math.Abs(range.Max - cutZ) < 1e-3);
        Assert.Contains(bounds, range => Math.Abs(range.Min - cutZ) < 1e-3 && Math.Abs(range.Max - 12) < 1e-3);
    }

    [Fact]
    public void SampledCylindricalSheetSplitsAtAClosedIntersectionLoop()
    {
        const double cutZ = 7.123;
        var api = Api();
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "circle_profile");
        profile.AddCircle(new Vec2D(0), 5);
        var wall = api.ExtrudeSurface(profile, 12, .01, "sampled_wall");
        var planeBlank = api.CreateCuboid(new Vec3D(-6, -6, cutZ - 1),
            new Vec3D(6, 6, cutZ), "plane_blank");
        var plane = api.ExtractFaceSurface(planeBlank,
            FindPatchOnCoordinate(planeBlank, 2, cutZ), "cut_plane");

        var halves = api.SplitSurface(wall, plane, "sampled_cylinder_halves");

        Assert.Equal(2, halves.Count);
        var bounds = new List<(double Min, double Max)>();
        foreach (var half in halves) bounds.Add(UsedZBounds(half));
        Assert.Contains(bounds, range => Math.Abs(range.Min) < 1e-3 && Math.Abs(range.Max - cutZ) < 1e-3);
        Assert.Contains(bounds, range => Math.Abs(range.Min - cutZ) < 1e-3 && Math.Abs(range.Max - 12) < 1e-3);
    }

    [Fact]
    public void SurfaceIntersectionReturnsAClosedSampledThreeDimensionalCurve()
    {
        const double cutZ = 7.123;
        var api = Api();
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 5, 12, .01, "cylinder");
        string sideName = cylinder.surfaceMetaData.Single(pair =>
            pair.Value.SurfaceType == SurfaceType.Cylindrical).Key;
        var wall = api.ExtractFaceSurface(cylinder, sideName, "wall");
        var planeBlank = api.CreateCuboid(new Vec3D(-6, -6, cutZ - 1),
            new Vec3D(6, 6, cutZ), "plane_blank");
        var plane = api.ExtractFaceSurface(planeBlank,
            FindPatchOnCoordinate(planeBlank, 2, cutZ), "cut_plane");

        IReadOnlyList<Curve3D> curves = api.IntersectionCurves(wall, plane, "cylinder_plane");

        var curve = Assert.IsType<PolylineCurve3D>(Assert.Single(curves));
        double tolerance = api.Converter.SmallestUnit() * 2;
        Assert.Equal("cylinder_plane", curve.Name);
        Assert.True(curve.Vertices.Count > 8);
        Assert.NotNull(curve.PrecisionPositions);
        Assert.NotNull(curve.PrecisionConverter);
        Assert.Equal(curve.Vertices.Count, curve.PrecisionPositions.Count);
        int exactCutZ = api.Converter.Convert(new Vec3D(0, 0, cutZ)).Z;
        Assert.All(curve.PrecisionPositions, point =>
            Assert.Equal(new BigRationalHybrid(exactCutZ), point.Z));
        for (int i = 0; i < curve.Vertices.Count; i++)
            Assert.Equal(curve.Vertices[i].Origin,
                curve.PrecisionConverter.Value.Convert(curve.PrecisionPositions[i]));
        Assert.True((curve.Start - curve.End).Length() < 1e-9);
        foreach (CurveVertex3D sample in curve.Vertices)
        {
            Assert.InRange(Math.Abs(sample.Origin.Z - cutZ), 0, tolerance);
            Assert.InRange(Math.Abs(Math.Sqrt(sample.Origin.X * sample.Origin.X +
                sample.Origin.Y * sample.Origin.Y) - 5), 0, .02);
            Assert.InRange(Math.Abs(sample.Tangent.Length() - 1), 0, 1e-9);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(sample.Tangent, sample.Up)), 0, 1e-9);
        }
        Assert.Contains(curve, api.GetCurves());
    }

    [Fact]
    public void SurfaceSolidIntersectionReturnsItsClosedBoundaryCurve()
    {
        var api = Api();
        var sheetBlank = api.CreateCuboid(new Vec3D(-2, -2, -1),
            new Vec3D(2, 2, 0), "sheet_blank");
        var sheet = api.ExtractFaceSurface(sheetBlank,
            FindPatchOnCoordinate(sheetBlank, 2, 0), "sheet");
        var solid = api.CreateCuboid(new Vec3D(-1), new Vec3D(1), "solid");

        var curves = api.IntersectionCurves(sheet, solid, "section_curve");

        var curve = Assert.IsType<PolylineCurve3D>(Assert.Single(curves));
        double tolerance = api.Converter.SmallestUnit() * 3;
        Assert.True((curve.Start - curve.End).Length() < 1e-9);
        Assert.All(curve.Vertices, sample => Assert.InRange(Math.Abs(sample.Origin.Z), 0, tolerance));
        (double minX, double maxX, double minY, double maxY) =
            (curve.Vertices.Min(point => point.Origin.X), curve.Vertices.Max(point => point.Origin.X),
             curve.Vertices.Min(point => point.Origin.Y), curve.Vertices.Max(point => point.Origin.Y));
        Assert.InRange(minX, -1 - tolerance, -1 + tolerance);
        Assert.InRange(maxX, 1 - tolerance, 1 + tolerance);
        Assert.InRange(minY, -1 - tolerance, -1 + tolerance);
        Assert.InRange(maxY, 1 - tolerance, 1 + tolerance);
    }

    [Fact]
    public void PlanarBoundaryCapsCreateAClosedSolid()
    {
        var api = Api();
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "cap_profile");
        profile.AddRectangle(new Vec2D(0, 0), 3, 2);
        var sides = api.ExtrudeSurface(profile, 5, name: "side_sheet");

        var capped = api.CapPlanarBoundaries(sides, makeSolid: true, name: "capped_solid");

        Assert.True(capped.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(capped.Mesh.PrecisionPositions, capped.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(capped.Mesh.PrecisionPositions,
            capped.Mesh.Triangles));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(capped.Mesh.Positions, capped.Mesh.Triangles),
            29.99, 30.01);
        Assert.Equal(2, capped.surfaceMetaData.Count(data =>
            data.Value.SurfaceType == SurfaceType.Planar && data.Key.StartsWith(EntityNaming.SurfaceCapPrefix)));
    }

    [Fact]
    public void PlanarCapsPreserveAnInnerBoundaryAsAHole()
    {
        var api = Api();
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "ring_profile");
        profile.AddCircle(new Vec2D(0), 3);
        profile.AddCircle(new Vec2D(0), 1);
        var sides = api.ExtrudeSurface(profile, 4, maxDeviation: .01, name: "ring_side_sheet");

        var capped = api.CapPlanarBoundaries(sides, name: "ring_solid");

        Assert.True(capped.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(capped.Mesh.PrecisionPositions, capped.Mesh.Triangles));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(capped.Mesh.Positions, capped.Mesh.Triangles),
            99.5, 101.6);
    }

    [Fact]
    public void SewnMultiPatchSheetCanBeThickened()
    {
        var api = Api();
        var box = api.CreateCuboid(new Vec3D(0), new Vec3D(4), "box");
        var xFace = api.ExtractFaceSurface(box, FindPatchOnCoordinate(box, 0, 4), "x_face");
        var yFace = api.ExtractFaceSurface(box, FindPatchOnCoordinate(box, 1, 4), "y_face");
        var sheet = api.Sew(new[] { xFace, yFace }, name: "two_face_sheet");

        var thickened = api.Thicken(sheet, .25, name: "thickened_sheet");

        Assert.True(thickened.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(thickened.Mesh.PrecisionPositions, thickened.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(thickened.Mesh.PrecisionPositions,
            thickened.Mesh.Triangles));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(thickened.Mesh.PrecisionPositions,
            thickened.Mesh.Triangles) > BigRationalHybrid.Zero);
    }

    [Fact]
    public void SewnCurvedHousingCanBeCappedThickenedFilletedAndExported()
    {
        var api = Api();
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 5, 10, .01, "housing_blank");
        string sideName = cylinder.surfaceMetaData.Single(pair =>
            pair.Value.SurfaceType == SurfaceType.Cylindrical).Key;
        string bottomName = FindPatchOnCoordinate(cylinder, 2, 0);
        var side = api.ExtractFaceSurface(cylinder, sideName, "housing_wall");
        var bottom = api.ExtractFaceSurface(cylinder, bottomName, "housing_base");

        var openHousing = api.Sew(new[] { side, bottom }, name: "open_housing");

        Assert.False(openHousing.IsVolume);
        Assert.False(MeshAnalysis.IsWatertightMesh(openHousing.Mesh.PrecisionPositions,
            openHousing.Mesh.Triangles));
        var cappedHousing = api.CapPlanarBoundaries(openHousing, makeSolid: true, name: "capped_housing");
        var thickenedHousing = api.Thicken(openHousing, .3, name: "thickened_housing");

        AssertSolid(cappedHousing);
        AssertSolid(thickenedHousing);
        Assert.InRange(cappedHousing.Mesh.Positions.Max(point => point.Z), 9.999, 10.001);
        Assert.InRange(thickenedHousing.Mesh.Positions.Min(point => point.Z), -.301, -.299);
        Assert.InRange(thickenedHousing.Mesh.Positions.Max(point => point.Z), 9.999, 10.001);

        int topRim = thickenedHousing.GroupEdges.FindIndex(edge => edge.LineStrips3D.Count > 0 &&
            edge.LineStrips3D.SelectMany(strip => strip.Points).All(point => Math.Abs(point.Z - 10) < .01));
        Assert.True(topRim >= 0, "Thickened housing has no selectable top rim edge.");
        var roundedHousing = api.Fillet(thickenedHousing,
            new List<string> { thickenedHousing.GetEdgeReference(topRim) }, .15, .02, "rounded_housing");
        AssertSolid(roundedHousing);

        string stepPath = Path.Combine(Path.GetTempPath(), $"surface_housing_{Guid.NewGuid():N}.stp");
        try
        {
            api.SaveStepFile(roundedHousing, stepPath);
            Assert.True(new FileInfo(stepPath).Length > 500);
        }
        finally
        {
            if (File.Exists(stepPath)) File.Delete(stepPath);
        }
    }

    [Fact]
    public void SurfaceSweepSupportsMiteredGuideCurveStrips()
    {
        var api = Api();
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        profile.AddRectangle(new Vec2D(-.5, -.5), 1, 1);
        var guides = new List<Curve3D> {
            new Line3D(new Vec3D(0), new Vec3D(0, 0, 5), "rise"),
            new Line3D(new Vec3D(0, 0, 5), new Vec3D(5, 0, 5), "turn")
        };

        var surface = api.SweepSurface(profile, guides, name: "guided_surface");

        Assert.False(surface.IsVolume);
        Assert.NotEmpty(surface.Mesh.Triangles);
        Assert.False(MeshAnalysis.IsWatertightMesh(surface.Mesh.PrecisionPositions, surface.Mesh.Triangles));
        Assert.Contains(surface.groupIdToExtendedName.Values, name => name.Contains("rise", StringComparison.Ordinal));
        Assert.Contains(surface.groupIdToExtendedName.Values, name => name.Contains("turn", StringComparison.Ordinal));
    }

    private static string FindPatchOnCoordinate(AnchorMesh mesh, int axis, double value)
    {
        foreach (var pair in mesh.groupIdToExtendedName)
        {
            bool found = false;
            bool matches = true;
            for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
            {
                if (mesh.Mesh.TrianglesEx[i].GroupId != pair.Key) continue;
                found = true;
                var triangle = mesh.Mesh.Triangles[i];
                int[] indices = { triangle.A, triangle.B, triangle.C };
                for (int j = 0; j < indices.Length; j++)
                {
                    Vec3D point = mesh.Mesh.Positions[indices[j]];
                    double coordinate = axis == 0 ? point.X : axis == 1 ? point.Y : point.Z;
                    if (Math.Abs(coordinate - value) > 1e-3) matches = false;
                }
            }
            if (found && matches) return pair.Value;
        }
        throw new InvalidOperationException("No patch lies on the requested coordinate plane.");
    }

    private static (double Min, double Max) UsedXBounds(AnchorMesh mesh)
    {
        double min = double.PositiveInfinity;
        double max = double.NegativeInfinity;
        foreach (Tri triangle in mesh.Mesh.Triangles)
        {
            int[] indices = { triangle.A, triangle.B, triangle.C };
            foreach (int index in indices)
            {
                double x = mesh.Mesh.Positions[index].X;
                min = Math.Min(min, x);
                max = Math.Max(max, x);
            }
        }
        return (min, max);
    }

    private static (double Min, double Max) UsedZBounds(AnchorMesh mesh)
    {
        double min = double.PositiveInfinity;
        double max = double.NegativeInfinity;
        foreach (Tri triangle in mesh.Mesh.Triangles)
        {
            int[] indices = { triangle.A, triangle.B, triangle.C };
            foreach (int index in indices)
            {
                double z = mesh.Mesh.Positions[index].Z;
                min = Math.Min(min, z);
                max = Math.Max(max, z);
            }
        }
        return (min, max);
    }

    private static void AssertSolid(AnchorMesh mesh)
    {
        Assert.True(mesh.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(mesh.Mesh.PrecisionPositions, mesh.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(mesh.Mesh.PrecisionPositions,
            mesh.Mesh.Triangles));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.PrecisionPositions,
            mesh.Mesh.Triangles).Sign() > 0);
    }

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-10), new Vec3D(20)), 1e-4);
}
