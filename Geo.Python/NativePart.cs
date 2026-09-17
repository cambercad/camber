using System.Collections.Generic;
using CSG;
using Curves;
using DotWrap;
using Geo;
using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Sketcher;

namespace GeoPy;

/// <summary>
/// AOT exports of GeoAPI. Poses are NativeFrame (CoordinateSystem); guides are NativeCurve.
/// </summary>
[DotWrapExpose]
public class NativePart
{
    readonly GeoAPI _inner;

    public NativePart(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, double tolerance)
    {
        var box = new Box3D(new Vec3D(minX, minY, minZ), new Vec3D(maxX, maxY, maxZ));
        _inner = new GeoAPI(box, tolerance);
    }

    public static void Clear()
    {
        GeoAPI.Clear();
    }

    public static string GenerateName(string prefix)
    {
        return GeoAPI.GenerateName(prefix);
    }

    public string Name { get { return _inner.Name; } }

    public double MaxDeviation { get { return _inner.MaxDeviation; } }

    public double SmallestUnit()
    {
        return _inner.Converter.SmallestUnit();
    }

    public string PackWorkingVolume()
    {
        return NativeLattice.PackVolume(_inner.Converter);
    }

    public NativeAssembly GetAssembly(string name)
    {
        return new NativeAssembly(_inner.GetAssembly(NativeUtil.EmptyToNull(name)));
    }

    public NativeSketch Sketch(string plane, string name)
    {
        string planeName = NativeUtil.ResolvePlane(plane);
        return new NativeSketch(_inner.GetPlotterSketcher(planeName, NativeUtil.EmptyToNull(name)));
    }

    public NativeSketch SketchAt(string plane, string originPointName, string name)
    {
        string planeName = NativeUtil.ResolvePlane(plane);
        return new NativeSketch(_inner.GetPlotterSketcher(planeName, originPointName, NativeUtil.EmptyToNull(name)));
    }

    public NativeSketch SketchOnFrame(NativeFrame frame, string name)
    {
        return new NativeSketch(_inner.GetPlotterSketcher(frame.Native, NativeUtil.EmptyToNull(name)));
    }

    public NativeFrame GetPlaneFrame(string plane)
    {
        string planeName = NativeUtil.ResolvePlane(plane);
        if (!_inner.TryGetPlaneFromName(planeName, out Plane3D result) || result == null)
            throw new System.ArgumentException($"'{plane}' does not resolve to a planar surface.");
        return new NativeFrame(result.GetCoordinateSystem());
    }

    public NativeSketch GetConstraintSketcher(string plane, string name)
    {
        string planeName = NativeUtil.ResolvePlane(plane);
        PlotterSketcherCoordSys sk = _inner.GetConstraintSketcher(planeName, NativeUtil.EmptyToNull(name));
        return new NativeSketch(sk);
    }

    public NativeSketch GetConstraintSketcherOnFrame(NativeFrame frame, string name)
    {
        return new NativeSketch(_inner.GetConstraintSketcher(
            frame.Native, NativeUtil.EmptyToNull(name)));
    }

    public void UnregisterSketch(NativeSketch sketch)
    {
        if (sketch != null)
            _inner.UnregisterSketch(sketch.Native);
    }

    public NativeSolid Extrude(NativeSketch sketch, double height, double maxDeviation, double twistRatePerExtrudeDistance, string name)
    {
        return new NativeSolid(_inner.Extrude(sketch.Native, height, maxDeviation, NativeUtil.EmptyToNull(name), twistRatePerExtrudeDistance));
    }

    public NativeSolid ExtrudeTwoSides(NativeSketch sketch, double heightPositive, double heightNegative, double maxDeviation, double twistRatePerExtrudeDistance, string name)
    {
        return new NativeSolid(_inner.ExtrudeTwoSides(sketch.Native, heightPositive, heightNegative, maxDeviation, NativeUtil.EmptyToNull(name), twistRatePerExtrudeDistance));
    }

    public NativeProjectedSketch ProjectSketchOntoMesh(NativeSketch sketch, NativeSolid target, double maxDeviation, string name)
    {
        return new NativeProjectedSketch(_inner.ProjectSketchOntoMesh(
            sketch.Native, target.Native, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid ExtrudeProjectedSketch(NativeProjectedSketch projected, double height, string name)
    {
        return new NativeSolid(_inner.ExtrudeProjectedSketch(
            projected.Native, height, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid Revolve(NativeSketch sketch, double angle, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.Revolve(sketch.Native, angle, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateCylinder(NativeFrame pose, double radius, double heightAlongZ, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.CreateCylinder(pose.Native, radius, heightAlongZ, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateCylinderRevolve(NativeFrame pose, double radius, double height, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.CreateCylinderRevolve(pose.Native, radius, height, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateSphere(NativeFrame pose, double radius, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.CreateSphere(pose.Native, radius, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateCube(NativeFrame pose, double extent, string name)
    {
        return new NativeSolid(_inner.CreateCube(pose.Native, extent, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateCuboidAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, string name)
    {
        return new NativeSolid(_inner.CreateCuboid(NativeUtil.V3(minX, minY, minZ), NativeUtil.V3(maxX, maxY, maxZ), NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateCuboid(NativeFrame pose, double extentX, double extentY, double extentZ, string name)
    {
        return new NativeSolid(_inner.CreateCuboid(pose.Native, NativeUtil.V3(extentX, extentY, extentZ), NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid CreateMetricThreadForBoltNegative(
        NativeFrame pose,
        double majorDiameter,
        double pitch,
        double length,
        double maxDeviation,
        int rightHanded,
        string name,
        double outerRadius)
    {
        return new NativeSolid(_inner.CreateMetricThreadForBoltNegative(
            pose.Native,
            majorDiameter,
            pitch,
            length,
            maxDeviation,
            rightHanded != 0,
            NativeUtil.EmptyToNull(name),
            outerRadius));
    }

    public NativeSolid CreateMetricThreadForNutNegative(
        NativeFrame pose,
        double majorDiameter,
        double pitch,
        double length,
        double maxDeviation,
        int rightHanded,
        string name,
        int includeBoreChamfers)
    {
        return new NativeSolid(_inner.CreateMetricThreadForNutNegative(
            pose.Native,
            majorDiameter,
            pitch,
            length,
            maxDeviation,
            rightHanded != 0,
            NativeUtil.EmptyToNull(name),
            includeBoreChamfers != 0));
    }

    public NativeSolid CreateMetricThreadForHoleNegative(
        NativeFrame pose,
        double majorDiameter,
        double pitch,
        double length,
        double maxDeviation,
        int rightHanded,
        string name,
        int includeBoreChamfers)
    {
        return new NativeSolid(_inner.CreateMetricThreadForHoleNegative(
            pose.Native,
            majorDiameter,
            pitch,
            length,
            maxDeviation,
            rightHanded != 0,
            NativeUtil.EmptyToNull(name),
            includeBoreChamfers != 0));
    }

    public NativeCurve AddLine(double x0, double y0, double z0, double x1, double y1, double z1, string name)
    {
        return new NativeCurve(_inner.AddLine(NativeUtil.V3(x0, y0, z0), NativeUtil.V3(x1, y1, z1), NativeUtil.EmptyToNull(name)));
    }

    public NativeCurve AddLineByNames(string startPointName, string endPointName, string lineName)
    {
        return new NativeCurve(_inner.AddLine(startPointName, endPointName, NativeUtil.EmptyToNull(lineName)));
    }

    public void AddPlane(string planeName, string originAnchorName)
    {
        _inner.AddPlane(planeName, originAnchorName);
    }

    public NativeSolid ExtrudeAlongCurve(NativeSketch sketch, NativeCurve curve, double maxDeviation, double twistRatePerExtrudeDistance, string name, string referenceDirection)
    {
        return new NativeSolid(_inner.ExtrudeAlongCurve(sketch.Native, curve.Native, maxDeviation, NativeUtil.EmptyToNull(name), twistRatePerExtrudeDistance,
            string.IsNullOrEmpty(referenceDirection) ? null : NativePack.ReadPoints3(referenceDirection)[0]));
    }

    public NativeSolid ExtrudeAlongCurveStrip(NativeSketch sketch, NativeCurveList guide, double maxDeviation, double twistRatePerExtrudeDistance, string name)
    {
        return new NativeSolid(_inner.ExtrudeAlongCurveStrip(sketch.Native, guide.Items, maxDeviation, twistRatePerExtrudeDistance, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid ExtrudeAlongSketch(NativeSketch profile, NativeSketch guide, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.ExtrudeAlongSketch(profile.Native, guide.Native, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid Loft(NativeSketchList sketches, NativeLoftOptions options, double maxDeviation, string name, string firstCurves)
    {
        LoftOptions loftOptions = options == null ? LoftOptions.Default : options.ToLoftOptions();
        if (!string.IsNullOrEmpty(firstCurves)) loftOptions.FirstCurves = firstCurves.Split('|');
        return new NativeSolid(_inner.Loft(sketches.Items, loftOptions, NativeUtil.EmptyToNull(name), maxDeviation));
    }

    public NativeSolid LoftSurface(NativeSketchList sections, NativeCurveList guides,
        string startTangent, string endTangent, double maxDeviation, string name)
    {
        static Vec3D? Tangent(string packed) => string.IsNullOrEmpty(packed)
            ? null : NativePack.ReadPoints3(packed)[0];
        return new NativeSolid(_inner.LoftSurface(sections.Items, guides?.Items,
            Tangent(startTangent), Tangent(endTangent), NativeUtil.EmptyToNull(name), maxDeviation));
    }

    public NativeSolid Boolean(NativeSolid a, NativeSolid b, int operation, string name)
    {
        return new NativeSolid(_inner.Boolean(a.Native, b.Native, NativeUtil.ToBooleanOp(operation), NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid Union(NativeSolid a, NativeSolid b, string name)
    {
        return Boolean(a, b, (int)BooleanOp.Union, name);
    }

    public NativeSolid Cut(NativeSolid a, NativeSolid b, string name)
    {
        return Boolean(a, b, (int)BooleanOp.Difference, name);
    }

    public NativeSolid Intersect(NativeSolid a, NativeSolid b, string name)
    {
        return Boolean(a, b, (int)BooleanOp.Intersect, name);
    }

    public NativeSolid BatchUnion(NativeSolidList meshes)
    {
        AnchorMesh result = _inner.BatchUnion(meshes.Items);
        if (result == null)
            return null;
        return new NativeSolid(result);
    }

    public NativeSolid BatchBooleanChain(NativeSolid meshA, NativeBooleanChain chain)
    {
        return new NativeSolid(_inner.BatchBooleanChain(meshA.Native, chain.Items));
    }

    public NativeSolid CopySolidAsInstance(NativeSolid source, string name)
    {
        return new NativeSolid(_inner.CopyMeshAsInstance(source.Native, name));
    }

    public NativeSolidList PatternLinear(NativeSolid seed, int count, double x, double y, double z, string name)
    {
        var result = new NativeSolidList();
        result.Items.AddRange(_inner.PatternLinear(seed.Native, count, new Vec3D(x, y, z), NativeUtil.EmptyToNull(name)));
        return result;
    }

    public NativeSolidList PatternCircular(NativeSolid seed, int count, NativeFrame axis, double angle, string name)
    {
        var result = new NativeSolidList();
        result.Items.AddRange(_inner.PatternCircular(seed.Native, count, axis.Native, angle, NativeUtil.EmptyToNull(name)));
        return result;
    }

    public NativeSolid Mirror(NativeSolid source, NativeFrame plane, string name)
        => new NativeSolid(_inner.Mirror(source.Native, plane.Native, NativeUtil.EmptyToNull(name)));

    public NativeSolid Fillet(NativeSolid mesh, string edgeNames, double radius, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.Fillet(mesh.Native, NativeUtil.SplitNames(edgeNames), radius, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid Chamfer(NativeSolid mesh, string edgeNames, double distance, double maxDeviation, string name)
    {
        return new NativeSolid(_inner.Chamfer(mesh.Native, NativeUtil.SplitNames(edgeNames), distance, maxDeviation, NativeUtil.EmptyToNull(name)));
    }

    public NativeSolid GetMeshFromName(string meshName)
    {
        AnchorMesh mesh = _inner.GetMeshFromName(meshName);
        if (mesh == null)
            return null;
        return new NativeSolid(mesh);
    }

    public NativeSolid LoadStlFile(string filePath, double groupBorderAngleThresholdDegree, string name, int requireWatertight)
    {
        return new NativeSolid(_inner.LoadStlFile(filePath, groupBorderAngleThresholdDegree, NativeUtil.EmptyToNull(name), requireWatertight != 0));
    }

    public NativeSolid LoadOffFile(string filePath, double groupBorderAngleThresholdDegree, string name, int requireWatertight)
    {
        return new NativeSolid(_inner.LoadOffFile(filePath, groupBorderAngleThresholdDegree, NativeUtil.EmptyToNull(name), requireWatertight != 0));
    }

    public NativeSolid LoadWavefrontObjFile(string filePath, double groupBorderAngleThresholdDegree, string name, double scale)
    {
        return new NativeSolid(_inner.LoadWavefrontObjFile(filePath, groupBorderAngleThresholdDegree, NativeUtil.EmptyToNull(name), scale));
    }

    public void SaveStl(NativeSolid mesh, string filePath, int binary)
    {
        if (binary != 0)
            _inner.SaveBinaryStlFile(mesh.Native, filePath);
        else
            _inner.SaveStlFile(mesh.Native, filePath);
    }

    public void SaveStep(NativeSolid mesh, string filePath)
    {
        _inner.SaveStepFile(mesh.Native, filePath);
    }

    public void SaveIges(NativeSolid mesh, string filePath)
    {
        _inner.SaveIgesFile(mesh.Native, filePath);
    }

    public void SaveOff(NativeSolid mesh, string filePath)
    {
        _inner.SaveOffFile(mesh.Native, filePath);
    }

    public void SaveWavefrontObj(NativeSolid mesh, string filePath)
    {
        _inner.SaveWavefrontObjFile(mesh.Native, filePath);
    }

    public void SaveUsda(NativeSolid mesh, string filePath)
    {
        _inner.SaveUsdaFile(mesh.Native, filePath);
    }

    public NativeSection Section(NativeFrame plane) => new(new SectionView(_inner.GetMeshes(), _inner.Converter, plane.Native));

    public NativeSection SectionSolid(NativeSolid solid, NativeFrame plane)
        => new(new SectionView(new[] { solid.Native }, _inner.Converter, plane.Native));

    public string DumpDisplay()
    {
        return DisplayPack.PackMeshes(_inner.GetMeshes());
    }

    public NativeSolid SolidFromMesh(string packedPositions, string packedTriangles, string name)
    {
        var positions = NativePack.ReadPoints3(packedPositions);
        var triangles = NativePack.ReadTriangles(packedTriangles);
        return new NativeSolid(_inner.CreateFromTriangles(positions, triangles, NativeUtil.EmptyToNull(name)));
    }

    public string Raycast(NativeSolid mesh, double ox, double oy, double oz, double dx, double dy, double dz)
    {
        if (mesh == null)
            return string.Empty;
        Geo.RayMeshHit hit;
        if (!Geo.RayMeshExact.TryCast(mesh.Native, _inner.Converter, NativeUtil.V3(ox, oy, oz), NativeUtil.V3(dx, dy, dz), out hit)
            || hit == null)
        {
            return string.Empty;
        }
        return NativeSection.PackHit(hit);
    }

    public string DumpSolidDisplay(NativeSolid mesh)
    {
        return DisplayPack.PackSolid(mesh.Native);
    }
}

[DotWrapExpose]
public class NativeSketch
{
    internal readonly PlotterSketcherCoordSys Native;
    readonly List<Curve2D> _constraintCurves = new List<Curve2D>();

    internal NativeSketch(PlotterSketcherCoordSys native)
    {
        Native = native;
    }

    public string Name { get { return Native.Name; } }

    public NativeFrame Frame()
    {
        return new NativeFrame(Native.CoordinateSystem);
    }

    public void AddLine(double x0, double y0, double x1, double y1)
    {
        if (Native is ConstrainedSketcher constrained)
            _constraintCurves.Add(constrained.AddCLine(new Vec2D(x0, y0), new Vec2D(x1, y1)));
        else
            Native.AddLine(new Vec2D(x0, y0), new Vec2D(x1, y1));
    }

    public void AddLineNamed(double x0, double y0, double x1, double y1, string name)
    {
        Line2D line;
        if (Native is ConstrainedSketcher constrained)
        {
            line = constrained.AddCLine(new Vec2D(x0, y0), new Vec2D(x1, y1));
            _constraintCurves.Add(line);
        }
        else
        {
            line = Native.AddLine(new Vec2D(x0, y0), new Vec2D(x1, y1));
        }
        ApplyCurveName(line, name);
    }

    public void AddEllipse(double cx, double cy, double rx, double ry, double rotation, string name)
    {
        var axis = new Vec2D(rx*Math.Cos(rotation),rx*Math.Sin(rotation));
        if (!double.IsFinite(rx) || rx <= 0 || !double.IsFinite(rotation))
            throw new ArgumentOutOfRangeException(nameof(rx), "Ellipse radii must be positive and rotation finite.");
        var ellipse = Native.AddEllipse(new Vec2D(cx,cy),axis,ry);
        if (Native is ConstrainedSketcher)
            _constraintCurves.Add(ellipse);
        ApplyCurveName(ellipse,name);
    }

    public void AddCircle(double cx, double cy, double radius)
    {
        if (Native is ConstrainedSketcher constrained)
            _constraintCurves.Add(constrained.AddCCircle(new Vec2D(cx, cy), radius));
        else
            Native.AddCircle(new Vec2D(cx, cy), radius);
    }

    public void AddCircleNamed(double cx, double cy, double radius, string name)
    {
        Circle2D circle;
        if (Native is ConstrainedSketcher constrained)
        {
            circle = constrained.AddCCircle(new Vec2D(cx, cy), radius);
            _constraintCurves.Add(circle);
        }
        else
        {
            circle = Native.AddCircle(new Vec2D(cx, cy), radius);
        }
        ApplyCurveName(circle, name);
    }

    public void AddArc(double x0, double y0, double xm, double ym, double x1, double y1)
    {
        if (Native is ConstrainedSketcher constrained)
            _constraintCurves.Add(constrained.AddCArc(
                new Vec2D(x0, y0), new Vec2D(xm, ym), new Vec2D(x1, y1)));
        else
            Native.AddArc(new Vec2D(x0, y0), new Vec2D(xm, ym), new Vec2D(x1, y1));
    }

    public void AddArcNamed(double x0, double y0, double xm, double ym, double x1, double y1, string name)
    {
        Arc2D arc;
        if (Native is ConstrainedSketcher constrained)
        {
            arc = constrained.AddCArc(
                new Vec2D(x0, y0), new Vec2D(xm, ym), new Vec2D(x1, y1));
            _constraintCurves.Add(arc);
        }
        else
        {
            arc = Native.AddArc(new Vec2D(x0, y0), new Vec2D(xm, ym), new Vec2D(x1, y1));
        }
        ApplyCurveName(arc, name);
    }

    static void ApplyCurveName(Curve2D curve, string name)
    {
        if (curve != null && !string.IsNullOrEmpty(name))
            curve.Name = name;
    }

    public void AddRectangle(double x0, double y0, double x1, double y1)
    {
        if (Native is ConstrainedSketcher constrained)
            _constraintCurves.AddRange(constrained.AddCRectangleFromCorners(
                new Vec2D(x0, y0), new Vec2D(x1, y1)));
        else
            Native.AddRectangleFromCorners(new Vec2D(x0, y0), new Vec2D(x1, y1));
    }

    public void AddRectangleNamed(double x0, double y0, double x1, double y1,
        string south, string east, string north, string west)
    {
        AddRectangle(x0, y0, x1, y1);
        NameRectangleSides(south, east, north, west);
    }

    public void AddRectangleCentered(double cx, double cy, double sizeX, double sizeY)
    {
        AddRectangle(cx - sizeX / 2, cy - sizeY / 2, cx + sizeX / 2, cy + sizeY / 2);
    }

    public void AddRectangleCenteredNamed(
        double cx, double cy, double sizeX, double sizeY,
        string south, string east, string north, string west)
    {
        AddRectangleCentered(cx, cy, sizeX, sizeY);
        NameRectangleSides(south, east, north, west);
    }

    void NameRectangleSides(string south, string east, string north, string west)
    {
        var all = Native.GetAllCurves();
        ApplyCurveName(all[all.Count - 4], south);
        ApplyCurveName(all[all.Count - 3], east);
        ApplyCurveName(all[all.Count - 2], north);
        ApplyCurveName(all[all.Count - 1], west);
    }

    public void AddText(string text, double originX, double originY, string family, double emSize, int styleFlags)
    {
        var font = new SketchFontSpec(
            string.IsNullOrWhiteSpace(family) ? "Arial" : family,
            emSize > 0 ? emSize : 100.0,
            (SketchFontStyleFlags)styleFlags);
        Native.AddText(text ?? string.Empty, new Vec2D(originX, originY), font);
    }

    public string TessellatePolylines(double maxDeviation)
    {
        return NativePack.WriteLoops2(CollectPolylines(maxDeviation, closedOnly: false, excludeHelpers: false));
    }

    /// <summary>
    /// Tessellate closed sketch contours and fill them. Nested holes / islands use
    /// <see cref="TriangulationWithHoles{Arithmetic,Vec,Scalar}"/> (polygon tree).
    /// Construction geometry is skipped. <paramref name="packedVolume"/> is the part lattice (or auto).
    /// </summary>
    public string Triangulate(double maxDeviation, string packedVolume)
    {
        var loops = CollectPolylines(maxDeviation, closedOnly: true, excludeHelpers: true);
        return NativeGeom.Triangulate(NativePack.WriteLoops2(loops), packedVolume);
    }

    List<List<Vec2D>> CollectPolylines(double maxDeviation, bool closedOnly, bool excludeHelpers)
    {
        if (maxDeviation <= 0)
            maxDeviation = 0.01;
        List<List<List<Vec2D>>> allNormals;
        List<List<string>> names;
        Dictionary<string, CurveMetaData> meta;
        var flags = excludeHelpers ? SketchTessellationFlags.ExcludeHelperGeometry : SketchTessellationFlags.None;
        var strips = Native.Tessellate(maxDeviation, out allNormals, out names, out meta, maxDeviation, flags);
        var loops = new List<List<Vec2D>>();
        if (strips == null)
            return loops;
        for (int s = 0; s < strips.Count; s++)
        {
            var combined = new List<Vec2D>();
            var strip = strips[s];
            if (strip == null)
                continue;
            for (int i = 0; i < strip.Count; i++)
            {
                var poly = strip[i];
                if (poly == null)
                    continue;
                for (int k = 0; k < poly.Count; k++)
                {
                    Vec2D p = poly[k];
                    if (combined.Count > 0 && (p - combined[combined.Count - 1]).LengthSquared() <= 1e-24)
                        continue;
                    combined.Add(p);
                }
            }
            NativePack.DropClosingDuplicate(combined);
            if (closedOnly)
            {
                if (combined.Count >= 3)
                    loops.Add(combined);
            }
            else if (combined.Count >= 2)
            {
                loops.Add(combined);
            }
        }
        return loops;
    }

    public void AppendLine(double x, double y)
    {
        Native.AppendLine(new Vec2D(x, y));
    }

    public void SetStartPoint(double x, double y)
    {
        Native.SetStartPoint(new Vec2D(x, y));
    }

    public void MoveTo(double x, double y)
    {
        Native.MoveToPointAndStartNewCurveStrip(new Vec2D(x, y));
    }

    public void AppendLineHorizontal(double endX)
    {
        Native.AppendLineHorizontal(endX);
    }

    public void AppendLineVertical(double endY)
    {
        Native.AppendLineVertical(endY);
    }

    public void AppendArcTangentialLeft(double radius, double angle)
    {
        Native.AppendArcTangentialLeft(radius, angle);
    }

    public void AppendArcTangentialRight(double radius, double angle)
    {
        Native.AppendArcTangentialRight(radius, angle);
    }

    public void AddCubicHermiteSpline(
        string pointsJoined, double startTx, double startTy, double endTx, double endTy,
        int hasStartTangent, int hasEndTangent)
    {
        List<Vec2D> points = ParseJoinedPoints(pointsJoined);
        Vec2D? startTangent = hasStartTangent != 0 ? new Vec2D(startTx, startTy) : (Vec2D?)null;
        Vec2D? endTangent = hasEndTangent != 0 ? new Vec2D(endTx, endTy) : (Vec2D?)null;
        Native.AddCubicHermiteSpline(points, startTangent, endTangent);
    }

    public void AppendCubicHermiteSpline(
        string pointsJoined, double startTx, double startTy, double endTx, double endTy,
        int hasStartTangent, int hasEndTangent)
    {
        List<Vec2D> points = ParseJoinedPoints(pointsJoined);
        Vec2D? startTangent = hasStartTangent != 0 ? new Vec2D(startTx, startTy) : (Vec2D?)null;
        Vec2D? endTangent = hasEndTangent != 0 ? new Vec2D(endTx, endTy) : (Vec2D?)null;
        Native.AppendCubicHermiteSpline(points, startTangent, endTangent);
    }

    public void AddInvolute(
        double cx, double cy, double baseRadius, double tStart, double tEnd, double rotation, double maxDeviation)
    {
        Native.AddInvolute(new Vec2D(cx, cy), baseRadius, tStart, tEnd, rotation, maxDeviation);
    }

    public void AddInvoluteGear(
        double cx, double cy, double module, int teeth, double pressureAngleRadians,
        double addendumFactor, double dedendumFactor, double maxDeviation)
    {
        Native.AddInvoluteGear(
            new Vec2D(cx, cy), module, teeth, pressureAngleRadians, addendumFactor, dedendumFactor, maxDeviation);
    }

    public void AddInvoluteInternalGear(
        double cx, double cy, double module, int teeth, double pressureAngleRadians,
        double addendumFactor, double dedendumFactor, double maxDeviation)
    {
        Native.AddInvoluteInternalGear(
            new Vec2D(cx, cy), module, teeth, pressureAngleRadians, addendumFactor, dedendumFactor, maxDeviation);
    }

    public string RepeatCircularNamed(
        string namesJoined, double cx, double cy, int count, double totalAngle, int includeOriginal)
    {
        List<Curve2D> sources = CollectCurvesByNames(namesJoined);
        List<TransformedSketchCurve2D> added = Native.RepeatCircular(
            sources, new Vec2D(cx, cy), count, totalAngle, includeOriginal != 0);
        return JoinCurveNames(added);
    }

    public string RepeatGridNamed(
        string namesJoined, int countX, int countY, double stepXx, double stepXy, double stepYx, double stepYy,
        int includeOriginal)
    {
        List<Curve2D> sources = CollectCurvesByNames(namesJoined);
        List<TransformedSketchCurve2D> added = Native.RepeatGrid(
            sources, countX, countY, new Vec2D(stepXx, stepXy), new Vec2D(stepYx, stepYy), includeOriginal != 0);
        return JoinCurveNames(added);
    }

    static List<Vec2D> ParseJoinedPoints(string pointsJoined)
    {
        var points = new List<Vec2D>();
        string[] parts = (pointsJoined ?? "").Split(new[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < parts.Length; i++)
        {
            string[] xy = parts[i].Split(',');
            if (xy.Length < 2)
                continue;
            points.Add(new Vec2D(
                double.Parse(xy[0], System.Globalization.CultureInfo.InvariantCulture),
                double.Parse(xy[1], System.Globalization.CultureInfo.InvariantCulture)));
        }
        return points;
    }

    static string JoinCurveNames(IList<TransformedSketchCurve2D> curves)
    {
        if (curves == null || curves.Count == 0)
            return "";
        var names = new List<string>(curves.Count);
        for (int i = 0; i < curves.Count; i++)
        {
            if (curves[i] != null && !string.IsNullOrEmpty(curves[i].Name))
                names.Add(curves[i].Name);
        }
        return string.Join("|", names);
    }

    public string OffsetNetworkNamed(
        string namesJoined, double offset, int joinType, int endCap)
    {
        return OffsetNamed(namesJoined, offset, joinType, endCap, 0);
    }

    /// <summary>
    /// side: 0 both (centerline network), 1 out, 2 in, 3 left, 4 right.
    /// Returns `|`-joined sampled-curve names (`source@in_offset` / `source@out_offset` / `source@end_cap`).
    /// </summary>
    public string OffsetNamed(
        string namesJoined, double distance, int joinType, int endCap, int side)
    {
        var sources = new List<Curve2D>();
        string[] names = (namesJoined ?? "").Split(new[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < names.Length; i++)
            sources.Add(FindCurveByName(names[i].Trim()));

        if (side <= 0)
        {
            var networkOptions = new SketchStripOffsetOptions(
                (SketchOffsetJoinType)joinType,
                SketchOffsetOpenMode.Outline,
                (SketchOffsetEndCap)endCap,
                -1,
                -1,
                1e-6);
            var pieces = Native.OffsetNetwork(sources, System.Math.Abs(distance), networkOptions);
            return JoinOffsetNames(pieces);
        }

        double signed = System.Math.Abs(distance);
        SketchOffsetOpenMode openMode = SketchOffsetOpenMode.Parallel;
        if (side == 2)
            signed = -signed;
        else if (side == 4)
            signed = -signed;
        var stripOptions = new SketchStripOffsetOptions(
            (SketchOffsetJoinType)joinType,
            openMode,
            (SketchOffsetEndCap)endCap,
            -1,
            -1,
            1e-6);
        var strip = Native.OffsetStrip(sources, signed, stripOptions);
        return JoinOffsetNames(strip.CreateSampledCurves());
    }

    /// <summary>
    /// openMode: 0 auto (side both → network, else strip parallel), 1 network,
    /// 2 strip outline, 3 strip parallel.
    /// </summary>
    public string OffsetStyled(
        string namesJoined, double distance, int joinType, int endCap, int side, int openMode)
    {
        var sources = new List<Curve2D>();
        string[] names = (namesJoined ?? "").Split(new[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < names.Length; i++)
            sources.Add(FindCurveByName(names[i].Trim()));

        var join = (SketchOffsetJoinType)joinType;
        var cap = (SketchOffsetEndCap)endCap;
        bool useNetwork = openMode == 1 || (openMode <= 0 && side <= 0);
        if (useNetwork)
        {
            var networkOptions = new SketchStripOffsetOptions(
                join, SketchOffsetOpenMode.Outline, cap, -1, -1, 1e-6);
            var pieces = Native.OffsetNetwork(sources, System.Math.Abs(distance), networkOptions);
            return JoinOffsetNames(pieces);
        }

        SketchOffsetOpenMode mode = openMode == 2
            ? SketchOffsetOpenMode.Outline
            : SketchOffsetOpenMode.Parallel;
        double signed = System.Math.Abs(distance);
        if (mode == SketchOffsetOpenMode.Parallel && (side == 2 || side == 4))
            signed = -signed;
        var stripOptions = new SketchStripOffsetOptions(join, mode, cap, -1, -1, 1e-6);
        var strip = Native.OffsetStrip(sources, signed, stripOptions);
        return JoinOffsetNames(strip.CreateSampledCurves());
    }

    static string JoinOffsetNames(System.Collections.Generic.IList<OffsetSampledCurve2D> pieces)
    {
        if (pieces == null || pieces.Count == 0)
            return "";
        var names = new System.Collections.Generic.List<string>(pieces.Count);
        for (int i = 0; i < pieces.Count; i++)
        {
            if (pieces[i] != null && !string.IsNullOrEmpty(pieces[i].Name))
                names.Add(pieces[i].Name);
        }
        return string.Join("|", names);
    }

    List<Curve2D> CollectCurvesByNames(string namesJoined)
    {
        var sources = new List<Curve2D>();
        string[] names = (namesJoined ?? "").Split(new[] { '|' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < names.Length; i++)
            sources.Add(FindCurveByName(names[i].Trim()));
        return sources;
    }

    Curve2D FindCurveByName(string name)
    {
        if (string.IsNullOrEmpty(name))
            throw new System.ArgumentException("Curve name is empty.");
        for (int i = 0; i < _constraintCurves.Count; i++)
        {
            if (string.Equals(_constraintCurves[i].Name, name, System.StringComparison.Ordinal))
                return _constraintCurves[i];
        }
        var all = Native.GetAllCurves();
        if (all != null)
        {
            for (int i = 0; i < all.Count; i++)
            {
                if (string.Equals(all[i].Name, name, System.StringComparison.Ordinal))
                    return all[i];
            }
        }
        throw new System.ArgumentException("Unknown sketch curve '" + name + "'.");
    }

    ConstrainedSketcher ConstraintSketcher()
    {
        ConstrainedSketcher constrained = Native as ConstrainedSketcher;
        if (constrained == null)
            throw new System.InvalidOperationException("This sketch is not constraint-capable.");
        return constrained;
    }

    public int SolveAfterEveryConstraint
    {
        get
        {
            ConstrainedSketcher constrained = Native as ConstrainedSketcher;
            return (constrained != null && constrained.SolveAfterEveryConstraint) ? 1 : 0;
        }
        set
        {
            ConstrainedSketcher constrained = Native as ConstrainedSketcher;
            if (constrained != null)
                constrained.SolveAfterEveryConstraint = value != 0;
        }
    }

    public string LastCurveName()
    {
        var curves = Native.GetAllCurves();
        if (curves.Count == 0)
            throw new InvalidOperationException("The sketch has no curves.");
        return curves[curves.Count-1].Name;
    }

    public int ConstraintCurveIndex(string name)
    {
        int index = _constraintCurves.IndexOf(FindCurveByName(name));
        if (index < 0)
            throw new ArgumentException($"Sketch curve '{name}' is not a constrainable curve.");
        return index;
    }

    public int ConstraintCurveCount
    {
        get { return _constraintCurves.Count; }
    }

    public void SetCurveConstruction(int index, int construction)
    {
        Curve2D curve = ConstraintCurve<Curve2D>(index);
        ApplyConstructionFlag(curve, construction);
    }

    public void SetCurveConstructionNamed(string name, int construction)
    {
        ApplyConstructionFlag(FindCurveByName(name), construction);
    }

    static void ApplyConstructionFlag(Curve2D curve, int construction)
    {
        if (construction != 0)
            curve.Flags = curve.Flags | CurveFlags.HelperGeometry;
        else
            curve.Flags = curve.Flags & ~CurveFlags.HelperGeometry;
    }

    public int CurveIsConstruction(int index)
    {
        return ConstraintCurve<Curve2D>(index).IsHelperGeometry ? 1 : 0;
    }

    public int ConstraintCount
    {
        get
        {
            ConstrainedSketcher constrained = Native as ConstrainedSketcher;
            if (constrained == null)
                return 0;
            return constrained.GetConstraints().Count;
        }
    }

    public void RemoveLastConstraint()
    {
        var list = ConstraintSketcher().GetConstraints();
        if (list.Count == 0)
            return;
        list.RemoveAt(list.Count - 1);
    }

    public void RemoveLastConstraintCurve()
    {
        if (_constraintCurves.Count == 0)
            return;
        Curve2D last = _constraintCurves[_constraintCurves.Count - 1];
        _constraintCurves.RemoveAt(_constraintCurves.Count - 1);
        ConstraintSketcher().RemoveCurve(last);
    }

    T ConstraintCurve<T>(int index) where T : class
    {
        if (index < 0 || index >= _constraintCurves.Count)
            throw new System.ArgumentOutOfRangeException(nameof(index));
        T result = _constraintCurves[index] as T;
        if (result == null)
            throw new System.ArgumentException($"Sketch curve {index} is not a {typeof(T).Name}.");
        return result;
    }

    CVec2D ConstraintPoint(int curveIndex, int point)
    {
        Curve2D curve = ConstraintCurve<Curve2D>(curveIndex);
        if (curve is CLine2D line)
        {
            if (point == 0)
                return line.CStart;
            if (point == 1)
                return line.CEnd;
            if (point == 3)
                return line.Evaluate(0.5);
            return line.CEnd;
        }
        if (curve is CArc2D arc)
        {
            if (point == 0)
                return arc.CStart;
            if (point == 1)
                return arc.CEnd;
            if (point == 2)
                return arc.CCenter;
            if (point == 3)
                return arc.Evaluate(0.5);
            return arc.CEnd;
        }
        if (curve is CCircle2D circle)
        {
            if (point == 2)
                return circle.CCenter;
            if (point == 3)
                return circle.Evaluate(0.0);
            if (point == 4)
                return circle.Evaluate(0.25);
            if (point == 5)
                return circle.Evaluate(0.5);
            if (point == 6)
                return circle.Evaluate(0.75);
            throw new System.ArgumentException(
                $"Circle curve {curveIndex} has no point role {point}.");
        }
        throw new System.ArgumentException($"Sketch curve {curveIndex} has no constrainable point.");
    }

    public void SetHorizontal(int curve) =>
        ConstraintSketcher().SetHorizontal(ConstraintCurve<CLine2D>(curve));

    public void SetVertical(int curve) =>
        ConstraintSketcher().SetVertical(ConstraintCurve<CLine2D>(curve));

    void LockDerivedCircleRadius(int curveIndex, int point)
    {
        Curve2D curve = ConstraintCurve<Curve2D>(curveIndex);
        if (curve is CCircle2D circle && point >= 3 && point <= 6)
            ConstraintSketcher().SetRadius(circle, circle.CRadius.Evaluate());
        else if (curve is CArc2D arc && point == 3)
            ConstraintSketcher().SetRadius(arc, arc.CRadius.Evaluate());
    }

    public void SetCoincident(int curveA, int pointA, int curveB, int pointB)
    {
        LockDerivedCircleRadius(curveA, pointA);
        LockDerivedCircleRadius(curveB, pointB);
        ConstraintSketcher().SetCoincident(
            ConstraintPoint(curveA, pointA), ConstraintPoint(curveB, pointB));
    }

    /// <summary>
    /// Coincident a free sketch handle with Evaluate(uniform) on another curve.
    /// Same path as the C# sketcher name resolver (Line@0.5, Circle@0.25, …).
    /// </summary>
    public void SetCoincidentOnCurve(int pointCurve, int point, int targetCurve, double uniform)
    {
        Curve2D target = ConstraintCurve<Curve2D>(targetCurve);
        IEvaluateCPoint evaluatable = target as IEvaluateCPoint;
        if (evaluatable == null)
            throw new System.ArgumentException(
                $"Sketch curve {targetCurve} has no parameterized point.");
        LockCircularRadius(targetCurve);
        ConstraintSketcher().SetCoincident(
            ConstraintPoint(pointCurve, point), evaluatable.Evaluate(uniform));
    }

    void LockCircularRadius(int curveIndex)
    {
        Curve2D curve = ConstraintCurve<Curve2D>(curveIndex);
        if (curve is CCircle2D circle)
            ConstraintSketcher().SetRadius(circle, circle.CRadius.Evaluate());
        else if (curve is CArc2D arc)
            ConstraintSketcher().SetRadius(arc, arc.CRadius.Evaluate());
    }

    public void SetCoincidentXY(int curve, int point, double x, double y)
    {
        LockDerivedCircleRadius(curve, point);
        ConstraintSketcher().SetCoincident(
            ConstraintPoint(curve, point), CVec2D.Constant(x, y));
    }

    CVec2D NamedConstraintPoint(string name) => ConstraintSketcher().GetConstraintPoint(name);

    public void SetCoincidentNamed(string pointA, string pointB)
    {
        SetCoincidentNames(pointA, pointB);
    }

    public void SetCoincidentNames(string pointA, string pointB)
    {
        ConstraintSketcher().SetCoincident(NamedConstraintPoint(pointA), NamedConstraintPoint(pointB));
    }

    public void SetCoincidentNamed(int curve, int point, string otherName)
    {
        SetCoincidentHandleName(curve, point, otherName);
    }

    public void SetCoincidentHandleName(int curve, int point, string otherName)
    {
        LockDerivedCircleRadius(curve, point);
        ConstraintSketcher().SetCoincident(ConstraintPoint(curve, point), NamedConstraintPoint(otherName));
    }

    public void SetDistanceNamed(string pointA, string pointB, double distance)
    {
        SetDistanceNames(pointA, pointB, distance);
    }

    public void SetDistanceNames(string pointA, string pointB, double distance)
    {
        ConstraintSketcher().SetDistance(NamedConstraintPoint(pointA), NamedConstraintPoint(pointB), distance);
    }

    public void SetDistanceNamed(int curve, int point, string otherName, double distance)
    {
        SetDistanceHandleName(curve, point, otherName, distance);
    }

    public void SetDistanceHandleName(int curve, int point, string otherName, double distance)
    {
        ConstraintSketcher().SetDistance(ConstraintPoint(curve, point), NamedConstraintPoint(otherName), distance);
    }

    public void FixPointNamed(string name)
    {
        ConstraintSketcher().FixPoint(NamedConstraintPoint(name));
    }

    public void SetMidpointNamed(string pointName, int lineCurve)
    {
        ConstraintSketcher().SetMidpoint(NamedConstraintPoint(pointName), ConstraintCurve<CLine2D>(lineCurve));
    }

    public void SetPointOnLineNamed(string pointName, int lineCurve)
    {
        ConstraintSketcher().SetPointOnLine(NamedConstraintPoint(pointName), ConstraintCurve<CLine2D>(lineCurve));
    }

    public void SetPointOnSketchAxisNamed(string pointName, int axis)
    {
        ConstrainedSketcher sketch = ConstraintSketcher();
        CLine2D axisLine = axis == 0 ? sketch.OriginUnitX : sketch.OriginUnitY;
        sketch.SetPointOnLine(NamedConstraintPoint(pointName), axisLine);
    }

    public void SetPointOnCircleNamed(string pointName, int circleCurve)
    {
        ConstraintSketcher().SetPointOnCircle(
            NamedConstraintPoint(pointName), ConstraintCurve<CCircle2D>(circleCurve));
    }

    public void SetDistancePointLineNamed(string pointName, int lineCurve, double distance)
    {
        ConstraintSketcher().SetDistancePointLine(
            NamedConstraintPoint(pointName), ConstraintCurve<CLine2D>(lineCurve), distance);
    }

    public void SetTangentCircles(int curveA, int curveB)
    {
        ConstraintSketcher().SetTangentCircles(
            ConstraintCurve<ICircular2D>(curveA), ConstraintCurve<ICircular2D>(curveB));
    }

    public void SetVerticalDistanceNames(string pointA, string pointB, double distance)
    {
        ConstraintSketcher().SetVerticalDistancePointPoint(
            NamedConstraintPoint(pointA), NamedConstraintPoint(pointB), distance);
    }

    public void SetHorizontalDistanceNames(string pointA, string pointB, double distance)
    {
        ConstraintSketcher().SetHorizontalDistancePointPoint(
            NamedConstraintPoint(pointA), NamedConstraintPoint(pointB), distance);
    }

    public void SetCoincidentNameXY(string name, double x, double y)
    {
        ConstraintSketcher().SetCoincident(NamedConstraintPoint(name), CVec2D.Constant(x, y));
    }

    public void SetDistanceNameXY(string name, double x, double y, double distance)
    {
        ConstraintSketcher().SetDistance(NamedConstraintPoint(name), CVec2D.Constant(x, y), distance);
    }

    public void SetCoincidentOnCurveNamed(string pointName, int targetCurve, double uniform)
    {
        Curve2D target = ConstraintCurve<Curve2D>(targetCurve);
        IEvaluateCPoint evaluatable = target as IEvaluateCPoint;
        if (evaluatable == null)
            throw new System.ArgumentException(
                $"Sketch curve {targetCurve} has no parameterized point.");
        LockCircularRadius(targetCurve);
        ConstraintSketcher().SetCoincident(NamedConstraintPoint(pointName), evaluatable.Evaluate(uniform));
    }

    public void EvaluateConstraintPoint(string name, out double x, out double y, out int isConstant)
    {
        CVec2D point = NamedConstraintPoint(name);
        Vec2D uv = point.Evaluate();
        x = uv.X;
        y = uv.Y;
        isConstant = ConstrainedSketcher.IsConstantPoint(point) ? 1 : 0;
    }

    /// <summary>DotWrap-friendly XY accessors; <c>out</c> triples are not callable from Python.</summary>
    public double EvaluateConstraintPointX(string name) => EvaluateSketchPoint(name).X;

    public double EvaluateConstraintPointY(string name) => EvaluateSketchPoint(name).Y;

    Vec2D EvaluateSketchPoint(string name)
    {
        if (Native is ConstrainedSketcher)
            return NamedConstraintPoint(name).Evaluate();
        if (Native.TryGetPointOnEdge(name,out var point))
            return point;
        throw new ArgumentException($"Sketch point '{name}' was not found.");
    }

    public void SetParallel(int curveA, int curveB) =>
        ConstraintSketcher().SetParallel(
            ConstraintCurve<CLine2D>(curveA), ConstraintCurve<CLine2D>(curveB));

    public void SetPerpendicular(int curveA, int curveB) =>
        ConstraintSketcher().SetPerpendicular(
            ConstraintCurve<CLine2D>(curveA), ConstraintCurve<CLine2D>(curveB));

    public void SetTangent(int lineCurve, int circularCurve) =>
        ConstraintSketcher().SetTangent(
            ConstraintCurve<CLine2D>(lineCurve), ConstraintCurve<ICircular2D>(circularCurve));

    public void SetEqual(int curveA, int curveB)
    {
        var sketch = ConstraintSketcher();
        Curve2D a = ConstraintCurve<Curve2D>(curveA);
        Curve2D b = ConstraintCurve<Curve2D>(curveB);
        if (a is CLine2D && b is CLine2D)
        {
            sketch.SetEqualLength(
                ConstraintCurve<CLine2D>(curveA), ConstraintCurve<CLine2D>(curveB));
            return;
        }
        sketch.SetEqualRadius(
            ConstraintCurve<ICircular2D>(curveA), ConstraintCurve<ICircular2D>(curveB));
    }

    public void SetMidpoint(int pointCurve, int point, int lineCurve) =>
        ConstraintSketcher().SetMidpoint(
            ConstraintPoint(pointCurve, point), ConstraintCurve<CLine2D>(lineCurve));

    public void SetConcentric(int curveA, int curveB) =>
        ConstraintSketcher().SetConcentric(
            ConstraintCurve<ICircular2D>(curveA), ConstraintCurve<ICircular2D>(curveB));

    public void FixPoint(int curve, int point) =>
        ConstraintSketcher().FixPoint(ConstraintPoint(curve, point));

    public void SetLength(int curve, double length) =>
        ConstraintSketcher().SetLength(ConstraintCurve<CLine2D>(curve), length);

    public void SetRadius(int curve, double radius)
    {
        if (ConstraintCurve<Curve2D>(curve) is Ellipse2D)
            throw new ArgumentException("An ellipse has two semiaxes; dimension distances from its center to its quarter points.");
        ConstraintSketcher().SetRadius(ConstraintCurve<ICircular2D>(curve), radius);
    }

    public void SetDistance(
        int curveA, int pointA, int curveB, int pointB, double distance) =>
        ConstraintSketcher().SetDistance(
            ConstraintPoint(curveA, pointA), ConstraintPoint(curveB, pointB), distance);

    public void SetDistanceXY(int curve, int point, double x, double y, double distance) =>
        ConstraintSketcher().SetDistance(
            ConstraintPoint(curve, point), CVec2D.Constant(x, y), distance);

    public void SetAngleDegrees(int curveA, int curveB, double angleDegrees) =>
        ConstraintSketcher().SetAngleDegrees(
            ConstraintCurve<CLine2D>(curveA), ConstraintCurve<CLine2D>(curveB), angleDegrees);

    public void SolveConstraints()
    {
        var result = ConstraintSketcher().SolveConstraintsDetailed();
        if (!result.Converged)
            throw new InvalidOperationException(
                $"Sketch constraint solve failed: {result.Message ?? "did not converge"}. " +
                $"Residual sum of squares: {result.SumOfSquaredErrors:R}; " +
                $"{result.NumEquations} equations, {result.NumParameters} parameters.");
    }

    /// <summary>
    /// Interactive handle drag. <paramref name="point"/> matches sketch overlay roles:
    /// line 0/1 ends, 3 midpoint; circle 2 center, 3–6 cardinals; arc 0/1 ends, 3 mid.
    /// Returns 1 if the solver accepted the drag.
    /// </summary>
    public int TryDragSketchPoint(int curve, int point, double x, double y)
    {
        try
        {
            ConstrainedSketcher sketch = ConstraintSketcher();
            Vec2D target = new Vec2D(x, y);
            Curve2D geom = ConstraintCurve<Curve2D>(curve);
            bool ok = false;
            if (geom is CLine2D line)
            {
                if (point == 3)
                {
                    Vec2D mid = (line.CStart.Evaluate() + line.CEnd.Evaluate()) * 0.5;
                    ok = sketch.TryTranslatePoints(target - mid, line.CStart, line.CEnd);
                }
                else
                    ok = sketch.TryDragPoint(point == 0 ? line.CStart : line.CEnd, target);
            }
            else if (geom is CCircle2D circle)
            {
                if (point == 2)
                    ok = sketch.TryDragPoint(circle.CCenter, target);
                else
                {
                    double radius = (target - circle.CCenter.Evaluate()).Length();
                    ok = sketch.TryDragRadius(circle.CRadius, radius);
                }
            }
            else if (geom is CArc2D arc)
            {
                if (point == 2)
                    ok = sketch.TryDragPoint(arc.CCenter, target);
                else if (point == 3)
                {
                    double radius = (target - arc.CCenter.Evaluate()).Length();
                    ok = sketch.TryDragRadius(arc.CRadius, radius);
                }
                else
                    ok = sketch.TryDragArcEnd(arc, point == 0, target);
            }
            return ok ? 1 : 0;
        }
        catch
        {
            return 0;
        }
    }

    public string DumpCurves2D()
    {
        var sb = new System.Text.StringBuilder();
        var seen = new HashSet<Curve2D>();
        var all = Native.GetAllCurves();
        if (all != null)
        {
            for (int i = 0; i < all.Count; i++)
            {
                DumpOneCurve(sb, all[i]);
                seen.Add(all[i]);
            }
        }
        for (int i = 0; i < _constraintCurves.Count; i++)
        {
            if (!seen.Contains(_constraintCurves[i]))
                DumpOneCurve(sb, _constraintCurves[i]);
        }
        if (sb.Length == 0)
            return "\n";
        return sb.ToString();
    }

    static void DumpOneCurve(System.Text.StringBuilder sb, Curve2D curve)
    {
        if (curve == null)
            return;
        string name = curve.Name ?? "";
        if (curve is Line2D line)
        {
            Vec2D a = line.StartPosition;
            Vec2D b = line.EndPosition;
            sb.Append(SketchCurveDump.Line(name, a.X, a.Y, b.X, b.Y));
            sb.Append('\n');
            return;
        }
        if (curve is Circle2D circle)
        {
            sb.Append(SketchCurveDump.Circle(name, circle.Center.X, circle.Center.Y, circle.Radius));
            sb.Append('\n');
            return;
        }
        if (curve is Arc2D arc)
        {
            Vec2D a = arc.StartPosition;
            Vec2D m = arc.EvaluateVertex(0.5).Position;
            Vec2D b = arc.EndPosition;
            sb.Append(SketchCurveDump.Arc(name, a.X, a.Y, m.X, m.Y, b.X, b.Y));
            sb.Append('\n');
            return;
        }
        if (curve is SampledCurve sampled)
        {
            DumpPolyline(sb, name, sampled.Points);
            return;
        }
        List<CurveVertex2D> verts = curve.Tessellate(1e-3);
        if (verts == null || verts.Count < 2)
            return;
        var points = new List<Vec2D>(verts.Count);
        for (int i = 0; i < verts.Count; i++)
            points.Add(verts[i].Position);
        DumpPolyline(sb, name, points);
    }

    static void DumpPolyline(System.Text.StringBuilder sb, string name, IReadOnlyList<Vec2D> points)
    {
        if (points == null || points.Count < 2)
            return;
        for (int i = 1; i < points.Count; i++)
        {
            sb.Append(SketchCurveDump.Line(name, points[i - 1].X, points[i - 1].Y, points[i].X, points[i].Y));
            sb.Append('\n');
        }
    }

    public void AddNaca4DigitAirfoil(
        int nacaCode,
        double leadingEdgeX,
        double leadingEdgeY,
        double chordLength,
        double chordAngleRadians,
        int samplesPerSide,
        int analyticEndTangents,
        double trailingEdgeTrimChordFraction)
    {
        Native.AddNaca4DigitAirfoil(
            nacaCode,
            new Vec2D(leadingEdgeX, leadingEdgeY),
            chordLength,
            chordAngleRadians,
            samplesPerSide,
            analyticEndTangents != 0,
            CurveFlags.None,
            trailingEdgeTrimChordFraction);
    }

    public void AddNaca4DigitAirfoilLabel(
        string nacaLabel,
        double leadingEdgeX,
        double leadingEdgeY,
        double chordLength,
        double chordAngleRadians,
        int samplesPerSide,
        int analyticEndTangents,
        double trailingEdgeTrimChordFraction)
    {
        Native.AddNaca4DigitAirfoil(
            nacaLabel,
            new Vec2D(leadingEdgeX, leadingEdgeY),
            chordLength,
            chordAngleRadians,
            samplesPerSide,
            analyticEndTangents != 0,
            CurveFlags.None,
            trailingEdgeTrimChordFraction);
    }
}

[DotWrapExpose]
public class NativeSolid
{
    internal readonly AnchorMesh Native;

    internal NativeSolid(AnchorMesh native)
    {
        Native = native;
    }

    public string Name { get { return Native.Name; } }

    public int TriangleCount { get { return Native.Mesh.Triangles.Count; } }

    public int IsVolume { get { return Native.IsVolume ? 1 : 0; } }

    public int EdgeCount
    {
        get { Native.EnsureCoplanarPostProcessed(); return Native.GroupEdges.Count; }
    }

    public string EdgeNameAt(int index)
    {
        return Native.GetEdgeReference(index);
    }

    public string DumpDisplay()
    {
        return DisplayPack.PackSolid(Native);
    }

    public double SignedVolume()
    {
        Native.EnsureCoplanarPostProcessed();
        return GeoCore.MeshAnalysis.ComputeSignedMeshVolume(Native.Mesh.Positions, Native.Mesh.Triangles);
    }

    public int IsWatertight()
    {
        Native.EnsureCoplanarPostProcessed();
        var mesh = Native.Mesh;
        bool watertight = mesh.PrecisionPositions != null && mesh.PrecisionPositions.Count == mesh.Positions.Count
            ? GeoCore.MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions, mesh.Triangles)
            : GeoCore.MeshAnalysis.IsWatertightMesh(mesh.Positions, mesh.Triangles);
        return watertight ? 1 : 0;
    }

    public string PackMesh()
    {
        Native.EnsureCoplanarPostProcessed();
        return NativePack.WriteIndexed3(Native.Mesh.Positions, Native.Mesh.Triangles);
    }
}

[DotWrapExpose]
public class NativeProjectedSketch
{
    internal readonly ProjectedSketch Native;

    internal NativeProjectedSketch(ProjectedSketch native)
    {
        Native = native;
    }

    public string Name { get { return Native.Name; } }

    public int CurveCount
    {
        get { return Native.Curves == null ? 0 : Native.Curves.Count; }
    }

    public string CurveNameAt(int index)
    {
        if (Native.Curves == null || index < 0 || index >= Native.Curves.Count)
            throw new System.ArgumentOutOfRangeException(nameof(index));
        return Native.Curves[index].Name;
    }
}
