using Curves;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class OffsetSketchIdentityTests : IDisposable
{
    const double Offset = 0.5;
    const double GeomTol = 0.08;

    public void Dispose() => GeoAPI.Clear();

    static SketchStripOffsetOptions MiterOpts() => new SketchStripOffsetOptions
    {
        JoinType = SketchOffsetJoinType.Miter,
        EndCap = SketchOffsetEndCap.Square,
        TessellationTolerance = 0.01,
        CornerArcTolerance = 0.01,
        ConnectionTolerance = 1e-6,
    };

    static Line2D NamedLine(PlotterSketcher sketch, string name, Vec2D a, Vec2D b)
    {
        var line = sketch.AddLine(a, b);
        line.Name = name;
        return line;
    }

    static OffsetSampledCurve2D RequireNamed(IEnumerable<OffsetSampledCurve2D> pieces, string name)
    {
        var hit = pieces.FirstOrDefault(p => p.Name == name);
        Assert.True(hit != null, "Missing offset piece " + name +
            ". Have: " + string.Join(", ", pieces.Select(p => p.Name).Where(n => !string.IsNullOrEmpty(n))));
        return hit;
    }

    static double PieceLength(OffsetSampledCurve2D piece)
    {
        var pts = piece.ToReferencePoints();
        double len = 0;
        for (int i = 1; i < pts.Count; i++)
            len += (pts[i] - pts[i - 1]).Length();
        return len;
    }

    static Vec2D PieceMid(OffsetSampledCurve2D piece)
        => piece.EvaluateVertex(0.5).Position;

    static double DistPointToSegment(Vec2D p, Vec2D a, Vec2D b)
    {
        Vec2D ab = b - a;
        double lenSq = ab.LengthSquared();
        if (lenSq < 1e-24)
            return (p - a).Length();
        double t = Vec2DOps.Dot(p - a, ab) / lenSq;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        return (p - (a + t * ab)).Length();
    }

    static bool CollinearOverlap(Vec2D a0, Vec2D a1, Vec2D b0, Vec2D b1, double tol)
    {
        Vec2D da = a1 - a0;
        Vec2D db = b1 - b0;
        double la = da.Length();
        double lb = db.Length();
        if (la < tol || lb < tol)
            return false;
        Vec2D ua = da * (1.0 / la);
        double cross = ua.X * db.Y - ua.Y * db.X;
        if (Math.Abs(cross) > tol * lb)
            return false;
        double off = ua.X * (b0.Y - a0.Y) - ua.Y * (b0.X - a0.X);
        if (Math.Abs(off) > tol)
            return false;

        double pa0 = 0;
        double pa1 = la;
        double pb0 = Vec2DOps.Dot(b0 - a0, ua);
        double pb1 = Vec2DOps.Dot(b1 - a0, ua);
        double lo = Math.Max(Math.Min(pa0, pa1), Math.Min(pb0, pb1));
        double hi = Math.Min(Math.Max(pa0, pa1), Math.Max(pb0, pb1));
        return hi - lo > 2.0 * tol;
    }

    static bool IsSourceSideOffsetName(string name)
    {
        if (string.IsNullOrEmpty(name) || name.StartsWith("cap[", StringComparison.Ordinal))
            return false;
        return name.Contains("@in_offset") || name.Contains("@out_offset");
    }

    static void AssertNoCollinearOverlap(IReadOnlyList<OffsetSampledCurve2D> pieces)
    {
        var named = pieces.Where(p => IsSourceSideOffsetName(p.Name)).ToList();
        for (int i = 0; i < named.Count; i++)
        {
            var a = named[i].ToReferencePoints();
            for (int j = i + 1; j < named.Count; j++)
            {
                if (named[i].SourceIndex == named[j].SourceIndex && named[i].Side == named[j].Side)
                    continue;
                var b = named[j].ToReferencePoints();
                for (int ia = 0; ia < a.Count - 1; ia++)
                {
                    for (int ib = 0; ib < b.Count - 1; ib++)
                    {
                        Assert.False(
                            CollinearOverlap(a[ia], a[ia + 1], b[ib], b[ib + 1], 1e-4),
                            named[i].Name + " overlaps " + named[j].Name +
                            " (two sources must not share a 2D edge).");
                    }
                }
            }
        }
    }

    static void AssertInAndOutForEachSource(
        IReadOnlyList<Curve2D> sources,
        IReadOnlyList<OffsetSampledCurve2D> pieces,
        double offset)
    {
        foreach (var source in sources)
        {
            string inn = EntityNaming.FormatSketchOffsetCurve(source.Name, false);
            string outn = EntityNaming.FormatSketchOffsetCurve(source.Name, true);
            var inPiece = RequireNamed(pieces, inn);
            var outPiece = RequireNamed(pieces, outn);

            double inDist = DistPointToSegment(PieceMid(inPiece), source.StartPosition, source.EndPosition);
            double outDist = DistPointToSegment(PieceMid(outPiece), source.StartPosition, source.EndPosition);
            Assert.True(inDist >= offset * 0.7 && inDist <= offset * 1.3,
                source.Name + "@in_offset mid dist=" + inDist + " mid=" + PieceMid(inPiece) + "\n" + DumpPieces(pieces));
            Assert.True(outDist >= offset * 0.7 && outDist <= offset * 1.3,
                source.Name + "@out_offset mid dist=" + outDist + " mid=" + PieceMid(outPiece) + "\n" + DumpPieces(pieces));

            double srcLen = (source.EndPosition - source.StartPosition).Length();
            Assert.True(PieceLength(inPiece) > 0.5 * srcLen,
                source.Name + "@in_offset too short: " + PieceLength(inPiece));
            Assert.True(PieceLength(outPiece) > 0.5 * srcLen,
                source.Name + "@out_offset too short: " + PieceLength(outPiece));
        }
    }

    [Fact]
    public void SampledCurve_EvaluateVertexHalfIsAlongPolyline()
    {
        var pts = new List<Vec2D>
        {
            new Vec2D(-0.5, -0.5), new Vec2D(0, -0.5), new Vec2D(4, -0.5), new Vec2D(4.5, -0.5),
        };
        var nrm = pts.Select(_ => new Vec2D(0, -1)).ToList();
        var c = new SampledCurve(pts, nrm);
        Vec2D mid = c.EvaluateVertex(0.5).Position;
        Assert.InRange(mid.X, 0.0, 4.0);
        Assert.InRange(mid.Y, -0.51, -0.49);
    }

    static SketchStripOffsetOptions OutlineOpts() => new SketchStripOffsetOptions
    {
        JoinType = SketchOffsetJoinType.Miter,
        OpenMode = SketchOffsetOpenMode.Outline,
        EndCap = SketchOffsetEndCap.Square,
        TessellationTolerance = 0.01,
        CornerArcTolerance = 0.01,
        ConnectionTolerance = 1e-6,
    };

    static SketchStripOffsetOptions RoundOutlineOpts() => new SketchStripOffsetOptions
    {
        JoinType = SketchOffsetJoinType.Round,
        OpenMode = SketchOffsetOpenMode.Outline,
        EndCap = SketchOffsetEndCap.Round,
        TessellationTolerance = 0.01,
        CornerArcTolerance = 0.01,
        ConnectionTolerance = 1e-6,
    };

    static SketchStripOffsetOptions ParallelOpts() => new SketchStripOffsetOptions
    {
        JoinType = SketchOffsetJoinType.Miter,
        OpenMode = SketchOffsetOpenMode.Parallel,
        TessellationTolerance = 0.01,
        CornerArcTolerance = 0.01,
        ConnectionTolerance = 1e-6,
    };

    [Fact]
    public void ClosedSquare_EachSideHasDistinctInAndOut()
    {
        var sketch = new PlotterSketcher("sq");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        var north = NamedLine(sketch, "north", new Vec2D(4, 4), new Vec2D(0, 4));
        var west = NamedLine(sketch, "west", new Vec2D(0, 4), Vec2DOps.Zero);
        var sources = new List<Curve2D> { south, east, north, west };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);

        Assert.True(PieceMid(RequireNamed(pieces, "south@out_offset")).Y < -0.2);
        Assert.True(PieceMid(RequireNamed(pieces, "south@in_offset")).Y > 0.2);
        Assert.True(PieceMid(RequireNamed(pieces, "east@out_offset")).X > 4.2);
        Assert.True(PieceMid(RequireNamed(pieces, "east@in_offset")).X < 3.8);
        Assert.True(PieceMid(RequireNamed(pieces, "west@out_offset")).X < -0.2);
        Assert.True(PieceMid(RequireNamed(pieces, "west@in_offset")).X > 0.2);
    }

    [Fact]
    public void OffsetStrip_ClosedSquare_SplitsNamedSides()
    {
        var sketch = new PlotterSketcher("sq-strip");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        var north = NamedLine(sketch, "north", new Vec2D(4, 4), new Vec2D(0, 4));
        var west = NamedLine(sketch, "west", new Vec2D(0, 4), Vec2DOps.Zero);
        var sources = new List<Curve2D> { south, east, north, west };

        var pieces = sketch.OffsetStrip(sources, Offset, MiterOpts()).CreateSampledCurves();
        foreach (var source in sources)
        {
            RequireNamed(pieces, EntityNaming.FormatSketchOffsetCurve(source.Name, true));
            Assert.DoesNotContain(pieces, p => p.Name == EntityNaming.FormatSketchOffsetCurve(source.Name, false));
        }
        AssertNamedPiecesAreStraight(pieces, 35.0);
        Assert.True(PieceMid(RequireNamed(pieces, "south@out_offset")).Y < -0.2);
        Assert.True(PieceMid(RequireNamed(pieces, "east@out_offset")).X > 4.2);
        Assert.True(pieces.Count >= 4, "Closed polygon offset must split per source, not one loop curve.");
    }

    [Fact]
    public void OffsetStrip_OpenL_Outline_SplitsAndNamesEndCaps()
    {
        var sketch = new PlotterSketcher("ell-strip");
        var h = NamedLine(sketch, "h", Vec2DOps.Zero, new Vec2D(4, 0));
        var v = NamedLine(sketch, "v", new Vec2D(4, 0), new Vec2D(4, 3));
        var sources = new List<Curve2D> { h, v };

        var pieces = sketch.OffsetStrip(sources, Offset, OutlineOpts()).CreateSampledCurves();
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);

        var startCap = RequireNamed(pieces, "h@start_cap");
        var endCap = RequireNamed(pieces, "v@end_cap");
        AssertCapJoinsInAndOut(pieces, startCap, "h");
        AssertCapJoinsInAndOut(pieces, endCap, "v");
        Assert.True(DistPointToSegment(PieceMid(startCap), Vec2DOps.Zero, Vec2DOps.Zero) < Offset * 2.2);
        Assert.True(DistPointToSegment(PieceMid(endCap), new Vec2D(4, 3), new Vec2D(4, 3)) < Offset * 2.2);
        AssertAllPiecesNamed(pieces);
    }

    [Fact]
    public void OffsetStrip_OpenL_RoundOutline_NamesOuterCornerJoinCap()
    {
        var sketch = new PlotterSketcher("ell-round");
        var h = NamedLine(sketch, "h", Vec2DOps.Zero, new Vec2D(4, 0));
        var v = NamedLine(sketch, "v", new Vec2D(4, 0), new Vec2D(4, 3));
        var sources = new List<Curve2D> { h, v };

        var pieces = sketch.OffsetStrip(sources, Offset, RoundOutlineOpts()).CreateSampledCurves();
        AssertInAndOutForEachSource(sources, pieces, Offset);
        var outerJoin = RequireNamed(pieces, "cap[h@out_offset,v@out_offset]");
        Assert.True(SharesEndpoint(outerJoin, RequireNamed(pieces, "h@out_offset")));
        Assert.True(SharesEndpoint(outerJoin, RequireNamed(pieces, "v@out_offset")));
        Assert.True(PieceMid(outerJoin).X > 4.0 && PieceMid(outerJoin).Y < 0.0,
            "Outer round join should sit at the convex corner. mid=" + PieceMid(outerJoin));
        AssertAllPiecesNamed(pieces);
    }

    [Fact]
    public void OffsetStrip_OpenL_Parallel_SplitsPerSource()
    {
        var sketch = new PlotterSketcher("ell-par");
        var h = NamedLine(sketch, "h", Vec2DOps.Zero, new Vec2D(4, 0));
        var v = NamedLine(sketch, "v", new Vec2D(4, 0), new Vec2D(4, 3));

        var pieces = sketch.OffsetStrip(new List<Curve2D> { h, v }, Offset, ParallelOpts()).CreateSampledCurves();
        var hPiece = RequireNamedSide(pieces, "h");
        var vPiece = RequireNamedSide(pieces, "v");
        Assert.True(PieceLength(hPiece) > 2.0, "h offset too short: " + PieceLength(hPiece));
        Assert.True(PieceLength(vPiece) > 1.5, "v offset too short: " + PieceLength(vPiece));
        Assert.True(PieceMid(hPiece).Y > 0.2, "parallel +X offset should sit above h");
        Assert.True(PieceMid(vPiece).X < 3.8, "parallel +Y offset should sit left of v");
        AssertNamedPiecesAreStraight(pieces, 35.0);
    }

    [Fact]
    public void OffsetNetwork_OpenLine_NamesStartAndEndCaps()
    {
        var sketch = new PlotterSketcher("seg");
        var seg = NamedLine(sketch, "seg", Vec2DOps.Zero, new Vec2D(10, 0));
        var pieces = sketch.OffsetNetwork(new List<Curve2D> { seg }, Offset, OutlineOpts());

        AssertInAndOutForEachSource(new List<Curve2D> { seg }, pieces, Offset);
        var startCap = RequireNamed(pieces, "seg@start_cap");
        var endCap = RequireNamed(pieces, "seg@end_cap");
        AssertCapJoinsInAndOut(pieces, startCap, "seg");
        AssertCapJoinsInAndOut(pieces, endCap, "seg");
        Assert.True(PieceMid(startCap).X < 1.0);
        Assert.True(PieceMid(endCap).X > 9.0);
    }

    [Fact]
    public void OffsetNetwork_Plus_NamesEndCapsFromArmEnds()
    {
        var sketch = new PlotterSketcher("plus");
        var right = NamedLine(sketch, "right", Vec2DOps.Zero, new Vec2D(4, 0));
        var up = NamedLine(sketch, "up", Vec2DOps.Zero, new Vec2D(0, 3));
        var left = NamedLine(sketch, "left", Vec2DOps.Zero, new Vec2D(-4, 0));
        var down = NamedLine(sketch, "down", Vec2DOps.Zero, new Vec2D(0, -3));
        var sources = new List<Curve2D> { right, up, left, down };

        var pieces = sketch.OffsetNetwork(sources, Offset, OutlineOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        foreach (var source in sources)
        {
            var cap = RequireNamed(pieces, source.Name + "@end_cap");
            AssertCapJoinsInAndOut(pieces, cap, source.Name);
            Assert.True(
                DistPointToSegment(PieceMid(cap), source.EndPosition, source.EndPosition) < Offset * 2.2,
                source.Name + "@end_cap should sit at the free arm end. mid=" + PieceMid(cap));
        }
        AssertAllPiecesNamed(pieces);
    }

    [Fact]
    public void OffsetStrip_OpenL_Outline_MatchesNetworkNames()
    {
        var sketch = new PlotterSketcher("ell-both");
        var h = NamedLine(sketch, "h", Vec2DOps.Zero, new Vec2D(4, 0));
        var v = NamedLine(sketch, "v", new Vec2D(4, 0), new Vec2D(4, 3));
        var sources = new List<Curve2D> { h, v };

        var stripNames = NamedSet(sketch.OffsetStrip(sources, Offset, OutlineOpts()).CreateSampledCurves());
        var netNames = NamedSet(sketch.OffsetNetwork(sources, Offset, OutlineOpts()));
        foreach (string name in new[]
                 {
                     "h@in_offset", "h@out_offset", "h@start_cap",
                     "v@in_offset", "v@out_offset", "v@end_cap",
                 })
        {
            Assert.Contains(name, stripNames);
            Assert.Contains(name, netNames);
        }
    }

    static HashSet<string> NamedSet(IEnumerable<OffsetSampledCurve2D> pieces)
        => new HashSet<string>(pieces.Select(p => p.Name).Where(n => !string.IsNullOrEmpty(n)));

    static OffsetSampledCurve2D RequireNamedSide(IEnumerable<OffsetSampledCurve2D> pieces, string sourceName)
    {
        var hit = pieces.FirstOrDefault(p =>
            p.Name == EntityNaming.FormatSketchOffsetCurve(sourceName, true) ||
            p.Name == EntityNaming.FormatSketchOffsetCurve(sourceName, false));
        Assert.True(hit != null, "Missing in/out offset piece for " + sourceName +
            ". Have: " + string.Join(", ", pieces.Select(p => p.Name).Where(n => !string.IsNullOrEmpty(n))));
        return hit;
    }

    static void AssertCapJoinsInAndOut(
        IReadOnlyList<OffsetSampledCurve2D> pieces,
        OffsetSampledCurve2D cap,
        string sourceName)
    {
        var inn = RequireNamed(pieces, EntityNaming.FormatSketchOffsetCurve(sourceName, false));
        var outt = RequireNamed(pieces, EntityNaming.FormatSketchOffsetCurve(sourceName, true));
        Assert.True(SharesEndpoint(cap, inn) && SharesEndpoint(cap, outt),
            cap.Name + " must meet " + sourceName + "@in_offset and @out_offset. " +
            "cap=" + Ends(cap) + " in=" + Ends(inn) + " out=" + Ends(outt));
    }

    static bool SharesEndpoint(OffsetSampledCurve2D a, OffsetSampledCurve2D b)
    {
        var ap = a.ToReferencePoints();
        var bp = b.ToReferencePoints();
        foreach (var p in new[] { ap[0], ap[^1] })
        {
            foreach (var q in new[] { bp[0], bp[^1] })
            {
                if ((p - q).LengthSquared() <= 1e-8)
                    return true;
            }
        }
        return false;
    }

    static void AssertAllPiecesNamed(IReadOnlyList<OffsetSampledCurve2D> pieces)
    {
        var unnamed = pieces.Where(p => string.IsNullOrEmpty(p.Name)).ToList();
        Assert.True(unnamed.Count == 0,
            unnamed.Count + " offset piece(s) have no name:\n" + DumpPieces(pieces));
    }

    static string Ends(OffsetSampledCurve2D piece)
    {
        var pts = piece.ToReferencePoints();
        return pts[0] + ".." + pts[^1];
    }

    [Fact]
    public void LPolyline_VerticalSourceKeepsVerticalOffset()
    {
        var sketch = new PlotterSketcher("ell");
        var h = NamedLine(sketch, "h", Vec2DOps.Zero, new Vec2D(4, 0));
        var v = NamedLine(sketch, "v", new Vec2D(4, 0), new Vec2D(4, 3));
        var sources = new List<Curve2D> { h, v };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);

        var vOut = RequireNamed(pieces, "v@out_offset").ToReferencePoints();
        var vIn = RequireNamed(pieces, "v@in_offset").ToReferencePoints();
        AssertVertical(vOut, "v@out_offset");
        AssertVertical(vIn, "v@in_offset");
    }

    [Fact]
    public void OpenPolylineChain_NoSharedEdgesBetweenSources()
    {
        var sketch = new PlotterSketcher("u");
        var a = NamedLine(sketch, "a", Vec2DOps.Zero, new Vec2D(0, 3));
        var b = NamedLine(sketch, "b", new Vec2D(0, 3), new Vec2D(4, 3));
        var c = NamedLine(sketch, "c", new Vec2D(4, 3), new Vec2D(4, 0));
        var sources = new List<Curve2D> { a, b, c };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);
        AssertVertical(RequireNamed(pieces, "a@out_offset").ToReferencePoints(), "a@out_offset");
        AssertVertical(RequireNamed(pieces, "c@out_offset").ToReferencePoints(), "c@out_offset");
    }

    [Fact]
    public void EndpointTree_ThreeBranchesFromOneVertex()
    {
        var sketch = new PlotterSketcher("tree");
        var left = NamedLine(sketch, "left", new Vec2D(-4, 0), Vec2DOps.Zero);
        var right = NamedLine(sketch, "right", Vec2DOps.Zero, new Vec2D(4, 0));
        var up = NamedLine(sketch, "up", Vec2DOps.Zero, new Vec2D(0, 3));
        var sources = new List<Curve2D> { left, right, up };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);
        AssertVertical(RequireNamed(pieces, "up@out_offset").ToReferencePoints(), "up@out_offset");
        AssertVertical(RequireNamed(pieces, "up@in_offset").ToReferencePoints(), "up@in_offset");
    }

    [Fact]
    public void CollinearPolyline_JointSplitsFusedClipperEdge()
    {
        var sketch = new PlotterSketcher("col");
        var a = NamedLine(sketch, "a", Vec2DOps.Zero, new Vec2D(4, 0));
        var b = NamedLine(sketch, "b", new Vec2D(4, 0), new Vec2D(8, 0));
        var sources = new List<Curve2D> { a, b };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);

        Assert.True(PieceMid(RequireNamed(pieces, "a@out_offset")).X < 4.0);
        Assert.True(PieceMid(RequireNamed(pieces, "b@out_offset")).X > 4.0);
    }

    [Fact]
    public void ClosedL_EachNamedEdgeKeepsOwnSides()
    {
        var sketch = new PlotterSketcher("ellclosed");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(6, 0));
        var east = NamedLine(sketch, "east", new Vec2D(6, 0), new Vec2D(6, 3));
        var notchN = NamedLine(sketch, "notchN", new Vec2D(6, 3), new Vec2D(3, 3));
        var notchE = NamedLine(sketch, "notchE", new Vec2D(3, 3), new Vec2D(3, 6));
        var north = NamedLine(sketch, "north", new Vec2D(3, 6), new Vec2D(0, 6));
        var west = NamedLine(sketch, "west", new Vec2D(0, 6), Vec2DOps.Zero);
        var sources = new List<Curve2D> { south, east, notchN, notchE, north, west };

        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());
        AssertInAndOutForEachSource(sources, pieces, Offset);
        AssertNoCollinearOverlap(pieces);
        AssertNamedPiecesAreStraight(pieces, 35.0);
        AssertVertical(RequireNamed(pieces, "west@out_offset").ToReferencePoints(), "west@out_offset");
        AssertVertical(RequireNamed(pieces, "east@out_offset").ToReferencePoints(), "east@out_offset");
        AssertVertical(RequireNamed(pieces, "notchE@out_offset").ToReferencePoints(), "notchE@out_offset");
    }

    [Fact]
    public void Extrude_NamedSquare_SidePatchesSplitAtCorners()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), 0.01);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "sq");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        south.Flags = CurveFlags.HelperGeometry;
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        east.Flags = CurveFlags.HelperGeometry;
        var north = NamedLine(sketch, "north", new Vec2D(4, 4), new Vec2D(0, 4));
        north.Flags = CurveFlags.HelperGeometry;
        var west = NamedLine(sketch, "west", new Vec2D(0, 4), Vec2DOps.Zero);
        west.Flags = CurveFlags.HelperGeometry;

        sketch.OffsetNetwork(
            new List<Curve2D> { south, east, north, west },
            Offset,
            MiterOpts());

        var mesh = api.Extrude(sketch, 1.0, 0.01, "wall");
        var names = mesh.groupIdToExtendedName.Values.Distinct().ToList();
        Assert.Contains(names, n => n.Contains("east@out_offset"));
        Assert.Contains(names, n => n.Contains("east@in_offset"));
        Assert.Contains(names, n => n.Contains("south@out_offset"));
        Assert.Contains(names, n => n.Contains("west@out_offset"));
        Assert.DoesNotContain(names, n => n.Contains("east@out_offset") && n.Contains("south@out_offset"));
    }

    static double MaxTurnDegrees(IReadOnlyList<Vec2D> pts)
    {
        double maxDeg = 0;
        for (int i = 1; i < pts.Count - 1; i++)
        {
            Vec2D a = pts[i] - pts[i - 1];
            Vec2D b = pts[i + 1] - pts[i];
            double la = a.Length();
            double lb = b.Length();
            if (la < 1e-12 || lb < 1e-12)
                continue;
            double cos = (a.X * b.X + a.Y * b.Y) / (la * lb);
            if (cos > 1) cos = 1;
            if (cos < -1) cos = -1;
            double deg = Math.Acos(cos) * 180.0 / Math.PI;
            if (deg > maxDeg)
                maxDeg = deg;
        }
        return maxDeg;
    }

    static string DumpPieces(IEnumerable<OffsetSampledCurve2D> pieces)
    {
        var sb = new System.Text.StringBuilder();
        foreach (var p in pieces)
        {
            var pts = p.ToReferencePoints();
            sb.Append(p.Name ?? "(unnamed)");
            sb.Append(" n=").Append(pts.Count);
            sb.Append(" turn=").Append(MaxTurnDegrees(pts).ToString("0.0"));
            sb.Append(" ").Append(pts[0]).Append(" -> ").Append(pts[^1]);
            sb.AppendLine();
        }
        return sb.ToString();
    }

    static void AssertNamedPiecesAreStraight(IReadOnlyList<OffsetSampledCurve2D> pieces, double maxTurnDeg)
    {
        var bent = pieces.Where(p =>
            IsSourceSideOffsetName(p.Name) &&
            MaxTurnDegrees(p.ToReferencePoints()) > maxTurnDeg).ToList();
        Assert.True(bent.Count == 0,
            "Named offset pieces must not wrap corners (one curve = one extrude patch, no vertical crease). Bent:\n" +
            DumpPieces(bent) + "\nAll:\n" + DumpPieces(pieces));
    }

    static List<Vec2D> VerticalGroupEdgeXys(AnchorMesh mesh, double minHeight)
    {
        mesh.EnsureCoplanarPostProcessed();
        var result = new List<Vec2D>();
        if (mesh.GroupEdges == null)
            return result;
        foreach (var edge in mesh.GroupEdges)
        {
            if (edge.LineStrips3D == null)
                continue;
            foreach (var strip in edge.LineStrips3D)
            {
                var pts = strip.Points;
                if (pts == null || pts.Count < 2)
                    continue;
                double minZ = pts[0].Z;
                double maxZ = pts[0].Z;
                bool vertical = true;
                for (int i = 0; i < pts.Count; i++)
                {
                    if (Math.Abs(pts[i].X - pts[0].X) > 1e-6 || Math.Abs(pts[i].Y - pts[0].Y) > 1e-6)
                    {
                        vertical = false;
                        break;
                    }
                    if (pts[i].Z < minZ) minZ = pts[i].Z;
                    if (pts[i].Z > maxZ) maxZ = pts[i].Z;
                }
                if (vertical && maxZ - minZ >= minHeight)
                    result.Add(new Vec2D(pts[0].X, pts[0].Y));
            }
        }
        return result;
    }

    static int CountVerticalGroupEdgesNear(AnchorMesh mesh, Vec2D xy, double tol)
    {
        int n = 0;
        foreach (var a in VerticalGroupEdgeXys(mesh, 0.1))
        {
            if ((a - xy).Length() <= tol)
                n++;
        }
        return n;
    }

    static void AssertVerticalNear(AnchorMesh mesh, Vec2D xy, double tol, string label)
    {
        int n = CountVerticalGroupEdgesNear(mesh, xy, tol);
        if (n >= 1)
            return;
        var all = VerticalGroupEdgeXys(mesh, 0.1);
            Assert.Fail(
            "Missing vertical crease near " + xy + " (" + label + "). Have " + all.Count +
            " verticals: " + string.Join(", ", all.Select(p => p.ToString())));
    }

    static List<Curve2D> HouseShellSources(PlotterSketcher sketch)
    {
        return new List<Curve2D>
        {
            NamedLine(sketch, "sw", Vec2DOps.Zero, new Vec2D(2, 0)),
            NamedLine(sketch, "bay_w", new Vec2D(2, 0), new Vec2D(2, -1.4)),
            NamedLine(sketch, "bay_s", new Vec2D(2, -1.4), new Vec2D(5.2, -1.4)),
            NamedLine(sketch, "bay_e", new Vec2D(5.2, -1.4), new Vec2D(5.2, 0)),
            NamedLine(sketch, "se", new Vec2D(5.2, 0), new Vec2D(13.2, 0)),
            NamedLine(sketch, "east", new Vec2D(13.2, 0), new Vec2D(13.2, 4.2)),
            NamedLine(sketch, "notch", new Vec2D(13.2, 4.2), new Vec2D(8, 4.2)),
            NamedLine(sketch, "stem", new Vec2D(8, 4.2), new Vec2D(8, 10.5)),
            NamedLine(sketch, "north", new Vec2D(8, 10.5), new Vec2D(0, 10.5)),
            NamedLine(sketch, "west", new Vec2D(0, 10.5), Vec2DOps.Zero),
        };
    }

    static void AssertClosedPieceChain(IReadOnlyList<OffsetSampledCurve2D> pieces)
    {
        var groups = pieces.GroupBy(p => p.LoopIndex).OrderBy(g => g.Key);
        foreach (var g in groups)
        {
            var loop = g.ToList();
            Assert.True(loop.Count >= 3, "Offset loop " + g.Key + " too short to be a closed wall.");
            for (int i = 0; i < loop.Count; i++)
            {
                var a = loop[i].ToReferencePoints();
                var b = loop[(i + 1) % loop.Count].ToReferencePoints();
                double d = (a[^1] - b[0]).Length();
                Assert.True(d < 1e-6,
                    "Loop " + g.Key + " gap between " + (loop[i].Name ?? "?") + " and " + (loop[(i + 1) % loop.Count].Name ?? "?") +
                    " d=" + d);
            }
        }
    }

    static void AssertPiecesDoNotWrap(IReadOnlyList<OffsetSampledCurve2D> pieces, double maxTurnDeg)
    {
        Assert.True(pieces.All(p => MaxTurnDegrees(p.ToReferencePoints()) <= maxTurnDeg),
            "Every offset piece (including unnamed joins) must split at corners so two 2D curves meet there:\n" +
            DumpPieces(pieces));
        AssertNamedPiecesAreStraight(pieces, maxTurnDeg);
        AssertClosedPieceChain(pieces);
    }

    [Fact]
    public void NamedPieces_DoNotWrapCorners_SquareLAndHouse()
    {
        const double maxTurn = 35.0;
        var square = new PlotterSketcher("sq2");
        var sqSrc = new List<Curve2D>
        {
            NamedLine(square, "south", Vec2DOps.Zero, new Vec2D(4, 0)),
            NamedLine(square, "east", new Vec2D(4, 0), new Vec2D(4, 4)),
            NamedLine(square, "north", new Vec2D(4, 4), new Vec2D(0, 4)),
            NamedLine(square, "west", new Vec2D(0, 4), Vec2DOps.Zero),
        };
        AssertPiecesDoNotWrap(square.OffsetNetwork(sqSrc, Offset, MiterOpts()), maxTurn);

        var ell = new PlotterSketcher("ell2");
        var ellSrc = new List<Curve2D>
        {
            NamedLine(ell, "h", Vec2DOps.Zero, new Vec2D(4, 0)),
            NamedLine(ell, "v", new Vec2D(4, 0), new Vec2D(4, 3)),
        };
        AssertPiecesDoNotWrap(ell.OffsetNetwork(ellSrc, Offset, MiterOpts()), maxTurn);

        var house = new PlotterSketcher("house");
        var houseSrc = HouseShellSources(house);
        var pieces = house.OffsetNetwork(houseSrc, 0.15, MiterOpts());
        AssertPiecesDoNotWrap(pieces, maxTurn);
        AssertInAndOutForEachSource(houseSrc, pieces, 0.15);
        AssertNoCollinearOverlap(pieces);
    }

    [Fact]
    public void Extrude_Square_VerticalCreaseAtEveryCorner()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), 0.01);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "sqv");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        south.Flags = CurveFlags.HelperGeometry;
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        east.Flags = CurveFlags.HelperGeometry;
        var north = NamedLine(sketch, "north", new Vec2D(4, 4), new Vec2D(0, 4));
        north.Flags = CurveFlags.HelperGeometry;
        var west = NamedLine(sketch, "west", new Vec2D(0, 4), Vec2DOps.Zero);
        west.Flags = CurveFlags.HelperGeometry;
        sketch.OffsetNetwork(new List<Curve2D> { south, east, north, west }, Offset, MiterOpts());
        var mesh = api.Extrude(sketch, 1.0, 0.01, "wall");

        var outer = new[]
        {
            new Vec2D(-0.5, -0.5), new Vec2D(4.5, -0.5), new Vec2D(4.5, 4.5), new Vec2D(-0.5, 4.5),
        };
        var inner = new[]
        {
            new Vec2D(0.5, 0.5), new Vec2D(3.5, 0.5), new Vec2D(3.5, 3.5), new Vec2D(0.5, 3.5),
        };
        foreach (var c in outer.Concat(inner))
            AssertVerticalNear(mesh, c, 0.08, "square miter");
    }

    [Fact]
    public void Extrude_HouseShell_VerticalCreaseAtSourceJoints()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-6, -6, -2), new Vec3D(18, 16, 6)), 0.001);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "house");
        var sources = HouseShellSources(sketch);
        foreach (var s in sources)
            s.Flags = CurveFlags.HelperGeometry;
        sketch.OffsetNetwork(sources, 0.15, MiterOpts());
        var mesh = api.Extrude(sketch, 2.5, 0.001, "shell");

        var joints = new[]
        {
            Vec2DOps.Zero, new Vec2D(2, 0), new Vec2D(2, -1.4), new Vec2D(5.2, -1.4),
            new Vec2D(5.2, 0), new Vec2D(13.2, 0), new Vec2D(13.2, 4.2), new Vec2D(8, 4.2),
            new Vec2D(8, 10.5), new Vec2D(0, 10.5),
        };
        var missing = new List<string>();
        foreach (var j in joints)
        {
            int n = CountVerticalGroupEdgesNear(mesh, j, 0.25);
            if (n < 2)
                missing.Add(j + " count=" + n);
        }
        Assert.True(missing.Count == 0,
            "Source joints need inner+outer vertical creases. Missing: " + string.Join("; ", missing));
    }

    static void AssertVertical(IReadOnlyList<Vec2D> pts, string name)
    {
        double minX = pts.Min(p => p.X);
        double maxX = pts.Max(p => p.X);
        double minY = pts.Min(p => p.Y);
        double maxY = pts.Max(p => p.Y);
        Assert.True(maxY - minY > 3.0 * (maxX - minX),
            name + " should be vertical, dx=" + (maxX - minX) + " dy=" + (maxY - minY));
    }

    [Fact]
    public void ClipperOffset_ClosedLineSquare_OffsetVerticesKeepSourceZ()
    {
        var square = new List<Vec2D>
        {
            Vec2DOps.Zero, new Vec2D(4, 0), new Vec2D(4, 4), new Vec2D(0, 4),
        };
        var indices = new List<List<int>>();
        var loops = NURBS.PolygonManipulator.Offset(
            square, Offset, indices, NURBS.JoinType.jtMiter, NURBS.EndType.etClosedLine, 0.01, 2);
        Assert.True(loops.Count >= 2, "Closed-line offset should emit inner and outer loops.");
        Assert.Equal(loops.Count, indices.Count);
        int tagged = 0;
        int total = 0;
        for (int i = 0; i < indices.Count; i++)
        {
            Assert.Equal(loops[i].Count, indices[i].Count);
            for (int j = 0; j < indices[i].Count; j++)
            {
                total++;
                int id = indices[i][j];
                if (id >= 0)
                {
                    Assert.InRange(id, 0, 3);
                    tagged++;
                }
            }
        }
        Assert.True(tagged >= 8,
            "Each outer/inner miter must keep the source-vertex Z. tagged=" + tagged + "/" + total);
    }

    [Fact]
    public void ClipperOffset_CollinearJoint_KeepsSourceZ()
    {
        var line = new List<Vec2D>
        {
            Vec2DOps.Zero, new Vec2D(4, 0), new Vec2D(8, 0),
        };
        var indices = new List<List<int>>();
        var loops = NURBS.PolygonManipulator.Offset(
            line, Offset, indices, NURBS.JoinType.jtMiter, NURBS.EndType.etOpenButt, 0.01, 2);
        bool sawJoint = false;
        for (int i = 0; i < indices.Count; i++)
        {
            for (int j = 0; j < indices[i].Count; j++)
            {
                if (indices[i][j] == 1)
                    sawJoint = true;
            }
        }
        Assert.True(sawJoint, "Collinear source vertex 1 must survive on the offset contour.");
    }

    [Fact]
    public void TwoSources_OffsetPiecesMeetAtOffsetOfSharedVertex()
    {
        var sketch = new PlotterSketcher("meet");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        var pieces = sketch.OffsetNetwork(new List<Curve2D> { south, east }, Offset, MiterOpts());

        AssertPiecesMeetAtOffsetCorner(
            RequireNamed(pieces, "south@out_offset"),
            RequireNamed(pieces, "east@out_offset"),
            new Vec2D(4, 0), Offset);
        AssertPiecesMeetAtOffsetCorner(
            RequireNamed(pieces, "south@in_offset"),
            RequireNamed(pieces, "east@in_offset"),
            new Vec2D(4, 0), Offset);
    }

    [Fact]
    public void ClosedSquare_NamedPieceEndpointsAreOffsetOfSourceEnds()
    {
        var sketch = new PlotterSketcher("ends");
        var south = NamedLine(sketch, "south", Vec2DOps.Zero, new Vec2D(4, 0));
        var east = NamedLine(sketch, "east", new Vec2D(4, 0), new Vec2D(4, 4));
        var north = NamedLine(sketch, "north", new Vec2D(4, 4), new Vec2D(0, 4));
        var west = NamedLine(sketch, "west", new Vec2D(0, 4), Vec2DOps.Zero);
        var sources = new List<Curve2D> { south, east, north, west };
        var pieces = sketch.OffsetNetwork(sources, Offset, MiterOpts());

        foreach (var source in sources)
        {
            AssertNamedPieceTracksSourceEnds(
                RequireNamed(pieces, EntityNaming.FormatSketchOffsetCurve(source.Name, true)),
                source, Offset);
            AssertNamedPieceTracksSourceEnds(
                RequireNamed(pieces, EntityNaming.FormatSketchOffsetCurve(source.Name, false)),
                source, Offset);
        }
    }

    [Fact]
    public void RoundJoin_OffsetVerticesAtCornerKeepSourceIdentity()
    {
        var square = new List<Vec2D>
        {
            Vec2DOps.Zero, new Vec2D(4, 0), new Vec2D(4, 4), new Vec2D(0, 4),
        };
        var indices = new List<List<int>>();
        var loops = NURBS.PolygonManipulator.Offset(
            square, Offset, indices, NURBS.JoinType.jtRound, NURBS.EndType.etClosedLine, 0.02, 2);
        int[] counts = new int[4];
        for (int i = 0; i < indices.Count; i++)
        {
            for (int j = 0; j < indices[i].Count; j++)
            {
                int id = indices[i][j];
                if (id >= 0 && id < 4)
                    counts[id]++;
            }
        }
        for (int v = 0; v < 4; v++)
            Assert.True(counts[v] >= 2,
                "Round join at source vertex " + v + " should tag its arc verts. count=" + counts[v]);
    }

    [Fact]
    public void CollinearPolyline_OffsetPiecesMeetAtOffsetOfJoint()
    {
        var sketch = new PlotterSketcher("colmeet");
        var a = NamedLine(sketch, "a", Vec2DOps.Zero, new Vec2D(4, 0));
        var b = NamedLine(sketch, "b", new Vec2D(4, 0), new Vec2D(8, 0));
        var pieces = sketch.OffsetNetwork(new List<Curve2D> { a, b }, Offset, MiterOpts());
        AssertPiecesMeetAtOffsetCorner(
            RequireNamed(pieces, "a@out_offset"),
            RequireNamed(pieces, "b@out_offset"),
            new Vec2D(4, 0), Offset);
        AssertPiecesMeetAtOffsetCorner(
            RequireNamed(pieces, "a@in_offset"),
            RequireNamed(pieces, "b@in_offset"),
            new Vec2D(4, 0), Offset);
    }

    static void AssertPiecesMeetAtOffsetCorner(
        OffsetSampledCurve2D a, OffsetSampledCurve2D b, Vec2D joint, double offset)
    {
        var ap = a.ToReferencePoints();
        var bp = b.ToReferencePoints();
        double best = double.MaxValue;
        Vec2D meet = default;
        foreach (var p in new[] { ap[0], ap[^1] })
        {
            foreach (var q in new[] { bp[0], bp[^1] })
            {
                double d = (p - q).Length();
                if (d < best)
                {
                    best = d;
                    meet = p;
                }
            }
        }
        Assert.True(best < 1e-4,
            a.Name + " and " + b.Name + " must meet at the offset of " + joint +
            ". endpoint gap=" + best + "\na=" + ap[0] + ".." + ap[^1] + " b=" + bp[0] + ".." + bp[^1]);
        double dist = (meet - joint).Length();
        Assert.True(dist > offset * 0.5 && dist < offset * 2.5,
            "Shared endpoint " + meet + " should be the offset of joint " + joint + " dist=" + dist);
    }

    static void AssertNamedPieceTracksSourceEnds(OffsetSampledCurve2D piece, Curve2D source, double offset)
    {
        var pts = piece.ToReferencePoints();
        double dStart0 = DistPointToSegment(pts[0], source.StartPosition, source.StartPosition);
        double dStart1 = DistPointToSegment(pts[^1], source.StartPosition, source.StartPosition);
        double dEnd0 = DistPointToSegment(pts[0], source.EndPosition, source.EndPosition);
        double dEnd1 = DistPointToSegment(pts[^1], source.EndPosition, source.EndPosition);
        double toStart = Math.Min(dStart0, dStart1);
        double toEnd = Math.Min(dEnd0, dEnd1);
        Assert.True(toStart < offset * 2.2,
            piece.Name + " should start or end at the offset of source start. dist=" + toStart +
            " ends=" + pts[0] + " / " + pts[^1]);
        Assert.True(toEnd < offset * 2.2,
            piece.Name + " should start or end at the offset of source end. dist=" + toEnd +
            " ends=" + pts[0] + " / " + pts[^1]);
    }
}
