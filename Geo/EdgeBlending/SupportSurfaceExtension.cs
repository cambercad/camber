using GeoCore;
using NURBS;
using Geo.NurbsConstruction;

namespace Geo;

internal static class SupportSurfaceExtension
{
    // Restore missing portions of a faithful construction domain before extending
    // its natural boundary. Original triangles and rational contact edges remain
    // authoritative; this adds support outside the trimmed patch only.
    internal static UVSurface Extend(UVSurface original, MeshNormalUV mesh, int group,
        double distance, double deviation, CoordinateConverter converter)
    {
        var restored = RestoreDomain(original, mesh, group, deviation, converter);
        return restored.GetExtendedSurface(distance, converter, out _);
    }

    internal static UVSurface RestoreDomain(UVSurface original, MeshNormalUV mesh, int group,
        double deviation, CoordinateConverter converter)
    {
        var mapping = original.NurbsSurface as MeshUvMappedSurface;
        var support = mapping?.Inner ?? original.NurbsSurface as BSplineSurface;
        if (support == null) return original;
        Vec2D Parameter(Vec2D value) => mapping?.MapMeshUv(value.X, value.Y) ?? value;

        var uv = new List<Vec2D>();
        var rational = new List<Rat2Hybrid>();
        var uvIndex = new Dictionary<Vec2D, int>();
        var sourceAtUv = new Dictionary<int, int>();
        var sourceCorners = new Dictionary<(int Vertex, Vec2D Parameter), int>();
        var chart = new List<Tri>();
        var constraints = new List<Int2>();
        var boundary = Adjacency.BuildEdgeList(original.Triangles).Where(edge => edge.IsOnBorder)
            .Select(edge => Algorithms.Key(edge.Start, edge.End)).ToHashSet();
        int Point(Vec2D parameter)
        {
            if (uvIndex.TryGetValue(parameter, out int found)) return found;
            int index = uv.Count;
            uv.Add(parameter);
            BigRational x = parameter.X, y = parameter.Y;
            var exact = new Rat2Hybrid(new BigRationalHybrid(x.Numerator, x.Denominator),
                new BigRationalHybrid(y.Numerator, y.Denominator));
            exact.Simplify();
            rational.Add(exact);
            uvIndex.Add(parameter, index);
            return index;
        }
        int Corner(int vertex, Vec2D parameter)
        {
            var key = (vertex, parameter);
            if (sourceCorners.TryGetValue(key, out int found)) return found;
            int index = Point(parameter);
            if (sourceAtUv.TryGetValue(index, out int previous) &&
                mesh.PrecisionPositions[previous] != mesh.PrecisionPositions[vertex])
                throw new InvalidOperationException("A support parameter identifies different exact source vertices.");
            sourceAtUv[index] = vertex;
            sourceCorners.Add(key, index);
            return index;
        }
        double fidelity = deviation + 2 * Math.Sqrt(3) * converter.SmallestUnit();
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var data = mesh.TrianglesEx[i];
            if (data.GroupId != group) continue;
            var triangle = mesh.Triangles[i];
            var vertices = new[] { triangle.A, triangle.B, triangle.C };
            var parameters = new[] { Parameter(data.V0.UV), Parameter(data.V1.UV), Parameter(data.V2.UV) };
            for (int j = 0; j < 3; j++)
            {
                var parameter = parameters[j];
                if (parameter.X < 0 || parameter.X > 1 || parameter.Y < 0 || parameter.Y > 1 ||
                    (support.Evaluate(parameter.X, parameter.Y) - mesh.Positions[vertices[j]]).Length() > fidelity)
                    throw new InvalidOperationException($"Support fidelity failed at triangle {i}, vertex {vertices[j]}, UV ({parameter.X:G17},{parameter.Y:G17}): error {(support.Evaluate(parameter.X, parameter.Y) - mesh.Positions[vertices[j]]).Length():G17}, permitted {fidelity:G17}.");
            }
            var mapped = new[] { Corner(triangle.A, parameters[0]), Corner(triangle.B, parameters[1]), Corner(triangle.C, parameters[2]) };
            chart.Add(new Tri(mapped[0], mapped[1], mapped[2]));
            for (int j = 0; j < 3; j++)
                if (boundary.Contains(Algorithms.Key(vertices[j], vertices[(j + 1) % 3])))
                    constraints.Add(new Int2(mapped[j], mapped[(j + 1) % 3]));
        }
        var corners = new[] { Parameter(new(0, 0)), Parameter(new(1, 0)), Parameter(new(1, 1)), Parameter(new(0, 1)) };
        double uMin = corners.Min(p => p.X), uMax = corners.Max(p => p.X);
        double vMin = corners.Min(p => p.Y), vMax = corners.Max(p => p.Y);
        var border = new List<int> { Point(new(uMin, vMin)), Point(new(uMax, vMin)), Point(new(uMax, vMax)), Point(new(uMin, vMax)) };
        int orientation = chart.Select(t => Rat2Hybrid.Orient2DSign(rational[t.A], rational[t.B], rational[t.C])).First(sign => sign != 0);

        bool OnBoundary(Rat2Hybrid point)
        {
            foreach (var edge in constraints)
            {
                var a = rational[edge.X]; var b = rational[edge.Y];
                if (Rat2Hybrid.Orient2DSign(a, b, point) != 0) continue;
                if (point.X >= BigRationalHybrid.Min(a.X, b.X) && point.X <= BigRationalHybrid.Max(a.X, b.X) &&
                    point.Y >= BigRationalHybrid.Min(a.Y, b.Y) && point.Y <= BigRationalHybrid.Max(a.Y, b.Y)) return true;
            }
            return false;
        }
        var adaptive = AdaptiveSurfaceSplitter.TriangulateAdaptive(support, maxDeviation: deviation);
        foreach (var parameter in adaptive.UV)
        {
            if (parameter.X < uMin || parameter.X > uMax || parameter.Y < vMin || parameter.Y > vMax ||
                uvIndex.ContainsKey(parameter)) continue;
            BigRational x = parameter.X, y = parameter.Y;
            var point = new Rat2Hybrid(new BigRationalHybrid(x.Numerator, x.Denominator), new BigRationalHybrid(y.Numerator, y.Denominator));
            // Do not subdivide an authoritative source contact edge.
            if (!OnBoundary(point)) Point(parameter);
        }
        bool InOriginal(Rat2Hybrid point)
        {
            double x = point.X.ToDouble(), y = point.Y.ToDouble();
            foreach (var triangle in chart)
            {
                var a = uv[triangle.A]; var b = uv[triangle.B]; var c = uv[triangle.C];
                if (x < Math.Min(a.X, Math.Min(b.X, c.X)) || x > Math.Max(a.X, Math.Max(b.X, c.X)) ||
                    y < Math.Min(a.Y, Math.Min(b.Y, c.Y)) || y > Math.Max(a.Y, Math.Max(b.Y, c.Y))) continue;
                var pa = rational[triangle.A]; var pb = rational[triangle.B]; var pc = rational[triangle.C];
                if (Rat2Hybrid.Orient2DSign(pa, pb, pc) == 0) continue;
                int ab = Rat2Hybrid.Orient2DSign(pa, pb, point), bc = Rat2Hybrid.Orient2DSign(pb, pc, point), ca = Rat2Hybrid.Orient2DSign(pc, pa, point);
                if ((ab >= 0 && bc >= 0 && ca >= 0) || (ab <= 0 && bc <= 0 && ca <= 0)) return true;
            }
            return false;
        }
        // Source interiors are retained verbatim, so only triangulate missing
        // support samples. Constraint endpoints are inserted by the triangulator.
        var insertionPoints = Enumerable.Range(0, rational.Count).Except(border)
            .Where(i => !sourceAtUv.ContainsKey(i) && !InOriginal(rational[i])).ToList();
        var triangles = Triangulator.TriangulatePolygon(rational, border, constraints,
            insertionPoints, delaunayPostProcess: true);
        var skirt = triangles.Where(triangle => !InOriginal(new Rat2Hybrid(
            (rational[triangle.A].X + rational[triangle.B].X + rational[triangle.C].X) / new BigRationalHybrid(3),
            (rational[triangle.A].Y + rational[triangle.B].Y + rational[triangle.C].Y) / new BigRationalHybrid(3)))).ToList();
        Vec3D Position(int index)
        {
            if (sourceAtUv.TryGetValue(index, out int source))
                return converter.Convert(mesh.PrecisionPositions[source]);
            var parameter = uv[index];
            var lattice = converter.Convert(support.Evaluate(parameter.X, parameter.Y));
            return converter.Convert(new Rat3Hybrid(lattice.X, lattice.Y, lattice.Z));
        }
        double ErrorBound(Tri triangle)
        {
            var indices = new[] { triangle.A, triangle.B, triangle.C };
            double displacement = indices.Max(i =>
                (Position(i) - support.Evaluate(uv[i].X, uv[i].Y)).Length());
            return AdaptiveSurfaceSplitter.TriangleDeviationBound(support,
                indices.Min(i => uv[i].X), indices.Min(i => uv[i].Y),
                indices.Max(i => uv[i].X), indices.Max(i => uv[i].Y)) + displacement;
        }
        for (int iteration = 0; ; iteration++)
        {
            var failed = skirt.Where(t => ErrorBound(t) > fidelity).ToList();
            if (failed.Count == 0) break;
            if (iteration == 8)
                throw new InvalidOperationException($"Support refinement did not converge: {failed.Count} triangles, worst bound {failed.Max(ErrorBound)}.");
            foreach (var triangle in failed)
            {
                var centre = (uv[triangle.A] + uv[triangle.B] + uv[triangle.C]) / 3;
                int count = uv.Count;
                insertionPoints.Add(Point(centre));
                if (uv.Count == count)
                    throw new InvalidOperationException("Support refinement exhausted distinct interior parameters.");
            }
            triangles = Triangulator.TriangulatePolygon(rational, border, constraints,
                insertionPoints, delaunayPostProcess: true);
            skirt = triangles.Where(triangle => !InOriginal(new Rat2Hybrid(
                (rational[triangle.A].X + rational[triangle.B].X + rational[triangle.C].X) / new BigRationalHybrid(3),
                (rational[triangle.A].Y + rational[triangle.B].Y + rational[triangle.C].Y) / new BigRationalHybrid(3)))).ToList();
        }
        if (skirt.Count == 0) return original;

        var positions = new List<Vec3D>(original.Points);
        var precise = new List<Rat3Hybrid>(original.PointsPrecise);
        var normals = new List<Vec3D>(original.Normals);
        var parametersOut = new List<Vec2D>(original.Uv);
        var sewn = new List<Tri>(original.Triangles);
        var exactIndices = new Dictionary<Rat3Hybrid, int>();
        // The topology view retains the parent mesh's vertex indices, including
        // unused vertices from other faces with no patch normal or UV. Only
        // referenced patch vertices can be reused as existing support geometry.
        foreach (int i in original.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
        {
            var sourcePoint = precise[i];
            var key = new Rat3Hybrid(in sourcePoint); key.Simplify();
            exactIndices.TryAdd(key, i);
        }
        var outputIndices = new Dictionary<int, int>();
        int Output(int index)
        {
            if (sourceAtUv.TryGetValue(index, out int source)) return source;
            if (outputIndices.TryGetValue(index, out int mapped)) return mapped;
            var parameter = uv[index];
            var point = support.Evaluate(parameter.X, parameter.Y);
            var lattice = converter.Convert(point);
            var exact = new Rat3Hybrid(lattice.X, lattice.Y, lattice.Z);
            if (!exactIndices.TryGetValue(exact, out mapped))
            {
                mapped = positions.Count;
                positions.Add(converter.Convert(exact)); precise.Add(exact);
                normals.Add(support.EvaluateNormal(parameter.X, parameter.Y).Normalized() * orientation); parametersOut.Add(parameter);
                exactIndices.Add(exact, mapped);
            }
            outputIndices.Add(index, mapped);
            return mapped;
        }
        foreach (var triangle in skirt)
        {
            var indices = new[] { triangle.A, triangle.B, triangle.C };
            var mapped = indices.Select(Output).ToArray();
            if (Rat2Hybrid.Orient2DSign(rational[triangle.A], rational[triangle.B], rational[triangle.C]) == 0)
                throw new InvalidOperationException("A restored support triangle has zero parameter area.");
            double minU = indices.Min(i => uv[i].X), maxU = indices.Max(i => uv[i].X);
            double minV = indices.Min(i => uv[i].Y), maxV = indices.Max(i => uv[i].Y);
            double displacement = 0;
            for (int corner = 0; corner < 3; corner++)
            {
                var parameter = uv[indices[corner]];
                displacement = Math.Max(displacement,
                    (converter.Convert(precise[mapped[corner]]) - support.Evaluate(parameter.X, parameter.Y)).Length());
            }
            double bound = AdaptiveSurfaceSplitter.TriangleDeviationBound(support, minU, minV, maxU, maxV) + displacement;
            if (bound > fidelity)
                throw new InvalidOperationException($"Restored support triangle exceeds deviation: bound {bound:G17}, permitted {fidelity:G17}.");
            // Canonicalizing evaluated points onto the exact construction
            // lattice can collapse a triangle; it contributes no surface area.
            if (mapped[0] != mapped[1] && mapped[1] != mapped[2] && mapped[2] != mapped[0])
                sewn.Add(orientation > 0 ? new Tri(mapped[0], mapped[1], mapped[2]) : new Tri(mapped[0], mapped[2], mapped[1]));
        }
        return new UVSurface(positions, normals, parametersOut, sewn, precise);
    }
}
