using GeoCore;
using NURBS;
using Geo.NurbsConstruction;

namespace Geo;

internal static class SupportSurfaceExtension
{
    internal static bool IsSupportingPlane(UVSurface plane, MeshNormalUV mesh)
    {
        var triangle = plane.Triangles[0];
        var origin = plane.PointsPrecise[triangle.A];
        var normal = Rat3Hybrid.Cross(plane.PointsPrecise[triangle.B] - origin,
            plane.PointsPrecise[triangle.C] - origin);
        foreach (var face in mesh.Triangles)
            foreach (int index in new[] { face.A, face.B, face.C })
                if (Rat3Hybrid.Dot(mesh.PrecisionPositions[index] - origin, normal).Sign() > 0)
                    return false;
        return !normal.IsZero();
    }

    // A planar trim has no holes: extend its support, not its face boundary.
    // In particular, growing the inner loop of an annulus folds its collar.
    // Use exact affine vertices covering the complete target mesh projection.
    internal static UVSurface CoverPlanarMesh(UVSurface plane, IReadOnlyList<Rat3Hybrid> points,
        CoordinateConverter converter)
    {
        if (!plane.IsSurfacePlanar()) throw new ArgumentException("Trim support must be planar.");
        var triangle = plane.Triangles[0];
        var origin = plane.PointsPrecise[triangle.A];
        var x = plane.PointsPrecise[triangle.B] - origin;
        var normal = Rat3Hybrid.Cross(x, plane.PointsPrecise[triangle.C] - origin);
        var y = Rat3Hybrid.Cross(normal, x);
        static BigRationalHybrid Scale(Rat3Hybrid vector) =>
            BigRationalHybrid.Max(BigRationalHybrid.Abs(vector.X),
                BigRationalHybrid.Max(BigRationalHybrid.Abs(vector.Y), BigRationalHybrid.Abs(vector.Z)));
        if (x.IsZero() || y.IsZero()) throw new ArgumentException("Trim support is degenerate.");
        x /= Scale(x); y /= Scale(y);
        var xx = Rat3Hybrid.Dot(x, x);
        var yy = Rat3Hybrid.Dot(y, y);
        if (xx.Sign() == 0 || yy.Sign() == 0) throw new ArgumentException("Trim support is degenerate.");
        BigRationalHybrid xmin = new(0), xmax = new(0), ymin = new(0), ymax = new(0);
        foreach (var point in points)
        {
            var delta = point - origin;
            var u = Rat3Hybrid.Dot(delta, x) / xx;
            var v = Rat3Hybrid.Dot(delta, y) / yy;
            xmin = BigRationalHybrid.Min(xmin, u); xmax = BigRationalHybrid.Max(xmax, u);
            ymin = BigRationalHybrid.Min(ymin, v); ymax = BigRationalHybrid.Max(ymax, v);
        }
        var margin = new BigRationalHybrid(1);
        xmin -= margin; xmax += margin; ymin -= margin; ymax += margin;
        var precise = new List<Rat3Hybrid> {
            origin + x * xmin + y * ymin, origin + x * xmax + y * ymin,
            origin + x * xmax + y * ymax, origin + x * xmin + y * ymax };
        var positions = converter.Convert(precise);
        var direction = Vec3DOps.Cross(positions[1] - positions[0], positions[2] - positions[0]).Normalized();
        return new UVSurface(positions, new List<Vec3D> { direction, direction, direction, direction },
            new List<Vec2D> { new(0, 0), new(1, 0), new(1, 1), new(0, 1) },
            new List<Tri> { new(0, 1, 2), new(0, 2, 3) }, precise);
    }

    // Restore missing portions of a faithful construction domain before extending
    // its natural boundary. Original triangles and rational contact edges remain
    // authoritative; this adds support outside the trimmed patch only.
    // Restoration is deviation-bounded against stored surface metadata. The
    // subsequent tangential collar is NOT a certified curved continuation beyond
    // that domain; do not use it to claim a curvature-accurate shell junction.
    internal static UVSurface Extend(UVSurface original, MeshNormalUV mesh, int group,
        double distance, double deviation, CoordinateConverter converter)
    {
        var restored = RestoreDomain(original, mesh, group, deviation, converter);
        return restored.GetExtendedSurface(distance, converter, out _);
    }

    internal static UVSurface RestoreDomain(UVSurface original, MeshNormalUV mesh, int group,
        double deviation, CoordinateConverter converter)
    {
        var parameterSurface = original.NurbsSurface;
        while (parameterSurface is TransformedNurbsSurface transformed) parameterSurface = transformed.Inner;
        var mapping = parameterSurface as MeshUvMappedSurface;
        if (mapping == null && parameterSurface is not BSplineSurface) return original;
        var support = MeshUvMappedSurface.ResolveTessellationTarget(original.NurbsSurface);
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
        var resolvedParameters = new Dictionary<(int Vertex, Vec2D Uv), Vec2D>();
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var data = mesh.TrianglesEx[i];
            if (data.GroupId != group) continue;
            var triangle = mesh.Triangles[i];
            var vertices = new[] { triangle.A, triangle.B, triangle.C };
            var parameters = new[]
            {
                ResolveParameter(data.V0.UV, vertices[0]),
                ResolveParameter(data.V1.UV, vertices[1]),
                ResolveParameter(data.V2.UV, vertices[2])
            };
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

        Vec2D ResolveParameter(Vec2D meshUv, int vertex)
        {
            if (resolvedParameters.TryGetValue((vertex, meshUv), out var cached)) return cached;
            var point = mesh.Positions[vertex];
            var parameter = meshUv;
            var error = (original.NurbsSurface.Evaluate(parameter.X, parameter.Y) - point).Length();
            if (error <= fidelity) return resolvedParameters[(vertex, meshUv)] = Parameter(meshUv);
            // CSG interpolates corner UVs affinely on the source triangle. That
            // is exact for affine charts, but not for arc-length mapped NURBS.
            // Recover the closest chart parameter only when the stored support
            // is already within the modeling tolerance of the authoritative
            // mesh point; this cannot turn unrelated metadata into a support.
            if (error > 2 * deviation + fidelity) return resolvedParameters[(vertex, meshUv)] = Parameter(meshUv);
            double best = error;
            for (int iteration = 0; iteration < 24; iteration++)
            {
                const double step = 1e-5;
                double u0 = Math.Max(0, parameter.X - step), u1 = Math.Min(1, parameter.X + step);
                double v0 = Math.Max(0, parameter.Y - step), v1 = Math.Min(1, parameter.Y + step);
                if (u0 == u1 || v0 == v1) break;
                var value = original.NurbsSurface.Evaluate(parameter.X, parameter.Y);
                var du = (original.NurbsSurface.Evaluate(u1, parameter.Y) -
                    original.NurbsSurface.Evaluate(u0, parameter.Y)) / (u1 - u0);
                var dv = (original.NurbsSurface.Evaluate(parameter.X, v1) -
                    original.NurbsSurface.Evaluate(parameter.X, v0)) / (v1 - v0);
                var residual = value - point;
                double a = Vec3DOps.Dot(du, du), b = Vec3DOps.Dot(du, dv), c = Vec3DOps.Dot(dv, dv);
                double determinant = a * c - b * b;
                if (!(determinant > 0) || !double.IsFinite(determinant)) break;
                double ru = Vec3DOps.Dot(du, residual), rv = Vec3DOps.Dot(dv, residual);
                double deltaU = (c * ru - b * rv) / determinant;
                double deltaV = (a * rv - b * ru) / determinant;
                bool improved = false;
                for (double factor = 1; factor >= 1.0 / 128; factor *= .5)
                {
                    var candidate = new Vec2D(Math.Clamp(parameter.X - factor * deltaU, 0, 1),
                        Math.Clamp(parameter.Y - factor * deltaV, 0, 1));
                    double candidateError = (original.NurbsSurface.Evaluate(candidate.X, candidate.Y) - point).Length();
                    if (candidateError >= best) continue;
                    parameter = candidate;
                    best = candidateError;
                    improved = true;
                    break;
                }
                if (!improved || best <= fidelity) break;
            }
            return resolvedParameters[(vertex, meshUv)] =
                best <= fidelity ? Parameter(parameter) : Parameter(meshUv);
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
        // The restoration is triangulated in the underlying support's chart.
        // Convert retained UVs too: mixing mesh/arc-length UVs with new raw
        // NURBS parameters makes the restored support impossible to evaluate.
        var parametersOut = new List<Vec2D>(original.Uv.Count);
        foreach (var parameter in original.Uv) parametersOut.Add(Parameter(parameter));
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
        return new UVSurface(positions, normals, parametersOut, sewn, precise)
            { NurbsSurface = support, NurbsParamRange = ParametricRange.UnitSquare };
    }
}
