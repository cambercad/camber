using Geo.NurbsConstruction;
using GeoCore;
using NURBS;

namespace Geo.BRep
{
    public static class BRepAssembler
    {
        public static BRepSolid BuildFromAnchorMesh(AnchorMesh mesh)
        {
            var solid = new BRepSolid { Name = mesh.Name, IsVolume = mesh.IsVolume };
            var groupIds = mesh.Mesh.GetTriangleGroups();

            foreach (var kv in mesh.extendedNameToGroupId)
            {
                string patchName = kv.Key;
                int groupId = kv.Value;

                bool hasTriangles = false;
                for (int i = 0; i < groupIds.Count; i++)
                {
                    if (groupIds[i] == groupId)
                    {
                        hasTriangles = true;
                        break;
                    }
                }
                if (!hasTriangles)
                    continue;

                var face = new BRepFace { PatchName = patchName, Id = solid.Shell.Faces.Count };
                if (mesh.surfaceMetaData != null && mesh.surfaceMetaData.TryGetValue(patchName, out var meta))
                {
                    face.Surface = meta.NurbsSurface;
                    face.ParamRange = meta.ParamRange;
                    face.SurfaceType = meta.SurfaceType;
                    face.PlaneParams = meta.PlaneParams;
                    face.CylinderParams = meta.CylinderParams;
                    face.ConeParams = meta.ConeParams;
                    face.SphereParams = meta.SphereParams;
                    face.TorusParams = meta.TorusParams;
                    face.TessellationSurface = ResolveTessellationSurface(meta.NurbsSurface);
                }

                UVSurface uvSurf = null;
                bool haveSurf = mesh.TryGetSurface(patchName, out uvSurf);
                FittedMeshSurface fitted = null;

                if (!face.Exportable || (face.TessellationSurface == null && !HasAnalyticParams(face)))
                {
                    if (haveSurf && uvSurf.IsPlanar(1e-6))
                    {
                        face.TessellationSurface = BuildPlaneFromMeshPatch(uvSurf);
                        face.UsesMeshFallback = true;
                    }
                    else if (haveSurf && MeshPatchSurfaceFitter.TryFit(uvSurf, MeshPatchSurfaceFitter.InferTolerance(uvSurf), out fitted))
                    {
                        face.TessellationSurface = fitted.Surface;
                        face.UsesFittedSurface = true;
                    }
                    else if (!face.Exportable)
                    {
                        continue;
                    }
                }

                // Trim loops are the patch silhouette in the triangle mesh. UV comes from
                // the owning triangle corners — never by projecting onto the NURBS surface.
                var patchUv = fitted != null && fitted.InventedUv
                    ? InventedUvLookup(fitted.VertexUv)
                    : BuildPatchUvLookup(mesh, groupId);
                var loops = PatchBoundaryExtractor.ExtractBoundaryLoops(
                    mesh.Mesh.Positions, mesh.Mesh.Triangles, groupIds, groupId, patchUv);

                foreach (var loop in loops)
                {
                    if (loop.Positions.Count < 3)
                        continue;
                    var uvPoints = new List<Vec2D>(loop.UvPoints);
                    if (fitted != null && fitted.NormalizeOriginalUv)
                    {
                        for (int i = 0; i < uvPoints.Count; i++)
                            uvPoints[i] = MeshPatchSurfaceFitter.NormalizeUv(uvPoints[i], fitted.UvMin, fitted.UvSpan);
                    }
                    face.Loops.Add(new BRepTrimLoop
                    {
                        WorldPoints = new List<Vec3D>(loop.Positions),
                        UvPoints = uvPoints
                    });
                }

                AssignOuterLoops(face.Loops);
                BRepCylindricalBoundary.MergeSeamLoopsIfNeeded(face);
                BRepPlanarBoundary.RemoveNonNestedInnerLoops(face);

                if (face.Loops.Count == 0)
                {
                    var domain = SynthesizeParametricDomainLoop(face);
                    if (domain != null)
                        face.Loops.Add(domain);
                }

                if (face.Loops.Count > 0)
                    solid.Shell.Faces.Add(face);
            }

            BuildSharedEdges(solid);
            return solid;
        }

        private static BRepTrimLoop SynthesizeParametricDomainLoop(BRepFace face)
        {
            INurbsSurface surf = face.Surface ?? (INurbsSurface)face.TessellationSurface;
            if (surf == null)
                return null;

            const int edgeSamples = 8;
            var world = new List<Vec3D>();
            var uv = new List<Vec2D>();
            AppendDomainEdge(surf, world, uv, 0, 0, 1, 0, edgeSamples);
            AppendDomainEdge(surf, world, uv, 1, 0, 1, 1, edgeSamples);
            AppendDomainEdge(surf, world, uv, 1, 1, 0, 1, edgeSamples);
            AppendDomainEdge(surf, world, uv, 0, 1, 0, 0, edgeSamples);
            world.Add(world[0]);
            uv.Add(uv[0]);
            return new BRepTrimLoop
            {
                WorldPoints = world,
                UvPoints = uv,
                IsOuter = true
            };
        }

        private static void AppendDomainEdge(
            INurbsSurface surf,
            List<Vec3D> world,
            List<Vec2D> uv,
            double u0, double v0, double u1, double v1,
            int samples)
        {
            int start = world.Count == 0 ? 0 : 1;
            for (int i = start; i < samples; i++)
            {
                double t = i / (double)(samples - 1);
                double u = u0 + (u1 - u0) * t;
                double v = v0 + (v1 - v0) * t;
                world.Add(surf.Evaluate(u, v));
                uv.Add(new Vec2D(u, v));
            }
        }

        private static bool HasAnalyticParams(BRepFace face)
        {
            return face.PlaneParams != null ||
                   face.CylinderParams != null ||
                   face.ConeParams != null ||
                   face.SphereParams != null ||
                   face.TorusParams != null;
        }

        private static PatchBoundaryExtractor.PatchUvLookup InventedUvLookup(Dictionary<int, Vec2D> vertexUv)
        {
            return (int fromVertex, int toVertex, out Vec2D uv) =>
            {
                if (vertexUv != null && vertexUv.TryGetValue(toVertex, out uv))
                    return true;
                if (vertexUv != null && vertexUv.TryGetValue(fromVertex, out uv))
                    return true;
                uv = default;
                return false;
            };
        }

        private static PatchBoundaryExtractor.PatchUvLookup BuildPatchUvLookup(AnchorMesh mesh, int groupId)
        {
            var groupIds = mesh.Mesh.GetTriangleGroups();
            var triangles = mesh.Mesh.Triangles;
            var trianglesEx = mesh.Mesh.TrianglesEx;
            return (int fromVertex, int toVertex, out Vec2D uv) =>
            {
                for (int i = 0; i < triangles.Count; i++)
                {
                    if (groupIds[i] != groupId)
                        continue;
                    var tri = triangles[i];
                    var ex = trianglesEx[i];
                    if (tri.A == fromVertex && tri.B == toVertex) { uv = ex.V1.UV; return true; }
                    if (tri.B == fromVertex && tri.C == toVertex) { uv = ex.V2.UV; return true; }
                    if (tri.C == fromVertex && tri.A == toVertex) { uv = ex.V0.UV; return true; }
                    if (tri.B == fromVertex && tri.A == toVertex) { uv = ex.V0.UV; return true; }
                    if (tri.C == fromVertex && tri.B == toVertex) { uv = ex.V1.UV; return true; }
                    if (tri.A == fromVertex && tri.C == toVertex) { uv = ex.V2.UV; return true; }
                }

                int dest = toVertex;
                for (int i = 0; i < triangles.Count; i++)
                {
                    if (groupIds[i] != groupId)
                        continue;
                    var tri = triangles[i];
                    var ex = trianglesEx[i];
                    if (tri.A == dest) { uv = ex.V0.UV; return true; }
                    if (tri.B == dest) { uv = ex.V1.UV; return true; }
                    if (tri.C == dest) { uv = ex.V2.UV; return true; }
                }
                uv = default;
                return false;
            };
        }

        private static BSplineSurface ResolveTessellationSurface(INurbsSurface surface)
        {
            if (surface == null)
                return null;
            try
            {
                return MeshUvMappedSurface.ResolveTessellationTarget(surface);
            }
            catch
            {
                return null;
            }
        }

        private static void AssignOuterLoops(List<BRepTrimLoop> loops)
        {
            if (loops.Count == 0)
                return;
            if (loops.Count == 1)
            {
                loops[0].IsOuter = true;
                return;
            }

            int outerIndex = 0;
            double maxArea = AbsProjectedLoopArea(loops[0].WorldPoints);
            for (int i = 1; i < loops.Count; i++)
            {
                double area = AbsProjectedLoopArea(loops[i].WorldPoints);
                if (area > maxArea)
                {
                    maxArea = area;
                    outerIndex = i;
                }
            }

            for (int i = 0; i < loops.Count; i++)
                loops[i].IsOuter = i == outerIndex;
        }

        private static double AbsProjectedLoopArea(IReadOnlyList<Vec3D> points)
        {
            if (points == null || points.Count < 4)
                return 0;

            var n = new Vec3D(0, 0, 0);
            for (int i = 0; i < points.Count - 1; i++)
            {
                var a = points[i];
                var b = points[i + 1];
                n.X += (a.Y - b.Y) * (a.Z + b.Z);
                n.Y += (a.Z - b.Z) * (a.X + b.X);
                n.Z += (a.X - b.X) * (a.Y + b.Y);
            }

            double ax = Math.Abs(n.X);
            double ay = Math.Abs(n.Y);
            double az = Math.Abs(n.Z);
            double signedArea = 0;
            if (ax >= ay && ax >= az)
            {
                for (int i = 0; i < points.Count - 1; i++)
                    signedArea += points[i].Y * points[i + 1].Z - points[i + 1].Y * points[i].Z;
            }
            else if (ay >= az)
            {
                for (int i = 0; i < points.Count - 1; i++)
                    signedArea += points[i].X * points[i + 1].Z - points[i + 1].X * points[i].Z;
            }
            else
            {
                for (int i = 0; i < points.Count - 1; i++)
                    signedArea += points[i].X * points[i + 1].Y - points[i + 1].X * points[i].Y;
            }

            return Math.Abs(0.5 * signedArea);
        }

        private static BSplineSurface BuildPlaneFromMeshPatch(UVSurface patch)
        {
            patch.ApproximatePlanarSurfaceCenter(out var normal, out var tangentX, out var tangentY);
            var origin = patch.Points[patch.Triangles[0].A];
            return new BSplinePlane(origin, normal, tangentX, tangentY);
        }

        private static void BuildSharedEdges(BRepSolid solid) =>
            BRepTrimCurves.PopulateSolidEdges(solid);
    }
}
