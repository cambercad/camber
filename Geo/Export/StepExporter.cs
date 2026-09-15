using Geo.BRep;
using GeoCore;
using NURBS;

namespace Geo.Export
{
    public static class StepExporter
    {
        public static void Export(BRepSolid solid, string filePath, double linearTolerance = 0)
        {
            BRepTrimCurves.PopulateSolidEdges(solid);

            var writer = new StepWriter();
            writer.LinearUncertainty = linearTolerance > 1e-12 ? linearTolerance : 1e-3;
            var vertexStepIds = new int[solid.Vertices.Count];
            for (int i = 0; i < solid.Vertices.Count; i++)
                vertexStepIds[i] = writer.WriteVertex(solid.Vertices[i].Position);

            var exportable = new List<BRepFace>();
            var surfIds = new Dictionary<int, int>();
            foreach (var face in solid.Shell.Faces)
            {
                if (face.TessellationSurface == null && !ExportAnalyticSurface.HasElementarySurface(face))
                    continue;
                exportable.Add(face);
                surfIds[face.Id] = WriteFaceSurface(writer, face);
            }

            var uses = new List<(int SurfId, List<Vec2D> Uv)>[solid.Edges.Count];
            for (int i = 0; i < solid.Edges.Count; i++)
                uses[i] = new List<(int, List<Vec2D>)>();

            foreach (var face in exportable)
            {
                int surfId = surfIds[face.Id];
                bool isCylinder = ExportAnalyticSurface.TryCylinder(
                    face, out _, out _, out _, out _, out double cylHeight);
                bool isPlane = ExportAnalyticSurface.TryPlane(
                    face, out var planeOrigin, out var planeNormal, out var planeRef);

                foreach (var loop in face.Loops)
                {
                    if (loop.EdgeIds == null || loop.EdgeIds.Count == 0)
                        continue;

                    var uvPoints = loop.UvPoints;
                    if (isCylinder && uvPoints != null && uvPoints.Count > 0)
                        uvPoints = ExportAnalyticSurface.UnwrapCylinderMeshUv(uvPoints);

                    var exportLoop = new BRepTrimLoop
                    {
                        WorldPoints = loop.WorldPoints,
                        UvPoints = uvPoints ?? loop.UvPoints,
                        EdgeIds = loop.EdgeIds,
                        EdgePointCounts = loop.EdgePointCounts,
                        IsOuter = loop.IsOuter
                    };

                    BRepTrimCurves.WalkLoopStrips(solid, exportLoop, (edge, start, count) =>
                    {
                        var world = new List<Vec3D>();
                        var meshUv = new List<Vec2D>();
                        BRepTrimCurves.CollectStripPoints(exportLoop, start, count, world, meshUv);
                        if (world.Count < 2)
                            return;

                        var uv = new List<Vec2D>(meshUv.Count);
                        for (int k = 0; k < meshUv.Count; k++)
                        {
                            if (isCylinder)
                                uv.Add(ExportAnalyticSurface.CylinderMeshUvToStep(meshUv[k], cylHeight));
                            else if (isPlane)
                                uv.Add(ExportAnalyticSurface.PlaneWorldToParam(
                                    world[k], planeOrigin, planeNormal, planeRef));
                            else
                                uv.Add(ExportAnalyticSurface.MeshUvToNative(face, meshUv[k]));
                        }

                        var startPos = solid.Vertices[edge.StartVertexId].Position;
                        if ((startPos - world[0]).LengthSquared() > (startPos - world[world.Count - 1]).LengthSquared())
                            uv.Reverse();

                        uses[edge.Id].Add((surfId, uv));
                    });
                }
            }

            var edgeStepIds = new int[solid.Edges.Count];
            for (int i = 0; i < solid.Edges.Count; i++)
            {
                if (uses[i].Count == 0)
                    continue;

                var edge = solid.Edges[i];
                int v0 = vertexStepIds[edge.StartVertexId];
                int v1 = vertexStepIds[edge.EndVertexId];
                var p0 = solid.Vertices[edge.StartVertexId].Position;
                var p1 = solid.Vertices[edge.EndVertexId].Position;
                int curve3d = WriteEdgeGeometry(writer, edge, p0, p1);

                var pcurveIds = new List<int>(uses[i].Count);
                for (int u = 0; u < uses[i].Count; u++)
                {
                    var use = uses[i][u];
                    if (use.Uv == null || use.Uv.Count < 2)
                        continue;
                    int uvCurve = CurveControlCount(edge.Curve) <= 2
                        ? writer.WriteUvLine(use.Uv[0], use.Uv[use.Uv.Count - 1])
                        : writer.WriteUvPolyline(use.Uv);
                    int defRep = writer.WriteDefinitionalRepresentation(uvCurve);
                    pcurveIds.Add(writer.WritePcurve(use.SurfId, defRep));
                }

                int edgeGeom = writer.WriteSurfaceCurve(curve3d, pcurveIds);
                edgeStepIds[i] = writer.WriteEdgeCurve(v0, v1, edgeGeom);
            }

            var faceIds = new List<int>();
            foreach (var face in exportable)
            {
                int surfId = surfIds[face.Id];
                var boundIds = new List<int>();
                foreach (var loop in face.Loops)
                {
                    if (loop.EdgeIds == null || loop.EdgeIds.Count == 0)
                        continue;

                    var orientedEdges = new List<int>();
                    BRepTrimCurves.WalkLoopStrips(solid, loop, (edge, start, count) =>
                    {
                        var world = new List<Vec3D>();
                        var uv = new List<Vec2D>();
                        BRepTrimCurves.CollectStripPoints(loop, start, count, world, uv);
                        if (world.Count < 2)
                            return;

                        var startPos = solid.Vertices[edge.StartVertexId].Position;
                        bool sameSense =
                            (startPos - world[0]).LengthSquared() <=
                            (solid.Vertices[edge.EndVertexId].Position - world[0]).LengthSquared();
                        orientedEdges.Add(writer.WriteOrientedEdge(
                            vertexStepIds[edge.StartVertexId],
                            vertexStepIds[edge.EndVertexId],
                            edgeStepIds[edge.Id],
                            sameSense));
                    });

                    if (orientedEdges.Count == 0)
                        continue;

                    int edgeLoopId = writer.WriteEdgeLoop(orientedEdges);
                    boundIds.Add(writer.WriteFaceBound(edgeLoopId, loop.IsOuter, orientation: true));
                }

                if (boundIds.Count == 0)
                    boundIds.Add(writer.WriteFaceBound(writer.WriteEdgeLoop(new List<int>()), isOuter: true, orientation: true));

                faceIds.Add(writer.WriteAdvancedFace(surfId, boundIds));
            }

            if (faceIds.Count == 0)
                throw new InvalidOperationException("No exportable faces in solid.");

            if (solid.IsVolume)
            {
                int shellId = writer.WriteClosedShell(faceIds);
                int solidId = writer.WriteManifoldSolidBrep(shellId);
                writer.WriteAp214ProductStructure(solidId, solid.Name, isSolid: true);
            }
            else
            {
                int shellId = writer.WriteOpenShell(faceIds);
                int modelId = writer.WriteShellBasedSurfaceModel(shellId);
                writer.WriteAp214ProductStructure(modelId, solid.Name, isSolid: false);
            }
            writer.Save(filePath);
        }

        public static void Export(AnchorMesh mesh, string filePath, double linearTolerance = 0)
        {
            var solid = BRepAssembler.BuildFromAnchorMesh(mesh);
            Export(solid, filePath, linearTolerance);
        }

        private static int CurveControlCount(BSplineCurve curve)
        {
            return curve == null || curve.ControlPoints == null ? 0 : curve.ControlPoints.Length;
        }

        private static int WriteEdgeGeometry(StepWriter writer, BRepEdge edge, Vec3D p0, Vec3D p1)
        {
            if (CurveControlCount(edge.Curve) > 2)
                return writer.WriteNurbsCurve(edge.Curve);
            return writer.WriteUnitIntervalSegment3D(p0, p1);
        }

        private static int WriteFaceSurface(StepWriter writer, BRepFace face)
        {
            if (ExportAnalyticSurface.TryPlane(face, out var origin, out var normal, out var refDir))
                return writer.WritePlane(origin, normal, refDir);
            if (ExportAnalyticSurface.TryCylinder(face, out origin, out var axis, out refDir, out double radius, out _))
                return writer.WriteCylindricalSurface(origin, axis, refDir, radius);
            if (ExportAnalyticSurface.TryCone(face, out origin, out axis, out refDir, out radius, out double semi))
                return writer.WriteConicalSurface(origin, axis, refDir, radius, semi);
            if (ExportAnalyticSurface.TrySphere(face, out var center, out axis, out refDir, out radius))
                return writer.WriteSphericalSurface(center, axis, refDir, radius);
            if (ExportAnalyticSurface.TryTorus(face, out center, out axis, out refDir, out double major, out double minor))
                return writer.WriteToroidalSurface(center, axis, refDir, major, minor);
            if (face.TessellationSurface == null)
                throw new InvalidOperationException($"Face '{face.PatchName}' has neither analytic params nor a NURBS payload.");
            return writer.WriteNurbsSurface(face.TessellationSurface);
        }
    }
}
