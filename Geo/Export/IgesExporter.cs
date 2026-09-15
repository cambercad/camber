using Geo.BRep;
using GeoCore;

namespace Geo.Export
{
    public static class IgesExporter
    {
        public static void Export(BRepSolid solid, string filePath)
        {
            BRepTrimCurves.PopulateSolidEdges(solid);

            var writer = new IgesWriter();
            int faces = 0;

            foreach (var face in solid.Shell.Faces)
            {
                if (face.Loops.Count == 0)
                    continue;
                if (face.TessellationSurface == null && !ExportAnalyticSurface.HasElementarySurface(face))
                    continue;

                int surfDe = WriteFaceSurface(writer, face);
                var outer = BRepLoopUtil.FindOuterLoop(face.Loops);
                int outerDe = WriteLoopOnSurface(writer, solid, surfDe, face, outer);
                if (outerDe == 0)
                    continue;

                var inners = new List<int>();
                foreach (var loop in face.Loops)
                {
                    if (loop == outer || loop.IsOuter)
                        continue;
                    int innerDe = WriteLoopOnSurface(writer, solid, surfDe, face, loop);
                    if (innerDe != 0)
                        inners.Add(innerDe);
                }

                writer.WriteTrimmedSurface(surfDe, outerDe, inners);
                faces++;
            }

            if (faces == 0)
                throw new InvalidOperationException("No exportable faces in solid.");

            writer.Save(filePath);
        }

        public static void Export(AnchorMesh mesh, string filePath)
        {
            var solid = BRepAssembler.BuildFromAnchorMesh(mesh);
            Export(solid, filePath);
        }

        private static int WriteLoopOnSurface(
            IgesWriter writer, BRepSolid solid, int surfaceDe, BRepFace face, BRepTrimLoop loop)
        {
            if (loop == null || loop.WorldPoints == null || loop.WorldPoints.Count < 2)
                return 0;

            BRepTrimLoop exportLoop = PrepareLoopForSurface(face, loop);
            exportLoop.EdgeIds = loop.EdgeIds;
            exportLoop.EdgePointCounts = loop.EdgePointCounts;

            var uvChild = new List<int>();
            var xyzChild = new List<int>();
            if (loop.EdgeIds != null && loop.EdgeIds.Count > 0)
            {
                BRepTrimCurves.WalkLoopStrips(solid, exportLoop, (_, start, count) =>
                {
                    var world = new List<Vec3D>();
                    var uv = new List<Vec2D>();
                    BRepTrimCurves.CollectStripPoints(exportLoop, start, count, world, uv);
                    if (world.Count < 2)
                        return;

                    xyzChild.Add(WriteIgesWorldStrip(writer, world));
                    uvChild.Add(WriteIgesUvStrip(writer, uv));
                });
            }
            else
            {
                foreach (var segment in BRepTrimCurves.ChordSegmentsFromLoopWithUv(exportLoop))
                {
                    var p0 = segment.Curve.EvaluateUniform(0);
                    var p1 = segment.Curve.EvaluateUniform(1);
                    if ((p1 - p0).Length() < BRepTrimCurves.MinSegmentLength)
                        continue;
                    xyzChild.Add(writer.WriteLine(p0, p1));
                    uvChild.Add(writer.WriteUvLine(segment.Uv0, segment.Uv1));
                }
            }

            if (xyzChild.Count == 0)
                return 0;

            int uvDe = uvChild.Count == 1
                ? uvChild[0]
                : writer.WriteCompositeCurve(uvChild, entityUseFlag: 5);
            int xyzDe = xyzChild.Count == 1
                ? xyzChild[0]
                : writer.WriteCompositeCurve(xyzChild);
            return writer.WriteCurveOnParametricSurface(surfaceDe, uvDe, xyzDe);
        }

        private static int WriteIgesWorldStrip(IgesWriter writer, List<Vec3D> world)
        {
            var curve = BRepTrimCurves.SmoothStripToBSpline(world);
            if (curve == null || curve.ControlPoints.Length <= 2)
                return writer.WriteLine(world[0], world[world.Count - 1]);
            return writer.WriteNurbsCurve(curve);
        }

        private static int WriteIgesUvStrip(IgesWriter writer, List<Vec2D> uv)
        {
            if (uv.Count == 2 || BRepTrimCurves.IsCollinearUv(uv))
                return writer.WriteUvLine(uv[0], uv[uv.Count - 1]);
            return writer.WriteUvNurbsCurve(uv);
        }

        private static BRepTrimLoop PrepareLoopForSurface(BRepFace face, BRepTrimLoop loop)
        {
            bool isCylinder = ExportAnalyticSurface.TryCylinder(
                face, out _, out _, out _, out _, out double cylHeight);
            if (isCylinder && loop.UvPoints != null && loop.UvPoints.Count > 0)
            {
                var unwrapped = ExportAnalyticSurface.UnwrapCylinderMeshUv(loop.UvPoints);
                var mapped = new List<Vec2D>(unwrapped.Count);
                for (int i = 0; i < unwrapped.Count; i++)
                    mapped.Add(ExportAnalyticSurface.CylinderMeshUvToIges(unwrapped[i], cylHeight));

                return new BRepTrimLoop
                {
                    WorldPoints = loop.WorldPoints,
                    UvPoints = mapped,
                    IsOuter = loop.IsOuter
                };
            }

            if (ExportAnalyticSurface.TryPlane(face, out var origin, out var normal, out var refDir))
                return ExportAnalyticSurface.LoopWithPlaneUv(loop, origin, normal, refDir);

            return ExportAnalyticSurface.LoopOnNurbsKeepWorld(face, loop);
        }

        private static int WriteFaceSurface(IgesWriter writer, BRepFace face)
        {
            if (ExportAnalyticSurface.TryPlane(face, out var origin, out var normal, out var refDir))
                return writer.WritePlaneSurface(origin, normal, refDir);
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
