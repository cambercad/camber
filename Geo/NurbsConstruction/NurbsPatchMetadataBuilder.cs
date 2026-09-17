using Curves;
using GeoCore;
using GeoMeta;
using NURBS;

namespace Geo.NurbsConstruction
{
    public static class NurbsPatchMetadataBuilder
    {
        public static List<Vec2D> CollectProfilePoints(IEnumerable<List<List<Vec2D>>> contours)
        {
            var points = new List<Vec2D>();
            foreach (var loop in contours)
            {
                foreach (var strip in loop)
                {
                    foreach (var p in strip)
                        points.Add(p);
                }
            }
            return points;
        }

        public static Dictionary<string, List<Vec2D>> ExtractNamedSegments(
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames)
        {
            var result = new Dictionary<string, List<Vec2D>>();
            for (int strip = 0; strip < contour.Count; strip++)
            {
                var loops = contour[strip];
                var names = contourNames[strip];
                for (int i = 0; i < loops.Count; i++)
                {
                    string segName = names[i];
                    if (string.IsNullOrEmpty(segName) || result.ContainsKey(segName))
                        continue;
                    result[segName] = new List<Vec2D>(loops[i]);
                }
            }
            return result;
        }

        public static Dictionary<string, SurfaceMetaData> BuildExtrudeMetadata(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem sketchFrame,
            double heightPositive,
            double heightNegative,
            string operationName,
            double twistRatePerExtrudeDistance,
            IEnumerable<Vec2D> allProfilePoints,
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames)
        {
            var segmentPolylines = ExtractNamedSegments(contour, contourNames);
            // The extruder normalizes winding on copies. Use that same ordering
            // here, rather than the untouched authored contour passed by GeoAPI.
            var reversals = MeshConstructionHelpers.GetContourReversalInfo(contour);
            for (int loop = 0; loop < contourNames.Count; loop++)
                if (reversals[loop])
                    foreach (string segment in contourNames[loop])
                        if (segmentPolylines.TryGetValue(segment, out var points)) points.Reverse();
            return BuildExtrudeMetadataCore(curveMeta, sketchFrame, heightPositive, heightNegative,
                operationName, twistRatePerExtrudeDistance, allProfilePoints, segmentPolylines);
        }

        private static Dictionary<string, SurfaceMetaData> BuildExtrudeMetadataCore(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem sketchFrame,
            double heightPositive,
            double heightNegative,
            string operationName,
            double twistRatePerExtrudeDistance,
            IEnumerable<Vec2D> allProfilePoints,
            Dictionary<string, List<Vec2D>> segmentPolylines)
        {
            var result = new Dictionary<string, SurfaceMetaData>();
            bool hasTwist = Math.Abs(twistRatePerExtrudeDistance) > 1e-15;

            double bottomZ = -heightNegative;
            double topZ = heightPositive;
            double extrudeLength = topZ - bottomZ;
            var extrudeDir = sketchFrame.Z;

            NurbsSurfaceFactory.ProfileBoundingBox(allProfilePoints, out var uvMin, out var uvMax);

            var bottomFrame = OffsetFrameAlongZ(sketchFrame, bottomZ);

            if (heightPositive > 0 || heightNegative > 0)
            {
                result[EntityNaming.ExtrudeBottom(operationName)] =
                    BuildPlanarCapMeta(bottomFrame, uvMin, uvMax);
                var topFrame = OffsetFrameAlongZ(sketchFrame, topZ);
                if (hasTwist)
                    topFrame = RotateFrameAboutAxis(topFrame, extrudeDir, twistRatePerExtrudeDistance * extrudeLength);
                result[EntityNaming.ExtrudeTop(operationName)] =
                    BuildPlanarCapMeta(topFrame, uvMin, uvMax);
            }

            foreach (var kv in curveMeta)
            {
                string sideName = EntityNaming.ExtrudeSide(operationName, kv.Key);
                var surfaceType = hasTwist ? SurfaceType.Unknown : SurfaceTypeFromCurve(kv.Value.CurveType);

                if (kv.Value.Instance is not Curve2D curve2d)
                {
                    result[sideName] = new SurfaceMetaData(surfaceType);
                    continue;
                }

                var profile = BuildProfileCurve(curve2d, bottomFrame, segmentPolylines, kv.Key);
                if (hasTwist)
                {
                    var frames = BuildTwistedFrames(bottomFrame, extrudeDir, extrudeLength, twistRatePerExtrudeDistance);
                    var bottomFrameCopy = bottomFrame;
                    result[sideName] = new SurfaceMetaData(surfaceType, () =>
                    {
                        var surface = NurbsSurfaceFactory.SweepProfileAlongFrames(profile, bottomFrameCopy, frames);
                        return NurbsSurfaceFactory.WrapForMeshUv(surface, profile);
                    }, ParametricRange.UnitSquare);
                    continue;
                }

                var extrudeDirCopy = extrudeDir;
                var sideMeta = new SurfaceMetaData(surfaceType, () =>
                {
                    var extrudeSurface = NurbsSurfaceFactory.LinearExtrude(profile, extrudeDirCopy, extrudeLength);
                    return NurbsSurfaceFactory.WrapLinearExtrudeForMeshUv(extrudeSurface, profile);
                }, ParametricRange.UnitSquare);
                if (curve2d is Circle2D circle2d)
                    sideMeta.CylinderParams = BuildCylinderParams(circle2d, bottomFrame, extrudeDir, extrudeLength);
                else if (curve2d is Arc2D arc2d)
                    sideMeta.CylinderParams = BuildCylinderParams(arc2d.Center, arc2d.StartPosition,
                        arc2d.Radius, bottomFrame, extrudeDir, extrudeLength);
                else if (curve2d is Line2D line2d)
                {
                    var origin = bottomFrame.PointTo3D(line2d.StartPosition);
                    var tangent = bottomFrame.PointTo3D(line2d.EndPosition) - origin;
                    sideMeta.PlaneParams = new PlaneSurfaceParams
                    {
                        Origin = origin,
                        Normal = Vec3DOps.Cross(tangent, extrudeDir).Normalized(),
                        RefDir = tangent.Normalized()
                    };
                }
                result[sideName] = sideMeta;
            }

            return result;
        }

        public static Dictionary<string, SurfaceMetaData> BuildRevolveMetadata(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem sketchFrame,
            double angle,
            string operationName,
            IEnumerable<Vec2D> allProfilePoints,
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames)
        {
            var segmentPolylines = ExtractNamedSegments(contour, contourNames);
            var result = new Dictionary<string, SurfaceMetaData>();
            var axis = sketchFrame.X;
            var zeroDir = sketchFrame.Y;
            var pointOnAxis = sketchFrame.Origin;
            var vRange = new ParametricRange(0, 1, 0, angle / (2 * Math.PI));

            foreach (var kv in curveMeta)
            {
                string sideName = operationName + "-" + kv.Key;

                if (kv.Value.Instance is not Curve2D curve2d)
                {
                    result[sideName] = new SurfaceMetaData(SurfaceType.Unknown);
                    continue;
                }

                var classified = ClassifyRevolveCurve(curve2d, sketchFrame, axis, pointOnAxis, zeroDir);
                var profile = BuildProfileCurve(curve2d, sketchFrame, segmentPolylines, kv.Key);
                var axisCopy = axis;
                var zeroDirCopy = zeroDir;
                var pointOnAxisCopy = pointOnAxis;
                var vRangeCopy = vRange;
                var sideMeta = new SurfaceMetaData(classified.SurfaceType, () =>
                {
                    var revolveSurface = NurbsSurfaceFactory.Revolve(pointOnAxisCopy, axisCopy, profile, zeroDirCopy);
                    // Mesh V is normalized rotation angle, whereas a rational
                    // circle's knot parameter is not uniform in angle.
                    var circle = new BSplineCircle(pointOnAxisCopy, axisCopy, 1, zeroDirCopy);
                    return NurbsSurfaceFactory.WrapForMeshUv(revolveSurface, profile, vRangeCopy,
                        railCurveForArcLengthV: circle);
                }, vRange);
                sideMeta.PlaneParams = classified.PlaneParams;
                sideMeta.CylinderParams = classified.CylinderParams;
                sideMeta.ConeParams = classified.ConeParams;
                sideMeta.SphereParams = classified.SphereParams;
                sideMeta.TorusParams = classified.TorusParams;
                result[sideName] = sideMeta;
            }

            bool isFull = Math.Abs(angle - 2 * Math.PI) < 1e-10;
            if (!isFull)
            {
                NurbsSurfaceFactory.ProfileBoundingBox(allProfilePoints, out var uvMin, out var uvMax);
                var startCapFrame = new CoordinateSystem(sketchFrame.Origin, sketchFrame.Y, sketchFrame.Z, sketchFrame.X);
                result[EntityNaming.RevolveStartCap(operationName)] =
                    BuildPlanarCapMeta(startCapFrame, uvMin, uvMax);
                result[EntityNaming.RevolveEndCap(operationName)] =
                    BuildPlanarCapMeta(RotateFrameAboutAxis(startCapFrame, axis, angle), uvMin, uvMax);
            }

            return result;
        }

        public static Dictionary<string, SurfaceMetaData> BuildSweepMetadata(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem profileFrame,
            Vec3D guideStart,
            Vec3D guideEnd,
            string operationName,
            bool hasEndCaps,
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames)
        {
            var guideDir = (guideEnd - guideStart).Normalized();
            double length = (guideEnd - guideStart).Length();
            var segmentPolylines = ExtractNamedSegments(contour, contourNames);
            var profilePoints = CollectProfilePoints(contour);
            return BuildSweepMetadataCore(curveMeta, profileFrame, null, guideDir, length, operationName, hasEndCaps, segmentPolylines, profilePoints);
        }

        public static Dictionary<string, SurfaceMetaData> BuildSweepMetadata(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem profileFrame,
            IReadOnlyList<CoordinateSystem> guideFrames,
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames,
            string operationName,
            bool hasEndCaps,
            double twistRatePerExtrudeDistance)
        {
            var frames = guideFrames;
            if (Math.Abs(twistRatePerExtrudeDistance) > 1e-15 && guideFrames != null)
                frames = TwistGuideFrames(guideFrames, twistRatePerExtrudeDistance);

            var segmentPolylines = ExtractNamedSegments(contour, contourNames);
            var profilePoints = CollectProfilePoints(contour);
            return BuildSweepMetadataCore(curveMeta, profileFrame, frames, null, 0, operationName, hasEndCaps, segmentPolylines, profilePoints);
        }

        public static Dictionary<string, SurfaceMetaData> BuildSweepStripMetadata(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem profileFrame,
            IReadOnlyList<IReadOnlyList<CoordinateSystem>> guideFrameSegments,
            IReadOnlyList<string> guideCurveNames,
            List<List<List<Vec2D>>> contour,
            List<List<string>> contourNames,
            string operationName,
            bool hasEndCaps,
            double twistRatePerExtrudeDistance)
        {
            IReadOnlyList<IReadOnlyList<CoordinateSystem>> segments = guideFrameSegments;
            if (Math.Abs(twistRatePerExtrudeDistance) > 1e-15)
                segments = TwistGuideFrameSegments(guideFrameSegments, twistRatePerExtrudeDistance);

            bool hasTwist = Math.Abs(twistRatePerExtrudeDistance) > 1e-15;
            var segmentPolylines = ExtractNamedSegments(contour, contourNames);
            var result = new Dictionary<string, SurfaceMetaData>();
            var allFrames = segments.SelectMany(s => s).ToList();
            var sharedRailMapper = new LazySharedRailMapper(allFrames);

            foreach (var kv in curveMeta)
            {
                if (kv.Value.Instance is not Curve2D curve2d)
                {
                    foreach (var guideName in guideCurveNames)
                        result[EntityNaming.ExtrudeSideWithGuide(operationName, kv.Key, guideName)] =
                            new SurfaceMetaData(SurfaceType.Unknown);
                    continue;
                }

                var profileForU = BuildProfileCurve(curve2d, profileFrame, segmentPolylines, kv.Key);
                var surfaceType = hasTwist ? SurfaceType.Unknown : SurfaceTypeFromCurve(kv.Value.CurveType);
                var circle2d = hasTwist ? null : curve2d as Circle2D;
                for (int gi = 0; gi < segments.Count; gi++)
                {
                    string sideName = EntityNaming.ExtrudeSideWithGuide(operationName, kv.Key, guideCurveNames[gi]);
                    var frames = segments[gi];
                    var profileFrameCopy = profileFrame;
                    var sideMeta = new SurfaceMetaData(surfaceType, () =>
                    {
                        var surface = NurbsSurfaceFactory.SweepProfileAlongFrames(profileForU, profileFrameCopy, frames);
                        return NurbsSurfaceFactory.WrapForMeshUv(surface, profileForU, null, sharedRailMapper.GetOrCreate());
                    });
                    if (circle2d != null && TryBuildCircleSweepCylinder(circle2d, frames, out var cylinder))
                        sideMeta.CylinderParams = cylinder;
                    result[sideName] = sideMeta;
                }
            }

            if (hasEndCaps)
                AddSweepCaps(result, allFrames, operationName, CollectProfilePoints(contour));
            return result;
        }

        public static Dictionary<string, SurfaceMetaData> BuildLoftMetadata(
            IReadOnlyList<PlotterSketcherCoordSys> sketches, double maxDeviation,
            LoftOptions options, string operationName)
            => BuildLoftMetadata(sketches, maxDeviation, options, operationName, null);

        internal static Dictionary<string, SurfaceMetaData> BuildLoftMetadata(
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            double maxDeviation,
            LoftOptions options,
            string operationName,
            Func<INurbsSurface> loftSideSupportFactory,
            IReadOnlyDictionary<string, ParametricRange> sideDomains = null)
        {
            var result = new Dictionary<string, SurfaceMetaData>();
            if (sketches == null || sketches.Count < 2)
            {
                result[EntityNaming.LoftSide(operationName)] = new SurfaceMetaData(SurfaceType.Unknown);
                return result;
            }

            int samplesU = Math.Max(2, options.ProfileSamplesU);
            var loftStyle = options.Style;
            var sketchesCopy = sketches;
            if (loftSideSupportFactory != null)
            {
                result[EntityNaming.LoftSide(operationName)] =
                    new SurfaceMetaData(SurfaceType.Unknown, loftSideSupportFactory, ParametricRange.UnitSquare);
            }
            else if (TryBuildCompatibleLoftProfiles(sketches, out var exactProfiles))
            {
                result[EntityNaming.LoftSide(operationName)] =
                    new SurfaceMetaData(SurfaceType.Unknown, () =>
                    {
                        var loft = NurbsSurfaceFactory.LoftFromProfileCurves(exactProfiles, loftStyle);
                        return NurbsSurfaceFactory.WrapForMeshUv(loft, exactProfiles[0]);
                    }, ParametricRange.UnitSquare);
            }
            else
            {
                result[EntityNaming.LoftSide(operationName)] =
                    new SurfaceMetaData(SurfaceType.Unknown, () =>
                    {
                        var profileSamples = SampleLoftProfiles(sketchesCopy, maxDeviation, samplesU);
                        return NurbsSurfaceFactory.LoftFromProfileSamples(profileSamples, loftStyle);
                    }, ParametricRange.UnitSquare);
            }

            if (sideDomains != null)
            {
                result.Remove(EntityNaming.LoftSide(operationName));
                foreach (var face in sideDomains)
                    result.Add(face.Key, new SurfaceMetaData(SurfaceType.Unknown,
                        loftSideSupportFactory, face.Value));
            }

            if (options.CapEnds)
            {
                var firstSketch = sketches[0];
                var lastSketch = sketches[sketches.Count - 1];
                var firstPoints = CollectSketchProfilePoints(firstSketch, maxDeviation);
                var lastPoints = CollectSketchProfilePoints(lastSketch, maxDeviation);
                NurbsSurfaceFactory.ProfileBoundingBox(firstPoints, out var uvMin0, out var uvMax0);
                NurbsSurfaceFactory.ProfileBoundingBox(lastPoints, out var uvMin1, out var uvMax1);
                result[EntityNaming.LoftStartCap(operationName)] =
                    BuildPlanarCapMeta(firstSketch.CoordinateSystem, uvMin0, uvMax0);
                result[EntityNaming.LoftEndCap(operationName)] =
                    BuildPlanarCapMeta(lastSketch.CoordinateSystem, uvMin1, uvMax1);
            }

            return result;
        }

        private static Dictionary<string, SurfaceMetaData> BuildSweepMetadataCore(
            Dictionary<string, CurveMetaData> curveMeta,
            CoordinateSystem profileFrame,
            IReadOnlyList<CoordinateSystem> guideFrames,
            Vec3D? linearGuideDir,
            double linearGuideLength,
            string operationName,
            bool hasEndCaps,
            Dictionary<string, List<Vec2D>> segmentPolylines,
            IEnumerable<Vec2D> capProfilePoints)
        {
            var result = new Dictionary<string, SurfaceMetaData>();
            LazySharedRailMapper sharedRailMapper = guideFrames != null
                ? new LazySharedRailMapper(guideFrames)
                : null;

            foreach (var kv in curveMeta)
            {
                string sideName = EntityNaming.ExtrudeSide(operationName, kv.Key);
                if (kv.Value.Instance is not Curve2D curve2d)
                {
                    result[sideName] = new SurfaceMetaData(SurfaceType.Unknown);
                    continue;
                }

                if (linearGuideDir.HasValue)
                {
                    var profile = BuildProfileCurve(curve2d, profileFrame, segmentPolylines, kv.Key);
                    var guideDir = linearGuideDir.Value;
                    var guideLength = linearGuideLength;
                    var sideMeta = new SurfaceMetaData(SurfaceTypeFromCurve(kv.Value.CurveType), () =>
                    {
                        var surface = NurbsSurfaceFactory.LinearExtrude(profile, guideDir, guideLength);
                        return NurbsSurfaceFactory.WrapLinearExtrudeForMeshUv(surface, profile);
                    });
                    if (curve2d is Circle2D linearCircle)
                        sideMeta.CylinderParams = BuildCylinderParams(linearCircle, profileFrame, guideDir, guideLength);
                    result[sideName] = sideMeta;
                }
                else if (guideFrames != null)
                {
                    var profileForU = BuildProfileCurve(curve2d, profileFrame, segmentPolylines, kv.Key);
                    var railMapper = sharedRailMapper;
                    var profileFrameCopy = profileFrame;
                    var sideMeta = new SurfaceMetaData(SurfaceTypeFromCurve(kv.Value.CurveType), () =>
                    {
                        var surface = NurbsSurfaceFactory.SweepProfileAlongFrames(profileForU, profileFrameCopy, guideFrames);
                        return NurbsSurfaceFactory.WrapForMeshUv(surface, profileForU, null, railMapper.GetOrCreate());
                    });
                    if (curve2d is Circle2D sweepCircle &&
                        TryBuildCircleSweepCylinder(sweepCircle, guideFrames, out var cylinder))
                        sideMeta.CylinderParams = cylinder;
                    result[sideName] = sideMeta;
                }
                else
                {
                    result[sideName] = new SurfaceMetaData(SurfaceTypeFromCurve(kv.Value.CurveType));
                }
            }

            if (hasEndCaps && guideFrames != null && guideFrames.Count >= 2)
                AddSweepCaps(result, guideFrames, operationName, capProfilePoints);
            else if (hasEndCaps && linearGuideDir.HasValue && capProfilePoints != null)
                AddLinearSweepCaps(result, profileFrame, linearGuideDir.Value, linearGuideLength, operationName, capProfilePoints);
            else if (hasEndCaps && linearGuideDir.HasValue)
            {
                result[EntityNaming.ExtrudeBottom(operationName)] = new SurfaceMetaData(SurfaceType.Planar);
                result[EntityNaming.ExtrudeTop(operationName)] = new SurfaceMetaData(SurfaceType.Planar);
            }

            return result;
        }

        private static void AddSweepCaps(
            Dictionary<string, SurfaceMetaData> result,
            IReadOnlyList<CoordinateSystem> guideFrames,
            string operationName,
            IEnumerable<Vec2D> profilePoints)
        {
            if (guideFrames == null || guideFrames.Count < 2)
                return;

            if (profilePoints != null)
            {
                NurbsSurfaceFactory.ProfileBoundingBox(profilePoints, out var uvMin, out var uvMax);
                result[EntityNaming.ExtrudeBottom(operationName)] =
                    BuildPlanarCapMeta(guideFrames[0], uvMin, uvMax);
                result[EntityNaming.ExtrudeTop(operationName)] =
                    BuildPlanarCapMeta(guideFrames[guideFrames.Count - 1], uvMin, uvMax);
            }
            else
            {
                result[EntityNaming.ExtrudeBottom(operationName)] = new SurfaceMetaData(SurfaceType.Planar);
                result[EntityNaming.ExtrudeTop(operationName)] = new SurfaceMetaData(SurfaceType.Planar);
            }
        }

        private static void AddLinearSweepCaps(
            Dictionary<string, SurfaceMetaData> result,
            CoordinateSystem profileFrame,
            Vec3D guideDir,
            double guideLength,
            string operationName,
            IEnumerable<Vec2D> profilePoints)
        {
            NurbsSurfaceFactory.ProfileBoundingBox(profilePoints, out var uvMin, out var uvMax);
            var endFrame = new CoordinateSystem(
                profileFrame.Origin + guideDir * guideLength,
                profileFrame.X, profileFrame.Y, profileFrame.Z);
            result[EntityNaming.ExtrudeBottom(operationName)] =
                BuildPlanarCapMeta(profileFrame, uvMin, uvMax);
            result[EntityNaming.ExtrudeTop(operationName)] =
                BuildPlanarCapMeta(endFrame, uvMin, uvMax);
        }

        private static bool TryBuildCompatibleLoftProfiles(
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            out List<BSplineCurve> profiles)
        {
            profiles = null;
            var curvesPerSketch = new List<List<Curve2D>>(sketches.Count);
            foreach (var sketch in sketches)
            {
                var curves = CollectSketchCurves(sketch);
                if (curves == null || curves.Count == 0)
                    return false;
                curvesPerSketch.Add(curves);
            }

            int segmentCount = curvesPerSketch[0].Count;
            for (int i = 1; i < curvesPerSketch.Count; i++)
            {
                if (curvesPerSketch[i].Count != segmentCount)
                    return false;
            }

            var converted = new List<BSplineCurve>[sketches.Count];
            for (int s = 0; s < sketches.Count; s++)
            {
                converted[s] = new List<BSplineCurve>(segmentCount);
                var frame = sketches[s].CoordinateSystem;
                for (int i = 0; i < segmentCount; i++)
                    converted[s].Add(Curve2DToBSpline.ToBSplineCurve(curvesPerSketch[s][i], frame));
            }

            for (int i = 0; i < segmentCount; i++)
            {
                int degree = converted[0][i].Degree;
                int cps = converted[0][i].ControlPoints.Length;
                for (int s = 1; s < converted.Length; s++)
                {
                    if (converted[s][i].Degree != degree || converted[s][i].ControlPoints.Length != cps)
                        return false;
                }
            }

            profiles = new List<BSplineCurve>(sketches.Count);
            for (int s = 0; s < converted.Length; s++)
                profiles.Add(NurbsSurfaceFactory.ConcatenateCompatible(converted[s]));
            return profiles.Count >= 2;
        }

        private static List<Curve2D> CollectSketchCurves(PlotterSketcherCoordSys sketch)
        {
            var strips = sketch.GetCurves();
            if (strips == null || strips.Count == 0)
                return null;

            var curves = new List<Curve2D>();
            foreach (var strip in strips)
            {
                if (strip == null)
                    continue;
                foreach (var curve in strip)
                {
                    if (curve == null || curve.IsHelperGeometry)
                        continue;
                    curves.Add(curve);
                }
            }

            return curves;
        }

        private static List<List<Vec3D>> SampleLoftProfiles(
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            double maxDeviation,
            int samplesU)
        {
            var profiles = new List<List<Vec3D>>(sketches.Count);
            foreach (var sketch in sketches)
            {
                var fromCurves = SampleSketchFromExactCurves(sketch, samplesU);
                if (fromCurves != null && fromCurves.Count >= 2)
                    profiles.Add(fromCurves);
                else
                {
                    var loop = CollectSketchProfilePolyline(sketch, maxDeviation);
                    profiles.Add(ResamplePolylineUniformArcLength(loop, sketch.CoordinateSystem, samplesU));
                }
            }
            return profiles;
        }

        private static List<Vec3D> SampleSketchFromExactCurves(PlotterSketcherCoordSys sketch, int samplesU)
        {
            var curves = CollectSketchCurves(sketch);
            if (curves == null || curves.Count == 0)
                return null;

            var lengths = new double[curves.Count];
            double total = 0;
            for (int i = 0; i < curves.Count; i++)
            {
                lengths[i] = Math.Max(1e-12, curves[i].Length());
                total += lengths[i];
            }

            var points = new List<Vec3D>(samplesU);
            var frame = sketch.CoordinateSystem;
            int remaining = samplesU;
            for (int i = 0; i < curves.Count; i++)
            {
                int n = i == curves.Count - 1
                    ? remaining
                    : Math.Max(2, (int)Math.Round(samplesU * (lengths[i] / total)));
                remaining -= n;
                var spline = Curve2DToBSpline.ToBSplineCurve(curves[i], frame);
                int start = i == 0 ? 0 : 1;
                for (int s = start; s < n; s++)
                {
                    double u = n == 1 ? 0 : (double)s / (n - 1);
                    points.Add(spline.EvaluateUniform(u));
                }
            }

            return points.Count >= 2 ? points : null;
        }

        private static List<Vec2D> CollectSketchProfilePolyline(PlotterSketcherCoordSys sketch, double maxDeviation)
        {
            var contour = sketch.Tessellate(maxDeviation, out _, out _, out _,
                maxDeviation, SketchTessellationFlags.ExcludeHelperGeometry);
            var points = new List<Vec2D>();
            foreach (var strip in contour)
            {
                foreach (var loop in strip)
                {
                    for (int i = 0; i < loop.Count - 1; i++)
                        points.Add(loop[i]);
                }
            }
            return points;
        }

        private static IEnumerable<Vec2D> CollectSketchProfilePoints(PlotterSketcherCoordSys sketch, double maxDeviation)
        {
            return CollectSketchProfilePolyline(sketch, maxDeviation);
        }

        private static List<Vec3D> ResamplePolylineUniformArcLength(
            IReadOnlyList<Vec2D> polyline, CoordinateSystem frame, int count)
        {
            if (polyline.Count < 2)
                throw new ArgumentException("Profile polyline too short.");

            var strip = new LineStrip2D(new List<Vec2D>(polyline));
            var result = new List<Vec3D>(count);
            for (int i = 0; i < count; i++)
            {
                double s = count == 1 ? 0 : (double)i / (count - 1);
                double dist = s * strip.TotalLength;
                var p2d = strip.Evaluate(dist);
                result.Add(frame.PointTo3D(p2d));
            }
            return result;
        }

        private static CoordinateSystem OffsetFrameAlongZ(CoordinateSystem frame, double z)
        {
            return new CoordinateSystem(
                frame.PointFromCoordSysToWorld(new Vec3D(0, 0, z)),
                frame.X, frame.Y, frame.Z);
        }

        private static CoordinateSystem RotateFrameAboutAxis(CoordinateSystem frame, Vec3D axis, double angle)
        {
            var x = RotateVector(frame.X, axis, angle);
            var y = RotateVector(frame.Y, axis, angle);
            var z = RotateVector(frame.Z, axis, angle);
            z = z.Normalized();
            x = x - z * Vec3DOps.Dot(x, z);
            if (x.LengthSquared() < 1e-16)
            {
                x = y - z * Vec3DOps.Dot(y, z);
            }
            x = x.Normalized();
            y = Vec3DOps.Cross(z, x).Normalized();
            return new CoordinateSystem(frame.Origin, x, y, z);
        }

        private static Vec3D RotateVector(Vec3D v, Vec3D axis, double angle)
        {
            var k = axis.Normalized();
            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);
            return v * cos + Vec3DOps.Cross(k, v) * sin + k * Vec3DOps.Dot(k, v) * (1 - cos);
        }

        private static BSplineCurve BuildProfileCurve(
            Curve2D curve,
            CoordinateSystem frame,
            Dictionary<string, List<Vec2D>> segmentPolylines,
            string segmentKey)
        {
            if (curve is Line2D &&
                segmentPolylines != null &&
                segmentPolylines.TryGetValue(segmentKey, out var polyline) &&
                polyline.Count >= 2)
            {
                return Curve2DToBSpline.FromPolylineEndpoints(polyline, frame);
            }
            // Contour winding can reverse a segment while its metadata still
            // refers to the authored curve. Mesh U follows that ordered segment;
            // the analytic support must follow it too (including end tangents).
            if (segmentPolylines != null && segmentPolylines.TryGetValue(segmentKey, out var samples) &&
                samples.Count >= 2 && curve.StartPosition != curve.EndPosition)
            {
                double forward = Vec2DOps.DistanceSquared(samples[0], curve.StartPosition) +
                                 Vec2DOps.DistanceSquared(samples[^1], curve.EndPosition);
                double reverse = Vec2DOps.DistanceSquared(samples[0], curve.EndPosition) +
                                 Vec2DOps.DistanceSquared(samples[^1], curve.StartPosition);
                if (reverse < forward) curve = curve.Reverse();
            }
            return Curve2DToBSpline.ToBSplineCurve(curve, frame);
        }

        private static SurfaceMetaData BuildPlanarCapMeta(CoordinateSystem frame, Vec2D uvMin, Vec2D uvMax)
        {
            return new SurfaceMetaData(
                SurfaceType.Planar,
                () => NurbsSurfaceFactory.PlanarCapWithMeshUv(frame, uvMin, uvMax, 0),
                ParametricRange.UnitSquare)
            {
                PlaneParams = PlaneParamsFromFrame(frame)
            };
        }

        private static PlaneSurfaceParams PlaneParamsFromFrame(CoordinateSystem frame)
        {
            return new PlaneSurfaceParams
            {
                Origin = frame.Origin,
                Normal = frame.Z.Normalized(),
                RefDir = frame.X.Normalized()
            };
        }

        private static bool TryBuildCircleSweepCylinder(
            Circle2D circle,
            IReadOnlyList<CoordinateSystem> frames,
            out CylinderSurfaceParams cylinder)
        {
            cylinder = null;
            if (circle == null || frames == null || frames.Count < 2)
                return false;

            CoordinateSystem start = frames[0];
            CoordinateSystem end = frames[frames.Count - 1];
            Vec3D axis = end.Origin - start.Origin;
            double height = axis.Length();
            if (height < 1e-12)
                return false;
            axis *= 1.0 / height;

            for (int i = 0; i < frames.Count; i++)
            {
                Vec3D z = frames[i].Z;
                double zLen = z.Length();
                if (zLen < 1e-12)
                    return false;
                z *= 1.0 / zLen;
                if (Math.Abs(Vec3DOps.Dot(z, axis)) < 0.999)
                    return false;

                Vec3D delta = frames[i].Origin - start.Origin;
                Vec3D offAxis = delta - axis * Vec3DOps.Dot(delta, axis);
                if (offAxis.LengthSquared() > 1e-8)
                    return false;
            }

            cylinder = BuildCylinderParams(circle, start, axis, height);
            return true;
        }

        private static CylinderSurfaceParams BuildCylinderParams(
            Circle2D circle,
            CoordinateSystem bottomFrame,
            Vec3D extrudeDir,
            double extrudeLength)
            => BuildCylinderParams(circle.Center, circle.StartPosition, circle.Radius,
                bottomFrame, extrudeDir, extrudeLength);

        private static CylinderSurfaceParams BuildCylinderParams(
            Vec2D profileCenter,
            Vec2D profileStart,
            double radius,
            CoordinateSystem bottomFrame,
            Vec3D extrudeDir,
            double extrudeLength)
        {
            var center = bottomFrame.PointTo3D(profileCenter);
            var refDir = bottomFrame.PointTo3D(profileStart) - center;
            if (refDir.Length() < 1e-12)
                refDir = bottomFrame.X;
            else
                refDir = refDir.Normalized();

            return new CylinderSurfaceParams
            {
                Origin = center,
                Axis = extrudeDir.Normalized(),
                RefDir = refDir,
                Radius = radius,
                Height = extrudeLength
            };
        }

        private static SurfaceType SurfaceTypeFromCurve(CurveType curveType) => curveType switch
        {
            CurveType.Line2D => SurfaceType.Planar,
            CurveType.Circle2D => SurfaceType.Cylindrical,
            CurveType.Arc2D => SurfaceType.Cylindrical,
            _ => SurfaceType.Unknown
        };

        private sealed class RevolveClassification
        {
            public SurfaceType SurfaceType;
            public PlaneSurfaceParams PlaneParams;
            public CylinderSurfaceParams CylinderParams;
            public ConeSurfaceParams ConeParams;
            public SphereSurfaceParams SphereParams;
            public TorusSurfaceParams TorusParams;
        }

        private static RevolveClassification ClassifyRevolveCurve(
            Curve2D curve,
            CoordinateSystem sketchFrame,
            Vec3D axis,
            Vec3D pointOnAxis,
            Vec3D zeroDir)
        {
            var result = new RevolveClassification { SurfaceType = SurfaceType.Unknown };
            axis = axis.Normalized();
            zeroDir = OrthonormalizeRef(axis, zeroDir);
            const double alignTol = 1e-6;

            if (curve is Line2D line)
            {
                var p0 = sketchFrame.PointTo3D(line.StartPosition);
                var p1 = sketchFrame.PointTo3D(line.EndPosition);
                var dir = p1 - p0;
                double dirLen = dir.Length();
                if (dirLen < 1e-12)
                    return result;
                dir *= 1.0 / dirLen;

                double along = Math.Abs(Vec3DOps.Dot(dir, axis));
                double r0 = RadiusFromAxis(p0, pointOnAxis, axis);
                double r1 = RadiusFromAxis(p1, pointOnAxis, axis);

                if (along > 1.0 - alignTol && Math.Abs(r0 - r1) < 1e-8)
                {
                    double h0 = Vec3DOps.Dot(p0 - pointOnAxis, axis);
                    double h1 = Vec3DOps.Dot(p1 - pointOnAxis, axis);
                    double z0 = Math.Min(h0, h1);
                    double height = Math.Abs(h1 - h0);
                    var origin = pointOnAxis + axis * z0;
                    result.SurfaceType = SurfaceType.Cylindrical;
                    result.CylinderParams = new CylinderSurfaceParams
                    {
                        Origin = origin,
                        Axis = axis,
                        RefDir = zeroDir,
                        Radius = 0.5 * (r0 + r1),
                        Height = height
                    };
                    return result;
                }

                if (along < alignTol)
                {
                    var origin = pointOnAxis + axis * Vec3DOps.Dot(p0 - pointOnAxis, axis);
                    result.SurfaceType = SurfaceType.Planar;
                    result.PlaneParams = new PlaneSurfaceParams
                    {
                        Origin = origin,
                        Normal = axis,
                        RefDir = zeroDir
                    };
                    return result;
                }

                var apex = EstimateConeApex(p0, p1, pointOnAxis, axis);
                double semi = Math.Acos(Math.Min(1, Math.Max(-1, along)));
                if (semi > Math.PI * 0.5)
                    semi = Math.PI - semi;
                double hStart = Vec3DOps.Dot(p0 - apex, axis);
                double radiusAtOrigin = RadiusFromAxis(p0, apex, axis);
                result.SurfaceType = SurfaceType.Conical;
                result.ConeParams = new ConeSurfaceParams
                {
                    Origin = p0,
                    Axis = Math.Sign(hStart) >= 0 ? axis : axis * -1,
                    RefDir = zeroDir,
                    Radius = radiusAtOrigin,
                    SemiAngle = semi,
                    Height = dirLen * along
                };
                return result;
            }

            if (curve is Arc2D arc)
            {
                var center = sketchFrame.PointTo3D(arc.Center);
                double offAxis = RadiusFromAxis(center, pointOnAxis, axis);
                if (offAxis < 1e-8)
                {
                    result.SurfaceType = SurfaceType.Spherical;
                    result.SphereParams = new SphereSurfaceParams
                    {
                        Center = center,
                        Axis = axis,
                        RefDir = zeroDir,
                        Radius = arc.Radius
                    };
                    return result;
                }

                result.SurfaceType = SurfaceType.Toroidal;
                result.TorusParams = new TorusSurfaceParams
                {
                    Center = ProjectOntoAxis(center, pointOnAxis, axis),
                    Axis = axis,
                    RefDir = zeroDir,
                    MajorRadius = offAxis,
                    MinorRadius = arc.Radius
                };
                return result;
            }

            if (curve is Circle2D circle)
            {
                var center = sketchFrame.PointTo3D(circle.Center);
                double offAxis = RadiusFromAxis(center, pointOnAxis, axis);
                result.SurfaceType = SurfaceType.Toroidal;
                result.TorusParams = new TorusSurfaceParams
                {
                    Center = ProjectOntoAxis(center, pointOnAxis, axis),
                    Axis = axis,
                    RefDir = zeroDir,
                    MajorRadius = offAxis,
                    MinorRadius = circle.Radius
                };
                return result;
            }

            return result;
        }

        private static Vec3D EstimateConeApex(Vec3D p0, Vec3D p1, Vec3D pointOnAxis, Vec3D axis)
        {
            double r0 = RadiusFromAxis(p0, pointOnAxis, axis);
            double r1 = RadiusFromAxis(p1, pointOnAxis, axis);
            double h0 = Vec3DOps.Dot(p0 - pointOnAxis, axis);
            double h1 = Vec3DOps.Dot(p1 - pointOnAxis, axis);
            if (Math.Abs(r1 - r0) < 1e-12)
                return pointOnAxis + axis * h0;
            double t = r0 / (r0 - r1);
            double hApex = h0 + t * (h1 - h0);
            return pointOnAxis + axis * hApex;
        }

        private static double RadiusFromAxis(Vec3D point, Vec3D pointOnAxis, Vec3D axis)
        {
            var d = point - pointOnAxis;
            var axial = axis * Vec3DOps.Dot(d, axis);
            return (d - axial).Length();
        }

        private static Vec3D ProjectOntoAxis(Vec3D point, Vec3D pointOnAxis, Vec3D axis)
        {
            return pointOnAxis + axis * Vec3DOps.Dot(point - pointOnAxis, axis);
        }

        private static Vec3D OrthonormalizeRef(Vec3D axis, Vec3D refDir)
        {
            var r = refDir - axis * Vec3DOps.Dot(refDir, axis);
            return r.LengthSquared() < 1e-16 ? Perpendicular(axis) : r.Normalized();
        }

        private static Vec3D Perpendicular(Vec3D axis)
        {
            var a = Math.Abs(axis.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
            return Vec3DOps.Cross(axis, a).Normalized();
        }

        private static List<CoordinateSystem> BuildTwistedFrames(
            CoordinateSystem bottomFrame, Vec3D extrudeDir, double length, double twistRate)
        {
            double twistAngle = Math.Abs(twistRate * length);
            int n = Math.Max(4, (int)Math.Ceiling(twistAngle / (Math.PI / 6.0)) + 1);
            var frames = new List<CoordinateSystem>(n + 1);
            var axis = extrudeDir.Normalized();
            for (int i = 0; i <= n; i++)
            {
                double t = (double)i / n;
                double z = t * length;
                double angle = twistRate * z;
                var x = RotateVector(bottomFrame.X, axis, angle);
                x = x - axis * Vec3DOps.Dot(x, axis);
                x = x.Normalized();
                var y = Vec3DOps.Cross(axis, x).Normalized();
                frames.Add(new CoordinateSystem(bottomFrame.Origin + axis * z, x, y, axis));
            }
            return frames;
        }

        private static List<CoordinateSystem> TwistGuideFrames(
            IReadOnlyList<CoordinateSystem> frames, double twistRatePerDistance)
        {
            var result = new List<CoordinateSystem>(frames.Count);
            double arc = 0;
            for (int i = 0; i < frames.Count; i++)
            {
                if (i > 0)
                    arc += (frames[i].Origin - frames[i - 1].Origin).Length();
                double angle = twistRatePerDistance * arc;
                var f = frames[i];
                var z = f.Z.Normalized();
                var x = RotateVector(f.X, z, angle);
                x = x - z * Vec3DOps.Dot(x, z);
                x = x.Normalized();
                var y = Vec3DOps.Cross(z, x).Normalized();
                result.Add(new CoordinateSystem(f.Origin, x, y, z));
            }
            return result;
        }

        private static List<IReadOnlyList<CoordinateSystem>> TwistGuideFrameSegments(
            IReadOnlyList<IReadOnlyList<CoordinateSystem>> segments, double twistRatePerDistance)
        {
            var all = segments.SelectMany(s => s).ToList();
            var twisted = TwistGuideFrames(all, twistRatePerDistance);
            var result = new List<IReadOnlyList<CoordinateSystem>>(segments.Count);
            int offset = 0;
            foreach (var seg in segments)
            {
                result.Add(twisted.GetRange(offset, seg.Count));
                offset += seg.Count;
            }
            return result;
        }
    }
}
