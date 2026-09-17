using System;
using System.Collections.Generic;
using System.Linq;
using Curves;
using GeoCore;
using GeoMeta;

namespace Geo
{
    public enum LoftStyle
    {
        Ruled,
        SmoothCatmullRom,
        /// <summary>
        /// Smooth loft in v using a cubic Hermite spline through the profile samples at each shared u column.
        /// U-columns merge uniform samples with normalized arc-length parameters from per-segment <c>Tessellate(maxDeviation)</c> on each
        /// analytic strip segment at the loft <c>maxDeviation</c> (sketch path), so chord-wise accuracy matches the tessellation tolerance.
        /// </summary>
        Hermite
    }

    /// <summary>
    /// Options for loft mesh generation between ordered profile sketches (single curve strip per sketch, no holes in v1).
    /// </summary>
    public sealed class LoftOptions
    {
        public LoftStyle Style { get; set; }
        /// <summary>
        /// Zero (default) uses tolerance-driven profile anchors without extra uniform samples.
        /// For <see cref="LoftStyle.Ruled"/> / <see cref="LoftStyle.SmoothCatmullRom"/>: minimum number of uniform u samples; the actual grid merges these with profile anchors per <see cref="LoftCorrespondenceMode"/>.
        /// For <see cref="LoftStyle.Hermite"/>: uniform u samples are always merged with max-deviation tessellation anchors (and optional feature anchors); this is the minimum uniform count.
        /// </summary>
        public int ProfileSamplesU { get; set; }
        /// <summary>
        /// Extra uniform v samples (in global Hermite parameter) merged into the v-grid. For sketch lofts with
        /// <see cref="LoftStyle.Hermite"/>, longitudinal spacing is primarily driven by <c>Tessellate(maxDeviation)</c> on each
        /// u-column spline (same <c>maxDeviation</c> as the loft call); this value adds a minimum uniform refinement per span.
        /// 0 with no adaptive v (e.g. polyline loft) = rows only on profiles ? straight v-lines for Hermite/Catmull-Rom.
        /// </summary>
        public int VSubdivisionsPerSpan { get; set; }
        /// <summary>When true and the profile is closed, triangulates the first and last loft sections as caps.</summary>
        public bool CapEnds { get; set; }
        /// <summary>Pass through to sketch tessellation for open contours.</summary>
        public bool AllowOpenContour { get; set; }
        /// <summary>Absolute tolerance for merging normalized u in [0,1] (combined with <see cref="LoftUmergeToleranceRel"/> as max of the two).</summary>
        public double LoftUmergeToleranceAbs { get; set; }
        /// <summary>Relative tolerance (multiplied by u-span 1.0); merged u uses max(<see cref="LoftUmergeToleranceAbs"/>, this).</summary>
        public double LoftUmergeToleranceRel { get; set; }
        /// <summary>Cap on tessellation-derived merged u anchors for <see cref="LoftStyle.Hermite"/> (after cross-profile merge); 0 = no cap.</summary>
        public int MaxMergedUAnchors { get; set; }

        /// <summary>How shared u-columns are assembled from profiles (default: merged tessellation anchors + uniform).</summary>
        public LoftCorrespondenceMode CorrespondenceMode { get; set; }
        /// <summary>Which profiles supply crease columns at segment joints.</summary>
        public LoftCreasePolicy CreasePolicy { get; set; }
        /// <summary>
        /// How closed profiles choose u=0 (seam). Default <see cref="LoftAlignmentMode.AsAuthored"/>
        /// preserves the section start points as authored connectors.
        /// Open profiles always keep authored start/end (u0=0). Per-profile <see cref="ProfileSeamPoints"/> override this when set.
        /// </summary>
        public LoftAlignmentMode AlignmentMode { get; set; }
        /// <summary>
        /// Optional seam hints in each profile's sketch XY (one entry per loft profile). Entries with <see cref="LoftProfileSeamHint.HasPoint"/>
        /// set u=0 to the closest point on that closed profile; unset entries use <see cref="AlignmentMode"/>. Ignored for open profiles.
        /// </summary>
        public IList<LoftProfileSeamHint> ProfileSeamPoints { get; set; }
        /// <summary>Optional first edge name per section for matched-vertex correspondence.</summary>
        public IReadOnlyList<string> FirstCurves { get; set; }
        /// <summary>
        /// Planar end-cap preflight strictness; caps triangulate the sanitized rim via integer-scaled sketch coordinates (ear clip + Delaunay post-process).
        /// Default <see cref="LoftCapTriangulationMode.Robust"/> tolerates borderline winding on NACA / near-degenerate trailing-edge rims.
        /// Use <see cref="LoftCapTriangulationMode.EarClipping"/> only when strict winding preflight is required.
        /// </summary>
        public LoftCapTriangulationMode CapTriangulation { get; set; }
        /// <summary>
        /// The default <see cref="LoftProfileSamplingSource.AnalyticCurveStrip"/> evaluates the authored curves.
        /// Each segment occupies a length-proportional interval, using its native parameter within that interval
        /// (see <see cref="CurveStrip2D.FromSegments"/>); tessellation supplies creases and optional anchors.
        /// Seam u0 is applied consistently to analytic evaluation.
        /// </summary>
        public LoftProfileSamplingSource ProfileSampling { get; set; }

        public LoftOptions()
        {
            Style = LoftStyle.Hermite;
            ProfileSamplesU = 0;
            VSubdivisionsPerSpan = 0;
            CapEnds = true;
            AllowOpenContour = false;
            LoftUmergeToleranceAbs = 1e-6;
            LoftUmergeToleranceRel = 1e-9;
            MaxMergedUAnchors = 0;
            CorrespondenceMode = LoftCorrespondenceMode.MergedArcLengthAnchors;
            CreasePolicy = LoftCreasePolicy.FromAllProfiles;
            AlignmentMode = LoftAlignmentMode.AsAuthored;
            ProfileSeamPoints = null;
            CapTriangulation = LoftCapTriangulationMode.Robust;
            ProfileSampling = LoftProfileSamplingSource.AnalyticCurveStrip;
        }

        public static LoftOptions Default
        {
            get { return new LoftOptions(); }
        }

        /// <summary>
        /// High-accuracy loft for multi-station NACA propeller blades:
        /// <see cref="LoftCorrespondenceMode.MergedArcLengthAnchors"/> so u-columns follow sketch
        /// <c>maxDeviation</c> tessellation; <see cref="LoftCreasePolicy.None"/> for smooth TE caps.
        /// Uses <see cref="LoftStyle.SmoothCatmullRom"/> for watertight skinning on tilted section frames.
        /// </summary>
        public static LoftOptions PropellerBlade
        {
            get
            {
                return new LoftOptions
                {
                    Style = LoftStyle.SmoothCatmullRom,
                    CorrespondenceMode = LoftCorrespondenceMode.MergedArcLengthAnchors,
                    CreasePolicy = LoftCreasePolicy.None,
                    CapEnds = true,
                    AllowOpenContour = false
                };
            }
        }
    }

    /// <summary>
    /// Lofts ordered 2D profiles into a triangle mesh: <see cref="GenerateLoftFromSketches"/> (sketches) and
    /// <see cref="GenerateLoftFromPolylines"/> (pre-built polylines). The static type is <c>LoftBuilder</c> to avoid clashing with
    /// <see cref="GeoAPI.Loft"/>.
    /// Each profile is defined in its own sketch plane.
    /// Correspondence rule: each profile uses a normalized length-proportional parameter u in [0,1] along its oriented strip
    /// after choosing a seam (u=0) via <see cref="LoftAlignmentMode"/> / <see cref="LoftOptions.ProfileSeamPoints"/>.
    /// All profiles are resampled on one shared u-grid in seam space (see <see cref="LoftCorrespondenceMode"/>).
    /// With <see cref="LoftProfileSamplingSource.AnalyticCurveStrip"/>, evaluation uses the analytic strip with the same seam u0.
    /// Closed profiles duplicate the first rim column (same 3D position, u = 1) for a clean UV seam. End caps
    /// share rim positions while retaining separate vertices for the sharp side-to-cap normal discontinuity.
    /// Sketch segment joints insert duplicate u-columns (crease left/right) when included via <see cref="LoftCreasePolicy"/>.
    /// </summary>
    public static partial class LoftBuilder
    {
        /// <summary>
        /// Loft from tessellated sketch profiles (one curve strip per sketch). Uses <paramref name="converter"/> for precise positions.
        /// </summary>
        public static int GenerateLoftFromSketches(
            CoordinateConverter converter,
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            double maxDeviation,
            LoftOptions options,
            MeshOutput output,
            string loftName,
            out Dictionary<int, string> triangleGroupToName,
            int baseGroupIndex = 0)
        {
            if (sketches == null || sketches.Count < 2)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            CollectPreparedProfilesFromSketches(sketches, maxDeviation, options, out var prepared);
            return GenerateLoftCore(converter, prepared, options, maxDeviation,
                output.Triangles, output.Vertices, output.Normals, output.UVs,
                output.TriangleGroups, output.PrecisePositions,
                loftName, out triangleGroupToName, baseGroupIndex, output);
        }

        /// <summary>Sketches overload that builds a <see cref="CoordinateConverter"/> from profile bounds.</summary>
        public static int GenerateLoftFromSketches(
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            double maxDeviation,
            LoftOptions options,
            MeshOutput output,
            string loftName,
            out Dictionary<int, string> triangleGroupToName,
            int baseGroupIndex = 0)
        {
            if (sketches == null || sketches.Count < 2)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            CollectPreparedProfilesFromSketches(sketches, maxDeviation, options, out var prepared);
            var polys = prepared.ConvertAll(p => p.Poly);
            var systems = prepared.ConvertAll(p => p.System);
            var converter = MakeConverterFromPolylines(polys, systems);
            return GenerateLoftCore(converter, prepared, options, maxDeviation,
                output.Triangles, output.Vertices, output.Normals, output.UVs,
                output.TriangleGroups, output.PrecisePositions,
                loftName, out triangleGroupToName, baseGroupIndex, output);
        }

        /// <summary>
        /// Loft from pre-flattened polylines (2D sketch coordinates) and a world-space plane per profile.
        /// No segment metadata: crease-aware duplicate columns are not inserted; <see cref="LoftProfileSamplingSource.AnalyticCurveStrip"/> is ignored.
        /// </summary>
        public static int GenerateLoftFromPolylines(
            CoordinateConverter converter,
            IReadOnlyList<List<Vec2D>> polylines2D,
            IReadOnlyList<List<Vec2D>> polylineNormals2D,
            IReadOnlyList<CoordinateSystem> profilePlanes,
            LoftOptions options,
            MeshOutput output,
            string loftName,
            out Dictionary<int, string> triangleGroupToName,
            int baseGroupIndex = 0)
        {
            if (polylines2D == null || polylineNormals2D == null || profilePlanes == null ||
                polylines2D.Count < 2 || polylines2D.Count != polylineNormals2D.Count || polylines2D.Count != profilePlanes.Count)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            var prepared = new List<LoftPreparedProfile>(polylines2D.Count);
            for (int i = 0; i < polylines2D.Count; i++)
            {
                prepared.Add(new LoftPreparedProfile
                {
                    Poly = new List<Vec2D>(polylines2D[i]),
                    Norms = new List<Vec2D>(polylineNormals2D[i]),
                    System = profilePlanes[i],
                    CreaseIdx = new List<int>(),
                    AnalyticStrip = null
                });
            }

            return GenerateLoftCore(converter, prepared, options, loftTessellationTolerance: 0,
                output.Triangles, output.Vertices, output.Normals, output.UVs,
                output.TriangleGroups, output.PrecisePositions,
                loftName, out triangleGroupToName, baseGroupIndex, output);
        }

        private static int GenerateLoftCore(
            CoordinateConverter converter,
            List<LoftPreparedProfile> prepared,
            LoftOptions options,
            double loftTessellationTolerance,
            List<Tri> triangles,
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<int> triangleGroups,
            List<Rat3Hybrid> precisePositions,
            string loftName,
            out Dictionary<int, string> triangleGroupToName,
            int baseGroupIndex,
            MeshOutput output)
        {
            if (options.FirstCurves != null)
            {
                if (options.FirstCurves.Count != prepared.Count)
                    throw new ArgumentException("first_curves must contain one curve name per section.");
                if (options.CorrespondenceMode != LoftCorrespondenceMode.MatchingVertices)
                    throw new ArgumentException("first_curves requires matching-vertex correspondence.");
                if (options.ProfileSeamPoints != null || options.AlignmentMode != LoftAlignmentMode.AsAuthored)
                    throw new ArgumentException("first_curves cannot be combined with another section alignment control.");
            }
            triangleGroupToName = new Dictionary<int, string>();
            int s = Math.Max(0, options.VSubdivisionsPerSpan);
            int pCount = prepared.Count;

            var polys = prepared.ConvertAll(p => p.Poly);
            var polyNormals = prepared.ConvertAll(p => p.Norms);
            var systems = prepared.ConvertAll(p => p.System);
            var creaseFlatPerProfile = prepared.ConvertAll(p => p.CreaseIdx);
            var analyticStrips = prepared.ConvertAll(p => p.AnalyticStrip);

            double tolU = Math.Max(options.LoftUmergeToleranceAbs, options.LoftUmergeToleranceRel);
            if (tolU <= 0)
                tolU = Math.Max(LoftOptions.Default.LoftUmergeToleranceAbs, LoftOptions.Default.LoftUmergeToleranceRel);

            bool[] profileClosed = new bool[pCount];
            var creaseIdxPerProfile = new List<List<int>>(pCount);
            var seamU0 = new double[pCount];
            List<Vec2D> prevPoly = null;
            List<Vec2D> prevNorms = null;
            bool prevClosed = false;
            CoordinateSystem? prevSystem = null;
            double prevSeamU0 = 0;

            for (int pi = 0; pi < pCount; pi++)
            {
                var crease = creaseFlatPerProfile[pi].Distinct().ToList();
                int nBeforePrepare = polys[pi].Count;
                PrepareProfilePolyline(polys[pi], polyNormals[pi], out bool closed, out int removedClosing);
                profileClosed[pi] = closed;
                if (polys[pi].Count < 2)
                    throw new InvalidOperationException("Loft profile has too few points.");

                crease = RemapCreaseAfterPrepare(crease, nBeforePrepare, polys[pi].Count, removedClosing);

                LoftProfileSeamHint seamHint = default;
                if (options.ProfileSeamPoints != null && pi < options.ProfileSeamPoints.Count)
                    seamHint = options.ProfileSeamPoints[pi];

                // Continuous seam target in current (pre-orient) parameter space.
                double u0Auth = ComputeProfileSeamU0(
                    pi, options, closed, polys[pi], polyNormals[pi], seamHint,
                    prevPoly, prevNorms, prevClosed, prevSeamU0, systems[pi], prevSystem);

                // Match legacy order: roll closed profiles first, then orient CCW (may reverse).
                bool rollClosed = closed &&
                    (seamHint.HasPoint || options.AlignmentMode != LoftAlignmentMode.AsAuthored);
                if (rollClosed)
                {
                    // OriginFoot / discrete: roll by vertex index directly (matches legacy GetOriginFootRollIndex).
                    int rollBy = options.AlignmentMode == LoftAlignmentMode.OriginFootRoll && !seamHint.HasPoint
                        ? GetOriginFootRollIndex(polys[pi])
                        : NearestVertexRollIndexForSeamU(polys[pi], polyNormals[pi], closed, u0Auth);
                    crease = RemapCreaseForRoll(crease, polys[pi].Count, rollBy, closed);
                    RollPolylineInPlace(polys[pi], polyNormals[pi], closed, rollBy);
                }

                bool reversed = options.FirstCurves == null && OrientClosedCounterClockwise(polys[pi], polyNormals[pi], closed);
                if (reversed)
                {
                    crease = RemapCreaseReverse(crease, polys[pi].Count);
                    // Reversing maps authored u ? (1-u); keep analytic seam consistent when used.
                    if (Math.Abs(u0Auth) > 1e-15)
                        u0Auth = Frac01(1.0 - u0Auth);
                }

                if (options.FirstCurves != null)
                {
                    prepared[pi].FirstCurveName = options.FirstCurves[pi];
                    if (prepared[pi].SourceCurves?.Any(curve => curve is not Line2D) == true)
                    {
                        // Curved correspondence is established on the authored
                        // curves, independent of tessellated sample locations.
                        if (!closed && prepared[pi].SourceCurves[0].Name != options.FirstCurves[pi])
                            throw new ArgumentException($"Section {pi + 1}: an open profile must start at its first boundary curve.");
                    }
                    else
                    {
                        int first = FindFirstCurveEdge(prepared[pi], options.FirstCurves[pi], pi, closed);
                        if (!closed && first != 0)
                            throw new ArgumentException($"Section {pi + 1}: an open profile must start at its first boundary edge.");
                        crease = RemapCreaseForRoll(crease, polys[pi].Count, first, closed);
                        RollPolylineInPlace(polys[pi], polyNormals[pi], closed, first);
                    }
                }

                double seamU0Eval = (analyticStrips[pi] != null && Math.Abs(u0Auth) > 1e-15) ? u0Auth : 0;

                seamU0[pi] = seamU0Eval;
                prepared[pi].SeamU0 = seamU0Eval;
                prepared[pi].Closed = closed;

                creaseIdxPerProfile.Add(crease);

                prevPoly = polys[pi];
                prevNorms = polyNormals[pi];
                prevClosed = closed;
                prevSystem = systems[pi];
                prevSeamU0 = 0;
            }

            for (int pi = 1; pi < pCount; pi++)
            {
                if (profileClosed[pi] != profileClosed[0])
                    throw new InvalidOperationException("All loft profiles must be either closed or open.");
            }

            bool closedForCaps = profileClosed[0];
            bool matchingVertices = options.CorrespondenceMode == LoftCorrespondenceMode.MatchingVertices;
            output.LoftSideSupportFactory = BuildExactPolygonLoftSupport(prepared, options.Style, out var polygonBreaks, matchingVertices);

            bool matchedCurves = prepared[0].MatchedCurve != null;
            int matchedCount = matchedCurves ? prepared[0].MatchedCurveNames.Count :
                closedForCaps ? polys[0].Count : polys[0].Count - 1;
            List<double> uColumns;
            List<UColumnKind> colKinds;
            int m;
            if (matchedCurves)
            {
                uColumns = polygonBreaks.Where(u => !closedForCaps || u < 1).ToList();
                colKinds = Enumerable.Repeat(UColumnKind.Uniform, uColumns.Count).ToList();
                m = uColumns.Count;
            }
            else
                BuildUColumns(options, polys, profileClosed, analyticStrips, seamU0, tolU, closedForCaps, loftTessellationTolerance,
                    out uColumns, out colKinds, out m);

            double[] surfaceRows = null;
            if (polygonBreaks != null && loftTessellationTolerance > 1e-30)
            {
                var samples = SurfaceLoftParameters(output.LoftSideSupportFactory, loftTessellationTolerance);
                polygonBreaks = polygonBreaks.Concat(samples.U).Distinct().OrderBy(u => u).ToArray();
                surfaceRows = samples.V;
            }

            // Every polygon corner is a construction knot, independent of the
            // optional sample-count/crease display policy. Do not bridge one
            // with a triangle intended to approximate a smooth U span.
            if (polygonBreaks != null)
            {
                uColumns = MergeSortedUniqueNormalizedU(uColumns,
                    polygonBreaks.Where(u => !closedForCaps || u < 1).ToArray(), 0);
                colKinds = Enumerable.Repeat(UColumnKind.Uniform, uColumns.Count).ToList();
            }

            List<double> unionCreaseU = UnionCreaseNormalizedU(polys, profileClosed, creaseIdxPerProfile, seamU0, options.CreasePolicy, tolU);
            if (matchingVertices)
            {
                int spans = matchedCount;
                unionCreaseU = Enumerable.Range(closedForCaps ? 0 : 1, closedForCaps ? spans : spans - 1)
                    .Select(i => (double)i / spans).ToList();
            }
            InsertCreaseColumnPairs(ref uColumns, ref colKinds, unionCreaseU, tolU);
            m = uColumns.Count;

            var profileWorld = new Vec3D[pCount][];
            var profileSketch2D = new Vec2D[pCount][];
            var profileTu2D = new Vec2D[pCount][];
            for (int pi = 0; pi < pCount; pi++)
            {
                profileWorld[pi] = new Vec3D[m];
                profileSketch2D[pi] = new Vec2D[m];
                profileTu2D[pi] = new Vec2D[m];
                ResampleProfileColumns(
                    polys[pi], polyNormals[pi], profileClosed[pi], systems[pi],
                    uColumns, colKinds, creaseIdxPerProfile[pi],
                    tolU,
                    analyticStrips[pi],
                    seamU0[pi],
                    profileWorld[pi], profileSketch2D[pi], profileTu2D[pi], matchingVertices, prepared[pi].MatchedCurve);
            }

            ConsolidateCoincidentUniformUColumnsInWorldSpace(
                ref uColumns, ref colKinds, profileWorld, profileSketch2D, profileTu2D, pCount, loftTessellationTolerance, ref m, polygonBreaks);

            double[] rowVUniform = null;
            Vec3D[][] grid;
            int vRows;
            bool hermiteAdaptiveV = (polygonBreaks != null || options.Style == LoftStyle.Hermite || options.Style == LoftStyle.SmoothCatmullRom) && loftTessellationTolerance > 1e-30;
            if (hermiteAdaptiveV)
            {
                BuildHermiteLoftGridAdaptive(profileWorld, loftTessellationTolerance, s, options.Style,
                    surfaceRows, out grid, out rowVUniform);
                vRows = grid.Length;
            }
            else
            {
                vRows = pCount + (pCount - 1) * s;
                grid = new Vec3D[vRows][];
                for (int i = 0; i < vRows; i++)
                    grid[i] = new Vec3D[m];
                BuildLoftGrid(profileWorld, options.Style, s, grid);
            }

            SnapCreaseDuplicateColumnsInGrid(grid, vRows, uColumns, colKinds, tolU);

            ComputeLoftMeshTolerancesFromGrid(grid, vRows, m, out double minSqCross, out double minSqEdge);

            int sideGroup = baseGroupIndex;
            int sideCount = matchingVertices ? matchedCount : 1;
            int startCapGroup = baseGroupIndex + sideCount;
            int endCapGroup = startCapGroup + 1;
            int[] sideColumns = null;
            if (matchingVertices)
            {
                output.LoftSideDomains = new Dictionary<string, ParametricRange>();
                var names = MatchedSideNames(prepared[0], loftName);
                for (int face = 0; face < sideCount; face++)
                {
                    triangleGroupToName[baseGroupIndex + face] = names[face];
                    output.LoftSideDomains.Add(names[face],
                        new ParametricRange((double)face / sideCount, (double)(face + 1) / sideCount, 0, 1));
                }
                sideColumns = Enumerable.Range(0, closedForCaps ? m : m - 1).Select(q =>
                {
                    double nextU = q + 1 == m ? 1 : uColumns[q + 1];
                    int face = Math.Min(sideCount - 1, (int)Math.Floor((uColumns[q] + nextU) * .5 * sideCount));
                    return baseGroupIndex + face;
                }).ToArray();
            }
            else
                triangleGroupToName[sideGroup] = EntityNaming.LoftSide(loftName);

            // End sections are construction planes. Quantize in their local
            // frames, then reuse the exact rim on both side and cap meshes.
            Rat3Hybrid[] PreciseRim(int profile)
            {
                var transform = new PreciseFrameTransform(converter, systems[profile]);
                return profileSketch2D[profile].Select(point =>
                    transform.Transform(new Vec3D(point.X, point.Y, 0))).ToArray();
            }
            var startRim = PreciseRim(0);
            var endRim = PreciseRim(pCount - 1);
            var planarGrid = matchingVertices && !matchedCurves
                ? PreservePlanarMatchedSides(prepared, uColumns, rowVUniform, vRows, options.Style,
                    converter, startRim, endRim) : null;
            int baseVert = vertices.Count;
            EmitGridVertices(
                grid, m, vRows, closedForCaps, colKinds.ToArray(),
                output.LoftSideSupportFactory != null ? uColumns : null, profileTu2D, systems, pCount, s, rowVUniform, startRim, endRim, planarGrid,
                vertices, normals, uv, precisePositions, converter);

            ApplyAnalyticLoftNormals(profileWorld, analyticStrips, systems, uColumns, colKinds,
                profileTu2D, seamU0, closedForCaps, options.Style, rowVUniform, vRows, baseVert, normals,
                matchingVertices || matchedCurves ? output.LoftSideSupportFactory() : null);

            EmitSideQuads(grid, m, vRows, baseVert, closedForCaps, vertices, minSqCross, minSqEdge, triangles, triangleGroups, sideGroup, sideColumns);

            if (options.CapEnds && closedForCaps)
            {
                if (m < 3)
                    throw new InvalidOperationException("Loft with caps requires at least three samples along the profile (ProfileSamplesU >= 3).");

                Vec3D nStart = -systems[0].Z.Normalized() * (Polygon.IsPolygonCCW(polys[0]) ? 1 : -1);
                Vec3D nEnd = systems[pCount - 1].Z.Normalized() * (Polygon.IsPolygonCCW(polys[^1]) ? 1 : -1);
                EmitPlanarCapFromSketch2D(
                    profileSketch2D[0], grid[0], startRim, nStart, flipWinding: false, startCapGroup,
                    options.CapTriangulation, minSqCross,
                    converter, vertices, normals, uv, precisePositions, triangles, triangleGroups);
                EmitPlanarCapFromSketch2D(
                    profileSketch2D[pCount - 1], grid[vRows - 1], endRim, nEnd, flipWinding: true, endCapGroup,
                    options.CapTriangulation, minSqCross,
                    converter, vertices, normals, uv, precisePositions, triangles, triangleGroups);

                triangleGroupToName[startCapGroup] = EntityNaming.LoftStartCap(loftName);
                triangleGroupToName[endCapGroup] = EntityNaming.LoftEndCap(loftName);
                EnsurePositiveVolume(vertices, triangles, normals);
                return sideCount + 2;
            }

            return sideCount;
        }
    }
}
