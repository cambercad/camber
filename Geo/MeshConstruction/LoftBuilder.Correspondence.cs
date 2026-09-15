using System;
using System.Collections.Generic;
using System.Linq;
using Curves;
using Curves.Base;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        private enum UColumnKind
        {
            Uniform,
            CreaseLeft,
            CreaseRight
        }

        private static List<double> CreaseNormalizedU(List<Vec2D> poly, bool closed, List<int> creaseIdx, double seamU0)
        {
            var result = new List<double>();
            if (creaseIdx == null || creaseIdx.Count == 0 || poly.Count < 2)
                return result;
            double total = CurveStrip2D.TotalLengthOfPolyline(poly, closed);
            if (total < EpsLen)
                return result;
            double[] distBuf = LineStrip2D.BuildDistanceBuffer(poly);
            foreach (int idx in creaseIdx.Distinct())
            {
                if (idx < 0 || idx >= poly.Count)
                    continue;
                double uAuth = distBuf[idx] / total;
                // Poly is usually already seam-aligned (seamU0≈0); only shift when analytic offset is active.
                result.Add(Math.Abs(seamU0) < 1e-15 ? uAuth : ToSeamU(uAuth, seamU0, closed));
            }
            return result.OrderBy(x => x).Distinct().ToList();
        }

        private static List<double> UnionCreaseNormalizedU(
            List<List<Vec2D>> polys,
            bool[] closed,
            List<List<int>> creaseIdxPerProfile,
            double[] seamU0,
            LoftCreasePolicy policy,
            double tolU)
        {
            if (policy == LoftCreasePolicy.None)
                return new List<double>();
            if (policy == LoftCreasePolicy.FromFirstProfileOnly)
                return CreaseNormalizedU(polys[0], closed[0], creaseIdxPerProfile[0], seamU0[0]);

            var merged = new List<double>();
            for (int pi = 0; pi < polys.Count; pi++)
                merged.AddRange(CreaseNormalizedU(polys[pi], closed[pi], creaseIdxPerProfile[pi], seamU0[pi]));
            merged.Sort();
            var dedup = new List<double>();
            foreach (double u in merged)
            {
                if (dedup.Count == 0 || Math.Abs(u - dedup[^1]) > tolU)
                    dedup.Add(u);
            }
            return dedup;
        }

        private static List<double> SubsampleSortedUAnchors(IReadOnlyList<double> dedup, int cap, double tolU)
        {
            var sub = new List<double>(cap);
            for (int i = 0; i < cap; i++)
            {
                double t = cap == 1 ? 0 : (double)i / (cap - 1);
                int j = (int)Math.Round(t * (dedup.Count - 1));
                j = Math.Clamp(j, 0, dedup.Count - 1);
                double u = dedup[j];
                if (sub.Count == 0 || Math.Abs(u - sub[^1]) > tolU)
                    sub.Add(u);
            }
            return sub;
        }

        private static double NormalizedUClusterTolerance(
            List<List<Vec2D>> polys,
            bool[] closed,
            double loftTessellationTolerance,
            double tolU)
        {
            double minPerim = double.MaxValue;
            for (int pi = 0; pi < polys.Count; pi++)
            {
                double p = CurveStrip2D.TotalLengthOfPolyline(polys[pi], closed[pi]);
                if (p > 1e-15)
                    minPerim = Math.Min(minPerim, p);
            }

            if (minPerim == double.MaxValue)
                minPerim = 1.0;

            double dev = loftTessellationTolerance > 1e-30 ? loftTessellationTolerance : 1e-6;
            return Math.Max(tolU, dev / minPerim);
        }

        private static List<double> DedupSortedNormalizedU(
            List<double> sortedU,
            double clusterTol,
            double tolU,
            int maxAnchors)
        {
            var dedup = new List<double>();
            foreach (double u in sortedU)
            {
                if (dedup.Count == 0 || Math.Abs(u - dedup[^1]) > clusterTol)
                    dedup.Add(u);
            }

            if (maxAnchors > 0 && dedup.Count > maxAnchors && maxAnchors >= 2)
                dedup = SubsampleSortedUAnchors(dedup, maxAnchors, tolU);
            return dedup;
        }

        private static List<double> MergeNormalizedUFromAllProfiles(
            List<List<Vec2D>> polys,
            bool[] closed,
            double[] seamU0,
            double tolU,
            double loftTessellationTolerance,
            int maxAnchors)
        {
            var all = new List<double>();
            for (int pi = 0; pi < polys.Count; pi++)
            {
                foreach (double uAuth in CurveStrip2D.VertexNormalizedUParameters(polys[pi], closed[pi]))
                {
                    if (Math.Abs(seamU0[pi]) < 1e-15)
                        all.Add(uAuth);
                    else
                        all.Add(ToSeamU(uAuth, seamU0[pi], closed[pi]));
                }
            }

            all.Sort();
            double clusterTol = NormalizedUClusterTolerance(polys, closed, loftTessellationTolerance, tolU);
            return DedupSortedNormalizedU(all, clusterTol, tolU, maxAnchors);
        }

        /// <summary>
        /// Normalized u at tessellation vertices of an analytic strip: each segment is sampled with <see cref="Curve2D.Tessellate(double)"/>.
        /// </summary>
        private static List<double> NormalizedUFromAnalyticStripTessellation(CurveStrip2D strip, bool closed, double maxDeviation)
        {
            double dev = maxDeviation > 1e-30 ? maxDeviation : 1e-6;
            var pts = new List<Vec2D>();
            foreach (Curve2D seg in strip.Segments)
            {
                List<CurveVertex2D> tess = seg.Tessellate(dev);
                for (int k = 0; k < tess.Count; k++)
                {
                    Vec2D p = tess[k].Position;
                    if (pts.Count > 0 && Vec2DOps.DistanceSquared(pts[^1], p) <= TolSq)
                        continue;
                    pts.Add(p);
                }
            }

            if (pts.Count < 2)
                return new List<double>();
            return CurveStrip2D.VertexNormalizedUParameters(pts, closed);
        }

        /// <summary>Merges tessellation-derived u parameters from every profile (analytic strip tessellated at <paramref name="loftTessellationTolerance"/>, else prepared polyline vertices).</summary>
        private static List<double> MergeNormalizedUFromProfilesAtTessellationTolerance(
            List<List<Vec2D>> polys,
            bool[] profileClosed,
            List<CurveStrip2D> analyticStrips,
            double[] seamU0,
            double loftTessellationTolerance,
            double tolU,
            int maxMergedUAnchors)
        {
            var all = new List<double>();
            for (int pi = 0; pi < polys.Count; pi++)
            {
                List<double> usAuth;
                if (analyticStrips[pi] != null && analyticStrips[pi]!.Segments.Count > 0)
                    usAuth = NormalizedUFromAnalyticStripTessellation(analyticStrips[pi]!, profileClosed[pi], loftTessellationTolerance);
                else
                    usAuth = CurveStrip2D.VertexNormalizedUParameters(polys[pi], profileClosed[pi]);
                foreach (double uAuth in usAuth)
                    all.Add(ToSeamU(uAuth, seamU0[pi], profileClosed[pi]));
            }

            all.Sort();
            double clusterTol = NormalizedUClusterTolerance(polys, profileClosed, loftTessellationTolerance, tolU);
            return DedupSortedNormalizedU(all, clusterTol, tolU, maxMergedUAnchors);
        }

        private static List<double> MergeAnalyticFeatureAnchors(
            List<List<Vec2D>> polys,
            bool[] closed,
            List<CurveStrip2D> analyticStrips,
            double[] seamU0,
            double tolU)
        {
            var all = new List<double>();
            for (int pi = 0; pi < polys.Count; pi++)
            {
                IEnumerable<double> usAuth = analyticStrips[pi] != null
                    ? analyticStrips[pi]!.SegmentStartNormalizedUParameters()
                    : CurveStrip2D.VertexNormalizedUParameters(polys[pi], closed[pi]);
                foreach (double uAuth in usAuth)
                    all.Add(ToSeamU(uAuth, seamU0[pi], closed[pi]));
            }
            all.Sort();
            var dedup = new List<double>();
            foreach (double u in all)
            {
                if (dedup.Count == 0 || Math.Abs(u - dedup[^1]) > tolU)
                    dedup.Add(u);
            }
            return dedup;
        }

        private static List<double> BuildUniformNormalizedUSamples(int m, bool closed)
        {
            var uCols = new List<double>(m);
            for (int k = 0; k < m; k++)
            {
                double u = closed
                    ? (m == 1 ? 0 : (double)k / m)
                    : (m == 1 ? 0 : (double)k / (m - 1));
                uCols.Add(u);
            }
            return uCols;
        }

        private static List<double> MergeSortedUniqueNormalizedU(IReadOnlyList<double> a, IReadOnlyList<double> b, double tolU)
        {
            var all = new List<double>(a.Count + b.Count);
            all.AddRange(a);
            all.AddRange(b);
            all.Sort();
            var dedup = new List<double>();
            foreach (double u in all)
            {
                if (dedup.Count == 0 || Math.Abs(u - dedup[^1]) > tolU)
                    dedup.Add(u);
            }
            return dedup;
        }

        private static void InsertCreaseColumnPairs(ref List<double> uColumns, ref List<UColumnKind> colKinds, List<double> creaseU, double tolU)
        {
            if (creaseU == null || creaseU.Count == 0)
                return;
            foreach (double cu in creaseU.OrderByDescending(x => x))
            {
                int i = -1;
                for (int k = 0; k < uColumns.Count; k++)
                {
                    if (Math.Abs(uColumns[k] - cu) <= tolU)
                    {
                        i = k;
                        break;
                    }
                }
                if (i >= 0)
                {
                    if (colKinds[i] != UColumnKind.Uniform)
                        continue;
                    uColumns.Insert(i + 1, uColumns[i]);
                    colKinds.Insert(i + 1, UColumnKind.CreaseRight);
                    colKinds[i] = UColumnKind.CreaseLeft;
                }
                else
                {
                    int ins = 0;
                    while (ins < uColumns.Count && uColumns[ins] < cu)
                        ins++;
                    uColumns.Insert(ins, cu);
                    colKinds.Insert(ins, UColumnKind.CreaseLeft);
                    uColumns.Insert(ins + 1, cu);
                    colKinds.Insert(ins + 1, UColumnKind.CreaseRight);
                }
            }
        }

        private static void BuildUColumns(
            LoftOptions options,
            List<List<Vec2D>> polys,
            bool[] profileClosed,
            List<CurveStrip2D> analyticStrips,
            double[] seamU0,
            double tolU,
            bool closedForCaps,
            double loftTessellationTolerance,
            out List<double> uColumns,
            out List<UColumnKind> colKinds,
            out int m)
        {
            int mReq = Math.Max(2, options.ProfileSamplesU);
            List<double> uniformU = BuildUniformNormalizedUSamples(mReq, closedForCaps);

            if (options.Style == LoftStyle.Hermite)
            {
                if (options.CorrespondenceMode == LoftCorrespondenceMode.UniformUOnly)
                {
                    uColumns = uniformU;
                    colKinds = Enumerable.Repeat(UColumnKind.Uniform, uColumns.Count).ToList();
                    m = uColumns.Count;
                    return;
                }

                List<double> tessU = MergeNormalizedUFromProfilesAtTessellationTolerance(
                    polys, profileClosed, analyticStrips, seamU0, loftTessellationTolerance, tolU, options.MaxMergedUAnchors);

                List<double> merged = MergeSortedUniqueNormalizedU(uniformU, tessU, tolU);

                if (options.CorrespondenceMode == LoftCorrespondenceMode.AnalyticFeatureAnchors)
                {
                    var feat = MergeAnalyticFeatureAnchors(polys, profileClosed, analyticStrips, seamU0, tolU);
                    merged = MergeSortedUniqueNormalizedU(merged, feat, tolU);
                }

                uColumns = merged;
                colKinds = Enumerable.Repeat(UColumnKind.Uniform, uColumns.Count).ToList();
                m = uColumns.Count;
                return;
            }

            List<double> mergedFeature = options.CorrespondenceMode switch
            {
                LoftCorrespondenceMode.UniformUOnly => new List<double>(),
                LoftCorrespondenceMode.AnalyticFeatureAnchors =>
                    MergeAnalyticFeatureAnchors(polys, profileClosed, analyticStrips, seamU0, tolU),
                _ => MergeNormalizedUFromAllProfiles(
                    polys, profileClosed, seamU0, tolU, loftTessellationTolerance, options.MaxMergedUAnchors)
            };
            uColumns = MergeSortedUniqueNormalizedU(uniformU, mergedFeature, tolU);
            colKinds = Enumerable.Repeat(UColumnKind.Uniform, uColumns.Count).ToList();
            m = uColumns.Count;
        }

        /// <summary>
        /// Merged normalized-u anchors from different profiles can land on distinct u-values that still map to the same (or
        /// numerically coincident) 3D positions on every profile — e.g. tessellation vertices near the trailing edge at
        /// slightly different arc-length fractions. Ruled/Catmull-Rom skinning then builds quads whose u-edges collapse in
        /// world space, producing sliver or zero-area triangles. Drop redundant <see cref="UColumnKind.Uniform"/> columns
        /// when every profile agrees the column is coincident with the last kept uniform column (crease left/right pairs
        /// are never merged).
        /// </summary>
        private static void ConsolidateCoincidentUniformUColumnsInWorldSpace(
            ref List<double> uColumns,
            ref List<UColumnKind> colKinds,
            Vec3D[][] profileWorld,
            Vec2D[][] profileSketch2D,
            Vec2D[][] profileTu2D,
            int pCount,
            double loftTessellationTolerance,
            ref int m)
        {
            if (m < 2 || uColumns.Count != m || colKinds.Count != m)
                return;

            double minX = double.MaxValue, minY = minX, minZ = minX;
            double maxX = double.MinValue, maxY = maxX, maxZ = maxX;
            for (int pi = 0; pi < pCount; pi++)
            {
                for (int k = 0; k < m; k++)
                {
                    Vec3D p = profileWorld[pi][k];
                    if (p.X < minX) minX = p.X;
                    if (p.Y < minY) minY = p.Y;
                    if (p.Z < minZ) minZ = p.Z;
                    if (p.X > maxX) maxX = p.X;
                    if (p.Y > maxY) maxY = p.Y;
                    if (p.Z > maxZ) maxZ = p.Z;
                }
            }

            double dx = maxX - minX;
            double dy = maxY - minY;
            double dz = maxZ - minZ;
            double L = Math.Max(dx, Math.Max(dy, dz));
            if (L < 1e-30)
                L = 1.0;
            // Must be at least as loose as EmitSideQuads' collapsed-edge skip (L*1e-14 floor was far too
            // tight for metre-scale lofts: near-duplicate merged-u anchors near NACA TE stayed as
            // separate columns, quads were skipped, and the solid was non-watertight).
            double epsWorld = Math.Max(1e-12, Math.Max(L * 1e-9, loftTessellationTolerance));
            double epsSq = epsWorld * epsWorld;

            var pick = new List<int>(m) { 0 };
            int lastOrig = 0;
            for (int k = 1; k < m; k++)
            {
                if (colKinds[k - 1] != UColumnKind.Uniform || colKinds[k] != UColumnKind.Uniform)
                {
                    pick.Add(k);
                    lastOrig = k;
                    continue;
                }

                bool allClose = true;
                for (int pi = 0; pi < pCount; pi++)
                {
                    if (Vec3DOps.DistanceSquared(profileWorld[pi][lastOrig], profileWorld[pi][k]) > epsSq)
                    {
                        allClose = false;
                        break;
                    }
                }

                if (allClose)
                    continue;

                pick.Add(k);
                lastOrig = k;
            }

            if (pick.Count == m)
                return;

            int mNew = pick.Count;
            for (int pi = 0; pi < pCount; pi++)
            {
                var w = new Vec3D[mNew];
                var s = new Vec2D[mNew];
                var t = new Vec2D[mNew];
                for (int j = 0; j < mNew; j++)
                {
                    int src = pick[j];
                    w[j] = profileWorld[pi][src];
                    s[j] = profileSketch2D[pi][src];
                    t[j] = profileTu2D[pi][src];
                }

                profileWorld[pi] = w;
                profileSketch2D[pi] = s;
                profileTu2D[pi] = t;
            }

            var nu = new List<double>(mNew);
            var nk = new List<UColumnKind>(mNew);
            for (int j = 0; j < mNew; j++)
            {
                int src = pick[j];
                nu.Add(uColumns[src]);
                nk.Add(colKinds[src]);
            }

            uColumns = nu;
            colKinds = nk;
            m = mNew;
        }
    }
}
