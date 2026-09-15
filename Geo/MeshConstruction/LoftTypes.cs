using System;
using System.Collections.Generic;
using Curves;
using GeoCore;
namespace Geo
{
    /// <summary>How shared u-columns are built from profile geometry.</summary>
    public enum LoftCorrespondenceMode
    {
        /// <summary>Uniform u merged with normalized arc-length parameters from profile tessellation (and Hermite: analytic strip tessellation).</summary>
        MergedArcLengthAnchors,
        /// <summary>Only uniform u in [0,1]; ignores tessellation vertex anchors (preview / fast mode only).</summary>
        UniformUOnly,
        /// <summary>Uniform u merged with analytic segment joints (and optional tessellation anchors when sampling is tessellated).</summary>
        AnalyticFeatureAnchors
    }

    /// <summary>Which profiles contribute crease columns at segment joints.</summary>
    public enum LoftCreasePolicy
    {
        /// <summary>Union of crease u from every profile (recommended for multi-segment strips).</summary>
        FromAllProfiles,
        /// <summary>Crease columns from the first profile’s segment joints only.</summary>
        FromFirstProfileOnly,
        /// <summary>No crease left/right column pairs (smoother TE; preferred for capped NACA solids through MeshNormalUV).</summary>
        None
    }

    /// <summary>How closed profiles choose the zero normalized arc-length point (u=0 / seam) before lofting.</summary>
    public enum LoftAlignmentMode
    {
        /// <summary>u=0 is the closest point on the closed profile to sketch origin (0,0). Default (enum 0).</summary>
        OriginFootRoll = 0,
        /// <summary>For closed profiles after the first: continuous u0 minimizing 3D mismatch vs the previous profile; first profile uses origin-foot.</summary>
        MinimumTwist = 1,
        /// <summary>u=0 is the sketch / tessellation strip start (no automatic reparameterization).</summary>
        AsAuthored = 2
    }

    /// <summary>Planar cap meshing strategy (sketch-space loop).</summary>
    public enum LoftCapTriangulationMode
    {
        /// <summary>Strict cap polygon preflight (winding / signed area).</summary>
        EarClipping,
        /// <summary>CCW-align then only reject degenerate caps (default; same triangulation pipeline as <see cref="EarClipping"/>).</summary>
        Robust
    }

    /// <summary>Whether arc length u follows sketch tessellation or analytic curve segments.</summary>
    public enum LoftProfileSamplingSource
    {
        /// <summary>Normalized u and evaluation follow the tessellated polyline.</summary>
        TessellatedPolyline,
        /// <summary>True arc length from composite <see cref="Curve2D"/> strip; tessellation only refines display anchors when needed.</summary>
        AnalyticCurveStrip
    }

    /// <summary>Optional per-profile seam hint in sketch XY (replaces nullable <c>Vec2D?</c> entries).</summary>
    public struct LoftProfileSeamHint
    {
        private Vec2D point;
        private bool hasPoint;

        public bool HasPoint { get { return hasPoint; } }

        public Vec2D Point
        {
            get
            {
                if (!hasPoint)
                    throw new InvalidOperationException("LoftProfileSeamHint has no point.");
                return point;
            }
        }

        public static LoftProfileSeamHint FromPoint(Vec2D p)
        {
            var hint = new LoftProfileSeamHint();
            hint.point = p;
            hint.hasPoint = true;
            return hint;
        }
    }

    /// <summary>Internal prepared profile for one loft section (after flatten, orient, seam u0).</summary>
    internal sealed class LoftPreparedProfile
    {
        public List<Vec2D> Poly = null!;
        public List<Vec2D> Norms = null!;
        public bool Closed;
        public List<int> CreaseIdx = null!;
        public CoordinateSystem System;
        /// <summary>Non-null when <see cref="LoftProfileSamplingSource.AnalyticCurveStrip"/>; evaluation uses true arc length.</summary>
        public CurveStrip2D AnalyticStrip;
        /// <summary>
        /// Seam in authored strip parameter: evaluate shared column u at authored <c>Frac(u + SeamU0)</c> (closed).
        /// Zero means as-authored / open profiles.
        /// </summary>
        public double SeamU0;
    }
}
