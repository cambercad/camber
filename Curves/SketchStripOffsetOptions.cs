using GeoCore;
using GeoMeta;

namespace Curves
{
    [APIDescription(@"SketchStripOffsetOptions: optional settings for PlotterSketcher.OffsetStrip.
  JoinType (SketchOffsetJoinType, default Round): corner join style.
  OpenMode (SketchOffsetOpenMode, default Parallel): for open strips — Parallel (one-sided) or Outline (both sides + caps). Ignored when the source is closed.
  EndCap (SketchOffsetEndCap, default Round): end caps when OpenMode is Outline. Ignored for Parallel and closed sources.
  TessellationTolerance (float, default -1): max deviation when tessellating source curves; -1 uses the parent GeoAPI maxDeviation.
  CornerArcTolerance (float, default -1): max deviation for round joins and outline round end caps; -1 uses the parent GeoAPI maxDeviation.
  ConnectionTolerance (float, default 1e-6): gap allowed between selected source curves.")]
    public readonly struct SketchStripOffsetOptions
    {
        public SketchOffsetJoinType JoinType { get; init; }
        public SketchOffsetOpenMode OpenMode { get; init; }
        public SketchOffsetEndCap EndCap { get; init; }
        public double TessellationTolerance { get; init; }
        public double CornerArcTolerance { get; init; }
        public double ConnectionTolerance { get; init; }

        public SketchStripOffsetOptions(
            SketchOffsetJoinType joinType = SketchOffsetJoinType.Round,
            SketchOffsetOpenMode openMode = SketchOffsetOpenMode.Parallel,
            SketchOffsetEndCap endCap = SketchOffsetEndCap.Round,
            double tessellationTolerance = -1,
            double cornerArcTolerance = -1,
            double connectionTolerance = 1e-6)
        {
            JoinType = joinType;
            OpenMode = openMode;
            EndCap = endCap;
            TessellationTolerance = tessellationTolerance;
            CornerArcTolerance = cornerArcTolerance;
            ConnectionTolerance = connectionTolerance;
        }

        public static SketchStripOffsetOptions Default => new SketchStripOffsetOptions();

        public static SketchStripOffsetOptions Resolve(SketchStripOffsetOptions options, double defaultMaxDeviation)
        {
            if (defaultMaxDeviation <= 0)
                defaultMaxDeviation = 0.01;

            return new SketchStripOffsetOptions
            {
                JoinType = options.JoinType,
                OpenMode = options.OpenMode,
                EndCap = options.EndCap,
                TessellationTolerance = options.TessellationTolerance > 0
                    ? options.TessellationTolerance
                    : defaultMaxDeviation,
                CornerArcTolerance = options.CornerArcTolerance > 0
                    ? options.CornerArcTolerance
                    : defaultMaxDeviation,
                ConnectionTolerance = options.ConnectionTolerance > 0
                    ? options.ConnectionTolerance
                    : 1e-6,
            };
        }
    }
}
