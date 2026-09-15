using System.Runtime.CompilerServices;
using GeoCore;

namespace Curves
{
    /// <summary>
    /// Groups a flat curve list into connected strips at consume time (tessellate / extrude / export).
    /// Construction geometry is connected separately so helper lines that touch the profile do not weld into it.
    /// Offset-network loops stay separate even when they share T-junction vertices.
    /// </summary>
    public static class SketchContourBuilder
    {
        public const double DefaultTolerance = 1e-8;

        public static List<List<Curve2D>> Connect(
            IList<Curve2D> curves,
            double tolerance = DefaultTolerance,
            bool splitHelpers = true)
        {
            var result = new List<List<Curve2D>>();
            if (curves == null || curves.Count == 0)
                return result;

            if (!splitHelpers)
            {
                result.AddRange(ConnectPartitioned(curves, tolerance, reverseCopies: true));
                return result;
            }

            var profile = new List<Curve2D>();
            var helpers = new List<Curve2D>();
            for (int i = 0; i < curves.Count; i++)
            {
                Curve2D curve = curves[i];
                if (curve == null)
                    continue;
                if (curve.IsHelperGeometry)
                    helpers.Add(curve);
                else
                    profile.Add(curve);
            }

            result.AddRange(ConnectPartitioned(profile, tolerance, reverseCopies: true));
            result.AddRange(ConnectPartitioned(helpers, tolerance, reverseCopies: true));
            return result;
        }

        /// <summary>
        /// Same grouping as <see cref="Connect"/> but every curve is the original instance (no Reverse copies).
        /// Use for Repeat / Offset source selection so dependents keep tracking the live sketch curves.
        /// </summary>
        public static List<List<Curve2D>> ConnectOriginals(
            IList<Curve2D> curves,
            double tolerance = DefaultTolerance)
        {
            if (curves == null || curves.Count == 0)
                return new List<List<Curve2D>>();
            return ConnectPartitioned(curves, tolerance, reverseCopies: false);
        }

        static List<List<Curve2D>> ConnectPartitioned(IList<Curve2D> curves, double tolerance, bool reverseCopies)
        {
            var result = new List<List<Curve2D>>();
            List<Curve2D> ungrouped;
            List<List<Curve2D>> offsetLoops;
            PartitionByDeclaredLoop(curves, out ungrouped, out offsetLoops);

            if (ungrouped.Count > 0)
                result.AddRange(ConnectPool(ungrouped, tolerance, reverseCopies));

            // Assembler order is the loop walk. Do not re-connect: shared T-junction
            // vertices would weld rooms, and reversing sampled pieces can invert a loop.
            for (int i = 0; i < offsetLoops.Count; i++)
            {
                if (offsetLoops[i].Count > 0)
                    result.Add(offsetLoops[i]);
            }

            return result;
        }

        /// <summary>
        /// Offset fragments already know which closed loop they belong to. Keep those
        /// loops apart so shared T-junction vertices do not weld rooms into one contour.
        /// </summary>
        static void PartitionByDeclaredLoop(
            IList<Curve2D> curves,
            out List<Curve2D> ungrouped,
            out List<List<Curve2D>> offsetLoops)
        {
            ungrouped = new List<Curve2D>();
            offsetLoops = new List<List<Curve2D>>();
            var indexOfKey = new Dictionary<LoopKey, int>();

            for (int i = 0; i < curves.Count; i++)
            {
                Curve2D curve = curves[i];
                if (curve == null)
                    continue;

                OffsetSampledCurve2D sampled = curve as OffsetSampledCurve2D;
                if (sampled == null)
                {
                    ungrouped.Add(curve);
                    continue;
                }

                var key = new LoopKey(sampled.Owner, sampled.LoopIndex);
                int idx;
                if (!indexOfKey.TryGetValue(key, out idx))
                {
                    idx = offsetLoops.Count;
                    indexOfKey[key] = idx;
                    offsetLoops.Add(new List<Curve2D>());
                }
                offsetLoops[idx].Add(curve);
            }
        }

        static List<List<Curve2D>> ConnectPool(IList<Curve2D> curves, double tolerance, bool reverseCopies)
        {
            var result = new List<List<Curve2D>>();
            if (curves == null || curves.Count == 0)
                return result;

            double tolSq = tolerance * tolerance;
            List<bool> closed;
            List<List<int>> indices = SegmentConnector.Connect(
                curves,
                c => c.StartPosition,
                c => c.EndPosition,
                (a, b) => Vec2DOps.DistanceSquared(a, b) <= tolSq,
                out closed);

            for (int s = 0; s < indices.Count; s++)
            {
                List<int> strip = indices[s];
                if (strip == null || strip.Count == 0)
                    continue;
                if (s < closed.Count && closed[s])
                    RotateClosedStripToEarliestOriginal(strip);
                var ordered = new List<Curve2D>(strip.Count);
                for (int i = 0; i < strip.Count; i++)
                {
                    int signedIndex = strip[i];
                    int index = signedIndex < 0 ? -signedIndex : signedIndex;
                    Curve2D curve = curves[index];
                    if (reverseCopies && signedIndex < 0)
                        curve = ReversedPreservingIdentity(curve);
                    ordered.Add(curve);
                }
                result.Add(ordered);
            }

            return result;
        }

        /// <summary>
        /// Closed loops often get the last edge prepended onto the first. Rotate so the
        /// earliest-added curve stays the strip start (loft AsAuthored seam, turtle origin).
        /// If that curve was reversed, reverse the whole loop so it is used unreversed.
        /// </summary>
        static void RotateClosedStripToEarliestOriginal(List<int> signedIndices)
        {
            if (signedIndices == null || signedIndices.Count < 2)
                return;

            int bestPos = 0;
            int bestAbs = int.MaxValue;
            for (int i = 0; i < signedIndices.Count; i++)
            {
                int abs = signedIndices[i] < 0 ? -signedIndices[i] : signedIndices[i];
                if (abs < bestAbs)
                {
                    bestAbs = abs;
                    bestPos = i;
                }
            }

            if (bestPos > 0)
            {
                var rotated = new List<int>(signedIndices.Count);
                for (int i = 0; i < signedIndices.Count; i++)
                    rotated.Add(signedIndices[(bestPos + i) % signedIndices.Count]);
                signedIndices.Clear();
                signedIndices.AddRange(rotated);
            }

            if (signedIndices[0] < 0)
            {
                signedIndices.Reverse();
                for (int i = 0; i < signedIndices.Count; i++)
                    signedIndices[i] = -signedIndices[i];
            }
        }

        static Curve2D ReversedPreservingIdentity(Curve2D curve)
        {
            Curve2D reversed = curve.Reverse();
            if (reversed == null)
                return curve;
            if (string.IsNullOrEmpty(reversed.Name))
                reversed.Name = curve.Name;
            reversed.Flags = curve.Flags;
            return reversed;
        }

        class LoopKey : IEquatable<LoopKey>
        {
            public readonly object Owner;
            public readonly int Loop;

            public LoopKey(object owner, int loop)
            {
                Owner = owner;
                Loop = loop;
            }

            public bool Equals(LoopKey other)
            {
                return other != null && ReferenceEquals(Owner, other.Owner) && Loop == other.Loop;
            }

            public override bool Equals(object obj)
            {
                return Equals(obj as LoopKey);
            }

            public override int GetHashCode()
            {
                int ownerHash = Owner != null ? RuntimeHelpers.GetHashCode(Owner) : 0;
                return ownerHash * 397 ^ Loop;
            }
        }
    }
}
