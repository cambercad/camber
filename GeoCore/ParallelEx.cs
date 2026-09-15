using System;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;

namespace GeoCore
{
    /// <summary>
    /// Avoids Parallel.For overhead on small workloads where sequential execution is faster.
    /// </summary>
    public static class ParallelEx
    {
        public const int MinParallelWorkItems = 2048;

        /// <summary>Minimum independent group count before parallelizing per-group work (e.g. coplanar retriangulation).</summary>
        public const int MinParallelGroups = 8;

        private static readonly ParallelOptions Options = new()
        {
            MaxDegreeOfParallelism = Math.Max(1, Environment.ProcessorCount),
        };

        public static void For(int fromInclusive, int toExclusive, Action<int> body)
        {
            int count = toExclusive - fromInclusive;
            if (count <= 0)
                return;

            if (count >= MinParallelWorkItems)
                Parallel.For(fromInclusive, toExclusive, Options, body);
            else
            {
                for (int i = fromInclusive; i < toExclusive; ++i)
                    body(i);
            }
        }

        /// <summary>
        /// Parallel loop with small stolen ranges so a few expensive items cannot pin a worker
        /// (default <see cref="Parallel.For"/> chunks are large and spatially correlated on a helix).
        /// </summary>
        public static void ForLoadBalanced(int fromInclusive, int toExclusive, Action<int> body)
        {
            int count = toExclusive - fromInclusive;
            if (count <= 0)
                return;
            if (count < MinParallelGroups)
            {
                for (int i = fromInclusive; i < toExclusive; ++i)
                    body(i);
                return;
            }

            int workers = Options.MaxDegreeOfParallelism;
            int rangeSize = Math.Max(1, count / (workers * 8));
            if (rangeSize > 32)
                rangeSize = 32;

            var parts = Partitioner.Create(fromInclusive, toExclusive, rangeSize);
            Parallel.ForEach(parts, Options, range =>
            {
                for (int i = range.Item1; i < range.Item2; ++i)
                    body(i);
            });
        }

        /// <summary>Parallel loop for coarse-grained tasks (few groups, each doing substantial work).</summary>
        public static void ForGroups(int fromInclusive, int toExclusive, Action<int> body)
        {
            int count = toExclusive - fromInclusive;
            if (count <= 0)
                return;

            if (count >= MinParallelGroups)
                Parallel.For(fromInclusive, toExclusive, Options, body);
            else
            {
                for (int i = fromInclusive; i < toExclusive; ++i)
                    body(i);
            }
        }
    }
}
