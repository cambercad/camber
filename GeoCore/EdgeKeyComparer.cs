namespace GeoCore
{
    /// <summary>
    /// Hash for keys built by <see cref="Algorithms.Key"/> (<c>min &lt;&lt; 32 | max</c>). The default long hash is
    /// <c>min ^ max</c>, which maps the edges of neighbouring vertices onto a tiny range of buckets and
    /// makes lookups degrade to long collision chains on large meshes. Enumeration order of a dictionary
    /// or set does not depend on the comparer, so swapping it never changes results.
    /// </summary>
    public sealed class EdgeKeyComparer : IEqualityComparer<long>
    {
        public static readonly EdgeKeyComparer Instance = new EdgeKeyComparer();

        private EdgeKeyComparer() { }

        public bool Equals(long x, long y) => x == y;

        public int GetHashCode(long key) => (int)(((ulong)key * 0x9E3779B97F4A7C15UL) >> 32);
    }
}
