namespace GeoCore
{
    /// <summary>
    /// Assigns dense ids to the undirected edges of a triangle list using an open-addressing hash
    /// on flat arrays. Ids are handed out in order of first occurrence (triangle, side), which is
    /// the enumeration order of a <see cref="Dictionary{TKey,TValue}"/> filled in the same order.
    /// </summary>
    public static class EdgeIdTable
    {
        /// <summary>
        /// Entry k of <paramref name="edgeOf"/> is side (k % 3) of triangle (k / 3), where side 0 is A-B,
        /// side 1 is B-C and side 2 is C-A. Triangles with a negative first index are skipped (their entries
        /// are set to -1). Returns the number of distinct edges.
        /// </summary>
        public static int Build(ReadOnlySpan<Tri> triangles, int[] edgeOf, bool skipNegative = true)
        {
            int count = triangles.Length;
            int capacity = 16;
            while (capacity < count * 2) capacity <<= 1;
            var keys = new long[capacity];
            var ids = new int[capacity];
            Array.Fill(ids, -1);
            int mask = capacity - 1;
            int next = 0;
            for (int i = 0; i < count; i++)
            {
                var t = triangles[i];
                if (skipNegative && t.A < 0)
                {
                    edgeOf[i * 3] = edgeOf[i * 3 + 1] = edgeOf[i * 3 + 2] = -1;
                    continue;
                }
                for (int side = 0; side < 3; side++)
                {
                    int a = side == 0 ? t.A : side == 1 ? t.B : t.C;
                    int b = side == 0 ? t.B : side == 1 ? t.C : t.A;
                    if (a > b) (a, b) = (b, a);
                    long key = ((long)a << 32) | (uint)b;
                    int slot = (int)(((ulong)key * 0x9E3779B97F4A7C15UL) >> 32) & mask;
                    while (true)
                    {
                        int id = ids[slot];
                        if (id < 0)
                        {
                            keys[slot] = key;
                            ids[slot] = id = next++;
                            edgeOf[i * 3 + side] = id;
                            break;
                        }
                        if (keys[slot] == key)
                        {
                            edgeOf[i * 3 + side] = id;
                            break;
                        }
                        slot = (slot + 1) & mask;
                    }
                    if (next * 10 > capacity * 7)
                    {
                        int newCapacity = capacity << 1;
                        var newKeys = new long[newCapacity];
                        var newIds = new int[newCapacity];
                        Array.Fill(newIds, -1);
                        int newMask = newCapacity - 1;
                        for (int s = 0; s < capacity; s++)
                        {
                            if (ids[s] < 0) continue;
                            int ns = (int)(((ulong)keys[s] * 0x9E3779B97F4A7C15UL) >> 32) & newMask;
                            while (newIds[ns] >= 0) ns = (ns + 1) & newMask;
                            newKeys[ns] = keys[s];
                            newIds[ns] = ids[s];
                        }
                        keys = newKeys;
                        ids = newIds;
                        capacity = newCapacity;
                        mask = newMask;
                    }
                }
            }
            return next;
        }
    }
}
