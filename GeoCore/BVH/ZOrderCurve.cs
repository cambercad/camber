namespace GeoCore
{
    public class ZOrderCurve
    {
        //https://stackoverflow.com/questions/1024754/how-to-compute-a-3d-morton-number-interleave-the-bits-of-3-ints/18528775#18528775
        private static ulong ExpandBits(ulong x)
        {
            //x &= 0x1fffff;
            x = (x | x << 32) & 0x1f00000000ffff;
            x = (x | x << 16) & 0x1f0000ff0000ff;
            x = (x | x << 8) & 0x100f00f00f00f00f;
            x = (x | x << 4) & 0x10c30c30c30c30c3;
            x = (x | x << 2) & 0x1249249249249249;
            return x;
        }
        private static uint ExpandBitsDebug(uint x)
        {
            //x &= 0x3ff;
            x = (x | x << 16) & 0x30000ff;  // THIS IS THE MASK for shifting 16 (for bit 8 and 9)
            x = (x | x << 8) & 0x300f00f;
            x = (x | x << 4) & 0x30c30c3;
            x = (x | x << 2) & 0x9249249;
            return x;
        }

        //http://stereopsis.com/radix.html
        static uint FloatFlip(uint f)
        {
            uint mask = (uint)(-(int)(f >> 31) | 0x80000000);
            return f ^ mask;
        }

        static uint IFloatFlip(uint f)
        {
            uint mask = ((f >> 31) - 1) | 0x80000000;
            return f ^ mask;
        }
        
        //Squeze 3 float (not normalized) into a 3d morton code
        public unsafe static ulong Morton3DNoBounds(float x, float y, float z)
        {
            const int shift = (32 - 21); //We need to get rid of the least significant bits because ExpandBits(ulong) can only consum 21bits per value
            uint ix = (*((uint*)&x)) >> shift;
            uint iy = (*((uint*)&y)) >> shift;
            uint iz = (*((uint*)&z)) >> shift;
            var xx = ExpandBits((ulong)FloatFlip(ix));
            var yy = ExpandBits((ulong)FloatFlip(iy));
            var zz = ExpandBits((ulong)FloatFlip(iz));
            return xx * 4 + yy * 2 + zz;
        }


        //http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-iii-tree-construction-gpu/#more-635
        // Expands a 10-bit integer into 30 bits
        // by inserting 2 zeros after each bit.
        private static uint ExpandBits(uint v)
        {
            v = (v * 0x00010001u) & 0xFF0000FFu;
            v = (v * 0x00000101u) & 0x0F00F00Fu;
            v = (v * 0x00000011u) & 0xC30C30C3u;
            v = (v * 0x00000005u) & 0x49249249u;
            return v;
        }

        // Calculates a 30-bit Morton code for the
        // given 3D point located within the unit cube [0,1].
        public static int Morton3D(float x, float y, float z)
        {
            x = Math.Min(Math.Max(x * 1024.0f, 0.0f), 1023.0f);
            y = Math.Min(Math.Max(y * 1024.0f, 0.0f), 1023.0f);
            z = Math.Min(Math.Max(z * 1024.0f, 0.0f), 1023.0f);
            uint xx = ExpandBits((uint)x);
            uint yy = ExpandBits((uint)y);
            uint zz = ExpandBits((uint)z);
            return (int)(xx * 4 + yy * 2 + zz);
        }

        public static int Morton3D(in Vec3F f)
        {
            return Morton3D(f.X, f.Y, f.Z);
        }

        //http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
        // method to seperate bits from a given integer 3 positions apart
        public static ulong splitBy3(uint a)
        {
            ulong x = (ulong)a & 0x00000000001fffff; // we only look at the first 21 bits
            x = (x | x << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
            x = (x | x << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
            x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
            x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
            x = (x | x << 2) & 0x1249249249249249;
            return x;
        }

        //http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
        public static ulong Morton3D(int x, int y, int z)
        {
            ulong answer = 0;
            //answer |= splitBy3(x) | splitBy3(y) << 1 | splitBy3(z) << 2;

            ulong v = (ulong)x & 0x00000000001fffff; // we only look at the first 21 bits
            v = (v | v << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
            v = (v | v << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
            v = (v | v << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
            v = (v | v << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
            v = (v | v << 2) & 0x1249249249249249;
            answer |= v;

            v = (ulong)y & 0x00000000001fffff; // we only look at the first 21 bits
            v = (v | v << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
            v = (v | v << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
            v = (v | v << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
            v = (v | v << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
            v = (v | v << 2) & 0x1249249249249249;
            answer |= v << 1;

            v = (ulong)z & 0x00000000001fffff; // we only look at the first 21 bits
            v = (v | v << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
            v = (v | v << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
            v = (v | v << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and 0001000000001111000000001111000000001111000000001111000000000000
            v = (v | v << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and 0001000011000011000011000011000011000011000011000011000100000000
            v = (v | v << 2) & 0x1249249249249249;
            answer |= v << 2;

            return answer;
        }

        //http://stackoverflow.com/questions/12157685/z-order-curve-coordinates
        public static int Morton2D(int x, int y)
        {
            x = (x | (x << 8)) & 0x00FF00FF;
            x = (x | (x << 4)) & 0x0F0F0F0F;
            x = (x | (x << 2)) & 0x33333333;
            x = (x | (x << 1)) & 0x55555555;

            y = (y | (y << 8)) & 0x00FF00FF;
            y = (y | (y << 4)) & 0x0F0F0F0F;
            y = (y | (y << 2)) & 0x33333333;
            y = (y | (y << 1)) & 0x55555555;

            return x | (y << 1);
        }

        //http://stackoverflow.com/questions/4909263/how-to-efficiently-de-interleave-bits-inverse-morton
        public static void InverseMorton2D(int morton, out int x, out int y)
        {
            x = morton & 0x55555555;
            x = (x | (x >> 1)) & 0x33333333;
            x = (x | (x >> 2)) & 0x0F0F0F0F;
            x = (x | (x >> 4)) & 0x00FF00FF;
            x = (x | (x >> 8)) & 0x0000FFFF;

            y = (morton >> 1) & 0x55555555;
            y = (y | (y >> 1)) & 0x33333333;
            y = (y | (y >> 2)) & 0x0F0F0F0F;
            y = (y | (y >> 4)) & 0x00FF00FF;
            y = (y | (y >> 8)) & 0x0000FFFF;
            //*x = morton1(z);
            //*y = morton1(z >> 1);
        }
    }
}
