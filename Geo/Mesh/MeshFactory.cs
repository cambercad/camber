namespace Geo
{
    public class MeshFactory
    {
        private static int uniqueTriangleIndexer = 0;

        public static int GetUniqueTriangleId()
        {
            return uniqueTriangleIndexer++;
        }
    }
}

