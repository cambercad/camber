namespace GeoCore
{
    public class SubsetEnumerator<T> : IEnumerable<T>
    {
        private readonly List<int> indices;
        private readonly List<T> triangles;

        public SubsetEnumerator(List<int> indices, List<T> triangles)
        {
            this.indices = indices;
            this.triangles = triangles;
        }

        public IEnumerator<T> GetEnumerator()
        {
            foreach (int index in indices)
            {
                yield return triangles[index];
            }
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}
