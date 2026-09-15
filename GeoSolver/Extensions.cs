namespace GeoSolver
{
    public static class Extensions
    {
        public static T Pop<T>(this List<T> list)
        {
            var t = list[list.Count - 1];
            list.RemoveAt(list.Count - 1);
            return t;
        }
    }
}
