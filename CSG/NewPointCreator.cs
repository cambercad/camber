using GeoCore;

namespace CSG
{
    public class NewPointCreator
    {
        private Dictionary<Rat3Hybrid, int> pointToIndex = new Dictionary<Rat3Hybrid, int>();
        private List<Rat3Hybrid> points = new List<Rat3Hybrid>();
        private int originalPointCount = 0;

        public NewPointCreator()
        {
        }

        public List<Rat3Hybrid> GetPoints()
        {
            return points;
        }

        public void EndInitialize()
        {
            originalPointCount = points.Count;
        }

        public Rat3Hybrid GetPoint(int index)
        {
            return points[index];
        }

        public int GetIndex(Rat3Hybrid p, out bool pointIsDuplicate)
        {
            p.Simplify();

            int id;
            if (pointToIndex.TryGetValue(p, out id))
            {
                pointIsDuplicate = true;
                return id;
            }

            pointIsDuplicate = false;
            var copy = new Rat3Hybrid(p);
            id = points.Count;
            pointToIndex.Add(copy, id);

#if DEBUG
            for (int i = 0; i < points.Count; i++)
                if (points[i].Equals(copy))
                    throw new Exception();
#endif

            points.Add(copy);
            return id;
        }

        public int GetIndex(Rat3Hybrid pt)
        {
            pt.Simplify();

            int id;
            if (pointToIndex.TryGetValue(pt, out id))
                return id;

            var copy = new Rat3Hybrid(pt);
            id = points.Count;
            pointToIndex.Add(copy, id);

#if DEBUG
            for (int i = 0; i < points.Count; i++)
                if (points[i].Equals(copy))
                    throw new Exception();
#endif

            points.Add(copy);
            return id;
        }

        public List<Rat3Hybrid> ExportPoints()
        {
            return points;
        }
    }
}
