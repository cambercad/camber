using System.Collections.Generic;
using GeoCore;


namespace Curves
{
    public class LineStripCurve2D : IEnumerable<Curve2D>
    {
        private List<Vec2D> _points = new List<Vec2D>();

        protected LineStripCurve2D() { }
        public LineStripCurve2D(params Vec2D[] points)
        {
            _points.AddRange(points);
        }

        public void Add(params Vec2D[] points)
        {
            _points.AddRange(points);
        }

        public IEnumerator<Curve2D> GetEnumerator()
        {
            for (int i = 1; i < _points.Count; ++i)
                yield return new Line2D(_points[i - 1], _points[i]);
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}