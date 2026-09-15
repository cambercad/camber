using System.Collections.Generic;

namespace NURBS
{
    public class CurveStrip
    {
        private List<BSplineCurve> _curves;

        public CurveStrip()
        {
            _curves = new List<BSplineCurve>();
        }

        public void Add(BSplineCurve curve) { _curves.Add(curve); }
        public int Count { get { return _curves.Count; } }
        public BSplineCurve this[int index] { get { return _curves[index]; } }
    }
}
