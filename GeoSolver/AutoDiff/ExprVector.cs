using System.Collections.Generic;

namespace GeoSolver
{
    public sealed class ExprVector : IEnumerableParams//, IStorable
    {
        private Expr _x; public Expr x { get { return _x; } set { _x = value; } }       
        private Expr _y; public Expr y { get { return _y; } set { _y = value; } }        
        private Expr _z; public Expr z { get { return _z; } set { _z = value; } }


        public ExprVector(Expr x, Expr y, Expr z)
        {
            _x = x; _y = y; _z = z;
        }

        public static ExprVector operator +(ExprVector l, ExprVector r)
        {
            return new ExprVector(l._x + r._x, l._y + r._y, l._z + r._z);
        }

        public static ExprVector operator -(ExprVector l, ExprVector r)
        {
            return new ExprVector(l._x - r._x, l._y - r._y, l._z - r._z);
        }

        public static Expr Dot(ExprVector l, ExprVector r)
        {
            return l._x * r._x + l._y * r._y + l._z * r._z;
        }

        public static ExprVector Cross(ExprVector l, ExprVector r)
        {
            return new ExprVector(l._y * r._z - l._z * r._y, l._z * r._x - l._x * r._z, l._x * r._y - l._y * r._x);
        }

        public Expr Magnitude()
        {
            return Expr.Sqrt(Expr.Pow2(_x) + Expr.Pow2(_y) + Expr.Pow2(_z));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in _x) yield return p;
            foreach (Param p in _y) yield return p;
            foreach (Param p in _z) yield return p;
        }
        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
}