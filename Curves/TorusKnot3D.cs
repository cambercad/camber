using GeoCore;

namespace Curves
{
    public class TorusKnot3D : Curve3D
    {
        //https://de.wikipedia.org/wiki/Torusknoten
        private double _p;
        private double _q;
        private double _radius;
        private const double TWO_PI = 2.0 * Math.PI;


        protected TorusKnot3D(string name = null) : base(name) { }
        public TorusKnot3D(double p, double q, double radius, string name = null) : base(name)
        {
            _p = p;
            _q = q;
            _radius = radius;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            double t = uniform * TWO_PI;
            double x = _radius * ((2 + Math.Cos(_p * t)) * Math.Cos(_q * t));
            double y = _radius * ((2 + Math.Cos(_p * t)) * Math.Sin(_q * t));
            double z = _radius * Math.Sin(_p * t);

            double dx = _radius * (-Math.Sin(_p * t) * _p * Math.Cos(_q * t) + (2 + Math.Cos(_p * t)) * -Math.Sin(_q * t) * _q);
            double dy = _radius * (-Math.Sin(_p * t) * _p * Math.Sin(_q * t) + (2 + Math.Cos(_p * t)) * Math.Cos(_q * t) * _q);
            double dz = _radius * Math.Cos(_p * t) * _p;

            double ddx = _radius * (-Math.Cos(_p * t) * _p * _p * Math.Cos(_q * t) - Math.Sin(_p * t) * _p * Math.Sin(_q * t) * _q
                + (Math.Sin(_p * t) * _p) * -Math.Sin(_q * t) * _q + (2 + Math.Cos(_p * t)) * -Math.Cos(_q * t) * _q * _q);
            double ddy = _radius * (-Math.Cos(_p * t) * _p * _p * Math.Sin(_q * t) - Math.Sin(_p * t) * _p * Math.Cos(_q * t) * _q
                + (-Math.Sin(_p * t) * _p) * Math.Cos(_q * t) * _q + (2 + Math.Cos(_p * t)) * -Math.Sin(_q * t) * _q * _q);
            double ddz = _radius * -Math.Sin(_p * t) * _p * _p;

            Vec3D p = new Vec3D(x, y, z);
            Vec3D tangent = new Vec3D(dx, dy, dz); tangent.Normalize();
            Vec3D n = new Vec3D(ddx, ddy, ddz); n.Normalize();

            //Matrix4d rot = Matrix4d.RotationAxis(tangent, 7*t);
            //n = rot.TransformDirection(n);

            return new CurveVertex3D(p, tangent, n, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double maxDeviation)
        {
            //TODO: This is a simplified version
            int num = 1000;
            List<CurveVertex3D> pts = new List<CurveVertex3D>(num);
            double scaling = 1.0 / (num - 1);
            for (int i = 0; i < num; ++i)
            {
                pts.Add(Evaluate(i * scaling));
            }
            return pts;
        }
    }
}