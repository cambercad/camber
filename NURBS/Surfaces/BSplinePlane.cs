using GeoCore;
using System.Collections.Generic;

namespace NURBS
{
    public class BSplinePlane : BSplineSurfaceBounded
    {
        protected Vec3D _normal;
        protected double _planeD;

        protected Vec3D _origin;
        protected Vec3D _tangentX;
        protected Vec3D _tangentY;

        public BSplinePlane(Vec3D origin, Vec3D normal, Vec3D tangentX, Vec3D tangentY, List<CurveStrip> borderLoops = null)
            : base(1, 1, BuildControlPoints(origin, normal, tangentX, tangentY), new double[] { 0, 0, 1, 1 }, new double[] { 0, 0, 1, 1 }, borderLoops)
        { }        

        private static Vec3D[][] BuildControlPoints(Vec3D origin, Vec3D normal, Vec3D tangentX, Vec3D tangentY)
        {
            // Outer index is u; inner index is v (see BSplineSurface control point layout).
            return new Vec3D[][] {
                new Vec3D[] { origin - tangentX - tangentY, origin - tangentX + tangentY },
                new Vec3D[] { origin + tangentX - tangentY, origin + tangentX + tangentY }
            };
        }
    }
}
