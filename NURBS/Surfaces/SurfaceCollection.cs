using System.Collections.Generic;
using GeoCore;

namespace NURBS
{
    public class SurfaceCollection
    {
        private List<BSplineSurface> _surfaces;

        public SurfaceCollection()
        {
            _surfaces = new List<BSplineSurface>();
        }

        public void Add(BSplineSurface curve) { _surfaces.Add(curve); }
        public int Count { get { return _surfaces.Count; } }
        public BSplineSurface this[int index] { get { return _surfaces[index]; } }

        public static SurfaceCollection Revolve(CurveStrip curves, Vec3D pointOnAxis, Vec3D axis, Vec3D zeroAngleDirection)
        {
            SurfaceCollection s = new SurfaceCollection();
            for (int i = 0; i < curves.Count; ++i)
            {
                s.Add(new BSplineSurfaceOfRevolution(pointOnAxis, axis, curves[i], zeroAngleDirection));
            }
            return s;
        }

        public static SurfaceCollection Extrude(CurveStrip curves, Vec3D extrudeDirection, double extrudeLength)
        {
            SurfaceCollection s = new SurfaceCollection();
            for (int i = 0; i < curves.Count; ++i)
            {
                s.Add(new BSplineLinearExtrudeSurface(curves[i], extrudeDirection, extrudeLength));
            }
            //TODO: Create Sufraces for bottom and top
            return s;
        }
    }
}
