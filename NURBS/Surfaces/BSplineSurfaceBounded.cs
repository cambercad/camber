using System.Collections.Generic;
using GeoCore;

namespace NURBS
{
    public class BSplineSurfaceBounded : BSplineSurface
    {
        protected List<CurveStrip> _borderLoops/* = new List<CurveStrip>()*/; public List<CurveStrip> BorderLoops { get { return _borderLoops; } }


        public BSplineSurfaceBounded(int degreeU, int degreeV, Vec3D[][] controlPoints, double[] knotsU, double[] knotsV, List<CurveStrip> borderLoops = null)
            : base(degreeU, degreeV, controlPoints, knotsU, knotsV)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, int degreeV, Vec3D[][] controlPoints, double[][] weights, double[] knotsU, double[] knotsV, List<CurveStrip> borderLoops = null)
            : base(degreeU, degreeV, controlPoints, weights, knotsU, knotsV)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, int degreeV, Vec4D[][] controlPoints, double[] knotsU, double[] knotsV, List<CurveStrip> borderLoops = null)
            : base(degreeU, degreeV, controlPoints, knotsU, knotsV)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, IList<BSplineCurve> vCurves, double[] knotsU, List<CurveStrip> borderLoops = null)
            : base(degreeU, vCurves, knotsU)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, IList<BSplineCurve> vCurves, double[] weights, double[] knotsU, List<CurveStrip> borderLoops = null)
            : base(degreeU, vCurves, weights, knotsU)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, int degreeV, List<List<Vec3D>> controlPoints, List<double> knotsU, List<double> knotsV, List<CurveStrip> borderLoops = null)
            : base(degreeU, degreeV, controlPoints, knotsU, knotsV)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        public BSplineSurfaceBounded(int degreeU, int degreeV, List<List<Vec3D>> controlPoints, List<List<double>> weights, List<double> knotsU, List<double> knotsV, List<CurveStrip> borderLoops = null)
            : base(degreeU, degreeV, controlPoints, weights, knotsU, knotsV)
        {
            if (borderLoops == null)
                _borderLoops = BuildDefaultBoundary();
            else
                _borderLoops = borderLoops;
        }

        private static List<CurveStrip> BuildDefaultBoundary()
        {
            CurveStrip cs = new CurveStrip();
            cs.Add(new BSplineLine(new Vec3D(0, 0, 0), new Vec3D(1, 0, 0)));
            cs.Add(new BSplineLine(new Vec3D(1, 0, 0), new Vec3D(1, 1, 0)));
            cs.Add(new BSplineLine(new Vec3D(1, 1, 0), new Vec3D(0, 1, 0)));
            cs.Add(new BSplineLine(new Vec3D(0, 1, 0), new Vec3D(0, 0, 0)));
            return new List<CurveStrip>() { cs };
        }

        public TriangulatedGeometry Triangulate(double maxDeviation = 0.1, double minimalBoundaryPointDistance = 1e-2, double tol = 1e-8, double eps = 1e-12)
        {
            /*if (_borderLoops.Count == 0)
            {
                CurveStrip cs = new CurveStrip();
                cs.Add(new BSplineLine(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0)));
                cs.Add(new BSplineLine(new Vector3d(1, 0, 0), new Vector3d(1, 1, 0)));
                cs.Add(new BSplineLine(new Vector3d(1, 1, 0), new Vector3d(0, 1, 0)));
                cs.Add(new BSplineLine(new Vector3d(0, 1, 0), new Vector3d(0, 0, 0)));
                _borderLoops.Add(cs);
            }*/

            //TODO: Extract boundary as uv-polyline
            return AdaptiveSurfaceSplitter.TriangulateAdaptive(this, null, maxDeviation, minimalBoundaryPointDistance, tol, eps); // NurbsTessellator.Triangulate(this, maxDeviation, minimalBoundaryPointDistance, tol, eps);
        }
    }
}
