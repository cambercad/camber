using System.Collections.Generic;
using GeoCore;

namespace NURBS
{
    //public static class NurbsVisualizer
    //{
    //    public static TriangulatedGeometry Triangulate(this BSplineSurface s, int numPointsU, int numPointsV)
    //    {
    //        List<Vec3D> points = new List<Vec3D>(numPointsU * numPointsV);
    //        List<Vec3D> normals = new List<Vec3D>(numPointsU * numPointsV);
    //        List<Vec2D> uv = new List<Vec2D>(numPointsU * numPointsV);

    //        double scalingU = 1.0 / (numPointsU - 1);
    //        double scalingV = 1.0 / (numPointsV - 1);

    //        for (int i = 0; i < numPointsU; ++i)
    //        {
    //            double u = i * scalingU;
    //            for (int j = 0; j < numPointsV; ++j)
    //            {
    //                double v = j * scalingV;

    //                points.Add(s.Evaluate(u, v));
    //                Vec3D n = s.EvaluateNormal(u, v);
    //                if (n.LengthSquared() > 1e-16)
    //                    n.Normalize();
    //                normals.Add(n);
    //                uv.Add(new Vec2D(u, v));
    //            }
    //        }

    //        //for(int i=0;i<numPointsU-1;++i)
    //        //    normals[i] = new Vector3d(

    //        //Vec3D a = s.Evaluate(0, 0);
    //        //Vec3D b = s.Evaluate(1, 1);

    //        List<Tri> triangles = Tri.RectangularBuffer(numPointsV, numPointsU);
    //        return new TriangulatedGeometry(triangles, points, normals, uv);
    //    }
    //}
}
