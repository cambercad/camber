using GeoCore;
using System.Collections.Generic;

namespace NURBS
{
    public class TriangulatedGeometry
    {
        protected List<Tri> _triangles; public List<Tri> Triangles { get { return _triangles; } }

        protected List<Vec3D> _points; public List<Vec3D> Points { get { return _points; } }
        protected List<Vec3D> _normals; public List<Vec3D> Normals { get { return _normals; } }
        protected List<Vec2D> _uv; public List<Vec2D> UV { get { return _uv; } }

        public TriangulatedGeometry(List<Tri> triangles, List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv)
        {
            _triangles = triangles;
            _points = points;
            _normals = normals;
            _uv = uv;
        }
    }

    public class TriangulatedGeometryWithBorder : TriangulatedGeometry
    {
        protected List<List<Vec3D>> _borderLoops;

        public TriangulatedGeometryWithBorder(List<Tri> triangles, List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv, List<List<Vec3D>> borderLoops)
            : base(triangles, points, normals, uv)
        {
            _borderLoops = borderLoops;
        }
    }
}
