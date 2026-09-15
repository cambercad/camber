using GeoCore;


namespace Curves
{
    /// <summary>
    /// Flags that modify curve behavior and rendering.
    /// </summary>
    [Flags]
    public enum CurveFlags
    {
        /// <summary>
        /// No special flags.
        /// </summary>
        None = 0,

        /// <summary>
        /// Curve is helper/construction geometry and should be rendered with dashed lines.
        /// </summary>
        HelperGeometry = 1 << 0,
    }

    public abstract class Curve2D : IName
    {
        private string _name; public string Name { get { return _name; } set { _name = value; } }
        private CurveFlags _flags = CurveFlags.None; 
        /// <summary>
        /// Flags that modify curve behavior and rendering.
        /// </summary>
        public CurveFlags Flags { get { return _flags; } set { _flags = value; } }

        protected Curve2D()
        {
            // Name is set by the parent sketch/container, not auto-generated here
            _name = null;
        }

        /// <summary>
        /// Returns true if the curve is marked as helper/construction geometry.
        /// </summary>
        public bool IsHelperGeometry { get { return (_flags & CurveFlags.HelperGeometry) != 0; } }

        public abstract CurveVertex2D EvaluateVertex(double uniform);
        public abstract List<CurveVertex2D> Tessellate(double maxDeviation/*, out List<Vector2d> normals, out List<double> uv*/);
        public abstract List<CurveVertex2D> Tessellate(int pointCount);

        // Allows all kind of transformations like mirror, translate, rotate, etc
        public virtual List<Vec2D> ToReferencePoints() { throw new NotImplementedException(); }
        public virtual void UpdateFromReferencePoints(List<Vec2D> referencePoints) { }

        public virtual Curve2D GetCopy() { throw new NotImplementedException(); }
        public virtual Curve2D Reverse() { throw new NotImplementedException(); }

        public CurveVertex2D StartVertex { get { return EvaluateVertex(0); } }
        public CurveVertex2D EndVertex { get { return EvaluateVertex(1); } }
        public CurveVertex2D CenterVertex { get { return EvaluateVertex(0.5); } }

        public Vec2D StartPosition { get { return EvaluateVertex(0).Position; } }
        public Vec2D EndPosition { get { return EvaluateVertex(1).Position; } }
        public Vec2D OnCurveCenterPosition { get { return EvaluateVertex(0.5).Position; } }

        public Vec2D StartTangent { get { return EvaluateVertex(0).Tangent; } }
        public Vec2D EndTangent { get { return EvaluateVertex(1).Tangent; } }

        public abstract double Length();
    }
}