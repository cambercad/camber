using Curves.Base;
using GeoCore;

namespace Curves
{
    /// <summary>
    /// Sketch curve that tracks a parent curve through a rigid transform.
    /// Dependent geometry is rebuilt in <see cref="Update"/> and is not part of the constraint solver.
    /// </summary>
    public class TransformedSketchCurve2D : Curve2D, IDependentSketchCurve
    {
        private Curve2D _geometry;

        public Curve2D Source { get; }
        public RigidTransform2D Transform { get; }
        public Curve2D Geometry => _geometry;

        public TransformedSketchCurve2D(Curve2D source, RigidTransform2D transform, CurveFlags flags = CurveFlags.None)
        {
            Source = source ?? throw new ArgumentNullException(nameof(source));
            Transform = transform;
            Flags = flags;
            Update();
        }

        public void Update()
        {
            _geometry = SketchCurveTransform.TransformCurve(
                Source,
                Transform,
                DefaultCurveFactory.Instance);
            _geometry.Flags = Flags;
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
            => _geometry.EvaluateVertex(uniform);

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
            => _geometry.Tessellate(maxDeviation);

        public override List<CurveVertex2D> Tessellate(int pointCount)
            => _geometry.Tessellate(pointCount);

        public override double Length()
            => _geometry.Length();

        public override List<Vec2D> ToReferencePoints()
            => _geometry.ToReferencePoints();

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
            => _geometry.UpdateFromReferencePoints(referencePoints);

        public override Curve2D GetCopy()
        {
            var copy = new TransformedSketchCurve2D(Source, Transform, Flags);
            copy.Name = Name;
            return copy;
        }

        public override Curve2D Reverse()
            => _geometry.Reverse();
    }
}
