using System;

namespace GeoCore
{

    public enum CurveType
    {
        Unknown,
        Line2D,
        Line3D,
        Circle2D,
        Circle3D,
        Arc2D,
        Arc3D,
        Bezier2D,
        Bezier3D,
        Spline2D,
        Spline3D,
        HermiteSpline2D,
        HermiteSpline3D,
        Ellipse2D,
        BSpline2D,
    }

    public enum SurfaceType
    {
        Unknown,
        Planar,
        Cylindrical,
        Conical,
        Spherical,
        Toroidal
    }

    public enum EdgeCurveType
    {
        Unknown,
        Line,
        Circle,
        Arc
    }

    public sealed class PlaneSurfaceParams
    {
        public Vec3D Origin;
        public Vec3D Normal;
        public Vec3D RefDir;

        public PlaneSurfaceParams Clone()
        {
            return new PlaneSurfaceParams { Origin = Origin, Normal = Normal, RefDir = RefDir };
        }

        public void Transform(in Mat4D t)
        {
            Origin = t.TransformPoint(Origin);
            Normal = t.TransformDirection(Normal);
            RefDir = t.TransformDirection(RefDir);
        }
    }

    public sealed class CylinderSurfaceParams
    {
        public Vec3D Origin;
        public Vec3D Axis;
        public Vec3D RefDir;
        public double Radius;
        public double Height;

        public CylinderSurfaceParams Clone()
        {
            return new CylinderSurfaceParams
            {
                Origin = Origin,
                Axis = Axis,
                RefDir = RefDir,
                Radius = Radius,
                Height = Height
            };
        }

        public void Transform(in Mat4D t)
        {
            Vec3D end = Origin + Axis.Normalized() * Height;
            Origin = t.TransformPoint(Origin);
            end = t.TransformPoint(end);
            Axis = t.TransformDirection(Axis);
            RefDir = t.TransformDirection(RefDir);
            Height = (end - Origin).Length();
        }
    }

    public sealed class ConeSurfaceParams
    {
        public Vec3D Origin;
        public Vec3D Axis;
        public Vec3D RefDir;
        public double Radius;
        public double SemiAngle;
        public double Height;

        public ConeSurfaceParams Clone()
        {
            return new ConeSurfaceParams
            {
                Origin = Origin,
                Axis = Axis,
                RefDir = RefDir,
                Radius = Radius,
                SemiAngle = SemiAngle,
                Height = Height
            };
        }

        public void Transform(in Mat4D t)
        {
            Vec3D end = Origin + Axis.Normalized() * Height;
            Origin = t.TransformPoint(Origin);
            end = t.TransformPoint(end);
            Axis = t.TransformDirection(Axis);
            RefDir = t.TransformDirection(RefDir);
            Height = (end - Origin).Length();
        }
    }

    public sealed class SphereSurfaceParams
    {
        public Vec3D Center;
        public Vec3D Axis;
        public Vec3D RefDir;
        public double Radius;

        public SphereSurfaceParams Clone()
        {
            return new SphereSurfaceParams
            {
                Center = Center,
                Axis = Axis,
                RefDir = RefDir,
                Radius = Radius
            };
        }

        public void Transform(in Mat4D t)
        {
            Center = t.TransformPoint(Center);
            Axis = t.TransformDirection(Axis);
            RefDir = t.TransformDirection(RefDir);
        }
    }

    public sealed class TorusSurfaceParams
    {
        public Vec3D Center;
        public Vec3D Axis;
        public Vec3D RefDir;
        public double MajorRadius;
        public double MinorRadius;

        public TorusSurfaceParams Clone()
        {
            return new TorusSurfaceParams
            {
                Center = Center,
                Axis = Axis,
                RefDir = RefDir,
                MajorRadius = MajorRadius,
                MinorRadius = MinorRadius
            };
        }

        public void Transform(in Mat4D t)
        {
            Center = t.TransformPoint(Center);
            Axis = t.TransformDirection(Axis);
            RefDir = t.TransformDirection(RefDir);
        }
    }

    public class EdgeMetaData
    {
        public EdgeCurveType CurveType { get; set; }
        public Vec3D LineStart;
        public Vec3D LineEnd;
        public Vec3D CircleCenter;
        public Vec3D CircleAxis;
        public Vec3D CircleRefDir;
        public double CircleRadius;
        public Vec3D ArcStart;
        public Vec3D ArcEnd;

        public bool HasCurve => CurveType != EdgeCurveType.Unknown;

        public EdgeMetaData()
        {
        }

        public EdgeMetaData(EdgeCurveType curveType)
        {
            CurveType = curveType;
        }

        public static EdgeMetaData Line(Vec3D start, Vec3D end)
        {
            return new EdgeMetaData(EdgeCurveType.Line)
            {
                LineStart = start,
                LineEnd = end
            };
        }

        public static EdgeMetaData Circle(Vec3D center, Vec3D axis, Vec3D refDir, double radius)
        {
            return new EdgeMetaData(EdgeCurveType.Circle)
            {
                CircleCenter = center,
                CircleAxis = axis,
                CircleRefDir = refDir,
                CircleRadius = radius
            };
        }

        public static EdgeMetaData Arc(Vec3D center, Vec3D axis, Vec3D refDir, double radius, Vec3D start, Vec3D end)
        {
            return new EdgeMetaData(EdgeCurveType.Arc)
            {
                CircleCenter = center,
                CircleAxis = axis,
                CircleRefDir = refDir,
                CircleRadius = radius,
                ArcStart = start,
                ArcEnd = end
            };
        }

        public EdgeMetaData Clone()
        {
            return new EdgeMetaData(CurveType)
            {
                LineStart = LineStart,
                LineEnd = LineEnd,
                CircleCenter = CircleCenter,
                CircleAxis = CircleAxis,
                CircleRefDir = CircleRefDir,
                CircleRadius = CircleRadius,
                ArcStart = ArcStart,
                ArcEnd = ArcEnd
            };
        }

        public void Transform(in Mat4D t)
        {
            LineStart = t.TransformPoint(LineStart);
            LineEnd = t.TransformPoint(LineEnd);
            CircleCenter = t.TransformPoint(CircleCenter);
            CircleAxis = t.TransformDirection(CircleAxis);
            CircleRefDir = t.TransformDirection(CircleRefDir);
            ArcStart = t.TransformPoint(ArcStart);
            ArcEnd = t.TransformPoint(ArcEnd);
        }
    }

    public class SurfaceMetaData
    {
        public SurfaceType SurfaceType { get; set; }
        public ParametricRange? ParamRange { get; set; }
        public PlaneSurfaceParams PlaneParams { get; set; }
        public CylinderSurfaceParams CylinderParams { get; set; }
        public ConeSurfaceParams ConeParams { get; set; }
        public SphereSurfaceParams SphereParams { get; set; }
        public TorusSurfaceParams TorusParams { get; set; }

        private INurbsSurface _nurbsSurface;
        private Func<INurbsSurface> _nurbsFactory;

        /// <summary>True when a NURBS surface is available (built or deferred).</summary>
        public bool HasNurbs => _nurbsSurface != null || _nurbsFactory != null;

        /// <summary>True after <see cref="NurbsSurface"/> has been built at least once.</summary>
        public bool IsNurbsMaterialized => _nurbsSurface != null;

        public bool HasAnalyticParams =>
            PlaneParams != null || CylinderParams != null || ConeParams != null ||
            SphereParams != null || TorusParams != null;

        public INurbsSurface NurbsSurface
        {
            get => _nurbsSurface ??= _nurbsFactory?.Invoke();
            set
            {
                _nurbsSurface = value;
                _nurbsFactory = null;
            }
        }

        public SurfaceMetaData(SurfaceType surfaceType, ParametricRange? paramRange = null)
        {
            SurfaceType = surfaceType;
            ParamRange = paramRange;
        }

        public SurfaceMetaData(SurfaceType surfaceType, INurbsSurface nurbsSurface, ParametricRange? paramRange = null)
            : this(surfaceType, paramRange)
        {
            _nurbsSurface = nurbsSurface;
        }

        public SurfaceMetaData(SurfaceType surfaceType, Func<INurbsSurface> nurbsFactory, ParametricRange? paramRange = null)
            : this(surfaceType, paramRange)
        {
            _nurbsFactory = nurbsFactory ?? throw new ArgumentNullException(nameof(nurbsFactory));
        }

        public SurfaceMetaData Clone()
        {
            return new SurfaceMetaData(this);
        }

        public static Dictionary<string, SurfaceMetaData> CloneDictionary(Dictionary<string, SurfaceMetaData> source)
        {
            var result = new Dictionary<string, SurfaceMetaData>();
            if (source == null)
                return result;
            foreach (var kv in source)
            {
                if (kv.Value == null)
                    continue;
                result[kv.Key] = kv.Value.Clone();
            }
            return result;
        }

        public void Transform(in Mat4D t)
        {
            PlaneParams?.Transform(in t);
            CylinderParams?.Transform(in t);
            ConeParams?.Transform(in t);
            SphereParams?.Transform(in t);
            TorusParams?.Transform(in t);

            if (_nurbsSurface != null)
                _nurbsSurface = TransformedNurbsSurface.Apply(_nurbsSurface, in t);
            else if (_nurbsFactory != null)
            {
                var innerFactory = _nurbsFactory;
                var captured = t;
                _nurbsFactory = () => TransformedNurbsSurface.Apply(innerFactory(), captured);
            }
        }

        private SurfaceMetaData(SurfaceMetaData source)
        {
            SurfaceType = source.SurfaceType;
            ParamRange = source.ParamRange;
            PlaneParams = source.PlaneParams?.Clone();
            CylinderParams = source.CylinderParams?.Clone();
            ConeParams = source.ConeParams?.Clone();
            SphereParams = source.SphereParams?.Clone();
            TorusParams = source.TorusParams?.Clone();
            _nurbsSurface = source._nurbsSurface;
            _nurbsFactory = source._nurbsFactory;
        }
    }

    public sealed class TransformedNurbsSurface : INurbsSurface
    {
        public INurbsSurface Inner { get; }
        public Mat4D Transform { get; }

        public TransformedNurbsSurface(INurbsSurface inner, Mat4D transform)
        {
            Inner = inner ?? throw new ArgumentNullException(nameof(inner));
            Transform = transform;
        }

        /// <summary>
        /// Applies <paramref name="t"/> without stacking wrappers: T2(T1(p)) becomes one matrix.
        /// </summary>
        public static INurbsSurface Apply(INurbsSurface inner, in Mat4D t)
        {
            if (inner is TransformedNurbsSurface existing)
                return new TransformedNurbsSurface(existing.Inner, t * existing.Transform);
            return new TransformedNurbsSurface(inner, t);
        }

        public Vec3D Evaluate(double u, double v)
        {
            return Transform.TransformPoint(Inner.Evaluate(u, v));
        }

        public Vec3D EvaluateNormal(double u, double v)
        {
            var n = Transform.TransformDirection(Inner.EvaluateNormal(u, v));
            double len = n.Length();
            return len > 1e-15 ? n * (1.0 / len) : n;
        }
    }

    public class CurveMetaData
    {
        public CurveType CurveType { get; set; }
        public object Instance { get; set; }
        /// <summary>
        /// Flags from the original curve (e.g., HelperGeometry for construction lines).
        /// </summary>
        public int Flags { get; set; }

        public CurveMetaData(CurveType curveType, object instance = null, int flags = 0)
        {
            CurveType = curveType;
            Instance = instance;
            Flags = flags;
        }

        /// <summary>
        /// Returns true if the curve is marked as helper/construction geometry.
        /// </summary>
        public bool IsHelperGeometry { get { return (Flags & 1) != 0; } }
    }
}
