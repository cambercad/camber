using System.Globalization;
using System.Text;
using GeoCore;
using NURBS;

namespace Geo.Export
{
    /// <summary>
    /// Hand-written IGES 5.3 writer. Entity pointers are Directory Entry sequence numbers (odd).
    /// </summary>
    public sealed class IgesWriter
    {
        private readonly List<IgesRec> _entities = new();
        private readonly CultureInfo _ci = CultureInfo.InvariantCulture;

        private sealed class IgesRec
        {
            public int Type;
            public int Form;
            public int Subordinate;
            public int EntityUseFlag;
            public int TransformDe;
            public List<object> Params = new();
            public int DeNumber;
            public int ParamStart;
            public int ParamLines;
        }

        public int AddEntity(
            int type,
            IEnumerable<object> parameters,
            int form = 0,
            bool dependent = true,
            int transformDe = 0,
            int entityUseFlag = 0)
        {
            var rec = new IgesRec
            {
                Type = type,
                Form = form,
                Subordinate = dependent ? 1 : 0,
                EntityUseFlag = entityUseFlag,
                TransformDe = transformDe,
                DeNumber = 1 + 2 * _entities.Count
            };
            rec.Params.AddRange(parameters);
            _entities.Add(rec);
            return rec.DeNumber;
        }

        public int WritePoint(Vec3D p)
        {
            return AddEntity(116, new object[] { p.X, p.Y, p.Z }, entityUseFlag: 2);
        }

        public int WriteDirection(Vec3D d)
        {
            var n = d.Length() > 1e-12 ? d.Normalized() : new Vec3D(0, 0, 1);
            return AddEntity(123, new object[] { n.X, n.Y, n.Z }, entityUseFlag: 2);
        }

        public int WriteLine(Vec3D p0, Vec3D p1)
        {
            return AddEntity(110, new object[] { p0.X, p0.Y, p0.Z, p1.X, p1.Y, p1.Z });
        }

        public int WriteCircularArc(Vec3D center, Vec3D axis, Vec3D refDir, double radius, Vec3D start, Vec3D end)
        {
            axis = axis.Normalized();
            refDir = Orthonormalize(axis, refDir);
            var yDir = Vec3DOps.Cross(axis, refDir).Normalized();
            int xform = WriteTransform(refDir, yDir, axis, center);

            Vec3D sLocal = WorldToLocal(start - center, refDir, yDir, axis);
            Vec3D eLocal = WorldToLocal(end - center, refDir, yDir, axis);
            return AddEntity(100, new object[]
            {
                0.0,
                0.0, 0.0,
                sLocal.X, sLocal.Y,
                eLocal.X, eLocal.Y
            }, transformDe: xform);
        }

        public int WriteFullCircle(Vec3D center, Vec3D axis, Vec3D refDir, double radius)
        {
            var start = center + Orthonormalize(axis, refDir) * radius;
            return WriteCircularArc(center, axis, refDir, radius, start, start);
        }

        public int WriteCompositeCurve(IReadOnlyList<int> childDe, int entityUseFlag = 0)
        {
            var p = new List<object> { childDe.Count };
            foreach (var id in childDe)
                p.Add(id);
            return AddEntity(102, p, entityUseFlag: entityUseFlag);
        }

        public int WriteUvLine(Vec2D uv0, Vec2D uv1)
        {
            // Type 142 B-curves live in surface parameter space (entity use flag 05).
            return AddEntity(110, new object[] { uv0.X, uv0.Y, 0.0, uv1.X, uv1.Y, 0.0 }, entityUseFlag: 5);
        }

        /// <summary>
        /// Type 142: curve on a parametric surface. B is the UV polyline from triangle corners;
        /// C is the matching model-space mesh boundary.
        /// </summary>
        public int WriteCurveOnParametricSurface(int surfaceDe, int uvCurveDe, int modelCurveDe)
        {
            return AddEntity(142, new object[] { 0, surfaceDe, uvCurveDe, modelCurveDe, 1 });
        }

        public int WritePlaneSurface(Vec3D origin, Vec3D normal, Vec3D refDir)
        {
            int pt = WritePoint(origin);
            int n = WriteDirection(normal);
            int r = WriteDirection(refDir);
            return AddEntity(190, new object[] { pt, n, r }, form: 1);
        }

        public int WriteCylindricalSurface(Vec3D origin, Vec3D axis, Vec3D refDir, double radius)
        {
            int pt = WritePoint(origin);
            int a = WriteDirection(axis);
            int r = WriteDirection(refDir);
            return AddEntity(192, new object[] { pt, a, radius, r }, form: 1);
        }

        public int WriteConicalSurface(Vec3D origin, Vec3D axis, Vec3D refDir, double radius, double semiAngleRadians)
        {
            int pt = WritePoint(origin);
            int a = WriteDirection(axis);
            int r = WriteDirection(refDir);
            double deg = semiAngleRadians * 180.0 / Math.PI;
            return AddEntity(194, new object[] { pt, a, radius, deg, r }, form: 1);
        }

        public int WriteSphericalSurface(Vec3D center, Vec3D axis, Vec3D refDir, double radius)
        {
            int pt = WritePoint(center);
            int a = WriteDirection(axis);
            int r = WriteDirection(refDir);
            return AddEntity(196, new object[] { pt, radius, a, r }, form: 1);
        }

        public int WriteToroidalSurface(Vec3D center, Vec3D axis, Vec3D refDir, double major, double minor)
        {
            int pt = WritePoint(center);
            int a = WriteDirection(axis);
            int r = WriteDirection(refDir);
            return AddEntity(198, new object[] { pt, a, major, minor, r }, form: 1);
        }

        public int WriteNurbsCurve(BSplineCurve curve, int entityUseFlag = 0)
        {
            var points = NurbsEntityConverter.CurveControlPoints(curve, out var weights);
            var knots = curve.Knots;
            int k = points.Count - 1;
            int m = curve.Degree;
            bool rational = NurbsEntityConverter.IsRational(curve);
            // Type 126: K, M, PROP1 planar, PROP2 closed, PROP3 polynomial, PROP4 periodic.
            var p = new List<object>
            {
                k, m,
                0, 0,
                rational ? 0 : 1,
                0
            };
            foreach (var knot in knots)
                p.Add(knot);
            for (int i = 0; i < points.Count; i++)
                p.Add(i < weights.Count ? weights[i] : 1.0);
            foreach (var pt in points)
            {
                p.Add(pt.X);
                p.Add(pt.Y);
                p.Add(pt.Z);
            }
            p.Add(knots[m]);
            p.Add(knots[knots.Length - 1 - m]);
            return AddEntity(126, p, entityUseFlag: entityUseFlag);
        }

        public int WriteUvNurbsCurve(IReadOnlyList<Vec2D> uv)
        {
            if (uv == null || uv.Count < 2)
                throw new ArgumentException("UV polyline needs at least two points.");
            if (uv.Count == 2)
                return WriteUvLine(uv[0], uv[uv.Count - 1]);

            var pts = new Vec3D[uv.Count];
            for (int i = 0; i < uv.Count; i++)
                pts[i] = new Vec3D(uv[i].X, uv[i].Y, 0);
            int degree = Math.Min(3, pts.Length - 1);
            var curve = new BSplineCurve(degree, pts, BSplineCurve.UniformKnotVector(degree, pts.Length), false);
            return WriteNurbsCurve(curve, entityUseFlag: 5);
        }

        public int WriteNurbsSurface(BSplineSurface surface)
        {
            var points = NurbsEntityConverter.FlattenSurfaceControlPoints(surface, out var weights);
            int nu = surface.NumControlPointsU;
            int nv = surface.NumControlPointsV;
            int k1 = nu - 1;
            int k2 = nv - 1;
            int m1 = surface.DegreeU;
            int m2 = surface.DegreeV;
            bool rational = NurbsEntityConverter.IsRational(surface);
            var p = new List<object>
            {
                k1, k2, m1, m2,
                0, 0,
                rational ? 0 : 1,
                0, 0
            };
            foreach (var knot in surface.KnotsU)
                p.Add(knot);
            foreach (var knot in surface.KnotsV)
                p.Add(knot);
            for (int v = 0; v < nv; v++)
            {
                for (int u = 0; u < nu; u++)
                    p.Add(weights[v * nu + u]);
            }
            for (int v = 0; v < nv; v++)
            {
                for (int u = 0; u < nu; u++)
                {
                    var pt = points[v * nu + u];
                    p.Add(pt.X);
                    p.Add(pt.Y);
                    p.Add(pt.Z);
                }
            }
            p.Add(surface.KnotsU[m1]);
            p.Add(surface.KnotsU[surface.KnotsU.Length - 1 - m1]);
            p.Add(surface.KnotsV[m2]);
            p.Add(surface.KnotsV[surface.KnotsV.Length - 1 - m2]);
            return AddEntity(128, p);
        }

        public int WriteTrimmedSurface(int surfaceDe, int outerCompositeDe, IReadOnlyList<int> innerCompositeDe)
        {
            // N1=1: outer boundary is the given 142 curve, not the (possibly unbounded) surface domain.
            var p = new List<object>
            {
                surfaceDe,
                1,
                innerCompositeDe.Count,
                outerCompositeDe
            };
            foreach (var inner in innerCompositeDe)
                p.Add(inner);
            return AddEntity(144, p, dependent: false);
        }

        private int WriteTransform(Vec3D x, Vec3D y, Vec3D z, Vec3D translation)
        {
            return AddEntity(124, new object[]
            {
                x.X, y.X, z.X, translation.X,
                x.Y, y.Y, z.Y, translation.Y,
                x.Z, y.Z, z.Z, translation.Z
            }, entityUseFlag: 2);
        }

        private static Vec3D WorldToLocal(Vec3D v, Vec3D x, Vec3D y, Vec3D z)
        {
            return new Vec3D(Vec3DOps.Dot(v, x), Vec3DOps.Dot(v, y), Vec3DOps.Dot(v, z));
        }

        private static Vec3D Orthonormalize(Vec3D axis, Vec3D refDir)
        {
            var r = refDir - axis * Vec3DOps.Dot(refDir, axis);
            if (r.LengthSquared() < 1e-16)
            {
                var hint = Math.Abs(axis.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
                r = Vec3DOps.Cross(axis, hint);
            }
            return r.Normalized();
        }

        private string P(double v)
        {
            if (double.IsNaN(v) || double.IsInfinity(v))
                return "0.0";
            return v.ToString("0.0################", _ci);
        }

        public void Save(string path)
        {
            var paramLines = new List<(string Data, int De)>();
            foreach (var rec in _entities)
            {
                rec.ParamStart = paramLines.Count + 1;
                var tokens = new List<string> { rec.Type.ToString(_ci) };
                foreach (var raw in rec.Params)
                {
                    if (raw is int i)
                        tokens.Add(i.ToString(_ci));
                    else if (raw is double d)
                        tokens.Add(P(d));
                    else
                        tokens.Add(Convert.ToString(raw, _ci));
                }

                foreach (string chunk in WrapParamTokens(tokens))
                    paramLines.Add((chunk, rec.DeNumber));
                rec.ParamLines = paramLines.Count - rec.ParamStart + 1;
            }

            using var w = new StreamWriter(path, false, new UTF8Encoding(false));
            string now = DateTime.Now.ToString("yyyyMMdd.HHmmss");
            WriteFixed(w, "CSG IGES export", 'S', 1);

            string fileName = Path.GetFileName(path);
            if (string.IsNullOrEmpty(fileName))
                fileName = "part.igs";
            string global =
                "1H,,1H;," +
                Hollerith("CSG") + "," +
                Hollerith(fileName) + "," +
                Hollerith("CSG") + "," +
                Hollerith("1.0") + "," +
                "32,8,23,11,52," +
                Hollerith("CSG") + "," +
                "1.0,2," +
                Hollerith("MM") + "," +
                "1,0.1," +
                Hollerith(now) + "," +
                "1.0E-7,10000.0," +
                Hollerith("CSG") + "," +
                Hollerith("CSG") + "," +
                "11,0," +
                Hollerith(now) + ";";
            int gSeq = 0;
            int gOff = 0;
            while (gOff < global.Length)
            {
                int take = Math.Min(72, global.Length - gOff);
                WriteFixed(w, global.Substring(gOff, take), 'G', ++gSeq);
                gOff += take;
            }

            int dSeq = 0;
            foreach (var rec in _entities)
            {
                string status = $"00{rec.Subordinate:D2}{rec.EntityUseFlag:D2}00";
                WriteDeLine(w, rec.Type, rec.ParamStart, 0, 0, 0, 0, rec.TransformDe, 0, status, ++dSeq);
                WriteDeLine(w, rec.Type, 0, 0, rec.ParamLines, rec.Form, 0, 0, 0, "", ++dSeq);
            }

            int pSeq = 0;
            foreach (var line in paramLines)
            {
                string data = line.Data.PadRight(64);
                string de = line.De.ToString(_ci).PadLeft(8);
                w.Write(data);
                w.Write(de);
                w.Write('P');
                w.WriteLine((++pSeq).ToString(_ci).PadLeft(7));
            }

            string term = $"S{1.ToString(_ci).PadLeft(7)}G{gSeq.ToString(_ci).PadLeft(7)}D{dSeq.ToString(_ci).PadLeft(7)}P{pSeq.ToString(_ci).PadLeft(7)}";
            WriteFixed(w, term, 'T', 1);
        }

        private static List<string> WrapParamTokens(List<string> tokens)
        {
            const int maxData = 64;
            var lines = new List<string>();
            var sb = new StringBuilder();
            for (int i = 0; i < tokens.Count; i++)
            {
                string piece = tokens[i] + (i == tokens.Count - 1 ? ";" : ",");
                if (sb.Length > 0 && sb.Length + piece.Length > maxData)
                {
                    lines.Add(sb.ToString());
                    sb.Clear();
                }
                if (piece.Length > maxData)
                {
                    int offset = 0;
                    while (offset < piece.Length)
                    {
                        int take = Math.Min(maxData, piece.Length - offset);
                        lines.Add(piece.Substring(offset, take));
                        offset += take;
                    }
                    continue;
                }
                sb.Append(piece);
            }
            if (sb.Length > 0)
                lines.Add(sb.ToString());
            return lines;
        }

        private static string Hollerith(string s)
        {
            if (string.IsNullOrEmpty(s))
                return "1H ";
            return s.Length.ToString(CultureInfo.InvariantCulture) + "H" + s;
        }

        private static void WriteFixed(StreamWriter w, string data, char section, int seq)
        {
            if (data.Length > 72)
                data = data.Substring(0, 72);
            w.Write(data.PadRight(72));
            w.Write(section);
            w.WriteLine(seq.ToString(CultureInfo.InvariantCulture).PadLeft(7));
        }

        private static void WriteDeLine(
            StreamWriter w,
            int f1, int f2, int f3, int f4, int f5, int f6, int f7, int f8, string f9, int seq)
        {
            w.Write(Field(f1));
            w.Write(Field(f2));
            w.Write(Field(f3));
            w.Write(Field(f4));
            w.Write(Field(f5));
            w.Write(Field(f6));
            w.Write(Field(f7));
            w.Write(Field(f8));
            w.Write(f9.PadLeft(8));
            w.Write('D');
            w.WriteLine(seq.ToString(CultureInfo.InvariantCulture).PadLeft(7));
        }

        private static string Field(int v)
        {
            return v == 0 ? new string(' ', 8) : v.ToString(CultureInfo.InvariantCulture).PadLeft(8);
        }
    }
}
