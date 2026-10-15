using System.Globalization;
using System.Text;
using System.Xml.Linq;
using Curves;
using Curves.Base;
using GeoCore;
using GeoSolver;
using GeoSolver.Sketcher;

namespace Geo
{
    /// <summary>
    /// Sketch ↔ SVG. Geometry is analytic for lines, circles, circular arcs, ellipses and Béziers.
    /// Coordinates are written Y-down (page upright). Load them with <c>flipY: true</c>
    /// so page-up is sketch +Y. Constraint markup lives in a group that import ignores.
    /// </summary>
    public static class SvgSketchWriter
    {
        const string Ns = "http://www.w3.org/2000/svg";

        public static void Write(string fileName, PlotterSketcher sketch, bool includeAnnotations, double maxDeviation)
        {
            if (sketch == null)
                throw new ArgumentNullException(nameof(sketch));
            if (string.IsNullOrWhiteSpace(fileName))
                throw new ArgumentException("SVG path is required.", nameof(fileName));
            if (maxDeviation <= 0)
                maxDeviation = 0.01;

            IReadOnlyList<Curve2D> curves = sketch.GetAllCurves();
            var geometry = new StringBuilder();
            Vec2D min = new Vec2D(double.MaxValue, double.MaxValue);
            Vec2D max = new Vec2D(double.MinValue, double.MinValue);
            var used = new HashSet<string>(StringComparer.Ordinal);
            if (curves != null)
            {
                foreach (Curve2D curve in curves)
                {
                    if (curve == null)
                        continue;
                    Expand(curve, maxDeviation, ref min, ref max);
                    geometry.AppendLine(CurveMarkup(curve, maxDeviation, used));
                }
            }

            var annotations = new StringBuilder();
            var labels = new StringBuilder();
            if (includeAnnotations && sketch is ConstrainedSketcher constrained)
                WriteAnnotations(annotations, labels, constrained.GetConstraints(), ref min, ref max);

            if (min.X > max.X)
            {
                min = new Vec2D(-1, -1);
                max = new Vec2D(1, 1);
            }
            double span = Math.Max(max.X - min.X, max.Y - min.Y);
            double pad = Math.Max(span * 0.08, maxDeviation);
            min -= new Vec2D(pad, pad);
            max += new Vec2D(pad, pad);
            double width = Math.Max(max.X - min.X, 1e-9);
            double height = Math.Max(max.Y - min.Y, 1e-9);
            double font = Math.Max(span * 0.035, pad * 0.35);
            // Page coordinates negate sketch Y, so the viewBox origin is the top of the drawing.
            double pageTop = -max.Y;

            var svg = new StringBuilder();
            svg.AppendLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
            svg.Append("<svg xmlns=\"").Append(Ns).Append("\" viewBox=\"");
            svg.Append(F(min.X)).Append(' ').Append(F(pageTop)).Append(' ').Append(F(width)).Append(' ').Append(F(height));
            svg.AppendLine("\">");
            svg.AppendLine("  <g id=\"geometry\">");
            svg.Append(geometry);
            svg.AppendLine("  </g>");
            if (annotations.Length > 0)
            {
                svg.AppendLine("  <g id=\"camber-annotations\" data-camber=\"annotations\"");
                svg.Append(" fill=\"none\" stroke=\"#c45c00\" stroke-width=\"").Append(F(font * 0.06)).AppendLine("\">");
                svg.Append(annotations);
                svg.Append("    <g fill=\"#c45c00\" stroke=\"none\" font-family=\"sans-serif\" font-size=\"").Append(F(font)).AppendLine("\" text-anchor=\"middle\">");
                svg.Append(labels);
                svg.AppendLine("    </g>");
                svg.AppendLine("  </g>");
            }
            svg.AppendLine("</svg>");
            string dir = Path.GetDirectoryName(fileName);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);
            File.WriteAllText(fileName, svg.ToString(), new UTF8Encoding(false));
        }

        static void Expand(Curve2D curve, double deviation, ref Vec2D min, ref Vec2D max)
        {
            if (curve is Circle2D circle)
            {
                Include(ref min, ref max, circle.Center + new Vec2D(circle.Radius, circle.Radius));
                Include(ref min, ref max, circle.Center - new Vec2D(circle.Radius, circle.Radius));
                return;
            }
            List<CurveVertex2D> verts = curve.Tessellate(deviation);
            if (verts == null)
                return;
            foreach (CurveVertex2D v in verts)
                Include(ref min, ref max, v.Position);
        }

        static void Include(ref Vec2D min, ref Vec2D max, Vec2D p)
        {
            if (p.X < min.X) min.X = p.X;
            if (p.Y < min.Y) min.Y = p.Y;
            if (p.X > max.X) max.X = p.X;
            if (p.Y > max.Y) max.Y = p.Y;
        }

        static string CurveMarkup(Curve2D curve, double deviation, HashSet<string> used)
        {
            string id = UniqueId(string.IsNullOrEmpty(curve.Name) ? "curve" : curve.Name, used);
            bool construction = curve.IsHelperGeometry;
            string extra = construction ? " data-construction=\"1\" stroke-dasharray=\"0.12 0.08\"" : "";
            string stroke = construction ? "#2f6fed" : "#1a1a1a";
            string common = " id=\"" + Xml(id) + "\" fill=\"none\" stroke=\"" + stroke + "\" stroke-width=\"0.02\"" + extra;

            if (curve is Line2D line)
            {
                Vec2D a = Page(line.StartPosition), b = Page(line.EndPosition);
                return "    <line" + common + " x1=\"" + F(a.X) + "\" y1=\"" + F(a.Y) + "\" x2=\"" + F(b.X) + "\" y2=\"" + F(b.Y) + "\"/>";
            }
            if (curve is Circle2D circle)
            {
                Vec2D c = Page(circle.Center);
                return "    <circle" + common + " cx=\"" + F(c.X) + "\" cy=\"" + F(c.Y) + "\" r=\"" + F(circle.Radius) + "\"/>";
            }
            if (curve is Arc2D arc)
            {
                Vec2D a = Page(arc.StartPosition), b = Page(arc.EndPosition);
                double sweep = arc.AngleEnd - arc.AngleStart;
                int large = Math.Abs(sweep) > Math.PI ? 1 : 0;
                // Negating Y reverses orientation, so the SVG sweep flag is inverted.
                int positive = sweep >= 0 ? 0 : 1;
                return "    <path" + common + " d=\"M " + F(a.X) + " " + F(a.Y) + " A " + F(arc.Radius) + " " + F(arc.Radius)
                    + " 0 " + large + " " + positive + " " + F(b.X) + " " + F(b.Y) + "\"/>";
            }
            if (curve is Ellipse2D ellipse)
            {
                double rotation = -Math.Atan2(ellipse.MajorAxis.Y, ellipse.MajorAxis.X) * 180.0 / Math.PI;
                Vec2D c = Page(ellipse.Center);
                if (ellipse.IsFullEllipse)
                {
                    return "    <ellipse" + common + " cx=\"" + F(c.X) + "\" cy=\"" + F(c.Y)
                        + "\" rx=\"" + F(ellipse.MajorAxisLength) + "\" ry=\"" + F(ellipse.MinorAxisLength)
                        + "\" transform=\"rotate(" + F(rotation) + " " + F(c.X) + " " + F(c.Y) + ")\"/>";
                }
                Vec2D a = Page(ellipse.StartPosition), b = Page(ellipse.EndPosition);
                double sweep = ellipse.EndAngle - ellipse.StartAngle;
                int large = Math.Abs(sweep) > Math.PI ? 1 : 0;
                int positive = sweep >= 0 ? 0 : 1;
                return "    <path" + common + " d=\"M " + F(a.X) + " " + F(a.Y) + " A " + F(ellipse.MajorAxisLength) + " "
                    + F(ellipse.MinorAxisLength) + " " + F(rotation) + " " + large + " " + positive + " " + F(b.X) + " " + F(b.Y) + "\"/>";
            }
            if (curve is Bezier2D bezier && bezier.ControlPoints.Length >= 2 && bezier.ControlPoints.Length <= 4)
            {
                Vec2D[] c = bezier.ControlPoints;
                var d = new StringBuilder();
                Vec2D p0 = Page(c[0]);
                d.Append("M ").Append(F(p0.X)).Append(' ').Append(F(p0.Y));
                if (c.Length == 2)
                {
                    Vec2D p1 = Page(c[1]);
                    d.Append(" L ").Append(F(p1.X)).Append(' ').Append(F(p1.Y));
                }
                else if (c.Length == 3)
                {
                    Vec2D p1 = Page(c[1]);
                    Vec2D p2 = Page(c[2]);
                    d.Append(" Q ").Append(F(p1.X)).Append(' ').Append(F(p1.Y)).Append(' ').Append(F(p2.X)).Append(' ').Append(F(p2.Y));
                }
                else
                {
                    Vec2D p1 = Page(c[1]);
                    Vec2D p2 = Page(c[2]);
                    Vec2D p3 = Page(c[3]);
                    d.Append(" C ").Append(F(p1.X)).Append(' ').Append(F(p1.Y)).Append(' ')
                        .Append(F(p2.X)).Append(' ').Append(F(p2.Y)).Append(' ')
                        .Append(F(p3.X)).Append(' ').Append(F(p3.Y));
                }
                return "    <path" + common + " data-kind=\"bezier\" d=\"" + d + "\"/>";
            }

            List<CurveVertex2D> verts = curve.Tessellate(deviation);
            var path = new StringBuilder();
            if (verts != null)
            {
                for (int i = 0; i < verts.Count; i++)
                {
                    Vec2D p = Page(verts[i].Position);
                    path.Append(i == 0 ? "M " : " L ").Append(F(p.X)).Append(' ').Append(F(p.Y));
                }
            }
            return "    <path" + common + " data-kind=\"sampled\" d=\"" + path + "\"/>";
        }

        static void WriteAnnotations(StringBuilder svg, StringBuilder labels, IList<IBaseEquation> constraints, ref Vec2D min, ref Vec2D max)
        {
            if (constraints == null)
                return;
            int slot = 0;
            foreach (IBaseEquation constraint in constraints)
            {
                if (constraint == null || constraint is UnitVector2d)
                    continue;
                try
                {
                    if (!AppendAnnotation(svg, labels, constraint, slot, ref min, ref max))
                        continue;
                    slot++;
                }
                catch
                {
                    // A constraint the solver uses internally may not have drawable points.
                }
            }
        }

        static bool AppendAnnotation(StringBuilder svg, StringBuilder labels, IBaseEquation constraint, int slot, ref Vec2D min, ref Vec2D max)
        {
            switch (constraint)
            {
                case DistanceBetweenPoints2d distance:
                    Dimension(svg, labels, distance.Point1.Evaluate(), distance.Point2.Evaluate(), Num(distance.Distance.Evaluate()), "distance", slot, ref min, ref max);
                    return true;
                case HorizontalDistanceBetweenPoints2d horizontal:
                    {
                        Vec2D a = horizontal.Point1.Evaluate();
                        Vec2D b = horizontal.Point2.Evaluate();
                        Dimension(svg, labels, a, new Vec2D(b.X, a.Y), Num(horizontal.Distance), "horizontal-distance", slot, ref min, ref max);
                        return true;
                    }
                case VerticalDistanceBetweenPoints2d vertical:
                    {
                        Vec2D a = vertical.Point1.Evaluate();
                        Vec2D b = vertical.Point2.Evaluate();
                        Dimension(svg, labels, a, new Vec2D(a.X, b.Y), Num(vertical.Distance), "vertical-distance", slot, ref min, ref max);
                        return true;
                    }
                case DistancePointLine2d offset:
                    {
                        Vec2D point = offset.Point.Evaluate();
                        Vec2D a = offset.Line.CStart.Evaluate();
                        Vec2D b = offset.Line.CEnd.Evaluate();
                        Vec2D dir = b - a;
                        double len2 = dir.X * dir.X + dir.Y * dir.Y;
                        double t = len2 > 0 ? ((point.X - a.X) * dir.X + (point.Y - a.Y) * dir.Y) / len2 : 0;
                        Dimension(svg, labels, point, a + dir * t, Num(offset.Distance), "distance-to-line", slot, ref min, ref max);
                        return true;
                    }
                case RadiusConstraint2d radius:
                    {
                        Vec2D center = radius.Circular.CCenter.Evaluate();
                        Vec2D rim = center + new Vec2D(radius.Radius, 0);
                        Dimension(svg, labels, center, rim, "R" + Num(radius.Radius), "radius", slot, ref min, ref max);
                        return true;
                    }
                case AngleBetweenLines2d angle:
                    LabelAt(labels, IntersectOrMid(angle.Line1, angle.Line2), Num(angle.AngleRadians * 180.0 / Math.PI) + "°", "angle", slot, ref min, ref max);
                    return true;
                case AngleBetweenLineAndArc2d angleArc:
                    LabelAt(labels, angleArc.Line.CStart.Evaluate(), Num(angleArc.AngleRadians * 180.0 / Math.PI) + "°", "angle", slot, ref min, ref max);
                    return true;
                case PointOnPoint2d coincident:
                    Mark(svg, (coincident.Point1.Evaluate() + coincident.Point2.Evaluate()) * 0.5, "coincident", ref min, ref max);
                    return true;
                case Horizontal2d horizontalLine:
                    LabelAt(labels, Mid(horizontalLine.Line), "H", "horizontal", slot, ref min, ref max);
                    return true;
                case Vertical2d verticalLine:
                    LabelAt(labels, Mid(verticalLine.Line), "V", "vertical", slot, ref min, ref max);
                    return true;
                case Parallel2d parallel:
                    LabelAt(labels, (Mid(parallel.Line1) + Mid(parallel.Line2)) * 0.5, "∥", "parallel", slot, ref min, ref max);
                    return true;
                case Perpendicular2d perpendicular:
                    LabelAt(labels, IntersectOrMid(perpendicular.Line1, perpendicular.Line2), "⊥", "perpendicular", slot, ref min, ref max);
                    return true;
                case EqualLength2d equal:
                    LabelAt(labels, (Mid(equal.Line1) + Mid(equal.Line2)) * 0.5, "=", "equal-length", slot, ref min, ref max);
                    return true;
                case EqualRadius2d equalRadius:
                    LabelAt(labels, (equalRadius.Circular1.CCenter.Evaluate() + equalRadius.Circular2.CCenter.Evaluate()) * 0.5, "R=", "equal-radius", slot, ref min, ref max);
                    return true;
                case Concentric2d concentric:
                    Mark(svg, concentric.Circular1.CCenter.Evaluate(), "concentric", ref min, ref max);
                    return true;
                case PointOnLineConstraint2d onLine:
                    Mark(svg, onLine.Point.Evaluate(), "point-on-line", ref min, ref max);
                    return true;
                case PointOnCircle2d onCircle:
                    Mark(svg, onCircle.Point.Evaluate(), "point-on-circle", ref min, ref max);
                    return true;
                case PointOnArc2d onArc:
                    Mark(svg, onArc.Point.Evaluate(), "point-on-arc", ref min, ref max);
                    return true;
                case Midpoint2d midpoint:
                    Mark(svg, midpoint.Point.Evaluate(), "midpoint", ref min, ref max);
                    return true;
                case TangentLineCircular2d tangent:
                    LabelAt(labels, tangent.Circular.CCenter.Evaluate(), "T", "tangent", slot, ref min, ref max);
                    return true;
                case TangentCircularCircular2d tangentCircles:
                    LabelAt(labels, (tangentCircles.Circular1.CCenter.Evaluate() + tangentCircles.Circular2.CCenter.Evaluate()) * 0.5, "T", "tangent", slot, ref min, ref max);
                    return true;
                default:
                    return false;
            }
        }

        static Vec2D Mid(CLine2D line)
        {
            return (line.CStart.Evaluate() + line.CEnd.Evaluate()) * 0.5;
        }

        static Vec2D IntersectOrMid(CLine2D a, CLine2D b)
        {
            Vec2D p = a.CStart.Evaluate();
            Vec2D r = a.CEnd.Evaluate() - p;
            Vec2D q = b.CStart.Evaluate();
            Vec2D s = b.CEnd.Evaluate() - q;
            double den = r.X * s.Y - r.Y * s.X;
            if (Math.Abs(den) < 1e-12)
                return (Mid(a) + Mid(b)) * 0.5;
            double t = ((q.X - p.X) * s.Y - (q.Y - p.Y) * s.X) / den;
            return p + r * t;
        }

        static void Dimension(StringBuilder svg, StringBuilder labels, Vec2D a, Vec2D b, string text, string kind, int slot, ref Vec2D min, ref Vec2D max)
        {
            Vec2D delta = b - a;
            double len = Math.Sqrt(delta.X * delta.X + delta.Y * delta.Y);
            Vec2D dir = len > 1e-12 ? delta / len : new Vec2D(1, 0);
            Vec2D normal = new Vec2D(-dir.Y, dir.X);
            double off = (0.04 + 0.03 * (slot % 4)) * (slot % 2 == 0 ? 1 : -1);
            if (Math.Abs(off) < 1e-6)
                off = 0.04;
            // Offset is a fraction of a unit only when the sketch is tiny; scale with the segment.
            double magnitude = Math.Max(len * Math.Abs(off), 0.02) * Math.Sign(off == 0 ? 1 : off);
            Vec2D a2 = a + normal * magnitude;
            Vec2D b2 = b + normal * magnitude;
            Vec2D pa = Page(a), pa2 = Page(a2), pb = Page(b), pb2 = Page(b2);
            svg.Append("    <path data-camber=\"annotation\" data-kind=\"").Append(kind).Append("\" d=\"M ")
                .Append(F(pa.X)).Append(' ').Append(F(pa.Y)).Append(" L ").Append(F(pa2.X)).Append(' ').Append(F(pa2.Y))
                .Append(" M ").Append(F(pb.X)).Append(' ').Append(F(pb.Y)).Append(" L ").Append(F(pb2.X)).Append(' ').Append(F(pb2.Y))
                .Append(" M ").Append(F(pa2.X)).Append(' ').Append(F(pa2.Y)).Append(" L ").Append(F(pb2.X)).Append(' ').Append(F(pb2.Y))
                .AppendLine("\"/>");
            Include(ref min, ref max, a2);
            Include(ref min, ref max, b2);
            LabelAt(labels, (a2 + b2) * 0.5, text, kind, slot, ref min, ref max);
        }

        static void Mark(StringBuilder svg, Vec2D p, string kind, ref Vec2D min, ref Vec2D max)
        {
            const double s = 0.04;
            Vec2D page = Page(p);
            svg.Append("    <path data-camber=\"annotation\" data-kind=\"").Append(kind).Append("\" d=\"M ")
                .Append(F(page.X - s)).Append(' ').Append(F(page.Y - s)).Append(" L ").Append(F(page.X + s)).Append(' ').Append(F(page.Y + s))
                .Append(" M ").Append(F(page.X - s)).Append(' ').Append(F(page.Y + s)).Append(" L ").Append(F(page.X + s)).Append(' ').Append(F(page.Y - s))
                .AppendLine("\"/>");
            Include(ref min, ref max, p);
        }

        static void LabelAt(StringBuilder labels, Vec2D p, string text, string kind, int slot, ref Vec2D min, ref Vec2D max)
        {
            double nudge = 0.02 * (slot % 5);
            Vec2D at = p + new Vec2D(nudge, nudge);
            Vec2D pageAt = Page(at);
            labels.Append("      <text data-camber=\"annotation\" data-kind=\"").Append(kind)
                .Append("\" x=\"").Append(F(pageAt.X)).Append("\" y=\"").Append(F(pageAt.Y)).Append("\">")
                .Append(Xml(text)).AppendLine("</text>");
            Include(ref min, ref max, at);
        }

        static string Num(double value) => value.ToString("0.###", CultureInfo.InvariantCulture);

        static string UniqueId(string name, HashSet<string> used)
        {
            var sb = new StringBuilder(name.Length);
            foreach (char c in name)
            {
                if (char.IsLetterOrDigit(c) || c == '_' || c == '-' || c == '.')
                    sb.Append(c);
                else if (sb.Length > 0 && sb[sb.Length - 1] != '_')
                    sb.Append('_');
            }
            if (sb.Length == 0 || char.IsDigit(sb[0]))
                sb.Insert(0, "c_");
            string id = sb.ToString();
            string unique = id;
            int n = 2;
            while (!used.Add(unique))
                unique = id + "_" + n++;
            return unique;
        }

        static Vec2D Page(Vec2D p) => new Vec2D(p.X, -p.Y);

        internal static string F(double value)
        {
            if (!double.IsFinite(value))
                return "0";
            return value.ToString("0.############", CultureInfo.InvariantCulture);
        }

        static string Xml(string text)
        {
            return (text ?? "").Replace("&", "&amp;").Replace("<", "&lt;").Replace(">", "&gt;").Replace("\"", "&quot;");
        }
    }

    public static class SvgSketchReader
    {
        public static int Load(PlotterSketcher sketch, string fileName, double scale = 1, bool flipY = false)
        {
            if (sketch == null)
                throw new ArgumentNullException(nameof(sketch));
            if (scale <= 0)
                throw new ArgumentOutOfRangeException(nameof(scale), "Scale must be positive.");
            XDocument doc = XDocument.Load(fileName);
            if (doc.Root == null)
                throw new InvalidDataException("SVG file is empty.");
            int before = sketch.GetAllCurves().Count;
            var xform = SvgTransform.Identity;
            if (flipY)
                xform = SvgTransform.Scale(1, -1);
            if (scale != 1)
                xform = SvgTransform.Scale(scale, scale).Multiply(xform);
            Walk(doc.Root, xform, sketch);
            int added = sketch.GetAllCurves().Count - before;
            if (added <= 0)
                throw new InvalidDataException("SVG file contains no supported geometry.");
            return added;
        }

        static void Walk(XElement element, SvgTransform xform, PlotterSketcher sketch)
        {
            if (element == null || IsHidden(element) || IsAnnotation(element) || IsDefinition(element))
                return;
            SvgTransform local = xform.Multiply(SvgTransform.Parse(Attr(element, "transform")));
            string name = element.Name.LocalName;
            if (name == "g" || name == "svg" || name == "a")
            {
                foreach (XElement child in element.Elements())
                    Walk(child, local, sketch);
                return;
            }
            bool construction = IsConstruction(element);
            string id = Attr(element, "id");
            var flags = construction ? CurveFlags.HelperGeometry : CurveFlags.None;
            switch (name)
            {
                case "line":
                    AddLine(sketch, local, id, flags,
                        Num(element, "x1"), Num(element, "y1"), Num(element, "x2"), Num(element, "y2"));
                    break;
                case "circle":
                    AddCircle(sketch, local, id, flags, new Vec2D(Num(element, "cx"), Num(element, "cy")), Num(element, "r"));
                    break;
                case "ellipse":
                    AddEllipseElement(sketch, local, id, flags, element);
                    break;
                case "rect":
                    AddRect(sketch, local, id, flags, element);
                    break;
                case "polyline":
                case "polygon":
                    AddPoly(sketch, local, id, flags, Attr(element, "points"), name == "polygon");
                    break;
                case "path":
                    AddPath(sketch, local, id, flags, Attr(element, "d"), Attr(element, "data-kind") == "sampled");
                    break;
            }
        }

        static void AddLine(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, double x1, double y1, double x2, double y2)
        {
            Vec2D a = xform.Apply(new Vec2D(x1, y1));
            Vec2D b = xform.Apply(new Vec2D(x2, y2));
            if ((b - a).LengthSquared() < 1e-20)
                return;
            Name(sketch.AddLine(a, b, flags), id, sketch);
        }

        static void AddCircle(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, Vec2D center, double radius)
        {
            if (radius <= 0)
                return;
            Vec2D c = xform.Apply(center);
            Vec2D axisX = xform.ApplyVector(new Vec2D(radius, 0));
            Vec2D axisY = xform.ApplyVector(new Vec2D(0, radius));
            AddEllipseAxes(sketch, id, flags, c, axisX, axisY, 0, 2 * Math.PI, fullCircle: true);
        }

        static void AddEllipseElement(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, XElement element)
        {
            double rx = Num(element, "rx");
            double ry = Num(element, "ry");
            if (rx <= 0 || ry <= 0)
                return;
            Vec2D c = xform.Apply(new Vec2D(Num(element, "cx"), Num(element, "cy")));
            Vec2D axisX = xform.ApplyVector(new Vec2D(rx, 0));
            Vec2D axisY = xform.ApplyVector(new Vec2D(0, ry));
            AddEllipseAxes(sketch, id, flags, c, axisX, axisY, 0, 2 * Math.PI, fullCircle: Math.Abs(rx - ry) < 1e-9);
        }

        static void AddRect(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, XElement element)
        {
            double x = Num(element, "x"), y = Num(element, "y");
            double w = Num(element, "width"), h = Num(element, "height");
            if (w == 0 || h == 0)
                return;
            double rx = Num(element, "rx");
            double ry = Num(element, "ry");
            if (rx <= 0) rx = ry;
            if (ry <= 0) ry = rx;
            rx = Math.Min(Math.Abs(rx), Math.Abs(w) * 0.5);
            ry = Math.Min(Math.Abs(ry), Math.Abs(h) * 0.5);
            if (rx < 1e-12 || ry < 1e-12)
            {
                Vec2D a = new Vec2D(x, y), b = new Vec2D(x + w, y), c = new Vec2D(x + w, y + h), d = new Vec2D(x, y + h);
                AddLine(sketch, xform, id, flags, a.X, a.Y, b.X, b.Y);
                AddLine(sketch, xform, null, flags, b.X, b.Y, c.X, c.Y);
                AddLine(sketch, xform, null, flags, c.X, c.Y, d.X, d.Y);
                AddLine(sketch, xform, null, flags, d.X, d.Y, a.X, a.Y);
                return;
            }
            // Straight spans plus circular-or-elliptical corner arcs in SVG user space, then transformed.
            var spans = new (Vec2D A, Vec2D B)[]
            {
                (new Vec2D(x + rx, y), new Vec2D(x + w - rx, y)),
                (new Vec2D(x + w, y + ry), new Vec2D(x + w, y + h - ry)),
                (new Vec2D(x + w - rx, y + h), new Vec2D(x + rx, y + h)),
                (new Vec2D(x, y + h - ry), new Vec2D(x, y + ry))
            };
            var corners = new (Vec2D Center, Vec2D Start, Vec2D End)[]
            {
                (new Vec2D(x + w - rx, y + ry), new Vec2D(x + w - rx, y), new Vec2D(x + w, y + ry)),
                (new Vec2D(x + w - rx, y + h - ry), new Vec2D(x + w, y + h - ry), new Vec2D(x + w - rx, y + h)),
                (new Vec2D(x + rx, y + h - ry), new Vec2D(x + rx, y + h), new Vec2D(x, y + h - ry)),
                (new Vec2D(x + rx, y + ry), new Vec2D(x, y + ry), new Vec2D(x + rx, y))
            };
            Vec2D localX = new Vec2D(rx, 0);
            Vec2D localY = new Vec2D(0, ry);
            for (int i = 0; i < 4; i++)
            {
                AddLine(sketch, xform, i == 0 ? id : null, flags, spans[i].A.X, spans[i].A.Y, spans[i].B.X, spans[i].B.Y);
                double a0 = EllipseAngle(corners[i].Start - corners[i].Center, localX, localY, rx, ry);
                double a1 = EllipseAngle(corners[i].End - corners[i].Center, localX, localY, rx, ry);
                if (a1 < a0)
                    a1 += 2 * Math.PI;
                Vec2D localMid = corners[i].Center + localX * Math.Cos((a0 + a1) * 0.5) + localY * Math.Sin((a0 + a1) * 0.5);
                AddArcOrEllipse(sketch, null, flags,
                    xform.Apply(corners[i].Center), xform.ApplyVector(localX), xform.ApplyVector(localY),
                    xform.Apply(corners[i].Start), xform.Apply(localMid), xform.Apply(corners[i].End), a0, a1);
            }
        }

        static double EllipseAngle(Vec2D offset, Vec2D axisX, Vec2D axisY, double rx, double ry)
        {
            double cos = Dot(offset, axisX) / (rx * rx);
            double sin = Dot(offset, axisY) / (ry * ry);
            return Math.Atan2(sin, cos);
        }

        static void AddPoly(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, string points, bool close)
        {
            var poly = new List<Vec2D>();
            var cursor = new PathCursor(points ?? "");
            while (cursor.ReadNum(out double x) && cursor.ReadNum(out double y))
                poly.Add(xform.Apply(new Vec2D(x, y)));
            if (poly.Count < 2)
                return;
            for (int i = 0; i < poly.Count - 1; i++)
                Name(sketch.AddLine(poly[i], poly[i + 1], flags), i == 0 ? id : null, sketch);
            if (close && (poly[0] - poly[poly.Count - 1]).LengthSquared() > 1e-20)
                sketch.AddLine(poly[poly.Count - 1], poly[0], flags);
        }

        static void AddPath(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, string d, bool sampled)
        {
            if (string.IsNullOrWhiteSpace(d))
                return;
            if (sampled)
            {
                var samples = new List<Vec2D>();
                var cursor = new PathCursor(d);
                Vec2D current = new Vec2D(0, 0);
                while (cursor.SkipSeparators())
                {
                    char cmd = cursor.Peek();
                    if (char.IsLetter(cmd))
                    {
                        cursor.Next();
                        if (cmd == 'Z' || cmd == 'z')
                        {
                            if (samples.Count > 0)
                                samples.Add(samples[0]);
                            break;
                        }
                        continue;
                    }
                    if (!cursor.ReadNum(out double x) || !cursor.ReadNum(out double y))
                        break;
                    current = xform.Apply(new Vec2D(x, y));
                    samples.Add(current);
                }
                if (samples.Count >= 2)
                    Name(sketch.AddSampledCurve(samples, flags), id, sketch);
                return;
            }

            var path = new PathCursor(d);
            Vec2D pen = new Vec2D(0, 0);
            Vec2D start = pen;
            Vec2D lastCubic = pen;
            Vec2D lastQuad = pen;
            char previous = 'M';
            bool named = false;
            char command = 'M';
            while (path.SkipSeparators())
            {
                char peek = path.Peek();
                if (char.IsLetter(peek))
                {
                    command = path.Next();
                    if (command == 'Z' || command == 'z')
                    {
                        if ((pen - start).LengthSquared() > 1e-20)
                            Name(sketch.AddLine(xform.Apply(pen), xform.Apply(start), flags), TakeName(ref named, id), sketch);
                        pen = start;
                        previous = 'Z';
                        continue;
                    }
                }
                bool relative = char.IsLower(command);
                char op = char.ToUpperInvariant(command);
                if (op == 'M' || op == 'L' || op == 'T')
                {
                    if (!path.ReadPair(out double x, out double y))
                        break;
                    Vec2D next = relative ? pen + new Vec2D(x, y) : new Vec2D(x, y);
                    if (op == 'M')
                    {
                        start = next;
                        pen = next;
                        command = relative ? 'l' : 'L';
                    }
                    else if (op == 'T')
                    {
                        Vec2D control = previous == 'Q' || previous == 'T' ? pen * 2 - lastQuad : pen;
                        Vec2D a = xform.Apply(pen);
                        Vec2D c = xform.Apply(control);
                        Vec2D b = xform.Apply(next);
                        Name(sketch.AddBezier(new[] { a, c, b }, flags), TakeName(ref named, id), sketch);
                        lastQuad = control;
                        pen = next;
                    }
                    else
                    {
                        Name(sketch.AddLine(xform.Apply(pen), xform.Apply(next), flags), TakeName(ref named, id), sketch);
                        pen = next;
                    }
                    previous = op;
                    continue;
                }
                if (op == 'H' || op == 'V')
                {
                    if (!path.ReadNum(out double value))
                        break;
                    Vec2D next = op == 'H'
                        ? new Vec2D(relative ? pen.X + value : value, pen.Y)
                        : new Vec2D(pen.X, relative ? pen.Y + value : value);
                    Name(sketch.AddLine(xform.Apply(pen), xform.Apply(next), flags), TakeName(ref named, id), sketch);
                    pen = next;
                    previous = op;
                    continue;
                }
                if (op == 'C' || op == 'S' || op == 'Q')
                {
                    Vec2D c1;
                    if (op == 'S')
                        c1 = previous == 'C' || previous == 'S' ? pen * 2 - lastCubic : pen;
                    else if (!ReadTransformed(path, relative, pen, out c1))
                        break;
                    if (op != 'Q')
                    {
                        if (!ReadTransformed(path, relative, pen, out Vec2D c2) || !ReadTransformed(path, relative, pen, out Vec2D end))
                            break;
                        Name(sketch.AddBezier(new[] { xform.Apply(pen), xform.Apply(c1), xform.Apply(c2), xform.Apply(end) }, flags), TakeName(ref named, id), sketch);
                        lastCubic = c2;
                        pen = end;
                    }
                    else
                    {
                        if (!ReadTransformed(path, relative, pen, out Vec2D end))
                            break;
                        Name(sketch.AddBezier(new[] { xform.Apply(pen), xform.Apply(c1), xform.Apply(end) }, flags), TakeName(ref named, id), sketch);
                        lastQuad = c1;
                        pen = end;
                    }
                    previous = op;
                    continue;
                }
                if (op == 'A')
                {
                    if (!path.ReadNum(out double rx) || !path.ReadNum(out double ry) || !path.ReadNum(out double rotation)
                        || !path.ReadNum(out double large) || !path.ReadNum(out double sweep) || !path.ReadPair(out double x, out double y))
                        break;
                    Vec2D end = relative ? pen + new Vec2D(x, y) : new Vec2D(x, y);
                    AddSvgArc(sketch, xform, TakeName(ref named, id), flags, pen, end, rx, ry, rotation, large >= 0.5, sweep >= 0.5);
                    pen = end;
                    previous = 'A';
                    continue;
                }
                break;
            }
        }

        static bool ReadTransformed(PathCursor path, bool relative, Vec2D pen, out Vec2D point)
        {
            point = default;
            if (!path.ReadPair(out double x, out double y))
                return false;
            point = relative ? pen + new Vec2D(x, y) : new Vec2D(x, y);
            return true;
        }

        static string TakeName(ref bool named, string id)
        {
            if (named || string.IsNullOrEmpty(id))
                return null;
            named = true;
            return id;
        }

        static void AddSvgArc(PlotterSketcher sketch, SvgTransform xform, string id, CurveFlags flags, Vec2D p1, Vec2D p2, double rx, double ry, double rotationDeg, bool large, bool sweep)
        {
            if (!SvgArc.EndpointToCenter(p1, p2, ref rx, ref ry, rotationDeg * Math.PI / 180.0, large, sweep, out Vec2D center, out double theta1, out double dTheta))
            {
                if ((p2 - p1).LengthSquared() > 1e-20)
                    Name(sketch.AddLine(xform.Apply(p1), xform.Apply(p2), flags), id, sketch);
                return;
            }
            double phi = rotationDeg * Math.PI / 180.0;
            Vec2D axisX = new Vec2D(Math.Cos(phi), Math.Sin(phi)) * rx;
            Vec2D axisY = new Vec2D(-Math.Sin(phi), Math.Cos(phi)) * ry;
            Vec2D midLocal = center + axisX * Math.Cos(theta1 + dTheta * 0.5) + axisY * Math.Sin(theta1 + dTheta * 0.5);
            AddArcOrEllipse(sketch, id, flags, xform.Apply(center), xform.ApplyVector(axisX), xform.ApplyVector(axisY),
                xform.Apply(p1), xform.Apply(midLocal), xform.Apply(p2), theta1, theta1 + dTheta);
        }

        static void AddEllipseAxes(PlotterSketcher sketch, string id, CurveFlags flags, Vec2D center, Vec2D axisX, Vec2D axisY, double start, double end, bool fullCircle)
        {
            AddArcOrEllipse(sketch, id, flags, center, axisX, axisY,
                center + axisX * Math.Cos(start) + axisY * Math.Sin(start),
                center + axisX * Math.Cos((start + end) * 0.5) + axisY * Math.Sin((start + end) * 0.5),
                center + axisX * Math.Cos(end) + axisY * Math.Sin(end),
                start, end, fullCircle);
        }

        static void AddArcOrEllipse(PlotterSketcher sketch, string id, CurveFlags flags, Vec2D center, Vec2D axisX, Vec2D axisY,
            Vec2D start, Vec2D mid, Vec2D end, double startAngle, double endAngle, bool fullCircle = false)
        {
            double rx = axisX.Length();
            double ry = axisY.Length();
            if (rx < 1e-12 || ry < 1e-12)
                return;
            bool perpendicular = Math.Abs(Dot(axisX, axisY)) <= 1e-6 * rx * ry;
            bool circular = perpendicular && Math.Abs(rx - ry) <= 1e-6 * Math.Max(rx, ry);
            double sweep = endAngle - startAngle;
            bool full = fullCircle || Math.Abs(Math.Abs(sweep) - 2 * Math.PI) < 1e-6;
            if (circular && full)
            {
                Name(sketch.AddCircle(center, (rx + ry) * 0.5, flags), id, sketch);
                return;
            }
            if (circular && !full)
            {
                Name(sketch.AddArc(start, mid, end, flags), id, sketch);
                return;
            }
            if (!perpendicular)
            {
                int steps = Math.Max(8, (int)Math.Ceiling(Math.Abs(sweep) / (Math.PI / 16)));
                var points = new List<Vec2D>(steps + 1);
                for (int i = 0; i <= steps; i++)
                {
                    double t = startAngle + sweep * i / steps;
                    points.Add(center + axisX * Math.Cos(t) + axisY * Math.Sin(t));
                }
                Name(sketch.AddSampledCurve(points, flags), id, sketch);
                return;
            }
            Vec2D majorDir = axisX / rx;
            Vec2D minorDir = new Vec2D(-majorDir.Y, majorDir.X);
            double align = Dot(axisY, minorDir);
            double a0 = startAngle;
            double a1 = endAngle;
            if (align < 0)
            {
                a0 = -startAngle;
                a1 = -endAngle;
            }
            if (full)
            {
                Name(sketch.AddEllipse(center, majorDir * rx, ry, flags), id, sketch);
                return;
            }
            if (Math.Abs(a1 - a0) < 1e-9)
                return;
            if (Math.Abs(a1 - a0) > 2 * Math.PI + 1e-8)
            {
                double sign = Math.Sign(a1 - a0);
                a1 = a0 + sign * 2 * Math.PI;
            }
            Name(sketch.AddCurve(new Ellipse2D(center, majorDir * rx, ry, a0, a1, flags)), id, sketch);
        }

        static void Name(Curve2D curve, string id, PlotterSketcher sketch)
        {
            if (curve == null || string.IsNullOrWhiteSpace(id))
                return;
            string name = id.Trim();
            var used = new HashSet<string>(StringComparer.Ordinal);
            foreach (Curve2D existing in sketch.GetAllCurves())
            {
                if (existing != null && existing != curve && !string.IsNullOrEmpty(existing.Name))
                    used.Add(existing.Name);
            }
            string unique = name;
            int n = 2;
            while (!used.Add(unique))
                unique = name + "_" + n++;
            curve.Name = unique;
        }

        static bool IsAnnotation(XElement element)
        {
            string marker = Attr(element, "data-camber");
            return marker == "annotations" || marker == "annotation" || Attr(element, "id") == "camber-annotations";
        }

        static bool IsDefinition(XElement element)
        {
            string name = element.Name.LocalName;
            return name == "defs" || name == "symbol" || name == "clipPath" || name == "mask" || name == "marker"
                || name == "style" || name == "metadata" || name == "title" || name == "desc" || name == "text";
        }

        static bool IsHidden(XElement element) => Attr(element, "display") == "none";

        static bool IsConstruction(XElement element)
        {
            if (Attr(element, "data-construction") == "1")
                return true;
            string cls = Attr(element, "class");
            return cls.Split(' ', StringSplitOptions.RemoveEmptyEntries).Contains("construction");
        }

        static string Attr(XElement element, string name) => element.Attribute(name)?.Value ?? "";

        static double Num(XElement element, string name)
        {
            double.TryParse(Attr(element, name), NumberStyles.Float, CultureInfo.InvariantCulture, out double value);
            return value;
        }

        static double Dot(Vec2D a, Vec2D b) => a.X * b.X + a.Y * b.Y;
        static double Cross(Vec2D a, Vec2D b) => a.X * b.Y - a.Y * b.X;
    }

    struct SvgTransform
    {
        public double A, B, C, D, E, F;

        public static SvgTransform Identity => new SvgTransform { A = 1, D = 1 };

        public static SvgTransform Scale(double x, double y) => new SvgTransform { A = x, D = y };

        public SvgTransform Multiply(SvgTransform other)
        {
            return new SvgTransform
            {
                A = A * other.A + C * other.B,
                B = B * other.A + D * other.B,
                C = A * other.C + C * other.D,
                D = B * other.C + D * other.D,
                E = A * other.E + C * other.F + E,
                F = B * other.E + D * other.F + F
            };
        }

        public Vec2D Apply(Vec2D p) => new Vec2D(A * p.X + C * p.Y + E, B * p.X + D * p.Y + F);

        public Vec2D ApplyVector(Vec2D p) => new Vec2D(A * p.X + C * p.Y, B * p.X + D * p.Y);

        public static SvgTransform Parse(string text)
        {
            SvgTransform result = Identity;
            if (string.IsNullOrWhiteSpace(text))
                return result;
            int i = 0;
            while (i < text.Length)
            {
                while (i < text.Length && (char.IsWhiteSpace(text[i]) || text[i] == ','))
                    i++;
                if (i >= text.Length)
                    break;
                int start = i;
                while (i < text.Length && char.IsLetter(text[i]))
                    i++;
                string name = text.Substring(start, i - start);
                while (i < text.Length && char.IsWhiteSpace(text[i]))
                    i++;
                if (i >= text.Length || text[i] != '(')
                    break;
                i++;
                var numbers = new List<double>();
                var cursor = new PathCursor(text.Substring(i));
                while (cursor.SkipSeparators() && cursor.Peek() != ')')
                {
                    if (!cursor.ReadNum(out double value))
                        break;
                    numbers.Add(value);
                }
                i += cursor.Index;
                if (i < text.Length && text[i] == ')')
                    i++;
                result = result.Multiply(FromFunction(name, numbers));
            }
            return result;
        }

        static SvgTransform FromFunction(string name, List<double> n)
        {
            double At(int index, double fallback = 0) => index < n.Count ? n[index] : fallback;
            if (name == "matrix" && n.Count >= 6)
                return new SvgTransform { A = n[0], B = n[1], C = n[2], D = n[3], E = n[4], F = n[5] };
            if (name == "translate")
                return new SvgTransform { A = 1, D = 1, E = At(0), F = At(1) };
            if (name == "scale")
            {
                double x = At(0, 1);
                return new SvgTransform { A = x, D = n.Count > 1 ? n[1] : x };
            }
            if (name == "rotate")
            {
                double rad = At(0) * Math.PI / 180.0;
                double cos = Math.Cos(rad), sin = Math.Sin(rad);
                var rotation = new SvgTransform { A = cos, B = sin, C = -sin, D = cos };
                if (n.Count >= 3)
                {
                    var to = new SvgTransform { A = 1, D = 1, E = n[1], F = n[2] };
                    var back = new SvgTransform { A = 1, D = 1, E = -n[1], F = -n[2] };
                    return to.Multiply(rotation).Multiply(back);
                }
                return rotation;
            }
            if (name == "skewX")
            {
                double t = Math.Tan(At(0) * Math.PI / 180.0);
                return new SvgTransform { A = 1, C = t, D = 1 };
            }
            if (name == "skewY")
            {
                double t = Math.Tan(At(0) * Math.PI / 180.0);
                return new SvgTransform { A = 1, B = t, D = 1 };
            }
            return Identity;
        }
    }

    static class SvgArc
    {
        public static bool EndpointToCenter(Vec2D p1, Vec2D p2, ref double rx, ref double ry, double phi, bool large, bool sweep,
            out Vec2D center, out double theta1, out double dTheta)
        {
            center = default;
            theta1 = 0;
            dTheta = 0;
            if (rx < 0) rx = -rx;
            if (ry < 0) ry = -ry;
            if (rx < 1e-14 || ry < 1e-14)
                return false;
            double dx = (p1.X - p2.X) * 0.5;
            double dy = (p1.Y - p2.Y) * 0.5;
            double cos = Math.Cos(phi), sin = Math.Sin(phi);
            double x1 = cos * dx + sin * dy;
            double y1 = -sin * dx + cos * dy;
            if (x1 * x1 + y1 * y1 < 1e-20)
                return false;
            double lam = (x1 * x1) / (rx * rx) + (y1 * y1) / (ry * ry);
            if (lam > 1)
            {
                double s = Math.Sqrt(lam);
                rx *= s;
                ry *= s;
            }
            double sign = large == sweep ? -1 : 1;
            double num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
            double den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
            if (den <= 0)
                return false;
            double root = Math.Sqrt(Math.Max(0, num / den));
            double cx = sign * root * rx * y1 / ry;
            double cy = sign * -root * ry * x1 / rx;
            double cxWorld = cos * cx - sin * cy + (p1.X + p2.X) * 0.5;
            double cyWorld = sin * cx + cos * cy + (p1.Y + p2.Y) * 0.5;
            center = new Vec2D(cxWorld, cyWorld);
            theta1 = Angle(1, 0, (x1 - cx) / rx, (y1 - cy) / ry);
            dTheta = Angle((x1 - cx) / rx, (y1 - cy) / ry, (-x1 - cx) / rx, (-y1 - cy) / ry);
            if (!sweep && dTheta > 0)
                dTheta -= 2 * Math.PI;
            else if (sweep && dTheta < 0)
                dTheta += 2 * Math.PI;
            return true;
        }

        static double Angle(double ux, double uy, double vx, double vy)
        {
            double dot = ux * vx + uy * vy;
            double len = Math.Sqrt(ux * ux + uy * uy) * Math.Sqrt(vx * vx + vy * vy);
            double cos = len > 0 ? Math.Clamp(dot / len, -1, 1) : 1;
            double sign = ux * vy - uy * vx < 0 ? -1 : 1;
            return sign * Math.Acos(cos);
        }
    }

    sealed class PathCursor
    {
        readonly string _text;
        int _i;
        public int Index => _i;
        public PathCursor(string text) { _text = text ?? ""; }

        public bool SkipSeparators()
        {
            while (_i < _text.Length && (char.IsWhiteSpace(_text[_i]) || _text[_i] == ','))
                _i++;
            return _i < _text.Length;
        }

        public char Peek() => _i < _text.Length ? _text[_i] : '\0';
        public char Next() => _text[_i++];

        public bool ReadPair(out double x, out double y)
        {
            y = 0;
            return ReadNum(out x) && ReadNum(out y);
        }

        public bool ReadNum(out double value)
        {
            value = 0;
            SkipSeparators();
            if (_i >= _text.Length)
                return false;
            char c = _text[_i];
            if (!(c == '+' || c == '-' || c == '.' || char.IsDigit(c)))
                return false;
            int start = _i;
            if (c == '+' || c == '-')
                _i++;
            bool digit = false;
            while (_i < _text.Length && char.IsDigit(_text[_i]))
            {
                digit = true;
                _i++;
            }
            if (_i < _text.Length && _text[_i] == '.')
            {
                _i++;
                while (_i < _text.Length && char.IsDigit(_text[_i]))
                {
                    digit = true;
                    _i++;
                }
            }
            if (digit && _i < _text.Length && (_text[_i] == 'e' || _text[_i] == 'E'))
            {
                int e = _i++;
                if (_i < _text.Length && (_text[_i] == '+' || _text[_i] == '-'))
                    _i++;
                int exp = _i;
                while (_i < _text.Length && char.IsDigit(_text[_i]))
                    _i++;
                if (_i == exp)
                    _i = e;
            }
            if (!digit)
            {
                _i = start;
                return false;
            }
            return double.TryParse(_text.Substring(start, _i - start), NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        }
    }
}
