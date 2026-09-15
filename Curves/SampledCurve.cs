using System.Collections.Generic;
using GeoCore;
using System;

namespace Curves
{
    public class SampledCurve : Curve2D
    {
        private List<Vec2D> _points;
        private List<Vec2D> _normals;
        //private List<double> _uValues;

        public SampledCurve()
        {
            _points = new List<Vec2D>();
            _normals = new List<Vec2D>();
        }

        public SampledCurve(int numPoints)
        {
            _points = new List<Vec2D>(numPoints);
            _normals = new List<Vec2D>(numPoints);
        }

        public SampledCurve(List<Vec2D> points, List<Vec2D> normals)
        {
            _points = points ?? new List<Vec2D>();
            _normals = normals ?? new List<Vec2D>();
        }

        public IReadOnlyList<Vec2D> Points { get { return _points; } }

        public void Reset(List<Vec2D> points, List<Vec2D> normals)
        {
            _points = points ?? new List<Vec2D>();
            _normals = normals ?? new List<Vec2D>();
        }

        public override double Length()
        {
            double length = 0;
            int l = _points.Count;
            for (int i = 1; i < l; ++i)
                length += (_points[i] - _points[i - 1]).Length();
            return length;
        }

        public void Add(Vec2D point, Vec2D normal)
        {
            _points.Add(point);
            _normals.Add(normal);
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            if (uniform <= 0) return new CurveVertex2D(_points[0], _normals[0], 0);
            if (uniform >= 1) return new CurveVertex2D(_points[_points.Count - 1], _normals[_normals.Count - 1], 1);

            double scaled = uniform * (_points.Count - 1);
            int i = (int)scaled;
            if (i >= _points.Count - 1)
                i = _points.Count - 2;
            double d = scaled - i;
            Vec2D p = (1 - d) * _points[i] + d * _points[i + 1];
            Vec2D n = (1 - d) * _normals[i] + d * _normals[i + 1];

            return new CurveVertex2D(p, n, uniform);
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            //This ignores maxDeviation...
            List<CurveVertex2D> result = new List<CurveVertex2D>(_points.Count);
            double scaling = 1.0 / (_points.Count - 1);
            for (int i = 0; i < _points.Count; ++i)
            {
                result.Add(new CurveVertex2D(_points[i], _normals[i], i * scaling));
            }
            return result;
        }
        public override List<CurveVertex2D> Tessellate(int numPoints)
        {
            //This ignores maxDeviation...
            List<CurveVertex2D> result = new List<CurveVertex2D>(_points.Count);
            double scaling = 1.0 / (_points.Count - 1);
            for (int i = 0; i < _points.Count; ++i)
            {
                result.Add(new CurveVertex2D(_points[i], _normals[i], i * scaling));
            }
            return result;
        }


        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>(_points);
        }

        public override Curve2D GetCopy()
        {
            SampledCurve sc = new SampledCurve();
            sc._points.AddRange(_points);
            sc._normals.AddRange(_normals);
            sc.Name = Name;
            sc.Flags = Flags;
            return sc;
        }

        public SampledCurve GetRotated(double angle) { return GetRotated(Vec2DOps.Zero, angle); }
        public SampledCurve GetRotated(Vec2D centerOfRotation, double angle)
        {
            List<Vec2D> points = new List<Vec2D>(_points.Count);
            List<Vec2D> normals = new List<Vec2D>(_normals.Count);

            double sin = Math.Sin(angle);
            double cos = Math.Cos(angle);
            for (int i = 0; i < _points.Count; ++i)
            {
                Vec2D trans = _points[i] - centerOfRotation;
                Vec2D rot = new Vec2D(cos * trans.X - sin * trans.Y, sin * trans.X + cos * trans.Y);
                points.Add(rot + centerOfRotation);

                Vec2D n = _normals[i];
                normals.Add(new Vec2D(cos * n.X - sin * n.Y, sin * n.X + cos * n.Y));
            }

            return new SampledCurve(points, normals);
        }

        public override Curve2D Reverse()
        {
            SampledCurve sc = new SampledCurve();
            sc._points.AddRange(_points);
            sc._normals.AddRange(_normals);
            sc._points.Reverse();
            sc._normals.Reverse();
            return sc;
        }
    }
}