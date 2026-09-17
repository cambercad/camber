using System;
using System.Runtime.CompilerServices;

namespace GeoCore
{
    public struct Mat4D
    {
        public double M11; public double M12; public double M13; public double M14;
        public double M21; public double M22; public double M23; public double M24;
        public double M31; public double M32; public double M33; public double M34;
        public double M41; public double M42; public double M43; public double M44;

        public Mat4D() { }

        public Mat4D(double m11, double m12, double m13, double m14, double m21, double m22, double m23, double m24, double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44)
        {
            this.M11 = m11;
            this.M12 = m12;
            this.M13 = m13;
            this.M14 = m14;
            this.M21 = m21;
            this.M22 = m22;
            this.M23 = m23;
            this.M24 = m24;
            this.M31 = m31;
            this.M32 = m32;
            this.M33 = m33;
            this.M34 = m34;
            this.M41 = m41;
            this.M42 = m42;
            this.M43 = m43;
            this.M44 = m44;
        }

        public static Mat4D operator *(in Mat4D lhs, in Mat4D rhs)
        {
            Mat4D result;
            result.M11 = lhs.M11 * rhs.M11 + lhs.M12 * rhs.M21 + lhs.M13 * rhs.M31 + lhs.M14 * rhs.M41;
            result.M12 = lhs.M11 * rhs.M12 + lhs.M12 * rhs.M22 + lhs.M13 * rhs.M32 + lhs.M14 * rhs.M42;
            result.M13 = lhs.M11 * rhs.M13 + lhs.M12 * rhs.M23 + lhs.M13 * rhs.M33 + lhs.M14 * rhs.M43;
            result.M14 = lhs.M11 * rhs.M14 + lhs.M12 * rhs.M24 + lhs.M13 * rhs.M34 + lhs.M14 * rhs.M44;

            result.M21 = lhs.M21 * rhs.M11 + lhs.M22 * rhs.M21 + lhs.M23 * rhs.M31 + lhs.M24 * rhs.M41;
            result.M22 = lhs.M21 * rhs.M12 + lhs.M22 * rhs.M22 + lhs.M23 * rhs.M32 + lhs.M24 * rhs.M42;
            result.M23 = lhs.M21 * rhs.M13 + lhs.M22 * rhs.M23 + lhs.M23 * rhs.M33 + lhs.M24 * rhs.M43;
            result.M24 = lhs.M21 * rhs.M14 + lhs.M22 * rhs.M24 + lhs.M23 * rhs.M34 + lhs.M24 * rhs.M44;

            result.M31 = lhs.M31 * rhs.M11 + lhs.M32 * rhs.M21 + lhs.M33 * rhs.M31 + lhs.M34 * rhs.M41;
            result.M32 = lhs.M31 * rhs.M12 + lhs.M32 * rhs.M22 + lhs.M33 * rhs.M32 + lhs.M34 * rhs.M42;
            result.M33 = lhs.M31 * rhs.M13 + lhs.M32 * rhs.M23 + lhs.M33 * rhs.M33 + lhs.M34 * rhs.M43;
            result.M34 = lhs.M31 * rhs.M14 + lhs.M32 * rhs.M24 + lhs.M33 * rhs.M34 + lhs.M34 * rhs.M44;

            result.M41 = lhs.M41 * rhs.M11 + lhs.M42 * rhs.M21 + lhs.M43 * rhs.M31 + lhs.M44 * rhs.M41;
            result.M42 = lhs.M41 * rhs.M12 + lhs.M42 * rhs.M22 + lhs.M43 * rhs.M32 + lhs.M44 * rhs.M42;
            result.M43 = lhs.M41 * rhs.M13 + lhs.M42 * rhs.M23 + lhs.M43 * rhs.M33 + lhs.M44 * rhs.M43;
            result.M44 = lhs.M41 * rhs.M14 + lhs.M42 * rhs.M24 + lhs.M43 * rhs.M34 + lhs.M44 * rhs.M44;
            return result;
        }

        public static Vec4D operator *(in Mat4D lhs, in Vec4D rhs)
        {
            Vec4D result;
            result.X = lhs.M11 * rhs.X + lhs.M12 * rhs.Y + lhs.M13 * rhs.Z + lhs.M14 * rhs.W;
            result.Y = lhs.M21 * rhs.X + lhs.M22 * rhs.Y + lhs.M23 * rhs.Z + lhs.M24 * rhs.W;
            result.Z = lhs.M31 * rhs.X + lhs.M32 * rhs.Y + lhs.M33 * rhs.Z + lhs.M34 * rhs.W;
            result.W = lhs.M41 * rhs.X + lhs.M42 * rhs.Y + lhs.M43 * rhs.Z + lhs.M44 * rhs.W;
            return result;
        }

        public Vec3D TransformDirection(in Vec3D v)
        {
            return new Vec3D(
               M11 * v.X + M12 * v.Y + M13 * v.Z,
               M21 * v.X + M22 * v.Y + M23 * v.Z,
               M31 * v.X + M32 * v.Y + M33 * v.Z);
        }

        public Vec3D TransformPoint(in Vec3D v)
        {
            return new Vec3D(
               M11 * v.X + M12 * v.Y + M13 * v.Z + M14,
               M21 * v.X + M22 * v.Y + M23 * v.Z + M24,
               M31 * v.X + M32 * v.Y + M33 * v.Z + M34);
        }
    }

    public static class Mat4DOps
    {
        public static Mat4D Identity()
        {
            Mat4D r = new Mat4D();

            r.M11 = 1.0;
            r.M22 = 1.0;
            r.M33 = 1.0;
            r.M44 = 1.0;

            return r;
        }

        public static bool Inverse(in Mat4D m, out Mat4D r)
        {
            var det = (m.M11 * m.M22 * m.M33 * m.M44 - m.M11 * m.M22 * m.M34 * m.M43 - m.M11 * m.M23 * m.M32 * m.M44 + m.M11 * m.M23 * m.M34 * m.M42 + m.M11 * m.M24 * m.M32 * m.M43 - m.M11 * m.M24 * m.M33 * m.M42 - m.M12 * m.M21 * m.M33 * m.M44 + m.M12 * m.M21 * m.M34 * m.M43 + m.M12 * m.M23 * m.M31 * m.M44 - m.M12 * m.M23 * m.M34 * m.M41 - m.M12 * m.M24 * m.M31 * m.M43 + m.M12 * m.M24 * m.M33 * m.M41 + m.M13 * (m.M21 * m.M32 * m.M44 - m.M21 * m.M34 * m.M42 - m.M22 * m.M31 * m.M44 + m.M22 * m.M34 * m.M41 + m.M24 * m.M31 * m.M42 - m.M24 * m.M32 * m.M41) + m.M14 * (-m.M21 * m.M32 * m.M43 + m.M21 * m.M33 * m.M42 + m.M22 * m.M31 * m.M43 - m.M22 * m.M33 * m.M41 - m.M23 * m.M31 * m.M42 + m.M23 * m.M32 * m.M41));

            if (det == 0.0)
            {
                r = new Mat4D();
                return false;
            }

            det = 1.0f / det;


            r.M11 = det * (-m.M24 * m.M33 * m.M42 + m.M23 * m.M34 * m.M42 + m.M24 * m.M32 * m.M43 - m.M22 * m.M34 * m.M43 - m.M23 * m.M32 * m.M44 + m.M22 * m.M33 * m.M44);
            r.M12 = det * (m.M14 * m.M33 * m.M42 - m.M13 * m.M34 * m.M42 - m.M14 * m.M32 * m.M43 + m.M12 * m.M34 * m.M43 + m.M13 * m.M32 * m.M44 - m.M12 * m.M33 * m.M44);
            r.M13 = det * (-m.M14 * m.M23 * m.M42 + m.M13 * m.M24 * m.M42 + m.M14 * m.M22 * m.M43 - m.M12 * m.M24 * m.M43 - m.M13 * m.M22 * m.M44 + m.M12 * m.M23 * m.M44);
            r.M14 = det * (m.M14 * m.M23 * m.M32 - m.M13 * m.M24 * m.M32 - m.M14 * m.M22 * m.M33 + m.M12 * m.M24 * m.M33 + m.M13 * m.M22 * m.M34 - m.M12 * m.M23 * m.M34);

            r.M21 = det * (m.M24 * m.M33 * m.M41 - m.M23 * m.M34 * m.M41 - m.M24 * m.M31 * m.M43 + m.M21 * m.M34 * m.M43 + m.M23 * m.M31 * m.M44 - m.M21 * m.M33 * m.M44);
            r.M22 = det * (-m.M14 * m.M33 * m.M41 + m.M13 * m.M34 * m.M41 + m.M14 * m.M31 * m.M43 - m.M11 * m.M34 * m.M43 - m.M13 * m.M31 * m.M44 + m.M11 * m.M33 * m.M44);
            r.M23 = det * (m.M14 * m.M23 * m.M41 - m.M13 * m.M24 * m.M41 - m.M14 * m.M21 * m.M43 + m.M11 * m.M24 * m.M43 + m.M13 * m.M21 * m.M44 - m.M11 * m.M23 * m.M44);
            r.M24 = det * (-m.M14 * m.M23 * m.M31 + m.M13 * m.M24 * m.M31 + m.M14 * m.M21 * m.M33 - m.M11 * m.M24 * m.M33 - m.M13 * m.M21 * m.M34 + m.M11 * m.M23 * m.M34);

            r.M31 = det * (-m.M24 * m.M32 * m.M41 + m.M22 * m.M34 * m.M41 + m.M24 * m.M31 * m.M42 - m.M21 * m.M34 * m.M42 - m.M22 * m.M31 * m.M44 + m.M21 * m.M32 * m.M44);
            r.M32 = det * (m.M14 * m.M32 * m.M41 - m.M12 * m.M34 * m.M41 - m.M14 * m.M31 * m.M42 + m.M11 * m.M34 * m.M42 + m.M12 * m.M31 * m.M44 - m.M11 * m.M32 * m.M44);
            r.M33 = det * (-m.M14 * m.M22 * m.M41 + m.M12 * m.M24 * m.M41 + m.M14 * m.M21 * m.M42 - m.M11 * m.M24 * m.M42 - m.M12 * m.M21 * m.M44 + m.M11 * m.M22 * m.M44);
            r.M34 = det * (m.M14 * m.M22 * m.M31 - m.M12 * m.M24 * m.M31 - m.M14 * m.M21 * m.M32 + m.M11 * m.M24 * m.M32 + m.M12 * m.M21 * m.M34 - m.M11 * m.M22 * m.M34);

            r.M41 = det * (m.M23 * m.M32 * m.M41 - m.M22 * m.M33 * m.M41 - m.M23 * m.M31 * m.M42 + m.M21 * m.M33 * m.M42 + m.M22 * m.M31 * m.M43 - m.M21 * m.M32 * m.M43);
            r.M42 = det * (-m.M13 * m.M32 * m.M41 + m.M12 * m.M33 * m.M41 + m.M13 * m.M31 * m.M42 - m.M11 * m.M33 * m.M42 - m.M12 * m.M31 * m.M43 + m.M11 * m.M32 * m.M43);
            r.M43 = det * (m.M13 * m.M22 * m.M41 - m.M12 * m.M23 * m.M41 - m.M13 * m.M21 * m.M42 + m.M11 * m.M23 * m.M42 + m.M12 * m.M21 * m.M43 - m.M11 * m.M22 * m.M43);
            r.M44 = det * (-m.M13 * m.M22 * m.M31 + m.M12 * m.M23 * m.M31 + m.M13 * m.M21 * m.M32 - m.M11 * m.M23 * m.M32 - m.M12 * m.M21 * m.M33 + m.M11 * m.M22 * m.M33);

            return true;
        }
    }

    
    public struct Vec2D
    {
        public double X;
        public double Y;

        public Vec2D(double x, double y)
        {
            this.X = x;
            this.Y = y;
        }
        public Vec2D(double all)
        {
            this.X = all;
            this.Y = all;
        }

        public static Vec2D operator +(Vec2D a, Vec2D b)
        {
            Vec2D result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            return result;
        }
        public static Vec2D operator -(Vec2D a, Vec2D b)
        {
            Vec2D result;
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            return result;
        }
        public static Vec2D operator -(Vec2D v)
        {
            Vec2D result;
            result.X = -v.X;
            result.Y = -v.Y;
            return result;
        }
        public static Vec2D operator *(Vec2D a, double s)
        {
            Vec2D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            return result;
        }

        public static Vec2D operator *(double s, Vec2D a)
        {
            Vec2D result;
            result.X = s * a.X;
            result.Y = s * a.Y;
            return result;
        }
        public static Vec2D operator /(Vec2D a, double s)
        {
            s = 1.0 / s;
            Vec2D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            return result;
        }

        public static bool operator ==(Vec2D a, Vec2D b)
        {
            return a.X == b.X && a.Y == b.Y;
        }

        public static bool operator !=(Vec2D a, Vec2D b)
        {
            return !(a == b);
        }

        public override string ToString()
        {
            return "(" + X.ToString("F6") + "," + Y.ToString("F6") + ")";
        }
    }

    public static class Vec2DOps
    {
        public static readonly Vec2D Zero = new Vec2D(0, 0);

        public static double DistanceSquared(Vec2D a, Vec2D b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return dx * dx + dy * dy;
        }

        public static double Dot(this Vec2D a, Vec2D b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static double Cross(this Vec2D a, Vec2D b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Computes the 2D cross product of vectors (b-a) and (c-a).
        /// Returns positive if c is to the left of the line from a to b,
        /// negative if c is to the right, and zero if they are collinear.
        /// </summary>
        public static double Cross(Vec2D a, Vec2D b, Vec2D c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }

        public static double Length(this Vec2D v)
        {
            return Math.Sqrt(v.X * v.X + v.Y * v.Y);
        }

        public static double LengthSquared(this Vec2D v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        public static Vec2D Normalized(this Vec2D v)
        {
            double len = v.Length();
            if (len == 0.0) 
                return new Vec2D(0, 0);
            return v / len;
        }

        public static void Normalize(this ref Vec2D v)
        {
            double len = v.Length();
            if (len == 0.0) 
                return;
            len = 1.0 / len;
            v.X = v.X * len;
            v.Y = v.Y * len;
        }

        public static Vec2D Rotated(Vec2D vector, double angle)
        {
            double sin = Math.Sin(angle);
            double cos = Math.Cos(angle);
            return new Vec2D(cos * vector.X - sin * vector.Y, sin * vector.X + cos * vector.Y);
        }

        public static Vec2D PerpendicularRight(this Vec2D v)
        {
            return new Vec2D(v.Y, -v.X);
        }
    }

    public static class Vec3DOps
    {
        public static double Angle(Vec3D l, Vec3D r)
        {
            double d = (l.X * r.X + l.Y * r.Y + l.Z * r.Z) / Math.Sqrt((l.X * l.X + l.Y * l.Y + l.Z * l.Z) * (r.X * r.X + r.Y * r.Y + r.Z * r.Z));
            /*if (double.IsNaN(d))
            {
            }*/
            if (d <= -1) return Math.PI;
            if (d >= 1) return 0;
            return Math.Acos(d);
        }


        public static double Angle(Vec3D fromDir, Vec3D toDir, Vec3D rotAxis)
        {
            //fromDir = GeometricAlgorithms.ProjectPointOntoPlane(fromDir, rotAxis, 0);
            double s = 1.0 / (rotAxis.X * rotAxis.X + rotAxis.Y * rotAxis.Y + rotAxis.Z * rotAxis.Z);
            double t = (rotAxis.X * fromDir.X + rotAxis.Y * fromDir.Y + rotAxis.Z * fromDir.Z) * s;  //(Dot(n, p) + d) / Dot(n, n);   
            fromDir.X = fromDir.X - t * rotAxis.X;
            fromDir.Y = fromDir.Y - t * rotAxis.Y;
            fromDir.Z = fromDir.Z - t * rotAxis.Z;

            //toDir = GeometricAlgorithms.ProjectPointOntoPlane(toDir, rotAxis, 0);
            t = (rotAxis.X * toDir.X + rotAxis.Y * toDir.Y + rotAxis.Z * toDir.Z) * s;  //(Dot(n, p) + d) / Dot(n, n);   
            toDir.X = toDir.X - t * rotAxis.X;
            toDir.Y = toDir.Y - t * rotAxis.Y;
            toDir.Z = toDir.Z - t * rotAxis.Z;

            double angle = Angle(fromDir, toDir);
            double sign = Math.Sign(Vec3DOps.Dot(Vec3DOps.Cross(fromDir, toDir), rotAxis));
            if (sign == 0)
                return angle; //Avoids error when angle is exactly 180°
            return sign * angle;
        }

        public static double DistanceSquared(Vec3D a, Vec3D b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            double dz = a.Z - b.Z;
            return dx * dx + dy * dy + dz * dz;
        }

        public static double Dot(this Vec3D a, Vec3D b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vec3D Cross(this Vec3D a, Vec3D b)
        {
            return new Vec3D(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );
        }

        public static double Length(this Vec3D v)
        {
            return Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        public static double LengthSquared(this Vec3D v)
        {
            return v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }

        public static Vec3D Normalized(this Vec3D v)
        {
            double len = v.Length();
            if (len == 0.0)
                return new Vec3D(0, 0, 0);
            return v / len;
        }

        public static void Normalize(this ref Vec3D v)
        {
            double len = v.Length();
            if (len == 0.0)
                return;
            len = 1.0 / len;
            v.X = v.X * len;
            v.Y = v.Y * len;
            v.Z = v.Z * len;
        }

        public static Vec3D GetOrthoNormal(Vec3D v)
        {
            // Normalize input vector to ensure robustness
            v = v.Normalized();
            
            // Handle zero vector case
            if (v.LengthSquared() < 1e-12)
                return new Vec3D(1, 0, 0);

            // Choose a vector that is not parallel to v
            // We pick the axis that has the smallest component in v to avoid near-parallel cases
            Vec3D candidate;
            if (Math.Abs(v.X) < Math.Abs(v.Y) && Math.Abs(v.X) < Math.Abs(v.Z))
                candidate = new Vec3D(1, 0, 0); // X axis
            else if (Math.Abs(v.Y) < Math.Abs(v.Z))
                candidate = new Vec3D(0, 1, 0); // Y axis
            else
                candidate = new Vec3D(0, 0, 1); // Z axis

            // Compute orthogonal vector using cross product
            Vec3D orthogonal = Cross(v, candidate);
            
            // Normalize the result
            return orthogonal.Normalized();
        }
    }

    public struct Int2 : IEquatable<Int2>
    {
        public int X;
        public int Y;
        public Int2(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }

        public static bool operator ==(Int2 a, Int2 b)
        {
            return a.X == b.X && a.Y == b.Y;
        }

        public static bool operator !=(Int2 a, Int2 b)
        {
            return !(a == b);
        }

        public bool Equals(Int2 other) => X == other.X && Y == other.Y;

        public override bool Equals(object obj) => obj is Int2 other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(X, Y);

        public override string ToString()
        {
            return "(" + X.ToString() + "," + Y.ToString() + ")";
        }
    }

    public struct Int3
    {
        public int X;
        public int Y;
        public int Z;
        public Int3(int x, int y, int z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }
        public Int3(int all)
        {
            this.X = all;
            this.Y = all;
            this.Z = all;
        }
    }


    public struct Box3F
    {
        public Vec3F Min;
        public Vec3F Max;


        public static readonly Box3F Empty = new Box3F() { Min = new Vec3F(float.MaxValue, float.MaxValue, float.MaxValue), Max = new Vec3F(float.MinValue, float.MinValue, float.MinValue) };

        public Box3F(Vec3F a, float enlargement = 1e-6f)
        {
            Min = a;
            Max = a;

            Enlarge(enlargement);
        }

        public Box3F(Vec3F a, Vec3F b, float enlargement = 1e-6f)
        {
            Min = a;
            Max = a;

            Extend(b);

            Enlarge(enlargement);
        }

        public Box3F(Vec3F a, Vec3F b, Vec3F c, float enlargement = 1e-6f)
        {
            Min = a;
            Max = a;

            Extend(b);
            Extend(c);

            Enlarge(enlargement);
        }

        public Box3F(Vec3F a, Vec3F b, Vec3F c, Vec3F d, float enlargement = 1e-6f)
        {
            Min = a;
            Max = a;

            Extend(b);
            Extend(c);
            Extend(d);

            Enlarge(enlargement);
        }

        public void Enlarge(float enlargement)
        {
            Min.X -= enlargement; Min.Y -= enlargement; Min.Z -= enlargement;
            Max.X += enlargement; Max.Y += enlargement; Max.Z += enlargement;
        }

        public void Extend(Vec3F p)
        {
            if (p.X < Min.X) Min.X = p.X; if (p.Y < Min.Y) Min.Y = p.Y; if (p.Z < Min.Z) Min.Z = p.Z;
            if (p.X > Max.X) Max.X = p.X; if (p.Y > Max.Y) Max.Y = p.Y; if (p.Z > Max.Z) Max.Z = p.Z;
        }

        public bool IsEmpty()
        {
            return Min.X >= Max.X || Min.Y >= Max.Y || Min.Z >= Max.Z;
        }

        public void Extend(Box3F b)
        {
            Extend(b.Min);
            Extend(b.Max);
        }

        public static Box3F IntersectionOfBoxes(Box3F a, Box3F b)
        {
            return new Box3F(new Vec3F(Math.Max(a.Min.X, b.Min.X), Math.Max(a.Min.Y, b.Min.Y), Math.Max(a.Min.Z, b.Min.Z)),
                new Vec3F(Math.Min(a.Max.X, b.Max.X), Math.Min(a.Max.Y, b.Max.Y), Math.Min(a.Max.Z, b.Max.Z)));
        }

        public static bool Overlap(Box3F a, Box3F b)
        {
            return !(a.Min.X > b.Max.X || b.Min.X > a.Max.X ||
                a.Min.Y > b.Max.Y || b.Min.Y > a.Max.Y ||
                a.Min.Z > b.Max.Z || b.Min.Z > a.Max.Z);
        }

        public bool ContainsPoint(Vec3F p)
        {
            return p.X >= Min.X && p.X <= Max.X &&
                p.Y >= Min.Y && p.Y <= Max.Y &&
                p.Z >= Min.Z && p.Z <= Max.Z;
        }

        public float DistanceSquaredToPoint(Vec3F p)
        {
            float sqDist = 0.0f;

            if (p.X < Min.X) sqDist += (Min.X - p.X) * (Min.X - p.X);
            if (p.X > Max.X) sqDist += (p.X - Max.X) * (p.X - Max.X);

            if (p.Y < Min.Y) sqDist += (Min.Y - p.Y) * (Min.Y - p.Y);
            if (p.Y > Max.Y) sqDist += (p.Y - Max.Y) * (p.Y - Max.Y);

            if (p.Z < Min.Z) sqDist += (Min.Z - p.Z) * (Min.Z - p.Z);
            if (p.Z > Max.Z) sqDist += (p.Z - Max.Z) * (p.Z - Max.Z);

            return sqDist;
        }

        public bool OverlapOrTouch(Box3F bounds)
        {
            return Overlap(this, bounds);
        }

        public override string ToString()
        {
            return Min.ToString() + "   " + Max.ToString();
        }
    }

    public struct Box3I
    {
        public Int3 Min;
        public Int3 Max;


        public Box3I(Int3 min, Int3 max)
        {
            Min = min;
            Max = max;
        }

        public Box3I(Int3 a, Int3 b, Int3 c)
        {
            Min = a;
            Max = a;

            Extend(b);
            Extend(c);
        }

        public static Box3I Empty()
        {
            return new Box3I(new Int3(int.MaxValue), new Int3(int.MinValue));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Extend(Int3 p)
        {
            if (p.X < Min.X) Min.X = p.X; if (p.Y < Min.Y) Min.Y = p.Y; if (p.Z < Min.Z) Min.Z = p.Z;
            if (p.X > Max.X) Max.X = p.X; if (p.Y > Max.Y) Max.Y = p.Y; if (p.Z > Max.Z) Max.Z = p.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Extend(Box3I p)
        {
            Extend(p.Min);
            Extend(p.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool OverlapOrTouch(Box3I bounds)
        {
            return !(Max.X < bounds.Min.X || Min.X > bounds.Max.X || Max.Y < bounds.Min.Y || Min.Y > bounds.Max.Y || Max.Z < bounds.Min.Z || Min.Z > bounds.Max.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool OverlapOrTouch(Int3 bounds)
        {
            return !(Max.X < bounds.X || Min.X > bounds.X || Max.Y < bounds.Y || Min.Y > bounds.Y || Max.Z < bounds.Z || Min.Z > bounds.Z);
        }

        public Int3 Clamp(Int3 p)
        {
            if (p.X < Min.X) p.X = Min.X;
            if (p.Y < Min.Y) p.Y = Min.Y;
            if (p.Z < Min.Z) p.Z = Min.Z;
            if (p.X > Max.X) p.X = Max.X;
            if (p.Y > Max.Y) p.Y = Max.Y;
            if (p.Z > Max.Z) p.Z = Max.Z;
            return p;
        }

        public void Enlarge(int margin)
        {
            Min.X -= margin;
            Min.Y -= margin;
            Min.Z -= margin;
            Max.X += margin;
            Max.Y += margin;
            Max.Z += margin;
        }

        public Box3F GetBoundsF()
        {
            // An Int32 span can be wider than Int32.MaxValue.
            long sizeX = (long)Max.X - Min.X;
            long sizeY = (long)Max.Y - Min.Y;
            long sizeZ = (long)Max.Z - Min.Z;
            float m = Math.Max(sizeX, Math.Max(sizeY, sizeZ));

            // Integer-to-float conversion can round inward. Moving one float
            // outward encloses the integer even at Int32 endpoints. Correctness
            // comes from this enclosure, not from the extra padding below.
            var result = new Box3F(
                new Vec3F(MathF.BitDecrement(Min.X), MathF.BitDecrement(Min.Y), MathF.BitDecrement(Min.Z)),
                new Vec3F(MathF.BitIncrement(Max.X), MathF.BitIncrement(Max.Y), MathF.BitIncrement(Max.Z)));
            result.Enlarge(Math.Max(m * 0.001f, 1.0f));

            // Every Int32 and binary32 value is exactly representable in binary64.
            // Comparing float directly with int would first round the integer
            // to float and could hide an inward-rounded bound.
            if ((double)result.Min.X > Min.X || (double)result.Min.Y > Min.Y || (double)result.Min.Z > Min.Z ||
                (double)result.Max.X < Max.X || (double)result.Max.Y < Max.Y || (double)result.Max.Z < Max.Z)
            {
                throw new Exception("Floating bounds do not enclose the integer box.");
            }

            return result;
        }
    }

    public struct Box2D
    {
        public Vec2D Min;
        public Vec2D Max;

        public Box2D(Vec2D min, Vec2D max)
        {
            Min = min;
            Max = max;
        }

        public void Extend(in Box2D other)
        {
            if (other.Min.X < Min.X) Min.X = other.Min.X;
            if (other.Min.Y < Min.Y) Min.Y = other.Min.Y;
            if (other.Max.X > Max.X) Max.X = other.Max.X;
            if (other.Max.Y > Max.Y) Max.Y = other.Max.Y;
        }

        public bool OverlapOrTouch(in Box2D other)
        {
            return !(Max.X < other.Min.X || Min.X > other.Max.X ||
                Max.Y < other.Min.Y || Min.Y > other.Max.Y);
        }
    }

    public struct Box3D
    {
        public Vec3D Min;
        public Vec3D Max;

        //public Vec3D Size { get { return Max - Min; } }
        public static readonly Box3D Empty = new Box3D() { Min = new Vec3D(double.MaxValue, double.MaxValue, double.MaxValue), Max = new Vec3D(double.MinValue, double.MinValue, double.MinValue) };

        public Box3D(Vec3D a) : this(a, 1e-6)
        { }
        public Box3D(Vec3D a, double enlargement)
        {
            Min = a;
            Max = a;
            Enlarge(enlargement);
        }

        public Box3D(Vec3D a, Vec3D b) : this(a, b, 1e-6)
        { }
        public Box3D(Vec3D a, Vec3D b, double enlargement)
        {
            Min = a;
            Max = a;

            IncludePoint(b);

            Enlarge(enlargement);
        }
        public Box3D(Vec3D a, Vec3D b, Vec3D c, double enlargement = 1e-6)
        {
            Min = a;
            Max = a;
            IncludePoint(b);
            IncludePoint(c);

            Enlarge(enlargement);
        }
        public Box3D(Vec3D a, Vec3D b, Vec3D c, Vec3D d, double enlargement = 1e-6)
        {
            Min = a;
            Max = a;
            IncludePoint(b);
            IncludePoint(c);
            IncludePoint(d);
            Enlarge(enlargement);
        }

        public void Enlarge(double enlargement)
        {
            Min.X -= enlargement; Min.Y -= enlargement; Min.Z -= enlargement;
            Max.X += enlargement; Max.Y += enlargement; Max.Z += enlargement;
        }

        public void IncludePoint(Vec3D p)
        {
            if (p.X < Min.X) Min.X = p.X; if (p.Y < Min.Y) Min.Y = p.Y; if (p.Z < Min.Z) Min.Z = p.Z;
            if (p.X > Max.X) Max.X = p.X; if (p.Y > Max.Y) Max.Y = p.Y; if (p.Z > Max.Z) Max.Z = p.Z;
        }

        public bool IsEmpty()
        {
            return Min.X >= Max.X || Min.Y >= Max.Y || Min.Z >= Max.Z;
        }

        public void IncludeBox(Box3D b)
        {
            IncludePoint(b.Min);
            IncludePoint(b.Max);
        }

        public static Box3D IntersectionOfBoxes(Box3D a, Box3D b)
        {
            return new Box3D(new Vec3D(Math.Max(a.Min.X, b.Min.X), Math.Max(a.Min.Y, b.Min.Y), Math.Max(a.Min.Z, b.Min.Z)),
                new Vec3D(Math.Min(a.Max.X, b.Max.X), Math.Min(a.Max.Y, b.Max.Y), Math.Min(a.Max.Z, b.Max.Z)));
        }

        public static bool Overlap(Box3D a, Box3D b)
        {
            return !(a.Min.X > b.Max.X || b.Min.X > a.Max.X ||
                a.Min.Y > b.Max.Y || b.Min.Y > a.Max.Y ||
                a.Min.Z > b.Max.Z || b.Min.Z > a.Max.Z);
        }

        public bool ContainsPoint(Vec3D p)
        {
            return p.X >= Min.X && p.X <= Max.X &&
                p.Y >= Min.Y && p.Y <= Max.Y &&
                p.Z >= Min.Z && p.Z <= Max.Z;
        }

        public double DistanceSquaredToPoint(Vec3D p)
        {
            double sqDist = 0.0;

            if (p.X < Min.X) sqDist += (Min.X - p.X) * (Min.X - p.X);
            if (p.X > Max.X) sqDist += (p.X - Max.X) * (p.X - Max.X);

            if (p.Y < Min.Y) sqDist += (Min.Y - p.Y) * (Min.Y - p.Y);
            if (p.Y > Max.Y) sqDist += (p.Y - Max.Y) * (p.Y - Max.Y);

            if (p.Z < Min.Z) sqDist += (Min.Z - p.Z) * (Min.Z - p.Z);
            if (p.Z > Max.Z) sqDist += (p.Z - Max.Z) * (p.Z - Max.Z);

            return sqDist;
        }
    }

    public struct CoordinateSystem
    {
        public Vec3D Origin;        
        public Vec3D X;
        public Vec3D Y;
        public Vec3D Z;

        public static readonly CoordinateSystem Default = new CoordinateSystem(new Vec3D(0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        /// <summary>
        /// Allowed deviation of <c>‖axis‖²</c> from 1 when <paramref name="checkOrthoNormal"/> is true.
        /// Loose enough for typical trig-built frames; tight enough to reject clearly wrong scales.
        /// </summary>
        public const double OrthonormalUnitSquaredTolerance = 1e-10;

        /// <summary>
        /// Max allowed <c>|a·b|</c> between distinct axes when <paramref name="checkOrthoNormal"/> is true.
        /// </summary>
        public const double OrthonormalDotTolerance = 1e-10;

        public CoordinateSystem(Vec3D origin, Vec3D x, Vec3D y, Vec3D z, bool checkRightHanded = true, bool checkOrthoNormal = true)
        {
            Origin = origin;
            X = x;
            Y = y;
            Z = z;

            if (checkOrthoNormal)
            {
                static void ThrowBadUnit(string axis, double lenSq)
                {
                    throw new Exception(
                        $"CoordinateSystem is not orthonormal: axis {axis} has squared length {lenSq}, expected 1±{OrthonormalUnitSquaredTolerance}.");
                }

                double xSq = x.LengthSquared();
                if (Math.Abs(xSq - 1.0) > OrthonormalUnitSquaredTolerance)
                    ThrowBadUnit(nameof(X), xSq);
                double ySq = y.LengthSquared();
                if (Math.Abs(ySq - 1.0) > OrthonormalUnitSquaredTolerance)
                    ThrowBadUnit(nameof(Y), ySq);
                double zSq = z.LengthSquared();
                if (Math.Abs(zSq - 1.0) > OrthonormalUnitSquaredTolerance)
                    ThrowBadUnit(nameof(Z), zSq);

                if (Math.Abs(Vec3DOps.Dot(x, y)) > OrthonormalDotTolerance ||
                    Math.Abs(Vec3DOps.Dot(y, z)) > OrthonormalDotTolerance ||
                    Math.Abs(Vec3DOps.Dot(x, z)) > OrthonormalDotTolerance)
                {
                    throw new Exception(
                        $"CoordinateSystem is not orthonormal: some axis pair has |dot| > {OrthonormalDotTolerance} (axes must be perpendicular).");
                }
            }

            if (checkRightHanded && Vec3DOps.Dot(Vec3DOps.Cross(x, y), z) < 0)
                throw new Exception("left handed");
        }
        public CoordinateSystem(Vec3D origin)
        {
            Origin = origin;
            X = new Vec3D(1,0,0);
            Y = new Vec3D(0,1,0);
            Z = new Vec3D(0,0,1);
        }

        public Vec3D PointFromCoordSysToWorld(Vec3D pointInCoordSysSpace)
        {
            return Origin + pointInCoordSysSpace.X * X + pointInCoordSysSpace.Y * Y + pointInCoordSysSpace.Z * Z;
        }

        public Vec3D PointFromWorldToCoordSys(Vec3D pointInWorldSpace)
        {
            pointInWorldSpace -= Origin;
            return new Vec3D(Vec3DOps.Dot(X, pointInWorldSpace), Vec3DOps.Dot(Y, pointInWorldSpace),
                Vec3DOps.Dot(Z, pointInWorldSpace));
        }

        public Vec3D DirectionFromCoordSysToWorld(Vec3D directionInCoordSysSpace)
        {
            return directionInCoordSysSpace.X * X + directionInCoordSysSpace.Y * Y + directionInCoordSysSpace.Z * Z;
        }

        public Vec3D DirectionFromWorldToCoordSys(Vec3D directionInWorldSpace)
        {
            return new Vec3D(Vec3DOps.Dot(X, directionInWorldSpace), Vec3DOps.Dot(Y, directionInWorldSpace),
                Vec3DOps.Dot(Z, directionInWorldSpace));
        }

        public Mat4D ToMatrix()
        {
            return new Mat4D()
            {
                M11 = X.X,
                M12 = Y.X,
                M13 = Z.X,
                M14 = Origin.X,

                M21 = X.Y,
                M22 = Y.Y,
                M23 = Z.Y,
                M24 = Origin.Y,

                M31 = X.Z,
                M32 = Y.Z,
                M33 = Z.Z,
                M34 = Origin.Z,

                M44 = 1
            };
        }

        public static Mat4D GetTransform(CoordinateSystem from, CoordinateSystem to)
        {
            var mFrom = from.ToMatrix();
            var mTo = to.ToMatrix();

            // Invert the "from" matrix (to go from world to 'from')
            Mat4DOps.Inverse(mFrom, out var mFromInv);
            Mat4DOps.Inverse(mTo, out var mToInv);

            var result = mTo * mFromInv;

            return result;
        }


        /// <summary>
        /// Express another coordinate system in THIS coordinate system.
        /// (World → Local)
        /// </summary>
        public CoordinateSystem ToLocal(CoordinateSystem other)
        {
            // Position
            Vec3D p = other.Origin - Origin;

            Vec3D localOrigin = new Vec3D(
                Vec3DOps.Dot(p, X),
                Vec3DOps.Dot(p, Y),
                Vec3DOps.Dot(p, Z)
            );

            // Axes (change of basis)
            Vec3D localX = new Vec3D(
                Vec3DOps.Dot(other.X, X),
                Vec3DOps.Dot(other.X, Y),
                Vec3DOps.Dot(other.X, Z)
            );

            Vec3D localY = new Vec3D(
                Vec3DOps.Dot(other.Y, X),
                Vec3DOps.Dot(other.Y, Y),
                Vec3DOps.Dot(other.Y, Z)
            );

            Vec3D localZ = new Vec3D(
                Vec3DOps.Dot(other.Z, X),
                Vec3DOps.Dot(other.Z, Y),
                Vec3DOps.Dot(other.Z, Z)
            );

            return new CoordinateSystem(localOrigin, localX, localY, localZ);
        }

        /// <summary>
        /// Convert a local coordinate system (relative to THIS one)
        /// into global/world space.
        /// (Local → World)
        /// </summary>
        public CoordinateSystem ToGlobal(CoordinateSystem local)
        {
            Vec3D globalOrigin =
                Origin +
                local.Origin.X * X +
                local.Origin.Y * Y +
                local.Origin.Z * Z;

            Vec3D globalX =
                local.X.X * X +
                local.X.Y * Y +
                local.X.Z * Z;

            Vec3D globalY =
                local.Y.X * X +
                local.Y.Y * Y +
                local.Y.Z * Z;

            Vec3D globalZ =
                local.Z.X * X +
                local.Z.Y * Y +
                local.Z.Z * Z;

            return new CoordinateSystem(globalOrigin, globalX, globalY, globalZ);
        }

        public override string ToString()
        {
            return
                $"Origin: {Origin}\n" +
                $"X:      {X}\n" +
                $"Y:      {Y}\n" +
                $"Z:      {Z}";
        }

        public Vec3D PointTo3D(Vec2D pointXY)
        {
            return Origin + pointXY.X * X + pointXY.Y * Y;
        }

        public Vec3D DirectionTo3D(Vec2D dirXY)
        {
            return dirXY.X * X + dirXY.Y * Y;
        }

        public CoordinateSystem GetOffsetCS(Vec3D originOffset)
        {
            return new CoordinateSystem(Origin + originOffset, X, Y, Z);
        }
    }

    public unsafe struct Vec4D
    {
        public double X;
        public double Y;
        public double Z;
        public double W;

        public Vec4D(double x, double y, double z, double w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }
        public Vec4D(Vec3D xyz, double w)
        {
            this.X = xyz.X;
            this.Y = xyz.Y;
            this.Z = xyz.Z;
            this.W = w;
        }


        public Vec4D(double all)
        {
            this.X = all;
            this.Y = all;
            this.Z = all;
            this.W = all;
        }

        public ref double this[int index]
        {
            get
            {
                return ref ((double*)Unsafe.AsPointer(ref X))[index];
            }
        }

        public static Vec4D operator +(Vec4D a, Vec4D b)
        {
            Vec4D result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
            return result;
        }

        public static Vec4D operator -(Vec4D a, Vec4D b)
        {
            Vec4D result;
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
            result.W = a.W - b.W;
            return result;
        }

        public static Vec4D operator -(Vec4D v)
        {
            Vec4D result;
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
            result.W = -v.W;
            return result;
        }

        public static Vec4D operator *(Vec4D a, double s)
        {
            Vec4D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            result.Z = a.Z * s;
            result.W = a.W * s;
            return result;
        }

        public static Vec4D operator *(double s, Vec4D a)
        {
            Vec4D result;
            result.X = s * a.X;
            result.Y = s * a.Y;
            result.Z = s * a.Z;
            result.W = s * a.W;
            return result;
        }

        public static Vec4D operator /(Vec4D a, double s)
        {
            s = 1.0 / s;
            Vec4D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            result.Z = a.Z * s;
            result.W = a.W * s;
            return result;
        }

        public static bool operator ==(Vec4D a, Vec4D b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
        }

        public static bool operator !=(Vec4D a, Vec4D b)
        {
            return !(a == b);
        }

        public override string ToString()
        {
            return "(" + X.ToString("F6") + "," + Y.ToString("F6") + "," + Z.ToString("F6") + "," + W.ToString("F6") + ")";
        }
    }

    public static class Vec4DOps
    {
        public static readonly Vec4D Zero = new Vec4D(0, 0, 0, 0);

        public static double DistanceSquared(Vec4D a, Vec4D b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            double dz = a.Z - b.Z;
            double dw = a.W - b.W;
            return dx * dx + dy * dy + dz * dz + dw * dw;
        }

        public static double Dot(this Vec4D a, Vec4D b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
        }

        public static double Length(this Vec4D v)
        {
            return Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z + v.W * v.W);
        }

        public static double LengthSquared(this Vec4D v)
        {
            return v.X * v.X + v.Y * v.Y + v.Z * v.Z + v.W * v.W;
        }

        public static Vec4D Normalized(this Vec4D v)
        {
            double len = v.Length();
            if (len == 0.0)
                return new Vec4D(0, 0, 0, 0);
            return v / len;
        }

        public static void Normalize(this ref Vec4D v)
        {
            double len = v.Length();
            if (len == 0.0)
                return;
            len = 1.0 / len;
            v.X = v.X * len;
            v.Y = v.Y * len;
            v.Z = v.Z * len;
            v.W = v.W * len;
        }
    }

    public unsafe struct Vec3D
    {
        public double X;
        public double Y;
        public double Z;
        public Vec3D(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }
        public Vec3D(double all)
        {
            this.X = all;
            this.Y = all;
            this.Z = all;
        }

        public ref double this[int index]
        {
            get
            {
                return ref ((double*)Unsafe.AsPointer(ref X))[index];
            }
        }

        public Vec3F ToF()
        {
            return new Vec3F((float)X, (float)Y, (float)Z);
        }

        public static Vec3D operator +(Vec3D a, Vec3D b)
        {
            Vec3D result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            return result;
        }
        public static Vec3D operator -(Vec3D a, Vec3D b)
        {
            Vec3D result;
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
            return result;
        }
        public static Vec3D operator -(Vec3D v)
        {
            Vec3D result;
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
            return result;
        }
        public static Vec3D operator *(Vec3D a, double s)
        {
            Vec3D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            result.Z = a.Z * s;
            return result;
        }

        public static Vec3D operator *(double s, Vec3D a)
        {
            Vec3D result;
            result.X = s * a.X;
            result.Y = s * a.Y;
            result.Z = s * a.Z;
            return result;
        }
        public static Vec3D operator /(Vec3D a, double s)
        {
            s = 1.0 / s;
            Vec3D result;
            result.X = a.X * s;
            result.Y = a.Y * s;
            result.Z = a.Z * s;
            return result;
        }


        public static bool operator ==(Vec3D a, Vec3D b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vec3D a, Vec3D b)
        {
            return !(a == b);
        }

        public override string ToString()
        {
            return "(" + X.ToString("F6") + "," + Y.ToString("F6") + "," + Z.ToString("F6") + ")";
        }
    }

    public struct Quaternion
    {
        public double X;
        public double Y;
        public double Z;
        public double W;

        public Quaternion(double x, double y, double z, double w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }
    }
    
    public struct Transform
    {
        public Vec3D Position;
        public Quaternion Orientation;

        public Transform(Vec3D position, Quaternion orientation)
        {
            Position = position;
            Orientation = orientation;
        }
    }

    public struct Plane
    {
        public Vec3D Normal;
        public double PlaneD;

        public Plane(Vec3D normal, double planeD)
        {
            Normal = normal;
            PlaneD = planeD;
        }

        public Plane(Vec3D a, Vec3D b, Vec3D c)
        {
            Normal = Vec3DOps.Cross(b - a, c - a);
            Normal.Normalize();
            PlaneD = -Vec3DOps.Dot(Normal, a);
        }

        public Plane(Vec3D normal, Vec3D pointOnPlane)
        {
            Normal = normal;
            PlaneD = -Vec3DOps.Dot(normal, pointOnPlane);
        }

        public Vec3D ProjectPointOntoPlane(Vec3D p)
        {
            return GeometricAlgorithms.ProjectPointOntoPlane(p, Normal, PlaneD);
        }

        public double SignedDistance(Vec3D point)
        {
            return GeometricAlgorithms.SignedDistancePointPlane(point, Normal, PlaneD);
        }
    }

    public struct Vec3F
    {
        public float X;
        public float Y;
        public float Z;
        public Vec3F(float x, float y, float z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }
        public override string ToString()
        {
            return "(" + X.ToString("F6") + "," + Y.ToString("F6") + "," + Z.ToString("F6") + ")";
        }

        public static bool operator ==(Vec3F a, Vec3F b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vec3F a, Vec3F b)
        {
            return !(a == b);
        }
    }


    public unsafe struct Tri
    {
        public int A;
        public int B;
        public int C;

        public Tri(int a, int b, int c)
        {
            A = a;
            B = b;
            C = c;
            if(a==b || a==c ||  b==c)
            {

            }
        }

        public override string ToString()
        {
            return A.ToString() + " " + B.ToString() + " " + C.ToString();
        }

        public int this[int id]
        {
            get
            {
                int* ptr = (int*)Unsafe.AsPointer<Tri>(ref this);
                return ptr[id];
            }
        }
    }
}
