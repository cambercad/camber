using CSG;
using GeoCore;
using System;
using System.Collections.Generic;

namespace Geo
{
    /// <summary>
    /// Represents a circular arc at the end of a blend edge where it terminates at a corner.
    /// The arc connects two boundary points and lies on a sphere centered at the corner.
    /// </summary>
    public class BlendTerminationArc
    {
        public Rat3Hybrid PointA { get; set; }  // First boundary point (on surface A)
        public Rat3Hybrid PointB { get; set; }  // Second boundary point (on surface B)
        public Rat3Hybrid CenterPoint { get; set; }  // Center of the blend curve
        
        // Computed properties
        public Vec3D OrientedPlaneNormal { get; set; }  // Oriented normal of the arc plane
        public Rat3Hybrid SphereCenter { get; set; }  // Center of the sphere (corner center)
        
        // Store the coordinate converter for conversions
        private CoordinateConverter cc;

        public BlendTerminationArc(Rat3Hybrid pointA, Rat3Hybrid pointB, Rat3Hybrid centerPoint, CoordinateConverter cc)
        {
            PointA = pointA;
            PointB = pointB;
            CenterPoint = centerPoint;
            this.cc = cc;
        }

        /// <summary>
        /// Generate points along the circular arc
        /// </summary>
        public List<Vec3D> GenerateArcPoints(Vec3D sphereCenter, double tolerance)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D dirA = (a - sphereCenter).Normalized();
            Vec3D dirB = (b - sphereCenter).Normalized();
            
            double radius = (a - sphereCenter).Length();
            
            Vec3D axis = Vec3DOps.Cross(dirA, dirB);
            if (axis.LengthSquared() < 1e-20)
            {
                // Degenerate case: points are collinear
                return new List<Vec3D> { a, b };
            }
            
            axis = axis.Normalized();
            double angle = Vec3DOps.Angle(dirA, dirB, axis);
            
            // Compute number of segments based on tolerance
            double delta = 2.0 * Math.Acos(Math.Max(-1, Math.Min(1, (radius - tolerance) / radius)));
            int numSegments = Math.Max(1, (int)Math.Ceiling(angle / delta));
            
            List<Vec3D> result = new List<Vec3D>();
            Vec3D right = Vec3DOps.Cross(axis, dirA).Normalized();
            
            for (int i = 0; i <= numSegments; i++)
            {
                double t = i / (double)numSegments;
                double a_t = t * angle;
                Vec3D point = sphereCenter + radius * (Math.Cos(a_t) * dirA + Math.Sin(a_t) * right);
                result.Add(point);
            }
            
            return result;
        }

        /// <summary>
        /// Compute the plane normal of the arc
        /// </summary>
        public Vec3D ComputePlaneNormal(Vec3D sphereCenter)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D dirA = a - sphereCenter;
            Vec3D dirB = b - sphereCenter;
            
            return Vec3DOps.Cross(dirA, dirB).Normalized();
        }

        /// <summary>
        /// Get the tangent vector at the start of the arc
        /// </summary>
        public Vec3D GetStartTangent(Vec3D sphereCenter)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D dirA = a - sphereCenter;
            Vec3D dirB = b - sphereCenter;
            Vec3D planeNormal = Vec3DOps.Cross(dirA, dirB);
            
            return Vec3DOps.Cross(planeNormal, dirA).Normalized();
        }

        /// <summary>
        /// Get the tangent vector at the end of the arc
        /// </summary>
        public Vec3D GetEndTangent(Vec3D sphereCenter)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D dirA = a - sphereCenter;
            Vec3D dirB = b - sphereCenter;
            Vec3D planeNormal = Vec3DOps.Cross(dirA, dirB);
            
            return Vec3DOps.Cross(planeNormal, dirB).Normalized();
        }

        /// <summary>
        /// Get the middle point on the arc
        /// </summary>
        public Vec3D GetMiddlePoint(Vec3D sphereCenter)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D dirA = a - sphereCenter;
            Vec3D dirB = b - sphereCenter;
            
            double radius = dirA.Length();
            
            Vec3D axis = Vec3DOps.Cross(dirA, dirB).Normalized();
            double angle = Vec3DOps.Angle(dirA, dirB, axis);
            double halfAngle = angle * 0.5;
            
            Vec3D dirANorm = dirA.Normalized();
            Vec3D right = Vec3DOps.Cross(axis, dirANorm).Normalized();
            
            return sphereCenter + radius * (Math.Cos(halfAngle) * dirANorm + Math.Sin(halfAngle) * right);
        }

        /// <summary>
        /// Check if a point is in the normal direction of the arc plane or on the plane
        /// </summary>
        public bool IsPointInNormalDirectionOrOnPlane(Vec3D point, Vec3D sphereCenter)
        {
            Vec3D a = cc.Convert(PointA);
            Vec3D b = cc.Convert(PointB);
            
            Vec3D n = Vec3DOps.Cross(a - sphereCenter, b - sphereCenter);
            double sign = 1.0;
            if (Vec3DOps.Dot(n, OrientedPlaneNormal) < 0)
                sign = -1.0;
            
            double orient = Orient3D(a, b, sphereCenter, point);
            return orient * sign <= 0;
        }

        /// <summary>
        /// Compute the orientation of four points (determinant of 4x4 matrix)
        /// </summary>
        private static double Orient3D(Vec3D pa, Vec3D pb, Vec3D pc, Vec3D pd)
        {
            double adx = pa.X - pd.X;
            double bdx = pb.X - pd.X;
            double cdx = pc.X - pd.X;
            double ady = pa.Y - pd.Y;
            double bdy = pb.Y - pd.Y;
            double cdy = pc.Y - pd.Y;
            double adz = pa.Z - pd.Z;
            double bdz = pb.Z - pd.Z;
            double cdz = pc.Z - pd.Z;

            return adx * (bdy * cdz - bdz * cdy)
                 + bdx * (cdy * adz - cdz * ady)
                 + cdx * (ady * bdz - adz * bdy);
        }
    }
}

