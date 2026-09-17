using GeoCore;

namespace Geo
{
    public static class InterpolationHelpers
    {
        public static Vec3D InterpolateBarycentricNormalized(in Vec3D v0, in Vec3D v1, in Vec3D v2, in Vec3D barycentricWeights)
        {
            var result = InterpolateBarycentric(v0, v1, v2, barycentricWeights);
            // Normalize the normal
            double length = Math.Sqrt(result.X * result.X + result.Y * result.Y + result.Z * result.Z);
            if (length > 1e-8)
            {
                result.X /= length;
                result.Y /= length;
                result.Z /= length;
            }
            return result;
        }
        public static Vec3D InterpolateBarycentric(in Vec3D v0, in Vec3D v1, in Vec3D v2, in Vec3D barycentricWeights)
        {
            return new Vec3D(
                v0.X * barycentricWeights.X + v1.X * barycentricWeights.Y + v2.X * barycentricWeights.Z,
                v0.Y * barycentricWeights.X + v1.Y * barycentricWeights.Y + v2.Y * barycentricWeights.Z,
                v0.Z * barycentricWeights.X + v1.Z * barycentricWeights.Y + v2.Z * barycentricWeights.Z);
        }
        public static Vec2D InterpolateBarycentric(in Vec2D v0, in Vec2D v1, in Vec2D v2, in Vec3D barycentricWeights)
        {
            return new Vec2D(
                v0.X * barycentricWeights.X + v1.X * barycentricWeights.Y + v2.X * barycentricWeights.Z,
                v0.Y * barycentricWeights.X + v1.Y * barycentricWeights.Y + v2.Y * barycentricWeights.Z);
        }

        public static Vec3D InterpolateBarycentricNormalizedRobust(in Vec3D v0, in Vec3D v1, in Vec3D v2, in Vec3D barycentricWeights)
        {
            var result = InterpolateBarycentricRobust(v0, v1, v2, barycentricWeights);
            // Normalize the normal
            double length = Math.Sqrt(result.X * result.X + result.Y * result.Y + result.Z * result.Z);
            if (length > 1e-8)
            {
                result.X /= length;
                result.Y /= length;
                result.Z /= length;
            }
            return result;
        }
        public static Vec3D InterpolateBarycentricRobust(Vec3D v0, Vec3D v1, Vec3D v2, Vec3D barycentricWeights)
        {
            if (barycentricWeights.X == 1 && barycentricWeights.Y == 0 && barycentricWeights.Z == 0)
            {
                return v0;
            }
            if (barycentricWeights.X == 0 && barycentricWeights.Y == 1 && barycentricWeights.Z == 0)
            {
                return v1;
            }
            if (barycentricWeights.X == 0 && barycentricWeights.Y == 0 && barycentricWeights.Z == 1)
            {
                return v2;
            }

            if (barycentricWeights.X == 0)
            {
                return InterpolateRobust(v1, v2, barycentricWeights.Y, barycentricWeights.Z);
            }
            if (barycentricWeights.Y == 0)
            {
                return InterpolateRobust(v0, v2, barycentricWeights.X, barycentricWeights.Z);
            }
            if (barycentricWeights.Z == 0)
            {
                return InterpolateRobust(v0, v1, barycentricWeights.X, barycentricWeights.Y);
            }

            // Sort v0, v1 and v2 using IsSmaller. Also swap barycentric weights accordingly
            double w0 = barycentricWeights.X;
            double w1 = barycentricWeights.Y;
            double w2 = barycentricWeights.Z;
            
            // Sort the three vertices (bubble sort for 3 elements)
            // First pass: compare v0 and v1
            if (IsSmaller(v1, v0))
            {
                var temp = v0; v0 = v1; v1 = temp;
                var tempW = w0; w0 = w1; w1 = tempW;
            }
            
            // Second pass: compare v1 and v2
            if (IsSmaller(v2, v1))
            {
                var temp = v1; v1 = v2; v2 = temp;
                var tempW = w1; w1 = w2; w2 = tempW;
            }
            
            // Third pass: compare v0 and v1 again (bubble sort completion)
            if (IsSmaller(v1, v0))
            {
                var temp = v0; v0 = v1; v1 = temp;
                var tempW = w0; w0 = w1; w1 = tempW;
            }

            return new Vec3D(
                v0.X * w0 + v1.X * w1 + v2.X * w2,
                v0.Y * w0 + v1.Y * w1 + v2.Y * w2,
                v0.Z * w0 + v1.Z * w1 + v2.Z * w2);
        }
        public static Vec2D InterpolateBarycentricRobust(Vec2D v0, Vec2D v1, Vec2D v2, Vec3D barycentricWeights)
        {
            if (barycentricWeights.X == 1 && barycentricWeights.Y == 0 && barycentricWeights.Z == 0)
            {
                return v0;
            }
            if (barycentricWeights.X == 0 && barycentricWeights.Y == 1 && barycentricWeights.Z == 0)
            {
                return v1;
            }
            if (barycentricWeights.X == 0 && barycentricWeights.Y == 0 && barycentricWeights.Z == 1)
            {
                return v2;
            }

            if (barycentricWeights.X == 0)
            {
                return InterpolateRobust(v1, v2, barycentricWeights.Y, barycentricWeights.Z);
            }
            if (barycentricWeights.Y == 0)
            {
                return InterpolateRobust(v0, v2, barycentricWeights.X, barycentricWeights.Z);
            }
            if (barycentricWeights.Z == 0)
            {
                return InterpolateRobust(v0, v1, barycentricWeights.X, barycentricWeights.Y);
            }

            // Sort v0, v1 and v2 using IsSmaller. Also swap barycentric weights accordingly
            double w0 = barycentricWeights.X;
            double w1 = barycentricWeights.Y;
            double w2 = barycentricWeights.Z;
            
            // Sort the three vertices (bubble sort for 3 elements)
            // First pass: compare v0 and v1
            if (IsSmaller(v1, v0))
            {
                var temp = v0; v0 = v1; v1 = temp;
                var tempW = w0; w0 = w1; w1 = tempW;
            }
            
            // Second pass: compare v1 and v2
            if (IsSmaller(v2, v1))
            {
                var temp = v1; v1 = v2; v2 = temp;
                var tempW = w1; w1 = w2; w2 = tempW;
            }
            
            // Third pass: compare v0 and v1 again (bubble sort completion)
            if (IsSmaller(v1, v0))
            {
                var temp = v0; v0 = v1; v1 = temp;
                var tempW = w0; w0 = w1; w1 = tempW;
            }

            return new Vec2D(
                v0.X * w0 + v1.X * w1 + v2.X * w2,
                v0.Y * w0 + v1.Y * w1 + v2.Y * w2);
        }

        private static Vec2D InterpolateRobust(Vec2D a, Vec2D b, double weightA, double weightB)
        {
            bool swap = IsSmaller(b, a);
            if (swap)
            {
                var temp = a;
                a = b;
                b = temp;
                var tempW = weightA;
                weightA = weightB;
                weightB = tempW;
            }
            return new Vec2D(a.X * weightA + b.X * weightB, a.Y * weightA + b.Y * weightB);
        }

        private static Vec3D InterpolateRobust(Vec3D a, Vec3D b, double weightA, double weightB)
        {
            bool swap = IsSmaller(b, a);
            if (swap)
            {
                var temp = a;
                a = b;
                b = temp;
                var tempW = weightA;
                weightA = weightB;
                weightB = tempW;
            }
            return new Vec3D(a.X * weightA + b.X * weightB, a.Y * weightA + b.Y * weightB, a.Z * weightA + b.Z * weightB);
        }

        private static bool IsSmaller(in Vec3D a, in Vec3D b)
        {
            if (a.X == b.X)
            {
                if (a.Y == b.Y)
                {
                    return a.Z < b.Z;
                }
                return a.Y < b.Y;
            }
            return a.X < b.X;
        }

        private static bool IsSmaller(in Vec2D a, in Vec2D b)
        {
            if (a.X == b.X)
            {
                return a.Y < b.Y;
            }
            return a.X < b.X;
        }






        // Boolean geometry is resolved in exact converter coordinates. Compute
        // its corner weights there too, so boundary points stay on their source
        // edges even when display coordinates differ after quantization.
        internal static Vec3D GetBarycentricWeights(Rat3Hybrid point, Tri triangle, List<Rat3Hybrid> positions)
        {
            var weights = GetExactBarycentricWeights(point, triangle, positions);
            return new Vec3D(weights.X.ToDouble(), weights.Y.ToDouble(), weights.Z.ToDouble());
        }

        internal static Rat3Hybrid GetExactBarycentricWeights(Rat3Hybrid point, Tri triangle, List<Rat3Hybrid> positions)
        {
            var a = positions[triangle.A];
            var b = positions[triangle.B];
            var c = positions[triangle.C];
            if (point == a) return new Rat3Hybrid(1, 0, 0);
            if (point == b) return new Rat3Hybrid(0, 1, 0);
            if (point == c) return new Rat3Hybrid(0, 0, 1);
            var ab = b - a;
            var ac = c - a;
            var ap = point - a;
            var normal = Rat3Hybrid.Cross(ab, ac);
            BigRationalHybrid denominator, numeratorB, numeratorC;
            if (normal.X.Sign() != 0)
            {
                denominator = normal.X;
                numeratorB = ap.Y * ac.Z - ap.Z * ac.Y;
                numeratorC = ab.Y * ap.Z - ab.Z * ap.Y;
            }
            else if (normal.Y.Sign() != 0)
            {
                denominator = normal.Y;
                numeratorB = ap.Z * ac.X - ap.X * ac.Z;
                numeratorC = ab.Z * ap.X - ab.X * ap.Z;
            }
            else
            {
                denominator = normal.Z;
                numeratorB = ap.X * ac.Y - ap.Y * ac.X;
                numeratorC = ab.X * ap.Y - ab.Y * ap.X;
            }
            if (denominator.Sign() == 0)
                throw new InvalidOperationException("Cannot interpolate attributes on an exact zero-area source triangle.");
            BigRationalHybrid Weight(BigRationalHybrid numerator)
            {
                var ratio = numerator / denominator;
                ratio.Simplify();
                return ratio;
            }
            return new Rat3Hybrid(Weight(denominator - numeratorB - numeratorC), Weight(numeratorB), Weight(numeratorC));
        }

        public static Vec3D GetBarycentricWeights(Vec3D point, Tri sourceTriangle, List<Vec3D> sourcePositions)
        {
            // Get the three vertices of the source triangle
            Vec3D v0 = sourcePositions[sourceTriangle.A];
            Vec3D v1 = sourcePositions[sourceTriangle.B];
            Vec3D v2 = sourcePositions[sourceTriangle.C];

            return GetBarycentricWeightsRobust(point, v0, v1, v2);
        }

        public static Vec3D GetBarycentricWeightsRobust(Vec3D point, Vec3D v0, Vec3D v1, Vec3D v2)
        {
            // Step 1: Make vertex ordering deterministic for bit-identical results
            // Use direct variable swapping instead of arrays
            int index0 = 0, index1 = 1, index2 = 2;

            // Sort vertices by position using direct swapping (bubble sort for 3 elements)
            // First pass: compare v0 and v1
            if (IsSmaller(v1, v0))
            {
                var tempV = v0; v0 = v1; v1 = tempV;
                var tempI = index0; index0 = index1; index1 = tempI;
            }
            
            // Second pass: compare v1 and v2
            if (IsSmaller(v2, v1))
            {
                var tempV = v1; v1 = v2; v2 = tempV;
                var tempI = index1; index1 = index2; index2 = tempI;
            }
            
            // Third pass: compare v0 and v1 again (bubble sort completion)
            if (IsSmaller(v1, v0))
            {
                var tempV = v0; v0 = v1; v1 = tempV;
                var tempI = index0; index0 = index1; index1 = tempI;
            }

            // Now v0, v1, v2 are sorted and index0, index1, index2 track original positions

            // Step 2: Handle exact vertex matches first (most robust case)
            if (point.IsIdentical(v0))
            {
                var result = new Vec3D(0, 0, 0);
                if (index0 == 0) result.X = 1;
                else if (index0 == 1) result.Y = 1;
                else result.Z = 1;
                return result;
            }
            if (point.IsIdentical(v1))
            {
                var result = new Vec3D(0, 0, 0);
                if (index1 == 0) result.X = 1;
                else if (index1 == 1) result.Y = 1;
                else result.Z = 1;
                return result;
            }
            if (point.IsIdentical(v2))
            {
                var result = new Vec3D(0, 0, 0);
                if (index2 == 0) result.X = 1;
                else if (index2 == 1) result.Y = 1;
                else result.Z = 1;
                return result;
            }

            // Step 3: Use numerically stable barycentric computation with consistent ordering
            // Compute vectors from sorted vertices
            Vec3D v0v1 = new Vec3D(v1.X - v0.X, v1.Y - v0.Y, v1.Z - v0.Z);
            Vec3D v0v2 = new Vec3D(v2.X - v0.X, v2.Y - v0.Y, v2.Z - v0.Z);
            Vec3D v0p = new Vec3D(point.X - v0.X, point.Y - v0.Y, point.Z - v0.Z);

            // Compute dot products with consistent ordering
            double dot00 = v0v2.X * v0v2.X + v0v2.Y * v0v2.Y + v0v2.Z * v0v2.Z;
            double dot01 = v0v2.X * v0v1.X + v0v2.Y * v0v1.Y + v0v2.Z * v0v1.Z;
            double dot02 = v0v2.X * v0p.X + v0v2.Y * v0p.Y + v0v2.Z * v0p.Z;
            double dot11 = v0v1.X * v0v1.X + v0v1.Y * v0v1.Y + v0v1.Z * v0v1.Z;
            double dot12 = v0v1.X * v0p.X + v0v1.Y * v0p.Y + v0v1.Z * v0p.Z;

            // Step 4: Compute barycentric coordinates
            // The denominator represents twice the triangle area squared
            double denom = dot00 * dot11 - dot01 * dot01;
            
            // Handle degenerate triangle case (zero area) - exact check, no epsilon
            if (denom == 0.0)
            {
                // Degenerate triangle - return closest vertex using exact arithmetic
                double dist0 = (point.X - v0.X) * (point.X - v0.X) + 
                              (point.Y - v0.Y) * (point.Y - v0.Y) + 
                              (point.Z - v0.Z) * (point.Z - v0.Z);
                double dist1 = (point.X - v1.X) * (point.X - v1.X) + 
                              (point.Y - v1.Y) * (point.Y - v1.Y) + 
                              (point.Z - v1.Z) * (point.Z - v1.Z);
                double dist2 = (point.X - v2.X) * (point.X - v2.X) + 
                              (point.Y - v2.Y) * (point.Y - v2.Y) + 
                              (point.Z - v2.Z) * (point.Z - v2.Z);

                var result = new Vec3D(0, 0, 0);
                if (dist0 <= dist1 && dist0 <= dist2)
                {
                    if (index0 == 0) result.X = 1;
                    else if (index0 == 1) result.Y = 1;
                    else result.Z = 1;
                }
                else if (dist1 <= dist2)
                {
                    if (index1 == 0) result.X = 1;
                    else if (index1 == 1) result.Y = 1;
                    else result.Z = 1;
                }
                else
                {
                    if (index2 == 0) result.X = 1;
                    else if (index2 == 1) result.Y = 1;
                    else result.Z = 1;
                }
                return result;
            }

            // Step 5: Compute barycentric coordinates with exact division
            double invDenom = 1.0 / denom;
            double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            double w = 1.0 - u - v;

            // Step 6: Map back to original vertex ordering
            var finalResult = new Vec3D(0, 0, 0);
            
            // Assign weights based on original vertex indices
            if (index0 == 0) finalResult.X = w;
            else if (index0 == 1) finalResult.Y = w;
            else finalResult.Z = w;
            
            if (index1 == 0) finalResult.X = v;
            else if (index1 == 1) finalResult.Y = v;
            else finalResult.Z = v;
            
            if (index2 == 0) finalResult.X = u;
            else if (index2 == 1) finalResult.Y = u;
            else finalResult.Z = u;

            return finalResult;
        }

        public static bool IsIdentical(this Vec3D a, Vec3D b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }
    }
}
