namespace GeoCore
{
    public partial class UVSurface
    {
        public List<Vec3D> Points;
        public List<Rat3Hybrid> PointsPrecise; //Can be null
        public List<Vec3D> Normals; // Optional?
        public List<Vec2D> Uv;
        public List<Tri> Triangles;
        public INurbsSurface NurbsSurface;
        public ParametricRange? NurbsParamRange;

        public UVSurface(List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv, List<Tri> triangles, List<Int3> pointsPrecise)
            : this(points, normals, uv, triangles, Convert(pointsPrecise))
        {
        }

        private static List<Rat3Hybrid> Convert(List<Int3> pointsPrecise)
        {
            List<Rat3Hybrid> result = new List<Rat3Hybrid>(pointsPrecise.Count);
            for(int i=0;i<pointsPrecise.Count;++i)
            {
                var p = pointsPrecise[i];
                result.Add(new Rat3Hybrid(p.X, p.Y, p.Z));
            }
            return result;
        }

        public UVSurface(List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv, List<Tri> triangles, List<Rat3Hybrid> pointsPrecise = null)
        {
            this.Points = points;
            this.Normals = normals;
            this.Uv = uv;
            this.Triangles = triangles;
            this.PointsPrecise = pointsPrecise;
        }

        public bool DirectionsAreCollinear(Rat3Hybrid dirA, Rat3Hybrid dirB)
        {
            var cross = Rat3Hybrid.Cross(dirA, dirB);
            return cross.IsZero();
        }

        /// <summary>
        /// Face normal from triangle vertex indices in <see cref="PointsPrecise"/>; false if the cross product
        /// simplifies to zero (colinear or duplicate lattice positions).
        /// </summary>
        private bool TryGetPreciseTriangleFaceNormal(Tri tri, out Rat3Hybrid normal)
        {
            var a = PointsPrecise[tri.A];
            var b = PointsPrecise[tri.B];
            var c = PointsPrecise[tri.C];
            normal = Rat3Hybrid.Cross(b - a, c - a);
            normal.Simplify();
            return !normal.IsZero();
        }

        private int FindFirstNonDegeneratePreciseTriangleIndex()
        {
            if (PointsPrecise == null)
                return -1;
            for (int i = 0; i < Triangles.Count; i++)
            {
                if (TryGetPreciseTriangleFaceNormal(Triangles[i], out _))
                    return i;
            }
            return -1;
        }

        private Rat3Hybrid GetSurfaceNormal(out BigRationalHybrid planeD)
        {
            int ti = FindFirstNonDegeneratePreciseTriangleIndex();
            if (ti < 0)
                throw new InvalidOperationException("No triangle with non-zero area in exact (lattice) coordinates; cannot derive a plane normal.");

            var tri = Triangles[ti];
            if (!TryGetPreciseTriangleFaceNormal(tri, out Rat3Hybrid normal))
                throw new InvalidOperationException("First non-degenerate triangle index produced a zero cross (internal inconsistency).");

            var a = PointsPrecise[tri.A];
            planeD = -Rat3Hybrid.Dot(normal, a);
            planeD.Simplify();
            return normal;
        }

        private Rat3Hybrid ProjectPointOntoPlane(Rat3Hybrid p, Rat3Hybrid n, BigRationalHybrid d)
        {
            var t = (n.X * p.X + n.Y * p.Y + n.Z * p.Z + d) / (n.X * n.X + n.Y * n.Y + n.Z * n.Z);  //(Dot(n, p) + d) / Dot(n, n);          

            p.X = p.X - t * n.X;
            p.Y = p.Y - t * n.Y;
            p.Z = p.Z - t * n.Z;

            return p;
        }



        /// <summary>
        /// Whether all triangles lie in one plane, judged in exact <see cref="PointsPrecise"/> (integer lattice) space.
        /// Triangles whose cross product is exactly zero — e.g. colinear vertices after quantization — are ignored;
        /// they do not define a face normal and must not abort the test (float-space meshes can be valid while the
        /// discrete model has degenerate simplices).
        /// </summary>
        public bool IsSurfacePlanar()
        {
            if (Triangles.Count == 0 || PointsPrecise == null || PointsPrecise.Count != Points.Count)
                return false;

            int refIdx = FindFirstNonDegeneratePreciseTriangleIndex();
            if (refIdx < 0)
                return false;

            TryGetPreciseTriangleFaceNormal(Triangles[refIdx], out Rat3Hybrid referenceNormal);

            for (int i = 0; i < Triangles.Count; i++)
            {
                var tri = Triangles[i];
                if (!TryGetPreciseTriangleFaceNormal(tri, out Rat3Hybrid normal))
                    continue;

                if (!DirectionsAreCollinear(referenceNormal, normal))
                    return false;
            }

            return true;
        }

        /// <summary>
        /// Creates an extended surface by adding a tangential collar around the boundary.
        /// The extension is created by offsetting boundary vertices tangentially (perpendicular to their normals)
        /// and connecting them with triangles to form a smooth rim.
        /// </summary>
        /// <param name="extensionLength">
        /// The distance to extend outward from the boundary edge (in world units).
        /// Example: extensionLength=10 creates a 10-unit wide collar around the entire boundary.
        /// </param>
        /// <param name="useAngleBisector">
        /// If true, uses angle bisector weighting for better handling of sharp corners.
        /// Default: false (uses simple tangent averaging).
        /// </param>
        /// <param name="useAdaptiveScaling">
        /// If true, adapts the extension distance based on local edge lengths for more uniform results.
        /// Default: false (uses constant extension distance).
        /// </param>
        /// <returns>
        /// A new UVSurface containing the original surface plus the extension collar, with properly computed
        /// normals and UV coordinates. The result has no duplicate vertices - the boundary is seamlessly shared.
        /// </returns>
        /// <exception cref="InvalidOperationException">
        /// Thrown if the surface has no boundary to extend (e.g., a closed surface like a sphere).
        /// </exception>
        public UVSurface GetExtendedSurface(double extensionLength, CoordinateConverter cs, out UVSurface extensionOnly, bool useAngleBisector = false, bool useAdaptiveScaling = false)
        {
            // Ensure normals are available for the extension algorithm
            List<Vec3D> surfaceNormals = Normals;
            
            // Create mutable copies for the enlargement algorithm
            List<Vec3D> vertices = new List<Vec3D>(Points);
            List<Vec3D> normals = new List<Vec3D>(surfaceNormals);
            List<Tri> triangles = new List<Tri>(Triangles);
            List<Vec2D> uvCoordinates = new List<Vec2D>(Uv);
            
            int originalVertexCount = vertices.Count;
            int originalTriangleCount = triangles.Count;
            
            // Apply the tangential enlargement
            TangentiallyEnlargeSurface(vertices, PointsPrecise, normals, triangles, extensionLength, 
                uvCoordinates, useAngleBisector, useAdaptiveScaling);
            
            // Check if any vertices were added
            if (vertices.Count == originalVertexCount)
                throw new InvalidOperationException("Surface has no boundary to extend");

            bool surfaceIsPlanar = IsSurfacePlanar();
            Rat3Hybrid planeNormal = default;
            BigRationalHybrid planeD = default;
            if(surfaceIsPlanar)
            {
                planeNormal = GetSurfaceNormal(out planeD);
            }

            List<Rat3Hybrid> pts = new List<Rat3Hybrid>(PointsPrecise);
            for (int i = originalVertexCount; i < vertices.Count; ++i)
            {
                var p = cs.Convert(vertices[i]);
                var precise = new Rat3Hybrid(p.X, p.Y, p.Z);

                if (surfaceIsPlanar)
                {
                    precise = ProjectPointOntoPlane(precise, planeNormal, planeD);
                    precise.Simplify();
                }

                pts.Add(precise);
            }

            List<Tri> extensionOnlyTriangles = new List<Tri>(triangles.Count - originalTriangleCount);
            for(int i=originalTriangleCount;i < triangles.Count; ++i)            
                extensionOnlyTriangles.Add(triangles[i]);
            
            extensionOnly = new UVSurface(vertices, normals, uvCoordinates, extensionOnlyTriangles, pts);

            return new UVSurface(vertices, normals, uvCoordinates, triangles, pts);
        }

        public void SaveOff(string fileName)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                writer.WriteLine("OFF");
                writer.WriteLine($"{Points.Count} {Triangles.Count} 0");

                // Write vertices
                foreach (var pt in Points)
                {
                    writer.WriteLine($"{pt.X} {pt.Y} {pt.Z}");
                }

                // Write faces
                foreach (var tri in Triangles)
                {
                    writer.WriteLine($"3 {tri.A} {tri.B} {tri.C}");
                }
            }
        }

        public void SaveObj(string fileName)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                // Write vertices
                foreach (var pt in Points)
                {
                    writer.WriteLine($"v {pt.X} {pt.Y} {pt.Z}");
                }

                // Write texture coordinates
                foreach (var uv in Uv)
                {
                    writer.WriteLine($"vt {uv.X} {uv.Y}");
                }

                // Write normals
                if (Normals != null && Normals.Count > 0)
                {
                    foreach (var n in Normals)
                    {
                        writer.WriteLine($"vn {n.X} {n.Y} {n.Z}");
                    }
                }

                // Write faces (OBJ uses 1-based indexing)
                bool hasNormals = Normals != null && Normals.Count > 0;
                foreach (var tri in Triangles)
                {
                    if (hasNormals)
                    {
                        // Format: f v/vt/vn v/vt/vn v/vt/vn
                        writer.WriteLine($"f {tri.A + 1}/{tri.A + 1}/{tri.A + 1} {tri.B + 1}/{tri.B + 1}/{tri.B + 1} {tri.C + 1}/{tri.C + 1}/{tri.C + 1}");
                    }
                    else
                    {
                        // Format: f v/vt v/vt v/vt
                        writer.WriteLine($"f {tri.A + 1}/{tri.A + 1} {tri.B + 1}/{tri.B + 1} {tri.C + 1}/{tri.C + 1}");
                    }
                }
            }
        }
        public static void SaveOff(List<UVSurface> surfaces, string fileName)
        {
            // Concatenate all vertices and triangles, adjusting triangle indices accordingly
            var allVerts = new List<Vec3D>();
            var allTris = new List<Tri>();

            int vertOffset = 0;
            foreach (var surf in surfaces)
            {
                // Add vertices
                allVerts.AddRange(surf.Points);

                // Add triangles with offset adjustment
                foreach (var tri in surf.Triangles)
                {
                    allTris.Add(new Tri(
                        tri.A + vertOffset,
                        tri.B + vertOffset,
                        tri.C + vertOffset
                    ));
                }
                vertOffset += surf.Points.Count;
            }

            using (var writer = new System.IO.StreamWriter(fileName))
            {
                writer.WriteLine("OFF");
                writer.WriteLine($"{allVerts.Count} {allTris.Count} 0");

                // Write vertices
                foreach (var pt in allVerts)
                {
                    writer.WriteLine($"{pt.X} {pt.Y} {pt.Z}");
                }

                // Write faces
                foreach (var tri in allTris)
                {
                    writer.WriteLine($"3 {tri.A} {tri.B} {tri.C}");
                }
            }
        }

        public UVSurface GetOffsetSurface(double offset, CoordinateConverter cc)
        {
            // Ensure we have normals (compute them if not available)
            List<Vec3D> surfaceNormals = Normals;            
            
            // Create offset points by moving each point along its normal
            List<Vec3D> offsetPoints = new List<Vec3D>(Points.Count);
            for (int i = 0; i < Points.Count; i++)
            {
                Vec3D offsetPoint = Points[i] + surfaceNormals[i] * offset;
                offsetPoints.Add(offsetPoint);
            }
            
            // UV coordinates and triangles remain the same
            List<Vec2D> offsetUv = new List<Vec2D>(Uv.Count);
            for (int i = 0; i < Uv.Count; i++)
            {
                offsetUv.Add(Uv[i]);
            }
            
            List<Tri> offsetTriangles = new List<Tri>(Triangles.Count);
            for (int i = 0; i < Triangles.Count; i++)
            {
                offsetTriangles.Add(Triangles[i]);
            }
            
            // Normals remain the same (direction doesn't change with offset)
            List<Vec3D> offsetNormals = new List<Vec3D>(surfaceNormals.Count);
            for (int i = 0; i < surfaceNormals.Count; i++)
            {
                offsetNormals.Add(surfaceNormals[i]);
            }
            
            return new UVSurface(offsetPoints, offsetNormals, offsetUv, offsetTriangles, cc.Convert(offsetPoints));
        }

        public static List<Vec3D> ComputeAngleWeightedNormals(List<Vec3D> points, List<Tri> triangles)
        {
            // Initialize normals to zero
            List<Vec3D> vertexNormals = new List<Vec3D>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                vertexNormals.Add(new Vec3D(0, 0, 0));
            }
            
            // Process each triangle and accumulate angle-weighted face normals
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri tri = triangles[i];
                Vec3D posA = points[tri.A];
                Vec3D posB = points[tri.B];
                Vec3D posC = points[tri.C];
                
                // Compute face normal using helper method
                Vec3D faceNormal = ComputeTriangleNormal(posA, posB, posC);
                double faceNormalLength = faceNormal.Length();
                
                if (faceNormalLength > 1e-10)
                {
                    // Normalize face normal
                    Vec3D normalizedFaceNormal = faceNormal / faceNormalLength;
                    
                    // Compute edges for angle calculation
                    Vec3D edgeAB = posB - posA;
                    Vec3D edgeAC = posC - posA;
                    Vec3D edgeBC = posC - posB;
                    Vec3D edgeBA = posA - posB;
                    Vec3D edgeCB = posB - posC;
                    Vec3D edgeCA = posA - posC;
                    
                    // Compute angle at vertex A (angle between edges AB and AC)
                    double angleA = Vec3DOps.Angle(edgeAB, edgeAC);
                    vertexNormals[tri.A] = vertexNormals[tri.A] + normalizedFaceNormal * angleA;
                    
                    // Compute angle at vertex B (angle between edges BA and BC)
                    double angleB = Vec3DOps.Angle(edgeBA, edgeBC);
                    vertexNormals[tri.B] = vertexNormals[tri.B] + normalizedFaceNormal * angleB;
                    
                    // Compute angle at vertex C (angle between edges CB and CA)
                    double angleC = Vec3DOps.Angle(edgeCB, edgeCA);
                    vertexNormals[tri.C] = vertexNormals[tri.C] + normalizedFaceNormal * angleC;
                }
            }
            
            // Normalize vertex normals
            for (int i = 0; i < vertexNormals.Count; i++)
            {
                double length = vertexNormals[i].Length();
                if (length > 1e-10)
                {
                    vertexNormals[i] = vertexNormals[i] / length;
                }
                else
                {
                    // Degenerate case - use default normal
                    vertexNormals[i] = new Vec3D(0, 0, 1);
                }
            }
            
            return vertexNormals;
        }
        
        // Helper method to compute triangle normal (not normalized)
        private static Vec3D ComputeTriangleNormal(Vec3D posA, Vec3D posB, Vec3D posC)
        {
            Vec3D edgeAB = posB - posA;
            Vec3D edgeAC = posC - posA;
            return edgeAB.Cross(edgeAC);
        }


        public bool IsPlanar(double tolerance = 1e-8)
        {
            (Vec3D normal, Vec3D pointOnPlane) = PlaneFitter.FitPlane(Points, Triangles);

            for (int i = 0; i < Triangles.Count; ++i)
            {
                var tri = Triangles[i];
                var a = Points[tri.A];
                var b = Points[tri.B];
                var c = Points[tri.C];
                var dist = GeometricAlgorithms.SignedDistancePointPlane(a, normal, pointOnPlane);
                if (Math.Abs(dist) > tolerance)
                    return false;
                dist = GeometricAlgorithms.SignedDistancePointPlane(b, normal, pointOnPlane);
                if (Math.Abs(dist) > tolerance)
                    return false;
                dist = GeometricAlgorithms.SignedDistancePointPlane(c, normal, pointOnPlane);
                if (Math.Abs(dist) > tolerance)
                    return false;
            }

            return true;
        }

        public Vec3D ApproximatePlanarSurfaceCenter(out Vec3D normal, out Vec3D tangentX, out Vec3D tangentY)
        {
            // Get UV bounds
            GetUvMinMax(out Vec2D uvMin, out Vec2D uvMax);
            
            // Calculate center UV coordinates
            double centerU = (uvMin.X + uvMax.X) * 0.5;
            double centerV = (uvMin.Y + uvMax.Y) * 0.5;
            
            // First try direct evaluation at center UV
            Vec2D centerUV;
            if (Evaluate(centerU, centerV, out Vec3D position, out normal, out centerUV, out tangentX, out tangentY))
            {
                return position;
            }
            
            // Fallback: Since surface is planar and UV coords are linear, 
            // we can establish a linear UV-to-3D mapping from existing triangles
            return ApproximateUsingLinearMapping(centerU, centerV, out normal, out tangentX, out tangentY);
        }
        
        private Vec3D ApproximateUsingLinearMapping(double centerU, double centerV, out Vec3D normal, out Vec3D tangentX, out Vec3D tangentY)
        {
            if (Triangles.Count == 0)
            {
                normal = new Vec3D(0, 0, 1);
                tangentX = new Vec3D(1, 0, 0);
                tangentY = new Vec3D(0, 1, 0);
                return new Vec3D(0, 0, 0);
            }
            
            // Use largest triangle to establish the linear mapping for better numerical stability
            // For planar surfaces with linear UV, any triangle will give the same mapping,
            // but larger triangles are more robust against degenerate cases
            var tri = FindLargestTriangle();
            var posA = Points[tri.A];
            var posB = Points[tri.B]; 
            var posC = Points[tri.C];
            var uvA = Uv[tri.A];
            var uvB = Uv[tri.B];
            var uvC = Uv[tri.C];
            
            // Set up linear system: position = origin + u*tangentU + v*tangentV
            // We have: posA = origin + uvA.X*tangentU + uvA.Y*tangentV
            //          posB = origin + uvB.X*tangentU + uvB.Y*tangentV  
            //          posC = origin + uvC.X*tangentU + uvC.Y*tangentV
            
            // Solve for tangentU and tangentV using two edge equations:
            // (posB - posA) = (uvB.X - uvA.X)*tangentU + (uvB.Y - uvA.Y)*tangentV
            // (posC - posA) = (uvC.X - uvA.X)*tangentU + (uvC.Y - uvA.Y)*tangentV
            
            var edge1_3D = posB - posA;
            var edge2_3D = posC - posA;
            var edge1_UV = uvB - uvA;
            var edge2_UV = uvC - uvA;
            
            // Calculate determinant for 2x2 UV matrix
            double uvDet = edge1_UV.Cross(edge2_UV);
            
            if (Math.Abs(uvDet) < 1e-10)
            {
                // Degenerate case - use face normal and arbitrary tangents
                var faceNormal = edge1_3D.Cross(edge2_3D);
                if (faceNormal.Length() > 1e-10)
                {
                    normal = faceNormal.Normalized();
                }
                else
                {
                    normal = new Vec3D(0, 0, 1);
                }
                
                tangentX = edge1_3D.Length() > 1e-10 ? edge1_3D.Normalized() : new Vec3D(1, 0, 0);
                tangentY = normal.Cross(tangentX).Normalized();
                
                // Return centroid as approximation
                return (posA + posB + posC) * (1.0 / 3.0);
            }
            
            // Solve for tangent vectors using Cramer's rule (inverted 2x2 matrix)
            double invDet = 1.0 / uvDet;
            tangentX = (edge1_3D * edge2_UV.Y - edge2_3D * edge1_UV.Y) * invDet;
            tangentY = (edge2_3D * edge1_UV.X - edge1_3D * edge2_UV.X) * invDet;
            
            // Calculate normal as cross product of tangents
            normal = tangentX.Cross(tangentY);
            if (normal.Length() > 1e-10)
            {
                normal = normal.Normalized();
            }
            else
            {
                normal = new Vec3D(0, 0, 1);
            }
            
            // Normalize tangent vectors
            if (tangentX.Length() > 1e-10)
                tangentX = tangentX.Normalized();
            if (tangentY.Length() > 1e-10) 
                tangentY = tangentY.Normalized();
                
            // Calculate origin: origin = posA - uvA.X*tangentX - uvA.Y*tangentY
            var origin = posA - tangentX * uvA.X - tangentY * uvA.Y;
            
            // Calculate center position: centerPos = origin + centerU*tangentX + centerV*tangentY
            var centerPosition = origin + tangentX * centerU + tangentY * centerV;
            
            return centerPosition;
        }
        
        private Tri FindLargestTriangle()
        {
            if (Triangles.Count == 0)
                throw new InvalidOperationException("No triangles available");
                
            double maxArea = 0;
            Tri largestTri = Triangles[0];
            
            for (int i = 0; i < Triangles.Count; i++)
            {
                var tri = Triangles[i];
                var posA = Points[tri.A];
                var posB = Points[tri.B];
                var posC = Points[tri.C];
                
                // Calculate triangle area using cross product
                var edge1 = posB - posA;
                var edge2 = posC - posA;
                double area = 0.5 * edge1.Cross(edge2).Length();
                
                if (area > maxArea)
                {
                    maxArea = area;
                    largestTri = tri;
                }
            }
            
            return largestTri;
        }

        public void GetUvMinMax(out Vec2D uvMin, out Vec2D uvMax)
        {
            if (Triangles.Count == 0)
            {
                uvMin = new Vec2D(0, 0);
                uvMax = new Vec2D(0, 0);
                return;
            }

            uvMin = new Vec2D(double.MaxValue, double.MaxValue);
            uvMax = new Vec2D(double.MinValue, double.MinValue);

            for (int i = 0; i < Triangles.Count; ++i)
            {
                var tri = Triangles[i];
                
                // Get UV coordinates of triangle vertices
                var uvA = Uv[tri.A];
                var uvB = Uv[tri.B];
                var uvC = Uv[tri.C];
                
                uvMin.X = Math.Min(uvMin.X, Math.Min(Math.Min(uvA.X, uvB.X), uvC.X));
                uvMin.Y = Math.Min(uvMin.Y, Math.Min(Math.Min(uvA.Y, uvB.Y), uvC.Y));
                uvMax.X = Math.Max(uvMax.X, Math.Max(Math.Max(uvA.X, uvB.X), uvC.X));
                uvMax.Y = Math.Max(uvMax.Y, Math.Max(Math.Max(uvA.Y, uvB.Y), uvC.Y));
            }
        }

        public bool Evaluate(double u, double v, out Vec3D position, out Vec3D normal, out Vec2D uv, out Vec3D tangentX, out Vec3D tangentY)
        {
            if (NurbsSurface != null)
            {
                position = NurbsSurface.Evaluate(u, v);
                normal = NurbsSurface.EvaluateNormal(u, v);
                if (normal.Length() > 1e-10)
                    normal = normal.Normalized();
                uv = new Vec2D(u, v);
                double eps = 1e-6;
                tangentX = NurbsSurface.Evaluate(u + eps, v) - position;
                tangentY = NurbsSurface.Evaluate(u, v + eps) - position;
                return true;
            }

            // Search all triangles to find the one that contains uv parameters u, v            
            for (int i = 0; i < Triangles.Count; i++)
            {
                var tri = Triangles[i];
                
                // Get UV coordinates of triangle vertices
                var uvA = this.Uv[tri.A];
                var uvB = this.Uv[tri.B];
                var uvC = this.Uv[tri.C];
                
                Vec3D barycentricWeight;
                if (TriangleContainsUV(uvA, uvB, uvC, u, v, out barycentricWeight))
                {
                    // Interpolate position using barycentric coordinates
                    var posA = Points[tri.A];
                    var posB = Points[tri.B];
                    var posC = Points[tri.C];
                    
                    position = posA * barycentricWeight.X + posB * barycentricWeight.Y + posC * barycentricWeight.Z;
                    
                    // Interpolate normal using barycentric coordinates (if normals are provided)
                    if (Normals != null && Normals.Count > tri.A && Normals.Count > tri.B && Normals.Count > tri.C)
                    {
                        var normA = Normals[tri.A];
                        var normB = Normals[tri.B];
                        var normC = Normals[tri.C];
                        
                        normal = normA * barycentricWeight.X + normB * barycentricWeight.Y + normC * barycentricWeight.Z;
                        
                        // Normalize the interpolated normal
                        if (normal.Length() > 1e-10)
                        {
                            normal = normal.Normalized();
                        }
                    }
                    else
                    {
                        // Calculate face normal if vertex normals are not available
                        var edge1 = posB - posA;
                        var edge2 = posC - posA;
                        
                        // Cross product for face normal
                        normal = edge1.Cross(edge2);
                        
                        // Normalize the face normal
                        if (normal.Length() > 1e-10)
                        {
                            normal = normal.Normalized();
                        }
                    }
                    
                    // Compute tangent vectors from UV-to-3D mapping
                    // We solve the system: edge3D = edgeU * tangentX + edgeV * tangentY
                    var edge1_3D = posB - posA;
                    var edge2_3D = posC - posA;
                    var edge1_UV = uvB - uvA;
                    var edge2_UV = uvC - uvA;
                    
                    // Calculate determinant of UV matrix using 2D cross product
                    double uvDet = edge1_UV.Cross(edge2_UV);
                    
                    if (Math.Abs(uvDet) > 1e-10)
                    {
                        // Solve using Cramer's rule (inverted 2x2 matrix)
                        double invDet = 1.0 / uvDet;
                        tangentX = (edge1_3D * edge2_UV.Y - edge2_3D * edge1_UV.Y) * invDet;
                        tangentY = (edge2_3D * edge1_UV.X - edge1_3D * edge2_UV.X) * invDet;
                        
                        // Normalize tangent vectors (optional, but often desirable)
                        if (tangentX.Length() > 1e-10)
                            tangentX = tangentX.Normalized();
                        if (tangentY.Length() > 1e-10)
                            tangentY = tangentY.Normalized();
                    }
                    else
                    {
                        // Fallback for degenerate UV mapping - use orthogonal vectors in the triangle plane
                        tangentX = edge1_3D.Length() > 1e-10 ? edge1_3D.Normalized() : new Vec3D(1, 0, 0);
                        tangentY = normal.Cross(tangentX);
                        if (tangentY.Length() > 1e-10)
                            tangentY = tangentY.Normalized();
                    }
                    
                    // Return the interpolated UV coordinates
                    uv = new Vec2D(u, v);
                    
                    return true;
                }
            }
            
            position = default;
            normal = default;
            uv = default;
            tangentX = default;
            tangentY = default;
            return false;
        }


        public  bool TriangleContainsUV(Vec2D a, Vec2D b, Vec2D c, double u, double v, out Vec3D barycentricWeight)
        {
            Vec2D p = new Vec2D(u, v);

            // Calculate barycentric coordinates using the standard formula
            // P = α*A + β*B + γ*C where α + β + γ = 1
            // We solve the system:
            // P.x = α*A.x + β*B.x + γ*C.x
            // P.y = α*A.y + β*B.y + γ*C.y
            // 1   = α     + β     + γ

            Vec2D v0 = b - c;  // Edge vector from c to b
            Vec2D v1 = a - c;  // Edge vector from c to a
            Vec2D v2 = p - c;  // Vector from c to query point

            // Calculate determinant using cross product (2D cross product gives scalar)
            double denom = v0.Cross(v1);

            // Check if triangle is degenerate
            if (Math.Abs(denom) < 1e-10)
            {
                barycentricWeight = new Vec3D(0, 0, 0);
                return false;
            }

            double alpha = v0.Cross(v2) / denom;
            double beta = v2.Cross(v1) / denom;
            double gamma = 1.0 - alpha - beta;

            // Check if point is inside triangle (all barycentric coordinates must be >= 0)
            if (alpha >= -1e-10 && beta >= -1e-10 && gamma >= -1e-10)
            {
                barycentricWeight = new Vec3D(alpha, beta, gamma);
                return true;
            }

            barycentricWeight = new Vec3D(0, 0, 0);
            return false;
        }
    }
}
