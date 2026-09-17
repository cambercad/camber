using CSG;
using GeoCore;
using System.Diagnostics;

namespace Geo
{
    public struct TriangleVertexNormalUV : ITriangleVertex<TriangleVertexNormalUV>
    {
        //Positions are stored in a separate array
        public Vec3D Normal;
        public Vec2D UV;

        internal TriangleVertexNormalUV InterpolateExact(in TriangleVertexNormalUV second,
            in TriangleVertexNormalUV third, in Rat3Hybrid weights)
        {
            var result = Interpolate(second, third,
                new Vec3D(weights.X.ToDouble(), weights.Y.ToDouble(), weights.Z.ToDouble()));
            var w = weights;
            double Component(double a, double b, double c)
            {
                static BigRationalHybrid Exact(double value)
                {
                    if (!double.IsFinite(value))
                        throw new InvalidOperationException("Surface UV coordinates must be finite.");
                    BigRational rational = value;
                    return new BigRationalHybrid(rational.Numerator, rational.Denominator);
                }
                // Keep the complete affine combination exact until its one
                // conversion to display UV, including constant seam values.
                var value = Exact(a) * w.X + Exact(b) * w.Y + Exact(c) * w.Z;
                value.Simplify();
                return value.ToDouble();
            }
            result.UV = new Vec2D(Component(UV.X, second.UV.X, third.UV.X),
                Component(UV.Y, second.UV.Y, third.UV.Y));
            return result;
        }

        public TriangleVertexNormalUV Interpolate(in TriangleVertexNormalUV second, in TriangleVertexNormalUV third, in Vec3D barycentricWeights)
        {
            TriangleVertexNormalUV result;
            result.UV = InterpolationHelpers.InterpolateBarycentricRobust(this.UV, second.UV, third.UV, barycentricWeights);

            result.Normal = InterpolationHelpers.InterpolateBarycentricNormalizedRobust(this.Normal, second.Normal, third.Normal, barycentricWeights);

            return result;
        }
    }

    public class MeshNormalUV : Mesh<TriangleVertexNormalUV>
    {
        public MeshNormalUV() { }

        internal MeshNormalUV SnapshotRigidPose(IEnumerable<Rat3Hybrid> restPoints,
            CoordinateConverter converter, Transform transform, int? groupId = null)
        {
            var exact = new PreciseRigidTransform(converter, in transform);
            var matrix = TransformMath.ToMat4D(in transform);
            var points = restPoints.Select(exact.Apply).ToList();
            var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(TrianglesEx);
            for (int i = 0; i < corners.Count; i++)
            {
                var triangle = corners[i];
                if (groupId.HasValue) triangle.GroupId = groupId.Value;
                triangle.V0.Normal = matrix.TransformDirection(triangle.V0.Normal);
                triangle.V1.Normal = matrix.TransformDirection(triangle.V1.Normal);
                triangle.V2.Normal = matrix.TransformDirection(triangle.V2.Normal);
                corners[i] = triangle;
            }
            return new MeshNormalUV {
                PrecisionPositions = points, Positions = converter.Convert(points),
                Triangles = new List<Tri>(Triangles), TrianglesEx = corners,
            };
        }


       
        public MeshNormalUV(CoordinateConverter converter, List<Vec3D> position, List<Tri> triangles, 
            List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData, List<int> perTriangleGroup, bool skipWatertightCheck = false)
        {
            if (!skipWatertightCheck && !MeshAnalysis.IsWatertightMesh(position, triangles))
                throw new Exception();

            // Create a dictionary to map unique positions to new vertex indices
            Dictionary<Vec3D, int> positionToIndex = new Dictionary<Vec3D, int>();
            List<Vec3D> uniquePositions = new List<Vec3D>();
            this.PrecisionPositions = new List<Rat3Hybrid>();

            // Deduplicate positions
            for (int i = 0; i < position.Count; i++)
            {
                Vec3D pos = position[i];
                if (!positionToIndex.ContainsKey(pos))
                {
                    positionToIndex[pos] = uniquePositions.Count;
                    uniquePositions.Add(pos);

                    var intPos = converter.Convert(pos);
                    PrecisionPositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
            }

            // Create mapping from original vertex indices to deduplicated indices
            int[] vertexMapping = new int[position.Count];
            for (int i = 0; i < position.Count; i++)
            {
                vertexMapping[i] = positionToIndex[position[i]];
            }

            // Build the result mesh
            this.Positions = uniquePositions;
            this.Triangles = new List<Tri>();
            this.TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>();

            // Process each triangle
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri originalTriangle = triangles[i];

                // Map to deduplicated vertex indices
                Tri newTriangle = new Tri
                {
                    A = vertexMapping[originalTriangle.A],
                    B = vertexMapping[originalTriangle.B],
                    C = vertexMapping[originalTriangle.C]
                };

                // Use the provided triangle corner data
                MeshTriangle<TriangleVertexNormalUV> extTriangle = triangleCornerData[i];
                
                // Set group ID from the provided array
                extTriangle.GroupId = perTriangleGroup[i];

                // Add to result
                this.Triangles.Add(newTriangle);
                this.TrianglesEx.Add(extTriangle);
            }

            if (!skipWatertightCheck && !MeshAnalysis.IsWatertightMesh(this.Positions, this.Triangles))
                throw new Exception();
            if (!skipWatertightCheck && !MeshAnalysis.AreTrianglesConsistentlyOriented(this.PrecisionPositions, this.Triangles))
                throw new Exception();
        }

        public MeshNormalUV(CoordinateConverter converter, List<Rat3Hybrid> position, List<Tri> triangles,
            List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData, List<int> perTriangleGroup)
        {
            // Create a dictionary to map unique positions to new vertex indices
            Dictionary<Rat3Hybrid, int> positionToIndex = new Dictionary<Rat3Hybrid, int>();
            List<Vec3D> uniquePositions = new List<Vec3D>();
            this.PrecisionPositions = new List<Rat3Hybrid>();

            // Deduplicate positions
            for (int i = 0; i < position.Count; i++)
            {
                Rat3Hybrid pos = position[i];
                pos.Simplify();
                position[i] = pos;
                if (!positionToIndex.ContainsKey(pos))
                {
                    positionToIndex[pos] = uniquePositions.Count;
                    PrecisionPositions.Add(pos);

                    var intPos = converter.Convert(pos);
                    uniquePositions.Add(converter.Convert(pos));
                }
            }

            // Create mapping from original vertex indices to deduplicated indices
            int[] vertexMapping = new int[position.Count];
            for (int i = 0; i < position.Count; i++)
            {
                vertexMapping[i] = positionToIndex[position[i]];
            }

            // Build the result mesh
            this.Positions = uniquePositions;
            this.Triangles = new List<Tri>();
            this.TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>();

            // Process each triangle
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri originalTriangle = triangles[i];

                // Map to deduplicated vertex indices
                Tri newTriangle = new Tri
                {
                    A = vertexMapping[originalTriangle.A],
                    B = vertexMapping[originalTriangle.B],
                    C = vertexMapping[originalTriangle.C]
                };

                if (newTriangle.ContainsDuplicateIndex())
                    continue;

                // Use the provided triangle corner data
                MeshTriangle<TriangleVertexNormalUV> extTriangle = triangleCornerData[i];

                // Set group ID from the provided array
                extTriangle.GroupId = perTriangleGroup[i];

                // Add to result
                this.Triangles.Add(newTriangle);
                this.TrianglesEx.Add(extTriangle);
            }

            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(this.PrecisionPositions, this.Triangles))
                throw new Exception();
        }

        public void RunSanityChecks()
        {
            //if (!MeshAnalysis.IsWatertightMesh(this.Triangles))
            //    throw new Exception();
            if (!MeshAnalysis.IsWatertightMesh(this.PrecisionPositions, this.Triangles))
            {
                MeshAnalysis.IsWatertightMesh(this.Positions, this.Triangles, out var problem);
                Trace.Write(TraceCommand.Clear);
                Trace.Write(new Tuple<string, Vec3D, List<Vec3D>>("edges", new Vec3D(1, 0, 0), problem));               
                Trace.Write(new Tuple<string, Vec3D, MeshNormalUV>("volume", new Vec3D(0, 0, 0.7), this));
                Trace.Write(TraceCommand.Fit);
                Trace.Write(TraceCommand.Hold);
                throw new Exception();
            }
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(this.PrecisionPositions, this.Triangles))
                throw new Exception();
        }

        // Overload for backward compatibility - converts Vec3D to Rat3Hybrid internally
        public MeshNormalUV(CoordinateConverter converter, List<Vec3D> position, List<Vec3D> normals, List<Vec2D> uv, 
            List<Tri> triangles, List<int> perTriangleGroup, bool skipWatertightCheck = false)
            : this(converter, position, normals, uv, triangles, perTriangleGroup, null, skipWatertightCheck)
        {
        }

        // Constructor that accepts pre-computed precise positions to avoid duplicate conversion
        public MeshNormalUV(CoordinateConverter converter, List<Vec3D> position, List<Vec3D> normals, List<Vec2D> uv, 
            List<Tri> triangles, List<int> perTriangleGroup, List<Rat3Hybrid> precisePositions, bool skipWatertightCheck = false)
        {
            bool completeExactPositions = precisePositions != null && precisePositions.Count == position.Count;
            if (!skipWatertightCheck && !(completeExactPositions
                ? MeshAnalysis.IsWatertightMesh(precisePositions, triangles)
                : MeshAnalysis.IsWatertightMesh(position, triangles)))
                throw new InvalidOperationException("Input mesh is not watertight in its authoritative coordinates.");

            // Exact positions define topology. Display coordinates can differ
            // for one lattice point, or coincide for distinct rational points.
            var positionToIndex = new Dictionary<Rat3Hybrid, int>();
            var uniquePositions = new List<Vec3D>();
            PrecisionPositions = new List<Rat3Hybrid>();
            int[] vertexMapping = new int[position.Count];
            for (int i = 0; i < position.Count; i++)
            {
                Rat3Hybrid exact;
                if (precisePositions != null && i < precisePositions.Count)
                {
                    var source = precisePositions[i];
                    exact = new Rat3Hybrid(in source);
                }
                else
                {
                    var point = converter.Convert(position[i]);
                    exact = new Rat3Hybrid(point.X, point.Y, point.Z);
                }
                exact.Simplify();
                if (!positionToIndex.TryGetValue(exact, out int index))
                {
                    index = uniquePositions.Count;
                    positionToIndex.Add(exact, index);
                    uniquePositions.Add(position[i]);
                    PrecisionPositions.Add(exact);
                }
                vertexMapping[i] = index;
            }

            // Build the result mesh
            //MeshNormalUV result = new MeshNormalUV();
            // For backward compatibility with old signature
            this.Positions = uniquePositions;
            this.Triangles = new List<Tri>();
            //this.TriangleMarker = new List<int>();
            this.TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>();

            // Process each triangle
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri originalTriangle = triangles[i];

                // Map to deduplicated vertex indices
                Tri newTriangle = new Tri
                {
                    A = vertexMapping[originalTriangle.A],
                    B = vertexMapping[originalTriangle.B],
                    C = vertexMapping[originalTriangle.C]
                };
                if (newTriangle.ContainsDuplicateIndex())
                    continue;

                // Create extended triangle with vertex attributes
                MeshTriangle<TriangleVertexNormalUV> extTriangle = new MeshTriangle<TriangleVertexNormalUV>();

                // Set vertex attributes from input arrays
                extTriangle.V0 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.A],
                    UV = uv[originalTriangle.A]
                };

                extTriangle.V1 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.B],
                    UV = uv[originalTriangle.B]
                };

                extTriangle.V2 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.C],
                    UV = uv[originalTriangle.C]
                };

                // Set group ID
                extTriangle.GroupId = perTriangleGroup[i];

                // Add to result
                this.Triangles.Add(newTriangle);
                //this.TriangleMarker.Add(MeshFactory.GetUniqueTriangleId());
                this.TrianglesEx.Add(extTriangle);
            }

#if DEBUG
            if (!skipWatertightCheck && !MeshAnalysis.IsWatertightMesh(this.PrecisionPositions, this.Triangles))
                throw new Exception();
            if (!skipWatertightCheck && !MeshAnalysis.AreTrianglesConsistentlyOriented(this.PrecisionPositions, this.Triangles))
                throw new Exception();
#endif
        }
        public MeshNormalUV(List<Rat3Hybrid> positionPrecise, List<Vec3D> position, List<Vec3D> normals, List<Vec2D> uv,
            List<Tri> triangles, List<int> perTriangleGroup)
        {
            this.Positions = position;
            this.PrecisionPositions = positionPrecise;


            this.Triangles = triangles;
            //this.TriangleMarker = new List<int>();
            this.TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>();

            // Process each triangle
            for (int i = 0; i < triangles.Count; i++)
            {
                var originalTriangle = triangles[i];
                if (originalTriangle.ContainsDuplicateIndex())
                    continue;

                // Create extended triangle with vertex attributes
                MeshTriangle<TriangleVertexNormalUV> extTriangle = new MeshTriangle<TriangleVertexNormalUV>();

                // Set vertex attributes from input arrays
                extTriangle.V0 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.A],
                    UV = uv[originalTriangle.A]
                };

                extTriangle.V1 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.B],
                    UV = uv[originalTriangle.B]
                };

                extTriangle.V2 = new TriangleVertexNormalUV
                {
                    Normal = normals[originalTriangle.C],
                    UV = uv[originalTriangle.C]
                };

                // Set group ID
                extTriangle.GroupId = perTriangleGroup[i];

                this.TrianglesEx.Add(extTriangle);
            }

            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(this.PrecisionPositions, this.Triangles))
                throw new Exception();
        }

        public static MeshNormalUV BooleanOperation(MeshNormalUV a, MeshNormalUV b, BooleanOp op, 
            CoordinateConverter converter, List<List<IntersectionSegmentEx>> intersectionStrips = null,
            Action<BooleanFragments> classifiedFragments = null)
        {
            MeshNormalUV result = BooleanOperation<MeshNormalUV>(a, b, op, converter, intersectionStrips, classifiedFragments);
            for (int i = 0; i < result.Triangles.Count; i++)
            {
                Tri tri = result.Triangles[i];
                Vec3D geometricNormal = Vec3DOps.Cross(
                    result.Positions[tri.B] - result.Positions[tri.A],
                    result.Positions[tri.C] - result.Positions[tri.A]);
                MeshTriangle<TriangleVertexNormalUV> data = result.TrianglesEx[i];
                Vec3D averageNormal = data.V0.Normal + data.V1.Normal + data.V2.Normal;
                if (Vec3DOps.Dot(geometricNormal, averageNormal) < 0)
                {
                    data.V0.Normal = -data.V0.Normal;
                    data.V1.Normal = -data.V1.Normal;
                    data.V2.Normal = -data.V2.Normal;
                    result.TrianglesEx[i] = data;
                }
            }
            return result;
        }


        public void Decompose(
            out List<Vec3D> positions, out List<Vec3D> normals, out List<Vec2D> uvs,
            out List<Tri> triangles, out List<int> perTriangleGroup)
            => DecomposeCore(out positions, out normals, out uvs, out triangles, out perTriangleGroup, out _, false);

        internal void Decompose(
            out List<Vec3D> positions, out List<Vec3D> normals, out List<Vec2D> uvs,
            out List<Tri> triangles, out List<int> perTriangleGroup, out List<Rat3Hybrid> precisePositions)
            => DecomposeCore(out positions, out normals, out uvs, out triangles, out perTriangleGroup, out precisePositions, true);

        private void DecomposeCore(
            out List<Vec3D> positions, out List<Vec3D> normals, out List<Vec2D> uvs,
            out List<Tri> triangles, out List<int> perTriangleGroup, out List<Rat3Hybrid> precisePositions, bool includePrecise)
        {
            if (includePrecise && (PrecisionPositions == null || PrecisionPositions.Count != Positions.Count))
                throw new InvalidOperationException("Exact decomposition requires one authoritative coordinate per source vertex.");
            // UV seams may duplicate a source vertex. Equal display coordinates
            // cannot establish source identity: distinct rational points can round
            // to the same double, particularly after oblique intersections.
            var vertexToIndex = new Dictionary<(int Source, TriangleVertexNormalUV Payload), int>();
            var outputPositions = new List<Vec3D>();
            var outputPrecise = includePrecise ? new List<Rat3Hybrid>() : null;
            var outputNormals = new List<Vec3D>();
            var outputUvs = new List<Vec2D>();
            int Vertex(int source, TriangleVertexNormalUV payload)
            {
                var key = (source, payload);
                if (vertexToIndex.TryGetValue(key, out int existing)) return existing;
                int index = outputPositions.Count;
                outputPositions.Add(Positions[source]);
                if (includePrecise) outputPrecise.Add(PrecisionPositions[source]);
                outputNormals.Add(payload.Normal);
                outputUvs.Add(payload.UV);
                vertexToIndex.Add(key, index);
                return index;
            }

            triangles = new List<Tri>(Triangles.Count);
            perTriangleGroup = new List<int>(Triangles.Count);
            for (int i = 0; i < Triangles.Count; ++i)
            {
                var triangle = Triangles[i];
                var data = TrianglesEx[i];
                triangles.Add(new Tri(Vertex(triangle.A, data.V0), Vertex(triangle.B, data.V1), Vertex(triangle.C, data.V2)));
                perTriangleGroup.Add(data.GroupId);
            }
            positions = outputPositions;
            precisePositions = outputPrecise;
            normals = outputNormals;
            uvs = outputUvs;
        }

        public void SaveOff(string fileName)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                writer.WriteLine("OFF");
                writer.WriteLine($"{Positions.Count} {Triangles.Count} 0");

                // Write vertices
                foreach (var pt in Positions)
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
    }
}
