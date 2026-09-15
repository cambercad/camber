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
            if (!MeshAnalysis.IsWatertightMesh(this.Positions, this.Triangles, out var problem))
            {
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

                    // Use pre-computed precise position if available, otherwise convert
                    if (precisePositions != null && i < precisePositions.Count)
                    {
                        PrecisionPositions.Add(precisePositions[i]);
                    }
                    else
                    {
                        var intPos = converter.Convert(pos);
                        PrecisionPositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                    }
                }
            }

            // Create mapping from original vertex indices to deduplicated indices
            int[] vertexMapping = new int[position.Count];
            for (int i = 0; i < position.Count; i++)
            {
                vertexMapping[i] = positionToIndex[position[i]];
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
            if (!skipWatertightCheck && !MeshAnalysis.IsWatertightMesh(this.Positions, this.Triangles))
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
            CoordinateConverter converter, List<List<IntersectionSegmentEx>> intersectionStrips = null) 
        {
            MeshNormalUV result = BooleanOperation<MeshNormalUV>(a, b, op, converter, intersectionStrips);
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


        private struct UVNormalVertex : IEquatable<UVNormalVertex>
        {
            public Vec3D Position;
            public TriangleVertexNormalUV Payload;

            public bool Equals(UVNormalVertex other)
            {
                return Position.X == other.Position.X && Position.Y == other.Position.Y && Position.Z == other.Position.Z &&
                       Payload.Normal.X == other.Payload.Normal.X && Payload.Normal.Y == other.Payload.Normal.Y && Payload.Normal.Z == other.Payload.Normal.Z &&
                       Payload.UV.X == other.Payload.UV.X && Payload.UV.Y == other.Payload.UV.Y;
            }

            public override bool Equals(object obj)
            {
                return obj is UVNormalVertex other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 23 + Position.X.GetHashCode();
                    hash = hash * 23 + Position.Y.GetHashCode();
                    hash = hash * 23 + Position.Z.GetHashCode();
                    hash = hash * 23 + Payload.Normal.X.GetHashCode();
                    hash = hash * 23 + Payload.Normal.Y.GetHashCode();
                    hash = hash * 23 + Payload.Normal.Z.GetHashCode();
                    hash = hash * 23 + Payload.UV.X.GetHashCode();
                    hash = hash * 23 + Payload.UV.Y.GetHashCode();
                    return hash;
                }
            }
        }



        public void Decompose(
           out List<Vec3D> positions, out List<Vec3D> normals, out List<Vec2D> uvs, out List<Tri> triangles, out List<int> perTriangleGroup)
        {
            Dictionary<UVNormalVertex, int> vertexToIndex = new Dictionary<UVNormalVertex, int>();

            // Initialize output lists
            positions = new List<Vec3D>();
            normals = new List<Vec3D>();
            uvs = new List<Vec2D>();
            triangles = new List<Tri>();
            perTriangleGroup = new List<int>();

            // Process each triangle in the mesh
            for (int i = 0; i < Triangles.Count; i++)
            {
                Tri meshTriangle = Triangles[i];
                MeshTriangle<TriangleVertexNormalUV> extTriangle = TrianglesEx[i];

                // Create vertices for each corner of the triangle
                UVNormalVertex vertex0 = new UVNormalVertex
                {
                    Position = Positions[meshTriangle.A],
                    Payload = extTriangle.V0
                };

                UVNormalVertex vertex1 = new UVNormalVertex
                {
                    Position = Positions[meshTriangle.B],
                    Payload = extTriangle.V1
                };

                UVNormalVertex vertex2 = new UVNormalVertex
                {
                    Position = Positions[meshTriangle.C],
                    Payload = extTriangle.V2
                };

                // Get or create indices for each vertex
                int index0 = GetOrCreateVertexIndex(vertex0, vertexToIndex, positions, normals, uvs);
                int index1 = GetOrCreateVertexIndex(vertex1, vertexToIndex, positions, normals, uvs);
                int index2 = GetOrCreateVertexIndex(vertex2, vertexToIndex, positions, normals, uvs);

                // Create the output triangle
                Tri outputTriangle = new Tri
                {
                    A = index0,
                    B = index1,
                    C = index2
                };

                triangles.Add(outputTriangle);
                perTriangleGroup.Add(extTriangle.GroupId);
            }
        }

        private static int GetOrCreateVertexIndex(UVNormalVertex vertex, Dictionary<UVNormalVertex, int> vertexToIndex,
           List<Vec3D> positions, List<Vec3D> normals, List<Vec2D> uvs)
        {
            if (vertexToIndex.TryGetValue(vertex, out int existingIndex))
            {
                return existingIndex;
            }

            // Create new vertex
            int newIndex = positions.Count;
            positions.Add(vertex.Position);
            normals.Add(vertex.Payload.Normal);
            uvs.Add(vertex.Payload.UV);
            vertexToIndex[vertex] = newIndex;

            return newIndex;
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
