using CSG;
using GeoCore;

namespace Geo
{

    public interface ITriangleVertex<T> where T : ITriangleVertex<T>
    {
        public T Interpolate(in T second, in T third, in Vec3D barycentricWeights);
    }


    public struct MeshTriangle<T> where T : ITriangleVertex<T>
    {
        public T V0;
        public T V1;
        public T V2;
        public int GroupId;

        public T Interpolate(in Vec3D barycentricWeights)
        {
            return V0.Interpolate(in V1, in V2, in barycentricWeights);
        }

        internal T InterpolateExact(in Rat3Hybrid weights)
        {
            if (V0 is TriangleVertexNormalUV a && V1 is TriangleVertexNormalUV b && V2 is TriangleVertexNormalUV c)
                return (T)(object)a.InterpolateExact(b, c, weights);
            return Interpolate(new Vec3D(weights.X.ToDouble(), weights.Y.ToDouble(), weights.Z.ToDouble()));
        }

        public override string ToString()
        {
            return GroupId.ToString();
        }
    }


    public class Mesh<T> where T : ITriangleVertex<T>
    {
        public List<Vec3D> Positions;
        public List<Rat3Hybrid> PrecisionPositions; // They are in CoordinateConverter space
        public List<Tri> Triangles;
        public List<MeshTriangle<T>> TrianglesEx;


        public List<int> GetTriangleGroups()
        {
            List<int> result = new List<int>(TrianglesEx.Count);
            for (int i = 0; i < TrianglesEx.Count; ++i)
            {
                result.Add(TrianglesEx[i].GroupId);
            }
            return result;
        }

        public void SetTriangleGroups(List<int> newGroupIdPerTriangle)
        {
            if (newGroupIdPerTriangle.Count != TrianglesEx.Count)
                throw new Exception();

            for (int i = 0; i < TrianglesEx.Count; ++i)
            {
                var tmp = TrianglesEx[i];
                tmp.GroupId = newGroupIdPerTriangle[i];
                TrianglesEx[i] = tmp;
            }
        }

        public void ApplyTranslation(Vec3D translation, CoordinateConverter c)
        {
            var t = c.ConvertDirection(translation);
            var tt = new Rat3Hybrid(t.X, t.Y, t.Z);
            for (int i = 0; i < Positions.Count; ++i)
            {
                PrecisionPositions[i] += tt;
                Positions[i] = c.Convert(PrecisionPositions[i]);
            }
        }


        // https://github.com/BrunoLevy/geogram/discussions/230
        //https://github.com/BrunoLevy/geogram/discussions/261
        protected static M BooleanOperation<M>(M a, M b, BooleanOp op, CoordinateConverter converter, List<List<IntersectionSegmentEx>> intersectionStrips = null, Action<BooleanFragments> classifiedFragments = null) where M : Mesh<T>, new()
        {
            if (a == null || b == null)
                return null;


            M result = new M();
            Resolver.Resolve(op, a.PrecisionPositions, a.Triangles, b.PrecisionPositions, b.Triangles,
                out result.PrecisionPositions, out result.Triangles, out List<SourceTriangle> sources, null, intersectionStrips, classifiedFragments);

            result.TrianglesEx = new List<MeshTriangle<T>>();

            result.Positions = converter.Convert(result.PrecisionPositions);

            // We already have efficient lookup dictionaries created above

            // Process result triangles
            int numTriangles = result.Triangles.Count; // resultFacets.Length / 3;
            for (int i = 0; i < numTriangles; i++)
            {
                Tri triangle = result.Triangles[i];

                // Determine source of this triangle and create extended triangle data
                MeshTriangle<T> sourceExtTriangle;
                Tri sourceTriangle;
                List<Rat3Hybrid> sourcePositions;

                SourceTriangle s = sources[i];
                //int sourceTriangleId = -1;
                //if (resultTriangleIds != null && i < resultTriangleIds.Length)
                {
                    if (s.MeshOrigin == MeshOrigin.MeshA)
                    {
                        // Triangle comes from mesh A
                        sourceExtTriangle = a.TrianglesEx[s.SourceTriangleIndex];
                        sourceTriangle = a.Triangles[s.SourceTriangleIndex];
                        sourcePositions = a.PrecisionPositions;
                    }
                    else if (s.MeshOrigin == MeshOrigin.MeshB)
                    {
                        // Triangle comes from mesh B
                        sourceExtTriangle = b.TrianglesEx[s.SourceTriangleIndex];
                        sourceTriangle = b.Triangles[s.SourceTriangleIndex];
                        sourcePositions = b.PrecisionPositions;
                    }
                    else
                    {
                        throw new InvalidOperationException(
                            $"Boolean result triangle has unknown source mesh origin '{s.MeshOrigin}'.");
                    }
                }
                //else
                //{
                //    throw new Exception("Tracking info should always be available");
                //}

                MeshTriangle<T> resultExtTriangle;
                resultExtTriangle.V0 = sourceExtTriangle.InterpolateExact(InterpolationHelpers.GetExactBarycentricWeights(result.PrecisionPositions[triangle.A], sourceTriangle, sourcePositions));
                resultExtTriangle.V1 = sourceExtTriangle.InterpolateExact(InterpolationHelpers.GetExactBarycentricWeights(result.PrecisionPositions[triangle.B], sourceTriangle, sourcePositions));
                resultExtTriangle.V2 = sourceExtTriangle.InterpolateExact(InterpolationHelpers.GetExactBarycentricWeights(result.PrecisionPositions[triangle.C], sourceTriangle, sourcePositions));
                var debug = sourceExtTriangle.GroupId;
                //resultExtTriangle.GroupId = sourceTriangleId;// sourceExtTriangle.GroupId;
                resultExtTriangle.GroupId = sourceExtTriangle.GroupId;

                result.TrianglesEx.Add(resultExtTriangle);
            }

            return result;
        }
    }
}
