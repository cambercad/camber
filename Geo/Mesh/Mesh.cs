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
        protected static M BooleanOperation<M>(M a, M b, BooleanOp op, CoordinateConverter converter, List<List<IntersectionSegmentEx>> intersectionStrips = null, Action<BooleanFragments> classifiedFragments = null, bool retainExteriorTrimCaps = false) where M : Mesh<T>, new()
        {
            if (a == null || b == null)
                return null;


            var dbgSw = System.Diagnostics.Stopwatch.StartNew();
            M result = new M();
            Resolver.Resolve(op, a.PrecisionPositions, a.Triangles, b.PrecisionPositions, b.Triangles,
                out result.PrecisionPositions, out result.Triangles, out List<SourceTriangle> sources, null,
                intersectionStrips, classifiedFragments, retainExteriorTrimCaps);

            long dbgResolve = dbgSw.ElapsedMilliseconds;
            result.Positions = converter.Convert(result.PrecisionPositions);
            long dbgConvert = dbgSw.ElapsedMilliseconds;

            // Corner attributes of one result triangle depend only on its source
            // triangle, so runs of fragments from the same source triangle are
            // independent. A run shares a corner cache; isolated triangles need none.
            int numTriangles = result.Triangles.Count;
            var extTriangles = new MeshTriangle<T>[numTriangles];
            var runStarts = new List<int>();
            for (int i = 0; i < numTriangles; i++)
            {
                if (i == 0 || sources[i].MeshOrigin != sources[i - 1].MeshOrigin ||
                    sources[i].SourceTriangleIndex != sources[i - 1].SourceTriangleIndex)
                    runStarts.Add(i);
            }
            runStarts.Add(numTriangles);

            bool cacheCorners = typeof(T) == typeof(TriangleVertexNormalUV);
            System.Runtime.ExceptionServices.ExceptionDispatchInfo failure = null;
            ParallelEx.ForLoadBalanced(0, runStarts.Count - 1, run =>
            {
                if (Volatile.Read(ref failure) != null) return;
                try
                {
                    int first = runStarts[run], end = runStarts[run + 1];
                    var corners = cacheCorners && end - first > 1 ? new Dictionary<int, T>() : null;
                    SourceTriangle s = sources[first];
                    var source = s.MeshOrigin switch
                    {
                        MeshOrigin.MeshA => a,
                        MeshOrigin.MeshB => b,
                        _ => throw new InvalidOperationException(
                            $"Boolean result triangle has unknown source mesh origin '{s.MeshOrigin}'.")
                    };
                    var sourceExtTriangle = source.TrianglesEx[s.SourceTriangleIndex];

                    T Corner(int point)
                    {
                        if (corners != null && corners.TryGetValue(point, out var cached)) return cached;
                        var vertex = sourceExtTriangle.InterpolateExact(InterpolationHelpers.GetExactBarycentricWeights(
                            result.PrecisionPositions[point], source.Triangles[s.SourceTriangleIndex], source.PrecisionPositions));
                        corners?.Add(point, vertex);
                        return vertex;
                    }

                    for (int i = first; i < end; i++)
                    {
                        Tri triangle = result.Triangles[i];
                        MeshTriangle<T> resultExtTriangle;
                        resultExtTriangle.V0 = Corner(triangle.A);
                        resultExtTriangle.V1 = Corner(triangle.B);
                        resultExtTriangle.V2 = Corner(triangle.C);
                        resultExtTriangle.GroupId = sourceExtTriangle.GroupId;
                        extTriangles[i] = resultExtTriangle;
                    }
                }
                catch (Exception e)
                {
                    // Surface the original exception type, as the sequential loop did.
                    System.Threading.Interlocked.CompareExchange(ref failure,
                        System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(e), null);
                }
            });
            failure?.Throw();
            result.TrianglesEx = new List<MeshTriangle<T>>(extTriangles);
            if (Resolver.LogBooleanOps)
                Console.WriteLine($"  MeshBool: resolve={dbgResolve}ms convert={dbgConvert - dbgResolve}ms corners={dbgSw.ElapsedMilliseconds - dbgConvert}ms");

            return result;
        }
    }
}
