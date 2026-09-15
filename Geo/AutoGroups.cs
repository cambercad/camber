using GeoCore;
using System.Linq;

namespace Geo
{

    public static class AutoGroups
    {
        // Returns a group id per triangle
        public static int AutoDetectPatches(List<Vec3D> pos, List<Tri> tris,
            double neighbouringTriangleAngleThresholdInRadians, out List<int> groupPerTriangle, int baseGroupIndex, bool ensureWatertightness = false)
        {
            //var pos = geo.GetChannel<Vec3D>(Channel.Position).GetList();
            //var tris = geo.Triangles.GetList();
            var duplicateMap = DuplicatePointRemover.DuplicateMap(pos);
            List<Tri> mappedTris = DuplicatePointRemover.MapTriangles(tris, duplicateMap);

            //bool watertight = Watertight.IsWatertight(mappedTris); // duplicateMap, tris);

            //if (!watertight)
            //    throw new Exception();

            int numTris = mappedTris.Count;
            groupPerTriangle = new List<int>(numTris);
            if (numTris == 0)
                return 0;

            Vec3D[] triNormals = new Vec3D[numTris];
            //TODO: Ensure consistent triangle orientation
            for (int i = 0; i < numTris; ++i)
            {
                var tri = mappedTris[i];
                triNormals[i] = GeometricAlgorithms.ComputeTriangleNormal(pos[tri.A], pos[tri.B], pos[tri.C]);
            }


            TriangleEdge[] edges;
            var adjacency = Adjacency.BuildAdjacencyInformation(mappedTris, out edges);

            if (ensureWatertightness)
            {
                for (int i = 0; i < edges.Length; ++i)
                    if (edges[i].NeighbourIndex2 < 0)
                        throw new Exception("Mesh is not watertight");
            }

            // Flood fill across dihedral-smooth interior edges. Boundary edges (open sheets /
            // non-watertight meshes) simply have no neighbour and stop the flood — that is fine.
            for (int i = 0; i < numTris; ++i)
                groupPerTriangle.Add(-1);

            Stack<int> stack = new Stack<int>();
            stack.Push(0);
            int currentGroupId = 0;
            groupPerTriangle[0] = currentGroupId + baseGroupIndex;
            int searchStartIndex = 1;
            //List<int> group = new List<int>() { 0 };
            while (true)
            {
                if (stack.Count == 0)
                {
                    //if (group.Count > 0)
                    //    result.Add(group);
                    //group = new List<int>();
                    ++currentGroupId;
                    while (searchStartIndex < groupPerTriangle.Count)
                    {
                        if (groupPerTriangle[searchStartIndex] == -1)
                        {
                            //group.Add(searchStartIndex);
                            stack.Push(searchStartIndex);
                            groupPerTriangle[searchStartIndex] = currentGroupId + baseGroupIndex;
                            ++searchStartIndex;
                            break;
                        }
                        ++searchStartIndex;
                    }
                    if (stack.Count == 0)
                        break;
                }

                int index = stack.Pop();
                var n = triNormals[index];
                var adj = adjacency[index];

                if (adj.NeighbourAB >= 0 && groupPerTriangle[adj.NeighbourAB] == -1 && Vec3DOps.Angle(n, triNormals[adj.NeighbourAB]) < neighbouringTriangleAngleThresholdInRadians)
                {
                    stack.Push(adj.NeighbourAB);
                    //group.Add(adj.NeighbourAB);
                    groupPerTriangle[adj.NeighbourAB] = currentGroupId + baseGroupIndex;
                }
                if (adj.NeighbourBC >= 0 && groupPerTriangle[adj.NeighbourBC] == -1 && Vec3DOps.Angle(n, triNormals[adj.NeighbourBC]) < neighbouringTriangleAngleThresholdInRadians)
                {
                    stack.Push(adj.NeighbourBC);
                    //group.Add(adj.NeighbourBC);
                    groupPerTriangle[adj.NeighbourBC] = currentGroupId + baseGroupIndex;
                }
                if (adj.NeighbourCA >= 0 && groupPerTriangle[adj.NeighbourCA] == -1 && Vec3DOps.Angle(n, triNormals[adj.NeighbourCA]) < neighbouringTriangleAngleThresholdInRadians)
                {
                    stack.Push(adj.NeighbourCA);
                    //group.Add(adj.NeighbourCA);
                    groupPerTriangle[adj.NeighbourCA] = currentGroupId + baseGroupIndex;
                }
            }

            return currentGroupId;

            //if (group.Count > 0)
            //    result.Add(group);


            //Sort triangles and channels such that ranges index into contigous memory blocks

            //sortedTris = new List<Tri>(mappedTris.Length);
            //sortedPoints = new List<int>(pos.Count);
            //List<GeometryRange> patchRanges = new List<GeometryRange>(result.Count);
            //int[] posMap = new int[pos.Count];
            //for (int i = 0; i < result.Count; ++i)
            //{
            //    for (int j = 0; j < posMap.Length; ++j)
            //        posMap[j] = -1;

            //    var g = result[i];
            //    for (int j = 0; j < g.Count; ++j)
            //    {
            //        var tri = mappedTris[g[j]];

            //        if (posMap[tri.A] < 0)
            //        {
            //            posMap[tri.A] = sortedPoints.Count;
            //            sortedPoints.Add(tri.A);
            //            tri.A = sortedPoints.Count - 1;
            //        }
            //        else
            //        {
            //            tri.A = posMap[tri.A];
            //        }

            //        if (posMap[tri.B] < 0)
            //        {
            //            posMap[tri.B] = sortedPoints.Count;
            //            sortedPoints.Add(tri.B);
            //            tri.B = sortedPoints.Count - 1;
            //        }
            //        else
            //        {
            //            tri.B = posMap[tri.B];
            //        }

            //        if (posMap[tri.C] < 0)
            //        {
            //            posMap[tri.C] = sortedPoints.Count;
            //            sortedPoints.Add(tri.C);
            //            tri.C = sortedPoints.Count - 1;
            //        }
            //        else
            //        {
            //            tri.C = posMap[tri.C];
            //        }

            //        sortedTris.Add(tri);
            //    }
            //    patchRanges.Add(new GeometryRange(sortedPoints.Count, sortedTris.Count));
            //}
            //if (tris.Count != sortedTris.Count)
            //    throw new Exception();


            //return patchRanges;
        }
    }
}
