using GeoCore;

namespace Geo
{
    // Flood filler to estimate a cylindrical region given a seed point on a triangle mesh
    public static class AutoCylinder
    {
        public static bool FindCylinder(List<Vec3D> pos, List<Tri> tris, int seedTriangle, out Vec3D center, out Vec3D axis,
            /*SceneNode tg,*/ double neighbouringTriangleAngleThresholdInRadians = 40.0 * Math.PI / 180.0)
        {
            center = new Vec3D(0);
            axis = new Vec3D(0);

            var duplicateMap = DuplicatePointRemover.DuplicateMap(pos);
            var mappedTris = DuplicatePointRemover.MapTriangles(tris, duplicateMap);

            int numTris = mappedTris.Count;
            Vec3D[] triNormals = new Vec3D[numTris];
            for (int i = 0; i < numTris; ++i)
            {
                var tri = mappedTris[i];
                triNormals[i] = GeometricAlgorithms.ComputeTriangleNormal(pos[tri.A], pos[tri.B], pos[tri.C]);
            }

            TriangleEdge[] edges;
            var adjacency = Adjacency.BuildAdjacencyInformation(mappedTris, out edges);





            double minAngle = neighbouringTriangleAngleThresholdInRadians * 0.01;
            double maxAngle = neighbouringTriangleAngleThresholdInRadians;

            double maxAbsAngle = Math.Max(Math.Abs(minAngle), Math.Abs(maxAngle));

            double perpendicularThreshold = 1e-2;
            double errorThreshold = 1e-2;



            AxisCandidate best = null;
            for (int side = 0; side < 2; ++side)
            {

                bool[] done = new bool[numTris];
                for (int i = 0; i < numTris; ++i)
                    done[i] = false;

                Stack<int> stack = new Stack<int>();
                stack.Push(seedTriangle);
                done[seedTriangle] = true;



                if (side == 1)
                    Algorithms.Swap(ref minAngle, ref maxAngle);


                List<AxisCandidate> candidates = new List<AxisCandidate>();
                var seedAdj = adjacency[seedTriangle];
                Vec3D ax;
                if (seedAdj.NeighbourAB >= 0)
                {
                    if (GetAxis(seedTriangle, seedAdj.NeighbourAB, mappedTris, pos, triNormals, out ax))
                    {
                        candidates.Add(new AxisCandidate(ax, seedTriangle, -minAngle, maxAngle));
                        //candidates.Add(new AxisCandidate(ax, -maxAngle, minAngle));
                    }
                }
                if (seedAdj.NeighbourBC >= 0)
                {
                    if (GetAxis(seedTriangle, seedAdj.NeighbourBC, mappedTris, pos, triNormals, out ax))
                    {
                        candidates.Add(new AxisCandidate(ax, seedTriangle, -minAngle, maxAngle));
                        //candidates.Add(new AxisCandidate(ax, -maxAngle, minAngle));
                    }
                }
                if (seedAdj.NeighbourCA >= 0)
                {
                    if (GetAxis(seedTriangle, seedAdj.NeighbourCA, mappedTris, pos, triNormals, out ax))
                    {
                        candidates.Add(new AxisCandidate(ax, seedTriangle, -minAngle, maxAngle));
                        //candidates.Add(new AxisCandidate(ax, seedTriangle, -maxAngle, minAngle));
                    }
                }

                var seedTri = tris[seedTriangle];
                candidates.Add(new AxisCandidate((pos[seedTri.B] - pos[seedTri.A]).Normalized(), seedTriangle, -minAngle, maxAngle));
                //candidates.Add(new AxisCandidate((pos[seedTri.B] - pos[seedTri.A]).Normalized(), -maxAngle, minAngle));
                candidates.Add(new AxisCandidate((pos[seedTri.C] - pos[seedTri.B]).Normalized(), seedTriangle, -minAngle, maxAngle));
                //candidates.Add(new AxisCandidate((pos[seedTri.C] - pos[seedTri.B]).Normalized(), -maxAngle, minAngle));
                candidates.Add(new AxisCandidate((pos[seedTri.A] - pos[seedTri.C]).Normalized(), seedTriangle, -minAngle, maxAngle));
                //candidates.Add(new AxisCandidate((pos[seedTri.A] - pos[seedTri.C]).Normalized(), -maxAngle, minAngle));

                //if (candidates.Count == 0)
                //    return false;


                while (stack.Count > 0)
                {
                    int index = stack.Pop();
                    var n = triNormals[index];
                    var adj = adjacency[index];

                    double signedAngle = SignedAngle(index, adj.NeighbourAB, triNormals, mappedTris, pos);
                    if (adj.NeighbourAB >= 0 && !done[adj.NeighbourAB] && ValueIsInRange(signedAngle, -maxAbsAngle, maxAbsAngle))// < neighbouringTriangleAngleThresholdInRadians && AreConvex(index, adj.NeighbourAB, mappedTris, pos))
                    {
                        if (AddToCandidate(candidates, adj.NeighbourAB, triNormals, signedAngle, perpendicularThreshold))
                            stack.Push(adj.NeighbourAB);
                        done[adj.NeighbourAB] = true;
                    }
                    signedAngle = SignedAngle(index, adj.NeighbourBC, triNormals, mappedTris, pos);
                    if (adj.NeighbourBC >= 0 && !done[adj.NeighbourBC] && ValueIsInRange(signedAngle, -maxAbsAngle, maxAbsAngle))// < neighbouringTriangleAngleThresholdInRadians && AreConvex(index, adj.NeighbourBC, mappedTris, pos))
                    {
                        if (AddToCandidate(candidates, adj.NeighbourBC, triNormals, signedAngle, perpendicularThreshold))
                            stack.Push(adj.NeighbourBC);
                        done[adj.NeighbourBC] = true;
                    }
                    signedAngle = SignedAngle(index, adj.NeighbourCA, triNormals, mappedTris, pos);
                    if (adj.NeighbourCA >= 0 && !done[adj.NeighbourCA] && ValueIsInRange(signedAngle, -maxAbsAngle, maxAbsAngle))// < neighbouringTriangleAngleThresholdInRadians && AreConvex(index, adj.NeighbourCA, mappedTris, pos))
                    {
                        if (AddToCandidate(candidates, adj.NeighbourCA, triNormals, signedAngle, perpendicularThreshold))
                            stack.Push(adj.NeighbourCA);
                        done[adj.NeighbourCA] = true;
                    }
                }

                //Now find a point on the cylinder axis
                AxisCandidate b = best != null ? best : candidates[0];
                for (int i = 0; i < candidates.Count; ++i)
                {
                    var c = candidates[i];
                    if (c.Members.Count > b.Members.Count)
                        b = c;
                }
                best = b;
            }

            if (best.Members.Count < 8)
                return false;



            Vec3D estimatedCenter = EstimatePointOnAxis(best.Axis, best.Members, pos, mappedTris, triNormals);


            double dampingFactor = 0.5f;
            List<double> convergenceHistoryDebug = new List<double>();

            int numCorrectionIterations = 200;
            double error = double.MaxValue;
            for (int iter = 0; iter < numCorrectionIterations; ++iter)
            {
                double averageDistance = 0;
                double min = double.MaxValue;
                double max = double.MinValue;
                for (int i = 0; i < best.Members.Count; ++i)
                {
                    var tri = mappedTris[best.Members[i]];
                    double dist = DistancePointTrianglePlane(tri, pos, estimatedCenter);
                    averageDistance += dist;
                    min = Math.Min(min, dist);
                    max = Math.Max(max, dist);
                }
                averageDistance /= best.Members.Count;

                error = max - min;
                convergenceHistoryDebug.Add(error);

                Vec3D correction = new Vec3D(0.0f);
                for (int i = 0; i < best.Members.Count; ++i)
                {
                    var tri = mappedTris[best.Members[i]];
                    double dist = DistancePointTrianglePlane(tri, pos, estimatedCenter);

                    correction += dampingFactor * (averageDistance - dist) * triNormals[best.Members[i]];
                }
                correction /= best.Members.Count;

                estimatedCenter += correction;
            }

            /*if (tg != null)
            {

                //tg.AddChannel<Vec3D>(pos, Channel.Position);
                List<Tri> t = new List<Tri>();
                for (int i = 0; i < best.Members.Count; ++i)
                {
                    var tri = mappedTris[best.Members[i]];
                    t.Add(tri);
                }
                //tg.AddTriangles(t);

                ////Build debug output
                //tg.ClearChannels();
                //tg.ApproximateNormals();

                PhongPTriangleGeometryDebug geo = new PhongPTriangleGeometryDebug(t, pos);

                tg.Add(geo);
            }*/


            if (error > errorThreshold)
                return false;


            center = estimatedCenter;
            axis = best.Axis;
            return true;
        }

        private static bool AddToCandidate(List<AxisCandidate> candidates, int triId, Vec3D[] triNormals, double signedAngle, double threshold = 1e-3)
        {
            bool result = false;
            for (int i = 0; i < candidates.Count; ++i)
            {
                var c = candidates[i];
                double dot = Vec3DOps.Dot(c.Axis, triNormals[triId]);
                if (Math.Abs(dot) < threshold && c.AngleIsInRange(signedAngle))
                {
                    c.Members.Add(triId);
                    result = true;
                }
            }
            return result;
        }

        private static bool GetAxis(int seedTriangle, int neighbourTri, IList<Tri> triangles, List<Vec3D> pos, Vec3D[] triNormals, out Vec3D axis)
        {
            var triN = triNormals[seedTriangle];
            var n = triNormals[neighbourTri];
            axis = Vec3DOps.Cross(triN, n);

            if (axis.LengthSquared() < 1e-20)
                return false;

            //Check for convexity
            if (!AreConvex(seedTriangle, neighbourTri, triangles, pos))
                return false;

            axis.Normalize();

            return true;
        }

        //TODO: Replace with SignedAngle
        private static bool AreConvex(int index, int neighbour, IList<Tri> tris, List<Vec3D> pos)
        {
            //return true;

            var tri1 = tris[index];
            var tri2 = tris[neighbour];

            List<int> list = new List<int>();
            if (tri2.Contains(tri1.A)) list.Add(tri1.A);
            if (tri2.Contains(tri1.B)) list.Add(tri1.B);
            if (tri2.Contains(tri1.C)) list.Add(tri1.C);

            int remaining = tri2.GetRemaining(list[0], list[1]);
            //return GeometricAlgorithms.Orient3D(pos[tri1.A], pos[tri1.B], pos[tri1.C], pos[remaining]) <= 0.0;

            double d = DistancePointTrianglePlane(tri1, pos, pos[remaining]);

            return d >= 0.0;
        }

       

        private class AxisCandidate
        {
            public Vec3D Axis;
            public List<int> Members;
            public double MinAngle;
            public double MaxAngle;

            public AxisCandidate(Vec3D axis, int seedMember, double minAngle, double maxAngle)
            {
                Axis = axis;
                Members = new List<int>();
                Members.Add(seedMember);
                MinAngle = minAngle;
                MaxAngle = maxAngle;
            }

            public bool AngleIsInRange(double angle)
            {
                //if (angle < MinAngle)
                //    return false;
                //if (angle > MaxAngle)
                //    return false;
                //return true;
                return ValueIsInRange(angle, MinAngle, MaxAngle);
            }

            public override string ToString()
            {
                return Axis.ToString() + " [" + Members.Count.ToString() + "] " + MinAngle.ToString() + " " + MaxAngle.ToString();
            }
        }

        private static bool ValueIsInRange(double value, double min, double max)
        {
            if (value < min)
                return false;
            if (value > max)
                return false;
            return true;
        }

        //Positive vor convex triangles, negative otherwise
        private static double SignedAngle(int index, int neighbour, IList<Vec3D> triNormals, IList<Tri> tris, List<Vec3D> pos)
        {
            double angle = Vec3DOps.Angle(triNormals[index], triNormals[neighbour]);

            var tri1 = tris[index];
            var tri2 = tris[neighbour];

            List<int> list = new List<int>();
            if (tri2.Contains(tri1.A)) list.Add(tri1.A);
            if (tri2.Contains(tri1.B)) list.Add(tri1.B);
            if (tri2.Contains(tri1.C)) list.Add(tri1.C);

            int remaining = tri2.GetRemaining(list[0], list[1]);

            double d = DistancePointTrianglePlane(tri1, pos, pos[remaining]);
            if (d < 0)
                angle = -angle;

            return angle;
        }


        private static double DistancePointTrianglePlane(Tri tri, List<Vec3D> pos, Vec3D estimatedCenter)
        {
            return GeometricAlgorithms.SignedDistancePointPlane(estimatedCenter,
                GeometricAlgorithms.ComputeTriangleNormal(pos[tri.A], pos[tri.B], pos[tri.C]), pos[tri.A]);

            //Plane p = new Plane(pos[tri.A], pos[tri.B], pos[tri.C]);
            //double dist = p.SignedDistance(estimatedCenter);

            ////if (dist < 0)
            ////    throw new Exception();

            //return dist;
        }

        private static Vec3D EstimatePointOnAxis(Vec3D axis, List<int> triIds, List<Vec3D> pos, IList<Tri> tris, Vec3D[] triNormals)
        {
            Vec3D estimatedCenter = new Vec3D(0);
            for (int i = 0; i < triIds.Count; ++i)
            {
                var tri = tris[triIds[i]];
                estimatedCenter += pos[tri.A];
                estimatedCenter += pos[tri.B];
                estimatedCenter += pos[tri.C];
            }
            estimatedCenter = estimatedCenter / (3 * triIds.Count);

            //Find two triangles with almost perpendicular normals
            int first = triIds[0];
            Vec3D n = triNormals[first];

            double minDot = double.MaxValue;
            int minDotId = -1;

            for (int i = 1; i < triIds.Count; ++i)
            {
                Vec3D n2 = triNormals[triIds[i]];
                double absDot = Math.Abs(Vec3DOps.Dot(n, n2));
                if (absDot < minDot)
                {
                    minDot = absDot;
                    minDotId = triIds[i];
                }
            }

            var center1 = GeometricAlgorithms.TriangleCenter(tris[first], pos);
            var center2 = GeometricAlgorithms.TriangleCenter(tris[minDotId], pos);
            var dir1 = triNormals[first];
            var dir2 = triNormals[minDotId];

            double s, t;
            if (!GeometricAlgorithms.ClosestPointsLineLine(center1, dir1, center2, dir2, 1e-8, out s, out t))
                throw new Exception();

            Vec3D p = 0.5 * ((center1 + s * dir1) + (center2 + t * dir2));

            estimatedCenter = GeometricAlgorithms.ProjectPointOntoLine(estimatedCenter, p, axis);

            return estimatedCenter;
        }
    }
}
