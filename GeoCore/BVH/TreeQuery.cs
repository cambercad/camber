using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;
using System.Xml.Linq;

namespace GeoCore
{
    public class GenericTraversalController : ITraversalController
    {
        private Func<BVHNode, int, TraversalControl> _func;

        public GenericTraversalController(Func<BVHNode, int, TraversalControl> func)
        {
            _func = func;
        }

        public TraversalControl Analyze(BVHNode node, int index)
        {
            return _func(node, index);
        }
    }

    public interface IBoxProcessor
    {
        //Box3F GetBoundingBox();

        //Return true if the query should terminate immediately
        bool Process(int index);
    }

    public struct BoxOverlapTraversalController<T> : ITraversalController where T : struct, IBoxProcessor
    {
        public T Processor;
        public Box3F Bounds;

        public BoxOverlapTraversalController(T processor, Box3F bounds)
        {
            Processor = processor;
            Bounds = bounds;
        }

        public TraversalControl Analyze(BVHNode node, int index)
        {
            if (!Bounds.OverlapOrTouch(node.Bounds))
            {
                //Boxes overlap
                return TraversalControl.DontGoDeeper;
            }

            if (node.ChildA < 0)
            {
                if (Processor.Process(node.ChildB))
                    return TraversalControl.Abort;
                else
                    return TraversalControl.DontGoDeeper;
            }

            return TraversalControl.GoDeeper;
        }
    }

    //public unsafe struct TreeEnumerator
    //{
    //    private BVHNode* tree;
    //    private fixed int todoStack[64];       
    //    private Box3F Bounds;
    //    private int index;
    //    private int stackCount;

    //    public TreeEnumerator(BVHNode* tree, Box3F bounds, int rootIndex = 0)
    //    {
    //        this.tree = tree;
    //        this.index = rootIndex;
    //        //this.todoStack = new int[64];
    //        this.stackCount = 0;
    //        Bounds = bounds;
    //    }

    //    public int Next()
    //    {
    //        if (stackCount < 0)
    //            return -1;

    //        while (true)
    //        {
    //            BVHNode a = tree[index];

    //            if (Bounds.OverlapOrTouch(a.Bounds))
    //            {
    //                if (a.ChildA >= 0)
    //                {
    //                    //if (control == TraversalControl.GoDeeper)
    //                    //{
    //                        todoStack[stackCount] = a.ChildB;
    //                        index = a.ChildA;
    //                    //}
    //                    //else
    //                    //{
    //                    //    todoStack[stackCount] = a.ChildA;
    //                    //    index = a.ChildB;
    //                    //}

    //                    ++stackCount;

    //                    //if (stackCount == todoStack.Length)
    //                    //{
    //                    //    var newStack = new int[2 * todoStack.Length];
    //                    //    Array.Copy(todoStack, newStack, todoStack.Length);
    //                    //    todoStack = newStack;
    //                    //}
    //                    continue;
    //                }
    //                else
    //                {
    //                    --stackCount;
    //                    if (stackCount >= 0)
    //                        index = todoStack[stackCount];
    //                    return a.ChildB;
    //                }
    //            }
    //            if (stackCount == 0) break;
    //            --stackCount;
    //            index = todoStack[stackCount];
    //        }
    //        return -1;
    //    }
        
    //}

    public struct TreeEnumerator2
    {
        private int current;
        private List<int> Indices;

        public TreeEnumerator2(List<int> indices)
        {
            current = 0;
            Indices = indices;
        }

        public int Next()
        {
            if(current < Indices.Count)
            {
                var result = Indices[current];
                ++current;
                return result;
            }
            return -1;
        }
    }

    public struct TreeBoxOverlapEnumerator : IEnumerable<int>
    {
        public struct Processor : IBoxProcessor
        {
            public List<int> Indices;

            public bool Process(int index)
            {
                Indices.Add(index);
                return false;
            }
        }
        private BVHNode[] tree;
        public Box3F QueryBox { get; set; }

        public TreeBoxOverlapEnumerator(BVHNode[] tree, Box3F queryBox)
        {
            this.tree = tree;
            this.QueryBox = queryBox;
        }

        public List<int> GetList()
        {
            BoxOverlapTraversalController<Processor> controller = new BoxOverlapTraversalController<Processor>(new Processor(), QueryBox);
            controller.Processor.Indices = new List<int>();
            Tree.Traverse(tree, controller);
            return controller.Processor.Indices;
        }

        public IEnumerator<int> GetEnumerator()
        {
            BoxOverlapTraversalController<Processor> controller = new BoxOverlapTraversalController<Processor>(new Processor(), QueryBox);
            controller.Processor.Indices = new List<int>();
            Tree.Traverse(tree, controller);

            for (int i = 0; i < controller.Processor.Indices.Count; ++i)
                yield return controller.Processor.Indices[i];
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    public struct TreeMultiBoxOverlapEnumerator : IEnumerable<Int2>
    {
        public struct Processor : IMultiBoxProcessor
        {
            public List<Int2> Pairs;

            public bool Process(int indexA, int indexB)
            {
                Pairs.Add(new Int2(indexA, indexB));
                return false;
            }
        }
        private BVHNode[] treeA;
        private BVHNode[] treeB;

        public TreeMultiBoxOverlapEnumerator(BVHNode[] treeA, BVHNode[] treeB)
        {
            this.treeA = treeA;
            this.treeB = treeB;
        }

        public List<Int2> GetList()
        {
            MultiBoxOverlapTraversalController<Processor> controller = new MultiBoxOverlapTraversalController<Processor>(new Processor());
            controller.Processor.Pairs = new List<Int2>();
            Tree.Traverse(treeA, treeB, controller);
            return controller.Processor.Pairs;
        }

        public IEnumerator<Int2> GetEnumerator()
        {
            MultiBoxOverlapTraversalController<Processor> controller = new MultiBoxOverlapTraversalController<Processor>(new Processor());
            controller.Processor.Pairs = new List<Int2>();

            Tree.Traverse(treeA, treeB, controller);

            for (int i = 0; i < controller.Processor.Pairs.Count; ++i)
                yield return controller.Processor.Pairs[i];
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }



    public interface IMultiBoxProcessor
    {
        //Return true if the query should terminate immediately
        bool Process(int indexA, int indexB);
    }

    public struct MultiBoxOverlapTraversalController<T> : IMultiTraversalController where T : struct, IMultiBoxProcessor
    {
        public T Processor;

        public MultiBoxOverlapTraversalController(T processor)
        {
            Processor = processor;
        }

        public MultiTraversalControl Analyze(BVHNode nodeX, int indexX, BVHNode nodeY, int indexY)
        {
            if (!nodeX.Bounds.OverlapOrTouch(nodeY.Bounds))
                return MultiTraversalControl.DontGoDeeper;

            if (nodeX.ChildA < 0 && nodeY.ChildA < 0)
            {
                if (Processor.Process(nodeX.ChildB, nodeY.ChildB))
                    return MultiTraversalControl.Abort;
                else
                    return MultiTraversalControl.DontGoDeeper;
            }

            if (nodeX.ChildA < 0)
                return MultiTraversalControl.GoDeeperYSubBoxA;

            if (nodeY.ChildA < 0)
                return MultiTraversalControl.GoDeeperXSubBoxA;

            //"Descend larger" rule
            if (nodeX.ComputeVolume() > nodeY.ComputeVolume())
                return MultiTraversalControl.GoDeeperXSubBoxA;
            else
                return MultiTraversalControl.GoDeeperYSubBoxA;
        }
    }


    public class ClosestDistanceToTrimeshTraversalController : ITraversalController
    {
        public double ClosestDistanceSquared = double.MaxValue;
        private IList<Tri> triangles;
        private IList<Vec3D> points;
        private IList<BVHNode> nodes;
        private Vec3D queryPoint;
        private float queryPointX, queryPointY, queryPointZ; //Store the float values to a void a lot of type casts
        private double thresholdDistanceSquared; //Allows for early out if only distances below a certain threshold are relevant

        public bool Success { get { return ClosestDistanceSquared < double.MaxValue; } } //Can be false if no point with a shorter distance to the query point than thresholdDistanceSquared was found

        private Vec3D closestPoint; public Vec3D ClosestPoint { get { return closestPoint; } }
        private int closestTriId;

        public ClosestDistanceToTrimeshTraversalController(IList<Tri> triangles, IList<Vec3D> points, IList<BVHNode> nodes, Vec3D queryPoint, double thresholdDistance = double.NaN)
        {
            this.triangles = triangles;
            this.points = points;
            this.nodes = nodes;
            this.queryPoint = queryPoint;
            queryPointX = (float)queryPoint.X;
            queryPointY = (float)queryPoint.Y;
            queryPointZ = (float)queryPoint.Z;

            if (double.IsNaN(thresholdDistance))
                this.thresholdDistanceSquared = double.MaxValue;
            else
                this.thresholdDistanceSquared = thresholdDistance * thresholdDistance;
        }

        public void Update(Vec3D queryPoint)
        {
            this.queryPoint = queryPoint;
            queryPointX = (float)queryPoint.X;
            queryPointY = (float)queryPoint.Y;
            queryPointZ = (float)queryPoint.Z;
            ClosestDistanceSquared = double.MaxValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float AABBDistanceSquaredToPoint(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, float pX, float pY, float pZ)
        {
            float sqDist = 0.0f;

            if (pX < minX) sqDist += (minX - pX) * (minX - pX);
            if (pX > maxX) sqDist += (pX - maxX) * (pX - maxX);

            if (pY < minY) sqDist += (minY - pY) * (minY - pY);
            if (pY > maxY) sqDist += (pY - maxY) * (pY - maxY);

            if (pZ < minZ) sqDist += (minZ - pZ) * (minZ - pZ);
            if (pZ > maxZ) sqDist += (pZ - maxZ) * (pZ - maxZ);

            return sqDist;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float AABBDistanceSquaredToPoint(Box3F box, float pX, float pY, float pZ)
        {
            float sqDist = 0.0f;

            if (pX < box.Min.X) sqDist += (box.Min.X - pX) * (box.Min.X - pX);
            if (pX > box.Max.X) sqDist += (pX - box.Max.X) * (pX - box.Max.X);

            if (pY < box.Min.Y) sqDist += (box.Min.Y - pY) * (box.Min.Y - pY);
            if (pY > box.Max.Y) sqDist += (pY - box.Max.Y) * (pY - box.Max.Y);

            if (pZ < box.Min.Z) sqDist += (box.Min.Z - pZ) * (box.Min.Z - pZ);
            if (pZ > box.Max.Z) sqDist += (pZ - box.Max.Z) * (pZ - box.Max.Z);

            return sqDist;
        }

        public TraversalControl Analyze(BVHNode node, int index)
        {
            if (AABBDistanceSquaredToPoint(node.Bounds, queryPointX, queryPointY, queryPointZ) >= Math.Min(thresholdDistanceSquared, ClosestDistanceSquared))
                return TraversalControl.DontGoDeeper;

            if (node.ChildA < 0)
            {
                var tri = triangles[node.ChildB];
                double u, v, w;
                var d2 = GeometricAlgorithms.DistancePointTriangleSquared(queryPoint, points[tri.A], points[tri.B], points[tri.C], out u, out v, out w);
                if (d2 < ClosestDistanceSquared)
                {
                    ClosestDistanceSquared = d2;
                    closestTriId = node.ChildB;
                    closestPoint = u * points[tri.A] + v * points[tri.B] + w * points[tri.C];
                }
                return TraversalControl.DontGoDeeper;
            }

            BVHNode nodeA = nodes[node.ChildA];
            float distSquaredA = AABBDistanceSquaredToPoint(nodeA.Bounds, queryPointX, queryPointY, queryPointZ);
            BVHNode nodeB = nodes[node.ChildB];
            float distSquaredB = AABBDistanceSquaredToPoint(nodeB.Bounds, queryPointX, queryPointY, queryPointZ);

            if (distSquaredA < distSquaredB)
            {
                if (distSquaredA < ClosestDistanceSquared)
                    return TraversalControl.GoDeeper;
            }
            else
            {
                if (distSquaredB < ClosestDistanceSquared)
                    return TraversalControl.GoDeeperBFirst;
            }
            return TraversalControl.DontGoDeeper;
        }
    }

}
