namespace GeoCore
{
    public struct BVHNode
    {
        public Box3F Bounds;

        public int ChildA;
        public int ChildB;

        public BVHNode(Box3F bounds, int childA, int childB)
        {
            Bounds = bounds;
            ChildA = childA;
            ChildB = childB;
        }

        public float ComputeVolume()
        {
            return (Bounds.Max.X - Bounds.Min.X) * (Bounds.Max.Y - Bounds.Min.Y) * (Bounds.Max.Z - Bounds.Min.Z);
        }

        public bool IsLeaveNode
        {
            get { return ChildA < 0; }
        }
    }

    public enum TraversalControl
    {
        DontGoDeeper,
        GoDeeper,
        GoDeeperBFirst,
        Abort
    }

    public interface ITraversalController
    {
        TraversalControl Analyze(BVHNode node, int index);
    }

    public enum MultiTraversalControl
    {
        DontGoDeeper,
        GoDeeperXSubBoxA,
        GoDeeperXSubBoxB,
        GoDeeperYSubBoxA,
        GoDeeperYSubBoxB,
        Abort
    }

    public interface IMultiTraversalController
    {
        MultiTraversalControl Analyze(BVHNode nodeX, int indexX, BVHNode nodeY, int indexY);
    }


    public struct Section
    {
        public int Start;
        public int End;

        public Section(int start, int end)
        {
            Start = start;
            End = end;
        }
    }

    public static class Tree
    {
        private struct Range
        {
            public int Start;
            public int End;
            public int ParentIndex;

            public Range(int start, int end, int parentIndex = -1)
            {
                Start = start;
                End = end;
                ParentIndex = parentIndex;
            }
        }


        //https://stackoverflow.com/questions/10439242/count-leading-zeroes-in-an-int32
        private static int Clz(int x)
        {
            const int numIntBits = sizeof(int) * 8; //compile time constant
                                                    //do the smearing
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            //count the ones
            x -= x >> 1 & 0x55555555;
            x = (x >> 2 & 0x33333333) + (x & 0x33333333);
            x = (x >> 4) + x & 0x0f0f0f0f;
            x += x >> 8;
            x += x >> 16;
            return numIntBits - (x & 0x0000003f); //subtract # of 1s from 32
        }

        private static int FindSplit(int[] sortedMortonCodes,
               int first, int last)
        {
            // Identical Morton codes => split the range in the middle.

            int firstCode = sortedMortonCodes[first];
            int lastCode = sortedMortonCodes[last];

            if (firstCode == lastCode)
                return (first + last) >> 1;

            // Calculate the number of highest bits that are the same
            // for all objects, using the count-leading-zeros intrinsic.

            int commonPrefix = Clz(firstCode ^ lastCode);

            // Use binary search to find where the next bit differs.
            // Specifically, we are looking for the highest object that
            // shares more than commonPrefix bits with the first one.

            int split = first; // initial guess
            int step = last - first;

            do
            {
                step = (step + 1) >> 1; // exponential decrease
                int newSplit = split + step; // proposed new position

                if (newSplit < last)
                {
                    int splitCode = sortedMortonCodes[newSplit];
                    int splitPrefix = Clz(firstCode ^ splitCode);
                    if (splitPrefix > commonPrefix)
                        split = newSplit; // accept proposal
                }
            }
            while (step > 1);

            return split;
        }

        //BVHNode generateHierarchy(int[] sortedMortonCodes,
        //                 int[] sortedObjectIDs,
        //                 int first,
        //                 int last)
        //{
        //    // Single object => create a leaf node.

        //    if (first == last)
        //        return LeafNode(sortedObjectIDs[first]);

        //    // Determine where to split the range.

        //    int split = FindSplit(sortedMortonCodes, first, last);

        //    // Process the resulting sub-ranges recursively.
        //    BVHNode childA = generateHierarchy(sortedMortonCodes, sortedObjectIDs,
        //                                     first, split);
        //    BVHNode childB = generateHierarchy(sortedMortonCodes, sortedObjectIDs,
        //                                     split + 1, last);
        //    return InternalNode(childA, childB);
        //}

        public static BVHNode[] BuildTreeFast(Func<int, Box3F> boxBuilder, int numBoxes)
        {
            return BuildTreeFastList(boxBuilder, numBoxes).ToArray();
        }

        //https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
        public static List<BVHNode> BuildTreeFastList(Func<int, Box3F> boxBuilder, int numBoxes)
        {
            List<BVHNode> tree = new List<BVHNode>((int)(2.1 * numBoxes));
            BVHNode[] leaveNodes = new BVHNode[numBoxes];

            Box3F boundingBox = Box3F.Empty;
            for (int i = 0; i < numBoxes; ++i)
            {
                Box3F box = boxBuilder(i);
                boundingBox.Extend(box);
                var node = new BVHNode(box, -1, i);
                leaveNodes[i] = node;
            }

            int[] mortonCodes = CalculateMortonCodes(leaveNodes, boundingBox);
            Array.Sort(mortonCodes, leaveNodes, 0, leaveNodes.Length);

            Stack<Range> todo = new Stack<Range>();
            todo.Push(new Range(0, numBoxes - 1));

            List<Range> todoLeaves = new List<Range>(numBoxes);

            while (todo.Count > 0)
            {
                Range r = todo.Pop();
                int start = r.Start;
                int end = r.End;
                if (start == end)
                {
                    todoLeaves.Add(r);

                    continue;
                }


                int split = FindSplit(mortonCodes, r.Start, r.End);

                if (r.ParentIndex >= 0)
                {
                    BVHNode parent = tree[r.ParentIndex];
                    if (parent.ChildA < 0)
                        parent.ChildA = tree.Count;
                    else
                        parent.ChildB = tree.Count;
                    tree[r.ParentIndex] = parent;
                }

                BVHNode aabb = new BVHNode() { ChildA = -1, ChildB = -1 };

                todo.Push(new Range(start, split, tree.Count));
                todo.Push(new Range(split + 1, end, tree.Count));

                tree.Add(aabb);
            }

            //This ensures that the leave nodes are at the end of the tree
            for (int i = 0; i < todoLeaves.Count; ++i)
            {
                Range r = todoLeaves[i];
                if (r.ParentIndex >= 0)
                {
                    BVHNode parent = tree[r.ParentIndex];
                    if (parent.ChildA < 0)
                        parent.ChildA = tree.Count;
                    else
                        parent.ChildB = tree.Count;
                    tree[r.ParentIndex] = parent;
                }

                tree.Add(leaveNodes[r.Start]);
            }

            //var result = tree.ToArray();

            Refit(tree);
            return tree;

            //return result;
        }


        ////Requires the leave node bounding boxes to be set
        //private static void Refit(IList<BVHNode> tree, int rootNodeId = 0)
        //{
        //    Stack<int> stack = new Stack<int>();
        //    stack.Push(rootNodeId);

        //    Stack<int> returnStack = new Stack<int>();

        //    while (stack.Count > 0)
        //    {
        //        rootNodeId = stack.Pop();

        //        if (rootNodeId >= 0)
        //        {
        //            var node = tree[rootNodeId];
        //            if (node.ChildA < 0)
        //            {
        //                returnStack.Push(rootNodeId);
        //                continue;
        //            }

        //            stack.Push(-rootNodeId - 1);
        //            stack.Push(node.ChildA);
        //            stack.Push(node.ChildB);
        //        }
        //        else
        //        {
        //            int a = returnStack.Pop();
        //            int b = returnStack.Pop();

        //            rootNodeId = -rootNodeId - 1;
        //            var node = tree[rootNodeId];
        //            node.Bounds = Merge(tree[a].Bounds, tree[b].Bounds);
        //            tree[rootNodeId] = node;

        //            returnStack.Push(rootNodeId);
        //        }
        //    }
        //}

        // The leaf nodes must already be set if getLeafNodeBox is null
        public static void Refit(IList<BVHNode> tree, Func<int, Box3F> getLeafNodeBox = null, int rootNodeId = 0)
        {
            Stack<int> stack = new Stack<int>();
            stack.Push(rootNodeId);

            Stack<int> returnStack = new Stack<int>();

            while (stack.Count > 0)
            {
                rootNodeId = stack.Pop();

                if (rootNodeId >= 0)
                {
                    var node = tree[rootNodeId];
                    if (node.ChildA < 0)
                    {
                        if (getLeafNodeBox != null)
                        {
                            node.Bounds = getLeafNodeBox(node.ChildB);
                            tree[rootNodeId] = node;
                        }
                        returnStack.Push(rootNodeId);
                        continue;
                    }

                    stack.Push(-rootNodeId - 1);
                    stack.Push(node.ChildA);
                    stack.Push(node.ChildB);
                }
                else
                {
                    int a = returnStack.Pop();
                    int b = returnStack.Pop();

                    rootNodeId = -rootNodeId - 1;
                    var node = tree[rootNodeId];
                    node.Bounds = Merge(tree[a].Bounds, tree[b].Bounds);
                    tree[rootNodeId] = node;

                    returnStack.Push(rootNodeId);
                }
            }
        }

        private static Box3F Merge(Box3F a, Box3F b)
        {
            a.Extend(b);
            return a;
        }

        private static int[] CalculateMortonCodes(BVHNode[] leaveNodes, Box3F boundingBox)
        {
            int[] result = new int[leaveNodes.Length];
            float scalingX = 1.0f / (boundingBox.Max.X - boundingBox.Min.X);
            float scalingY = 1.0f / (boundingBox.Max.Y - boundingBox.Min.Y);
            float scalingZ = 1.0f / (boundingBox.Max.Z - boundingBox.Min.Z);
            for (int i = 0; i < result.Length; ++i)
            {
                var n = leaveNodes[i];
                Vec3F f;
                f.X = (0.5f * (n.Bounds.Min.X + n.Bounds.Max.X) - boundingBox.Min.X) * scalingX;
                f.Y = (0.5f * (n.Bounds.Min.Y + n.Bounds.Max.Y) - boundingBox.Min.Y) * scalingY;
                f.Z = (0.5f * (n.Bounds.Min.Z + n.Bounds.Max.Z) - boundingBox.Min.Z) * scalingZ;
                result[i] = ZOrderCurve.Morton3D(f.X, f.Y, f.Z);
            }
            return result;
        }

        public static BVHNode[] BuildTree(Func<int, Box3F> boxBuilder, int numBoxes)
        {
            List<BVHNode> tree = new List<BVHNode>((int)(2.1 * numBoxes));
            BVHNode[] leaveNodes = new BVHNode[numBoxes];

            for (int i = 0; i < numBoxes; ++i)
            {
                var node = new BVHNode(boxBuilder(i), -1, i);
                leaveNodes[i] = node;
            }

            float[] centers = new float[numBoxes];
            Queue<Range> todo = new Queue<Range>();
            todo.Enqueue(new Range(0, numBoxes));

            List<Range> todoLeaves = new List<Range>(numBoxes);

            while (todo.Count > 0)
            {
                Range r = todo.Dequeue();
                int start = r.Start;
                int end = r.End;
                int length = end - start;
                if (length == 1)
                {
                    /*//Create a leave node
                    if (r.ParentIndex >= 0)
                    {
                        BVHNode parent = tree[r.ParentIndex];
                        if (parent.ChildA < 0)
                            parent.ChildA = tree.Count;
                        else
                            parent.ChildB = tree.Count;
                        tree[r.ParentIndex] = parent;
                    }

                    tree.Add(leaveNodes[start]);*/
                    todoLeaves.Add(r);

                    continue;
                }

                BVHNode a = leaveNodes[start];
                //float minX = a.MinX; float minY = a.MinY; float minZ = a.MinZ;
                //float maxX = a.MaxX; float maxY = a.MaxY; float maxZ = a.MaxZ;
                Box3F box = a.Bounds;
                for (int i = start + 1; i < end; ++i)
                {
                    Box3F v = leaveNodes[i].Bounds;
                    box.Extend(v);
                }


                for (int i = start; i < end; ++i)
                    centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.X + leaveNodes[i].Bounds.Max.X);
                float varianceX = Variance(centers, start, length);
                for (int i = start; i < end; ++i)
                    centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.Y + leaveNodes[i].Bounds.Max.Y);
                float varianceY = Variance(centers, start, length);
                for (int i = start; i < end; ++i)
                    centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.Z + leaveNodes[i].Bounds.Max.Z);
                float varianceZ = Variance(centers, start, length);

                float sizeX = varianceX; // maxX - minX;
                float sizeY = varianceY; // maxY - minY;
                float sizeZ = varianceZ; // maxZ - minZ;
                if (sizeX >= sizeY && sizeX >= sizeZ)
                {
                    for (int i = start; i < end; ++i)
                        centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.X + leaveNodes[i].Bounds.Max.X);

                    Array.Sort(centers, leaveNodes, start, length);
                }
                else if (sizeY >= sizeX && sizeY >= sizeZ)
                {
                    for (int i = start; i < end; ++i)
                        centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.Y + leaveNodes[i].Bounds.Max.Y);

                    Array.Sort(centers, leaveNodes, start, length);
                }
                else
                {
                    for (int i = start; i < end; ++i)
                        centers[i] = 0.5f * (leaveNodes[i].Bounds.Min.Z + leaveNodes[i].Bounds.Max.Z);

                    Array.Sort(centers, leaveNodes, start, length);
                }
                int mid = start + length / 2;


                if (r.ParentIndex >= 0)
                {
                    BVHNode parent = tree[r.ParentIndex];
                    if (parent.ChildA < 0)
                        parent.ChildA = tree.Count;
                    else
                        parent.ChildB = tree.Count;
                    tree[r.ParentIndex] = parent;
                }

                BVHNode aabb = new BVHNode() { Bounds = box, ChildA = -1, ChildB = -1 };

                todo.Enqueue(new Range(start, mid, tree.Count));
                todo.Enqueue(new Range(mid, end, tree.Count));

                tree.Add(aabb);
            }

            //This ensures that the leave nodes are at the end of the tree
            for (int i = 0; i < todoLeaves.Count; ++i)
            {
                Range r = todoLeaves[i];
                if (r.ParentIndex >= 0)
                {
                    BVHNode parent = tree[r.ParentIndex];
                    if (parent.ChildA < 0)
                        parent.ChildA = tree.Count;
                    else
                        parent.ChildB = tree.Count;
                    tree[r.ParentIndex] = parent;
                }

                tree.Add(leaveNodes[r.Start]);
            }

            return tree.ToArray();
        }


        //Used to determine split axis according to splatter points rule
        //http://www.jan-ohlenburg.de/pdf/Diploma.pdf page 29
        public static float Variance(float[] centers, int start, int length)
        {
            float mean = 0;
            int end = start + length;
            for (int i = start; i < end; ++i)
                mean += centers[i];
            mean /= length;

            float variance = 0;
            for (int i = start; i < end; ++i)
            {
                var delta = centers[i] - mean;
                variance += delta * delta;
            }
            return variance;
        }

        public static void Traverse(BVHNode[] tree, Func<BVHNode, int, TraversalControl> traversalController, int rootNodeIndex = 0)
        {
            Traverse(tree, new GenericTraversalController(traversalController), rootNodeIndex);
        }

        //public struct IncrementalTraverser<T> where T : ITraversalController
        //{
        //    T traversalController;
        //    BVHNode[] tree;
        //    int index;
        //    int[] todoStack;
        //    int stackCount;

        //    public IncrementalTraverser(T traversalController, int rootIndex = 0)
        //    {

        //    }

        //    public bool Next()
        //    {
        //        while (true)
        //        {
        //            BVHNode a = tree[index];

        //            TraversalControl control = traversalController.Analyze(a, index);
        //            if (control == TraversalControl.Abort)
        //                return false;
        //            if (control == TraversalControl.GoDeeper || control == TraversalControl.GoDeeperBFirst)
        //            {
        //                //if (a.ChildA >= 0)
        //                {
        //                    if (control == TraversalControl.GoDeeper)
        //                    {
        //                        todoStack[stackCount] = a.ChildB;
        //                        index = a.ChildA;
        //                    }
        //                    else
        //                    {
        //                        todoStack[stackCount] = a.ChildA;
        //                        index = a.ChildB;
        //                    }

        //                    ++stackCount;

        //                    if (stackCount == todoStack.Length)
        //                    {
        //                        var newStack = new int[2 * todoStack.Length];
        //                        Array.Copy(todoStack, newStack, todoStack.Length);
        //                        todoStack = newStack;
        //                    }
        //                    continue;
        //                }
        //            }
        //            if (stackCount == 0) break;
        //            --stackCount;
        //            index = todoStack[stackCount];
        //        }
        //    }
        //}

        public unsafe static void Traverse<T>(BVHNode[] tree, in T traversalController, int rootNodeIndex = 0) where T : ITraversalController
        {
            int index = rootNodeIndex;

            int* todoStack = stackalloc int[64];
            Span<int> stack = new Span<int>(todoStack, 64);

            int stackCount = 0;
            while (true)
            {
                BVHNode a = tree[index];

                TraversalControl control = traversalController.Analyze(a, index);
                if (control == TraversalControl.Abort)
                    return;
                if (control == TraversalControl.GoDeeper || control == TraversalControl.GoDeeperBFirst)
                {
                    //if (a.ChildA >= 0)
                    {
                        if (control == TraversalControl.GoDeeper)
                        {
                            stack[stackCount] = a.ChildB;
                            index = a.ChildA;
                        }
                        else
                        {
                            stack[stackCount] = a.ChildA;
                            index = a.ChildB;
                        }

                        ++stackCount;

                        if (stackCount == stack.Length)
                        {
                            var newStack = new int[2 * stack.Length];
                            stack.CopyTo(newStack);
                            stack = newStack;
                        }
                        continue;
                    }
                }
                if (stackCount == 0) break;
                --stackCount;
                index = stack[stackCount];
            }
        }

        public static void Traverse<T>(BVHNode[] treeA, BVHNode[] treeB, T traversalController, int rootNodeIndexA = 0, int rootNodeIndexB = 0) where T : IMultiTraversalController
        {
            Int2 index = new Int2(rootNodeIndexA, rootNodeIndexB);

            Int2[] todoStack = new Int2[64];

            int stackCount = 0;
            while (true)
            {
                BVHNode x = treeA[index.X];
                BVHNode y = treeB[index.Y];

                MultiTraversalControl control = traversalController.Analyze(x, index.X, y, index.Y);
                if (control == MultiTraversalControl.Abort)
                    return;
                if (control != MultiTraversalControl.DontGoDeeper)
                {
                    //if (a.ChildA >= 0)
                    {
                        if (control == MultiTraversalControl.GoDeeperXSubBoxA || control == MultiTraversalControl.GoDeeperXSubBoxB)
                        {
                            //todoStack[stackCount] = a.ChildB;
                            //index = a.ChildA;

                            //Push(s, a->right, b);
                            //a = a->left;

                            bool swap = control == MultiTraversalControl.GoDeeperXSubBoxB;
                            Int2 push = new Int2();
                            push.X = swap ? x.ChildA : x.ChildB;
                            push.Y = index.Y;
                            todoStack[stackCount] = push; //s.Push(push);                         
                            index.X = swap ? x.ChildB : x.ChildA;
                        }
                        else
                        {
                            //todoStack[stackCount] = a.ChildA;
                            //index = a.ChildB;

                            //Push(s, a, b->right);
                            //b = b->left;

                            bool swap = control == MultiTraversalControl.GoDeeperYSubBoxB;
                            Int2 push = new Int2();
                            push.X = index.X;
                            push.Y = swap ? y.ChildA : y.ChildB;
                            todoStack[stackCount] = push; //s.Push(push);                        
                            index.Y = swap ? y.ChildB : y.ChildA;
                        }

                        ++stackCount;

                        if (stackCount == todoStack.Length)
                        {
                            var newStack = new Int2[2 * todoStack.Length];
                            Array.Copy(todoStack, newStack, todoStack.Length);
                            todoStack = newStack;
                        }
                        continue;
                    }
                }
                if (stackCount == 0) break;
                --stackCount;
                index = todoStack[stackCount];
            }
        }
    }
}
