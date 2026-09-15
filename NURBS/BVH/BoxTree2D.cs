using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace NURBS
{
    public class BoxTree2D
    {
        public static List<BoxNode2D> BuildTree(BoxNode2D[] boxes, double enlargement = 1e-6)
        {
            XComparerBox2D xComparer = new XComparerBox2D();
            YComparerBox2D yComparer = new YComparerBox2D();

            List<BoxNode2D> tree = new List<BoxNode2D>();
            int rootIndex = BuildTree(boxes, 0, boxes.Length, enlargement, xComparer, yComparer, tree);
            //Debug(tree);
            return tree;
        }

        private static int BuildTree(BoxNode2D[] triBoxes, int startIndex, int length, double enlargement,
            XComparerBox2D xComparer, YComparerBox2D yComparer, List<BoxNode2D> tree)
        {
            if (length == 1)
            {
                //We reached a leave node
                BoxNode2D boxNode = triBoxes[startIndex];
                BoxNode2D leave = new BoxNode2D()
                {
                    MinX = boxNode.MinX - enlargement,
                    MinY = boxNode.MinY - enlargement,
                    MaxX = boxNode.MaxX + enlargement,
                    MaxY = boxNode.MaxY + enlargement,

                    IndexA = boxNode.IndexA,
                    IndexB = -1
                };
                tree.Add(leave);
                return tree.Count - 1;
            }

            //Top down tree construction
            double minX, minY, maxX, maxY;
            MinMax(triBoxes, startIndex, length, out minX, out minY, out maxX, out maxY);

            double deltaX = maxX - minX;
            double deltaY = maxY - minY;

            if (deltaX >= deltaY)
            {
                Array.Sort(triBoxes, startIndex, length, xComparer);
            }
            else
            {
                Array.Sort(triBoxes, startIndex, length, yComparer);
            }

            BoxNode2D node = new BoxNode2D(); //TODO: Determine the size of the node

            int halfLength = length / 2;
            node.IndexA = BuildTree(triBoxes, startIndex, halfLength, enlargement, xComparer, yComparer, tree);
            node.IndexB = BuildTree(triBoxes, startIndex + halfLength, length - halfLength, enlargement, xComparer, yComparer, tree);
            node.MinX = minX - enlargement;
            node.MinY = minY - enlargement;
            node.MaxX = maxX + enlargement;
            node.MaxY = maxY + enlargement;
            tree.Add(node);
            return tree.Count - 1;
        }

        public static void MinMax(BoxNode2D[] boxes, int startIndex, int length,
            out double minX, out double minY, out double maxX, out double maxY)
        {
            BoxNode2D b = boxes[startIndex];
            minX = b.MinX; minY = b.MinY;
            maxX = b.MaxX; maxY = b.MaxY;
            int end = startIndex + length;
            for (int i = startIndex + 1; i < end; ++i)
            {
                b = boxes[i];
                if (b.MinX < minX) minX = b.MinX; if (b.MaxX > maxX) maxX = b.MaxX;
                if (b.MinY < minY) minY = b.MinY; if (b.MaxY > maxY) maxY = b.MaxY;
            }
        }

        public static void Traverse(IList<BoxNode2D> treeA, BoxNode2D b, Func<BoxNode2D, BoxNode2D, int, bool> processor, int rootIndex = -1)
        {
            int index = treeA.Count - 1;
            if (rootIndex >= 0)
                index = rootIndex;

            Stack<int> todoStack = new Stack<int>();

            while (true)
            {
                BoxNode2D a = treeA[index];

                if (BoxNode2D.Overlap(a, b))
                {
                    if (a.IndexB < 0) //Check if both of them are leaves
                    {
                        if (processor(a, b, index))
                            return;
                    }
                    else
                    {
                        if (treeA[a.IndexA].Area >= treeA[a.IndexB].Area)
                        {
                            todoStack.Push(a.IndexB);
                            index = a.IndexA;
                        }
                        else
                        {
                            todoStack.Push(a.IndexA);
                            index = a.IndexB;
                        }
                        continue;
                    }
                }
                if (todoStack.Count == 0) break;

                index = todoStack.Pop();
            }
        }

        //public static void Traverse(IList<BoxNode2D> treeA, IList<BoxNode2D> treeB, Func<BoxNode2D, BoxNode2D, int, int, bool> processor)
        //{
        //    Int2 index = new Int2();
        //    index.X = treeA.Count - 1;
        //    index.Y = treeB.Count - 1;

        //    Stack<Int2> todoStack = new Stack<Int2>();

        //    while (true)
        //    {
        //        BoxNode2D a = treeA[index.X];
        //        BoxNode2D b = treeB[index.Y];

        //        if (BoxNode2D.Overlap(a, b))
        //        {
        //            if (a.IndexB < 0 && b.IndexB < 0) //Check if both of them are leaves
        //            {
        //                if (processor(a, b, index.X, index.Y))
        //                    return;
        //            }
        //            else
        //            {
        //                if (b.IndexB < 0 || (a.IndexB >= 0 && (a.Area >= b.Area))) // ‘Descend larger’ descent rule
        //                {
        //                    //Push(s, a->right, b);
        //                    //a = a->left;
        //                    Int2 push = new Int2();
        //                    push.X = a.IndexB;
        //                    push.Y = index.Y;
        //                    todoStack.Push(push);
        //                    index.X = a.IndexA;
        //                }
        //                else
        //                {
        //                    //Push(s, a, b->right);
        //                    //b = b->left;
        //                    Int2 push = new Int2();
        //                    push.X = index.X;
        //                    push.Y = b.IndexB;
        //                    todoStack.Push(push);
        //                    index.Y = b.IndexA;
        //                }
        //                continue;
        //            }
        //        }
        //        if (todoStack.Count == 0) break;

        //        index = todoStack.Pop();
        //    }
        //}
    }










    //BE CAREFUL: This tree has its root node at index zero
    public class BoxTree2DParallel
    {


        public static List<BoxNode2D> BuildTree(BoxNode2D[] boxes, double enlargement = 1e-6)
        {
            XComparerBox2D xComparer = new XComparerBox2D();
            YComparerBox2D yComparer = new YComparerBox2D();

            List<BoxNode2D> tree = new List<BoxNode2D>();
            /*int rootIndex =*/
            List<Task> tasks = new List<Task>();
            BuildTreeTask(0, boxes, 0, boxes.Length, enlargement, xComparer, yComparer, tree, 0, /*5*/3, tasks);
            for (int i = 0; i < tasks.Count; ++i)
                tasks[i].Wait();
            //Debug(tree);
            return tree;
        }

        private static void BuildTreeTask(int parentIndex, BoxNode2D[] triBoxes, int startIndex, int length, double enlargement,
            XComparerBox2D xComparer, YComparerBox2D yComparer, List<BoxNode2D> tree, int depth, int splitDepth, List<Task> tasks)
        {
            if (splitDepth == depth)
            {
                Task t = new Task(delegate ()
                {
                    BuildTree(parentIndex, triBoxes, startIndex, length, enlargement, xComparer, yComparer, tree, depth, splitDepth, tasks);
                });
                t.Start();
                lock (tasks)
                {
                    tasks.Add(t);
                }
            }
            else
            {
                BuildTree(parentIndex, triBoxes, startIndex, length, enlargement, xComparer, yComparer, tree, depth, splitDepth, tasks);
            }
        }

        //private static void BuildTreeTask(int parentIndex, BoxNode2D[] triBoxes, int startIndex, int length, double enlargement,
        //   XComparerBox2D xComparer, YComparerBox2D yComparer, List<BoxNode2D> tree, int depth, int splitDepth, List<Task> tasks, int parentTaskIndex)
        //{
        //    if (splitDepth > depth)
        //    {
        //        if (parentTaskIndex >= 0)
        //            tasks[parentTaskIndex].Wait();

        //        Task t;
        //        lock (tasks)
        //        {
        //            int id = tasks.Count;
        //            t = new Task(delegate ()
        //            {
        //                BuildTree(parentIndex, triBoxes, startIndex, length, enlargement, xComparer, yComparer, tree, depth, splitDepth, tasks, id);
        //            });

        //            tasks.Add(t);
        //        }
        //        t.Start();
        //    }
        //    else
        //    {
        //        BuildTree(parentIndex, triBoxes, startIndex, length, enlargement, xComparer, yComparer, tree, depth, splitDepth, tasks, -1);
        //    }
        //}

        private static void BuildTree(int parentIndex, BoxNode2D[] triBoxes, int startIndex, int length, double enlargement,
            XComparerBox2D xComparer, YComparerBox2D yComparer, List<BoxNode2D> tree, int depth, int splitDepth, List<Task> tasks)
        {
            if (length == 1)
            {
                //We reached a leave node
                BoxNode2D boxNode = triBoxes[startIndex];
                BoxNode2D leave = new BoxNode2D()
                {
                    MinX = boxNode.MinX - enlargement,
                    MinY = boxNode.MinY - enlargement,
                    MaxX = boxNode.MaxX + enlargement,
                    MaxY = boxNode.MaxY + enlargement,

                    IndexA = boxNode.IndexA,
                    IndexB = -1
                };
                //BoxNode2D leave = new BoxNode2D(boxNode.MinX - enlargement, boxNode.MinY - enlargement, boxNode.MaxX + enlargement, boxNode.MaxY + enlargement, boxNode.IndexA);

                lock (tree)
                {
                    int id = tree.Count;
                    tree.Add(leave);


                    if (parentIndex != 0)
                    {
                        BoxNode2D parent;
                        if (parentIndex > 0)
                        {
                            parent = tree[parentIndex - 1];
                            parent.IndexA = id;
                            tree[parentIndex - 1] = parent;

                        }
                        else
                        {
                            parent = tree[-parentIndex - 1];
                            parent.IndexB = id;
                            tree[-parentIndex - 1] = parent;
                        }
                    }

                }
                //return tree.Count - 1;
                return;
            }

            //Top down tree construction
            double minX, minY, maxX, maxY;
            BoxTree2D.MinMax(triBoxes, startIndex, length, out minX, out minY, out maxX, out maxY);

            double deltaX = maxX - minX;
            double deltaY = maxY - minY;

            if (deltaX >= deltaY)
            {
                Array.Sort(triBoxes, startIndex, length, xComparer);
            }
            else
            {
                Array.Sort(triBoxes, startIndex, length, yComparer);
            }

            BoxNode2D node = new BoxNode2D(); //TODO: Determine the size of the node         
            int index;
            lock (tree)
            {
                index = tree.Count;
                node.MinX = minX - enlargement;
                node.MinY = minY - enlargement;
                node.MaxX = maxX + enlargement;
                node.MaxY = maxY + enlargement;
                tree.Add(node);

                if (parentIndex != 0)
                {
                    BoxNode2D parent;
                    if (parentIndex > 0)
                    {
                        parent = tree[parentIndex - 1];
                        parent.IndexA = index;
                        tree[parentIndex - 1] = parent;

                    }
                    else
                    {
                        parent = tree[-parentIndex - 1];
                        parent.IndexB = index;
                        tree[-parentIndex - 1] = parent;
                    }
                }
            }
            int halfLength = length / 2;


            BuildTreeTask(index + 1, triBoxes, startIndex, halfLength, enlargement, xComparer, yComparer, tree, depth + 1, splitDepth, tasks);
            /*node.IndexB =*/
            BuildTreeTask(-(index + 1), triBoxes, startIndex + halfLength, length - halfLength, enlargement, xComparer, yComparer, tree, depth + 1, splitDepth, tasks);
            

            //return tree.Count - 1;
        }
    }
}
