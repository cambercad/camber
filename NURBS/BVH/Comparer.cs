using System.Collections.Generic;

namespace NURBS
{
    public class XComparerBox2D : IComparer<BoxNode2D> { public int Compare(BoxNode2D a, BoxNode2D b) { return (a.MinX).CompareTo(b.MinX); } }
    public class YComparerBox2D : IComparer<BoxNode2D> { public int Compare(BoxNode2D a, BoxNode2D b) { return (a.MinY).CompareTo(b.MinY); } }
}
