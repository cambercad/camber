using GeoCore;

namespace NURBS
{
    public struct BoxNode2D
    {
        public double MinX;
        public double MinY;
        public double MaxX;
        public double MaxY;

        public int IndexA;
        public int IndexB;

        public BoxNode2D(double minX, double minY, double maxX, double maxY, int id)
        {
            MinX = minX; MinY = minY;
            MaxX = maxX; MaxY = maxY;

            IndexA = id;
            IndexB = -1; //The user usually only builds leave nodes marked by -1 as IndexB
        }

        public BoxNode2D(Vec2D a, Vec2D b, int id)
        {
            MinX = a.X; MinY = a.Y;
            MaxX = a.X; MaxY = a.Y;

            if (b.X < MinX) MinX = b.X; if (b.Y < MinY) MinY = b.Y;
            if (b.X > MaxX) MaxX = b.X; if (b.Y > MaxY) MaxY = b.Y;

            IndexA = id;
            IndexB = -1; //The user usually only builds leave nodes marked by -1 as IndexB
        }

        public BoxNode2D(Vec2D a, Vec2D b, Vec2D c, int id/*, double enlargement = 1e-6*/)
        {
            MinX = a.X; MinY = a.Y;
            MaxX = a.X; MaxY = a.Y;

            if (b.X < MinX) MinX = b.X; if (b.Y < MinY) MinY = b.Y;
            if (b.X > MaxX) MaxX = b.X; if (b.Y > MaxY) MaxY = b.Y;

            if (c.X < MinX) MinX = c.X; if (c.Y < MinY) MinY = c.Y;
            if (c.X > MaxX) MaxX = c.X; if (c.Y > MaxY) MaxY = c.Y;

            //MinX -= enlargement; MaxX += enlargement;
            //MinY -= enlargement; MaxY += enlargement;

            IndexA = id;
            IndexB = -1; //The user usually only builds leave nodes marked by -1 as IndexB
        }


        public bool IsLeaveNode { get { return IndexB < 0; } }
        public int ID { get { return IndexA; } }

        public static bool Overlap(BoxNode2D a, BoxNode2D b)
        {
            return !(a.MaxX < b.MinX || b.MaxX < a.MinX || a.MaxY < b.MinY || b.MaxY < a.MinY);
        }
                
        public bool IntersectsLineSegment(Vec2D p0, Vec2D p1, double eps = 1e-12)
        {
            return GeometricAlgorithms.IntersectBoxLineSegment(MinX, MaxX, MinY, MaxY, p0, p1, eps);
        }

        public double Area
        {
            get
            {
                double dx = MaxX - MinX;
                double dy = MaxY - MinY;
                return dx * dy;
            }
        }

        public override string ToString()
        {
            return "MinX:" + MinX.ToString() + "   MinY:" + MinY.ToString()
                + "   MaxX:" + MaxX.ToString() + "   MaxY:" + MaxY.ToString()
                + "   A:" + IndexA.ToString() + "   B:" + IndexB.ToString();
        }
    }
}
