using GeoCore;

namespace NURBS
{
    public interface IBorderPolygon
    {
        Vec2D this[int index] { get; }
        int Count { get; }
    }
}
