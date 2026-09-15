using System.Collections.Generic;
using CSG;
using GeoCore;

namespace GeoPy;

internal static class NativeUtil
{
    public static string EmptyToNull(string s)
    {
        if (string.IsNullOrEmpty(s))
            return null;
        return s;
    }

    public static string ResolvePlane(string plane)
    {
        if (string.IsNullOrEmpty(plane) || plane == "xy" || plane == "XY")
            return Geo.DefaultPlanes.OriginXY;
        if (plane == "yz" || plane == "YZ")
            return Geo.DefaultPlanes.OriginYZ;
        if (plane == "zx" || plane == "ZX" || plane == "xz" || plane == "XZ")
            return Geo.DefaultPlanes.OriginZX;
        return plane;
    }

    public static Vec3D V3(double x, double y, double z)
    {
        return new Vec3D(x, y, z);
    }

    public static List<string> SplitNames(string joined)
    {
        var list = new List<string>();
        if (string.IsNullOrEmpty(joined))
            return list;
        string[] parts = joined.Split('|');
        for (int i = 0; i < parts.Length; i++)
        {
            if (parts[i].Length > 0)
                list.Add(parts[i]);
        }
        return list;
    }

    public static BooleanOp ToBooleanOp(int operation)
    {
        return (BooleanOp)operation;
    }
}
