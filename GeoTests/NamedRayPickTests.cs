using GeoMeta;

namespace GeoTests;

public sealed class NamedRayPickTests
{
    [Fact]
    public void Closest_ReturnsPointName_WhenRayAimsAtPoint()
    {
        var targets = new List<NamedPickTarget>
        {
            Point("box:[A,B]@0.000", 0, 0, 0),
            Point("box:[A,B]@1.000", 10, 0, 0),
        };

        string hit = NamedRayPick.Closest(0, 0, 10, 0, 0, -1, targets, 0.25, 0.25);
        Assert.Equal("box:[A,B]@0.000", hit);
    }

    [Fact]
    public void Closest_PrefersNearerPoint()
    {
        var targets = new List<NamedPickTarget>
        {
            Point("far", 2, 0, 0),
            Point("near", 0.2, 0, 0),
        };

        string hit = NamedRayPick.Closest(0, 0, 10, 0, 0, -1, targets, 3.0, 3.0);
        Assert.Equal("near", hit);
    }

    [Fact]
    public void Closest_PointWinsOverNearbyCurve()
    {
        var targets = new List<NamedPickTarget>
        {
            Curve("Sketch1:Line1", 0, 0, 0, 4, 0, 0),
            Point("Sketch1:Line1@1.000", 4, 0, 0),
        };

        string hit = NamedRayPick.Closest(4, 0, 10, 0, 0, -1, targets, 0.35, 0.35);
        Assert.Equal("Sketch1:Line1@1.000", hit);
    }

    [Fact]
    public void Closest_HitsCurveWhenRayMissesPoints()
    {
        var targets = new List<NamedPickTarget>
        {
            Point("Sketch1:Line1@0.000", 0, 0, 0),
            Point("Sketch1:Line1@1.000", 4, 0, 0),
            Curve("Sketch1:Line1", 0, 0, 0, 4, 0, 0),
        };

        string hit = NamedRayPick.Closest(2, 0, 10, 0, 0, -1, targets, 0.2, 0.2);
        Assert.Equal("Sketch1:Line1", hit);
    }

    [Fact]
    public void Closest_ReturnsEmptyWhenRayMisses()
    {
        var targets = new List<NamedPickTarget>
        {
            Point("only", 10, 10, 0),
        };

        string hit = NamedRayPick.Closest(0, 0, 10, 0, 0, -1, targets, 0.2, 0.2);
        Assert.Equal("", hit);
    }

    [Fact]
    public void Closest_IgnoresPointsBehindRay()
    {
        var targets = new List<NamedPickTarget>
        {
            Point("behind", 0, 0, 20),
            Point("ahead", 0, 0, 0),
        };

        string hit = NamedRayPick.Closest(0, 0, 10, 0, 0, -1, targets, 0.5, 0.5);
        Assert.Equal("ahead", hit);
    }

    static NamedPickTarget Point(string name, double x, double y, double z) =>
        new NamedPickTarget { Name = name, Kind = NamedRayPick.KindPoint, X = x, Y = y, Z = z };

    static NamedPickTarget Curve(string name, double x0, double y0, double z0, double x1, double y1, double z1)
    {
        return new NamedPickTarget
        {
            Name = name,
            Kind = NamedRayPick.KindCurve,
            Polyline = new List<NamedPickPoint>
            {
                new NamedPickPoint { X = x0, Y = y0, Z = z0 },
                new NamedPickPoint { X = x1, Y = y1, Z = z1 },
            },
        };
    }
}
