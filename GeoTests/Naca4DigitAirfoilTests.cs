using Curves;
using GeoCore;

namespace GeoTests;

public class Naca4DigitAirfoilTests
{
    [Fact]
    public void FromCode_PadsLeadingZeros_0012()
    {
        var s = Naca4DigitSpec.FromCode(12);
        Assert.True(s.MaxCamber < 1e-12);
        Assert.InRange(s.MaxThickness, 0.119, 0.121);
    }

    [Fact]
    public void FromCode_2412_CamberAndThickness()
    {
        var s = Naca4DigitSpec.FromCode(2412);
        Assert.InRange(s.MaxCamber, 0.019, 0.021);
        Assert.InRange(s.MaxCamberPosition, 0.39, 0.41);
        Assert.InRange(s.MaxThickness, 0.119, 0.121);
    }

    [Fact]
    public void TryParse_StripsNacaPrefix()
    {
        Assert.True(Naca4DigitSpec.TryParse("NACA 0012", out var s));
        Assert.InRange(s.MaxThickness, 0.119, 0.121);
    }

    [Fact]
    public void PlotterSketcher_ClosedTessellation_NoGaps()
    {
        var sk = new PlotterSketcher("airfoil");
        sk.AddNaca4DigitAirfoil(12, Vec2DOps.Zero, 1.0, 0, samplesPerSide: 40, analyticEndTangents: true);
        sk.Tessellate(1e-5, out _, out _, out _, 1e-7, SketchTessellationFlags.None);
    }

    [Fact]
    public void Symmetric0012_UpperYPositive_AtMidChord()
    {
        double x = 0.3;
        Naca4DigitAirfoil.UpperChordCoords(Naca4DigitSpec.FromCode(12), x, out _, out double yu);
        Naca4DigitAirfoil.LowerChordCoords(Naca4DigitSpec.FromCode(12), x, out _, out double yl);
        Assert.True(yu > 0 && yl < 0 && yu > -yl * 0.99 && yu < -yl * 1.01);
    }

    [Fact]
    public void SolveUpperLowerStationForChordwiseX_BisectionHitsTrimLine_2412()
    {
        var spec = Naca4DigitSpec.FromCode(2412);
        const double target = 0.99;
        double sUp = Naca4DigitAirfoil.SolveUpperStationForChordwiseX(spec, target);
        double sLo = Naca4DigitAirfoil.SolveLowerStationForChordwiseX(spec, target);
        Naca4DigitAirfoil.UpperChordCoords(spec, sUp, out double xu, out _);
        Naca4DigitAirfoil.LowerChordCoords(spec, sLo, out double xl, out _);
        Assert.InRange(xu, target - 1e-9, target + 1e-9);
        Assert.InRange(xl, target - 1e-9, target + 1e-9);
    }

    [Fact]
    public void UpperChordDerivative_MatchesSecant_2412()
    {
        var spec = Naca4DigitSpec.FromCode(2412);
        double x = 0.35;
        const double h = 1e-6;
        Naca4DigitAirfoil.UpperChordDerivativeWrtX(spec, x, out double duX, out double duY);
        Naca4DigitAirfoil.UpperChordCoords(spec, x - h, out double xa, out double ya);
        Naca4DigitAirfoil.UpperChordCoords(spec, x + h, out double xb, out double yb);
        double sx = (xb - xa) / (2 * h);
        double sy = (yb - ya) / (2 * h);
        Assert.True(Math.Abs(duX - sx) < 1e-5 && Math.Abs(duY - sy) < 1e-5);
    }
}
