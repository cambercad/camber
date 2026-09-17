using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class CurvedLeverFilletTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedFingerContactRimsRemainClosed(bool reverse)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-420,-300,-50),new Vec3D(1420,300,1120)),.2);
        var sk=new PlotterSketcherCoordSys("blade",new CoordinateSystem(new Vec3D(1033,-193,878),new Vec3D(1,0,0),new Vec3D(0,0,1),new Vec3D(0,-1,0)));
        sk.AddCubicHermiteSpline(new Vec2D[]{new(-6,5),new(0,7),new(4,7),new(8,2)},new(0,1),new(1,-2));
        var finger=sk.AddCubicHermiteSpline(new Vec2D[]{new(8,2),new(12,-15),new(14,-42),new(9,-69),new(2,-87)},new(1,-2),new(0,-1));
        sk.AddArc(new(2,-87),new(-2.5,-91.5),new(-7,-87));
        sk.AddCubicHermiteSpline(new Vec2D[]{new(-7,-87),new(-1,-73),new(4,-47)},new(0,1),new(0,1));
        sk.AddLine(new(4,-47),new(4,-22));
        sk.AddCubicHermiteSpline(new Vec2D[]{new(4,-22),new(0,-10),new(-6,-2)},new(0,1),new(0,1));
        sk.AddLine(new(-6,-2),new(-6,5));
        if (reverse) sk.SetCurves(sk.GetCurves().Select(loop => loop.AsEnumerable().Reverse().Select(c => { var reversed=c.Reverse(); reversed.Name=c.Name; return reversed; }).ToList()).ToList());
        var blank=api.Extrude(sk,14,name:"blade",maxDeviation:.03);
        var edges=new List<string>{$"[blade-{finger.Name},blade-ExtrudeTop]",$"[blade-{finger.Name},blade-ExtrudeBottom]"};
        foreach(var name in blank.groupIdToExtendedName.Values.Where(n=>!n.Contains("Extrude")))
        {
            Assert.True(blank.TryGetTopologySurface(name,out var surface));
            var support=blank.surfaceMetaData[name].NurbsSurface;
            foreach(int index in surface.Triangles.SelectMany(t=>new[]{t.A,t.B,t.C}).Distinct())
            {
                var uv=surface.Uv[index];
                if (uv.X != 0 && uv.X != 1) continue;
                Assert.InRange((support.Evaluate(uv.X,uv.Y)-surface.Points[index]).Length(),0,.005);
            }
        }
        var result=api.Fillet(blank,edges,.6,.02,"rounded");
        MeshTestHelpers.AssertValidMesh(result.Mesh.Positions,result.Mesh.Triangles);
    }
}
