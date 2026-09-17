// Regression reduced to the original 19 mm wrench plan/elevation extrusions.
// Coordinates preserve the failing authored circular run-outs and head transitions;
// no machining holes, panels, Python dependency or display mesh are required.
using Geo;
using GeoCore;
using Curves;
namespace GeoTests;
public class WrenchFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

[Theory]
[InlineData("all")]
[InlineData("[plan_blank-upper_grip,side_die-grip_top]")]
[InlineData("[plan_blank-upper_open_head,side_die-jaw_top]")]
public void TrimmedExtrusionEdgeRound(string edge){
var api=new GeoAPI(new Box3D(new Vec3D(-20,-40,-25),new Vec3D(250,40,40)),.035);
var s0=api.GetPlotterSketcher(new Plane3D(new Vec3D(0,0,-15),new Vec3D(0,0,1),new Vec3D(1,0,0),new Vec3D(0,1,0)),"forging_plan");
s0.SolveAfterEveryConstraint=false;
s0.AddArc(new Vec2D(32.340502194729837,11.891304347826086),new Vec2D(13.716790941031945,15.08038928135279),new Vec2D(0.068898833370909074,2.0133343495020801)).Name="upper_open_head";
s0.AddArc(new Vec2D(0.068898833370906631,2.0133343495020806),new Vec2D(0.31874323620323564,0.79808695631723336),new Vec2D(1.5105828541230242,0.45346139512650829)).Name="upper_tip";
s0.AddLine(new Vec2D(1.5105828541230246,0.45346139512650829),new Vec2D(19.268091626062365,5.2115715301562311)).Name="upper_jaw";
s0.AddArc(new Vec2D(19.268091626062365,5.211571530156232),new Vec2D(22.682777329023708,4.7620196288853975),new Vec2D(24.779443547324515,2.0295910148167677)).Name="upper_throat";
s0.AddLine(new Vec2D(24.779443547324515,2.0295910148167677),new Vec2D(27.367633998349721,-7.6296672480739147)).Name="throat";
s0.AddArc(new Vec2D(27.367633998349721,-7.6296672480739147),new Vec2D(26.918082097078887,-11.044352951035258),new Vec2D(24.185653483010256,-13.141019169336065)).Name="lower_throat";
s0.AddLine(new Vec2D(24.185653483010256,-13.141019169336065),new Vec2D(5.7854427969896278,-18.071340763217805)).Name="lower_jaw";
s0.AddArc(new Vec2D(5.7854427969896278,-18.071340763217805),new Vec2D(4.9343848080168247,-18.929469111325545),new Vec2D(5.2616533118237205,-20.092903376265578)).Name="lower_tip";
s0.AddArc(new Vec2D(5.2616533118237232,-20.092903376265582),new Vec2D(21.694667066668604,-25.919981255853632),new Vec2D(36.865156790260798,-17.326086956521738)).Name="lower_open_head";
s0.AddArc(new Vec2D(36.865156790260798,-17.326086956521735),new Vec2D(45.744452409122367,-9.7306508167957801),new Vec2D(57.10561814845218,-7)).Name="lower_shoulder";
s0.AddLine(new Vec2D(57.10561814845218,-7),new Vec2D(192.78362646751219,-7)).Name="lower_grip";
s0.AddArc(new Vec2D(192.78362646751219,-7),new Vec2D(200.86653023616697,-8.1920409828225331),new Vec2D(208.26120882250407,-11.666666666666668)).Name="lower_ring_neck";
s0.AddArc(new Vec2D(208.26120882250407,-11.666666666666666),new Vec2D(230,0),new Vec2D(208.26120882250407,11.666666666666666)).Name="ring_outline";
s0.AddArc(new Vec2D(208.26120882250407,11.666666666666668),new Vec2D(200.86653023616697,8.1920409828225331),new Vec2D(192.78362646751219,7)).Name="upper_ring_neck";
s0.AddLine(new Vec2D(192.78362646751219,7),new Vec2D(47.194469986812933,7)).Name="upper_grip";
s0.AddArc(new Vec2D(47.194469986812933,7),new Vec2D(39.37517945967263,8.2542910054853493),new Vec2D(32.340502194729844,11.891304347826082)).Name="upper_shoulder";
var s1=api.GetPlotterSketcher(new Plane3D(new Vec3D(0,35,0),new Vec3D(0,-1,0),new Vec3D(1,0,0),new Vec3D(0,0,1)),"forging_side");
s1.SolveAfterEveryConstraint=false;
s1.AddLine(new Vec2D(-10,3.25),new Vec2D(35,3.25)).Name="jaw_top";
s1.AddArc(new Vec2D(35,3.25),new Vec2D(38.00751662505796,3.1436170596417341),new Vec2D(41.000000000000007,2.8249999999999957)).Name="head_runout_top_a";
s1.AddArc(new Vec2D(41,2.8250000000000028),new Vec2D(43.992483374942047,2.5063829403582645),new Vec2D(47,2.3999999999999986)).Name="head_runout_top_b";
s1.AddLine(new Vec2D(47,2.3999999999999999),new Vec2D(183.59812224850006,2.3999999999999999)).Name="grip_top";
s1.AddArc(new Vec2D(183.59812224850006,2.4000000000000004),new Vec2D(185.16443655514067,2.5026616635142762),new Vec2D(186.7039507897303,2.8088900845311802)).Name="upper_crank";
s1.AddLine(new Vec2D(186.7039507897303,2.8088900845311802),new Vec2D(245,18.429269392347933)).Name="ring_top";
s1.AddLine(new Vec2D(245,18.429269392347933),new Vec2D(245,9.111783768657185)).Name="end";
s1.AddLine(new Vec2D(245,9.111783768657185),new Vec2D(203.56343665980614,-1.9911099154688197)).Name="ring_bottom";
s1.AddArc(new Vec2D(203.56343665980614,-1.9911099154688205),new Vec2D(202.02392242521651,-2.2973383364857245),new Vec2D(200.45760811857591,-2.4000000000000004)).Name="lower_crank";
s1.AddLine(new Vec2D(200.45760811857591,-2.3999999999999999),new Vec2D(47,-2.3999999999999999)).Name="grip_bottom";
s1.AddArc(new Vec2D(47,-2.3999999999999986),new Vec2D(43.992483374942047,-2.5063829403582645),new Vec2D(41,-2.8250000000000028)).Name="head_runout_bottom_b";
s1.AddArc(new Vec2D(41.000000000000007,-2.8249999999999957),new Vec2D(38.00751662505796,-3.1436170596417341),new Vec2D(35,-3.25)).Name="head_runout_bottom_a";
s1.AddLine(new Vec2D(35,-3.25),new Vec2D(-10,-3.25)).Name="jaw_bottom";
s1.AddLine(new Vec2D(-10,-3.25),new Vec2D(-10,3.25)).Name="start";
var a=api.Extrude(s0,40,name:"plan_blank");
var b=api.Extrude(s1,70,name:"side_die");
var blank=api.Boolean(a,b,CSG.BooleanOp.Intersect,"forging");
double Volume(AnchorMesh mesh)=>Math.Abs(mesh.Mesh.Triangles.Sum(t=>Vec3DOps.Dot(mesh.Mesh.Positions[t.A],Vec3DOps.Cross(mesh.Mesh.Positions[t.B],mesh.Mesh.Positions[t.C]))))/6;
var before=Volume(blank);
blank.EnsureCoplanarPostProcessed();
var selected=edge=="all" ? blank.GroupEdges.Select(e=>e.Name).Where(n=>n.Contains("plan_blank-")&&n.Contains("side_die-")).ToList() : new List<string>{edge};
var rounded=api.Fillet(blank,selected,.55,.035,"rounded");
Assert.NotEmpty(rounded.Mesh.Triangles);
Assert.InRange(Volume(rounded),0,before-1);
Assert.Equal(before,Volume(blank),8);
var incidence=new Dictionary<Int2,int>();
foreach(var triangle in rounded.Mesh.Triangles)
 foreach(var pair in new[]{(triangle.A,triangle.B),(triangle.B,triangle.C),(triangle.C,triangle.A)})
 { var key=new Int2(Math.Min(pair.Item1,pair.Item2),Math.Max(pair.Item1,pair.Item2));incidence[key]=incidence.GetValueOrDefault(key)+1; }
Assert.All(incidence.Values,count=>Assert.Equal(2,count));
}
}