using System.Diagnostics;
using CSG;
using Curves;
using Geo;
using GeoCore;

// The first three junctions of the racing bike, with the seat collar omitted,
// without Python or wheel builds.
var api = new GeoAPI(new Box3D(new Vec3D(-420,-300,-50), new Vec3D(1420,300,1120)), .2);
var seat = new Vec3D(266,0,748);
var seatAxis = (new Vec3D(207,0,951)-seat).Normalized();
var top = new Vec3D(799,0,832);
var bottom = new Vec3D(844,0,681);
var steeringAxis = (top-bottom).Normalized();
AnchorMesh Time(string name, Func<AnchorMesh> build)
{
    var watch = Stopwatch.StartNew();
    var result = build();
    Console.WriteLine($"{name}: {watch.Elapsed.TotalSeconds:F3}s, {result.Mesh.Triangles.Count} triangles");
    return result;
}
AnchorMesh Tube(string name, (Vec3D center, double depth, double width)[] stations)
{
    var profiles = new List<PlotterSketcherCoordSys>();
    for (int i=0; i<stations.Length; i++)
    {
        var axis=(stations[Math.Min(i+1,stations.Length-1)].center-stations[Math.Max(i-1,0)].center).Normalized();
        var x=Vec3DOps.Cross(new Vec3D(0,1,0),axis).Normalized();
        var y=Vec3DOps.Cross(axis,x).Normalized();
        var sketch=new PlotterSketcherCoordSys(name+"_"+i, new CoordinateSystem(stations[i].center,x,y,axis), new Vec2D(0,0));
        sketch.AddEllipse(new(0,0),new(stations[i].depth,0),stations[i].width);
        profiles.Add(sketch);
    }
    return api.Loft(profiles, new LoftOptions(), name);
}
var total = Stopwatch.StartNew();
(Vec3D center,double depth,double width)[] stations = [
    (new(404,0,267),23,26),(new(475,0,360),34,29),
    (new(685,0,585),32,25),(new(831,0,725),27,24)];
var down = Time("down tube", () => api.Boolean(Tube("frame_down_tube",stations),
    Tube("frame_down_tube_inner",stations.Select(s=>(s.center,s.depth-3,s.width-3)).ToArray()),
    BooleanOp.Subtract,"frame_down_tube"));
var topTube = Time("top tube", () => Tube("frame_top_tube", [
    (seat-seatAxis*18,13,14),(new(380,0,764),14,17),
    (new(659,0,799),17,20),(top-steeringAxis*24,18,20)]));
var head = Time("head tube", () => Tube("frame_head_tube", [
    (bottom,27,25),((bottom+top)*.5,30,25),(top,25,24)]));
var frame = Time("head union", () => api.BatchUnion([down,topTube,head]));
frame = Time("top/head fillet", () => api.Fillet(frame,["[frame_top_tube-Side,frame_head_tube-Side]"],4,.1,"frame_top_head_transition"));
frame = Time("down/head fillet", () => api.Fillet(frame,["[frame_down_tube-Side,frame_head_tube-Side]"],4,.1,"frame_down_head_transition"));
var seatTube = Time("seat tube", () => Tube("frame_seat_tube", [
    (new(404,0,267),23,22),(new(337,0,490),23,18),(new(288,0,670),19,16),(seat,17,16)]));
frame = Time("seat union", () => api.Boolean(frame,seatTube,BooleanOp.Union));
frame = Time("top/seat fillet", () => api.Fillet(frame,["[frame_top_tube-Side,frame_seat_tube-Side]"],6,.1,"frame_top_seat_transition"));
frame.EnsureCoplanarPostProcessed();
bool watertight = MeshAnalysis.IsWatertightMesh(frame.Mesh.PrecisionPositions,frame.Mesh.Triangles);
Console.WriteLine($"Total: {total.Elapsed.TotalSeconds:F3}s; watertight: {watertight}");
return watertight ? 0 : 1;
