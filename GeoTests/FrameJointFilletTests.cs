using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class FrameJointFilletTests
{
    [Theory]
    [InlineData(LoftProfileSamplingSource.TessellatedPolyline)]
    [InlineData(LoftProfileSamplingSource.AnalyticCurveStrip)]
    [InlineData(LoftProfileSamplingSource.AnalyticCurveStrip,true)]
    [InlineData(LoftProfileSamplingSource.AnalyticCurveStrip,true,true)]
    public void TopTubeToHeadTubeRoundProducesWatertightJoinedSurface(LoftProfileSamplingSource sampling,bool hollowDownTube=false,bool blendDownHead=false)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-420,-300,-50),new Vec3D(1420,300,1120)),.2);
        var seat = new Vec3D(266,0,748);
        var seatAxis = (new Vec3D(207,0,951)-seat).Normalized();
        var top = new Vec3D(799,0,832);
        var bottom = new Vec3D(844,0,681);
        var steeringAxis = (top-bottom).Normalized();
        AnchorMesh Tube(string name, (Vec3D center, double depth, double width)[] stations)
        {
            var profiles = new List<PlotterSketcherCoordSys>();
            for (int i=0;i<stations.Length;i++)
            {
                var axis=(stations[Math.Min(i+1,stations.Length-1)].center-stations[Math.Max(i-1,0)].center).Normalized();
                var x=Vec3DOps.Cross(new Vec3D(0,1,0),axis).Normalized();
                var y=Vec3DOps.Cross(axis,x).Normalized();
                var sketch=new PlotterSketcherCoordSys(name+"_"+i,new CoordinateSystem(stations[i].center,x,y,axis),new Vec2D(0,0));
                sketch.AddEllipse(new Vec2D(0,0),new Vec2D(stations[i].depth,0),stations[i].width);
                profiles.Add(sketch);
            }
            return api.Loft(profiles,new LoftOptions { ProfileSampling=sampling },name);
        }
        var topTube=Tube("frame_top_tube",[
            (seat-seatAxis*18,13,14),(new Vec3D(380,0,764),14,17),
            (new Vec3D(659,0,799),17,20),(top-steeringAxis*24,18,20)]);
        var headTube=Tube("frame_head_tube",[
            (bottom,27,25),((bottom+top)*.5,30,25),(top,25,24)]);
        AnchorMesh blank;
        if(hollowDownTube)
        {
        var bb=new Vec3D(404,0,267);
        (Vec3D center,double depth,double width)[] stations=[(bb,23,26),(new Vec3D(475,0,360),34,29),
            (new Vec3D(685,0,585),32,25),(new Vec3D(831,0,725),27,24)];
        var downOutside=Tube("frame_down_tube",stations);
        var downInside=Tube("frame_down_tube_inner",stations.Select(s => (s.center,s.depth-3,s.width-3)).ToArray());
        var down=api.Boolean(downOutside,downInside,CSG.BooleanOp.Difference,"frame_down_tube");
        var seatTube=Tube("frame_seat_tube",[(bb,23,22),(new Vec3D(337,0,490),23,18),
            (new Vec3D(288,0,670),19,16),(seat,17,16)]);
        var bbSketch=new PlotterSketcherCoordSys("bottom_bracket",new CoordinateSystem(new Vec3D(404,34,267),
            new Vec3D(1,0,0),new Vec3D(0,0,1),new Vec3D(0,-1,0)),new Vec2D(0,0));
        bbSketch.AddCircle(new Vec2D(0,0),25);bbSketch.AddCircle(new Vec2D(0,0),15);
        var bbShell=api.Extrude(bbSketch,68,name:"frame_bottom_bracket");
        blank=api.BatchUnion([topTube,headTube,down,seatTube,bbShell]);
        }
        else blank=api.BatchUnion([topTube,headTube]);
        // Boolean results defer coplanar fusion until first feature/export.
        // Materialize before snapshotting; the fillet must preserve this source geometry.
        blank.EnsureCoplanarPostProcessed();
        MeshTestHelpers.AssertValidMesh(blank.Mesh.Positions,blank.Mesh.Triangles);
        var graph=new EdgeGraph(blank.Mesh.Triangles,blank.Mesh.GetTriangleGroups(),blank.Mesh.Positions,blank.Mesh.PrecisionPositions,blank.groupIdToExtendedName);
        Assert.True(graph.TryGetEdge("[frame_top_tube-Side,frame_head_tube-Side]",out var joint));
        Console.WriteLine($"Joint sampling={sampling}: {joint.EdgeSegments.Count} segments; closed={joint.LineStripExact[0]==joint.LineStripExact[^1]}; endpoints {joint.LineStrip.Points[0]} / {joint.LineStrip.Points[^1]}");
        var originalPositions=blank.Mesh.Positions.ToArray();
        var originalPrecisePositions=blank.Mesh.PrecisionPositions.ToArray();
        var originalTriangles=blank.Mesh.Triangles.ToArray();
        double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t =>
            Vec3DOps.Dot(solid.Mesh.Positions[t.A],Vec3DOps.Cross(solid.Mesh.Positions[t.B],solid.Mesh.Positions[t.C]))))/6;
        double sourceVolume=Volume(blank);
        var sourceGroups=blank.groupIdToExtendedName.ToArray();
        var rounded=api.Fillet(blank,["[frame_top_tube-Side,frame_head_tube-Side]"],4,.1,"frame_joint_round");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions,rounded.Mesh.Triangles);
        // A concave tube junction must gain its requested transition material;
        // returning the unchanged valid blank is not a successful fillet.
        Assert.True(Volume(rounded)>Volume(blank));
        Assert.Equal(sourceVolume,Volume(blank));
        Assert.Equal(sourceGroups,blank.groupIdToExtendedName.ToArray());
        Assert.Equal(originalPositions,blank.Mesh.Positions);
        Assert.Equal(originalPrecisePositions.Length,blank.Mesh.PrecisionPositions.Count);
        for (int i=0;i<originalPrecisePositions.Length;i++)
            Assert.True(originalPrecisePositions[i]==blank.Mesh.PrecisionPositions[i]);
        Assert.Equal(originalTriangles,blank.Mesh.Triangles);
        if(!blendDownHead) return;
        // The closed down/head seam needs termination at the adjacent finite head cap.
        rounded.EnsureCoplanarPostProcessed();
        var names=rounded.GroupEdges.Select(e=>e.Name).Where(n=>n.Contains("frame_head_tube-Side")&&n.Contains("frame_down_tube-Side")).ToList();
        Assert.Single(names);
        double originalRoundedVolume=Volume(rounded);
        var transition=api.Fillet(rounded,["[frame_down_tube-Side,frame_head_tube-Side]"],4,.1,"down_head_round");
        MeshTestHelpers.AssertValidMesh(transition.Mesh.Positions,transition.Mesh.Triangles);
        Assert.True(Volume(transition)>Volume(rounded));
        Assert.Equal(originalRoundedVolume,Volume(rounded),8);
        var next=api.CreateCuboid(new Vec3D(1200,0,0),new Vec3D(1201,1,1),"after_fillet");
        Assert.Empty(next.groupIdToExtendedName.Keys.Intersect(transition.groupIdToExtendedName.Keys));
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(transition.Mesh);
        foreach(var station in new[]{(new Vec3D(475,0,360),29.0),(new Vec3D(685,0,585),25.0)})
            foreach(int side in new[]{-1,1})
            {
                var direction=new Vec3D(0,side,0);
                Assert.True(RayMeshExact.TryCast(transition,api.Converter,station.Item1,direction,out var inner));
                Assert.True(RayMeshExact.TryCast(transition,api.Converter,station.Item1+direction*40,-direction,out var outer));
                Assert.InRange(Vec3DOps.Dot(inner.Point-station.Item1,direction),station.Item2-3-.02,station.Item2-3+.02);
                Assert.InRange(Vec3DOps.Dot(outer.Point-station.Item1,direction),station.Item2-.02,station.Item2+.02);
            }
    }
}
