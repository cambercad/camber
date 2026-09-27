using System.Globalization;
using System.Text;
using System.Text.Json;
using Geo;
using GeoCore;
using GeoSolver;
namespace GeoPy;

// Explicit writer keeps the small diagnostic schema compatible with Native AOT.
internal static class AssemblyDiagnosticsJson
{
    public static string SerializeConstraints(Assembly assembly)
    {
        using var stream=new System.IO.MemoryStream();
        using(var writer=new Utf8JsonWriter(stream))
        {
            var leaves=assembly.GetLeaves();
            writer.WriteStartArray();
            for(int index=0;index<assembly.GetMateRecords().Count;index++)
            {
                AssemblyMateRecord mate=assembly.GetMateRecords()[index];
                writer.WriteStartObject();
                writer.WriteNumber("index",index);writer.WriteString("kind",mate.Kind.ToString());writer.WriteString("label",mate.Label);
                writer.WritePropertyName("value");if(double.IsFinite(mate.Scalar))writer.WriteNumberValue(mate.Scalar);else writer.WriteNullValue();
                writer.WriteStartArray("entities");foreach(string entity in mate.Entities)writer.WriteStringValue(entity);writer.WriteEndArray();
                Datum(writer,"first",assembly,leaves,mate.PartA,mate.LocalA,mate.DirA,DatumKind(mate.Kind,true));
                Datum(writer,"second",assembly,leaves,mate.PartB,mate.LocalB,mate.DirB,DatumKind(mate.Kind,false));
                writer.WriteEndObject();
            }
            writer.WriteEndArray();
        }
        return Encoding.UTF8.GetString(stream.ToArray());
    }

    private static string DatumKind(AssemblyMateKind kind,bool first) => kind switch {
        AssemblyMateKind.FixPart=>"body",
        AssemblyMateKind.CoincidentPoints or AssemblyMateKind.DistancePoints=>"point",
        AssemblyMateKind.CoincidentAxes or AssemblyMateKind.ParallelAxes or AssemblyMateKind.PerpendicularAxes or AssemblyMateKind.Concentric or AssemblyMateKind.AngleAxes=>"axis",
        AssemblyMateKind.CoincidentPlanes or AssemblyMateKind.ParallelPlanes or AssemblyMateKind.PerpendicularPlanes or AssemblyMateKind.DistancePlanes=>"plane",
        AssemblyMateKind.PointOnPlane or AssemblyMateKind.Contact=>first?"point":"plane",
        _=>"point"};

    private static void Datum(Utf8JsonWriter writer,string name,Assembly assembly,IReadOnlyList<AssemblyLeaf> leaves,AssemblyPart part,Vec3D local,Vec3D direction,string kind)
    {
        writer.WritePropertyName(name);
        if(part==null){writer.WriteNullValue();return;}
        AssemblyLeaf leaf=leaves.FirstOrDefault(item=>RefersTo(item.Part,part));
        Transform pose=assembly.WorldPoseOf(part);
        writer.WriteStartObject();writer.WriteString("kind",kind);writer.WriteString("part",part.Mesh.Name);
        writer.WriteString("path",leaf?.Path??"");
        Vector(writer,"local_point",local);Vector(writer,"local_direction",direction);
        Vector(writer,"world_point",TransformMath.TransformPoint(in pose,local));
        Vector(writer,"world_direction",TransformMath.TransformDirection(in pose,direction));
        writer.WriteEndObject();
    }

    private static bool RefersTo(AssemblyPart candidate,AssemblyPart requested)
    {
        if(!ReferenceEquals(candidate.DefinitionPart,requested.DefinitionPart))return false;
        if(requested.OccurrenceContext==null)
            return ReferenceEquals(candidate.Assembly,requested.Assembly);
        if(!ReferenceEquals(candidate.OccurrenceContext,requested.OccurrenceContext) ||
            candidate.DefinitionOccurrencePath.Count!=requested.DefinitionOccurrencePath.Count)return false;
        for(int i=0;i<candidate.DefinitionOccurrencePath.Count;i++)
            if(!ReferenceEquals(candidate.DefinitionOccurrencePath[i],requested.DefinitionOccurrencePath[i]))return false;
        return true;
    }

    private static void Vector(Utf8JsonWriter writer,string name,Vec3D value)
    {writer.WriteStartArray(name);writer.WriteNumberValue(value.X);writer.WriteNumberValue(value.Y);writer.WriteNumberValue(value.Z);writer.WriteEndArray();}
    public static string Serialize(SolveResult result,double characteristicLength,IReadOnlyList<AssemblyMateResidual> mates)
    {
        using var stream=new System.IO.MemoryStream();
        using(var writer=new Utf8JsonWriter(stream))
        {
            writer.WriteStartObject();
            writer.WriteBoolean("converged",result.Converged);
            Number(writer,"sum_squared_error",result.SumOfSquaredErrors);
            writer.WriteNumber("num_parameters",result.NumParameters);
            writer.WriteNumber("num_equations",result.NumEquations);
            writer.WriteString("message",result.Message??"");
            Number(writer,"characteristic_length",characteristicLength);
            writer.WriteStartArray("mates");
            foreach(var mate in mates)
            {
                writer.WriteStartObject();
                writer.WriteNumber("index",mate.Index);writer.WriteString("kind",mate.Kind);writer.WriteString("label",mate.Label);
                writer.WriteStartArray("entities");foreach(string entity in mate.Entities)writer.WriteStringValue(entity);writer.WriteEndArray();
                writer.WriteStartArray("residuals");foreach(double residual in mate.Residuals)Value(writer,residual);writer.WriteEndArray();
                Number(writer,"max_residual",mate.MaxResidual);Number(writer,"tolerance",mate.Tolerance);
                writer.WriteBoolean("satisfied",mate.Satisfied);
                writer.WriteEndObject();
            }
            writer.WriteEndArray();writer.WriteEndObject();
        }
        return Encoding.UTF8.GetString(stream.ToArray());
    }
    private static void Number(Utf8JsonWriter writer,string name,double value)
    {
        writer.WritePropertyName(name);Value(writer,value);
    }
    private static void Value(Utf8JsonWriter writer,double value)
    {
        if(double.IsFinite(value))writer.WriteNumberValue(value);
        else writer.WriteStringValue(value.ToString(CultureInfo.InvariantCulture));
    }
}
