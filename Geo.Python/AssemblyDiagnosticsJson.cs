using System.Globalization;
using System.Text;
using System.Text.Json;
using Geo;
using GeoSolver;
namespace GeoPy;

// Explicit writer keeps the small diagnostic schema compatible with Native AOT.
internal static class AssemblyDiagnosticsJson
{
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
