using GeoCore;

namespace Geo;

/// <summary>A recursively placed part in an assembly's current world pose.</summary>
public sealed record AssemblyLeaf(string Path, AssemblyPart Part, Transform Pose);
