namespace GeoTests;

// Only tests of process-wide registry/counter resets require exclusive access.
// Geometry tests own their CAD instances; cleanup must not reset name counters.
[CollectionDefinition("GlobalCadState", DisableParallelization = true)]
public sealed class GlobalCadStateCollection { }
