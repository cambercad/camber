namespace Geo;

/// <summary>A successful solid-producing operation in a part's chronological feature ledger.</summary>
public sealed record PartOperation(
    int Index,
    string Kind,
    AnchorMesh Result,
    IReadOnlyList<string> Inputs,
    IReadOnlyList<string> Entities,
    string Details);
