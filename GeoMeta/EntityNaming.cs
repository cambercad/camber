using System.Globalization;
using System.Text.RegularExpressions;

namespace GeoMeta
{
    /// <summary>Thrown when a name is duplicated at registration or ambiguous during lookup.</summary>
    public sealed class NameCollisionException : Exception
    {
        public NameCollisionException(string message) : base(message) { }
    }

    /// <summary>
    /// Single source of truth for entity naming conventions, string composition, parsing,
    /// rename rules, and auto-name generation across Geo, Curves, GeoSolver, and scripting.
    /// </summary>
    public static class EntityNaming
    {
        #region Constants and defaults

        public static class DefaultPoints
        {
            public const string Origin = "Origin";
        }

        public static class DefaultPlanes
        {
            public const string OriginXY = "OriginXY";
            public const string OriginYZ = "OriginYZ";
            public const string OriginZX = "OriginZX";
        }

        public const string ExtrudeTopSegment = "ExtrudeTop";
        public const string ExtrudeBottomSegment = "ExtrudeBottom";

        public const string ObjAutoGroupPrefix = "ObjAutoGroup_";
        public const string OffAutoGroupPrefix = "OffAutoGroup_";
        public const string StlAutoGroupPrefix = "StlAutoGroup_";

        public const string BlendEdgePrefix = "BlendEdge_";
        public const string BlendCornerPrefix = "BlendCorner_";
        public const string ChamferEdgePrefix = "ChamferEdge_";
        public const string ChamferCornerPrefix = "ChamferCorner_";

        public const string SketchCurveCenterParam = "center";
        public const string SketchCurveControlVertexPrefix = "cv";
        public const string SketchCurveInOffsetParam = "in_offset";
        public const string SketchCurveOutOffsetParam = "out_offset";
        public const string SketchCurveStartCapParam = "start_cap";
        public const string SketchCurveEndCapParam = "end_cap";
        public const string SketchCurveCapParam = "cap";

        /// <summary>
        /// True if the patch name is a fillet/chamfer surface (implicit boundary curves use lex start).
        /// </summary>
        public static bool IsBlendOrChamferPatchName(string patchName)
        {
            if (string.IsNullOrEmpty(patchName))
                return false;
            return patchName.Contains(BlendEdgePrefix, StringComparison.Ordinal)
                || patchName.Contains(BlendCornerPrefix, StringComparison.Ordinal)
                || patchName.Contains(ChamferEdgePrefix, StringComparison.Ordinal)
                || patchName.Contains(ChamferCornerPrefix, StringComparison.Ordinal);
        }

        public static class SketchCurvePrefixes
        {
            public const string Line = "Line";
            public const string Arc = "Arc";
            public const string Circle = "Circle";
            public const string Bezier = "Bezier";
            public const string Hermite = "Hermite";
            public const string Curve = "Curve";
        }

        #endregion

        #region Auto-name generation

        private static readonly object GlobalCounterLock = new();
        private static readonly Dictionary<string, int> GlobalNameCounters = new();

        /// <summary>Resets global auto-name counters (call from <c>GeoAPI.Clear()</c>).</summary>
        public static void ResetGlobalNameCounters()
        {
            lock (GlobalCounterLock)
            {
                GlobalNameCounters.Clear();
            }
        }

        /// <summary>Returns the next unique name: <c>{prefix}1</c>, <c>{prefix}2</c>, …</summary>
        public static string GenerateGlobalName(string prefix)
        {
            lock (GlobalCounterLock)
            {
                GlobalNameCounters.TryGetValue(prefix, out int n);
                n++;
                GlobalNameCounters[prefix] = n;
                return prefix + n;
            }
        }

        /// <summary>
        /// Returns the next unique name within a scoped counter dictionary (e.g. per-sketch curve names).
        /// </summary>
        public static string GenerateScopedName(string prefix, Dictionary<string, int> counters)
        {
            if (!counters.TryGetValue(prefix, out int n))
                n = 0;
            n++;
            counters[prefix] = n;
            return prefix + n;
        }

        #endregion

        #region Patch name formatters

        /// <summary>Readable structural face reference; escaped atoms cannot collide with delimiters.</summary>
        public static string FormatFaceProvenance(IEnumerable<string> roots, IEnumerable<string> boundaries)
        {
            string Atoms(IEnumerable<string> values) => string.Join("&", values.Distinct(StringComparer.Ordinal)
                .Order(StringComparer.Ordinal).Select(Uri.EscapeDataString));
            return Atoms(roots) + "{" + Atoms(boundaries) + "}";
        }

        public static string ExtrudeBottom(string meshName) => meshName + "-" + ExtrudeBottomSegment;
        public static string ExtrudeTop(string meshName) => meshName + "-" + ExtrudeTopSegment;
        public static string ExtrudeSide(string meshName, string curveName) => meshName + "-" + curveName;
        public static string ExtrudeSideWithGuide(string meshName, string contourCurveName, string guideCurveName) =>
            meshName + "-" + contourCurveName + "-" + guideCurveName;

        public static string RevolveSide(string meshName, string profileName) => meshName + "-" + profileName;
        public static string RevolveStartCap(string meshName) => meshName + "-StartCap";
        public static string RevolveEndCap(string meshName) => meshName + "-EndCap";

        public static string LoftSide(string meshName) => meshName + "-Side";
        public static string LoftSide(string meshName, string curveName) => LoftSide(meshName) + "-" + curveName;
        public static string LoftStartCap(string meshName) => meshName + "-StartCap";
        public static string LoftEndCap(string meshName) => meshName + "-EndCap";

        public static string BlendEdge(string sourceEdgeName) => BlendEdgePrefix + sourceEdgeName;
        public static string BlendCorner(int index) => BlendCornerPrefix + index;
        public static string ChamferEdge(string sourceEdgeName) => ChamferEdgePrefix + sourceEdgeName;
        public static string ChamferCorner(int index) => ChamferCornerPrefix + index;

        public static string ImportAutoGroup(string prefix, int groupIndex) => prefix + groupIndex;

        /// <summary>Decorates a patch name with a prefix or suffix affix.</summary>
        public static string DecoratePatchName(string name, string affix, bool affixIsPrefix)
        {
            if (string.IsNullOrEmpty(affix))
                return name;
            return affixIsPrefix ? affix + name : name + affix;
        }

        /// <summary>
        /// Group edge base name: <c>[{patchA},{patchB}]</c> or with disambiguation index <c>_{i}</c>.
        /// </summary>
        public static string FormatGroupEdgeName(string patchA, string patchB, int index = 0)
        {
            string baseName = "[" + patchA + "," + patchB + "]";
            return index > 0 ? baseName + "_" + index : baseName;
        }

        /// <summary>
        /// Patch face-component name after a connected-component split:
        /// index 0 keeps the origin name; index &gt; 0 appends <c>_{i}</c>
        /// (same disambiguator as <see cref="FormatGroupEdgeName"/>).
        /// </summary>
        public static string FormatPatchComponentName(string baseName, int index = 0)
        {
            if (index <= 0)
                return baseName;
            return baseName + "_" + index;
        }

        /// <summary>
        /// Legacy programmatic edge-point name (underscore-separated patch names).
        /// </summary>
        public static string FormatLegacyEdgePointName(string patchA, string patchB, int edgeIndex, double uniformParam) =>
            patchA + "_" + patchB + "_" + edgeIndex + "_" + uniformParam.ToString("F6", CultureInfo.InvariantCulture);

        /// <summary>
        /// Surface point anchor: <c>[{patch}]@X.XXX,Y.YYY</c> (3 decimal places).
        /// </summary>
        public static string FormatSurfacePointAddress(string patchName, double uniformX, double uniformY, int index = 0)
        {
            string bracket = index > 0
                ? "[" + patchName + "]_" + index
                : "[" + patchName + "]";
            return bracket + "@" +
                   uniformX.ToString("F3", CultureInfo.InvariantCulture) + "," +
                   uniformY.ToString("F3", CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// Edge point anchor: <c>[{patchA},{patchB}]@U.UUU</c> (3 decimal places).
        /// </summary>
        public static string FormatEdgePointAddress(string patchA, string patchB, double uniform, int edgeIndex = 0) =>
            FormatGroupEdgeName(patchA, patchB, edgeIndex) + "@" +
            uniform.ToString("F3", CultureInfo.InvariantCulture);

        /// <summary>Throws if <paramref name="contourSegmentName"/> is reserved for extrude caps.</summary>
        public static void ValidateContourSegmentName(string contourSegmentName)
        {
            if (contourSegmentName == ExtrudeTopSegment || contourSegmentName == ExtrudeBottomSegment)
                throw new Exception(contourSegmentName + " is a reserved name");
        }

        /// <summary>
        /// Returns the auto-name prefix for a sketch curve type (e.g. Line2D → "Line").
        /// </summary>
        public static string GetSketchCurveTypePrefix(string curveTypeName)
        {
            return curveTypeName switch
            {
                "Line2D" or "CLine2D" => SketchCurvePrefixes.Line,
                "Arc2D" or "CArc2D" => SketchCurvePrefixes.Arc,
                "Circle2D" or "CCircle2D" => SketchCurvePrefixes.Circle,
                "Ellipse2D" or "CEllipse2D" => "Ellipse",
                "Bezier2D" => SketchCurvePrefixes.Bezier,
                "CubicHermiteSpline2D" => SketchCurvePrefixes.Hermite,
                "SampledCurve" or "OffsetSampledCurve2D" => SketchCurvePrefixes.Curve,
                _ => SketchCurvePrefixes.Curve,
            };
        }

        /// <summary>Owner-qualified address: <c>{owner}:{local}</c>. Empty owner returns <paramref name="localName"/>.</summary>
        public static string Qualify(string ownerName, string localName)
        {
            if (string.IsNullOrEmpty(localName))
                return "";
            if (string.IsNullOrEmpty(ownerName))
                return localName;
            return ownerName + ":" + localName;
        }

        /// <summary>Sketch curve sample: <c>Line1@0.500</c> (3 decimal places).</summary>
        public static string FormatSketchCurveAddress(string curveName, double uniform) =>
            curveName + "@" + uniform.ToString("F3", CultureInfo.InvariantCulture);

        /// <summary>Circle/arc center handle: <c>Circle1@center</c>.</summary>
        public static string FormatSketchCurveCenter(string curveName) =>
            curveName + "@" + SketchCurveCenterParam;

        /// <summary>Bézier/B-spline control vertex: <c>Bezier1@cv2</c>.</summary>
        public static string FormatSketchCurveControlVertex(string curveName, int index) =>
            curveName + "@" + SketchCurveControlVertexPrefix + index;

        /// <summary>
        /// Offset child of a sketch curve: <c>north@out_offset</c>, extra fragments <c>north@out_offset_2</c>.
        /// </summary>
        public static string FormatSketchOffsetCurve(string sourceName, bool outward, int occurrence = 0)
        {
            string param = outward ? SketchCurveOutOffsetParam : SketchCurveInOffsetParam;
            return FormatSketchOffsetChild(sourceName, param, occurrence);
        }

        /// <summary>
        /// Outline end cap of a sketch curve: <c>north@end_cap</c> / <c>north@start_cap</c>.
        /// </summary>
        public static string FormatSketchOffsetEndCap(string sourceName, bool atStart, int occurrence = 0)
        {
            string param = atStart ? SketchCurveStartCapParam : SketchCurveEndCapParam;
            return FormatSketchOffsetChild(sourceName, param, occurrence);
        }

        /// <summary>
        /// Join cap between two named offset curves: <c>cap[h@out_offset,v@out_offset]</c>.
        /// Pair is sorted so the same two curves always produce the same name.
        /// </summary>
        public static string FormatSketchOffsetJoinCap(string curveA, string curveB, int occurrence = 0)
        {
            string a = curveA ?? "";
            string b = curveB ?? "";
            if (string.CompareOrdinal(a, b) > 0)
            {
                string swap = a;
                a = b;
                b = swap;
            }
            string name = SketchCurveCapParam + FormatGroupEdgeName(a, b);
            if (occurrence > 0)
                name += "_" + (occurrence + 1).ToString(CultureInfo.InvariantCulture);
            return name;
        }

        public static string FormatSketchOffsetChild(string sourceName, string param, int occurrence = 0)
        {
            string name = sourceName + "@" + param;
            if (occurrence > 0)
                name += "_" + (occurrence + 1).ToString(CultureInfo.InvariantCulture);
            return name;
        }

        /// <summary>
        /// True for <c>name@in_offset</c>, <c>@out_offset</c>, <c>@start_cap</c>, <c>@end_cap</c>,
        /// and join caps <c>cap[a,b]</c> (optional <c>_2</c> suffix).
        /// </summary>
        public static bool IsSketchOffsetChildName(string local)
        {
            if (string.IsNullOrEmpty(local))
                return false;
            if (local.StartsWith(SketchCurveCapParam + "[", StringComparison.Ordinal))
                return true;
            int at = local.LastIndexOf('@');
            if (at < 0 || at >= local.Length - 1)
                return false;
            string param = local.Substring(at + 1);
            int us = param.LastIndexOf('_');
            if (us > 0 && us < param.Length - 1)
            {
                bool digits = true;
                for (int i = us + 1; i < param.Length; i++)
                {
                    if (!char.IsDigit(param[i]))
                    {
                        digits = false;
                        break;
                    }
                }
                if (digits)
                    param = param.Substring(0, us);
            }
            return param == SketchCurveInOffsetParam
                || param == SketchCurveOutOffsetParam
                || param == SketchCurveStartCapParam
                || param == SketchCurveEndCapParam
                || param == SketchCurveCapParam;
        }

        /// <summary>
        /// Interactive sketch handle. Line 0/1 ends, 3 mid; circle 2 center, 3–6 E/N/W/S;
        /// arc 0/1 ends, 2 center, 3 mid. Unknown role returns an empty string.
        /// </summary>
        public static string FormatSketchHandleAddress(string curveName, string kind, int role)
        {
            if (string.IsNullOrEmpty(curveName))
                return "";
            string key = (kind ?? "").Trim().ToLowerInvariant();
            if (key == "circle")
            {
                if (role == 2)
                    return FormatSketchCurveCenter(curveName);
                if (role == 3)
                    return FormatSketchCurveAddress(curveName, 0.0);
                if (role == 4)
                    return FormatSketchCurveAddress(curveName, 0.25);
                if (role == 5)
                    return FormatSketchCurveAddress(curveName, 0.5);
                if (role == 6)
                    return FormatSketchCurveAddress(curveName, 0.75);
                return "";
            }
            if (key == "arc")
            {
                if (role == 0)
                    return FormatSketchCurveAddress(curveName, 0.0);
                if (role == 1)
                    return FormatSketchCurveAddress(curveName, 1.0);
                if (role == 2)
                    return FormatSketchCurveCenter(curveName);
                if (role == 3)
                    return FormatSketchCurveAddress(curveName, 0.5);
                return "";
            }
            if (role == 0)
                return FormatSketchCurveAddress(curveName, 0.0);
            if (role == 1)
                return FormatSketchCurveAddress(curveName, 1.0);
            if (role == 3)
                return FormatSketchCurveAddress(curveName, 0.5);
            return "";
        }

        #endregion

        #region Qualified mesh addresses

        public readonly struct QualifiedMeshAddress
        {
            public string MeshName { get; init; }
            public string LocalAddress { get; init; }
            public bool HasMeshQualifier => MeshName != null;
        }

        /// <summary>
        /// Splits <c>{meshName}:{localAddress}</c> when the suffix is a mesh anchor (bracket address or patch name).
        /// Sketch curve names <c>{sketchName}:{curveName}</c> are not split here (suffix has no leading '[').
        /// </summary>
        public static bool TryParseQualifiedMeshAddress(
            string name,
            Func<string, bool> isKnownMesh,
            out QualifiedMeshAddress result)
        {
            result = default;
            if (string.IsNullOrEmpty(name))
                return false;

            int colon = name.IndexOf(':');
            if (colon <= 0 || colon >= name.Length - 1)
                return false;

            string prefix = name.Substring(0, colon);
            string suffix = name.Substring(colon + 1);
            if (!LooksLikeMeshLocalAddress(suffix))
                return false;
            if (isKnownMesh != null && !isKnownMesh(prefix))
                return false;

            result = new QualifiedMeshAddress { MeshName = prefix, LocalAddress = suffix };
            return true;
        }

        /// <summary>True when <paramref name="localAddress"/> is a 3D mesh anchor, not a sketch curve name.</summary>
        public static bool LooksLikeMeshLocalAddress(string localAddress)
        {
            if (string.IsNullOrEmpty(localAddress))
                return false;
            if (localAddress.StartsWith("[", StringComparison.Ordinal))
                return true;
            // Bracket edge / surface point forms
            if (TryParseEdgePointAddress(localAddress, out _) ||
                TryParseSurfacePointAddress(localAddress, out _) ||
                TryParseGroupEdgeAddress(localAddress, out _, requireFullMatch: true))
                return true;
            // Exact patch name (no @ — sketch curve addresses always contain @)
            return !localAddress.Contains('@');
        }

        #endregion

        #region Group-edge matching (EdgeGraph / blend lookup)

        /// <summary>
        /// Whether <paramref name="candidateEdgeName"/> satisfies a group-edge query.
        /// Supports exact match and base-name prefix (<c>[A,B]</c> matches <c>[A,B]_0</c>).
        /// </summary>
        public static bool MatchesGroupEdgeName(string candidateEdgeName, string queryName)
        {
            if (string.IsNullOrEmpty(candidateEdgeName) || string.IsNullOrEmpty(queryName))
                return false;
            if (candidateEdgeName == queryName)
                return true;
            if (!TryParseGroupEdgeAddress(candidateEdgeName, out var candidate, requireFullMatch: true) ||
                !TryParseGroupEdgeAddress(queryName, out var query, requireFullMatch: true))
                return false;
            bool sameFaces = (candidate.PatchA == query.PatchA && candidate.PatchB == query.PatchB) ||
                             (candidate.PatchA == query.PatchB && candidate.PatchB == query.PatchA);
            // The two incident faces define an unordered pair. A suffix still
            // selects a particular connected edge between that pair.
            return sameFaces && (queryName.TrimEnd().EndsWith("]", StringComparison.Ordinal) ||
                                 candidate.EdgeIndex == query.EdgeIndex);
        }

        #endregion

        #region Legacy edge-point format

        private static readonly Regex LegacyEdgePointTailRegex = new(
            @"_(?<index>\d+)_(?<uniform>\d+(?:\.\d+)?)$",
            RegexOptions.Compiled);

        /// <summary>
        /// Parses legacy <c>{patchA}_{patchB}_{edgeIndex}_{uniform}</c> using known patch names for disambiguation.
        /// </summary>
        public static bool TryParseLegacyEdgePointAddress(
            string name,
            IEnumerable<string> knownPatchNames,
            out EdgePointAddress result)
        {
            result = default;
            if (string.IsNullOrEmpty(name))
                return false;

            Match tailMatch = LegacyEdgePointTailRegex.Match(name);
            if (!tailMatch.Success)
                return false;

            int edgeIndex = int.Parse(tailMatch.Groups["index"].Value, CultureInfo.InvariantCulture);
            double uniform = double.Parse(tailMatch.Groups["uniform"].Value, CultureInfo.InvariantCulture);
            string head = name.Substring(0, tailMatch.Index);

            var patchSet = knownPatchNames as HashSet<string> ?? new HashSet<string>(knownPatchNames);
            foreach (string patchA in patchSet)
            {
                string prefix = patchA + "_";
                if (!head.StartsWith(prefix, StringComparison.Ordinal))
                    continue;
                string patchB = head.Substring(prefix.Length);
                if (patchSet.Contains(patchB))
                {
                    result = new EdgePointAddress
                    {
                        PatchA = patchA,
                        PatchB = patchB,
                        EdgeIndex = edgeIndex,
                        Uniform = uniform,
                    };
                    return true;
                }
            }

            return false;
        }

        #endregion

        #region Anchor / address parsers

        // Accepts fixed 3-decimal params (0.500) and flexible forms (0.5).
        private static readonly Regex SurfacePointAddressRegex = new(
            @"\[(?<surf>[^,]+)\](_(?<index>\d+))?@(?<uniformX>\d+(?:\.\d+)?),(?<uniformY>\d+(?:\.\d+)?)",
            RegexOptions.Compiled);

        private static readonly Regex EdgePointAddressRegex = new(
            @"\[(?<surfA>[^,]+),(?<surfB>[^]]+)\](_(?<index>\d+))?@(?<uniform>\d+(?:\.\d+)?)",
            RegexOptions.Compiled);

        private static readonly Regex GroupEdgeAddressRegex = new(
            @"\[(?<surfA>[^,]+),(?<surfB>[^]]+)\](_(?<index>\d+))?",
            RegexOptions.Compiled);

        private static readonly Regex SketchCurveAddressRegex = new(
            @"^(?<curveName>.+)@(?<param>\d+\.?\d*|center|cv\d+)$",
            RegexOptions.Compiled);

        public readonly struct SurfacePointAddress
        {
            public string PatchName { get; init; }
            public int Index { get; init; }
            public double UniformX { get; init; }
            public double UniformY { get; init; }
        }

        public readonly struct EdgePointAddress
        {
            public string PatchA { get; init; }
            public string PatchB { get; init; }
            public int EdgeIndex { get; init; }
            public double Uniform { get; init; }
        }

        public readonly struct GroupEdgeAddress
        {
            public string PatchA { get; init; }
            public string PatchB { get; init; }
            public int EdgeIndex { get; init; }
        }

        public readonly struct SketchCurveAddress
        {
            public string CurveName { get; init; }
            public bool IsCenter { get; init; }
            /// <summary>True when the address is a Bézier/B-spline control vertex (<c>@cvN</c>).</summary>
            public bool IsControlVertex { get; init; }
            public int ControlVertexIndex { get; init; }
            public double Uniform { get; init; }
        }

        public readonly struct QualifiedCurveName
        {
            public string SketchName { get; init; }
            public string CurveName { get; init; }
            public bool HasSketchQualifier => SketchName != null;
        }

        public readonly struct QualifiedSketchCurveAddress
        {
            public string SketchName { get; init; }
            public string LocalAddress { get; init; }
            public bool HasSketchQualifier => SketchName != null;
        }

        public static bool TryParseSurfacePointAddress(string name, out SurfacePointAddress result)
        {
            result = default;
            Match match = SurfacePointAddressRegex.Match(name);
            if (!match.Success)
                return false;

            int index = 0;
            var indexGroup = match.Groups["index"];
            if (indexGroup.Success)
                index = int.Parse(indexGroup.Value, CultureInfo.InvariantCulture);

            result = new SurfacePointAddress
            {
                PatchName = match.Groups["surf"].Value,
                Index = index,
                UniformX = double.Parse(match.Groups["uniformX"].Value, CultureInfo.InvariantCulture),
                UniformY = double.Parse(match.Groups["uniformY"].Value, CultureInfo.InvariantCulture),
            };
            return true;
        }

        public static bool TryParseEdgePointAddress(string name, out EdgePointAddress result)
        {
            result = default;
            Match match = EdgePointAddressRegex.Match(name);
            if (!match.Success)
                return false;

            int edgeIndex = 0;
            var indexGroup = match.Groups["index"];
            if (indexGroup.Success)
                edgeIndex = int.Parse(indexGroup.Value, CultureInfo.InvariantCulture);

            result = new EdgePointAddress
            {
                PatchA = match.Groups["surfA"].Value,
                PatchB = match.Groups["surfB"].Value,
                EdgeIndex = edgeIndex,
                Uniform = double.Parse(match.Groups["uniform"].Value, CultureInfo.InvariantCulture),
            };
            return true;
        }

        /// <summary>
        /// Parses a group-edge address. When <paramref name="requireFullMatch"/> is true the entire
        /// string must match (no trailing characters).
        /// </summary>
        public static bool TryParseGroupEdgeAddress(string name, out GroupEdgeAddress result, bool requireFullMatch = false)
        {
            result = default;
            name = name.Trim();
            Match match = GroupEdgeAddressRegex.Match(name);
            if (!match.Success)
                return false;
            if (requireFullMatch && name.Length != match.Length)
                return false;

            int edgeIndex = 0;
            var indexGroup = match.Groups["index"];
            if (indexGroup.Success)
                edgeIndex = int.Parse(indexGroup.Value, CultureInfo.InvariantCulture);

            result = new GroupEdgeAddress
            {
                PatchA = match.Groups["surfA"].Value,
                PatchB = match.Groups["surfB"].Value,
                EdgeIndex = edgeIndex,
            };
            return true;
        }

        public static bool TryParseSketchCurveAddress(string name, out SketchCurveAddress result)
        {
            result = default;
            Match match = SketchCurveAddressRegex.Match(name);
            if (!match.Success)
                return false;

            string paramValue = match.Groups["param"].Value;
            bool isCenter = paramValue == SketchCurveCenterParam;
            int cvIndex = 0;
            bool isCv = !isCenter &&
                        paramValue.StartsWith(SketchCurveControlVertexPrefix, StringComparison.Ordinal) &&
                        int.TryParse(paramValue.AsSpan(SketchCurveControlVertexPrefix.Length),
                            NumberStyles.Integer, CultureInfo.InvariantCulture, out cvIndex);
            if (isCv && cvIndex < 0)
                return false;

            result = new SketchCurveAddress
            {
                CurveName = match.Groups["curveName"].Value,
                IsCenter = isCenter,
                IsControlVertex = isCv,
                ControlVertexIndex = isCv ? cvIndex : 0,
                Uniform = isCenter || isCv ? 0 : double.Parse(paramValue, CultureInfo.InvariantCulture),
            };
            return true;
        }

        /// <summary>Parses <c>{sketchName}:{curveName}</c> or bare <c>{curveName}</c>.</summary>
        public static QualifiedCurveName ParseQualifiedCurveName(string name)
        {
            int colonIndex = name.IndexOf(':');
            if (colonIndex < 0)
                return new QualifiedCurveName { CurveName = name };
            return new QualifiedCurveName
            {
                SketchName = name.Substring(0, colonIndex),
                CurveName = name.Substring(colonIndex + 1),
            };
        }

        /// <summary>Parses <c>{sketchName}:{curveName}@{param}</c> or bare <c>{curveName}@{param}</c>.</summary>
        public static bool TryParseQualifiedSketchCurveAddress(string name, out QualifiedSketchCurveAddress result)
        {
            result = default;
            if (string.IsNullOrWhiteSpace(name))
                return false;

            name = name.Trim();
            int colonIndex = name.IndexOf(':');
            if (colonIndex >= 0)
            {
                string localAddress = name.Substring(colonIndex + 1);
                if (!TryParseSketchCurveAddress(localAddress, out _))
                    return false;

                result = new QualifiedSketchCurveAddress
                {
                    SketchName = name.Substring(0, colonIndex),
                    LocalAddress = localAddress,
                };
                return true;
            }

            if (!TryParseSketchCurveAddress(name, out _))
                return false;

            result = new QualifiedSketchCurveAddress { LocalAddress = name };
            return true;
        }

        #endregion

        #region Mesh rename

        /// <summary>
        /// Replaces mesh-name prefix on a patch name (<c>old</c> → <c>new</c>, <c>old-X</c> → <c>new-X</c>).
        /// Does not rewrite <c>old10-X</c> when <paramref name="oldPrefix"/> is <c>old1</c>.
        /// </summary>
        public static string RenameMeshPrefix(string patchName, string oldPrefix, string newPrefix)
        {
            if (string.IsNullOrEmpty(patchName) || string.IsNullOrEmpty(oldPrefix))
                return patchName;
            if (patchName == oldPrefix)
                return newPrefix;
            if (patchName.StartsWith(oldPrefix + "-", StringComparison.Ordinal))
                return newPrefix + patchName.Substring(oldPrefix.Length);
            return patchName;
        }

        /// <summary>
        /// Rewrites every mesh-name token in a patch, group-edge, or blend/chamfer name.
        /// Handles <c>pipe1-south</c>, <c>[pipe1-a,pipe1-b]</c>, and <c>BlendEdge_[pipe1-a,pipe1-b]</c>.
        /// </summary>
        public static string RewriteMeshNameInEntity(string name, string oldPrefix, string newPrefix)
        {
            if (string.IsNullOrEmpty(name) || string.IsNullOrEmpty(oldPrefix) || oldPrefix == newPrefix)
                return name;

            int open = name.IndexOf('[');
            if (open >= 0)
            {
                int close = name.IndexOf(']', open);
                if (close > open)
                {
                    string before = RewriteMeshNameInEntity(name.Substring(0, open), oldPrefix, newPrefix);
                    string inner = name.Substring(open + 1, close - open - 1);
                    string[] parts = inner.Split(',');
                    for (int i = 0; i < parts.Length; i++)
                        parts[i] = RenameMeshPrefix(parts[i], oldPrefix, newPrefix);
                    string after = close + 1 < name.Length
                        ? RewriteMeshNameInEntity(name.Substring(close + 1), oldPrefix, newPrefix)
                        : "";
                    return before + "[" + string.Join(",", parts) + "]" + after;
                }
            }

            return RenameMeshPrefix(name, oldPrefix, newPrefix);
        }

        /// <summary>
        /// Replaces mesh-name prefix inside bracketed group-edge names (and BlendEdge_/ChamferEdge_ wrappers).
        /// </summary>
        public static string RenameBracketedEdgeName(string edgeName, string oldPrefix, string newPrefix)
        {
            return RewriteMeshNameInEntity(edgeName, oldPrefix, newPrefix);
        }

        #endregion

        #region Dictionary helpers

        public static Dictionary<int, string> BuildGroupIdToName(Dictionary<string, int> extendedNameToGroupId)
        {
            var result = new Dictionary<int, string>(extendedNameToGroupId.Count);
            foreach (var kv in extendedNameToGroupId)
            {
                if (result.TryGetValue(kv.Value, out string existingName) && existingName != kv.Key)
                    throw new NameCollisionException(
                        $"Patch name maps to multiple group IDs: '{existingName}' and '{kv.Key}' both use group {kv.Value}.");
                result[kv.Value] = kv.Key;
            }
            return result;
        }

        public static Dictionary<string, int> BuildNameToGroupId(Dictionary<int, string> groupIdToExtendedName)
        {
            var result = new Dictionary<string, int>(groupIdToExtendedName.Count);
            foreach (var kv in groupIdToExtendedName)
            {
                if (result.TryGetValue(kv.Value, out int existingId) && existingId != kv.Key)
                    throw new NameCollisionException(
                        $"Duplicate patch name '{kv.Value}' for group IDs {existingId} and {kv.Key}.");
                result[kv.Value] = kv.Key;
            }
            return result;
        }

        /// <summary>Throws if <paramref name="name"/> already exists in <paramref name="existingNames"/>.</summary>
        public static void EnsureUniqueName(string name, IEnumerable<string> existingNames, string entityKind)
        {
            foreach (string existing in existingNames)
            {
                if (existing == name)
                    throw new NameCollisionException($"{entityKind} name already registered: '{name}'.");
            }
        }

        /// <summary>Throws when more than one owner matches an unqualified address lookup.</summary>
        public static void ThrowIfAmbiguous(string address, IReadOnlyList<string> ownerNames, string entityKind)
        {
            if (ownerNames.Count <= 1)
                return;
            throw new NameCollisionException(
                $"{entityKind} address '{address}' is ambiguous — matches {ownerNames.Count} entities: " +
                string.Join(", ", ownerNames) +
                ". Qualify with '{{meshName}}:' or '{{sketchName}}:' prefix.");
        }

        /// <summary>
        /// Merges two name→groupId maps. Throws if the same name maps to different group IDs.
        /// </summary>
        public static void MergePatchNameMaps(
            Dictionary<string, int> target,
            Dictionary<string, int> source,
            string conflictMessagePrefix = "Group name conflict: ")
        {
            foreach (var kv in source)
            {
                if (!target.ContainsKey(kv.Key))
                {
                    target.Add(kv.Key, kv.Value);
                    continue;
                }

                if (target[kv.Key] != kv.Value)
                    throw new NameCollisionException(conflictMessagePrefix + kv.Key);
            }
        }

        public static Dictionary<TKey, TValue> RewriteDictionaryKeys<TKey, TValue>(
            Dictionary<TKey, TValue> source,
            Func<TKey, TKey> rewriteKey) where TKey : notnull
        {
            var result = new Dictionary<TKey, TValue>(source.Count);
            foreach (var kv in source)
                result[rewriteKey(kv.Key)] = kv.Value;
            return result;
        }

        public static Dictionary<TKey, TValue> RewriteDictionaryValues<TKey, TValue>(
            Dictionary<TKey, TValue> source,
            Func<TValue, TValue> rewriteValue) where TKey : notnull
        {
            var result = new Dictionary<TKey, TValue>(source.Count);
            foreach (var kv in source)
                result[kv.Key] = rewriteValue(kv.Value);
            return result;
        }

        #endregion
    }
}
