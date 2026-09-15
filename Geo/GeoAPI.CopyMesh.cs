using GeoCore;
using GeoMeta;

namespace Geo
{
    /// <summary>
    /// Whether <see cref="GeoAPI.CopyMesh"/> decorates surface patch names with an affix before or after the original name.
    /// </summary>
    public enum SurfacePatchNameAffix
    {
        Prefix,
        Suffix
    }

    public partial class GeoAPI
    {
        private const double CopyMeshIdentityRotationEpsilon = 1e-8;

        /// <summary>
        /// Deep-copies an <see cref="AnchorMesh"/> by the rigid map
        /// <see cref="CoordinateSystem.GetTransform"/> (<paramref name="fromPose"/> → <paramref name="toPose"/>).
        /// Translation-only copies offset precision lattice coordinates by a snapped world translation.
        /// Rotations apply the double matrix to world positions.
        /// Surface metadata is cloned and transformed. Triangulation is preserved.
        /// </summary>
        [APIDescription(@"CopyMesh(source: AnchorMesh, fromPose: CoordinateSystem, toPose: CoordinateSystem, surfaceNameAffix: str, affixKind: SurfacePatchNameAffix, newMeshName: str = None) -> AnchorMesh
Deep-copies a mesh, transforms it by the rigid map fromPose -> toPose, and registers it like other builders.
  source: must come from this same GeoAPI (same converter / operating space).
  fromPose: the source mesh's current placement frame in world space (must match how `source` was built; otherwise the copy is rigidly wrong).
  toPose: target placement frame.
  surfaceNameAffix: string added to every surface patch name (and surface metadata key) to avoid collisions with the source.
  affixKind: SurfacePatchNameAffix.Prefix or SurfacePatchNameAffix.Suffix.
  newMeshName: auto-generated if null/empty.
Translation-only transforms snap to the integer lattice (exact). Rotations apply the rigid matrix to world positions.")]
        public AnchorMesh CopyMesh(
            AnchorMesh source,
            CoordinateSystem fromPose,
            CoordinateSystem toPose,
            string surfaceNameAffix,
            SurfacePatchNameAffix affixKind,
            string newMeshName = null)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));

            source.EnsureCoplanarPostProcessed();

            newMeshName = string.IsNullOrEmpty(newMeshName) ? GenerateName("MeshCopy") : newMeshName;

            MeshNormalUV srcMesh = source.Mesh;
            if (srcMesh.Positions.Count != srcMesh.PrecisionPositions.Count)
                throw new ArgumentException("Source mesh must have one precision position per unique vertex.", nameof(source));

            Mat4D t = CoordinateSystem.GetTransform(fromPose, toPose);
            bool translationOnly = IsApproximatelyIdentityRotation(in t, CopyMeshIdentityRotationEpsilon);
            if (translationOnly)
                SnapTranslationToConverterLattice(ref t, converter);

            var transformedPos = new Vec3D[srcMesh.Positions.Count];
            var transformedPrec = new Rat3Hybrid[srcMesh.Positions.Count];

            if (translationOnly)
            {
                Int3 latticeDelta = converter.ConvertDirection(new Vec3D(t.M14, t.M24, t.M34));
                var delta = new Rat3Hybrid(latticeDelta.X, latticeDelta.Y, latticeDelta.Z);
                for (int vi = 0; vi < srcMesh.Positions.Count; vi++)
                {
                    transformedPrec[vi] = srcMesh.PrecisionPositions[vi] + delta;
                    transformedPos[vi] = converter.Convert(transformedPrec[vi]);
                }
            }
            else
            {
                for (int vi = 0; vi < srcMesh.Positions.Count; vi++)
                {
                    Vec3D pw = t.TransformPoint(srcMesh.Positions[vi]);
                    transformedPos[vi] = pw;
                    Int3 ip = converter.Convert(pw);
                    transformedPrec[vi] = new Rat3Hybrid(ip.X, ip.Y, ip.Z);
                }
            }

            var usedGroupIds = new HashSet<int>();
            foreach (var ex in srcMesh.TrianglesEx)
                usedGroupIds.Add(ex.GroupId);

            var sortedOldIds = new List<int>(usedGroupIds);
            sortedOldIds.Sort();

            int baseGroup = GetBaseGroupIndex();
            var oldToNewGroup = new Dictionary<int, int>();
            var newGroupIdToName = new Dictionary<int, string>();
            for (int i = 0; i < sortedOldIds.Count; i++)
            {
                int oldId = sortedOldIds[i];
                int newId = baseGroup + i;
                oldToNewGroup[oldId] = newId;
                if (!source.groupIdToExtendedName.TryGetValue(oldId, out string oldName))
                    throw new InvalidOperationException($"Missing surface name for group id {oldId}.");
                newGroupIdToName[newId] = EntityNaming.DecoratePatchName(
                    oldName, surfaceNameAffix, affixKind == SurfacePatchNameAffix.Prefix);
            }

            IncrementBaseGroupIndex(sortedOldIds.Count);

            var newSurfaceMeta = new Dictionary<string, SurfaceMetaData>();
            foreach (var kv in source.surfaceMetaData)
            {
                string decoratedKey = EntityNaming.DecoratePatchName(
                    kv.Key, surfaceNameAffix, affixKind == SurfacePatchNameAffix.Prefix);
                var cloned = kv.Value.Clone();
                cloned.Transform(in t);
                newSurfaceMeta[decoratedKey] = cloned;
            }

            var cornerPos = new List<Vec3D>();
            var cornerNormals = new List<Vec3D>();
            var cornerUv = new List<Vec2D>();
            var cornerPrecise = new List<Rat3Hybrid>();
            var newTris = new List<Tri>();
            var newGroups = new List<int>();

            for (int ti = 0; ti < srcMesh.Triangles.Count; ti++)
            {
                Tri tri = srcMesh.Triangles[ti];
                MeshTriangle<TriangleVertexNormalUV> ex = srcMesh.TrianglesEx[ti];

                AppendCorner(cornerPos, cornerNormals, cornerUv, cornerPrecise, transformedPos, transformedPrec, tri.A, ex.V0, in t);
                AppendCorner(cornerPos, cornerNormals, cornerUv, cornerPrecise, transformedPos, transformedPrec, tri.B, ex.V1, in t);
                AppendCorner(cornerPos, cornerNormals, cornerUv, cornerPrecise, transformedPos, transformedPrec, tri.C, ex.V2, in t);

                int i0 = cornerPos.Count - 3;
                newTris.Add(new Tri(i0, i0 + 1, i0 + 2));
                newGroups.Add(oldToNewGroup[ex.GroupId]);
            }

            // Weld exploded corners; skip the strict pre-weld watertight check (AnchorMesh uses allowTouch).
            // Source is already fused — do not run coplanar fusion again on the rigid copy.
            var newMesh = new MeshNormalUV(converter, cornerPos, cornerNormals, cornerUv, newTris, newGroups, cornerPrecise, skipWatertightCheck: true);
            var result = new AnchorMesh(
                newMeshName,
                newMesh,
                newGroupIdToName,
                newSurfaceMeta,
                deferCoplanarPostProcess: false,
                skipCoplanarFusion: true,
                isVolume: source.IsVolume,
                preserveTriangulation: true);
            RegisterMesh(result);
            return result;
        }

        /// <summary>
        /// Deep-copies a solid without moving it and rewrites every entity name from the
        /// source mesh name to <paramref name="newMeshName"/>. Intended for assembly instances.
        /// </summary>
        [APIDescription(@"CopyMeshAsInstance(source: AnchorMesh, newMeshName: str) -> AnchorMesh
Deep-copies a solid in place. `newMeshName` is required and must differ from the source name; all patch, edge, blend, and metadata names that used the source mesh prefix are rewritten to the new name.")]
        public AnchorMesh CopyMeshAsInstance(AnchorMesh source, string newMeshName)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            if (string.IsNullOrWhiteSpace(newMeshName))
                throw new ArgumentException("Copy requires a new part name.", nameof(newMeshName));
            if (newMeshName == source.Name)
                throw new ArgumentException("Copy requires a name different from the source mesh.", nameof(newMeshName));

            source.EnsureCoplanarPostProcessed();

            var usedGroupIds = new HashSet<int>();
            foreach (MeshTriangle<TriangleVertexNormalUV> triangle in source.Mesh.TrianglesEx)
                usedGroupIds.Add(triangle.GroupId);

            var sortedOldIds = new List<int>(usedGroupIds);
            sortedOldIds.Sort();
            int baseGroup = GetBaseGroupIndex();
            var oldToNewGroup = new Dictionary<int, int>();
            var newGroupIdToName = new Dictionary<int, string>();
            for (int i = 0; i < sortedOldIds.Count; i++)
            {
                int oldId = sortedOldIds[i];
                int newId = baseGroup + i;
                oldToNewGroup[oldId] = newId;
                if (!source.groupIdToExtendedName.TryGetValue(oldId, out string oldName))
                    throw new InvalidOperationException($"Missing surface name for group id {oldId}.");
                newGroupIdToName[newId] =
                    EntityNaming.RewriteMeshNameInEntity(oldName, source.Name, newMeshName);
            }
            IncrementBaseGroupIndex(sortedOldIds.Count);

            var triangleData = new List<MeshTriangle<TriangleVertexNormalUV>>(source.Mesh.TrianglesEx.Count);
            foreach (MeshTriangle<TriangleVertexNormalUV> sourceTriangle in source.Mesh.TrianglesEx)
            {
                MeshTriangle<TriangleVertexNormalUV> copyTriangle = sourceTriangle;
                copyTriangle.GroupId = oldToNewGroup[sourceTriangle.GroupId];
                triangleData.Add(copyTriangle);
            }

            var exactMesh = new MeshNormalUV
            {
                Positions = new List<Vec3D>(source.Mesh.Positions),
                PrecisionPositions = new List<Rat3Hybrid>(source.Mesh.PrecisionPositions),
                Triangles = new List<Tri>(source.Mesh.Triangles),
                TrianglesEx = triangleData
            };

            var newSurfaceMeta = new Dictionary<string, SurfaceMetaData>();
            foreach (var kv in source.surfaceMetaData)
            {
                string newName =
                    EntityNaming.RewriteMeshNameInEntity(kv.Key, source.Name, newMeshName);
                newSurfaceMeta[newName] = kv.Value.Clone();
            }

            var result = new AnchorMesh(
                newMeshName,
                exactMesh,
                newGroupIdToName,
                newSurfaceMeta,
                deferCoplanarPostProcess: false,
                skipCoplanarFusion: true,
                isVolume: source.IsVolume,
                preserveTriangulation: true);
            RegisterMesh(result);
            return result;
        }

        private static bool IsApproximatelyIdentityRotation(in Mat4D m, double eps)
        {
            static bool Near(double a, double b, double e) => Math.Abs(a - b) <= e;

            return Near(m.M11, 1, eps) && Near(m.M22, 1, eps) && Near(m.M33, 1, eps)
                && Near(m.M12, 0, eps) && Near(m.M13, 0, eps)
                && Near(m.M21, 0, eps) && Near(m.M23, 0, eps)
                && Near(m.M31, 0, eps) && Near(m.M32, 0, eps);
        }

        private static void SnapTranslationToConverterLattice(ref Mat4D m, CoordinateConverter conv)
        {
            double cell = conv.SmallestUnit();
            m.M14 = Math.Round(m.M14 / cell) * cell;
            m.M24 = Math.Round(m.M24 / cell) * cell;
            m.M34 = Math.Round(m.M34 / cell) * cell;
        }

        private static void AppendCorner(
            List<Vec3D> cornerPos,
            List<Vec3D> cornerNormals,
            List<Vec2D> cornerUv,
            List<Rat3Hybrid> cornerPrecise,
            Vec3D[] transformedPos,
            Rat3Hybrid[] transformedPrec,
            int vertexIndex,
            TriangleVertexNormalUV corner,
            in Mat4D t)
        {
            cornerPos.Add(transformedPos[vertexIndex]);
            cornerPrecise.Add(transformedPrec[vertexIndex]);
            Vec3D n = t.TransformDirection(corner.Normal);
            n.Normalize();
            cornerNormals.Add(n);
            cornerUv.Add(corner.UV);
        }
    }
}
