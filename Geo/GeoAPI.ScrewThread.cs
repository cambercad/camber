using CSG;
using Curves;
using GeoCore;
using GeoMeta;

namespace Geo
{
    public partial class GeoAPI
    {
        /// <summary>
        /// Approximate ISO 60° external-thread <i>negative</i> solid for boolean subtraction (groove + shell, trimmed to
        /// <paramref name="length"/> along <paramref name="coordinateSystem"/>.Z). Not metrologically certified.
        /// </summary>
        /// <param name="majorDiameter">Nominal major diameter (same units as the part, typically metres).</param>
        /// <param name="pitch">Axial pitch (same units).</param>
        /// <param name="outerRadius">Cutter shell outer radius; &lt; 0 uses major radius plus a small clearance.</param>
        [APIDescription(@"CreateMetricThreadForBoltNegative(coordinateSystem: CoordinateSystem, majorDiameter: float, pitch: float, length: float, maxDeviation: float = -1, rightHanded: bool = True, name: str = None, outerRadius: float = -1) -> AnchorMesh
ISO-metric-style 60° external thread cutter (subtract from a shank). Axis is coordinateSystem.Z.
  majorDiameter / pitch: thread metrics in part units (not an M-key).
  length: axial extent of the usable thread after end trims.
  outerRadius: hollow-shell outer radius; -1 picks majorRadius + clearance.
Not a certified thread form.")]
        public AnchorMesh CreateMetricThreadForBoltNegative(
            CoordinateSystem coordinateSystem,
            double majorDiameter,
            double pitch,
            double length,
            double maxDeviation = -1,
            bool rightHanded = true,
            string name = null,
            double outerRadius = -1)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            string meshName = string.IsNullOrEmpty(name) ? "metricThreadBoltNegative" : name;
            AnchorMesh helixTooth;
            AnchorMesh hollowShell;
            CreateMetricThreadForBoltNegativeUntrimmed(
                coordinateSystem, majorDiameter, pitch, length, maxDeviation, rightHanded, meshName,
                ref outerRadius, out helixTooth, out hollowShell);

            AnchorMesh fullThread = Boolean(
                helixTooth, hollowShell, BooleanOp.Union, meshName + "_extrudedThread_union_hollowShell");
            return TrimThreadEnds(fullThread, coordinateSystem, outerRadius, pitch, length, meshName);
        }

        /// <summary>
        /// Internal thread removal solid plus revolved countersinks just inside the bore (from each face inward).
        /// </summary>
        [APIDescription(@"CreateMetricThreadForNutNegative(coordinateSystem: CoordinateSystem, majorDiameter: float, pitch: float, length: float, maxDeviation: float = -1, rightHanded: bool = True, name: str = None, includeBoreChamfers: bool = True) -> AnchorMesh
ISO-metric-style internal thread cutter (subtract from a nut body). Axis is coordinateSystem.Z through the bore.
  majorDiameter / pitch: thread metrics in part units (not an M-key).
  includeBoreChamfers: lead-in/out countersinks at both bore ends (expensive CSG unions).
Not a certified thread form.")]
        public AnchorMesh CreateMetricThreadForNutNegative(
            CoordinateSystem coordinateSystem,
            double majorDiameter,
            double pitch,
            double length,
            double maxDeviation = -1,
            bool rightHanded = true,
            string name = null,
            bool includeBoreChamfers = true)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            ValidateThreadMetrics(majorDiameter, pitch);

            string meshName = string.IsNullOrEmpty(name) ? "metricThreadNutNegative" : name;
            double rMaj = 0.5 * majorDiameter;
            double outerRadius = rMaj + 0.002;

            AnchorMesh helixTooth;
            AnchorMesh hollowShell;
            CreateMetricThreadForBoltNegativeUntrimmed(
                coordinateSystem, majorDiameter, pitch, length, maxDeviation, rightHanded, meshName,
                ref outerRadius, out helixTooth, out hollowShell);
            var fullThreadBoltNegative = Boolean(
                helixTooth, hollowShell, BooleanOp.Union, meshName + "_extrudedThread_union_hollowShell");

            var coreCylinder = CreateCylinder(
                coordinateSystem.GetOffsetCS(-0.5 * pitch * coordinateSystem.Z),
                rMaj + 0.001,
                length + pitch,
                maxDeviation,
                meshName + "_nut_core");

            var fullThread = Boolean(coreCylinder, fullThreadBoltNegative, BooleanOp.Difference, meshName + "_nut_core_minus_thread");

            double eps = 2 * Converter.SmallestUnit();
            Vec3D cuboidSize = new Vec3D(2 * outerRadius + 0.001, 2 * outerRadius + 0.001, pitch + 0.001);
            Vec3D offset = -(outerRadius + 0.0005) * (coordinateSystem.X + coordinateSystem.Y);

            AnchorMesh startCutHelper = CreateCuboid(
                coordinateSystem.GetOffsetCS((-pitch - 0.001 - eps) * coordinateSystem.Z + offset),
                cuboidSize,
                meshName + "_nut_start_cut_helper");
            AnchorMesh endCutHelper = CreateCuboid(
                coordinateSystem.GetOffsetCS(length * coordinateSystem.Z + offset),
                cuboidSize,
                meshName + "_nut_end_cut_helper");

            AnchorMesh tmp = Boolean(fullThread, startCutHelper, BooleanOp.Difference, meshName + "_nut_start_cut");
            AnchorMesh trimmed = Boolean(tmp, endCutHelper, BooleanOp.Difference, meshName + "_nut_trim");
            if (!includeBoreChamfers)
                return trimmed;

            double h = (Math.Sqrt(3.0) / 2.0) * pitch;
            double rMinor = rMaj - (5.0 / 8.0) * h;
            const double innerEaseTowardMajor = 0.05;
            double rChamferInner = rMinor + innerEaseTowardMajor * (rMaj - rMinor);

            double chamferLen = 0.58 * pitch;
            double maxLenByNut = 0.42 * length;
            if (chamferLen > maxLenByNut)
                chamferLen = Math.Max(0.24 * pitch, maxLenByNut);
            chamferLen = Math.Min(chamferLen, 0.88 * pitch);
            chamferLen = Math.Min(chamferLen, length);
            double radialFlare = Math.Clamp(0.62 * pitch, 2.5 * maxDeviation, 0.92 * pitch);
            double rMouth = rChamferInner + radialFlare;

            AnchorMesh startChamfer = CreateInternalBoreChamferNegative(
                coordinateSystem, 0, chamferLen, rMouth, rChamferInner, maxDeviation, meshName + "_nut_bore_chamf_lo");
            AnchorMesh endChamfer = CreateInternalBoreChamferNegative(
                coordinateSystem, length - chamferLen, length, rChamferInner, rMouth, maxDeviation, meshName + "_nut_bore_chamf_hi");

            AnchorMesh withStartChamf = Boolean(trimmed, startChamfer, BooleanOp.Union, meshName + "_nut_u_chamf_start");
            return Boolean(withStartChamf, endChamfer, BooleanOp.Union, meshName);
        }

        /// <summary>
        /// Internal thread removal solid for a pre-drilled hole (same cutter as the nut: threaded plug plus
        /// revolved lead-in/out countersinks). Subtract from a plate after drilling a tap-size bore.
        /// </summary>
        [APIDescription(@"CreateMetricThreadForHoleNegative(coordinateSystem: CoordinateSystem, majorDiameter: float, pitch: float, length: float, maxDeviation: float = -1, rightHanded: bool = True, name: str = None, includeBoreChamfers: bool = False) -> AnchorMesh
ISO-metric-style internal thread cutter (subtract from a holed plate). Axis is coordinateSystem.Z through the bore.
  majorDiameter / pitch: thread metrics in part units (not an M-key).
  includeBoreChamfers: optional lead-in/out countersinks (same as the nut cutter). Default off.
Same threaded-plug solid as CreateMetricThreadForNutNegative. Not a certified thread form.")]
        public AnchorMesh CreateMetricThreadForHoleNegative(
            CoordinateSystem coordinateSystem,
            double majorDiameter,
            double pitch,
            double length,
            double maxDeviation = -1,
            bool rightHanded = true,
            string name = null,
            bool includeBoreChamfers = false)
        {
            string meshName = string.IsNullOrEmpty(name) ? "metricThreadHoleNegative" : name;
            return CreateMetricThreadForNutNegative(
                coordinateSystem, majorDiameter, pitch, length, maxDeviation, rightHanded, meshName, includeBoreChamfers);
        }

        private static void ValidateThreadMetrics(double majorDiameter, double pitch)
        {
            if (majorDiameter <= 0)
                throw new ArgumentException("majorDiameter must be positive.", nameof(majorDiameter));
            if (pitch <= 0)
                throw new ArgumentException("pitch must be positive.", nameof(pitch));
        }

        // majorDiameter: ISO major diameter; internally uses r_maj = majorDiameter/2.
        private void CreateMetricThreadForBoltNegativeUntrimmed(
            CoordinateSystem coordinateSystem,
            double majorDiameter,
            double pitch,
            double length,
            double maxDeviation,
            bool rightHanded,
            string name,
            ref double outerRadius,
            out AnchorMesh helixTooth,
            out AnchorMesh hollowShell)
        {
            ValidateThreadMetrics(majorDiameter, pitch);
            string baseName = string.IsNullOrEmpty(name) ? "metricThreadNegative" : name;

            double rMaj = 0.5 * majorDiameter;
            var pDiv2 = pitch / 2.0;
            var pDiv8 = pitch / 8.0;
            var pDiv16 = pitch / 16.0;
            var h = (Math.Sqrt(3.0) / 2.0) * pitch;
            var hDiv4 = h * 0.25;
            var rMin = rMaj - (5.0 / 8.0) * h;
            var zigZagMin = rMin - 0.25 * h;
            var zigZagMax = rMaj + 0.125 * h;

            var contourCs = new CoordinateSystem(
                coordinateSystem.Origin,
                coordinateSystem.X,
                coordinateSystem.Z,
                -coordinateSystem.Y);
            var sketch = GetPlotterSketcher(contourCs, baseName + "_Contour");
            var first = sketch.AddLine(
                new Vec2D(zigZagMax - pDiv16, pDiv16 * 0.5 - pitch),
                new Vec2D(zigZagMin + hDiv4, pDiv2 - pDiv8 - pitch));
            sketch.AppendLine(new Vec2D(zigZagMin + hDiv4, pDiv2 + pDiv8 - pitch));
            sketch.AppendLine(new Vec2D(zigZagMax - pDiv16, pitch - pDiv16 * 0.5 - pitch));
            sketch.AppendLine(first.StartPosition);

            double numRevolutions = length / pitch + 1;
            Helix3D helix = new Helix3D(
                coordinateSystem.Origin,
                coordinateSystem.X,
                coordinateSystem.Y,
                rMaj,
                pitch,
                numRevolutions,
                rightHanded,
                baseName + "_HelixGuide");

            if (outerRadius < 0)
                outerRadius = rMaj + Math.Max(2.0 * maxDeviation, 0.001);

            hollowShell = CreateHollowCylinder(
                coordinateSystem.GetOffsetCS(-pitch * coordinateSystem.Z),
                rMaj,
                outerRadius,
                length + 2 * pitch,
                maxDeviation,
                baseName + "_hollowShell");

            helixTooth = ExtrudeAlongCurve(sketch, helix, maxDeviation, baseName + "_extrudedThread");
        }

        private AnchorMesh TrimThreadEnds(
            AnchorMesh solid,
            CoordinateSystem coordinateSystem,
            double outerRadius,
            double pitch,
            double length,
            string meshName)
        {
            double eps = 2 * Converter.SmallestUnit();
            Vec3D cuboidSize = new Vec3D(2 * outerRadius + 0.001, 2 * outerRadius + 0.001, pitch + 0.001);
            Vec3D offset = -(outerRadius + 0.0005) * (coordinateSystem.X + coordinateSystem.Y);

            AnchorMesh startCutHelper = CreateCuboid(
                coordinateSystem.GetOffsetCS((-pitch - 0.001 - eps) * coordinateSystem.Z + offset),
                cuboidSize,
                meshName + "_start_cut_helper");
            AnchorMesh endCutHelper = CreateCuboid(
                coordinateSystem.GetOffsetCS(length * coordinateSystem.Z + offset),
                cuboidSize,
                meshName + "_end_cut_helper");

            AnchorMesh tmp = Boolean(solid, startCutHelper, BooleanOp.Difference, meshName + "_start_cut");
            return Boolean(tmp, endCutHelper, BooleanOp.Difference, meshName);
        }

        private AnchorMesh CreateHollowCylinder(
            CoordinateSystem coordinateSystem,
            double innerRadius,
            double outerRadius,
            double length,
            double maxDeviation,
            string name)
        {
            var sketch = GetPlotterSketcher(coordinateSystem, name + "_hollow_profile");
            sketch.AddCircle(new Vec2D(0, 0), innerRadius);
            sketch.MoveToPointAndStartNewCurveStrip();
            sketch.AddCircle(new Vec2D(0, 0), outerRadius);
            return Extrude(sketch, length, maxDeviation, name);
        }

        private static CoordinateSystem ThreadRevolveMeridianFrame(CoordinateSystem threadAxis) =>
            new CoordinateSystem(threadAxis.Origin, threadAxis.Z, threadAxis.X, threadAxis.Y);

        private AnchorMesh CreateInternalBoreChamferNegative(
            CoordinateSystem threadAxis,
            double xLo,
            double xHi,
            double rAtLo,
            double rAtHi,
            double maxDeviation,
            string name)
        {
            var meridian = ThreadRevolveMeridianFrame(threadAxis);
            var sketch = new PlotterSketcherCoordSys(name + "_bore_chamfer_meridian", meridian);
            var pA = new Vec2D(xLo, 0);
            var pB = new Vec2D(xLo, rAtLo);
            var pC = new Vec2D(xHi, rAtHi);
            var pD = new Vec2D(xHi, 0);
            sketch.AddLine(pA, pB);
            sketch.AddLine(pB, pC);
            sketch.AddLine(pC, pD);
            sketch.AddLine(pD, pA);
            return Revolve(sketch, 2 * Math.PI, maxDeviation, name + "_boreChamferRev");
        }
    }
}
