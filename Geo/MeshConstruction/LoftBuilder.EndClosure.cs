using GeoCore;
using NURBS;

namespace Geo;

/// <summary>An exact point cap, including its normal limit at the collapsed pole.</summary>
internal sealed class LoftPointCap : INurbsSurface
{
    internal BSplineSurface Surface { get; }
    internal Vec3D OutwardNormal { get; }
    internal Vec3D Point { get; }
    internal bool Rounded { get; }
    internal double NormalSign { get; }
    internal double[] U { get; set; }
    internal double[] V { get; set; }

    internal LoftPointCap(BSplineSurface surface, Vec3D point, Vec3D normal, bool rounded, double normalSign)
    {
        Surface = surface;
        Point = point;
        OutwardNormal = normal;
        Rounded = rounded;
        NormalSign = normalSign;
    }

    public Vec3D Evaluate(double u, double v) => v == 1 ? Point : Surface.Evaluate(u, v);
    public Vec3D EvaluateNormal(double u, double v) => v == 1
        ? Rounded ? OutwardNormal : Surface.EvaluateNormal(u, 0) * NormalSign
        : Surface.EvaluateNormal(u, v) * NormalSign;
}

public static partial class LoftBuilder
{
    private static void EmitPointCap(LoftPointCap cap, List<double> columns, Rat3Hybrid[] rim,
        int group, CoordinateConverter converter, List<Vec3D> vertices, List<Vec3D> normals,
        List<Vec2D> uv, List<Rat3Hybrid> precise, List<Tri> triangles, List<int> groups)
    {
        var retained = Enumerable.Range(0, columns.Count).Where(i => i == 0 || rim[i] != rim[i-1]).ToList();
        if (retained.Count > 1 && rim[retained[0]] == rim[retained[^1]]) retained.RemoveAt(retained.Count-1);
        if (retained.Count < 3) throw new ArgumentException("A point closure requires a noncollapsed closed rim.");
        int width = retained.Count, stride = width+1, first = vertices.Count;
        foreach (double v in cap.V.Where(v => v < 1))
        {
            foreach (int i in retained)
            {
                var exact = v == 0 ? rim[i] : converter.ConvertExact(cap.Evaluate(columns[i], v));
                precise.Add(exact);
                vertices.Add(converter.Convert(exact));
                normals.Add(cap.EvaluateNormal(columns[i], v).Normalized());
                uv.Add(new Vec2D(columns[i], v));
            }
            // Keep the same exact seam position, with a separate UV corner at U=1.
            int seam = vertices.Count-width;
            precise.Add(precise[seam]);
            vertices.Add(vertices[seam]);
            normals.Add(normals[seam]);
            uv.Add(new Vec2D(1, v));
        }
        int pole = vertices.Count, rows = (pole-first)/stride;
        var tip = converter.ConvertExact(cap.Point);
        // Each fan wedge needs its own UV at the one welded geometric pole.
        for (int i=0; i<width; i++)
        {
            double nextU = i+1 < width ? columns[retained[i+1]] : 1;
            precise.Add(tip);
            vertices.Add(converter.Convert(tip));
            normals.Add(cap.OutwardNormal);
            uv.Add(new Vec2D((columns[retained[i]]+nextU)*.5, 1));
        }
        void Triangle(int a, int b, int c)
        {
            if (precise[a] == precise[b] || precise[b] == precise[c] || precise[c] == precise[a])
                throw new ArgumentException("Point closure collapses at this model's coordinate precision.");
            var cross = Vec3DOps.Cross(vertices[b]-vertices[a], vertices[c]-vertices[a]);
            if (!(cross.LengthSquared() > 0))
                throw new ArgumentException("Point closure has a singular interior cell.");
            // U=0 and U=1 are the same seam. Averaging their UVs would sample
            // the opposite side of the cap and could reverse a seam triangle.
            triangles.Add(cap.NormalSign < 0 ? new Tri(a,c,b) : new Tri(a,b,c));
            groups.Add(group);
        }
        for (int row = 0; row < rows; row++)
            for (int i = 0; i < width; i++)
            {
                int a = first+row*stride+i, b = a+1;
                if (row == rows-1) Triangle(a,b,pole+i);
                else
                {
                    int c = a+stride, d = b+stride;
                    Triangle(a,b,d);
                    Triangle(a,d,c);
                }
            }
    }

    private static LoftPointCap BuildPointCap(BSplineSurface side, Vec3D point, Vec3D outward,
        bool start, LoftPointClosure closure, double deviation, double normalSign)
    {
        if (!double.IsFinite(point.X) || !double.IsFinite(point.Y) || !double.IsFinite(point.Z))
            throw new ArgumentException("A loft endpoint must have finite coordinates.");
        if (!Enum.IsDefined(closure)) throw new ArgumentException("Unknown loft point closure.");
        int edge = start ? 0 : side.NumControlPointsV - 1;
        int adjacent = start ? 1 : edge - 1;
        double span = start ? side.KnotsV[side.DegreeV + 1] - side.KnotsV[side.DegreeV]
            : side.KnotsV[side.NumControlPointsV] - side.KnotsV[side.NumControlPointsV - 1];
        var rim = side.ControlPoints.Select(column => column[edge]).ToArray();
        Vec3D Cartesian(Vec4D p) => new(p.X / p.W, p.Y / p.W, p.Z / p.W);
        var origin = Cartesian(rim[0]);
        double height = Vec3DOps.Dot(point - origin, outward);
        double planeTolerance = 64*2.2204460492503131e-16*Math.Max(1,Math.Max(point.Length(),origin.Length()));
        if (!(height > planeTolerance))
            throw new ArgumentException("A loft endpoint must lie strictly beyond its end-section plane.");
        var derivative = side.ControlPoints.Select((column, i) =>
            (rim[i] - column[adjacent]) * (side.DegreeV / span)).ToArray();
        double influence = derivative.Select((d, i) =>
            (new Vec3D(d.X, d.Y, d.Z) - Cartesian(rim[i]) * d.W).Length() / rim[i].W).Max();
        if (!(influence > 0) || !double.IsFinite(influence))
            throw new ArgumentException("The loft boundary has no regular outward derivative for point closure.");
        double scale = height / influence;
        bool rounded = closure == LoftPointClosure.Rounded;
        var controls = new Vec4D[rim.Length][];
        for (int i = 0; i < rim.Length; i++)
        {
            var p = rim[i];
            if (!(p.W > 0)) throw new ArgumentException("Point closure requires positive rational weights.");
            var tip = new Vec4D(point.X * p.W, point.Y * p.W, point.Z * p.W, p.W);
            if (!rounded) { controls[i] = [p, tip]; continue; }
            var handle = p + derivative[i] * (scale / 3);
            if (!(handle.W > 0)) throw new ArgumentException("Point closure would produce a nonpositive rational weight.");
            double handleDepth = Vec3DOps.Dot(Cartesian(handle) - origin, outward);
            if (handleDepth < -planeTolerance || handleDepth > height+planeTolerance)
                throw new ArgumentException("The loft boundary derivative folds the rounded cap outside its endpoint planes.");
            var radius = Cartesian(p) - point;
            radius -= outward * Vec3DOps.Dot(radius, outward);
            // Cubic Hermite conversion: the terminal derivative is the inward
            // radial vector, entirely in the pole's common tangent plane.
            var poleHandle = point + radius / 3;
            controls[i] = [p, handle, new Vec4D(poleHandle.X*p.W, poleHandle.Y*p.W, poleHandle.Z*p.W, p.W), tip];
        }
        if (rounded) ValidateRadialClosure(side.DegreeU, side.KnotsU, controls, point, outward);
        var surface = new BSplineSurface(side.DegreeU, rounded ? 3 : 1, controls,
            (double[])side.KnotsU.Clone(), rounded ? [0,0,0,0,1,1,1,1] : [0,0,1,1]);
        var cap = new LoftPointCap(surface, point, outward, rounded, normalSign);
        var samples = SurfaceLoftParameters(() => surface, deviation);
        cap.U = samples.U;
        cap.V = samples.V;
        return cap;
    }

    // Positive Bernstein coefficients certify radial winding on every exact
    // curve span, including the mixed term between the two independent cap
    // control rows. Reject ambiguous/folding caps instead of sampling or fitting.
    private static void ValidateRadialClosure(int degree, double[] knots, Vec4D[][] controls, Vec3D tip, Vec3D normal)
    {
        var rows=new[] {0,1}.Select(row=>new BSplineCurve(degree,
            controls.Select(c=>c[row]).ToArray(),(double[])knots.Clone(),false)).ToArray();
        var breaks=knots.Distinct().Order().ToArray();
        foreach(double knot in breaks.Skip(1).SkipLast(1))
        {
            int missing=degree-knots.Count(k=>k==knot);
            if(missing>0) for(int row=0;row<2;row++) rows[row]=rows[row].InsertKnot(knot,missing);
        }
        double Choose(int n,int k)
        {
            double value=1;
            for(int j=1;j<=k;j++) value=value*(n-j+1)/j;
            return value;
        }
        int orientation=0;
        for(int span=0;span<breaks.Length-1;span++)
        {
            var radial=rows.Select(row=>row.ControlPoints.Skip(span*degree).Take(degree+1)
                .Select(p=>new Vec3D(p.X-tip.X*p.W,p.Y-tip.Y*p.W,p.Z-tip.Z*p.W)).ToArray()).ToArray();
            double[] CrossPolynomial(int a,int b)
            {
                var result=new double[2*degree];
                for(int i=0;i<=degree;i++) for(int j=0;j<degree;j++)
                    result[i+j]+=Vec3DOps.Dot(Vec3DOps.Cross(radial[a][i],
                        (radial[b][j+1]-radial[b][j])*degree),normal)*Choose(degree,i)*Choose(degree-1,j)/Choose(2*degree-1,i+j);
                return result;
            }
            var outer=CrossPolynomial(0,0);
            orientation=orientation==0 ? Math.Sign(outer.FirstOrDefault(c=>c!=0)) : orientation;
            var inner=CrossPolynomial(1,1);
            var mixed=CrossPolynomial(0,1).Zip(CrossPolynomial(1,0),(a,b)=>a+b);
            if(orientation==0 || outer.Concat(inner).Concat(mixed).Any(c=>!double.IsFinite(c)||c*orientation<=0))
                throw new ArgumentException($"Rounded point closure requires its projected tip inside a regular, star-shaped end section with compatible boundary tangency (U span {breaks[span]:G17} to {breaks[span+1]:G17}).");
        }
    }
}
