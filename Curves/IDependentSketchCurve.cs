namespace Curves
{
    /// <summary>
    /// Curve whose geometry is derived from a parent curve and updated after constraint solves.
    /// </summary>
    public interface IDependentSketchCurve
    {
        void Update();
    }

}
