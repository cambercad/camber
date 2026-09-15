namespace Geo.BRep
{
    public static class BRepLoopUtil
    {
        public static BRepTrimLoop FindOuterLoop(List<BRepTrimLoop> loops)
        {
            if (loops == null || loops.Count == 0)
                return null;

            for (int i = 0; i < loops.Count; i++)
            {
                if (loops[i].IsOuter)
                    return loops[i];
            }

            return loops[0];
        }
    }
}
