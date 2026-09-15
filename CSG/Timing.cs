using System.Diagnostics;

namespace CSG
{

    public class TimingEntry
    {
        public string Name;
        public long MillisecondsStart;
        public long MillisecondsEnd;
        public Timing Parent;

        public TimingEntry(Timing parent, string name)
        {
            Parent = parent;
            Name = name;
        }

        public void Start()
        {
            MillisecondsStart = Parent.Watch.ElapsedMilliseconds;
        }

        public void Stop()
        {
            MillisecondsEnd = Parent.Watch.ElapsedMilliseconds;
        }

        public override string ToString()
        {
            return Name + " " + (MillisecondsEnd - MillisecondsStart).ToString();
        }
    }

    public class Timing
    {
        public Stopwatch Watch = new Stopwatch();
        List<TimingEntry> timings = new List<TimingEntry>();

        public Timing()
        {
            Watch.Start();
        }

        public void Clear()
        {
            timings.Clear();
        }

        public TimingEntry Start(string name)
        {
            var e = new TimingEntry(this, name);
            timings.Add(e);
            e.Start();
            return e;
        }

        public override string ToString()
        {
            string s = "";
            for (int i = 0; i < timings.Count; ++i)
                s += timings[i].ToString() + "    ";
            return s;
        }
    }
}
