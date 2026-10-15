using System.Diagnostics.Tracing;

// Opt-in (PIPE90_ALLOC=1): attributes allocated bytes to types using the runtime's sampled AllocationTick events.
sealed class AllocListener : EventListener
{
    private static AllocListener? instance;
    private readonly Dictionary<string, long> bytesByType = new();

    public static void Start() => instance = new AllocListener();

    public static void Report()
    {
        var self = instance;
        if (self == null) return;
        Thread.Sleep(500);
        lock (self.bytesByType)
        {
            long total = self.bytesByType.Values.Sum();
            Console.WriteLine($"alloc_sampled_total_mb={total / 1e6:F0}");
            foreach (var kv in self.bytesByType.Where(k => k.Key.StartsWith("gcstart")))
                Console.WriteLine($"  {kv.Key} count={kv.Value / 1_000_000}");
            foreach (var kv in self.bytesByType.Where(k => !k.Key.StartsWith("gcstart")).OrderByDescending(k => k.Value).Take(15))
                Console.WriteLine($"  {kv.Value / 1e6,9:F0} MB {100.0 * kv.Value / total,5:F1}%  {kv.Key}");
        }
    }

    protected override void OnEventSourceCreated(EventSource source)
    {
        if (source.Name == "Microsoft-Windows-DotNETRuntime")
            EnableEvents(source, EventLevel.Verbose, (EventKeywords)0x1);
    }

    protected override void OnEventWritten(EventWrittenEventArgs e)
    {
        if (e.EventName != null && e.EventName.StartsWith("GCStart") && e.Payload != null && e.PayloadNames != null)
        {
            string key = "gcstart";
            for (int i = 0; i < e.PayloadNames.Count; i++)
                if (e.PayloadNames[i] == "Depth" || e.PayloadNames[i] == "Reason" || e.PayloadNames[i] == "Type")
                    key += $" {e.PayloadNames[i]}={e.Payload[i]}";
            lock (bytesByType)
                bytesByType[key] = bytesByType.GetValueOrDefault(key) + 1_000_000;
            return;
        }
        if (e.EventName == null || !e.EventName.StartsWith("GCAllocationTick") || e.Payload == null || e.PayloadNames == null) return;
        string type = "?";
        long amount = 0;
        for (int i = 0; i < e.PayloadNames.Count; i++)
        {
            if (e.PayloadNames[i] == "TypeName") type = e.Payload[i]?.ToString() ?? "?";
            else if (e.PayloadNames[i] == "AllocationAmount64") amount = Convert.ToInt64(e.Payload[i]);
        }
        lock (bytesByType)
            bytesByType[type] = bytesByType.GetValueOrDefault(type) + amount;
    }
}
