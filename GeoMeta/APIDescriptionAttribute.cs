namespace GeoMeta
{
    /// <summary>
    /// Annotates a public API element (class, method, constructor, property) with a description suitable
    /// for LLM-driven Python script generation against this codebase. The text should list every parameter,
    /// optional defaults, return type and any non-obvious constraints.
    /// </summary>
    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Struct | AttributeTargets.Method
                  | AttributeTargets.Constructor | AttributeTargets.Property | AttributeTargets.Field
                  | AttributeTargets.Enum,
                    AllowMultiple = false, Inherited = false)]
    public class APIDescriptionAttribute : Attribute
    {
        public string Description;

        public APIDescriptionAttribute(string description)
        {
            Description = description;
        }
    }
}
