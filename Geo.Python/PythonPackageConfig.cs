using DotWrap.Configuration.Python;

namespace GeoPy;

internal class PythonPackageConfig : DotWrapPythonGlobalConfig
{
    public override string PythonPackageName
    {
        get { return "_camber_native"; }
    }
}
