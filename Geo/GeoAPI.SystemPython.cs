using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using GeoMeta;

namespace Geo
{
    /// <summary>
    /// Gated launch of the host system Python interpreter (sandbox escape with UI confirm).
    /// </summary>
    public partial class GeoAPI
    {
        /// <summary>
        /// Host UI sets this to show a Yes/No confirmation before any system process launch.
        /// Must return true only when the user explicitly approves.
        /// Signature: (title, message) => approved.
        /// </summary>
        public static Func<string, string, bool> ConfirmExternalProcess { get; set; }

        /// <summary>
        /// Optional override for the system Python launcher (default: <c>py</c> on Windows, <c>python3</c> elsewhere).
        /// </summary>
        public static string SystemPythonExecutable { get; set; }

        /// <summary>
        /// Optional live stdout/stderr sink (e.g. GeoScriptViewer Python output window).
        /// Invoked on the process I/O thread; host should marshal to UI if needed.
        /// </summary>
        public static Action<string> SystemPythonOutput { get; set; }

        /// <summary>
        /// Directory of the script currently being executed (set by the host). Used by
        /// <see cref="ResolvePath"/> so relative paths like <c>../Fluid</c> work regardless of process cwd.
        /// </summary>
        public static string CurrentScriptDirectory { get; set; }

        [APIDescription(@"ResolvePath(path: str) -> str
Resolves a path to an absolute path. Relative paths are resolved against CurrentScriptDirectory
when set (the folder of the running .py file), otherwise against the process current directory.")]
        public static string ResolvePath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("path is required", nameof(path));

            if (Path.IsPathRooted(path))
                return Path.GetFullPath(path);

            string root = !string.IsNullOrWhiteSpace(CurrentScriptDirectory)
                ? CurrentScriptDirectory
                : Directory.GetCurrentDirectory();

            return Path.GetFullPath(Path.Combine(root, path));
        }

        [APIDescription(@"WriteTextFile(path: str, contents: str) -> str
Writes a UTF-8 text file (creates parent directories). Relative paths use ResolvePath.
Returns the absolute path written. Use for CAD↔CFD case JSON handoff.")]
        public static string WriteTextFile(string path, string contents)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("path is required", nameof(path));
            contents ??= "";
            string abs = ResolvePath(path);
            string dir = Path.GetDirectoryName(abs);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);
            File.WriteAllText(abs, contents, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
            return abs;
        }

        [APIDescription(@"RunSystemPython(arguments: str, workingDirectory: str = None, timeoutSeconds: float = 0) -> str
Runs the system Python interpreter (not the CSG sandbox) with the given argument string.
ALWAYS prompts the user to confirm before launching (ConfirmExternalProcess).
  arguments: e.g. '-3 script.py --flag'
  workingDirectory: optional cwd (relative paths use ResolvePath / CurrentScriptDirectory).
  timeoutSeconds: 0 = wait indefinitely; otherwise kill after timeout.
Stdout/stderr are streamed live to SystemPythonOutput when set, and also returned combined.
Raises if the user declines or the process fails.")]
        public static string RunSystemPython(string arguments, string workingDirectory = null, double timeoutSeconds = 0)
        {
            if (string.IsNullOrWhiteSpace(arguments))
                throw new ArgumentException("arguments are required", nameof(arguments));

            string exe = ResolveSystemPythonExecutable();
            string cwd = string.IsNullOrWhiteSpace(workingDirectory)
                ? Directory.GetCurrentDirectory()
                : ResolvePath(workingDirectory);

            if (!Directory.Exists(cwd))
                throw new DirectoryNotFoundException(
                    $"System Python working directory does not exist: {cwd}");

            string title = "Allow system Python?";
            string message =
                "CSG sandbox wants to run an external system Python command.\n\n" +
                $"Executable: {exe}\n" +
                $"Working directory: {cwd}\n" +
                $"Arguments: {arguments}\n\n" +
                "This leaves the restricted Geo script environment and can run arbitrary code.\n" +
                "Only continue if you trust this script.";

            EnsureExternalProcessConfirmed(title, message);

            var sink = SystemPythonOutput;
            sink?.Invoke($"[system python] {exe} {arguments}\n");
            sink?.Invoke($"[system python] cwd={cwd}\n");

            var psi = new ProcessStartInfo
            {
                FileName = exe,
                Arguments = arguments,
                WorkingDirectory = cwd,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };

            using var process = new Process { StartInfo = psi };
            var stdout = new StringBuilder();
            var stderr = new StringBuilder();
            process.OutputDataReceived += (_, e) =>
            {
                if (e.Data == null) return;
                stdout.AppendLine(e.Data);
                sink?.Invoke(e.Data + Environment.NewLine);
            };
            process.ErrorDataReceived += (_, e) =>
            {
                if (e.Data == null) return;
                stderr.AppendLine(e.Data);
                // Keep stderr visible in the CSG output window with a light marker.
                sink?.Invoke("[stderr] " + e.Data + Environment.NewLine);
            };

            if (!process.Start())
                throw new InvalidOperationException($"Failed to start '{exe}'.");

            process.BeginOutputReadLine();
            process.BeginErrorReadLine();

            bool exited;
            if (timeoutSeconds > 0)
                exited = process.WaitForExit((int)(timeoutSeconds * 1000.0));
            else
            {
                process.WaitForExit();
                exited = true;
            }

            if (!exited)
            {
                try { process.Kill(entireProcessTree: true); } catch { /* ignore */ }
                throw new TimeoutException($"System Python timed out after {timeoutSeconds} s.");
            }

            process.WaitForExit();

            string combined = stdout.ToString();
            if (stderr.Length > 0)
            {
                if (combined.Length > 0) combined += Environment.NewLine;
                combined += stderr.ToString();
            }

            if (process.ExitCode != 0)
            {
                sink?.Invoke($"[system python] exited with code {process.ExitCode}\n");
                throw new InvalidOperationException(
                    $"System Python exited with code {process.ExitCode}.\n{combined}");
            }

            sink?.Invoke($"[system python] exited with code 0\n");
            return combined;
        }

        static void EnsureExternalProcessConfirmed(string title, string message)
        {
            var confirm = ConfirmExternalProcess;
            if (confirm == null)
            {
                throw new InvalidOperationException(
                    "External system Python is blocked: no confirmation UI is registered. " +
                    "GeoScriptViewer must set GeoAPI.ConfirmExternalProcess before scripts can leave the sandbox.");
            }

            bool ok;
            try
            {
                ok = confirm(title, message);
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException("Confirmation dialog failed; system Python was not started.", ex);
            }

            if (!ok)
                throw new OperationCanceledException("User declined system Python execution.");
        }

        static string ResolveSystemPythonExecutable()
        {
            if (!string.IsNullOrWhiteSpace(SystemPythonExecutable))
                return SystemPythonExecutable;
            return OperatingSystem.IsWindows() ? "py" : "python3";
        }
    }
}
