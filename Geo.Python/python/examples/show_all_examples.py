"""Build the CamberCAD example galleries and display them in one viewer.

Run from the repository root with the project virtual environment::

    ./.venv/Scripts/python.exe Geo.Python/python/examples/show_all_examples.py

Use ``--no-show`` to build and report without opening the viewer.
"""
import argparse
import gc
import runpy
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"


def _example_scripts(folder):
    return sorted(path for path in folder.glob("*.py")
                  if not path.name.startswith("_"))


def _display_scene(value):
    from camber.display import decode

    display_scene = getattr(value, "display_scene", None)
    if callable(display_scene):
        return display_scene()
    if hasattr(value, "val") and callable(value.val):
        value = value.val()
    native = getattr(value, "_n", value)
    dump = getattr(native, "dump_display", None)
    if not callable(dump):
        raise TypeError("example result has no displayable geometry")
    return decode(dump())


def _bounds(scene):
    points = []
    for patch in scene.patches:
        points.extend(patch["vertices"])
    for curve in scene.curves:
        points.extend(curve["points"])
    points.extend(point["position"] for point in scene.points)
    if not points:
        raise ValueError("example result contains no displayable geometry")
    return tuple(min(p[i] for p in points) for i in range(3)), \
        tuple(max(p[i] for p in points) for i in range(3))


def _translate_and_label(scene, offset, label):
    dx, dy, dz = offset
    for patch in scene.patches:
        patch["vertices"] = [(x + dx, y + dy, z + dz)
                             for x, y, z in patch["vertices"]]
        patch["name"] = label + " / " + patch.get("name", "surface")
    for curve in scene.curves:
        curve["points"] = [(x + dx, y + dy, z + dz)
                           for x, y, z in curve["points"]]
        curve["name"] = label + " / " + curve.get("name", "curve")
    for point in scene.points:
        x, y, z = point["position"]
        point["position"] = (x + dx, y + dy, z + dz)
        point["name"] = label + " / " + point.get("name", "point")


def _collect_gallery(folder, category, failures):
    scenes = []
    previous_common = sys.modules.pop("_common", None)
    sys.path.insert(0, str(folder))
    try:
        for path in _example_scripts(folder):
            label = "{} {}".format(category, path.stem)
            namespace = build = scene = None
            try:
                start = time.perf_counter()
                print("Building: " + label, flush=True)
                # A non-main name prevents standalone examples from opening viewers.
                namespace = runpy.run_path(str(path), run_name="_camber_all_examples")
                build = namespace.get("build")
                if not callable(build):
                    raise TypeError("example does not provide build()")
                scene = _display_scene(build())
                scenes.append((label, scene))
                print("Built: {} ({:.1f} s)".format(
                    label, time.perf_counter() - start), flush=True)
            except Exception as error:
                message = str(error)
                failures.append((label, message))
                print("Skipped: {} ({})".format(label, message), flush=True)
            finally:
                # Drop native model wrappers before the next, potentially large build.
                namespace = build = scene = None
                gc.collect()
    finally:
        sys.path.remove(str(folder))
        sys.modules.pop("_common", None)
        if previous_common is not None:
            sys.modules["_common"] = previous_common
    return scenes


def _combine(scenes, columns=7):
    from camber.display import DisplayScene

    # Measure each example before positioning. Cell dimensions are local to each
    # column/row, so one unusually large model does not create huge gaps everywhere.
    measured = []
    for label, scene in scenes:
        lo, hi = _bounds(scene)
        measured.append((label, scene, lo, hi))

    rows = (len(measured) + columns - 1) // columns
    widths = [0.0] * columns
    heights = [0.0] * rows
    for index, (_label, _scene, lo, hi) in enumerate(measured):
        col, row = index % columns, index // columns
        widths[col] = max(widths[col], hi[0] - lo[0])
        heights[row] = max(heights[row], hi[1] - lo[1])

    gap = max((max(hi[i] - lo[i] for _label, _scene, lo, hi in measured)
               for i in range(3)), default=1.0) * 0.12
    x_starts = []
    cursor = 0.0
    for width in widths:
        x_starts.append(cursor)
        cursor += width + gap
    y_starts = []
    cursor = 0.0
    for height in heights:
        y_starts.append(cursor)
        cursor -= height + gap

    combined = DisplayScene()
    for index, (label, scene, lo, hi) in enumerate(measured):
        col, row = index % columns, index // columns
        width, height = hi[0] - lo[0], hi[1] - lo[1]
        x = x_starts[col] + (widths[col] - width) * 0.5 - lo[0]
        y = y_starts[row] - (heights[row] - height) * 0.5 - lo[1]
        _translate_and_label(scene, (x, y, -lo[2]), label)
        combined.extend(scene)
    return combined


def build_all():
    """Build all CadQuery example and sketch-gallery samples into one scene."""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from camber.api import progress_log_enabled, set_progress_log

    old_progress_logging = progress_log_enabled()
    set_progress_log(False)
    try:
        scenes = []
        failures = []
        scenes.extend(_collect_gallery(
            EXAMPLES / "cadquery_gallery", "gallery", failures))
        scenes.extend(_collect_gallery(
            EXAMPLES / "cadquery_sketch_gallery", "sketch", failures))
        scenes.extend(_collect_gallery(
            EXAMPLES / "cadquery_assembly_gallery", "assembly", failures))
        if not scenes:
            raise RuntimeError("No examples produced displayable geometry")
        combined = _combine(scenes)
    finally:
        set_progress_log(old_progress_logging)
    return combined, failures, len(scenes)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-show", action="store_true",
                        help="build everything and report, without opening the viewer")
    args = parser.parse_args(argv)

    combined, failures, count = build_all()
    print("Combined {} examples: {} surfaces, {} curves, {} points".format(
        count, len(combined.patches), len(combined.curves),
        len(combined.points)))
    if failures:
        print("{} example(s) did not build:".format(len(failures)))
        for label, error in failures:
            print("  {}: {}".format(label, error))

    if not args.no_show:
        from camber.view import show
        show(combined, title="CamberCAD - all examples")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
