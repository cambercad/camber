"""Multi-view image capture using the same GL host, shaders and AA as show()."""

import math
from pathlib import Path


VIEWS = {
    "+X": ((1, 0, 0), (0, 0, 1)),
    "-X": ((-1, 0, 0), (0, 0, 1)),
    "+Y": ((0, 1, 0), (0, 0, 1)),
    "-Y": ((0, -1, 0), (0, 0, 1)),
    "+Z": ((0, 0, 1), (0, 1, 0)),
    "-Z": ((0, 0, -1), (0, 1, 0)),
    "iso": ((1, -1, .7), (0, 0, 1)),
    "rear iso": ((-1, 1, .7), (0, 0, 1)),
}


def render_views(obj, path, *, views=None, tile_size=(1600, 1200), columns=3, colors=None, individual=False, checker=False):
    """Save an orthographic contact sheet and return its Path.

    Uses Camber's existing pyglet renderer; no additional packages. Defaults
    to six cube sides and two isometrics. ``views`` may be a list of preset
    names or a mapping ``label: (eye_direction, up_direction)``. ``colors``
    maps shell-style entity name patterns to RGB triples in [0, 1], with the
    last matching pattern winning. The interactive ``camber.show`` accepts
    the same colors. Named patches and nested assembly poses are preserved.

    Requires a GL context. On a headless Linux machine set
    ``PYGLET_HEADLESS=true`` before importing pyglet. No event loop is started.
    ``tile_size`` is in logical pixels (framebuffer pixels on a HiDPI display
    may be larger). Views fit individually, so labels do not imply a scale.
    Section snapshots default to plane-normal and isometric views, with a
    scale bar and stored point-to-point measurements in model units.
    Set ``individual=True`` to also save each full-resolution view beside the
    sheet as ``<stem>_01.png``, etc., in the selected view order. This allows
    close inspection without downscaling a large contact sheet.
    """
    import numpy as np
    from . import glview as g
    from .host import Viewer

    if columns < 1 or int(columns) != columns:
        raise ValueError("columns must be a positive integer")
    if len(tile_size) != 2 or any(int(v) != v or v < 64 for v in tile_size):
        raise ValueError("tile_size must contain two integer dimensions >= 64")
    from .inspection import Section
    if views is None and isinstance(obj, Section):
        views = {"Section": (tuple(obj.plane.z), tuple(obj.plane.y)),
                 "Section isometric": (tuple(obj.plane.z + obj.plane.x * .7 + obj.plane.y * .4), tuple(obj.plane.y))}
    selected = VIEWS if views is None else views
    if not hasattr(selected, "items"):
        selected = {name: VIEWS[name] for name in selected}
    if not selected:
        raise ValueError("at least one view is required")
    # Validate camera input before creating a GL context.
    cameras = []
    for label, (direction, up) in selected.items():
        direction, up = np.asarray(direction, float), np.asarray(up, float)
        if direction.shape != (3,) or up.shape != (3,) or not np.isfinite([direction, up]).all():
            raise ValueError("view directions must be finite 3-vectors")
        if np.linalg.norm(np.cross(direction, up)) < 1e-10:
            raise ValueError("eye and up directions must be nonzero and not parallel")
        cameras.append((label, direction / np.linalg.norm(direction), up / np.linalg.norm(up)))
    packed = g.pack_scene(g._as_scene(obj), colors=colors)
    if not packed["mesh_pos"]:
        raise ValueError("render_views requires nonempty solid geometry")
    viewer = Viewer("Camber image capture", visible=False, size=tile_size)
    try:
        viewer.set_solid(packed)
        viewer.inspection = obj if isinstance(obj, Section) else None
        viewer.checker = bool(checker)
        # Engineering outlines and anchor glyphs obscure thin spokes and chain links.
        for part in viewer._solid["parts"]:
            for key in ("line", "line_cap", "point"):
                viewer._delete(part.get(key))
                part[key] = None
        viewer.camera.fit(*packed["bounds"])
        positions = np.asarray(packed["mesh_pos"])
        center = (positions.min(axis=0) + positions.max(axis=0)) / 2
        radius = max(float(np.linalg.norm(np.ptp(positions, axis=0))) / 2, 1e-6)
        tiles = []
        for label, direction, up in cameras:
            cam = viewer.camera
            cam.center = tuple(center)
            cam.eye = tuple(center + direction * radius * 4)
            cam.up = tuple(up)
            cam.ortho = True
            _, camera_up, right = cam.frame()
            projected = np.column_stack((positions @ right, positions @ camera_up))
            lower, upper = projected.min(axis=0), projected.max(axis=0)
            midpoint = (lower + upper) / 2
            shift = (midpoint[0] - center @ right) * np.asarray(right)
            shift += (midpoint[1] - center @ camera_up) * np.asarray(camera_up)
            cam.center = tuple(center + shift)
            cam.eye = tuple(center + shift + direction * radius * 4)
            cam.zoom = max((upper[1]-lower[1])/2, (upper[0]-lower[0])/2/cam.aspect, 1e-6)*1.14
            cam._clip(radius, radius*4)
            tiles.append(viewer.capture_rgba(label))
        height, width = tiles[0].shape[:2]
        cols = min(int(columns), len(tiles))
        rows = math.ceil(len(tiles)/cols)
        sheet = np.full((rows*height, cols*width, 4), 247, dtype=np.uint8)
        sheet[:, :, 3] = 255
        for i, tile in enumerate(tiles):
            y, x = divmod(i, cols)
            sheet[y*height:(y+1)*height, x*width:(x+1)*width] = tile
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        image = viewer._pyglet.image.ImageData(cols*width, rows*height, "RGBA", sheet.tobytes(), pitch=-cols*width*4)
        with path.open("wb") as target:
            image.save(str(path), file=target)
        if individual:
            for i, tile in enumerate(tiles, 1):
                single = viewer._pyglet.image.ImageData(width, height, "RGBA",
                    tile.tobytes(), pitch=-width*4)
                individual_path = path.with_name(f"{path.stem}_{i:02d}.png")
                with individual_path.open("wb") as target:
                    single.save(str(individual_path), file=target)
        return path
    finally:
        viewer.close()
