"""camber — GeoAPI for Python (sketch, extrude, revolve, loft, boolean, mesh export).

The API is meant to feel like numpy: snake_case, tuples or vec2/vec3, Frame poses,
and + / - / & on solids.

A CadQuery-shaped fluent wrapper lives in ``camber.cqcompat``
(``from camber.cqcompat import Workplane``).
"""

from .vec import vec2, vec3
from .geom import (
    RayHit,
    convex_hull,
    is_ccw,
    point_in_polygon,
    signed_area,
    tessellate_bezier,
    text_outlines,
    triangulate,
)
from .api import (
    BOOLEAN_DIFFERENCE,
    BOOLEAN_INTERSECT,
    BOOLEAN_UNION,
    Assembly,
    AssemblyAxisDatum,
    AssemblyPart,
    AssemblyPlaneDatum,
    AssemblyPointDatum,
    Curve,
    Frame,
    LoftOptions,
    Part,
    ProjectedSketch,
    Sketch,
    SketchCurve,
    Solid,
    frame_from_axis,
    progress_log_enabled,
    set_progress_log,
)

def show(obj, title="Camber"):
    """Open the 3D viewer (pyglet + imgui). Requires: pip install pyglet imgui[pyglet] numpy."""
    from .view import show as _show
    _show(obj, title=title)

__all__ = [
    "Part", "Sketch", "SketchCurve", "Solid", "ProjectedSketch", "Frame", "Curve", "LoftOptions",
    "Assembly", "AssemblyPart", "AssemblyPointDatum", "AssemblyAxisDatum", "AssemblyPlaneDatum",
    "vec2", "vec3", "show", "frame_from_axis",
    "BOOLEAN_UNION", "BOOLEAN_DIFFERENCE", "BOOLEAN_INTERSECT",
    "RayHit", "triangulate", "signed_area", "is_ccw", "point_in_polygon",
    "convex_hull", "tessellate_bezier", "text_outlines",
    "set_progress_log", "progress_log_enabled",
]
