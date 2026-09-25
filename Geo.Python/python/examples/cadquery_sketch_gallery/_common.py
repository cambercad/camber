"""Small helpers shared by the CadQuery sketch-gallery ports."""
import sys

from camber.cqcompat import Workplane


def show(result, title):
    solid = result.val() if isinstance(result, Workplane) else result
    if solid is None or solid.triangle_count == 0:
        raise AssertionError("gallery example produced no displayable triangles")
    if sys._getframe(1).f_globals.get("__name__") == "__main__":
        solid.show(title=title)
    return result
