"""Shared geometry for the CadQuery assembly documentation ports."""

from camber import Part, vec3
from camber.cqcompat import Workplane
from math import cos, pi, sin


def part(size=10):
    return Part(vec3(-size), vec3(size), tolerance=0.01)


def cone(model, bottom=1, top=0, height=2):
    """Revolve a straight profile; exact cone geometry rather than a tapered prism."""
    profile = [(0, 0), (bottom, 0), (top, height)]
    if top:
        profile.append((0, height))
    return Workplane("XZ", part=model).polyline(profile).close().revolve()


def sinusoidal_surface(model):
    return Workplane(part=model).parametricSurface(
        lambda u, v: (u, v, 5 * sin(pi * u / 10) * cos(pi * v / 10)),
        N=40, start=0, stop=20)
