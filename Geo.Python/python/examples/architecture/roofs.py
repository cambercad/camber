"""Rectangular roofs. X = width, Y = length, Z = up; lengths in millimetres."""
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Literal

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from camber import Frame
from camber.modeling_parameters import register_generator
from examples.architecture._building import Builder, nonnegative, polygon, positive, show_component


@dataclass(frozen=True, kw_only=True)
class _RoofLayout:
    width: float = 8000
    length: float = 10000
    shape: Literal['gable', 'hip', 'flat'] = 'gable'
    pitch: float = 35
    eave_height: float = 3000
    overhang: float = 400

    def __post_init__(self):
        for name in ('width', 'length'):
            positive(name, getattr(self, name))
        for name in ('eave_height', 'overhang', 'pitch'):
            nonnegative(name, getattr(self, name))
        if self.shape not in ('gable', 'hip', 'flat'):
            raise ValueError("shape must be 'gable', 'hip' or 'flat'")
        if self.shape != 'flat' and not 0 < self.pitch < 80:
            raise ValueError('sloping roofs require a pitch between 0 and 80 degrees')


@dataclass(frozen=True, kw_only=True)
class RoofParameters(_RoofLayout):
    thickness: float = 180

    def __post_init__(self):
        super().__post_init__()
        positive('thickness', self.thickness)


@dataclass(frozen=True)
class RoofFace:
    name: str
    frame: Frame
    outline: tuple


def roof_faces(p):
    """Top surfaces; eave_height is the roof top at the exterior wall line."""
    o = p.overhang
    w, length = p.width+2*o, p.length+2*o
    if p.shape == 'flat':
        return (RoofFace('top', Frame((-o, -o, p.eave_height)),
                         ((0, 0), (w, 0), (w, length), (0, length))),)
    a = math.radians(p.pitch)
    c, s = math.cos(a), math.sin(a)
    z = p.eave_height-o*math.tan(a)
    run = w/2 if p.shape == 'gable' else min(w, length)/2
    def outline(span):
        if p.shape == 'gable':
            return ((0, 0), (span, 0), (span, run/c), (0, run/c))
        top = [(span-run, run/c), (run, run/c)]
        if span == 2*run:
            top.pop()
        return tuple([(0, 0), (span, 0)]+top)
    faces = [RoofFace('west', Frame((-o, p.length+o, z), x=(0, -1, 0), y=(c, 0, s), z=(-s, 0, c)), outline(length)),
             RoofFace('east', Frame((p.width+o, -o, z), x=(0, 1, 0), y=(-c, 0, s), z=(s, 0, c)), outline(length))]
    if p.shape == 'hip':
        faces += [RoofFace('south', Frame((-o, -o, z), x=(1, 0, 0), y=(0, c, s), z=(0, -s, c)), outline(w)),
                  RoofFace('north', Frame((p.width+o, p.length+o, z), x=(-1, 0, 0), y=(0, -c, s), z=(0, s, c)), outline(w))]
    return tuple(faces)


def roof_bounds(p, *, margin=500):
    extent = max(p.width, p.length)+p.overhang+margin
    peak = p.eave_height+(max(p.width, p.length)/2)*math.tan(math.radians(p.pitch)) if p.shape != 'flat' else p.eave_height
    return (-extent, -extent, -extent), (2*extent, 2*extent, peak+extent)


def build_roof(parameters=RoofParameters(), *, part=None, name='roof'):
    """Build gable, hip or flat roof panels with mitred vertical joints.

    Thickness is vertical. Width/length describe the wall footprint; overhang
    extends beyond it. Flat roofs have no inferred drainage or structural framing.
    """
    p = parameters
    if not isinstance(p, RoofParameters):
        raise TypeError('parameters must be RoofParameters')
    b = Builder(part, name, *roof_bounds(p, margin=p.thickness+500))
    for face in roof_faces(p):
        label = name+'_'+face.name
        bottom = polygon(b.part, face.outline, face.frame.offset((0, 0, -p.thickness)), label+'_bottom')
        top = polygon(b.part, face.outline, face.frame, label+'_top')
        b.add(b.part.loft([bottom, top], name=label), (.65, .56, .40))
    return b.finish()


roof_component = register_generator(build_roof, RoofParameters, name='roof_component')

if __name__ == '__main__':
    show_component(build_roof, RoofParameters())
