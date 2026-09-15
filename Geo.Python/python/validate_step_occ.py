"""Load a STEP file with OpenCASCADE (cadquery-ocp) and print topology."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def count_shape(shape, kind) -> int:
    n = 0
    exp = TopExp_Explorer(shape, kind)
    while exp.More():
        n += 1
        exp.Next()
    return n


def volume(shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return abs(props.Mass())


def face_types(shape) -> dict:
    kinds = {}
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        ad = BRepAdaptor_Surface(TopoDS.Face_s(exp.Current()))
        key = str(ad.GetType())
        kinds[key] = kinds.get(key, 0) + 1
        exp.Next()
    return kinds


def load(path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {status}")
    transferred = reader.TransferRoots()
    shape = reader.OneShape()
    return reader, transferred, shape


def main(argv):
    path = Path(argv[1] if len(argv) > 1 else "python/v_gear.step")
    result = {
        "path": str(path),
        "precision_mode": Interface_Static.IVal_s("read.precision.mode"),
        "precision_val": Interface_Static.RVal_s("read.precision.val"),
        "maxprecision": Interface_Static.RVal_s("read.maxprecision.val"),
        "surfacecurve_mode": Interface_Static.IVal_s("read.surfacecurve.mode"),
    }
    reader, transferred, shape = load(path)
    result["transferred"] = transferred
    result["nb_shapes"] = reader.NbShapes()
    result["solids"] = count_shape(shape, TopAbs_SOLID)
    result["shells"] = count_shape(shape, TopAbs_SHELL)
    result["faces"] = count_shape(shape, TopAbs_FACE)
    result["edges"] = count_shape(shape, TopAbs_EDGE)
    result["volume"] = volume(shape)
    result["face_types"] = face_types(shape)
    print(json.dumps(result, indent=2))
    if result["solids"] < 1 or result["faces"] < 50 or result["volume"] < 1000:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
