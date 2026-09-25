# CadQuery examples parity audit

Source: [CadQuery's examples page](https://cadquery.readthedocs.io/en/latest/examples.html).
The numbered scripts follow its examples in order, use `camber.cqcompat.Workplane`,
and call `result.show()` when run directly so the final object opens in the viewer.

All 40 scripts pass independently with the current Windows wheel. This is an
execution check, not a claim of identical geometry: Camber tessellates
curved surfaces and `parametricCurve()` samples its Python callback within a
chord tolerance. In particular, the bottle's shell/round joins and the
cycloidal gear are tolerance-bounded meshes rather than CadQuery B-reps.

The third, unnumbered spherical-joint snippet is represented by script 40.
The native C# API and Python `Part` expose named face extraction and exact CSG
extrusion to an open face surface; the compatibility layer supports
`cutBlind(surface)`, compound/ranked face selectors, and the source chain.
Surface trims must span the complete profile intersection; incomplete trims
fail explicitly.

The parametric-enclosure script matches the documented inner-cut height and
includes four real counterbores. `cskHole()` is supported for its optional
countersink configuration. Other optional parameter combinations have not all
been audited.

Run one example without a viewer (for automated checks):

```powershell
.venv\Scripts\python.exe -c "import runpy; runpy.run_path('Geo.Python/python/examples/cadquery_gallery/35_parametric_enclosure.py')"
```

Run the file normally to open the viewer:

```powershell
.venv\Scripts\python.exe Geo.Python/python/examples/cadquery_gallery/35_parametric_enclosure.py
```
