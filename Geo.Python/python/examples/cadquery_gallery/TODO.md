# Remaining parity work

All 40 code snippets on the current CadQuery examples page run through Camber's
compatibility layer. Remaining work is fidelity auditing rather than missing
gallery calls:

| Gap | Needed |
| --- | --- |
| Full geometric comparison | Compare each result's dimensions, volume, placement, and feature layout against CadQuery output; add targeted assertions for confirmed mismatches. |
| Curved surface representation | Bottle shell joins and `parametricCurve()` remain tolerance-bounded meshes/sampled curves, not exact OCC B-reps. |
| Parameter variations | Exercise optional enclosure countersink settings and non-default Lego/Braille parameters. |
