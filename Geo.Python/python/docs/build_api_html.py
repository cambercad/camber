#!/usr/bin/env python
"""Extract camber's public API and write a single HTML reference page."""
from __future__ import print_function

import argparse
import html
import inspect
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import camber
from camber import cqcompat

DUNDERS = {
    "__init__": None,
    "__add__": "+",
    "__sub__": "-",
    "__mul__": "*",
    "__truediv__": "/",
    "__neg__": "-",
    "__and__": "&",
    "__or__": "|",
}

SECTIONS = [
    ("Core", ["Part", "Sketch", "SketchCurve", "Solid", "ProjectedSketch"]),
    ("Pose", ["Frame", "Curve", "LoftOptions", "frame_from_axis"]),
    ("Assembly", [
        "Assembly", "AssemblyOccurrence", "AssemblyPart", "AssemblyPointDatum",
        "AssemblyAxisDatum", "AssemblyPlaneDatum",
    ]),
    ("Vectors", ["vec2", "vec3"]),
    ("Geom", [
        "RayHit", "triangulate", "signed_area", "is_ccw", "point_in_polygon",
        "convex_hull", "tessellate_bezier", "text_outlines",
    ]),
    ("Viewer", ["show"]),
    ("Constants", ["BOOLEAN_UNION", "BOOLEAN_DIFFERENCE", "BOOLEAN_INTERSECT"]),
]


CONST_DOCS = {
    "BOOLEAN_UNION": "CSG union. Same as Solid + Solid / Part.union.",
    "BOOLEAN_DIFFERENCE": "CSG difference. Same as Solid - Solid / Part.cut.",
    "BOOLEAN_INTERSECT": "CSG intersection. Same as Solid & Solid / Part.intersect.",
}

# Do not list native-handle constructors; users get these from factories.
FACTORY_HEAD = {
    "Sketch": "Sketch  # Part.sketch / Part.sketch(..., constrained=True)",
    "Solid": "Solid  # Part.extrude / union / cut / …",
    "SketchCurve": "SketchCurve  # Sketch.add_line / add_circle / add_arc",
    "ProjectedSketch": "ProjectedSketch  # Part.project_sketch",
    "Curve": "Curve  # Curve.line / helix / circle / arc / spiral",
    "Assembly": "Assembly  # Part.assembly",
    "AssemblyOccurrence": "AssemblyOccurrence  # Assembly.add_subassembly",
    "AssemblyPart": "AssemblyPart  # Assembly.add_part",
    "AssemblyPointDatum": "AssemblyPointDatum  # AssemblyPart.point / point_at",
    "AssemblyAxisDatum": "AssemblyAxisDatum  # AssemblyPart.axis / axis_at",
    "AssemblyPlaneDatum": "AssemblyPlaneDatum  # AssemblyPart.plane / plane_at",
    "RayHit": "RayHit  # Part.raycast",
    "cqcompat.Sketch": "cqcompat.Sketch  # Workplane.sketch",
}

# Viewer / dump / unique-name helpers stay underscore-prefixed and are omitted.


def first_paragraph(text):
    if not text:
        return ""
    text = inspect.cleandoc(text).strip()
    para = text.split("\n\n", 1)[0]
    para = " ".join(line.strip() for line in para.splitlines() if line.strip())
    return para


def fmt_anno(obj):
    if obj is inspect.Signature.empty:
        return None
    if isinstance(obj, type):
        return obj.__name__
    return str(obj).replace("typing.", "")


def fmt_default(obj):
    if obj is inspect.Signature.empty:
        return None
    if obj is None:
        return "None"
    if isinstance(obj, str):
        return repr(obj)
    if isinstance(obj, (int, float, bool)):
        return repr(obj)
    return repr(obj)


def signature_of(fn, drop_self=True):
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return "()"
    parts = []
    for name, p in sig.parameters.items():
        if drop_self and name in ("self", "cls"):
            continue
        piece = name
        if p.kind == inspect.Parameter.VAR_POSITIONAL:
            piece = "*" + name
        elif p.kind == inspect.Parameter.VAR_KEYWORD:
            piece = "**" + name
        elif p.kind == inspect.Parameter.KEYWORD_ONLY and not any(
            x.startswith("*") for x in parts
        ):
            parts.append("*")
        anno = fmt_anno(p.annotation)
        if anno:
            piece += ": " + anno
        default = fmt_default(p.default)
        if default is not None:
            piece += "=" + default
        parts.append(piece)
    ret = fmt_anno(sig.return_annotation)
    tail = ")" if not ret else ") -> " + ret
    return "(" + ", ".join(parts) + tail


def is_public_name(name):
    if name in DUNDERS:
        return True
    if name.startswith("_"):
        return False
    return True


def class_members(cls):
    found = []
    seen = set()
    for name, val in inspect.getmembers(cls):
        if name in seen or not is_public_name(name):
            continue
        if name in ("__module__", "__dict__", "__weakref__", "__doc__", "__slots__"):
            continue
        if name in DUNDERS:
            kind = "op"
        elif isinstance(val, property):
            kind = "prop"
        elif inspect.isfunction(val) or inspect.ismethod(val) or isinstance(
            val, (staticmethod, classmethod, types.FunctionType)
        ):
            kind = "method"
        elif inspect.ismethoddescriptor(val) or inspect.isdatadescriptor(val):
            continue
        else:
            continue
        seen.add(name)
        target = val.fget if isinstance(val, property) else val
        if isinstance(val, (staticmethod, classmethod)):
            target = val.__func__
        found.append((kind, name, target, val))
    order = {"op": 0, "prop": 1, "method": 2}
    found.sort(key=lambda row: (order[row[0]], row[1].lower()))
    return found


def member_sig(kind, name, target, raw, cls_name):
    if kind == "prop":
        return cls_name + "." + name
    if kind == "op":
        op = DUNDERS[name]
        if op is None:
            return cls_name + signature_of(target)
        if name == "__neg__":
            return op + cls_name.lower()
        return cls_name + " " + op + " " + cls_name
    if name == "__init__":
        return cls_name + signature_of(target)
    return cls_name + "." + name + signature_of(target)


def member_doc(kind, name, target, raw):
    if kind == "prop":
        return first_paragraph(inspect.getdoc(raw) or inspect.getdoc(target))
    return first_paragraph(inspect.getdoc(target))


def describe_value(name, obj):
    if inspect.isclass(obj):
        return "class", first_paragraph(inspect.getdoc(obj)), None
    if inspect.isfunction(obj) or inspect.ismethod(obj):
        return "function", first_paragraph(inspect.getdoc(obj)), signature_of(obj, drop_self=True)
    return "const", CONST_DOCS.get(name, ""), repr(obj)


def collect():
    catalog = []
    exported = dict((name, getattr(camber, name)) for name in camber.__all__)
    for title, names in SECTIONS:
        items = []
        for name in names:
            obj = exported[name]
            kind, doc, sig = describe_value(name, obj)
            entry = {
                "name": name,
                "kind": kind,
                "doc": doc,
                "sig": sig,
                "members": [],
            }
            if inspect.isclass(obj):
                if name in FACTORY_HEAD:
                    entry["sig"] = None
                    entry["head"] = FACTORY_HEAD[name]
                else:
                    ctor = getattr(obj, "__init__", None)
                    if inspect.isfunction(ctor) or inspect.ismethod(ctor):
                        entry["sig"] = signature_of(ctor)
                for mk, mn, mt, raw in class_members(obj):
                    if mn == "__init__":
                        if name not in FACTORY_HEAD:
                            entry["sig"] = signature_of(mt)
                        continue
                    entry["members"].append({
                        "kind": mk,
                        "name": mn,
                        "sig": member_sig(mk, mn, mt, raw, name),
                        "doc": member_doc(mk, mn, mt, raw),
                    })
            items.append(entry)
        catalog.append((title, items))

    cq_items = []
    for name in cqcompat.__all__:
        obj = getattr(cqcompat, name)
        kind, doc, sig = describe_value(name, obj)
        entry = {
            "name": "cqcompat." + name,
            "kind": kind,
            "doc": doc,
            "sig": sig,
            "members": [],
        }
        if inspect.isclass(obj):
            if entry["name"] in FACTORY_HEAD:
                entry["sig"] = None
                entry["head"] = FACTORY_HEAD[entry["name"]]
            for mk, mn, mt, raw in class_members(obj):
                if mn == "__init__":
                    if entry["name"] not in FACTORY_HEAD:
                        entry["sig"] = signature_of(mt)
                    continue
                entry["members"].append({
                    "kind": mk,
                    "name": mn,
                    "sig": member_sig(mk, mn, mt, raw, name),
                    "doc": member_doc(mk, mn, mt, raw),
                })
        cq_items.append(entry)
    catalog.append(("CadQuery compat", cq_items))
    return catalog


def esc(text):
    return html.escape(text or "", quote=True)


def render(catalog):
    nav = []
    body = []
    for title, items in catalog:
        sid = title.lower().replace(" ", "-")
        nav.append(
            '<div class="nav-group" data-section="{0}">'
            '<a class="nav-section" href="#{0}">{1}</a>'.format(esc(sid), esc(title))
        )
        body.append('<section id="{0}"><h2>{1}</h2>'.format(esc(sid), esc(title)))
        for item in items:
            iid = item["name"].replace(".", "-")
            nav.append(
                '<a class="nav-symbol" data-target="{0}" href="#{0}">{1}</a>'.format(
                    esc(iid), esc(item["name"])
                )
            )
            kind = item["kind"]
            head = item.get("head") or item["name"]
            if not item.get("head"):
                if kind == "function" and item["sig"]:
                    head = item["name"] + item["sig"]
                elif kind == "class" and item["sig"]:
                    head = item["name"] + item["sig"]
                elif kind == "const":
                    head = item["name"] + " = " + (item["sig"] or "")
            item_search = " ".join([
                item["name"], kind, head, item["doc"] or "",
            ]).lower()
            body.append(
                '<article class="sym" id="{0}" data-search="{1}">'
                '<div class="k">{2}</div>'
                "<h3><code>{3}</code></h3>"
                "{4}".format(
                    esc(iid),
                    esc(item_search),
                    esc(kind),
                    esc(head),
                    ("<p>{0}</p>".format(esc(item["doc"])) if item["doc"] else ""),
                )
            )
            if item["members"]:
                body.append("<ul>")
                for m in item["members"]:
                    if m["name"] == "__init__":
                        continue
                    member_search = " ".join([
                        item["name"] + "." + m["name"],
                        m["sig"],
                        m["doc"] or "",
                    ]).lower()
                    body.append(
                        '<li data-search="{0}"><code>{1}</code>{2}</li>'.format(
                            esc(member_search),
                            esc(m["sig"]),
                            (" <span>{0}</span>".format(esc(m["doc"])) if m["doc"] else ""),
                        )
                    )
                body.append("</ul>")
            body.append("</article>")
        body.append("</section>")
        nav.append("</div>")

    return PAGE.replace("__NAV__", "\n".join(nav)).replace("__BODY__", "\n".join(body))


PAGE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>camber API</title>
<style>
:root {
  --bg: #f5f7f8;
  --paper: #ffffff;
  --ink: #172126;
  --mute: #66757d;
  --line: #dfe7e9;
  --accent: #e5b82e;
  --accent-dark: #3f484d;
  --accent-soft: #fff3c4;
  --warm: #e5b82e;
  --shadow: 0 8px 24px rgba(31, 54, 61, .07);
}
* { box-sizing: border-box; }
html { scroll-behavior: smooth; }
html, body { margin: 0; background: var(--bg); color: var(--ink);
  font: 15px/1.55 Inter, ui-sans-serif, system-ui, "Segoe UI", sans-serif; }
a { color: var(--accent); text-decoration: none; }
a:hover { color: var(--accent-dark); }
code, h3 code { font-family: ui-monospace, "Cascadia Code", Consolas, monospace;
  font-size: 13px; font-weight: 550; }
.wrap { display: grid; grid-template-columns: 244px minmax(0, 1fr); min-height: 100vh; }
nav {
  position: sticky; top: 0; height: 100vh; overflow: auto;
  padding: 30px 18px 44px; border-right: 1px solid var(--line);
  background: rgba(255, 255, 255, .72); backdrop-filter: blur(12px);
}
.brand { display: flex; align-items: center; gap: 10px; margin: 0 8px 24px;
  color: var(--ink); font-size: 15px; font-weight: 750; letter-spacing: .02em; }
.brand::before { content: ""; width: 11px; height: 11px; border-radius: 3px;
  background: linear-gradient(135deg, var(--accent), var(--warm));
  box-shadow: 0 0 0 4px var(--accent-soft); transform: rotate(10deg); }
.nav-group { margin: 0 0 17px; }
.nav-section { display: block; padding: 4px 8px; color: var(--ink);
  font-size: 11px; font-weight: 750; letter-spacing: .1em; text-transform: uppercase; }
.nav-symbol { display: block; margin: 1px 0; padding: 4px 8px 4px 16px;
  border-radius: 6px; color: var(--mute); font-size: 13px;
  white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.nav-symbol:hover { background: var(--accent-soft); color: var(--accent-dark); }
main { width: min(100%, 1040px); padding: 48px 52px 96px; }
.hero { position: relative; overflow: hidden; padding: 28px 30px 30px; margin-bottom: 30px;
  border: 1px solid var(--line); border-radius: 18px; background: var(--paper); box-shadow: var(--shadow); }
h1 { position: relative; z-index: 1; font-size: clamp(28px, 4vw, 38px);
  font-weight: 750; margin: 0 0 7px; letter-spacing: -.035em; }
.lead { position: relative; z-index: 1; color: var(--mute); margin: 0 0 22px; max-width: 700px; }
.search-row { position: relative; z-index: 1; display: flex; align-items: center;
  gap: 12px; max-width: 600px; }
.search { position: relative; flex: 1; }
.search::before { content: "⌕"; position: absolute; left: 13px; top: 7px;
  color: var(--accent); font-size: 21px; line-height: 1; pointer-events: none; }
input {
  width: 100%; border: 1px solid var(--line); background: #fbfdfd;
  padding: 10px 38px 10px 40px; border-radius: 10px; font: inherit; color: var(--ink);
  outline: none; transition: border-color .15s, box-shadow .15s, background .15s;
}
input:focus { border-color: var(--accent); background: var(--paper);
  box-shadow: 0 0 0 4px var(--accent-soft); }
.shortcut { position: absolute; right: 9px; top: 9px; color: var(--mute);
  border: 1px solid var(--line); border-radius: 5px; padding: 0 5px; font-size: 11px; }
#result-count { min-width: 74px; color: var(--mute); font-size: 12px; text-align: right; }
h2 { font-size: 13px; letter-spacing: .12em; text-transform: uppercase;
  color: var(--accent-dark); margin: 38px 0 13px; font-weight: 750; }
h2::before { content: ""; display: inline-block; width: 18px; height: 3px;
  margin: 0 8px 3px 0; border-radius: 2px; background: var(--warm); }
article {
  background: var(--paper); border: 1px solid var(--line);
  border-radius: 12px; padding: 17px 19px 12px; margin: 0 0 11px;
  box-shadow: 0 2px 8px rgba(31, 54, 61, .035);
  transition: border-color .15s, box-shadow .15s, transform .15s;
}
article:hover { border-color: #e5d399; box-shadow: var(--shadow); transform: translateY(-1px); }
article:target { border-color: var(--accent); box-shadow: 0 0 0 4px var(--accent-soft); }
.k { float: right; font-size: 11px; letter-spacing: .08em; text-transform: uppercase;
  color: var(--accent-dark); background: var(--accent-soft); padding: 3px 8px; border-radius: 999px; }
h3 { margin: 0 0 6px; padding-right: 72px; font-size: 15px; font-weight: 650; }
article p { margin: 0 0 8px; color: var(--mute); }
ul { list-style: none; margin: 8px 0 0; padding: 0; border-top: 1px solid var(--line); }
li { padding: 7px 0; border-bottom: 1px solid var(--line); }
li:last-child { border-bottom: 0; }
li code { display: block; color: #293f48; }
li span { display: block; color: var(--mute); font-size: 13px; margin-top: 2px; white-space: normal; }
.hide { display: none !important; }
#no-results { display: none; padding: 32px; border: 1px dashed #dfce99;
  border-radius: 12px; color: var(--mute); text-align: center; background: rgba(255,255,255,.5); }
#no-results.show { display: block; }
@media (max-width: 860px) {
  .wrap { grid-template-columns: 1fr; }
  nav { position: relative; height: auto; border-right: 0; border-bottom: 1px solid var(--line); }
  .nav-group { display: inline-block; vertical-align: top; min-width: 150px; }
  main { padding: 30px 22px 70px; }
}
@media (max-width: 520px) {
  nav { padding: 24px 14px; }
  .hero { padding: 23px 20px; }
  .search-row { align-items: stretch; flex-direction: column; }
  #result-count { text-align: left; }
}
</style>
</head>
<body>
<div class="wrap">
<nav>
<div class="brand">camber API</div>
__NAV__
</nav>
<main>
<header class="hero">
<h1>camber API</h1>
<p class="lead">Public Python surface. Points are tuples or vec2/vec3. Solids: + union, − cut, &amp; intersect. max_deviation=-1 uses the Part default. There is no Solid | operator.</p>
<div class="search-row">
  <label class="search">
    <input id="q" type="search" placeholder="Search symbols and descriptions…" autocomplete="off" aria-label="Search API">
    <span class="shortcut">/</span>
  </label>
  <span id="result-count" aria-live="polite"></span>
</div>
</header>
__BODY__
<div id="no-results">No API symbols match this search.</div>
</main>
</div>
<script>
const q = document.getElementById("q");
const symbols = Array.from(document.querySelectorAll(".sym"));
const sections = Array.from(document.querySelectorAll("main section"));
const navSymbols = Array.from(document.querySelectorAll(".nav-symbol"));
const resultCount = document.getElementById("result-count");
const noResults = document.getElementById("no-results");

function matches(text, terms) {
  return terms.every(term => text.includes(term));
}

function applyFilter() {
  const terms = q.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
  let visibleCount = 0;
  symbols.forEach(el => {
    const ownHit = !terms.length || matches(el.dataset.search || "", terms);
    let memberHit = false;
    el.querySelectorAll("li").forEach(li => {
      const hit = !terms.length || ownHit || matches(li.dataset.search || "", terms);
      li.classList.toggle("hide", !hit);
      memberHit = memberHit || hit;
    });
    const hit = ownHit || memberHit;
    el.classList.toggle("hide", !hit);
    if (hit) visibleCount++;
  });

  sections.forEach(sec => {
    const any = Array.from(sec.querySelectorAll(".sym"))
      .some(el => !el.classList.contains("hide"));
    sec.classList.toggle("hide", !any);
  });

  navSymbols.forEach(link => {
    const target = document.getElementById(link.dataset.target);
    link.classList.toggle("hide", !target || target.classList.contains("hide"));
  });
  document.querySelectorAll(".nav-group").forEach(group => {
    const section = document.getElementById(group.dataset.section);
    group.classList.toggle("hide", !section || section.classList.contains("hide"));
  });

  resultCount.textContent = visibleCount + (visibleCount === 1 ? " symbol" : " symbols");
  noResults.classList.toggle("show", visibleCount === 0);
}

q.addEventListener("input", applyFilter);
q.addEventListener("search", applyFilter);
document.addEventListener("keydown", event => {
  if (event.key === "/" && document.activeElement !== q) {
    event.preventDefault();
    q.focus();
  } else if (event.key === "Escape" && document.activeElement === q) {
    q.value = "";
    applyFilter();
    q.blur();
  }
});
applyFilter();
</script>
</body>
</html>
'''


def main():
    parser = argparse.ArgumentParser(description="Build camber public API HTML.")
    parser.add_argument(
        "-o", "--output",
        default=os.path.join(HERE, "camber-api.html"),
        help="output HTML path",
    )
    args = parser.parse_args()
    page = render(collect())
    out = os.path.abspath(args.output)
    with open(out, "w", encoding="utf-8") as f:
        f.write(page)
    print(out)


if __name__ == "__main__":
    main()
