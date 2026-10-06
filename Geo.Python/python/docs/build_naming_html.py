#!/usr/bin/env python
"""Build an example-led naming guide with the API reference's visual styling."""
import argparse
import html
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Exact compositions audited against EntityNaming, AnchorMesh, BooleanFaceLineage,
# FaceLineageNaming, Python naming.py, and their regression tests.
# Each item is (input, resulting name, short qualification).
SECTIONS = [
    ("From sketches to faces", [
        ("Extrude an authored sketch", [
            ('sketch = "outline"\ncurve = "north"\nextrude result = "case"',
             'case-north\ncase-ExtrudeBottom\ncase-ExtrudeTop',
             "The side uses result-curve. The sketch name is not included. The two cap labels are reserved."),
            ('cuboid result = "box"', 'box-Line1\nbox-Line2\nbox-Line3\nbox-Line4\nbox-ExtrudeBottom\nbox-ExtrudeTop',
             "A cuboid internally extrudes a four-line sketch."),
            ('cylinder result = "pin"', 'pin-Circle1\npin-ExtrudeBottom\npin-ExtrudeTop',
             "An extruded cylinder internally uses Circle1."),
        ]),
        ("Revolve, sweep and loft", [
            ('profile curve = "profile"\nrevolve result = "shaft"', 'shaft-profile',
             "A partial revolution also uses shaft-StartCap and shaft-EndCap when those caps exist. A sphere named ball uses ball-Arc1."),
            ('contour curve = "profile"\nguide curve = "guide"\nsweep result = "duct"', 'duct-profile-guide',
             "The guided side label combines result, contour curve and guide curve."),
            ('corresponding section curve = "north"\nloft result = "fairing"',
             'fairing-Side-north\nfairing-StartCap\nfairing-EndCap',
             "Curve-corresponding loft sides include the source curve. A loft without side correspondence may instead use fairing-Side. Caps appear only when constructed."),
        ]),
    ]),
    ("From faces to edges and points", [
        ("Combine two incident faces", [
            ('face A = "case-north"\nface B = "case-ExtrudeTop"', '[case-north,case-ExtrudeTop]',
             "One connected boundary between these faces. Matching also accepts the reversed face order."),
            ('faces = "A", "B"\ntwo disconnected boundaries', '[A,B]_1\n[A,B]_2',
             "Every connected edge is numbered when a pair has several edges. [A,B] does not select the first one."),
        ]),
        ("Choose the owner, then a point", [
            ('solid = "case"\nedge = "[case-north,case-ExtrudeTop]"',
             'case:[case-north,case-ExtrudeTop]', "Colon qualification chooses the solid. This is the form returned by solid.curve_names."),
            ('edge = "[case-north,case-ExtrudeTop]"\nnormalized distance = 0.5',
             '[case-north,case-ExtrudeTop]@0.500', "Halfway along the sampled edge. Add case: to scope it to that solid."),
            ('patch = "case-ExtrudeTop"\nUV = (0.5, 0.5)', '[case-ExtrudeTop]@0.500,0.500',
             "A point at the stored patch UV coordinates, if it is inside the trim. This is not necessarily the face centroid."),
            ('sketch = "outline"\ncurve = "north"\ncurve parameter = 0.5', 'outline:north@0.500',
             "Sketch curve parameters need not be arc-length parameters. SketchCurve @ 0.5 produces the local name north@0.500."),
            ('sketch = "holes"\ncurve = "Circle1"\ncenter handle', 'holes:Circle1@center',
             "Arc and circle center. Bezier1 control vertex 2 uses Bezier1@cv2 (zero-based)."),
        ]),
    ]),
    ("Offset sketch curves", [
        ("Keep the source curve in child names", [
            ('source curve = "north"\noutward offset', 'north@out_offset',
             "Inward uses north@in_offset. A second fragment uses north@out_offset_2, then _3, and so on."),
            ('source curve = "north"\noutline of an open curve', 'north@start_cap\nnorth@end_cap',
             "Caps closing the two ends of the offset outline."),
            ('offset curves = "v@out_offset", "h@out_offset"\njoin these curves', 'cap[h@out_offset,v@out_offset]',
             "The pair is ordinal-sorted. Additional join fragments use _2, _3, ... ."),
            ('sketch = "outline"\noffset curve = "north@out_offset"\nparameter = 0.5', 'outline:north@out_offset@0.500',
             "The offset label is a curve name; the final @ addresses a point on it."),
        ]),
    ]),
    ("Boolean results and split faces", [
        ("A result name does not replace every face name", [
            ('source face = "base-ExtrudeTop"\nBoolean result = "finished"', 'base-ExtrudeTop',
             "A surviving source face keeps its label. To select it specifically on the result, use finished:base-ExtrudeTop."),
            ('first operand = "first", patch = "panel"\nsecond operand = "second", distinct patch = "panel"',
             'first-panel\nsecond-panel',
             "Conflicting labels on distinct group identities are scoped to their operands; an additional numeric suffix is used if these labels are already occupied."),
        ]),
        ("Record separating boundaries", [
            ('source face = "base-ExtrudeTop"\nseparating slot boundary = "main_slot-Line2"',
             'base-ExtrudeTop{main_slot-Line2}',
             "A surviving planar region next to this side of the slot. The other region has its own boundary-derived name. A nonseparating hole does not add a separator."),
            ('fused roots = "right-ExtrudeTop", "left-ExtrudeTop"\nseparator = "main_slot-Line2"',
             'left-ExtrudeTop&right-ExtrudeTop{main_slot-Line2}',
             "Creation roots and separators are deduplicated and ordinal-sorted. Without a split, fused roots can use left-ExtrudeTop&right-ExtrudeTop."),
            ('root atom = "a&b"\nseparator atom = "c:d"', 'a%26b{c%3Ad}',
             "Provenance atoms are URI-percent-escaped before joining. Ordinary authored names are not universally escaped; avoid grammar delimiters in them."),
        ]),
        ("Disambiguate current components", [
            ('one patch label = "panel"\nthree disconnected components', 'panel\npanel_1\npanel_2',
             "Component suffixes keep current labels unique. Supported unique provenance may replace a component label; additional collisions can require other suffixes."),
            ('old face = "base-ExtrudeTop"\nnow split into separate regions', 'use result.patch_names',
             "The old ancestor can be ambiguous or obsolete. Select a current region, not an arbitrary descendant. There is no universal persistent-topology guarantee, especially for curved split ancestry."),
        ]),
    ]),
    ("Faces added by finishing operations", [
        ("Fillet and chamfer", [
            ('source edge = "[case-north,case-ExtrudeTop]"\nfillet', 'BlendEdge_[case-north,case-ExtrudeTop]',
             "The source edge label follows BlendEdge_. Corner patches, when needed, use BlendCorner_0, BlendCorner_1, ... ."),
            ('source edge = "[case-north,case-ExtrudeTop]"\nchamfer', 'ChamferEdge_[case-north,case-ExtrudeTop]',
             "The same rule with ChamferEdge_. Generated corner patches use ChamferCorner_0, ChamferCorner_1, ... ."),
        ]),
        ("Shell, thicken and cap", [
            ('source patch = "case-north"\nshell', 'ShellInner_case-north',
             "Cavity face descended from the source patch. Retained exterior patches generally keep their source labels."),
            ('opening source edge = "[case-north,case-ExtrudeTop]"', 'ShellRim_[case-north,case-ExtrudeTop]',
             "A sharp opening rim descended from that edge."),
            ('source surface patch = "panel"\nthicken', 'ThickenNegative_panel\nThickenPositive_panel\nThickenRim_1',
             "Negative/positive offset faces and the first boundary rim. Rim indices start at 1."),
            ('surface = "panel"\nfirst generated planar cap', 'SurfaceCap_panel_1',
             "Surface-cap indices start at 1."),
        ]),
    ]),
    ("Copies, patterns and assemblies", [
        ("Rewrite matching source tokens", [
            ('mesh renamed from "pipe1" to "pipe2"\npatch = "pipe1-Line1"', 'pipe2-Line1',
             "Only matching mesh prefixes change: pipe10-Line1 is not rewritten. Earlier Boolean source labels need not match the result mesh prefix."),
            ('same rename\npatch = "BlendEdge_[pipe1-a,pipe1-b]"', 'BlendEdge_[pipe2-a,pipe2-b]',
             "Matching mesh tokens inside bracketed edge/blend labels change too. Instance copies use this rewrite; CopyMesh can instead add a chosen prefix or suffix affix."),
            ('pattern seed = "seed"\npattern name = "row"\ncount = 3', 'seed\nrow_1\nrow_2',
             "Index zero is the unchanged seed. Later entries are independently named solids. Mirrors and patterns remap current names and ancestry; query each result for its actual patches."),
        ]),
        ("Distinguish assembly occurrences", [
            ('assembly = "machine"\nfirst occurrence = "module"\nfirst leaf solid = "shaft"', 'machine/module[1]/shaft[1]',
             "One-based occurrence and part positions. A second module occurrence has machine/module[2]/shaft[1]. Paths are diagnostic occurrence identities, not mesh selection addresses."),
            ('component solid = "shaft"\ncylindrical face = "shaft-profile"\ncreate an axis datum', 'shaft-profile',
             "The datum selects a current component-local face. Where supported, shaft:shaft-profile scopes it to the mesh; an assembly leaf path is not substituted for the face label."),
        ]),
    ]),
    ("Automatic and imported names", [
        ("Defaults and counters", [
            ('unnamed extrudes\nauto-name counter starts empty', 'Extrude1\nExtrude2',
             "Counters are process-global per prefix, not permanent IDs. Sketch curves have separate per-sketch counters: Line1, Arc1, Circle1, Ellipse1, Bezier1, Hermite1, Curve1."),
            ('default kernel references', 'Origin\nOriginXY\nOriginYZ\nOriginZX',
             "Python also accepts origin and sketch axes x/y (plus recognized axis aliases). Built-in origin/center aliases normalize casing; authored labels otherwise match case-sensitively."),
            ('importer assigns automatic group index 3', 'ObjAutoGroup_3\nOffAutoGroup_3\nStlAutoGroup_3',
             "Each importer uses its own prefix. OBJ can retain file group labels. Grouping depends on file content and import options. CreateFromTriangles initially labels its single group with the supplied mesh name."),
        ]),
    ]),
]


def render_naming(render_api):
    # Reuse the exact API stylesheet, but not its symbol/search layout or script.
    api_page = render_api([])
    style = api_page.split("<style>", 1)[1].split("</style>", 1)[0]
    nav, body = [], []
    for index, (title, cards) in enumerate(SECTIONS):
        section_id = "section-" + str(index)
        nav.append('<a class="nav-symbol" href="#{}">{}</a>'.format(section_id, html.escape(title)))
        body.append('<section id="{}"><h2>{}</h2>'.format(section_id, html.escape(title)))
        for heading, examples in cards:
            body.append("<article><h3>{}</h3>".format(html.escape(heading)))
            for source, result, explanation in examples:
                body.append('<div class="composition"><div><h4>Input</h4><pre>{}</pre></div>'
                            '<div><h4>Composed name</h4><pre>{}</pre></div></div><p>{}</p>'.format(
                                html.escape(source), html.escape(result), html.escape(explanation)))
            body.append("</article>")
        body.append("</section>")
    return '''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>camber naming</title><style>''' + style + '''
.composition { display: grid; grid-template-columns: 1fr 1fr; gap: 20px;
  margin: 18px 0 8px; padding-top: 10px; border-top: 1px solid var(--line); }
.composition h4 { margin: 0 0 7px; }
pre { margin: 0; padding: 12px; border-radius: 8px; background: var(--bg);
  font: 13px/1.6 ui-monospace, "Cascadia Code", Consolas, monospace;
  white-space: pre-wrap; overflow-wrap: anywhere; }
.composition > div:last-child pre { background: var(--accent-soft); }
article p { max-width: 100ch; }
article h3 { padding-right: 0; }
nav .nav-symbol { white-space: normal; }
@media (max-width: 800px) { .composition { grid-template-columns: 1fr; gap: 10px; } }
</style></head><body><div class="wrap"><nav><div class="brand">camber naming</div>
<a class="nav-section" href="camber-api.html">Python API reference</a>
''' + "\n".join(nav) + '''</nav><main><header class="hero"><h1>How names are composed</h1>
<p class="lead">Concrete inputs and the names they produce. Names are plain strings.
Use <code>solid.patch_names</code>, <code>solid.curve_names</code> and <code>solid.point_names</code>
to retrieve the current names shown in the viewer. Prefer those over guessing topology indices.</p>
<p class="lead">Qualification chooses an owner, not an arbitrary descendant. Keep authored names
unique and avoid naming-grammar delimiters. Duplicate sketch names are rejected; registering or
renaming a solid can replace an existing registered solid with that name.</p></header>
''' + "\n".join(body) + "</main></div></body></html>\n"


def main():
    from build_api_html import render
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", default=os.path.join(HERE, "camber-naming.html"))
    args = parser.parse_args()
    output = os.path.abspath(args.output)
    with open(output, "w", encoding="utf-8") as stream:
        stream.write(render_naming(render))
    print(output)


if __name__ == "__main__":
    main()
