"""Immutable component reuse is scoped to one GUI session and saved sources."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from camber.feature_source import ModelDocument
from camber.modeling import ModelSession, _TEMPLATE, _ASSEMBLY_TEMPLATE, load_component
from camber.modeling_components import _managed_geometry


def solid_source(size=3):
    return _TEMPLATE.replace("    return part, part",f"""    # camber:begin body
    body = part.cuboid((0,0,0),(1,2,{size}),name='body')
    # camber:end body
    return part, body""")


def assembly_source(component="child.py", position=10):
    return _ASSEMBLY_TEMPLATE.replace("from camber import", "from camber.modeling import load_component\nfrom camber import").replace(
        "    return part, assembly",f"""    # camber:begin component
    component = load_component({component!r})
    # camber:end component
    # camber:begin first
    first = assembly.add_part(component,position=(0,0,0))
    # camber:end first
    # camber:begin second
    second = assembly.add_part(component,position=({position},0,0))
    # camber:end second
    return part, assembly""")


class ComponentCacheTests(unittest.TestCase):
    def setUp(self):
        from camber import set_progress_log
        set_progress_log(False)

    def test_rebuild_reuses_rest_solid_and_keeps_session_ownership_and_public_load_fresh(self):
        from camber.api import Part
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            child = folder/"child.py"
            child.write_text(solid_source())
            path = folder/"main.py"
            path.write_text(assembly_source())
            session = ModelSession(path)
            original = Part.cuboid
            calls = []
            def count(*args,**kwargs):
                calls.append(True)
                return original(*args,**kwargs)
            with patch.object(Part,"cuboid",new=count):
                self.assertTrue(session.rebuild(),session.error)
                cached = session.component_cache.entries[child][0]
                self.assertTrue(session.rebuild(assembly_source(position=20)),session.error)
                self.assertEqual(1,len(calls))
                self.assertIs(cached,session.component_cache.entries[child][0])
                self.assertAlmostEqual(6,cached.volume(),places=3)
                self.assertEqual((20,0,0),session.displayed.parts[1].pose)
                leaves=session.displayed.leaves()
                self.assertAlmostEqual(0,leaves[0].bounds[0].x,places=3)
                self.assertAlmostEqual(21,leaves[1].bounds[1].x,places=3)
                from camber import glview
                vertices=[point for patch in glview._as_scene(cached).patches for point in patch["vertices"]]
                self.assertAlmostEqual(0,min(v[0] for v in vertices),places=3)
                self.assertAlmostEqual(1,max(v[0] for v in vertices),places=3)
                second = ModelSession(path)
                self.assertTrue(second.rebuild(),second.error)
                self.assertEqual(2,len(calls))
                self.assertIsNot(cached,second.component_cache.entries[child][0])
                load_component(str(child)); load_component(str(child))
                self.assertEqual(4,len(calls))

    def test_nested_saved_source_change_failure_and_obsolete_definition_invalidate(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            child=folder/"child.py"
            child.write_text(solid_source())
            parent=folder/"parent.py"
            parent.write_text(_TEMPLATE.replace("from camber import", "from camber.modeling import load_component\nfrom camber import").replace(
                "    return part, part","    # camber:begin body\n    body = load_component('child.py')\n    # camber:end body\n    return part, body"))
            path=folder/"main.py"
            path.write_text(assembly_source("parent.py"))
            session=ModelSession(path)
            self.assertTrue(session.rebuild(),session.error)
            before=session.component_cache.entries[parent][0]
            child.write_text(solid_source(5))
            self.assertTrue(session.rebuild(),session.error)
            self.assertIsNot(before,session.component_cache.entries[parent][0])
            self.assertAlmostEqual(10,session.component_cache.entries[parent][0].volume(),places=3)
            displayed=session.displayed
            child.write_text("invalid python (")
            self.assertFalse(session.rebuild())
            self.assertIs(displayed,session.displayed)
            child.write_text(solid_source(7))
            self.assertTrue(session.rebuild(),session.error)
            self.assertAlmostEqual(14,session.component_cache.entries[parent][0].volume(),places=3)
            child.write_text(parent.read_text())
            self.assertFalse(session.rebuild())
            self.assertIn("Circular component reference",session.error)
            self.assertTrue(session.rebuild(_TEMPLATE),session.error)
            self.assertFalse(session.component_cache.entries)

    def test_custom_code_assets_and_mutable_assemblies_are_not_cached(self):
        self.assertFalse(_managed_geometry(ModelDocument(solid_source().replace("from camber import","import random\nfrom camber import"))))
        self.assertFalse(_managed_geometry(ModelDocument(solid_source().replace("    return part, body","    print('custom')\n    return part, body"))))
        self.assertFalse(_managed_geometry(ModelDocument(solid_source().replace("part.cuboid((0,0,0),(1,2,3),name='body')","part.load_stl('asset.stl',30)"))))
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/"child.py").write_text(solid_source())
            (folder/"nested.py").write_text(assembly_source())
            path=folder/"main.py"
            path.write_text(_TEMPLATE.replace("from camber import","from camber.modeling import load_component\nfrom camber import").replace(
                "    return part, part","    # camber:begin nested\n    nested = load_component('nested.py')\n    # camber:end nested\n    return part, nested"))
            session=ModelSession(path)
            self.assertTrue(session.rebuild(),session.error)
            self.assertNotIn(folder/"nested.py",session.component_cache.entries)

    def test_discovery_and_prefix_evaluation_do_not_evict_other_components(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            child=folder/"child.py"
            later=folder/"later.py"
            child.write_text(solid_source())
            later.write_text(solid_source(5))
            source=assembly_source().replace("    return part, assembly",
                "    # camber:begin late\n    late = load_component('later.py')\n    # camber:end late\n    return part, assembly")
            path=folder/"main.py"
            path.write_text(source)
            session=ModelSession(path)
            self.assertTrue(session.rebuild(),session.error)
            first=session.component_cache.entries[child][0]
            last=session.component_cache.entries[later][0]
            session.build(ModelDocument(source),before=session.document.block("late").start)
            self.assertIs(last,session.component_cache.entries[later][0])
            session.build(ModelDocument(assembly_source("later.py")))
            self.assertIs(first,session.component_cache.entries[child][0])
            self.assertTrue(session.rebuild(),session.error)
            self.assertIs(first,session.component_cache.entries[child][0])
            self.assertIs(last,session.component_cache.entries[later][0])


if __name__ == "__main__":
    unittest.main()
