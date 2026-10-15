"""Architectural component dimensions, reuse, validation and editable documents."""
from dataclasses import replace
import math
from pathlib import Path
import unittest

from camber import Part, Frame
from camber.modeling import ModelSession
from camber.modeling_commands import operations
from examples.architecture.roofs import RoofParameters, build_roof
from examples.architecture.roof_coverings import RoofCoveringParameters, build_roof_covering
from examples.architecture.stairs import StairParameters, build_stairs
from examples.architecture.rainwater import GutterParameters, DownpipeParameters, build_gutter, build_downpipe


class BuildingParameterTests(unittest.TestCase):
    def test_dataclass_literal_options_use_the_existing_gui_dropdowns(self):
        operation = operations()['roof_component']
        shape = next(p for p in operation.parameters if p.name == 'shape')
        self.assertEqual(('gable', 'hip', 'flat'), operation.choices(shape))

    def test_invalid_construction_inputs_are_rejected(self):
        cases = ((RoofParameters, ({'width': True}, {'shape': 'unknown'}, {'pitch': 0}, {'pitch': 90}, {'length': math.inf})),
                 (RoofCoveringParameters, ({'material': 'stone'}, {'shape': 'flat'}, {'headlap': 380}, {'detail': 'unknown'}, {'ridge_caps': 1})),
                 (StairParameters, ({'risers': True}, {'risers': 1}, {'tread_thickness': 200}, {'stringer_depth': 100}, {'railings': 'yes'})),
                 (GutterParameters, ({'length': 50}, {'wall_thickness': 100}, {'outlet_offset': 0}, {'fall': math.nan}, {'end_caps': 1})),
                 (DownpipeParameters, ({'points': ((0, 0, 0),)}, {'points': ((0, 0, 0), (0, 0, 0))},
                                       {'points': ((0, 0, 0), (0, 0, 100), (0, 0, 0))},
                                       {'points': ((0, 0, 0), (0, 0, 10), (10, 0, 10))}, {'diameter': 250})))
        for parameter_type, values in cases:
            for value in values:
                with self.subTest(parameters=parameter_type.__name__, value=value), self.assertRaises(ValueError):
                    parameter_type(**value)

    def test_routes_and_colors_are_copied_and_parameters_are_replaceable(self):
        points = [[0, 0, 1000], [0, 0, 0]]
        p = DownpipeParameters(points=points)
        points[0][2] = -1
        self.assertEqual(1000, p.points[0][2])
        self.assertEqual(900, replace(StairParameters(), width=900).width)


class BuildingGeometryTests(unittest.TestCase):
    def check_model(self, model):
        self.assertTrue(model.assembly.solve().converged)
        for solid in model.solids:
            with self.subTest(solid=solid.name):
                self.assertTrue(solid.is_watertight())
                self.assertGreater(solid.volume(), 0)

    def test_roof_joints_preserve_footprint_volume_for_all_shapes(self):
        for shape, width, length in (('gable', 1200, 1600), ('hip', 1200, 1600),
                                     ('hip', 1600, 1200), ('hip', 1200, 1200), ('flat', 1200, 1600)):
            p = RoofParameters(shape=shape, width=width, length=length, overhang=100, thickness=100)
            model = build_roof(p)
            self.check_model(model)
            expected = (p.width+2*p.overhang)*(p.length+2*p.overhang)*p.thickness
            self.assertAlmostEqual(1, sum(s.volume() for s in model.solids)/expected, delta=1e-4)

    def test_coverings_trim_hips_and_reuse_full_tiles(self):
        p = RoofCoveringParameters(width=1200, length=1600, overhang=0)
        for config in (p, replace(p, shape='hip'), replace(p, material='metal', sheet_thickness=.8),
                       replace(p, material='metal', shape='hip', sheet_thickness=.8), replace(p, detail='surface')):
            with self.subTest(config=config):
                model = build_roof_covering(config)
                self.check_model(model)
                if config == p:
                    self.assertEqual(2, sum(s.name.endswith('_tile') for s in model.solids))
                    self.assertGreater(len(model.assembly.leaves()), len(model.solids)+10)
                if config.shape == 'hip':
                    self.assertTrue(any('_trimmed_' in solid.name or '_sheet' in solid.name for solid in model.solids))

    def test_tiles_have_real_channels_nibs_and_through_fixing_holes(self):
        from examples.architecture._roof_details import clay_tile
        for profile in ('pantile', 'plain'):
            p = RoofCoveringParameters(tile_profile=profile)
            owner = Part((-1000, -1000, -1000), (2000, 2000, 2000), tolerance=.025)
            tile = clay_tile(owner, Frame(), p, 'tested_tile')
            self.assertTrue(tile.is_watertight())
            x, y = p.tile_width*.28, p.tile_length-38
            self.assertIsNone(tile.raycast((x, y, 100), (0, 0, -200)))
            self.assertIsNotNone(tile.raycast((x+10, y, 100), (0, 0, -200)))
            self.assertLess(tile.volume(), p.tile_width*p.tile_length*p.tile_thickness*1.15)
            # Underside lugs extend below the tile's body, independently of its colour.
            self.assertIsNotNone(tile.raycast((x, p.tile_length-20, -7), (0, 0, 5)))

    def test_gutter_hangers_are_drilled_and_stair_nosings_and_mortises_are_geometry(self):
        p = GutterParameters(length=800, fall=0)
        gutter = build_gutter(p)
        hanger = next(s for s in gutter.solids if s.name.endswith('_hanger'))
        from camber.modeling import _pack_result
        low, high = _pack_result(gutter.part, hanger)['bounds']
        y = (low[1]+high[1])/2
        self.assertIsNone(hanger.raycast((-p.radius-20, y, p.elevation+55), (50, 0, 0)))
        self.assertIsNotNone(hanger.raycast((-p.radius-20, y, p.elevation+70), (50, 0, 0)))
        p = StairParameters(construction='timber', railing_style='timber', risers=3, total_rise=525, landing_length=400)
        stairs = build_stairs(p)
        tread = next(s for s in stairs.solids if s.name.endswith('_tread1'))
        section = (p.going+p.nosing)*p.tread_thickness-2*p.nosing_radius**2*(1-math.pi/4)
        self.assertAlmostEqual(1, tread.volume()/(section*p.width), delta=.001)
        post = next(s for s in stairs.solids if s.name.endswith('_left_newel1'))
        blank_height = p.total_rise/p.risers+p.railing_height+p.handrail_height+60
        self.assertLess(post.volume(), p.newel_width**2*blank_height)

    def test_stairs_have_one_fewer_treads_than_risers_and_optional_landing(self):
        p = StairParameters(total_rise=875, risers=5, landing_length=500)
        for config in (p, replace(p, railings=False, closed_risers=False, landing_length=0),
                       replace(p, risers=2, total_rise=350, stringer_depth=600)):
            model = build_stairs(config)
            self.check_model(model)
            names = [s.name for s in model.solids]
            self.assertEqual(config.risers-1, sum('_tread' in n for n in names))
            self.assertEqual(config.risers if config.closed_risers else 0, sum('_riser' in n for n in names))
            self.assertEqual(bool(config.landing_length), 'stairs_landing' in names)
            self.assertEqual(config.railings, any('_handrail' in n for n in names))

    def test_modern_stair_construction_and_guard_variants(self):
        base = StairParameters(risers=3, total_rise=525, landing_length=400)
        for p in (base, replace(base, stringer_layout='sides', railing_style='glass'),
                  replace(base, construction='concrete', closed_risers=True, tread_material='none')):
            with self.subTest(construction=p.construction, guard=p.railing_style):
                model = build_stairs(p)
                names = [s.name for s in model.solids]
                for solid in model.solids:
                    self.assertTrue(solid.is_watertight(), solid.name)
                    self.assertGreater(solid.volume(), 0, solid.name)
                if p.construction == 'steel':
                    self.assertEqual(1 if p.stringer_layout == 'central' else 2,
                                     sum(n.endswith(('_stringer1', '_stringer2')) for n in names))
                    tread = next(s for s in model.solids if s.name.endswith('_tread1'))
                    from examples.architecture._stair_structure import fixing_positions
                    x, y = fixing_positions(p, 0)[0]
                    self.assertIsNone(tread.raycast((x, y, 200), (0, 0, -100)))
                    self.assertTrue(any('_saddle' in n for n in names))
                else:
                    self.assertTrue(any('_concrete_waist' in n for n in names))
                    self.assertFalse(any('_tread' in n for n in names))
                if p.railing_style == 'glass':
                    self.assertTrue(any('_glass0' in n for n in names))
                    clamp = next(s for s in model.solids if s.name.endswith('_glass_clamp'))
                    self.assertLess(clamp.volume(), (p.glass_thickness+16)*30*40)

    def test_gutter_has_a_real_outlet_and_pipe_is_hollow_through_bends(self):
        for end_caps in (True, False):
            model = build_gutter(GutterParameters(length=1200, end_caps=end_caps))
            self.check_model(model)
            self.assertEqual(1100, model.outlet[1])
            self.assertEqual(5 if end_caps else 3, len(model.solids))
        for points in (((0, 0, 1000), (0, 0, 0)),
                       ((0, 0, 1000), (0, 0, 300), (300, 0, 300)),
                       ((0, 0, 1000), (0, 0, 500), (300, 0, 500), (300, 300, 500))):
            p = DownpipeParameters(points=points, collars=False)
            model = build_downpipe(p)
            self.check_model(model)
            path_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
            path_length -= (len(points)-2)*p.bend_radius*(2-math.pi/2)
            area = math.pi*((p.diameter/2)**2-(p.diameter/2-p.wall_thickness)**2)
            self.assertAlmostEqual(1, model.solids[0].volume()/(area*path_length), delta=.02)

    def test_components_share_a_part_and_registered_gui_documents_rebuild(self):
        part = Part((-10000, -10000, -2000), (20000, 20000, 20000), tolerance=.025)
        roof = build_roof(RoofParameters(width=1200, length=1600), part=part)
        stairs = build_stairs(StairParameters(risers=3, total_rise=525, railings=False), part=part)
        self.assertIs(part, roof.part)
        self.assertIs(part, stairs.part)
        root = Path(__file__).resolve().parents[1]/'examples'/'architecture'
        for filename in ('parametric_roof.py', 'parametric_stairs.py'):
            session = ModelSession(root/filename)
            self.assertTrue(session.rebuild(), session.error)
            components = [block for block in session.document.blocks if block.operation and block.operation.endswith('_component')]
            self.assertTrue(components)
            self.assertTrue(all(block.editable for block in components))
            self.assertTrue(all(block.operation in operations() for block in components))


if __name__ == '__main__':
    unittest.main()
