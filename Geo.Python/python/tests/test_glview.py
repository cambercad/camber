import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camber.display import DisplayScene
from camber import glview


class LinePointExpandTests(unittest.TestCase):
    def test_point_is_a_six_vertex_diamond(self):
        verts = glview.expand_point((3.0, 4.0, 5.0))
        self.assertEqual(6, len(verts))
        self.assertEqual(glview.DIAMOND_CORNERS, tuple(v[1] for v in verts))
        self.assertTrue(all(v[0] == (3.0, 4.0, 5.0) for v in verts))

    def test_polyline_uses_one_shared_cross_section_per_join(self):
        points = [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0), (2.0, 0.0, 0.0)]
        positions, prev_sides, nexts, indices = [], [], [], []
        glview._append_polyline_mesh(
            points, False, positions, prev_sides, nexts, indices)
        self.assertEqual(6, len(positions))
        self.assertEqual(6, len(prev_sides))
        self.assertEqual(6, len(nexts))
        self.assertEqual(12, len(indices))


class CameraRayTests(unittest.TestCase):
    def test_ortho_center_ray_follows_look(self):
        cam = glview.Camera()
        cam.eye = (0.0, -4.0, 0.0)
        cam.center = (0.0, 0.0, 0.0)
        cam.up = (0.0, 0.0, 1.0)
        cam.ortho = True
        cam.zoom = 2.0
        origin, direction = cam.ray(0.0, 0.0)
        self.assertAlmostEqual(0.0, origin[0])
        self.assertAlmostEqual(-4.0, origin[1])
        self.assertAlmostEqual(0.0, origin[2])
        self.assertAlmostEqual(0.0, direction[0])
        self.assertAlmostEqual(1.0, direction[1])
        self.assertAlmostEqual(0.0, direction[2])

    def test_view_scale_is_ortho_zoom(self):
        cam = glview.Camera()
        cam.ortho = True
        cam.zoom = 3.25
        self.assertEqual(3.25, cam.view_scale)

    def test_overlay_radii_match_the_polyscope_pixel_sizes(self):
        cam = glview.Camera()
        cam.ortho = True
        cam.zoom = 2.0
        cam.height = 800
        edge_r, point_r, wpp = glview.overlay_radii(cam)
        self.assertAlmostEqual(2.0 * 2.0 / 800.0, wpp)
        self.assertAlmostEqual(wpp * glview._EDGE_PX_ORTHO, edge_r)
        self.assertAlmostEqual(wpp * glview._POINT_PX_ORTHO, point_r)
        cam.ortho = False
        edge_r, point_r, wpp = glview.overlay_radii(cam)
        self.assertAlmostEqual(wpp * glview._EDGE_PX_PERSPECTIVE, edge_r)
        self.assertAlmostEqual(wpp * glview._POINT_PX_PERSPECTIVE, point_r)

    def test_pick_radii_default_to_framebuffer_height(self):
        cam = glview.Camera()
        cam.height = 1200
        self.assertEqual(glview.pick_radii(cam), glview.pick_radii(cam, 1200))

    def test_orbit_moves_the_eye_around_the_center(self):
        cam = glview.Camera()
        cam.eye = (0.0, -4.0, 0.0)
        cam.center = (0.0, 0.0, 0.0)
        cam.up = (0.0, 0.0, 1.0)
        before = cam.eye
        cam.orbit(0.2, 0.0)
        self.assertNotEqual(before, cam.eye)
        dist0 = math.sqrt(sum(x * x for x in before))
        dist1 = math.sqrt(sum((cam.eye[i] - cam.center[i]) ** 2 for i in range(3)))
        self.assertAlmostEqual(dist0, dist1, places=6)


class WireframeTests(unittest.TestCase):
    def test_expand_wireframe_has_barycentrics_per_triangle(self):
        verts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        positions, barycentrics = glview.expand_wireframe(verts, [(0, 1, 2)])
        self.assertEqual(verts, positions)
        self.assertEqual(
            [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)],
            barycentrics,
        )


class MeshPickTests(unittest.TestCase):
    def test_ray_hits_the_nearer_triangle(self):
        verts = [
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 0.0, 2.0), (1.0, 0.0, 2.0), (0.0, 1.0, 2.0),
        ]
        faces = [(0, 1, 2), (3, 4, 5)]
        hit = glview.closest_triangle((0.2, 0.2, 10.0), (0.0, 0.0, -1.0), verts, faces)
        self.assertEqual(1, hit)
        cache = glview.build_triangle_cache(verts, faces)
        cached_hit = glview.closest_triangle(
            (0.2, 0.2, 10.0), (0.0, 0.0, -1.0), verts, faces, cache)
        self.assertEqual(hit, cached_hit)


class ImguiCompatTests(unittest.TestCase):
    def test_polyscope_style_aliases_exist(self):
        from camber.imgui_compat import import_imgui
        imgui = import_imgui()
        self.assertTrue(callable(imgui.BeginTabBar))
        self.assertTrue(callable(imgui.BeginTabItem))
        self.assertTrue(callable(imgui.Checkbox))

    def test_viewer_style_keeps_window_background_translucent(self):
        from camber.imgui_compat import apply_viewer_style, import_imgui
        imgui = import_imgui()
        imgui.create_context()
        apply_viewer_style(imgui)
        bg = tuple(imgui.get_style().colors[imgui.COLOR_WINDOW_BACKGROUND])
        self.assertLess(bg[3], 0.7)
        self.assertGreater(bg[3], 0.3)
        self.assertEqual(imgui.WINDOW_NO_TITLE_BAR, imgui.ImGuiWindowFlags_NoTitleBar)
        self.assertEqual(imgui.KEY_ENTER, imgui.ImGuiKey_Enter)


class ScenePackTests(unittest.TestCase):
    def test_supplied_normals_are_preserved_when_another_patch_has_none(self):
        supplied = [(0.25, 0.5, 0.75)] * 3
        scene = DisplayScene()
        scene.patches.extend([
            {
                "name": "supplied",
                "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
                "faces": [(0, 1, 2)],
                "normals": supplied,
            },
            {
                "name": "generated",
                "vertices": [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0)],
                "faces": [(0, 1, 2)],
            },
        ])
        packed = glview.pack_scene(scene)
        self.assertEqual(supplied, packed["mesh_n"][:3])

    def test_supplied_uvs_are_packed_for_the_shader(self):
        uvs = [(0.0, 0.0), (1.0, 0.0), (0.25, 1.0)]
        scene = DisplayScene()
        scene.patches.append({
            "name": "face",
            "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            "faces": [(0, 1, 2)],
            "uvs": uvs,
        })
        packed = glview.pack_scene(scene)
        self.assertEqual(uvs, packed["mesh_uv"])

    def test_checkerboard_darkens_odd_cells(self):
        self.assertEqual(1.0, glview.checkerboard_scale((0.0, 0.0)))
        self.assertEqual(0.75, glview.checkerboard_scale((0.15, 0.0)))
        self.assertEqual(0.75, glview.checkerboard_scale((0.0, 0.15)))
        self.assertEqual(1.0, glview.checkerboard_scale((0.15, 0.15)))
        self.assertEqual(glview._CHECKER_DARKEN, glview.checkerboard_scale((0.15, 0.0)))

    def test_mesh_shader_evaluates_uv_checkerboard(self):
        self.assertIn("in vec2 in_uv", glview._MESH_VERT)
        self.assertIn("v_uv * 10.0", glview._MESH_FRAG)
        self.assertIn("0.75", glview._MESH_FRAG)

    def test_rendered_and_pick_curves_share_cleaned_points(self):
        scene = DisplayScene()
        scene.curves.append({
            "name": "edge",
            "points": [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
            "closed": False,
        })
        packed = glview.pack_scene(scene)
        curve = next(item for item in packed["catalog"] if item["kind"] == "curve")
        self.assertEqual([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], curve["points"])

    def test_coincident_point_aliases_have_one_drawable(self):
        scene = DisplayScene()
        scene.points.extend([
            {"name": "edge-b@0.000", "position": (1.0, 2.0, 3.0)},
            {"name": "edge-a@1.000", "position": (1.0, 2.0, 3.0)},
        ])
        packed = glview.pack_scene(scene)
        self.assertEqual(6, len(packed["point_pos"]))
        self.assertEqual(["edge-a@1.000"] * 6, packed["point_names"])
        self.assertEqual(2, len(packed["catalog"]))
        self.assertEqual(
            ("edge-a@1.000", "edge-b@0.000"),
            glview.point_proxy_aliases(packed, "edge-b@0.000"),
        )
        self.assertEqual(
            "edge-a@1.000",
            glview.pick_name(
                packed, (1.0, 2.0, 10.0), (0.0, 0.0, -1.0), 0.1, 0.1),
        )

    def test_named_curve_and_point_stay_pickable(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "box:top",
            "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.curves.append({
            "name": "box:[top,side]",
            "points": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
            "closed": False,
        })
        scene.points.append({"name": "box:[top,side]@0.000", "position": (0.0, 0.0, 0.0)})
        packed = glview.pack_scene(scene)
        self.assertEqual(1, len(packed["face_names"]))
        self.assertEqual("box:top", packed["face_names"][0])
        self.assertEqual(4, len(packed["line_start"]))
        self.assertEqual(12, len(packed["line_cap_pos"]))
        self.assertEqual(6, len(packed["point_pos"]))
        names = [item["name"] for item in packed["catalog"]]
        self.assertIn("box:[top,side]", names)
        self.assertIn("box:[top,side]@0.000", names)

    def test_click_near_edge_picks_curve_not_face(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "box:top",
            "vertices": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.curves.append({
            "name": "box:[top,side]",
            "points": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            "closed": False,
        })
        scene.points.append({"name": "box:[top,side]@0.000", "position": (0.0, 0.0, 0.0)})
        packed = glview.pack_scene(scene)
        down = (0.0, 0.0, -1.0)
        self.assertEqual(
            "box:[top,side]@0.000",
            glview.pick_name(packed, (0.05, 0.05, 10.0), down, 0.2, 0.2),
        )
        self.assertEqual(
            "box:[top,side]",
            glview.pick_name(packed, (1.0, 0.05, 10.0), down, 0.2, 0.2),
        )
        self.assertEqual(
            "box:top",
            glview.pick_name(packed, (0.7, 0.7, 10.0), down, 0.2, 0.2),
        )

    def test_nearer_surface_occludes_curve_for_picking(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "front",
            "vertices": [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0), (0.0, 2.0, 1.0)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.curves.append({
            "name": "rear-edge",
            "points": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            "closed": False,
        })
        packed = glview.pack_scene(scene)
        self.assertEqual(
            "front",
            glview.pick_name(
                packed, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
        )

    def test_adjacent_surface_allows_concave_edge_depth_tolerance(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "part:front",
            "vertices": [(0.0, 0.0, 0.5), (2.0, 0.0, 0.5), (0.0, 2.0, 0.5)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.curves.append({
            "name": "part:[front,side]",
            "points": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            "closed": False,
        })
        packed = glview.pack_scene(scene)
        self.assertEqual(
            "part:[front,side]",
            glview.pick_name(
                packed, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
        )

    def test_point_wins_within_same_depth_layer(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "surface",
            "vertices": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.curves.append({
            "name": "curve",
            "points": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
            "closed": False,
        })
        scene.points.append({
            "name": "point",
            "position": (1.0, 0.05, -0.05),
        })
        packed = glview.pack_scene(scene)
        uncached = dict(packed)
        uncached["catalog_index"] = None
        self.assertEqual(
            "point",
            glview.pick_name(
                packed, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
        )
        self.assertEqual(
            glview.pick_name(
                uncached, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
            glview.pick_name(
                packed, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
        )

    def test_surface_still_occludes_point_outside_depth_layer(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "surface",
            "vertices": [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)],
            "faces": [(0, 1, 2)],
            "normals": [(0.0, 0.0, 1.0)] * 3,
        })
        scene.points.append({
            "name": "hidden-point",
            "position": (1.0, 0.05, -1.0),
        })
        packed = glview.pack_scene(scene)
        self.assertEqual(
            "surface",
            glview.pick_name(
                packed, (1.0, 0.05, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2),
        )

    def test_segment_color_matches_the_csharp_hue(self):
        self.assertEqual(glview.segment_color(0, 1), glview.segment_color(0, 4))
        a = glview.segment_color(0, 4)
        b = glview.segment_color(1, 4)
        self.assertNotEqual(a, b)
        self.assertTrue(all(c >= 0.3 for c in a))


class AssemblyPartVisibilityTests(unittest.TestCase):
    def _two_part_scene(self, with_curves=False):
        scene = DisplayScene()
        scene.patches.extend([
            {
                "name": "pipe1:flange",
                "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
                "faces": [(0, 1, 2)],
            },
            {
                "name": "pipe2:flange",
                "vertices": [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0), (2.0, 1.0, 0.0)],
                "faces": [(0, 1, 2)],
            },
        ])
        if with_curves:
            scene.curves.extend([
                {
                    "name": "pipe1:edge",
                    "points": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
                    "closed": False,
                },
                {
                    "name": "pipe2:edge",
                    "points": [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0)],
                    "closed": False,
                },
            ])
        return glview.pack_scene(scene)

    def test_part_names_come_from_qualified_prefixes(self):
        packed = self._two_part_scene()
        self.assertEqual(["pipe1", "pipe2"], glview.packed_part_names(packed))

    def test_split_keeps_each_part_and_flat_line_indices(self):
        packed = self._two_part_scene(with_curves=True)
        self.assertTrue(packed["line_idx"])
        self.assertTrue(all(isinstance(i, int) for i in packed["line_idx"]))
        parts = dict(glview.split_packed_by_part(packed))
        self.assertEqual(["pipe1", "pipe2"], list(parts))
        self.assertEqual(["pipe1:flange"], parts["pipe1"]["face_names"])
        self.assertEqual(["pipe2:flange"], parts["pipe2"]["face_names"])
        self.assertEqual(len(parts["pipe1"]["mesh_pos"]), len(parts["pipe1"]["mesh_uv"]))
        self.assertTrue(all(isinstance(i, int) for i in parts["pipe1"]["line_idx"]))
        self.assertTrue(parts["pipe1"]["line_idx"])
        self.assertTrue(parts["pipe2"]["line_idx"])

    def test_pick_skips_hidden_part_packeds(self):
        packed = self._two_part_scene()
        parts = dict(glview.split_packed_by_part(packed))
        hit = glview.pick_name(
            packed, (0.2, 0.2, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2,
            packeds=[parts["pipe2"]])
        self.assertNotEqual("pipe1:flange", hit)
        self.assertEqual("pipe2:flange", glview.pick_name(
            packed, (2.2, 0.2, 10.0), (0.0, 0.0, -1.0), 0.2, 0.2,
            packeds=[parts["pipe2"]]))


class SelectionDisplayTests(unittest.TestCase):
    def test_display_prefixes_known_surface_type(self):
        names = ["base/base:top"]
        types = {"base/base:top": "pln"}
        self.assertEqual(
            'pln:"base/base:top"',
            glview._selection_display(names, types))

    def test_display_leaves_unknown_unprefixed(self):
        self.assertEqual(
            '"mesh:face"',
            glview._selection_display(["mesh:face"], {}))

    def test_display_inherits_curve_tag_for_point_pick(self):
        types = {"mesh:edge": "lin"}
        self.assertEqual(
            'lin:"mesh:edge@0.500"',
            glview._selection_display(["mesh:edge@0.500"], types))

    def test_pack_scene_carries_entity_types(self):
        scene = DisplayScene()
        scene.patches.append({
            "name": "part:cap",
            "vertices": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            "faces": [(0, 1, 2)],
            "surface_type": 1,
        })
        packed = glview.pack_scene(scene)
        self.assertEqual({"part:cap": "pln"}, packed.get("entity_types"))


if __name__ == "__main__":
    unittest.main()
