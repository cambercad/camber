"""Pyglet window + imgui + pyglet GL. Used by solids and the sketch UI."""

import ctypes
import math

from . import glview as _g
from .clipboard import copy_to_clipboard


class MeshHandle(object):
    def __init__(self, viewer, name):
        self._viewer = viewer
        self._name = name

    def set_enabled(self, on):
        rec = self._viewer._overlays.get(self._name)
        if rec is not None:
            rec["enabled"] = bool(on)

    def set_transparency(self, amount):
        rec = self._viewer._overlays.get(self._name)
        if rec is not None:
            rec["alpha"] = max(0.0, min(1.0, float(amount)))

    def set_selection_mode(self, _mode):
        return None

    def set_smooth_shade(self, _on):
        return None

    def update_vertex_positions(self, verts):
        self._viewer._update_overlay_verts(self._name, verts)


class _CamParams(object):
    def __init__(self, cam):
        self._cam = cam

    def get_position(self):
        return self._cam.eye

    def get_look_dir(self):
        return self._cam.frame()[0]

    def get_up_dir(self):
        return self._cam.frame()[1]

    def get_right_dir(self):
        return self._cam.frame()[2]

    def get_aspect(self):
        return self._cam.aspect

    def get_fov_vertical_deg(self):
        return math.degrees(self._cam.fov)


class Viewer(object):
    """Window host. Sketch code talks to this the way it used to talk to Polyscope."""

    def __init__(self, title="Camber"):
        pyglet, np, imgui, create_renderer = _g._import_gl()
        self._pyglet = pyglet
        self._np = np
        self._imgui = imgui
        self._gl = pyglet.gl
        w, h = _g._WINDOW_W, _g._WINDOW_H
        try:
            screen = pyglet.canvas.get_display().get_default_screen()
            w = min(w, int(screen.width))
            h = min(h, int(screen.height))
        except Exception:
            pass
        window = None
        for major, minor in ((4, 0), (3, 3)):
            try:
                config = pyglet.gl.Config(
                    double_buffer=True,
                    depth_size=24,
                    sample_buffers=0,
                    samples=0,
                    major_version=major,
                    minor_version=minor,
                )
                window = pyglet.window.Window(w, h, title, resizable=True, config=config)
                break
            except Exception:
                window = None
        if window is None:
            window = pyglet.window.Window(w, h, title, resizable=True)
        self.window = window
        self.title = title
        imgui.create_context()
        from .imgui_compat import apply_viewer_style
        apply_viewer_style(imgui)
        self._impl = create_renderer(window)
        self._batch = pyglet.graphics.Batch()
        self._mesh_prog = self._compile(_g._MESH_VERT, _g._MESH_FRAG)
        self._wire_prog = self._compile(_g._WIRE_VERT, _g._WIRE_FRAG)
        self._line_prog = self._compile(_g._LINE_VERT, _g._LINE_FRAG)
        self._point_prog = self._compile(_g._POINT_VERT, _g._POINT_FRAG)
        self._flat_prog = self._compile(_g._FLAT_VERT, _g._FLAT_FRAG)
        self._blit_prog = self._compile(_g._BLIT_VERT, _g._BLIT_FRAG)
        self._resolve_prog = self._compile(_g._BLIT_VERT, _g._RESOLVE_FRAG)
        self._depth_copy_prog = self._compile(_g._BLIT_VERT, _g._DEPTH_COPY_FRAG)
        quad = (-1.0, -1.0, 0.0, 3.0, -1.0, 0.0, -1.0, 3.0, 0.0)
        self._blit_vl = self._blit_prog.vertex_list(
            3, self._gl.GL_TRIANGLES, in_pos=("f", quad))
        self._resolve_vl = self._resolve_prog.vertex_list(
            3, self._gl.GL_TRIANGLES, in_pos=("f", quad))
        self._depth_copy_vl = self._depth_copy_prog.vertex_list(
            3, self._gl.GL_TRIANGLES, in_pos=("f", quad))
        self.camera = _g.Camera()
        self._tick = None
        self._overlays = {}
        self._solid = None
        self._hidden_parts = set()
        self._wireframe = False
        self._solid_transparent = False
        self._held = set()
        self._transparency = "none"
        self._peel = None
        self._scene = None
        self._draw_fbo = 0
        self._draw_w = 1
        self._draw_h = 1
        self._draw_samples = 1
        self._draw_ssaa = 1
        gl = self._gl
        self._dummy_depth = self._make_tex(
            gl.GL_DEPTH_COMPONENT24, gl.GL_DEPTH_COMPONENT, gl.GL_UNSIGNED_INT, 1, 1, True)
        window.push_handlers(self)
        # pyglet's Window.on_key_press closes on Esc. Keep Esc for selection.
        window.on_key_press = self._ignore_default_escape
        self._sync_size()
        try:
            from .view import _center_native_window
            _center_native_window(title)
        except Exception:
            pass

    def _compile(self, vert, frag):
        shader = self._pyglet.graphics.shader
        return shader.ShaderProgram(shader.Shader(vert, "vertex"), shader.Shader(frag, "fragment"))

    def _flat(self, array):
        return self._np.asarray(array, dtype="f4").ravel()

    def _colors(self, n, rgb):
        n = int(n)
        if n <= 0:
            return self._np.zeros((0, 3), dtype="f4")
        return self._np.repeat(self._np.asarray([rgb], dtype="f4"), n, axis=0)

    def _indexed(self, prog, nvert, indices, batch=None, **attrs):
        gl = self._gl
        data = {}
        for name, array in attrs.items():
            data[name] = ("f", self._flat(array))
        idx = self._np.asarray(indices, dtype="i4").ravel()
        return prog.vertex_list_indexed(
            int(nvert), gl.GL_TRIANGLES, idx, batch=batch or self._batch, **data)

    def _array(self, prog, nvert, **attrs):
        gl = self._gl
        data = {}
        for name, array in attrs.items():
            data[name] = ("f", self._flat(array))
        return prog.vertex_list(int(nvert), gl.GL_TRIANGLES, batch=self._batch, **data)

    def _delete(self, vl):
        if vl is None:
            return
        try:
            vl.delete()
        except Exception:
            pass

    def _draw(self, prog, vl):
        if vl is None:
            return
        prog.use()
        vl.draw(self._gl.GL_TRIANGLES)

    def _make_wire(self, verts, faces):
        if not verts or not faces:
            return None
        positions, barycentrics = _g.expand_wireframe(verts, faces)
        if not positions:
            return None
        return self._array(
            self._wire_prog,
            len(positions),
            in_pos=positions,
            in_barycentric=barycentrics,
        )

    def _ignore_default_escape(self, symbol, modifiers):
        return None

    def on_key_press(self, symbol, modifiers):
        if symbol in self._held:
            return None
        self._held.add(symbol)
        self._letter(symbol, True)
        if self._wants_keyboard():
            return None
        if symbol == self._pyglet.window.key.W:
            self._wireframe = not self._wireframe
        elif symbol == self._pyglet.window.key.T:
            self._solid_transparent = not self._solid_transparent
            self._transparency = "pretty" if self._solid_transparent else "none"
        return None

    def on_key_release(self, symbol, modifiers):
        self._held.discard(symbol)
        self._letter(symbol, False)

    def _wants_keyboard(self):
        try:
            return bool(self._imgui.GetIO().WantCaptureKeyboard)
        except Exception:
            return False

    def _letter(self, symbol, down):
        try:
            name = self._pyglet.window.key.symbol_string(symbol)
        except Exception:
            return
        if not name or len(name) != 1 or not name.isalpha():
            return
        letters = getattr(self._imgui, "_camber_letters", None)
        if letters is None:
            return
        if down:
            letters.add(name.upper())
        else:
            letters.discard(name.upper())

    def on_resize(self, width, height):
        self._sync_size()

    def on_close(self):
        try:
            self._impl.shutdown()
        except Exception:
            pass

    def on_draw(self):
        self._sync_size()
        if hasattr(self._impl, "process_inputs"):
            self._impl.process_inputs()
        self._imgui.new_frame()
        if self._tick is not None:
            self._tick()
        self._draw_scene()
        self._bind_window()
        self._imgui.render()
        gl = self._gl
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        try:
            self._impl.render(self._imgui.get_draw_data())
        finally:
            gl.glDisable(gl.GL_BLEND)
            gl.glEnable(gl.GL_DEPTH_TEST)

    def _sync_size(self):
        w, h = self.window.get_size()
        fb = getattr(self.window, "get_framebuffer_size", None)
        if fb is not None:
            try:
                fw, fh = fb()
            except Exception:
                fw, fh = w, h
        else:
            fw, fh = w, h
        self.camera.width = max(int(fw), 1)
        self.camera.height = max(int(fh), 1)
        self._gl.glViewport(0, 0, self.camera.width, self.camera.height)

    def capture_rgba(self):
        """Read the resolved 3D framebuffer (no imgui). Bottom row first is flipped to top-down."""
        self._sync_size()
        self._draw_scene()
        self._bind_window()
        gl = self._gl
        w, h = int(self.camera.width), int(self.camera.height)
        buf = (ctypes.c_ubyte * (w * h * 4))()
        gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)
        gl.glReadBuffer(gl.GL_BACK)
        gl.glReadPixels(0, 0, w, h, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, buf)
        arr = self._np.frombuffer(buf, dtype=self._np.uint8).reshape(h, w, 4)
        return arr[::-1].copy()

    def set_user_callback(self, fn):
        self._tick = fn

    def show(self):
        self._pyglet.app.run()

    def unshow(self):
        try:
            self.window.close()
        except Exception:
            pass

    def set_solid(self, packed):
        self._hidden_parts = set()
        self._release_solid()
        parts = []
        for name, subset in _g.split_packed_by_part(packed):
            gpu = self._upload_solid(subset)
            gpu["name"] = name
            parts.append(gpu)
        self._solid = {"packed": packed, "parts": parts}

    def set_hidden_parts(self, hidden):
        self._hidden_parts = set(hidden or [])

    def _gpu_parts(self):
        solid = self._solid
        if solid is None:
            return []
        return solid.get("parts") or []

    def _visible_gpu_parts(self):
        hidden = getattr(self, "_hidden_parts", set())
        return [part for part in self._gpu_parts() if part.get("name") not in hidden]

    def visible_part_packeds(self):
        return [part["packed"] for part in self._visible_gpu_parts()]

    def look_at_dir(self, pos, center, up, *args):
        self.camera.eye = (float(pos[0]), float(pos[1]), float(pos[2]))
        self.camera.center = (float(center[0]), float(center[1]), float(center[2]))
        self.camera.up = (float(up[0]), float(up[1]), float(up[2]))

    def set_view_center_raw(self, center, *args):
        self.camera.center = (float(center[0]), float(center[1]), float(center[2]))

    def set_view_center_and_look_at(self, center, *args):
        self.set_view_center_raw(center)

    def get_view_center(self):
        return self.camera.center

    def get_view_camera_parameters(self):
        return _CamParams(self.camera)

    def get_view_projection_mode(self):
        return "orthographic" if self.camera.ortho else "perspective"

    def set_view_projection_mode(self, mode):
        self.camera.ortho = "ortho" in str(mode).lower()

    def get_vertical_fov_degrees(self):
        return math.degrees(self.camera.fov)

    def set_vertical_fov_degrees(self, deg):
        self.camera.fov = math.radians(min(170.0, max(1.0, float(deg))))

    def get_length_scale(self):
        tan_half = math.tan(0.5 * self.camera.fov)
        return self.camera.zoom / max(2.0 * tan_half, 1e-8)

    def get_window_size(self):
        return self.window.get_size()

    def get_bounding_box(self):
        solid = self._solid
        if solid is None:
            raise RuntimeError("no solid")
        return solid["packed"]["bounds"]

    def set_transparency_mode(self, mode):
        name = str(mode or "none").lower()
        if "pretty" in name:
            self._transparency = "pretty"
        elif "simple" in name:
            self._transparency = "simple"
        else:
            self._transparency = "none"

    def key_down(self, name):
        key = getattr(self._pyglet.window.key, name, None)
        return key is not None and key in self._held

    def register_surface_mesh(
            self, name, verts, faces, color=(0.7, 0.7, 0.7), transparency=0.0,
            smooth_shade=True, material=None, back_face_policy=None, **kwargs):
        alpha = max(0.0, min(1.0, 1.0 - float(transparency or 0.0)))
        self._overlays[name] = {
            "verts": [tuple(v) for v in verts],
            "faces": [tuple(f[:3]) for f in faces],
            "color": (float(color[0]), float(color[1]), float(color[2])),
            "alpha": alpha,
            "flat": material == "flat" or not smooth_shade,
            "enabled": True,
            "gpu": None,
        }
        return MeshHandle(self, name)

    def remove_surface_mesh(self, name, error_if_absent=False):
        rec = self._overlays.pop(name, None)
        if rec is None and error_if_absent:
            raise KeyError(name)
        if rec is not None:
            self._release_gpu(rec)

    def _update_overlay_verts(self, name, verts):
        rec = self._overlays.get(name)
        if rec is None:
            return
        rec["verts"] = [tuple(v) for v in verts]
        self._release_gpu(rec)

    def _release_gpu(self, rec):
        gpu = rec.get("gpu")
        rec["gpu"] = None
        if not gpu:
            return
        self._delete(gpu.get("vl"))
        self._delete(gpu.get("wire"))

    def _release_solid(self):
        solid = self._solid
        self._solid = None
        if not solid:
            return
        parts = solid.get("parts")
        if not parts:
            parts = [solid]
        for part in parts:
            for key in ("mesh", "line", "line_cap", "point", "wire", "opaque_mesh"):
                self._delete(part.get(key))

    def _upload_overlay(self, rec):
        if rec.get("gpu") is not None:
            return rec["gpu"]
        verts = rec["verts"]
        faces = rec["faces"]
        if not verts or not faces:
            return None
        nvert = len(verts)
        idx = rec["faces"]
        if rec["flat"]:
            vl = self._indexed(self._flat_prog, nvert, idx, in_pos=verts)
            rec["gpu"] = {
                "vl": vl,
                "nvert": nvert,
                "flat": True,
                "wire": self._make_wire(verts, faces),
            }
            return rec["gpu"]
        nrm = _g._face_normals(verts, faces)
        vl = self._indexed(
            self._mesh_prog,
            nvert,
            idx,
            in_pos=verts,
            in_n=nrm,
            in_color=self._colors(nvert, rec["color"]),
            in_uv=[(0.0, 0.0)] * nvert,
        )
        rec["gpu"] = {
            "vl": vl,
            "nvert": nvert,
            "flat": False,
            "wire": self._make_wire(verts, faces),
        }
        return rec["gpu"]

    def _upload_solid(self, packed):
        out = {
            "packed": packed,
            "mesh": None,
            "line": None,
            "line_cap": None,
            "point": None,
            "wire": None,
            "base": None,
            "line_base": None,
            "line_cap_base": None,
            "point_base": None,
        }
        if packed["mesh_idx"]:
            pos = packed["mesh_pos"]
            nrm = packed["mesh_n"]
            uvs = packed.get("mesh_uv")
            if not uvs or len(uvs) != len(pos):
                uvs = [(0.0, 0.0)] * len(pos)
            base = self._np.asarray(packed["mesh_color"], dtype="f4")
            out["mesh"] = self._indexed(
                self._mesh_prog,
                len(pos),
                packed["mesh_idx"],
                in_pos=pos,
                in_n=nrm,
                in_color=base,
                in_uv=uvs,
            )
            out["base"] = base
            out["wire"] = self._make_wire(pos, packed["mesh_idx"])
        if packed["line_idx"]:
            n = len(packed["line_start"])
            base = self._colors(n, _g._LINE_COLOR)
            out["line"] = self._indexed(
                self._line_prog,
                n,
                packed["line_idx"],
                in_start=packed["line_start"],
                in_end=packed["line_end"],
                in_next=packed["line_next"],
                in_color=base,
            )
            out["line_base"] = base
        if packed["line_cap_pos"]:
            n = len(packed["line_cap_pos"])
            base = self._colors(n, _g._LINE_COLOR)
            out["line_cap"] = self._array(
                self._point_prog,
                n,
                in_pos=packed["line_cap_pos"],
                in_corner=packed["line_cap_corner"],
                in_color=base,
            )
            out["line_cap_base"] = base
        if packed["point_pos"]:
            n = len(packed["point_pos"])
            base = self._colors(n, _g._POINT_COLOR)
            out["point"] = self._array(
                self._point_prog,
                n,
                in_pos=packed["point_pos"],
                in_corner=packed["point_corner"],
                in_color=base,
            )
            out["point_base"] = base
        out["opaque_mesh"] = None
        out["selected_line"] = False
        return out

    def _is_sel(self, name, sel):
        if not name or not sel:
            return False
        if name in sel:
            return True
        try:
            from .view import _name_is_selected
            return _name_is_selected(name, sel)
        except Exception:
            return False

    def _write_colors(self, vl, colors):
        if vl is None:
            return
        flat = self._flat(colors).tolist()
        try:
            vl.set_attribute_data("in_color", flat)
            return
        except Exception:
            pass
        try:
            vl.in_color = flat
        except Exception:
            pass

    def highlight_solid(self, selected, mate_refs=None):
        solid = self._solid
        if solid is None:
            return
        sel = set(selected or [])
        refs = list(mate_refs or [])
        from .view import _constraint_ref_color

        def paint(vl, base, names, hl_rgb, alias_lookup=None):
            if vl is None or base is None:
                return False, None
            if not sel and not refs:
                self._write_colors(vl, base)
                return False, None
            colors = base.copy()
            hl = self._np.asarray(hl_rgb, dtype="f4")
            any_hl = False
            for i, name in enumerate(names):
                aliases = None
                if alias_lookup is not None:
                    aliases = alias_lookup.get(name)
                mate = _constraint_ref_color(name, refs, aliases)
                if mate is not None:
                    colors[i] = mate
                    any_hl = True
                elif sel and (
                        self._is_sel(name, sel)
                        or (aliases and any(self._is_sel(alias, sel) for alias in aliases))):
                    colors[i] = hl
                    any_hl = True
            self._write_colors(vl, colors)
            return any_hl, colors

        selected_line = False
        for part in self._gpu_parts():
            packed = part["packed"]
            _, mesh_colors = paint(
                part["mesh"], part["base"], packed["vert_names"], _g._SELECT_SURFACE)
            selected_line = paint(
                part["line"], part["line_base"], packed["line_names"], _g._SELECT_CURVE)[0] or selected_line
            paint(
                part.get("line_cap"), part.get("line_cap_base"),
                packed.get("line_cap_names") or [], _g._SELECT_CURVE)
            paint(
                part["point"], part["point_base"], packed["point_names"], _g._SELECT_POINT,
                packed.get("point_aliases") or {})
            self._rebuild_opaque_highlight(part, refs, mesh_colors)
        solid["selected_line"] = selected_line

    def _rebuild_opaque_highlight(self, part, refs, colors):
        self._delete(part.get("opaque_mesh"))
        part["opaque_mesh"] = None
        packed = part.get("packed") or {}
        if part.get("mesh") is None or colors is None:
            return
        from .view import _mate_triangle_indices
        kept = _mate_triangle_indices(packed.get("mesh_idx") or [], packed.get("vert_names") or [], refs)
        if not kept:
            return
        pos = packed["mesh_pos"]
        nrm = packed["mesh_n"]
        uvs = packed.get("mesh_uv")
        if not uvs or len(uvs) != len(pos):
            uvs = [(0.0, 0.0)] * len(pos)
        part["opaque_mesh"] = self._indexed(
            self._mesh_prog,
            len(pos),
            kept,
            in_pos=pos,
            in_n=nrm,
            in_color=colors,
            in_uv=uvs,
        )

    def set_title(self, text):
        try:
            self.window.set_caption(text)
        except Exception:
            pass

    def _set_camera(self, view, proj, cam):
        light = tuple(float(x) for x in _g._LIGHT_DIR)
        self._mesh_prog.use()
        names = self._mesh_prog.uniforms
        self._mesh_prog["u_view"] = view
        self._mesh_prog["u_proj"] = proj
        if "u_eye" in names:
            self._mesh_prog["u_eye"] = tuple(float(x) for x in cam.eye)
        if "u_light_dir" in names:
            self._mesh_prog["u_light_dir"] = light
        if "u_alpha" in names:
            self._mesh_prog["u_alpha"] = 1.0
        self._flat_prog.use()
        self._flat_prog["u_view"] = view
        self._flat_prog["u_proj"] = proj

    def _set_wire(self, view, proj):
        self._wire_prog.use()
        self._wire_prog["u_view"] = view
        self._wire_prog["u_proj"] = proj
        self._wire_prog["u_color"] = _g._WIRE_COLOR
        ssaa = max(float(getattr(self, "_draw_ssaa", _g._SSAA)), 1.0)
        self._wire_prog["u_width_px"] = _g._WIRE_WIDTH_PX * ssaa

    def _set_impostor(
            self, prog, view, proj, radius, cam, depth_pull_px):
        prog.use()
        names = prog.uniforms
        if "u_view" in names:
            prog["u_view"] = view
        if "u_proj" in names:
            prog["u_proj"] = proj
        rad = float(max(radius, 1e-8))
        if "u_ortho" in names:
            prog["u_ortho"] = 1.0 if cam.ortho else 0.0
        if "u_near" in names:
            prog["u_near"] = float(cam.near)
        if "u_far" in names:
            prog["u_far"] = float(cam.far)
        if "u_view_scale" in names:
            prog["u_view_scale"] = float(cam.view_scale)
        if "u_viewport" in names:
            prog["u_viewport"] = (float(self._draw_w), float(self._draw_h))
        wpp = 2.0 * float(cam.view_scale) / float(max(int(self._draw_h), 1))
        if "u_radius_px" in names:
            prog["u_radius_px"] = rad / max(wpp, 1e-12)
        if "u_aa_px" in names:
            prog["u_aa_px"] = max(float(getattr(self, "_draw_ssaa", _g._SSAA)), 1.0)
        if "u_depth_pull_px" in names:
            ssaa = max(float(getattr(self, "_draw_ssaa", _g._SSAA)), 1.0)
            prog["u_depth_pull_px"] = float(depth_pull_px) * ssaa

    def _begin_impostors(self):
        gl = self._gl
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDepthMask(gl.GL_FALSE)
        gl.glEnable(gl.GL_MULTISAMPLE)
        a2c = getattr(gl, "GL_SAMPLE_ALPHA_TO_COVERAGE", None)
        a2one = getattr(gl, "GL_SAMPLE_ALPHA_TO_ONE", None)
        msaa = int(getattr(self, "_draw_samples", 1) or 1) > 1
        if msaa and a2c is not None:
            gl.glDisable(gl.GL_BLEND)
            gl.glEnable(a2c)
            if a2one is not None:
                gl.glEnable(a2one)
        else:
            if a2c is not None:
                gl.glDisable(a2c)
            if a2one is not None:
                gl.glDisable(a2one)
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    def _end_impostors(self):
        gl = self._gl
        a2c = getattr(gl, "GL_SAMPLE_ALPHA_TO_COVERAGE", None)
        a2one = getattr(gl, "GL_SAMPLE_ALPHA_TO_ONE", None)
        if a2c is not None:
            gl.glDisable(a2c)
        if a2one is not None:
            gl.glDisable(a2one)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_BLEND)

    def _begin_line_union(self, scene):
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, int(scene["line_fbo"]))
        gl.glViewport(0, 0, int(self._draw_w), int(self._draw_h))
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        gl.glClearColor(0.0, 0.0, 0.0, 0.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glDepthMask(gl.GL_FALSE)
        alpha_to_coverage = getattr(gl, "GL_SAMPLE_ALPHA_TO_COVERAGE", None)
        if alpha_to_coverage is not None:
            gl.glDisable(alpha_to_coverage)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendEquation(gl.GL_MAX)
        gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE)

    def _finish_line_union(self, scene):
        gl = self._gl
        if int(scene.get("samples") or 1) > 1:
            gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, int(scene["line_fbo"]))
            gl.glBindFramebuffer(
                gl.GL_DRAW_FRAMEBUFFER, int(scene["line_resolve_fbo"]))
            gl.glBlitFramebuffer(
                0, 0, int(self._draw_w), int(self._draw_h),
                0, 0, int(self._draw_w), int(self._draw_h),
                gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
        self._bind_draw_target()
        gl.glBlendEquation(gl.GL_FUNC_ADD)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_FALSE)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        self._blit_color(scene["line_color"])
        gl.glDisable(gl.GL_BLEND)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glEnable(gl.GL_DEPTH_TEST)

    def _bind_draw_target(self):
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, int(self._draw_fbo))
        gl.glViewport(0, 0, int(self._draw_w), int(self._draw_h))
        if self._draw_fbo:
            gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        else:
            gl.glDrawBuffer(gl.GL_BACK)

    def _bind_window(self):
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        gl.glViewport(0, 0, int(self.camera.width), int(self.camera.height))
        gl.glDrawBuffer(gl.GL_BACK)

    def _make_rb(self, internal, w, h, samples):
        gl = self._gl
        rb = self._gl_gen(gl.glGenRenderbuffers)
        gl.glBindRenderbuffer(gl.GL_RENDERBUFFER, rb)
        samples = int(samples)
        if samples > 1:
            gl.glRenderbufferStorageMultisample(
                gl.GL_RENDERBUFFER, samples, internal, int(w), int(h))
        else:
            gl.glRenderbufferStorage(gl.GL_RENDERBUFFER, internal, int(w), int(h))
        gl.glBindRenderbuffer(gl.GL_RENDERBUFFER, 0)
        return rb

    def _release_scene(self):
        scene = self._scene
        self._scene = None
        if not scene:
            return
        gl = self._gl
        for key in ("fbo", "resolve_fbo", "line_fbo", "line_resolve_fbo"):
            self._gl_del(gl.glDeleteFramebuffers, scene.get(key))
        for key in ("color_rb", "depth_rb", "line_rb"):
            self._gl_del(gl.glDeleteRenderbuffers, scene.get(key))
        for key in ("color", "depth", "line_color"):
            self._gl_del(gl.glDeleteTextures, scene.get(key))

    def _max_rb_size(self):
        gl = self._gl
        buf = (ctypes.c_int * 1)()
        try:
            gl.glGetIntegerv(gl.GL_MAX_RENDERBUFFER_SIZE, buf)
        except Exception:
            return 8192
        return max(int(buf[0]), 1)

    def _max_samples(self):
        gl = self._gl
        buf = (ctypes.c_int * 1)()
        try:
            gl.glGetIntegerv(gl.GL_MAX_SAMPLES, buf)
        except Exception:
            return 1
        return max(int(buf[0]), 1)

    def _try_scene(self, w, h, samples):
        gl = self._gl
        samples = max(int(samples), 1)
        color_rb = depth_rb = fbo = resolve_fbo = color = depth = 0
        line_rb = line_fbo = line_resolve_fbo = line_color = 0
        if samples > 1:
            color_rb = self._make_rb(gl.GL_RGBA8, w, h, samples)
            depth_rb = self._make_rb(gl.GL_DEPTH_COMPONENT24, w, h, samples)
            fbo = self._gl_gen(gl.glGenFramebuffers)
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, fbo)
            gl.glFramebufferRenderbuffer(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0, gl.GL_RENDERBUFFER, color_rb)
            gl.glFramebufferRenderbuffer(gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT, gl.GL_RENDERBUFFER, depth_rb)
        else:
            color = self._make_tex(gl.GL_RGBA8, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, w, h)
            depth = self._make_tex(gl.GL_DEPTH_COMPONENT24, gl.GL_DEPTH_COMPONENT, gl.GL_UNSIGNED_INT, w, h, True)
            fbo = self._gl_gen(gl.glGenFramebuffers)
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, fbo)
            gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0, gl.GL_TEXTURE_2D, color, 0)
            gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT, gl.GL_TEXTURE_2D, depth, 0)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        status = gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        if samples > 1:
            color = self._make_tex(gl.GL_RGBA8, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, w, h)
            resolve_fbo = self._gl_gen(gl.glGenFramebuffers)
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, resolve_fbo)
            gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0, gl.GL_TEXTURE_2D, color, 0)
            gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
            resolve_ok = gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER) == gl.GL_FRAMEBUFFER_COMPLETE
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        else:
            resolve_ok = True
            resolve_fbo = 0
            depth_rb = 0
        if status == gl.GL_FRAMEBUFFER_COMPLETE and resolve_ok:
            line_color = self._make_tex(
                gl.GL_RGBA8, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, w, h)
            line_fbo = self._gl_gen(gl.glGenFramebuffers)
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, line_fbo)
            if samples > 1:
                line_rb = self._make_rb(gl.GL_RGBA8, w, h, samples)
                gl.glFramebufferRenderbuffer(
                    gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0,
                    gl.GL_RENDERBUFFER, line_rb)
                gl.glFramebufferRenderbuffer(
                    gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT,
                    gl.GL_RENDERBUFFER, depth_rb)
            else:
                gl.glFramebufferTexture2D(
                    gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0,
                    gl.GL_TEXTURE_2D, line_color, 0)
                gl.glFramebufferTexture2D(
                    gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT,
                    gl.GL_TEXTURE_2D, depth, 0)
            gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
            line_ok = gl.glCheckFramebufferStatus(
                gl.GL_FRAMEBUFFER) == gl.GL_FRAMEBUFFER_COMPLETE
            if samples > 1 and line_ok:
                line_resolve_fbo = self._gl_gen(gl.glGenFramebuffers)
                gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, line_resolve_fbo)
                gl.glFramebufferTexture2D(
                    gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0,
                    gl.GL_TEXTURE_2D, line_color, 0)
                gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
                line_ok = gl.glCheckFramebufferStatus(
                    gl.GL_FRAMEBUFFER) == gl.GL_FRAMEBUFFER_COMPLETE
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        else:
            line_ok = False
        if status != gl.GL_FRAMEBUFFER_COMPLETE or not resolve_ok or not line_ok:
            self._gl_del(gl.glDeleteFramebuffers, fbo)
            self._gl_del(gl.glDeleteFramebuffers, resolve_fbo)
            self._gl_del(gl.glDeleteFramebuffers, line_fbo)
            self._gl_del(gl.glDeleteFramebuffers, line_resolve_fbo)
            self._gl_del(gl.glDeleteRenderbuffers, color_rb)
            self._gl_del(gl.glDeleteRenderbuffers, depth_rb)
            self._gl_del(gl.glDeleteRenderbuffers, line_rb)
            self._gl_del(gl.glDeleteTextures, color)
            self._gl_del(gl.glDeleteTextures, depth)
            self._gl_del(gl.glDeleteTextures, line_color)
            return None
        scene = {
            "w": w, "h": h, "samples": samples,
            "fbo": fbo, "resolve_fbo": resolve_fbo,
            "color_rb": color_rb, "depth_rb": depth_rb, "color": color,
            "line_rb": line_rb, "line_fbo": line_fbo,
            "line_resolve_fbo": line_resolve_fbo, "line_color": line_color,
        }
        if samples <= 1:
            scene["depth"] = depth
        return scene

    def _ensure_scene(self, w, h):
        samples = min(max(int(getattr(_g, "_MSAA", 4)), 1), self._max_samples())
        scene = self._scene
        if scene is not None and scene["w"] == w and scene["h"] == h and scene.get("samples") == samples:
            return scene
        self._release_scene()
        scene = self._try_scene(w, h, samples)
        if scene is None and samples > 1:
            scene = self._try_scene(w, h, 1)
        self._scene = scene
        return scene

    def _set_sample_shading(self, on):
        gl = self._gl
        cap = getattr(gl, "GL_SAMPLE_SHADING", None)
        if cap is None:
            return
        if on:
            gl.glEnable(gl.GL_MULTISAMPLE)
            gl.glEnable(cap)
            try:
                gl.glMinSampleShading(1.0)
            except Exception:
                pass
        else:
            gl.glDisable(cap)

    def _resolve_ssaa(self, scene, fw, fh):
        gl = self._gl
        w, h = int(scene["w"]), int(scene["h"])
        if int(scene.get("samples") or 1) > 1 and scene.get("resolve_fbo"):
            gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, int(scene["fbo"]))
            gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, int(scene["resolve_fbo"]))
            gl.glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
            color = int(scene["color"])
        else:
            color = int(scene["color"])
        self._bind_window()
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_FALSE)
        gl.glDisable(gl.GL_BLEND)
        self._resolve_prog.use()
        try:
            self._resolve_prog["u_image"] = 0
        except Exception:
            pass
        self._resolve_prog["u_ssaa"] = float(max(getattr(self, "_draw_ssaa", _g._SSAA), 1))
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, color)
        self._resolve_vl.draw(gl.GL_TRIANGLES)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glEnable(gl.GL_DEPTH_TEST)

    def _unbind_tex_units(self, count=3):
        gl = self._gl
        for i in range(int(count)):
            gl.glActiveTexture(gl.GL_TEXTURE0 + i)
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glActiveTexture(gl.GL_TEXTURE0)

    def _set_peel(self, on):
        peel = 1.0 if on else 0.0
        vp = (float(self._draw_w), float(self._draw_h))
        dummy = self._dummy_depth
        opaque = dummy
        prev = dummy
        if on and self._peel is not None:
            opaque = self._peel["opaque_depth"]
            prev = self._peel["prev_depth"]
        gl = self._gl
        gl.glActiveTexture(gl.GL_TEXTURE1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, int(opaque))
        gl.glActiveTexture(gl.GL_TEXTURE2)
        gl.glBindTexture(gl.GL_TEXTURE_2D, int(prev))
        gl.glActiveTexture(gl.GL_TEXTURE0)
        for prog in (self._mesh_prog, self._flat_prog):
            prog.use()
            prog["u_peel"] = peel
            prog["u_viewport"] = vp
            try:
                prog["t_opaque_depth"] = 1
                prog["t_peel_depth"] = 2
            except Exception:
                pass

    def _gl_gen(self, fn):
        ids = (ctypes.c_uint * 1)()
        fn(1, ids)
        return int(ids[0])

    def _gl_del(self, fn, handle):
        if not handle:
            return
        ids = (ctypes.c_uint * 1)(int(handle))
        fn(1, ids)

    def _make_tex(self, internal, fmt, ty, w, h, depth=False):
        gl = self._gl
        tex = self._gl_gen(gl.glGenTextures)
        gl.glBindTexture(gl.GL_TEXTURE_2D, tex)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
        if depth:
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_COMPARE_MODE, gl.GL_NONE)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, internal, int(w), int(h), 0, fmt, ty, None)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        return tex

    def _make_depth_fbo(self, depth_tex):
        gl = self._gl
        fbo = self._gl_gen(gl.glGenFramebuffers)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, fbo)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT, gl.GL_TEXTURE_2D, depth_tex, 0)
        gl.glDrawBuffer(gl.GL_NONE)
        gl.glReadBuffer(gl.GL_NONE)
        ok = gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER) == gl.GL_FRAMEBUFFER_COMPLETE
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        if not ok:
            self._gl_del(gl.glDeleteFramebuffers, fbo)
            return 0
        return fbo

    def _make_color_fbo(self, color_tex):
        gl = self._gl
        fbo = self._gl_gen(gl.glGenFramebuffers)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, fbo)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0, gl.GL_TEXTURE_2D, color_tex, 0)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        ok = gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER) == gl.GL_FRAMEBUFFER_COMPLETE
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        if not ok:
            self._gl_del(gl.glDeleteFramebuffers, fbo)
            return 0
        return fbo

    def _release_peel(self):
        peel = self._peel
        self._peel = None
        if not peel:
            return
        gl = self._gl
        for key in ("fbo", "opaque_fbo", "prev_fbo", "accum_fbo"):
            self._gl_del(gl.glDeleteFramebuffers, peel.get(key))
        for key in ("color", "depth", "opaque_depth", "prev_depth", "accum"):
            self._gl_del(gl.glDeleteTextures, peel.get(key))

    def _ensure_peel(self, w, h):
        peel = self._peel
        if peel is not None and peel["w"] == w and peel["h"] == h:
            return peel
        self._release_peel()
        gl = self._gl
        formats = []
        if getattr(gl, "GL_RGBA16F", None):
            formats.append((gl.GL_RGBA16F, gl.GL_FLOAT))
        formats.append((gl.GL_RGBA8, gl.GL_UNSIGNED_BYTE))
        for color_fmt, color_ty in formats:
            built = self._make_peel(w, h, color_fmt, color_ty)
            if built is not None:
                self._peel = built
                return built
        return None

    def _make_peel(self, w, h, color_fmt, color_ty):
        gl = self._gl
        color = self._make_tex(color_fmt, gl.GL_RGBA, color_ty, w, h)
        depth = self._make_tex(gl.GL_DEPTH_COMPONENT24, gl.GL_DEPTH_COMPONENT, gl.GL_UNSIGNED_INT, w, h, True)
        opaque = self._make_tex(gl.GL_DEPTH_COMPONENT24, gl.GL_DEPTH_COMPONENT, gl.GL_UNSIGNED_INT, w, h, True)
        prev = self._make_tex(gl.GL_DEPTH_COMPONENT24, gl.GL_DEPTH_COMPONENT, gl.GL_UNSIGNED_INT, w, h, True)
        fbo = self._gl_gen(gl.glGenFramebuffers)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, fbo)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT0, gl.GL_TEXTURE_2D, color, 0)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_DEPTH_ATTACHMENT, gl.GL_TEXTURE_2D, depth, 0)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        status = gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        opaque_fbo = self._make_depth_fbo(opaque)
        prev_fbo = self._make_depth_fbo(prev)
        accum = self._make_tex(color_fmt, gl.GL_RGBA, color_ty, w, h)
        accum_fbo = self._make_color_fbo(accum)
        if status != gl.GL_FRAMEBUFFER_COMPLETE or not opaque_fbo or not prev_fbo or not accum_fbo:
            self._gl_del(gl.glDeleteFramebuffers, fbo)
            self._gl_del(gl.glDeleteFramebuffers, opaque_fbo)
            self._gl_del(gl.glDeleteFramebuffers, prev_fbo)
            self._gl_del(gl.glDeleteFramebuffers, accum_fbo)
            self._gl_del(gl.glDeleteTextures, color)
            self._gl_del(gl.glDeleteTextures, depth)
            self._gl_del(gl.glDeleteTextures, opaque)
            self._gl_del(gl.glDeleteTextures, prev)
            self._gl_del(gl.glDeleteTextures, accum)
            return None
        return {
            "w": w, "h": h, "fbo": fbo, "opaque_fbo": opaque_fbo, "prev_fbo": prev_fbo,
            "accum_fbo": accum_fbo, "color": color, "depth": depth,
            "opaque_depth": opaque, "prev_depth": prev, "accum": accum,
        }

    def _blit_depth(self, read_fbo, draw_fbo, w, h):
        gl = self._gl
        self._unbind_tex_units()
        gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, int(read_fbo))
        gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, int(draw_fbo))
        gl.glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, gl.GL_DEPTH_BUFFER_BIT, gl.GL_NEAREST)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

    def _copy_min_depth(self, depth_tex, dest_fbo, w, h):
        """Raise dest depth with this pass (Polyscope DepthMode::Greater)."""
        gl = self._gl
        self._unbind_tex_units()
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, int(dest_fbo))
        gl.glViewport(0, 0, int(w), int(h))
        gl.glDrawBuffer(gl.GL_NONE)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_GREATER)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glDisable(gl.GL_BLEND)
        gl.glColorMask(gl.GL_FALSE, gl.GL_FALSE, gl.GL_FALSE, gl.GL_FALSE)
        self._depth_copy_prog.use()
        try:
            self._depth_copy_prog["u_depth"] = 0
        except Exception:
            pass
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, int(depth_tex))
        self._depth_copy_vl.draw(gl.GL_TRIANGLES)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glColorMask(gl.GL_TRUE, gl.GL_TRUE, gl.GL_TRUE, gl.GL_TRUE)
        gl.glDepthFunc(gl.GL_LESS)

    def _clear_depth_tex(self, fbo, value):
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, int(fbo))
        gl.glDrawBuffer(gl.GL_NONE)
        gl.glClearDepth(float(value))
        gl.glClear(gl.GL_DEPTH_BUFFER_BIT)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

    def _clear_color_fbo(self, fbo):
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, int(fbo))
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        gl.glClearColor(0.0, 0.0, 0.0, 0.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

    def _blit_color(self, tex):
        gl = self._gl
        self._blit_prog.use()
        try:
            self._blit_prog["u_image"] = 0
        except Exception:
            pass
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, int(tex))
        self._blit_vl.draw(gl.GL_TRIANGLES)

    def _draw_overlay(self, rec, alpha=None):
        gpu = self._upload_overlay(rec)
        if gpu is None:
            return
        a = rec["alpha"] if alpha is None else alpha
        if gpu["flat"]:
            c = rec["color"]
            self._flat_prog["u_color"] = (c[0], c[1], c[2], a)
            self._draw(self._flat_prog, gpu["vl"])
            return
        self._mesh_prog["u_alpha"] = float(a)
        self._draw(self._mesh_prog, gpu["vl"])

    def _draw_overlay_list(self, recs, alpha=None):
        for rec in recs:
            self._draw_overlay(rec, alpha=alpha)

    def _draw_transparents(self, recs):
        if not recs:
            return
        gl = self._gl
        mode = self._transparency
        if mode == "none":
            self._set_peel(False)
            self._draw_overlay_list(recs, alpha=1.0)
            return
        if mode == "simple" or self._ensure_peel(self._draw_w, self._draw_h) is None:
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glDepthMask(gl.GL_FALSE)
            try:
                self._set_peel(False)
                self._draw_overlay_list(recs)
            finally:
                gl.glDepthMask(gl.GL_TRUE)
                gl.glDisable(gl.GL_BLEND)
            return
        self._draw_pretty(recs)

    def _draw_pretty(self, recs):
        """Front-to-back depth peel + under-composite (Polyscope Pretty)."""
        gl = self._gl
        peel = self._peel
        w, h = self._draw_w, self._draw_h
        self._unbind_tex_units()
        self._clear_depth_tex(peel["opaque_fbo"], 1.0)
        self._blit_depth(self._draw_fbo, peel["opaque_fbo"], w, h)
        self._clear_depth_tex(peel["prev_fbo"], 0.0)
        self._clear_color_fbo(peel["accum_fbo"])
        gl.glBlendEquation(gl.GL_FUNC_ADD)
        gl.glColorMask(gl.GL_TRUE, gl.GL_TRUE, gl.GL_TRUE, gl.GL_TRUE)
        for _pass in range(_g._PEEL_PASSES):
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, peel["fbo"])
            gl.glViewport(0, 0, w, h)
            gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
            gl.glClearColor(0.0, 0.0, 0.0, 0.0)
            gl.glClearDepth(1.0)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glDepthFunc(gl.GL_LESS)
            gl.glDepthMask(gl.GL_TRUE)
            gl.glDisable(gl.GL_BLEND)
            self._set_peel(True)
            self._draw_overlay_list(recs)
            self._unbind_tex_units()
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, peel["accum_fbo"])
            gl.glViewport(0, 0, w, h)
            gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
            gl.glDisable(gl.GL_DEPTH_TEST)
            gl.glDepthMask(gl.GL_FALSE)
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_ONE_MINUS_DST_ALPHA, gl.GL_ONE)
            self._blit_color(peel["color"])
            self._unbind_tex_units()
            self._copy_min_depth(peel["depth"], peel["prev_fbo"], w, h)
        self._bind_draw_target()
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_FALSE)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE_MINUS_SRC_ALPHA)
        self._blit_color(peel["accum"])
        gl.glDisable(gl.GL_BLEND)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LESS)
        self._set_peel(False)
        self._unbind_tex_units()

    def _draw_scene(self):
        np = self._np
        cam = self.camera
        gl = self._gl
        fw, fh = cam.width, cam.height
        dist = _g._len(_g._sub(cam.eye, cam.center))
        cam._clip(max(cam.view_scale, 1e-3), max(dist, cam.view_scale))
        ssaa = max(int(_g._SSAA), 1)
        maxs = self._max_rb_size()
        while ssaa > 1 and (fw * ssaa > maxs or fh * ssaa > maxs):
            ssaa -= 1
        self._draw_ssaa = ssaa
        sw, sh = fw * ssaa, fh * ssaa
        use_fbo = ssaa > 1 or int(getattr(_g, "_MSAA", 1)) > 1
        scene = self._ensure_scene(sw, sh) if use_fbo else None
        if scene is None:
            self._draw_fbo = 0
            self._draw_w = fw
            self._draw_h = fh
            self._draw_samples = 1
        else:
            self._draw_fbo = scene["fbo"]
            self._draw_w = sw
            self._draw_h = sh
            self._draw_samples = int(scene.get("samples") or 1)
        self._bind_draw_target()
        use_msaa = scene is not None and int(scene.get("samples") or 1) > 1
        if use_msaa:
            gl.glEnable(gl.GL_MULTISAMPLE)
            self._set_sample_shading(True)
        else:
            self._set_sample_shading(False)
        gl.glDisable(gl.GL_SCISSOR_TEST)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glDisable(gl.GL_BLEND)
        gl.glDisable(gl.GL_CULL_FACE)
        gl.glClearColor(_g._BG[0], _g._BG[1], _g._BG[2], 1.0)
        gl.glClearDepth(1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        view = _g._mat4(cam.view_matrix(), np)
        proj = _g._mat4(cam.proj_matrix(), np)
        edge_r, point_r, _ = _g.overlay_radii(cam)
        self._set_camera(view, proj, cam)
        self._set_peel(False)

        parts = self._visible_gpu_parts()
        if not self._solid_transparent:
            for part in parts:
                if part.get("mesh") is not None:
                    self._draw(self._mesh_prog, part["mesh"])

        opaque = []
        translucent = []
        for rec in self._overlays.values():
            if not rec.get("enabled"):
                continue
            if rec["alpha"] < 0.999 and self._transparency != "none":
                translucent.append(rec)
            else:
                opaque.append(rec)
        if self._solid_transparent:
            for part in parts:
                if part.get("mesh") is None:
                    continue
                translucent.append({
                    "alpha": _g._SURFACE_ALPHA,
                    "gpu": {
                        "vl": part["mesh"],
                        "flat": False,
                        "wire": part.get("wire"),
                    },
                })
        self._draw_overlay_list(opaque)
        if self._solid_transparent:
            self._mesh_prog["u_alpha"] = 1.0
            for part in parts:
                self._draw(self._mesh_prog, part.get("opaque_mesh"))
        self._draw_transparents(translucent)

        if self._wireframe:
            self._begin_impostors()
            self._set_wire(view, proj)
            for part in parts:
                self._draw(self._wire_prog, part.get("wire"))
            for rec in self._overlays.values():
                if not rec.get("enabled"):
                    continue
                gpu = rec.get("gpu")
                if gpu is not None:
                    self._draw(self._wire_prog, gpu.get("wire"))
            self._end_impostors()

        if parts:
            draw_line_strip = any(part.get("line") is not None for part in parts)
            if draw_line_strip:
                use_line_union = scene is not None and scene.get("line_fbo")
                if use_line_union:
                    self._begin_line_union(scene)
                else:
                    self._begin_impostors()
                self._set_impostor(
                    self._line_prog, view, proj, edge_r, cam,
                    depth_pull_px=_g._CURVE_DEPTH_PULL_PX)
                for part in parts:
                    self._draw(self._line_prog, part.get("line"))
                self._set_impostor(
                    self._point_prog, view, proj, edge_r, cam,
                    depth_pull_px=_g._CURVE_DEPTH_PULL_PX)
                for part in parts:
                    self._draw(self._point_prog, part.get("line_cap"))
                if use_line_union:
                    self._finish_line_union(scene)
                else:
                    self._end_impostors()
            if any(part.get("point") is not None for part in parts):
                self._begin_impostors()
                self._set_impostor(
                    self._point_prog, view, proj, point_r, cam,
                    depth_pull_px=_g._POINT_DEPTH_PULL_PX)
                for part in parts:
                    self._draw(self._point_prog, part.get("point"))
                self._end_impostors()

        if use_msaa:
            self._set_sample_shading(False)
        if scene is not None:
            self._resolve_ssaa(scene, fw, fh)
        else:
            self._bind_window()


def run_solid(obj, title="Camber"):
    import time

    from .view import (
        tick_cad_camera,
        _left_click_released,
        _want_mouse,
        _CAD,
        _draw_assembly_panel,
        _draw_footnote,
        _draw_copy_toast,
        _enter_sketch_mode,
        _leave_sketch_mode,
        _load_constraints,
        _orient_view_to_frame,
        _selected_plane_context,
    )

    viewer = Viewer(title)
    packed = _g.pack_scene(_g._as_scene(obj))
    viewer.set_solid(packed)
    viewer.camera.fit(*packed["bounds"])
    selected = []
    viewer.selected = selected
    imgui = viewer._imgui
    latch = {"f": False, "p": False, "s": False, "v": False, "esc": False, "m": False}
    toast_until = [0.0]
    sketch_session = [None]
    action_toast = ["", 0.0]
    transparent = [False]
    constraints = _load_constraints(obj)
    is_assembly = callable(getattr(obj, "add_part", None))
    part_names = _g.packed_part_names(packed) if is_assembly else []
    hidden_parts = set()
    mate_sel = [-1]
    mate_refs = [[]]
    menu_open = [True]

    def apply_colors():
        viewer.highlight_solid(selected, mate_refs[0])

    def apply_mate_highlights():
        refs = []
        index = mate_sel[0]
        if 0 <= index < len(constraints):
            item = constraints[index]
            refs = item.get("entities") or item.get("refs") or []
        mate_refs[0] = refs
        apply_colors()

    def apply_visibility():
        viewer.set_hidden_parts(hidden_parts)

    def announce():
        label = _g._selection_python(selected) if selected else ""
        viewer.set_title(title + ("  —  " + label if label else ""))
        if label:
            copy_to_clipboard(label)
            toast_until[0] = time.monotonic() + 3.5
        else:
            toast_until[0] = 0.0

    def pick_at():
        io = imgui.GetIO()
        pos = getattr(io, "MousePos", None)
        if pos is None:
            return None
        mx = float(pos[0] if not hasattr(pos, "x") else pos.x)
        my = float(pos[1] if not hasattr(pos, "y") else pos.y)
        try:
            size = io.DisplaySize
            w = float(size[0] if hasattr(size, "__len__") else size.x)
            h = float(size[1] if hasattr(size, "__len__") else size.y)
        except Exception:
            w, h = viewer.get_window_size()
        if w < 2 or h < 2:
            return None
        ndc_x = 2.0 * mx / float(w) - 1.0
        ndc_y = 1.0 - 2.0 * my / float(h)
        origin, direction = viewer.camera.ray(ndc_x, ndc_y)
        point_r, curve_r = _g.pick_radii(viewer.camera)
        visible = viewer.visible_part_packeds()
        if not visible:
            return None
        return _g.pick_name(
            visible[0], origin, direction, point_r, curve_r, packeds=visible)

    def tick():
        io = imgui.GetIO()
        want = _want_mouse(imgui, io)
        if viewer.key_down("Q"):
            viewer.unshow()
            return

        if sketch_session[0] is not None:
            session = sketch_session[0]
            session["tick"]()
            sketch_selected = list(selected)
            picked_name = session["state"].get("last_picked_name") or ""
            if sketch_selected:
                _draw_footnote(imgui, "Picked: " + _g._selection_python(sketch_selected))
            elif picked_name:
                _draw_footnote(imgui, "Picked: \"" + picked_name + "\"")
            else:
                _draw_footnote(imgui, "Sketch: click a handle or named part anchor")
            if session["state"]["done"] is not None:
                transparent[0] = bool(getattr(viewer, "_solid_transparent", False))
                _leave_sketch_mode(viewer, session, sketch_session, action_toast, transparent)
                apply_colors()
            return

        tick_cad_camera(viewer, imgui, io, want)
        if viewer.key_down("F"):
            if not latch["f"]:
                viewer.camera.fit(*packed["bounds"])
                latch["f"] = True
        else:
            latch["f"] = False
        if viewer.key_down("P"):
            if not latch["p"]:
                viewer.camera.ortho = not viewer.camera.ortho
                latch["p"] = True
        else:
            latch["p"] = False
        if viewer.key_down("ESCAPE"):
            if not latch["esc"]:
                latch["esc"] = True
                if selected:
                    selected[:] = []
                    apply_colors()
                    announce()
        else:
            latch["esc"] = False
        if constraints or part_names:
            if viewer.key_down("M"):
                if not latch["m"]:
                    menu_open[0] = not menu_open[0]
                    latch["m"] = True
            else:
                latch["m"] = False
            mate_changed, vis_changed = _draw_assembly_panel(
                imgui, constraints, mate_sel, menu_open, part_names, hidden_parts)
            if mate_changed:
                apply_mate_highlights()
            if vis_changed:
                apply_visibility()
        if viewer.key_down("V"):
            if not latch["v"]:
                plane = _selected_plane_context(obj, selected)
                if plane is None:
                    action_toast[:] = ["Select one planar surface", time.monotonic() + 1.5]
                else:
                    _orient_view_to_frame(viewer, plane["frame"])
                    action_toast[:] = ["View normal to " + plane["name"], time.monotonic() + 1.5]
                latch["v"] = True
        else:
            latch["v"] = False
        if viewer.key_down("S"):
            if not latch["s"]:
                plane = _selected_plane_context(obj, selected)
                if plane is None:
                    action_toast[:] = [
                        "Select one planar face first, then press S",
                        time.monotonic() + 3.5,
                    ]
                elif plane["part"] is None:
                    action_toast[:] = [
                        "Selected object has no editable Part",
                        time.monotonic() + 3.5,
                    ]
                else:
                    _enter_sketch_mode(
                        viewer, imgui, plane, sketch_session, action_toast, selected, apply_colors)
                latch["s"] = True
        else:
            latch["s"] = False
        if sketch_session[0] is not None:
            return
        if not want:
            released = _left_click_released(imgui, io)
            if released is not None:
                ctrl = bool(getattr(io, "KeyCtrl", False))
                name = pick_at()
                if name:
                    if ctrl:
                        if name in selected:
                            selected.remove(name)
                        else:
                            selected.append(name)
                    else:
                        selected[:] = [name]
                    apply_colors()
                    announce()
                elif not ctrl and selected:
                    selected[:] = []
                    apply_colors()
                    announce()
        label = _g._selection_python(selected) if selected else ""
        display_label = _g._selection_display(
            selected, _g.merged_entity_types(viewer.visible_part_packeds())) if selected else ""
        if label:
            footnote = display_label + "    V: normal to planar face    S: sketch on that face"
        else:
            footnote = "Click a planar face, then press S to sketch  (V: view normal)"
        if constraints or part_names:
            footnote += "    M: assembly panel"
        sel_top = _draw_footnote(imgui, footnote)
        if time.monotonic() < toast_until[0]:
            _draw_copy_toast(imgui, sel_top)
        elif time.monotonic() < action_toast[1]:
            _draw_copy_toast(imgui, sel_top, action_toast[0])
        _ = _CAD

    viewer.set_user_callback(tick)
    viewer.show()
