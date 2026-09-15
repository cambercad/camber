"""Entity addresses used for pick and highlight. Mirrors GeoMeta.EntityNaming."""

# Python / CAD script style: lowercase tokens, same as plane names ("xy").
ORIGIN = "origin"
AXIS_X = "x"
AXIS_Y = "y"
CENTER = "center"
CONTROL_VERTEX = "cv"
IN_OFFSET = "in_offset"
OUT_OFFSET = "out_offset"
START_CAP = "start_cap"
END_CAP = "end_cap"
CAP = "cap"


def qualify(owner_name, local_name):
    if not local_name:
        return ""
    if not owner_name:
        return local_name
    return owner_name + ":" + local_name


def format_sketch_curve_address(curve_name, uniform):
    return "{0}@{1:.3f}".format(curve_name, float(uniform))


def format_sketch_curve_center(curve_name):
    return curve_name + "@" + CENTER


def format_sketch_curve_control_vertex(curve_name, index):
    return "{0}@{1}{2}".format(curve_name, CONTROL_VERTEX, int(index))


def format_sketch_offset_child(source_name, param, occurrence=0):
    name = source_name + "@" + param
    if occurrence > 0:
        name += "_" + str(int(occurrence) + 1)
    return name


def format_sketch_offset_curve(source_name, outward, occurrence=0):
    param = OUT_OFFSET if outward else IN_OFFSET
    return format_sketch_offset_child(source_name, param, occurrence)


def format_sketch_offset_end_cap(source_name, at_start, occurrence=0):
    param = START_CAP if at_start else END_CAP
    return format_sketch_offset_child(source_name, param, occurrence)


def format_sketch_offset_join_cap(curve_a, curve_b, occurrence=0):
    a = curve_a or ""
    b = curve_b or ""
    if a > b:
        a, b = b, a
    name = CAP + format_group_edge_name(a, b)
    if occurrence > 0:
        name += "_" + str(int(occurrence) + 1)
    return name


def format_sketch_handle_address(curve_name, kind, role):
    if not curve_name:
        return ""
    key = (kind or "").strip().lower()
    if key == "circle":
        if role == 2:
            return format_sketch_curve_center(curve_name)
        if role == 3:
            return format_sketch_curve_address(curve_name, 0.0)
        if role == 4:
            return format_sketch_curve_address(curve_name, 0.25)
        if role == 5:
            return format_sketch_curve_address(curve_name, 0.5)
        if role == 6:
            return format_sketch_curve_address(curve_name, 0.75)
        return ""
    if key == "arc":
        if role == 0:
            return format_sketch_curve_address(curve_name, 0.0)
        if role == 1:
            return format_sketch_curve_address(curve_name, 1.0)
        if role == 2:
            return format_sketch_curve_center(curve_name)
        if role == 3:
            return format_sketch_curve_address(curve_name, 0.5)
        return ""
    if role == 0:
        return format_sketch_curve_address(curve_name, 0.0)
    if role == 1:
        return format_sketch_curve_address(curve_name, 1.0)
    if role == 3:
        return format_sketch_curve_address(curve_name, 0.5)
    return ""


def format_edge_point_address(patch_a, patch_b, uniform, edge_index=0):
    name = "[{0},{1}]".format(patch_a, patch_b)
    if edge_index > 0:
        name += "_{0}".format(int(edge_index))
    return "{0}@{1:.3f}".format(name, float(uniform))


def format_group_edge_name(patch_a, patch_b, index=0):
    name = "[{0},{1}]".format(patch_a, patch_b)
    if index > 0:
        return name + "_{0}".format(int(index))
    return name


def format_patch_component_name(base_name, index=0):
    """Face-component name: index 0 keeps origin; index > 0 appends _{i}."""
    if int(index) <= 0:
        return base_name
    return "{0}_{1}".format(base_name, int(index))


def parse_qualified(name):
    if not name:
        return "", ""
    colon = name.find(":")
    if colon < 0:
        return "", name
    return name[:colon], name[colon + 1:]


def is_origin_name(name):
    if not name:
        return False
    local = parse_qualified(name)[1] or name
    return local.lower() == ORIGIN


def sketch_axis_index(name):
    """0 for sketch X-axis, 1 for Y-axis, or None."""
    if not name:
        return None
    local = (parse_qualified(name)[1] or name).strip().lower()
    if local in (AXIS_X, "xaxis", "x_axis", "originunitx"):
        return 0
    if local in (AXIS_Y, "yaxis", "y_axis", "originunity"):
        return 1
    return None


def is_center_param(param):
    return bool(param) and param.strip().lower() == CENTER


def canonicalize_point_name(name):
    """Normalize Origin/origin and @Center/@center to the lowercase script form."""
    if not name:
        return name
    stripped = name.strip()
    owner, local = parse_qualified(stripped)
    key = local or stripped
    if key.lower() == ORIGIN:
        return qualify(owner, ORIGIN) if owner else ORIGIN
    if "@" in key:
        curve, param = key.rsplit("@", 1)
        if param.lower() == CENTER:
            local_name = format_sketch_curve_center(curve)
            return qualify(owner, local_name) if owner else local_name
    return stripped


def parse_sketch_curve_address(name):
    if not name or "@" not in name:
        return None
    curve, param = name.rsplit("@", 1)
    if not curve or not param:
        return None
    if param.lower() == CENTER:
        return {"curve": curve, "center": True, "control_vertex": False, "index": 0, "uniform": 0.0}
    if param.startswith(CONTROL_VERTEX) and param[len(CONTROL_VERTEX):].isdigit():
        return {
            "curve": curve,
            "center": False,
            "control_vertex": True,
            "index": int(param[len(CONTROL_VERTEX):]),
            "uniform": 0.0,
        }
    try:
        uniform = float(param)
    except ValueError:
        return None
    return {"curve": curve, "center": False, "control_vertex": False, "index": 0, "uniform": uniform}


def is_offset_curve_name(local):
    if not local or "@" not in local:
        return False
    param = local.rsplit("@", 1)[1]
    if "_" in param:
        head, tail = param.rsplit("_", 1)
        if tail.isdigit():
            param = head
    if local.startswith(CAP + "["):
        return True
    return param in (IN_OFFSET, OUT_OFFSET, START_CAP, END_CAP, CAP)


def is_point_address(name):
    local = parse_qualified(name)[1]
    if not local:
        return False
    if is_origin_name(local):
        return True
    if is_offset_curve_name(local):
        return False
    return parse_sketch_curve_address(local) is not None


def is_curve_address(name):
    local = parse_qualified(name)[1]
    if not local:
        return False
    if "@" not in local:
        return True
    return is_offset_curve_name(local)


def highlight_names(all_names, selected_names):
    chosen = set(selected_names or ())
    return [name for name in (all_names or ()) if name in chosen]
