"""Air propeller: lofted NACA 4412 blades + revolved spinner.

Sized like a small Piper with a direct-drive Lycoming (PA-28 / O-320 class):
  D ≈ 74″, cruise ~2400 rpm, V ≈ 51 m/s (~99 kt).

Twist: geometric pitch β(r) = min(atan(V/Ωr) + α, β_max).
Uses only the public camber API — same math as GeoScriptViewer/TestScriptPropeller.py.
"""
import math

from camber import Frame, LoftOptions, Part, vec3

blade_count = 3
rpm = 2400.0
forward_speed_m_s = 51.0
angle_of_attack_deg = 3.0
max_geometric_pitch_deg = 38.0

tip_radius_m = 0.94
hub_radius_m = 0.085
hub_outer_radius_m = 0.140
c_root = 0.20 * tip_radius_m
c_tip = 0.09 * tip_radius_m
spinner_length_m = 1.85 * hub_outer_radius_m
blade_into_ogive = 0.30
mid_span_chord_bump = 0.10
span_stations = 9
airfoil_samples_per_side = 32

omega = rpm * (2.0 * math.pi / 60.0)
alpha_rad = angle_of_attack_deg * (math.pi / 180.0)
max_geometric_pitch_rad = max_geometric_pitch_deg * (math.pi / 180.0)
max_dev = max(5e-6, tip_radius_m * 2e-5)


def chord_at_span_fraction(xi):
    chord = c_tip + (c_root - c_tip) * math.pow(1.0 - xi, 0.82)
    chord *= 1.0 + mid_span_chord_bump * 4.0 * xi * (1.0 - xi)
    return chord


def geometric_pitch_rad(radius_m):
    phi = math.atan2(forward_speed_m_s, max(omega * radius_m, 1e-8))
    return min(phi + alpha_rad, max_geometric_pitch_rad)


def propeller_section_frame(radius_m, blade_azimuth_rad, pitch_rad, chord_m):
    ca = math.cos(blade_azimuth_rad)
    sa = math.sin(blade_azimuth_rad)
    r_hat = vec3(ca, sa, 0.0)
    t_hat = vec3(-sa, ca, 0.0)
    forward = vec3(0.0, 0.0, 1.0)
    chord_dir = (-(math.cos(pitch_rad) * t_hat + math.sin(pitch_rad) * forward)).normalized()
    thick_dir = chord_dir.cross(r_hat).normalized()
    section_normal = chord_dir.cross(thick_dir).normalized()
    origin = radius_m * r_hat - chord_dir * (0.5 * chord_m)
    return Frame(origin, x=chord_dir, y=thick_dir, z=section_normal)


def propeller_revolve_meridian_frame(origin):
    return Frame(origin, x=(0, 0, 1), y=(1, 0, 0), z=(0, 1, 0))


def add_spinner_meridian(sk, radius_m, spinner_len_m, ogive_samples=12):
    sk.add_line((0, 0), (0, radius_m))
    through = []
    for i in range(1, ogive_samples + 1):
        th = (math.pi * 0.5) * i / float(ogive_samples)
        through.append((spinner_len_m * math.sin(th), radius_m * math.cos(th)))
    sk.append_spline(through, start_tangent=(1, 0), end_tangent=(0, -1))
    sk.append_line((0, 0))


def create_propeller_spinner(part, max_deviation):
    z_aft = -blade_into_ogive * spinner_length_m
    sk = part.sketch(frame=propeller_revolve_meridian_frame(vec3(0, 0, z_aft)), name="PropSpinProf")
    add_spinner_meridian(sk, hub_outer_radius_m, spinner_length_m)
    return part.revolve(sk, 2.0 * math.pi, name="PropSpinner", max_deviation=max_deviation)


ext = tip_radius_m * 1.2
part = Part(vec3(-ext), vec3(ext), tolerance=max_dev)
loft_options = LoftOptions.propeller_blade()

blades = []
for b in range(blade_count):
    blade_azimuth = (2.0 * math.pi * b) / blade_count
    sketches = []
    for i in range(span_stations):
        t = i / float(span_stations - 1)
        r = hub_radius_m + t * (tip_radius_m - hub_radius_m)
        xi = (r - hub_radius_m) / (tip_radius_m - hub_radius_m)
        chord = chord_at_span_fraction(xi)
        pitch_rad = geometric_pitch_rad(r)
        cs = propeller_section_frame(r, blade_azimuth, pitch_rad, chord)
        sk = part.sketch(frame=cs, name="blade{0}_r{1}".format(b, i))
        sk.add_naca4("4412", (0.0, 0.0), chord, 0.0, airfoil_samples_per_side)
        sketches.append(sk)
    blades.append(part.loft(sketches, loft_options, name="PropBlade{0}".format(b), max_deviation=max_dev))

propeller = create_propeller_spinner(part, max_dev)
for blade in blades:
    propeller = part.union(propeller, blade, name="Propeller")

diameter_m = 2.0 * tip_radius_m
tip_speed = omega * tip_radius_m
advance_j = forward_speed_m_s / ((rpm / 60.0) * diameter_m)
phi_hub = math.degrees(math.atan2(forward_speed_m_s, omega * hub_radius_m))
beta_hub = math.degrees(geometric_pitch_rad(hub_radius_m))
beta_tip = math.degrees(geometric_pitch_rad(tip_radius_m))
print(
    "Propeller assembly:",
    propeller,
    "| D=",
    round(diameter_m * 1000.0),
    "mm blades=",
    blade_count,
    "rpm=",
    rpm,
    "V=",
    forward_speed_m_s,
    "m/s J=",
    round(advance_j, 3),
    "Vtip=",
    round(tip_speed),
    "m/s | hub inflow φ=",
    round(phi_hub, 1),
    "deg geometric β hub/tip=",
    round(beta_hub, 1),
    "/",
    round(beta_tip, 1),
    "deg",
)
propeller.show(title="camber propeller")
