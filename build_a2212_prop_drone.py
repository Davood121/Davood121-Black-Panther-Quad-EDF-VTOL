"""Black Panther Quad Tilt-Rotor built around A2212/10T 1400KV motors + 8045 (8x4.5) propellers."""

import math
import os
import sys
import FreeCAD as App
import Mesh
import Part

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from build_gimbal_edf import (
    add_part,
    build_cf_servo_bracket,
    build_drive_gimbal_ring,
    build_edf_mount_bolts,
    build_inner_gimbal_ring,
    build_outer_gimbal_ring,
    build_sg90_servo,
    make_spur_gear,
    make_tube,
)
POD_POSITIONS = {
    "FL": App.Vector(195.0, 195.0, 15.0),
    "FR": App.Vector(195.0, -195.0, 15.0),
    "RL": App.Vector(-195.0, 195.0, 15.0),
    "RR": App.Vector(-195.0, -195.0, 15.0),
}


def build_a2212_motor():
    """A2212/10T 1400KV outrunner (27.5mm OD, 30mm raised mount for +/-25 deg prop clearance)."""
    x0 = 30.0
    # Base stator mount flange at x=30..34mm
    base = Part.makeCylinder(13.75, 4.0, App.Vector(x0, 0, 0), App.Vector(1, 0, 0))
    # Main rotating silver bell x=34..54mm
    bell = Part.makeCylinder(13.75, 20.0, App.Vector(x0 + 4.0, 0, 0), App.Vector(1, 0, 0))
    # Tapered top bell cap x=54..58mm
    cap = Part.makeCone(13.75, 10.5, 4.0, App.Vector(x0 + 24.0, 0, 0), App.Vector(1, 0, 0))
    # Boss + 3.17mm shaft + collet prop adapter + bullet spinner
    boss = Part.makeCylinder(5.0, 2.5, App.Vector(x0 + 28.0, 0, 0), App.Vector(1, 0, 0))
    shaft = Part.makeCylinder(1.585, 13.5, App.Vector(x0 + 30.5, 0, 0), App.Vector(1, 0, 0))
    collet = Part.makeCylinder(4.5, 6.0, App.Vector(x0 + 31.5, 0, 0), App.Vector(1, 0, 0))
    spinner = Part.makeCone(5.5, 0.5, 11.0, App.Vector(x0 + 41.5, 0, 0), App.Vector(1, 0, 0))
    motor = base.fuse(bell).fuse(cap).fuse(boss).fuse(shaft).fuse(collet).fuse(spinner)

    # 4x M3 bottom mount holes (16mm x 19mm pattern)
    for dx, dy in ((8.0, 0.0), (-8.0, 0.0), (0.0, 9.5), (0.0, -9.5)):
        hole = Part.makeCylinder(1.5, 4.5, App.Vector(x0 - 0.2, dx, dy), App.Vector(1, 0, 0))
        motor = motor.cut(hole)
    return motor


def build_prop_8045():
    """8045 (8x4.5 inch, 203.2mm) swept-paddle slow-fly propeller matching user's prop."""
    x_hub = 65.5
    hub = Part.makeCylinder(7.2, 6.8, App.Vector(x_hub, 0, 0), App.Vector(1, 0, 0))
    bore = Part.makeCylinder(3.0, 7.6, App.Vector(x_hub - 0.4, 0, 0), App.Vector(1, 0, 0))
    prop = hub.cut(bore)

    # Swept elliptical paddle planform: narrow root neck -> wide mid-blade -> rounded tip
    radii = [5.5, 16.0, 36.0, 62.0, 86.0, 101.6]
    chords = [8.5, 11.5, 22.5, 24.0, 17.0, 7.5]
    thicks = [3.2, 2.8, 2.2, 1.6, 1.1, 0.7]
    pitches = [24.0, 21.0, 16.5, 12.5, 9.5, 7.5]
    sweeps = [0.0, 1.5, 4.0, 5.0, 3.0, -1.0]

    for side in (0.0, 180.0):
        wires = []
        for r, c, t, deg, sw in zip(radii, chords, thicks, pitches, sweeps):
            pts = [
                App.Vector(-t * 0.5, -c * 0.42 + sw, r),
                App.Vector(t * 0.5, -c * 0.42 + sw, r),
                App.Vector(t * 0.5, c * 0.58 + sw, r),
                App.Vector(-t * 0.5, c * 0.58 + sw, r),
                App.Vector(-t * 0.5, -c * 0.42 + sw, r),
            ]
            w = Part.makePolygon(pts)
            w.rotate(App.Vector(0, sw, r), App.Vector(0, 0, 1), deg)
            wires.append(w)
        blade = Part.makeLoft(wires, True, False)
        blade.translate(App.Vector(x_hub + 3.4, 0, 0))
        blade.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), side)
        prop = prop.fuse(blade)
    return prop


def build_a2212_cradle():
    """Spider motor cradle with 30mm raised hollow riser tower for +/-25 deg prop clearance."""
    outer_ring = make_tube(25.2, 27.95, 10.0, x_start=-5.0)
    center_plate = Part.makeCylinder(15.5, 4.0, App.Vector(-4.0, 0, 0), App.Vector(1, 0, 0))
    riser_tube = make_tube(11.5, 14.5, 28.0, x_start=0.0)
    top_plate = Part.makeCylinder(15.5, 4.0, App.Vector(26.0, 0, 0), App.Vector(1, 0, 0))
    cradle = outer_ring.fuse(center_plate).fuse(riser_tube).fuse(top_plate)

    # 4 radial spokes connecting center_plate (R=15) to outer_ring (R=26)
    for ang in (45.0, 135.0, 225.0, 315.0):
        spoke = Part.makeBox(4.0, 5.5, 14.0)
        spoke.translate(App.Vector(-4.0, -2.75, 13.5))
        spoke.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), ang)
        cradle = cradle.fuse(spoke)

        rad = math.radians(ang)
        bore = Part.makeCylinder(1.55, 4.5, App.Vector(0, 24.5 * math.cos(rad), 24.5 * math.sin(rad)), App.Vector(0, math.cos(rad), math.sin(rad)))
        cradle = cradle.cut(bore)

    # Central 8mm E-clip clearance + 4x M3 16x19mm motor mount bores on top_plate (x=26..30mm)
    eclip_bore = Part.makeCylinder(4.0, 6.0, App.Vector(25.0, 0, 0), App.Vector(1, 0, 0))
    cradle = cradle.cut(eclip_bore)
    for dx, dy in ((8.0, 0.0), (-8.0, 0.0), (0.0, 9.5), (0.0, -9.5)):
        m3 = Part.makeCylinder(1.6, 6.0, App.Vector(25.0, dx, dy), App.Vector(1, 0, 0))
        cradle = cradle.cut(m3)
    return cradle


def add_a2212_pod(doc, tag, offset):
    """Instantiate one vertical-lift A2212/10T + 8045 prop 2-axis gimbal pod at `offset`."""
    inward_deg = math.degrees(math.atan2(-offset.y, -offset.x))

    def place(shape):
        s = shape.copy()
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
        s.translate(offset)
        return s

    add_part(doc, f"{tag}_A2212_1400KV", place(build_a2212_motor()), (0.85, 0.55, 0.12))
    add_part(doc, f"{tag}_Prop_8045", place(build_prop_8045()), (0.15, 0.17, 0.20))
    add_part(doc, f"{tag}_A2212_Cradle", place(build_a2212_cradle()), (0.28, 0.30, 0.34))
    add_part(doc, f"{tag}_Mount_Bolts", place(build_edf_mount_bolts()), (0.82, 0.84, 0.86))

    add_part(doc, f"{tag}_Innermost_Gimbal_Ring", place(build_drive_gimbal_ring()), (0.55, 0.68, 0.82))
    add_part(doc, f"{tag}_Inner_Gimbal_Ring", place(build_inner_gimbal_ring()), (0.92, 0.42, 0.12))
    pin_left = Part.makeCylinder(1.5, 8.0, App.Vector(0, 28.0, 0), App.Vector(0, 1, 0))
    pin_right = Part.makeCylinder(1.5, 8.0, App.Vector(0, -36.0, 0), App.Vector(0, 1, 0))
    add_part(doc, f"{tag}_Inner_Pivot_Pins", place(pin_left.fuse(pin_right)), (0.75, 0.77, 0.80))

    pin_top = Part.makeCylinder(1.5, 8.0, App.Vector(0, 0, 38.8), App.Vector(0, 0, 1))
    add_part(doc, f"{tag}_Outer_Pivot_Pins", place(pin_top), (0.75, 0.77, 0.80))

    side = 1.0 if offset.y >= 0 else -1.0
    main_shaft = Part.makeCylinder(2.5, 25.5, App.Vector(0, 0, -60.5), App.Vector(0, 0, 1))
    upper_gear = make_spur_gear(5.2, 3.5, 12, App.Vector(0, 0, -55.0))
    lower_gear = make_spur_gear(5.2, 3.5, 12, App.Vector(0, 11.0 * side, -55.0))
    lower_gear.rotate(App.Vector(0, 11.0 * side, -55.0), App.Vector(0, 0, 1), 15.0)
    add_part(doc, f"{tag}_Gimbal_Drive_Shaft", place(main_shaft), (0.78, 0.80, 0.82))
    add_part(doc, f"{tag}_Spur_Gear_Upper", place(upper_gear), (0.60, 0.62, 0.64))
    add_part(doc, f"{tag}_Spur_Gear_Lower", place(lower_gear), (0.60, 0.62, 0.64))

    yaw_servo = build_sg90_servo(App.Vector(0, 5.8 * side, -61.0), rot_z_deg=90.0 * side)
    add_part(doc, f"{tag}_Servo_Yaw_MG90S", place(yaw_servo), (0.10, 0.35, 0.85))

    # 2nd gimbal axis actuator (tangent-axis pitch servo + pushrod + printable mount bracket)
    pitch_servo = build_sg90_servo(App.Vector(18.0, 26.0 * side, -28.0), rot_z_deg=0.0)
    pushrod = Part.makeCylinder(0.8, 22.0, App.Vector(18.0, 22.0 * side, -16.0), App.Vector(0, 0, 1))
    bracket = build_pitch_servo_bracket(side)
    add_part(doc, f"{tag}_Servo_Pitch_MG90S", place(pitch_servo), (0.12, 0.45, 0.90))
    add_part(doc, f"{tag}_Pitch_Pushrod", place(pushrod), (0.78, 0.80, 0.82))
    add_part(doc, f"{tag}_Pitch_Servo_Mount", place(bracket), (0.22, 0.25, 0.30))


def build_pitch_servo_bracket(side=1.0):
    """Printable clip-on MG90S pitch servo mount bridging Inner_Gimbal_Ring to Servo_Pitch_MG90S."""
    # Saddle clamp hugging Inner_Gimbal_Ring (R_out=37.5mm, x=-5..5mm)
    ring_arc = make_tube(37.4, 40.5, 10.0, x_start=-5.0)
    clip_box = Part.makeBox(12.0, 18.0, 40.0)
    clip_box.translate(App.Vector(-6.0, 20.0 * side - 9.0, -35.0))
    ring_clamp = ring_arc.common(clip_box)

    # Servo cradle arm extending to servo flange at (x=18, y=26*side, z=-28)
    arm = Part.makeBox(26.0, 16.0, 8.0)
    arm.translate(App.Vector(-2.0, 26.0 * side - 8.0, -32.0))
    servo_pocket = Part.makeBox(23.2, 12.6, 12.0)
    servo_pocket.translate(App.Vector(6.4, 26.0 * side - 6.3, -34.0))
    bracket = ring_clamp.fuse(arm).cut(servo_pocket)

    # 2x M2 servo mounting ear holes
    for dx in (4.5, 31.5):
        m2 = Part.makeCylinder(1.05, 10.0, App.Vector(dx, 26.0 * side, -33.0), App.Vector(0, 0, 1))
        bracket = bracket.cut(m2)
    return bracket


def main():
    # 1. Single pod document + STL for OpenFOAM-12
    pod_doc = App.newDocument("GimbalA2212_8045")
    motor = build_a2212_motor()
    cradle = build_a2212_cradle()
    prop = build_prop_8045()
    add_part(pod_doc, "A2212_1400KV", motor, (0.85, 0.55, 0.12))
    add_part(pod_doc, "Prop_8045", prop, (0.15, 0.17, 0.20))
    add_part(pod_doc, "A2212_Cradle", cradle, (0.28, 0.30, 0.34))
    add_part(pod_doc, "Innermost_Gimbal_Ring", build_drive_gimbal_ring(), (0.55, 0.68, 0.82))
    add_part(pod_doc, "Inner_Gimbal_Ring", build_inner_gimbal_ring(), (0.92, 0.42, 0.12))
    add_part(pod_doc, "Outer_Gimbal_Ring", build_outer_gimbal_ring(), (0.12, 0.12, 0.14))
    add_part(pod_doc, "CF_Servo_Bracket", build_cf_servo_bracket(), (0.15, 0.16, 0.18))
    pod_doc.recompute()

    pod_fcstd = os.path.join(SCRIPT_DIR, "gimbal_a2212_8045.FCStd")
    pod_step = os.path.join(SCRIPT_DIR, "gimbal_a2212_8045.step")
    pod_doc.saveAs(pod_fcstd)
    Part.export(pod_doc.Objects, pod_step)
    print("SAVED:", pod_fcstd)

    # Export vertical Z-axis STL (in meters) for OpenFOAM-12
    cfd_dir = os.path.join(SCRIPT_DIR, "cfd_case_a2212", "constant", "triSurface")
    os.makedirs(cfd_dir, exist_ok=True)
    pod_solid = motor.fuse(cradle).fuse(build_drive_gimbal_ring()).fuse(build_inner_gimbal_ring())
    pod_solid.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
    pod_solid.scale(0.001)
    stl_doc = App.newDocument("PodSTL")
    pf = stl_doc.addObject("Part::Feature", "A2212_Pod")
    pf.Shape = pod_solid
    stl_doc.recompute()
    stl_path = os.path.join(cfd_dir, "a2212_pod.stl")
    Mesh.export([pf], stl_path)
    App.closeDocument(stl_doc.Name)
    App.closeDocument(pod_doc.Name)
    print("EXPORTED_STL:", stl_path)

    # 2. Full Black Panther A2212 + 8045 Quad Assembly (reusing frame/arms from black_panther_quad_edf.FCStd)
    src_doc = App.openDocument(os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd"))
    quad_doc = App.newDocument("BlackPantherA2212Quad")
    keep_prefixes = (
        "Panther_X_Chassis",
        "Panther_Battery_Bay",
        "Panther_Electronics_Bay",
        "Panther_Top_Cover",
        "Panther_Top_Edge_Lights",
        "Panther_Landing_Skids",
        "Panther_Arm_",
    )
    for obj in src_doc.Objects:
        if obj.Name.startswith(keep_prefixes):
            color = (0.15, 0.88, 1.00) if "Lights" in obj.Name else (0.11, 0.12, 0.14)
            add_part(quad_doc, obj.Name, obj.Shape.copy(), color)
    App.closeDocument(src_doc.Name)
    for tag, pos in POD_POSITIONS.items():
        add_a2212_pod(quad_doc, tag, pos)
    quad_doc.recompute()

    quad_fcstd = os.path.join(SCRIPT_DIR, "black_panther_a2212_quad.FCStd")
    quad_step = os.path.join(SCRIPT_DIR, "black_panther_a2212_quad.step")
    quad_doc.saveAs(quad_fcstd)
    Part.export(quad_doc.Objects, quad_step)
    print("SAVED_QUAD:", quad_fcstd)
    App.closeDocument(quad_doc.Name)


if __name__ in ("__main__", "build_a2212_prop_drone"):
    main()
