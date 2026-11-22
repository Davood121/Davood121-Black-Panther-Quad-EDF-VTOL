"""Black Panther Quad-EDF Drone Frame with 4x 50mm 2-Axis Gimbaled EDF Thrusters."""

import math
import os
import sys
import FreeCAD as App
import Part

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from build_gimbal_edf import (
    add_part,
    build_edf_nozzle,
    build_edf_rotor,
    build_edf_shroud,
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


def build_panther_fuselage():
    """Stealthy faceted Black Panther center fuselage + canopy + nose sensor pod."""
    # Main lower chassis lofted/faceted hull
    pts_mid = [
        App.Vector(145, 0, 0),
        App.Vector(105, 58, 0),
        App.Vector(-85, 65, 0),
        App.Vector(-140, 26, 0),
        App.Vector(-140, -26, 0),
        App.Vector(-85, -65, 0),
        App.Vector(105, -58, 0),
        App.Vector(145, 0, 0),
    ]
    pts_top = [
        App.Vector(120, 0, 34),
        App.Vector(85, 42, 34),
        App.Vector(-70, 46, 34),
        App.Vector(-120, 16, 34),
        App.Vector(-120, -16, 34),
        App.Vector(-70, -46, 34),
        App.Vector(85, -42, 34),
        App.Vector(120, 0, 34),
    ]
    pts_bot = [
        App.Vector(125, 0, -24),
        App.Vector(85, 44, -24),
        App.Vector(-70, 48, -24),
        App.Vector(-120, 18, -24),
        App.Vector(-120, -18, -24),
        App.Vector(-70, -48, -24),
        App.Vector(85, -44, -24),
        App.Vector(125, 0, -24),
    ]
    w_bot = Part.makePolygon(pts_bot)
    w_mid = Part.makePolygon(pts_mid)
    w_top = Part.makePolygon(pts_top)
    hull = Part.makeLoft([w_bot, w_mid, w_top], True, False)

    # Solid top and bottom cover plates so the fuselage is completely closed
    top_cover = Part.Face(w_top).extrude(App.Vector(0, 0, -3.0))
    bot_cover = Part.Face(w_bot).extrude(App.Vector(0, 0, 3.0))

    # Dorsal stealth ridge / battery canopy
    canopy = Part.makeBox(150.0, 56.0, 18.0)
    canopy.translate(App.Vector(-75.0, -28.0, 32.0))
    # Nose camera/sensor dome
    nose = Part.makeSphere(16.0, App.Vector(136.0, 0, 4.0))
    return hull.fuse(top_cover).fuse(bot_cover).fuse(canopy).fuse(nose)


def build_panther_accent_trim():
    """Orange stealth accent stripes on top canopy."""
    stripe_l = Part.makeBox(110.0, 4.0, 3.0)
    stripe_l.translate(App.Vector(-55.0, 20.0, 49.0))
    stripe_r = Part.makeBox(110.0, 4.0, 3.0)
    stripe_r.translate(App.Vector(-55.0, -24.0, 49.0))
    return stripe_l.fuse(stripe_r)


def build_swept_arm(target_pos):
    """Build a carbon-fiber swept arm from fuselage to the side-mounted pod servo bracket."""
    inward = App.Vector(-target_pos.x, -target_pos.y, 0)
    inward.normalize()
    # Servo bracket sits 55 mm inward from the vertical pod centerline
    tip = App.Vector(
        target_pos.x + inward.x * 55.0,
        target_pos.y + inward.y * 55.0,
        target_pos.z,
    )
    sx = 65.0 if target_pos.x > 0 else -65.0
    sy = 42.0 if target_pos.y > 0 else -42.0
    root = App.Vector(sx, sy, -4.0)

    vec = tip.sub(root)
    length = vec.Length
    arm = Part.makeBox(length, 22.0, 12.0)
    arm.translate(App.Vector(0, -11.0, -6.0))

    yaw = math.degrees(math.atan2(vec.y, vec.x))
    horiz = math.hypot(vec.x, vec.y)
    pitch = -math.degrees(math.atan2(vec.z, horiz))
    arm.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), pitch)
    arm.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), yaw)
    arm.translate(root)

    pad = Part.makeCylinder(14.0, 14.0, App.Vector(tip.x, tip.y, tip.z - 7.0), App.Vector(0, 0, 1))
    return arm.fuse(pad)


def build_landing_skids():
    """Twin carbon landing skids with angled struts for servo ground clearance."""
    skids = None
    for side in (-1.0, 1.0):
        y_skid = side * 90.0
        rail = Part.makeCylinder(4.5, 220.0, App.Vector(-110.0, y_skid, -92.0), App.Vector(1, 0, 0))
        strut_f = Part.makeCylinder(4.0, 80.0, App.Vector(65.0, side * 42.0, -20.0), App.Vector(0, side * 0.55, -0.85))
        strut_r = Part.makeCylinder(4.0, 80.0, App.Vector(-65.0, side * 42.0, -20.0), App.Vector(0, side * 0.55, -0.85))
        part = rail.fuse(strut_f).fuse(strut_r)
        skids = part if skids is None else skids.fuse(part)
    return skids


def add_gimbal_pod(doc, tag, offset):
    """Instantiate one vertical-lift (VTOL) 50mm 2-axis gimbaled EDF unit at `offset`."""
    # Rotate -90 deg around Y so +X intake points +Z (up) and -X exhaust points -Z (down),
    # then rotate around Z so the servo bracket faces inward toward the drone arm.
    inward_deg = math.degrees(math.atan2(-offset.y, -offset.x))

    def place(shape):
        s = shape.copy()
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
        s.translate(offset)
        return s

    add_part(doc, f"{tag}_EDF_Shroud", place(build_edf_shroud()), (0.35, 0.37, 0.40))
    add_part(doc, f"{tag}_EDF_Exhaust_Nozzle", place(build_edf_nozzle()), (0.28, 0.25, 0.23))
    add_part(doc, f"{tag}_EDF_Rotor_11Blade", place(build_edf_rotor(11)), (0.25, 0.27, 0.30))

    inner_ring = make_tube(29.5, 34.5, 10.0, x_start=-5.0)
    pin_top = Part.makeCylinder(2.0, 7.5, App.Vector(0, 0, 27.0), App.Vector(0, 0, 1))
    pin_bot = Part.makeCylinder(2.0, 7.5, App.Vector(0, 0, -34.5), App.Vector(0, 0, 1))
    add_part(doc, f"{tag}_Inner_Gimbal_Ring", place(inner_ring), (0.92, 0.42, 0.12))
    add_part(doc, f"{tag}_Inner_Pivot_Pins", place(pin_top.fuse(pin_bot)), (0.75, 0.77, 0.80))

    outer_ring = make_tube(36.5, 42.5, 12.0, x_start=-6.0)
    pin_left = Part.makeCylinder(2.0, 8.5, App.Vector(0, 34.0, 0), App.Vector(0, 1, 0))
    pin_right = Part.makeCylinder(2.0, 8.5, App.Vector(0, -42.5, 0), App.Vector(0, 1, 0))
    add_part(doc, f"{tag}_Outer_Gimbal_Ring", place(outer_ring), (0.12, 0.12, 0.14))
    add_part(doc, f"{tag}_Outer_Pivot_Pins", place(pin_left.fuse(pin_right)), (0.75, 0.77, 0.80))

    main_shaft = Part.makeCylinder(2.5, 18.0, App.Vector(0, 0, -60.0), App.Vector(0, 0, 1))
    upper_gear = make_spur_gear(6.5, 3.5, 14, App.Vector(0, 0, -50.5))
    lower_gear = make_spur_gear(7.5, 3.5, 16, App.Vector(0, 0, -59.5))
    add_part(doc, f"{tag}_Gimbal_Drive_Shaft", place(main_shaft), (0.78, 0.80, 0.82))
    add_part(doc, f"{tag}_Spur_Gear_Upper", place(upper_gear), (0.60, 0.62, 0.64))
    add_part(doc, f"{tag}_Spur_Gear_Lower", place(lower_gear), (0.60, 0.62, 0.64))

    top_plate = Part.makeBox(26.0, 18.0, 2.2)
    top_plate.translate(App.Vector(-8.0, -9.0, -55.0))
    shaft_hole = Part.makeCylinder(3.2, 4.0, App.Vector(0, 0, -56.0), App.Vector(0, 0, 1))
    side_arm = Part.makeBox(2.2, 18.0, 24.0)
    side_arm.translate(App.Vector(15.8, -9.0, -76.8))
    bracket = top_plate.cut(shaft_hole).fuse(side_arm)
    add_part(doc, f"{tag}_CF_Servo_Bracket", place(bracket), (0.15, 0.16, 0.18))

    yaw_servo = build_sg90_servo(App.Vector(-5.2, 0, -63.0), rot_z_deg=0.0)
    add_part(doc, f"{tag}_Servo_Yaw_SG90", place(yaw_servo), (0.10, 0.35, 0.85))

    pitch_servo = build_sg90_servo(App.Vector(0, 48.0, 0), rot_z_deg=90.0)
    pitch_servo.rotate(App.Vector(0, 48.0, 0), App.Vector(1, 0, 0), -90.0)
    add_part(doc, f"{tag}_Servo_Pitch_SG90", place(pitch_servo), (0.10, 0.35, 0.85))


def main():
    doc = App.newDocument("BlackPantherQuadEDF")

    # Black Panther stealth frame
    add_part(doc, "Panther_Fuselage", build_panther_fuselage(), (0.08, 0.08, 0.10))
    add_part(doc, "Panther_Accent_Trim", build_panther_accent_trim(), (0.92, 0.42, 0.12))
    add_part(doc, "Panther_Landing_Skids", build_landing_skids(), (0.12, 0.12, 0.14))

    for tag, pos in POD_POSITIONS.items():
        add_part(doc, f"Panther_Arm_{tag}", build_swept_arm(pos), (0.10, 0.10, 0.12))
        add_gimbal_pod(doc, tag, pos)

    doc.recompute()

    fcstd_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd")
    step_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.step")
    doc.saveAs(fcstd_path)
    Part.export(doc.Objects, step_path)
    print(f"Saved: {fcstd_path}")
    print(f"Exported: {step_path}")


if __name__ in ("__main__", "build_black_panther_quad"):
    main()
