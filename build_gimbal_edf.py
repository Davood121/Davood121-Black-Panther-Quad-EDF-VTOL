"""Parametric 50mm 2-Axis Gimbaled EDF Thruster — Concept CAD for FreeCAD 1.1+."""

import math
import os
import FreeCAD as App
import Part


def make_tube(r_in, r_out, length, x_start=0.0):
    """Create a hollow cylinder along +X starting at x_start."""
    outer = Part.makeCylinder(r_out, length, App.Vector(x_start, 0, 0), App.Vector(1, 0, 0))
    inner = Part.makeCylinder(r_in, length, App.Vector(x_start, 0, 0), App.Vector(1, 0, 0))
    return outer.cut(inner)


def make_cone_tube(r_in1, r_out1, r_in2, r_out2, length, x_start=0.0):
    """Create a tapered hollow cone along +X starting at x_start."""
    outer = Part.makeCone(r_out1, r_out2, length, App.Vector(x_start, 0, 0), App.Vector(1, 0, 0))
    inner = Part.makeCone(r_in1, r_in2, length, App.Vector(x_start, 0, 0), App.Vector(1, 0, 0))
    return outer.cut(inner)


def make_spur_gear(radius, height, teeth, center=App.Vector(0, 0, 0)):
    """Create a simplified spur gear along +Z."""
    base_r = radius - 1.1
    gear = Part.makeCylinder(base_r, height, center, App.Vector(0, 0, 1))
    tooth_w = (2.0 * math.pi * radius / teeth) * 0.45
    tooth_d = 2.0
    for i in range(teeth):
        angle = 2.0 * math.pi * i / teeth
        tx = center.x + (base_r + tooth_d * 0.35) * math.cos(angle)
        ty = center.y + (base_r + tooth_d * 0.35) * math.sin(angle)
        tooth = Part.makeBox(tooth_d, tooth_w, height)
        tooth.translate(App.Vector(-tooth_d / 2.0, -tooth_w / 2.0, 0))
        tooth.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), math.degrees(angle))
        tooth.translate(App.Vector(tx, ty, center.z))
        gear = gear.fuse(tooth)
    return gear


def build_edf_shroud():
    """50mm EDF shroud with flared intake lip (+X)."""
    body = make_tube(25.0, 27.5, 40.0, x_start=-12.0)
    # Flared intake lip at x = +28..+33
    lip_outer = Part.makeCone(27.5, 30.5, 5.0, App.Vector(28.0, 0, 0), App.Vector(1, 0, 0))
    lip_inner = Part.makeCone(25.0, 27.2, 5.0, App.Vector(28.0, 0, 0), App.Vector(1, 0, 0))
    lip = lip_outer.cut(lip_inner)
    # Rounded rim torus at intake
    rim = Part.makeTorus(28.85, 1.65, App.Vector(33.0, 0, 0), App.Vector(1, 0, 0))
    return body.fuse(lip).fuse(rim)


def build_edf_nozzle():
    """Tapered rear exhaust nozzle (-X)."""
    return make_cone_tube(18.5, 21.0, 25.0, 27.5, 28.0, x_start=-40.0)


def build_edf_rotor(num_blades=11):
    """Multi-blade EDF rotor with pointed nose cone spinner."""
    hub = Part.makeCylinder(9.0, 14.0, App.Vector(6.0, 0, 0), App.Vector(1, 0, 0))
    spinner = Part.makeCone(9.0, 0.4, 15.0, App.Vector(20.0, 0, 0), App.Vector(1, 0, 0))
    rotor = hub.fuse(spinner)

    blade_len = 15.6  # r=8.8 to r=24.4
    blade_chord = 9.5
    blade_thick = 1.1
    for i in range(num_blades):
        angle_deg = 360.0 * i / num_blades
        b = Part.makeBox(blade_chord, blade_thick, blade_len)
        b.translate(App.Vector(-blade_chord / 2.0, -blade_thick / 2.0, 8.8))
        # Pitch the blade 32 deg around Z (radial axis)
        b.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 32.0)
        b.translate(App.Vector(14.0, 0, 0))
        b.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), angle_deg)
        rotor = rotor.fuse(b)
    return rotor


def build_sg90_servo(pos=App.Vector(0, 0, 0), rot_z_deg=0.0):
    """Simplified SG90 micro servo body + mounting tabs + output shaft."""
    body = Part.makeBox(22.5, 12.2, 22.5)
    body.translate(App.Vector(-11.25, -6.1, -22.5))
    tabs = Part.makeBox(32.0, 12.2, 2.5)
    tabs.translate(App.Vector(-16.0, -6.1, -6.0))
    boss = Part.makeCylinder(5.8, 4.0, App.Vector(5.2, 0, 0), App.Vector(0, 0, 1))
    shaft = Part.makeCylinder(2.4, 5.0, App.Vector(5.2, 0, 4.0), App.Vector(0, 0, 1))
    servo = body.fuse(tabs).fuse(boss).fuse(shaft)
    if rot_z_deg != 0.0:
        servo.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), rot_z_deg)
    servo.translate(pos)
    return servo


def add_part(doc, name, shape, color):
    """Add a Part::Feature with display color (RGB tuple 0..1)."""
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if hasattr(obj, "ViewObject") and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
    return obj


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    doc = App.newDocument("GimbalEDF50mm")

    # 1. EDF Unit
    add_part(doc, "EDF_Shroud", build_edf_shroud(), (0.35, 0.37, 0.40))
    add_part(doc, "EDF_Exhaust_Nozzle", build_edf_nozzle(), (0.28, 0.25, 0.23))
    add_part(doc, "EDF_Rotor_11Blade", build_edf_rotor(11), (0.25, 0.27, 0.30))

    # 2. Inner Gimbal Ring (Orange) + Z-axis pivot pins
    inner_ring = make_tube(29.5, 34.5, 10.0, x_start=-5.0)
    pin_top = Part.makeCylinder(2.0, 7.5, App.Vector(0, 0, 27.0), App.Vector(0, 0, 1))
    pin_bot = Part.makeCylinder(2.0, 7.5, App.Vector(0, 0, -34.5), App.Vector(0, 0, 1))
    add_part(doc, "Inner_Gimbal_Ring", inner_ring, (0.92, 0.42, 0.12))
    add_part(doc, "Inner_Pivot_Pins", pin_top.fuse(pin_bot), (0.75, 0.77, 0.80))

    # 3. Outer Gimbal Ring (Black) + Y-axis pivot pins
    outer_ring = make_tube(36.5, 42.5, 12.0, x_start=-6.0)
    pin_left = Part.makeCylinder(2.0, 8.5, App.Vector(0, 34.0, 0), App.Vector(0, 1, 0))
    pin_right = Part.makeCylinder(2.0, 8.5, App.Vector(0, -42.5, 0), App.Vector(0, 1, 0))
    add_part(doc, "Outer_Gimbal_Ring", outer_ring, (0.12, 0.12, 0.14))
    add_part(doc, "Outer_Pivot_Pins", pin_left.fuse(pin_right), (0.75, 0.77, 0.80))

    # 4. Bottom Gimbal Shaft & Spur Gears
    main_shaft = Part.makeCylinder(2.5, 18.0, App.Vector(0, 0, -60.0), App.Vector(0, 0, 1))
    upper_gear = make_spur_gear(6.5, 3.5, 14, App.Vector(0, 0, -50.5))
    lower_gear = make_spur_gear(7.5, 3.5, 16, App.Vector(0, 0, -59.5))
    add_part(doc, "Gimbal_Drive_Shaft", main_shaft, (0.78, 0.80, 0.82))
    add_part(doc, "Spur_Gear_Upper", upper_gear, (0.60, 0.62, 0.64))
    add_part(doc, "Spur_Gear_Lower", lower_gear, (0.60, 0.62, 0.64))

    # 5. Carbon-Fiber L-Bracket & Fasteners
    top_plate = Part.makeBox(26.0, 18.0, 2.2)
    top_plate.translate(App.Vector(-8.0, -9.0, -55.0))
    shaft_hole = Part.makeCylinder(3.2, 4.0, App.Vector(0, 0, -56.0), App.Vector(0, 0, 1))
    side_arm = Part.makeBox(2.2, 18.0, 24.0)
    side_arm.translate(App.Vector(15.8, -9.0, -76.8))
    bracket = top_plate.cut(shaft_hole).fuse(side_arm)
    add_part(doc, "CF_Servo_Bracket", bracket, (0.15, 0.16, 0.18))

    screw1 = Part.makeCylinder(1.5, 2.0, App.Vector(14.0, 0, -52.8), App.Vector(0, 0, 1))
    screw2 = Part.makeCylinder(1.5, 2.0, App.Vector(-5.0, 0, -52.8), App.Vector(0, 0, 1))
    screw3 = Part.makeCylinder(1.8, 2.0, App.Vector(18.0, 0, -70.0), App.Vector(1, 0, 0))
    add_part(doc, "Bracket_Screws", screw1.fuse(screw2).fuse(screw3), (0.82, 0.84, 0.86))

    # 6. Servos (2-axis: bottom yaw/roll servo + side pitch servo on outer ring)
    yaw_servo = build_sg90_servo(App.Vector(-5.2, 0, -63.0), rot_z_deg=0.0)
    add_part(doc, "Servo_Yaw_SG90", yaw_servo, (0.10, 0.35, 0.85))

    pitch_servo = build_sg90_servo(App.Vector(0, 48.0, 0), rot_z_deg=90.0)
    pitch_servo.rotate(App.Vector(0, 48.0, 0), App.Vector(1, 0, 0), -90.0)
    add_part(doc, "Servo_Pitch_SG90", pitch_servo, (0.10, 0.35, 0.85))

    doc.recompute()

    fcstd_path = os.path.join(out_dir, "gimbal_edf_50mm.FCStd")
    step_path = os.path.join(out_dir, "gimbal_edf_50mm.step")
    doc.saveAs(fcstd_path)
    Part.export(doc.Objects, step_path)
    print(f"Saved: {fcstd_path}")
    print(f"Exported: {step_path}")


if __name__ in ("__main__", "build_gimbal_edf"):
    main()
