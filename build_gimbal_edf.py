"""Detailed Parametric 50mm 2-Axis Gimbaled EDF Thruster — FreeCAD 1.1+."""

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


def make_spur_gear(radius, height, teeth, center=App.Vector(0, 0, 0), bore_r=2.55):
    """Create a spur gear along +Z with central bore and hub collar."""
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
    hub = Part.makeCylinder(bore_r + 1.6, height + 1.5, center, App.Vector(0, 0, 1))
    bore = Part.makeCylinder(bore_r, height + 2.0, App.Vector(center.x, center.y, center.z - 0.2), App.Vector(0, 0, 1))
    return gear.fuse(hub).cut(bore)


def build_edf_shroud():
    """50mm EDF shroud with smooth bellmouth intake lip, 6 stator vanes, motor can, and pivot bosses."""
    body = make_tube(25.0, 27.5, 40.0, x_start=-12.0)
    # Two-stage smooth bellmouth flare + 2.8mm rolled torus lip to suppress intake separation
    lip1_out = Part.makeCone(27.5, 29.5, 3.5, App.Vector(28.0, 0, 0), App.Vector(1, 0, 0))
    lip1_in = Part.makeCone(25.0, 26.8, 3.5, App.Vector(28.0, 0, 0), App.Vector(1, 0, 0))
    lip1 = lip1_out.cut(lip1_in)
    lip2_out = Part.makeCone(29.5, 32.0, 2.5, App.Vector(31.5, 0, 0), App.Vector(1, 0, 0))
    lip2_in = Part.makeCone(26.8, 29.2, 2.5, App.Vector(31.5, 0, 0), App.Vector(1, 0, 0))
    lip2 = lip2_out.cut(lip2_in)
    rim = Part.makeTorus(29.2, 2.8, App.Vector(34.0, 0, 0), App.Vector(1, 0, 0))

    # Internal motor housing + aerodynamic tail cone + 6 angled stator vanes
    motor_can = Part.makeCylinder(9.2, 19.2, App.Vector(-14.0, 0, 0), App.Vector(1, 0, 0))
    # Tapered aerodynamic tail cone pointing -X (exhaust side): R=9.2 at x=-14 -> R=2.0 at x=-26
    motor_tail = Part.makeCone(2.0, 9.2, 12.0, App.Vector(-26.0, 0, 0), App.Vector(1, 0, 0))
    # Internal 4mm motor shaft bridging motor_can (x=+5.2) to rotor hub (x=+6.0, 0.1mm clear)
    motor_shaft = Part.makeCylinder(2.0, 1.9, App.Vector(4.0, 0, 0), App.Vector(1, 0, 0))
    shroud = body.fuse(lip1).fuse(lip2).fuse(rim).fuse(motor_can).fuse(motor_tail).fuse(motor_shaft)

    for i in range(6):
        vane = Part.makeBox(11.0, 1.3, 16.5)
        vane.translate(App.Vector(-5.5, -0.65, 9.0))
        vane.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), -20.0)
        vane.translate(App.Vector(-5.0, 0, 0))
        vane.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), i * 60.0)
        shroud = shroud.fuse(vane)

    # 4x 45-deg M3 radial mount pads on outer shroud wall (flush up to R=27.95mm)
    for ang in (45.0, 135.0, 225.0, 315.0):
        rad = math.radians(ang)
        pad = Part.makeCylinder(3.0, 1.2, App.Vector(0, 26.55 * math.cos(rad), 26.55 * math.sin(rad)), App.Vector(0, math.cos(rad), math.sin(rad)))
        bore = Part.makeCylinder(1.3, 3.0, App.Vector(0, 25.8 * math.cos(rad), 25.8 * math.sin(rad)), App.Vector(0, math.cos(rad), math.sin(rad)))
        shroud = shroud.fuse(pad).cut(bore)
    return shroud


def build_edf_nozzle():
    """Tapered rear exhaust nozzle (-X) with clean slip-fit collar at x=-12mm."""
    nozzle = make_cone_tube(18.5, 21.0, 25.0, 27.5, 28.0, x_start=-40.0)
    flange = make_tube(27.5, 29.0, 4.0, x_start=-16.0)
    return nozzle.fuse(flange)


def build_edf_rotor(num_blades=11):
    """Multi-blade EDF rotor with pointed nose cone spinner flush inside intake rim."""
    hub = Part.makeCylinder(9.0, 14.0, App.Vector(6.0, 0, 0), App.Vector(1, 0, 0))
    spinner = Part.makeCone(9.0, 0.4, 13.0, App.Vector(20.0, 0, 0), App.Vector(1, 0, 0))
    rotor = hub.fuse(spinner)

    blade_len = 15.6
    blade_chord = 9.5
    blade_thick = 1.1
    for i in range(num_blades):
        angle_deg = 360.0 * i / num_blades
        b = Part.makeBox(blade_chord, blade_thick, blade_len)
        b.translate(App.Vector(-blade_chord / 2.0, -blade_thick / 2.0, 8.8))
        b.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 32.0)
        b.translate(App.Vector(14.0, 0, 0))
        b.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), angle_deg)
        rotor = rotor.fuse(b)
    return rotor


def build_inner_gimbal_ring():
    """Orange middle gimbal ring pivoting on Z (inward) inside Outer_Gimbal_Ring, holding Y-pins for Innermost_Gimbal_Ring."""
    ring = make_tube(33.5, 37.5, 10.0, x_start=-5.0)
    for sign in (1.0, -1.0):
        # Z-axis drive/pivot bosses (0.5mm clearance to Outer_Gimbal_Ring R_in=39.0)
        z_boss = Part.makeCylinder(4.8, 2.0, App.Vector(0, 0, 36.5 if sign > 0 else -38.5), App.Vector(0, 0, 1))
        z_bore = Part.makeCylinder(2.55, 4.5, App.Vector(0, 0, 35.5 if sign > 0 else -39.5), App.Vector(0, 0, 1))
        ring = ring.fuse(z_boss).cut(z_bore)
        # Y-axis bearing bosses (0.5mm clearance to Innermost_Gimbal_Ring boss at R=31.5)
        y_boss = Part.makeCylinder(4.5, 3.0, App.Vector(0, 32.0 if sign > 0 else -35.0, 0), App.Vector(0, 1, 0))
        y_bore = Part.makeCylinder(1.6, 5.0, App.Vector(0, 31.5 if sign > 0 else -36.0, 0), App.Vector(0, 1, 0))
        ring = ring.fuse(y_boss).cut(y_bore)
    return ring


def build_drive_gimbal_ring():
    """Innermost ring pivoting on Y (tangent) inside the orange ring, holding the BLDC EDF shroud."""
    ring = make_tube(28.0, 31.0, 10.0, x_start=-5.0)
    for sign in (1.0, -1.0):
        y_boss = Part.makeCylinder(4.0, 2.0, App.Vector(0, 29.5 if sign > 0 else -31.5, 0), App.Vector(0, 1, 0))
        y_bore = Part.makeCylinder(1.6, 4.0, App.Vector(0, 29.0 if sign > 0 else -32.5, 0), App.Vector(0, 1, 0))
        ring = ring.fuse(y_boss).cut(y_bore)
    for ang in (45.0, 135.0, 225.0, 315.0):
        rad = math.radians(ang)
        hole = Part.makeCylinder(1.6, 4.5, App.Vector(0, 27.5 * math.cos(rad), 27.5 * math.sin(rad)), App.Vector(0, math.cos(rad), math.sin(rad)))
        ring = ring.cut(hole)
    return ring


def build_edf_mount_bolts():
    """Four flush M3x4mm radial bolts locking the BLDC EDF shroud into Innermost_Gimbal_Ring (R<=31.2mm)."""
    bolts = None
    for ang in (45.0, 135.0, 225.0, 315.0):
        rad = math.radians(ang)
        dir_v = App.Vector(0, math.cos(rad), math.sin(rad))
        shank = Part.makeCylinder(1.45, 4.2, App.Vector(0, 26.5 * dir_v.y, 26.5 * dir_v.z), dir_v)
        head = Part.makeCylinder(2.4, 1.0, App.Vector(0, 30.3 * dir_v.y, 30.3 * dir_v.z), dir_v)
        b = shank.fuse(head)
        bolts = b if bolts is None else bolts.fuse(b)
    return bolts


def build_outer_gimbal_ring():
    """Black outer gimbal ring fused to arm, with Z-axis bearing bores for Inner_Gimbal_Ring."""
    ring = make_tube(39.0, 45.0, 12.0, x_start=-6.0)
    for sign in (1.0, -1.0):
        z_boss = Part.makeCylinder(5.5, 4.0, App.Vector(0, 0, 43.0 if sign > 0 else -47.0), App.Vector(0, 0, 1))
        z_bore = Part.makeCylinder(2.6, 8.0, App.Vector(0, 0, 40.0 if sign > 0 else -48.0), App.Vector(0, 0, 1))
        ring = ring.fuse(z_boss).cut(z_bore)
    return ring


def build_cf_servo_bracket():
    """Flush low-profile carbon-fiber servo flange collar (14mm thick to sit inside arm)."""
    collar_plate = Part.makeBox(14.0, 26.0, 2.5)
    collar_plate.translate(App.Vector(-7.0, -13.0, -55.0))
    bearing_collar = Part.makeCylinder(5.5, 4.5, App.Vector(0, 0, -56.0), App.Vector(0, 0, 1))
    bracket = collar_plate.fuse(bearing_collar)

    shaft_hole = Part.makeCylinder(3.0, 8.0, App.Vector(0, 0, -57.5), App.Vector(0, 0, 1))
    bracket = bracket.cut(shaft_hole)
    for hy in (-9.5, 9.5):
        h = Part.makeCylinder(1.6, 5.0, App.Vector(0, hy, -56.5), App.Vector(0, 0, 1))
        bracket = bracket.cut(h)
    return bracket


def build_sg90_servo(pos=App.Vector(0, 0, 0), rot_z_deg=0.0):
    """Detailed MG90S/ES08MA micro servo body + mounting tabs + output spline + wire boot."""
    body = Part.makeBox(22.5, 12.2, 22.5)
    body.translate(App.Vector(-11.25, -6.1, -22.5))
    tabs = Part.makeBox(32.0, 12.2, 2.5)
    tabs.translate(App.Vector(-16.0, -6.1, -6.0))
    boss = Part.makeCylinder(5.8, 4.0, App.Vector(5.2, 0, 0), App.Vector(0, 0, 1))
    shaft = Part.makeCylinder(2.5, 5.8, App.Vector(5.2, 0, 4.0), App.Vector(0, 0, 1))
    wire_boot = Part.makeBox(4.0, 5.0, 4.0)
    wire_boot.translate(App.Vector(-13.5, -2.5, -19.0))
    servo = body.fuse(tabs).fuse(boss).fuse(shaft).fuse(wire_boot)
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

    add_part(doc, "EDF_Shroud", build_edf_shroud(), (0.35, 0.37, 0.40))
    add_part(doc, "EDF_Exhaust_Nozzle", build_edf_nozzle(), (0.28, 0.25, 0.23))
    add_part(doc, "EDF_Rotor_11Blade", build_edf_rotor(11), (0.25, 0.27, 0.30))
    add_part(doc, "EDF_Mount_Bolts", build_edf_mount_bolts(), (0.82, 0.84, 0.86))

    add_part(doc, "Innermost_Gimbal_Ring", build_drive_gimbal_ring(), (0.55, 0.68, 0.82))
    add_part(doc, "Inner_Gimbal_Ring", build_inner_gimbal_ring(), (0.92, 0.42, 0.12))
    pin_left = Part.makeCylinder(1.5, 8.0, App.Vector(0, 28.0, 0), App.Vector(0, 1, 0))
    pin_right = Part.makeCylinder(1.5, 8.0, App.Vector(0, -36.0, 0), App.Vector(0, 1, 0))
    add_part(doc, "Inner_Pivot_Pins", pin_left.fuse(pin_right), (0.75, 0.77, 0.80))

    add_part(doc, "Outer_Gimbal_Ring", build_outer_gimbal_ring(), (0.12, 0.12, 0.14))
    pin_top = Part.makeCylinder(1.5, 8.0, App.Vector(0, 0, 38.8), App.Vector(0, 0, 1))
    add_part(doc, "Outer_Pivot_Pins", pin_top, (0.75, 0.77, 0.80))

    main_shaft = Part.makeCylinder(2.5, 25.5, App.Vector(0, 0, -60.5), App.Vector(0, 0, 1))
    upper_gear = make_spur_gear(5.2, 3.5, 12, App.Vector(0, 0, -55.0))
    lower_gear = make_spur_gear(5.2, 3.5, 12, App.Vector(0, 11.0, -55.0))
    lower_gear.rotate(App.Vector(0, 11.0, -55.0), App.Vector(0, 0, 1), 15.0)
    add_part(doc, "Gimbal_Drive_Shaft", main_shaft, (0.78, 0.80, 0.82))
    add_part(doc, "Spur_Gear_Upper", upper_gear, (0.60, 0.62, 0.64))
    add_part(doc, "Spur_Gear_Lower", lower_gear, (0.60, 0.62, 0.64))

    add_part(doc, "CF_Servo_Bracket", build_cf_servo_bracket(), (0.15, 0.16, 0.18))

    screws = None
    for sy in (-9.5, 9.5):
        sc = Part.makeCylinder(2.4, 2.2, App.Vector(0, sy, -52.5), App.Vector(0, 0, 1))
        screws = sc if screws is None else screws.fuse(sc)
    add_part(doc, "Bracket_Screws", screws, (0.82, 0.84, 0.86))

    yaw_servo = build_sg90_servo(App.Vector(0, 5.8, -61.0), rot_z_deg=90.0)
    add_part(doc, "Servo_Yaw_SG90", yaw_servo, (0.10, 0.35, 0.85))

    doc.recompute()

    fcstd_path = os.path.join(out_dir, "gimbal_edf_50mm.FCStd")
    step_path = os.path.join(out_dir, "gimbal_edf_50mm.step")
    doc.saveAs(fcstd_path)
    Part.export(doc.Objects, step_path)
    print(f"Saved: {fcstd_path}")
    print(f"Exported: {step_path}")


if __name__ in ("__main__", "build_gimbal_edf"):
    main()
