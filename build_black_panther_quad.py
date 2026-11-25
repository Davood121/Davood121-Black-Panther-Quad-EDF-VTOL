"""Detailed Black Panther Quad-EDF VTOL Drone Frame with 4x 50mm 2-Axis Gimbaled EDFs."""

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
    build_edf_nozzle,
    build_edf_rotor,
    build_edf_shroud,
    build_inner_gimbal_ring,
    build_drive_gimbal_ring,
    build_edf_mount_bolts,
    build_outer_gimbal_ring,
    build_sg90_servo,
    make_spur_gear,
)

POD_POSITIONS = {
    "FL": App.Vector(195.0, 195.0, 15.0),
    "FR": App.Vector(195.0, -195.0, 15.0),
    "RL": App.Vector(-195.0, 195.0, 15.0),
    "RR": App.Vector(-195.0, -195.0, 15.0),
}


def build_panther_fuselage():
    """Hollow stealthy Black Panther fuselage with FC/ESC 30.5mm standoffs, LiPo bay, wire tunnels, and nose dome."""
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
    top_cover = Part.Face(w_top).extrude(App.Vector(0, 0, -3.0))
    bot_cover = Part.Face(w_bot).extrude(App.Vector(0, 0, 3.0))
    body = hull.fuse(top_cover).fuse(bot_cover)

    # Hollow electronics & 6S LiPo battery bay (3.5mm floor & side walls)
    bay_cut = Part.makeBox(166.0, 68.0, 52.0)
    bay_cut.translate(App.Vector(-88.0, -34.0, -17.0))
    body = body.cut(bay_cut)

    # 30.5x30.5mm Flight Controller & 4-in-1 ESC stack standoffs (M3)
    for sx in (-15.25, 15.25):
        for sy in (-15.25, 15.25):
            post = Part.makeCylinder(3.2, 10.0, App.Vector(sx, sy, -17.5), App.Vector(0, 0, 1))
            bore = Part.makeCylinder(1.5, 11.0, App.Vector(sx, sy, -17.5), App.Vector(0, 0, 1))
            body = body.fuse(post).cut(bore)

    # LiPo battery strap slots & center tray ribs
    for bx in (-48.0, 48.0):
        strap_slot = Part.makeBox(12.0, 46.0, 4.0)
        strap_slot.translate(App.Vector(bx - 6.0, -23.0, -21.5))
        body = body.cut(strap_slot)

    # 4x arm-root wire pass-through ports + canopy M2.5 corner screw bosses
    for ax in (-58.0, 58.0):
        for ay in (-58.0, 58.0):
            port = Part.makeBox(16.0, 16.0, 10.0)
            port.translate(App.Vector(ax - 8.0, ay - 8.0, 10.0))
            body = body.cut(port)
    for cx in (-74.0, 74.0):
        for cy in (-26.0, 26.0):
            boss = Part.makeCylinder(3.5, 10.0, App.Vector(cx, cy, 24.0), App.Vector(0, 0, 1))
            hole = Part.makeCylinder(1.25, 12.0, App.Vector(cx, cy, 23.5), App.Vector(0, 0, 1))
            body = body.fuse(boss).cut(hole)

    # Nose camera/LiDAR gimbal dome + dual lens bezels
    nose = Part.makeSphere(16.0, App.Vector(136.0, 0, 4.0))
    lens_l = Part.makeCylinder(4.2, 6.0, App.Vector(148.0, 6.0, 5.0), App.Vector(1, 0, 0))
    lens_r = Part.makeCylinder(4.2, 6.0, App.Vector(148.0, -6.0, 5.0), App.Vector(1, 0, 0))

    # Rear XT90 battery plug housing
    xt90 = Part.makeBox(16.0, 22.0, 12.0)
    xt90.translate(App.Vector(-136.0, -11.0, -6.0))

    return body.fuse(nose).fuse(lens_l).fuse(lens_r).fuse(xt90)


def build_panther_canopy():
    """Removable tapered stealth canopy lid with cooling louvers and dorsal GPS mast."""
    c_bot = [
        App.Vector(92, 0, 34),
        App.Vector(76, 36, 34),
        App.Vector(-76, 36, 34),
        App.Vector(-94, 0, 34),
        App.Vector(-76, -36, 34),
        App.Vector(76, -36, 34),
        App.Vector(92, 0, 34),
    ]
    c_top = [
        App.Vector(72, 0, 52),
        App.Vector(54, 24, 52),
        App.Vector(-58, 24, 52),
        App.Vector(-74, 0, 52),
        App.Vector(-58, -24, 52),
        App.Vector(54, -24, 52),
        App.Vector(72, 0, 52),
    ]
    canopy = Part.makeLoft([Part.makePolygon(c_bot), Part.makePolygon(c_top)], True, False)
    for vx in (-42.0, -24.0, -6.0, 12.0, 30.0):
        slot = Part.makeBox(7.0, 30.0, 5.0)
        slot.translate(App.Vector(vx, -15.0, 48.0))
        canopy = canopy.cut(slot)
    gps_mast = Part.makeCylinder(3.5, 26.0, App.Vector(-55.0, 0, 51.0), App.Vector(0, 0, 1))
    gps_puck = Part.makeCylinder(18.0, 6.0, App.Vector(-55.0, 0, 77.0), App.Vector(0, 0, 1))
    return canopy.fuse(gps_mast).fuse(gps_puck)


def build_panther_x_chassis():
    """Crosswise X-braced carbon center chassis connecting all 4 arm roots with FC/ESC hub & standoffs."""
    def make_beam(p1, p2, width, height):
        v = p2.sub(p1)
        b = Part.makeBox(v.Length, width, height)
        b.translate(App.Vector(0, -width / 2.0, -height / 2.0))
        b.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), math.degrees(math.atan2(v.y, v.x)))
        b.translate(p1)
        return b

    fl = App.Vector(58.0, 58.0, 15.0)
    fr = App.Vector(58.0, -58.0, 15.0)
    rl = App.Vector(-58.0, 58.0, 15.0)
    rr = App.Vector(-58.0, -58.0, 15.0)

    # Crosswise diagonal X-spars + perimeter box truss for maximum torsional rigidity
    diag1 = make_beam(fl, rr, 20.0, 14.0)
    diag2 = make_beam(fr, rl, 20.0, 14.0)
    rail_f = make_beam(fl, fr, 12.0, 10.0)
    rail_r = make_beam(rl, rr, 12.0, 10.0)
    rail_l = make_beam(fl, rl, 12.0, 10.0)
    rail_rgt = make_beam(fr, rr, 12.0, 10.0)
    chassis = diag1.fuse(diag2).fuse(rail_f).fuse(rail_r).fuse(rail_l).fuse(rail_rgt)

    # Center FC/ESC deck plate + 30.5x30.5mm M3 stack holes + LiPo strap slots
    deck = Part.makeBox(86.0, 86.0, 6.0)
    deck.translate(App.Vector(-43.0, -43.0, 9.0))
    chassis = chassis.fuse(deck)
    for sx in (-15.25, 15.25):
        for sy in (-15.25, 15.25):
            chassis = chassis.cut(Part.makeCylinder(1.6, 16.0, App.Vector(sx, sy, 7.0), App.Vector(0, 0, 1)))
    for bx in (-32.0, 32.0):
        slot = Part.makeBox(6.0, 44.0, 10.0)
        slot.translate(App.Vector(bx - 3.0, -22.0, 7.0))
        chassis = chassis.cut(slot)

    # Cut clean 0.25mm-clearance arm-root pockets and add lower M3 arm-support tabs (Z=5.0..7.3mm)
    cuts = None
    for pos in POD_POSITIONS.values():
        sx = 58.0 if pos.x > 0 else -58.0
        sy = 58.0 if pos.y > 0 else -58.0
        inward_deg = math.degrees(math.atan2(-pos.y, -pos.x))
        flange_cut = Part.makeCylinder(15.25, 16.0, App.Vector(sx, sy, 7.0), App.Vector(0, 0, 1))
        neck_cut = Part.makeBox(40.0, 26.5, 16.0)
        neck_cut.translate(App.Vector(-40.0, -13.25, -8.0))
        neck_cut.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
        neck_cut.translate(App.Vector(sx, sy, 15.0))
        c = flange_cut.fuse(neck_cut)
        cuts = c if cuts is None else cuts.fuse(c)
    chassis = chassis.cut(cuts)
    for p in (fl, fr, rl, rr):
        tab = Part.makeCylinder(17.5, 4.0, App.Vector(p.x, p.y, 5.0), App.Vector(0, 0, 1))
        bolt = Part.makeCylinder(1.6, 5.0, App.Vector(p.x, p.y, 4.5), App.Vector(0, 0, 1))
        chassis = chassis.fuse(tab).cut(bolt)
    return chassis


def build_panther_battery_bay():
    """Bottom chamber: 6S LiPo battery bay bolted under X-chassis and linked to landing skids."""
    pts_bot = [
        App.Vector(84, 0, -32),
        App.Vector(66, 48, -32),
        App.Vector(-66, 48, -32),
        App.Vector(-84, 0, -32),
        App.Vector(-66, -48, -32),
        App.Vector(66, -48, -32),
        App.Vector(84, 0, -32),
    ]
    pts_top = [
        App.Vector(90, 0, 7.8),
        App.Vector(68, 54, 7.8),
        App.Vector(-68, 54, 7.8),
        App.Vector(-90, 0, 7.8),
        App.Vector(-68, -54, 7.8),
        App.Vector(68, -54, 7.8),
        App.Vector(90, 0, 7.8),
    ]
    bay = Part.makeLoft([Part.makePolygon(pts_bot), Part.makePolygon(pts_top)], True, False)
    bay = bay.fuse(Part.Face(Part.makePolygon(pts_bot)).extrude(App.Vector(0, 0, 3.5)))
    # Hollow battery tunnel (144 x 48 x 35mm) + rear XT90 cutout + side strap/cooling slots
    tunnel = Part.makeBox(144.0, 48.0, 35.0)
    tunnel.translate(App.Vector(-72.0, -24.0, -27.5))
    bay = bay.cut(tunnel)
    for bx in (-36.0, 36.0):
        vent = Part.makeBox(10.0, 120.0, 12.0)
        vent.translate(App.Vector(bx - 5.0, -60.0, -16.0))
        bay = bay.cut(vent)
    # 4x angled skid strut sockets coaxial with landing struts (Ø8.4mm bore for Ø8.0mm struts)
    for sx in (-56.0, 56.0):
        for side in (-1.0, 1.0):
            d = App.Vector(0, side * 0.55, -0.85)
            d.normalize()
            p0 = App.Vector(sx, side * 44.0, -20.0)
            sock = Part.makeCylinder(7.0, 18.0, p0.sub(d.multiply(4.0)), d)
            bore = Part.makeCylinder(4.2, 30.0, p0.sub(d.multiply(2.0)), d)
            bay = bay.fuse(sock).cut(bore)
    return bay


def build_panther_electronics_bay():
    """Middle chamber (Z=22.2..36.0mm) above X-chassis with LED channel & M2.5 lid bosses."""
    pts_outer = [
        App.Vector(92, 0, 22.2),
        App.Vector(64, 62, 22.2),
        App.Vector(-64, 62, 22.2),
        App.Vector(-92, 0, 22.2),
        App.Vector(-64, -62, 22.2),
        App.Vector(64, -62, 22.2),
        App.Vector(92, 0, 22.2),
    ]
    pts_inner = [
        App.Vector(82, 0, 22.2),
        App.Vector(57, 54, 22.2),
        App.Vector(-57, 54, 22.2),
        App.Vector(-82, 0, 22.2),
        App.Vector(-57, -54, 22.2),
        App.Vector(57, -54, 22.2),
        App.Vector(82, 0, 22.2),
    ]
    wall = Part.Face(Part.makePolygon(pts_outer)).extrude(App.Vector(0, 0, 13.8)).cut(
        Part.Face(Part.makePolygon(pts_inner)).extrude(App.Vector(0, 0, 14.5))
    )
    # Recessed LED channel along top outer rim (0.2mm clear of Panther_Top_Edge_Lights)
    led_pts = [
        App.Vector(92, 0, 36.2),
        App.Vector(64, 62, 36.2),
        App.Vector(-64, 62, 36.2),
        App.Vector(-92, 0, 36.2),
        App.Vector(-64, -62, 36.2),
        App.Vector(64, -62, 36.2),
    ]
    for i in range(len(led_pts)):
        p1 = led_pts[i]
        p2 = led_pts[(i + 1) % len(led_pts)]
        v = p2.sub(p1)
        ch = Part.makeBox(v.Length + 4.0, 3.0, 2.6)
        ch.translate(App.Vector(-2.0, -1.5, -1.3))
        ch.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), math.degrees(math.atan2(v.y, v.x)))
        ch.translate(p1)
        wall = wall.cut(ch)
    # 4x top rim M2.5 screw bosses (clean stealth front without nose camera face)
    for cx, cy in ((56.0, 52.0), (56.0, -52.0), (-56.0, 52.0), (-56.0, -52.0)):
        boss = Part.makeCylinder(3.5, 13.8, App.Vector(cx, cy, 22.2), App.Vector(0, 0, 1))
        hole = Part.makeCylinder(1.25, 15.0, App.Vector(cx, cy, 21.8), App.Vector(0, 0, 1))
        wall = wall.fuse(boss).cut(hole)
    return wall


def build_panther_top_frame():
    """Top chamber: removable tapered stealth cover seated on middle chamber with cooling louvers & GPS."""
    pts_bot = [
        App.Vector(90, 0, 36.0),
        App.Vector(62, 60, 36.0),
        App.Vector(-62, 60, 36.0),
        App.Vector(-90, 0, 36.0),
        App.Vector(-62, -60, 36.0),
        App.Vector(62, -60, 36.0),
        App.Vector(90, 0, 36.0),
    ]
    pts_top = [
        App.Vector(72, 0, 49.0),
        App.Vector(48, 38, 49.0),
        App.Vector(-48, 38, 49.0),
        App.Vector(-72, 0, 49.0),
        App.Vector(-48, -38, 49.0),
        App.Vector(48, -38, 49.0),
        App.Vector(72, 0, 49.0),
    ]
    top = Part.makeLoft([Part.makePolygon(pts_bot), Part.makePolygon(pts_top)], True, False)
    for vx in (-32.0, -16.0, 0.0, 16.0, 32.0):
        slot = Part.makeBox(7.0, 38.0, 6.0)
        slot.translate(App.Vector(vx - 3.5, -19.0, 45.0))
        top = top.cut(slot)
    gps_mast = Part.makeCylinder(3.0, 20.0, App.Vector(-50.0, 0, 49.0), App.Vector(0, 0, 1))
    gps_puck = Part.makeCylinder(16.0, 5.0, App.Vector(-50.0, 0, 69.0), App.Vector(0, 0, 1))
    return top.fuse(gps_mast).fuse(gps_puck)


def build_top_edge_lights():
    """Continuous LED edge light strips along the perimeter edges of the top frame."""
    pts = [
        App.Vector(92, 0, 36.2),
        App.Vector(64, 62, 36.2),
        App.Vector(-64, 62, 36.2),
        App.Vector(-92, 0, 36.2),
        App.Vector(-64, -62, 36.2),
        App.Vector(64, -62, 36.2),
    ]
    lights = None
    for i in range(len(pts)):
        p1 = pts[i]
        p2 = pts[(i + 1) % len(pts)]
        v = p2.sub(p1)
        seg = Part.makeBox(v.Length, 2.6, 2.2)
        seg.translate(App.Vector(0, -1.3, -1.1))
        seg.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), math.degrees(math.atan2(v.y, v.x)))
        seg.translate(p1)
        lights = seg if lights is None else lights.fuse(seg)
    return lights


def build_swept_arm(target_pos):
    """Detailed carbon-fiber swept arm aligned on exact pod radial axis with flush flat servo hole."""
    inward = App.Vector(-target_pos.x, -target_pos.y, 0)
    inward.normalize()
    inward_deg = math.degrees(math.atan2(inward.y, inward.x))

    tip = App.Vector(
        target_pos.x + inward.x * 58.0,
        target_pos.y + inward.y * 58.0,
        target_pos.z,
    )
    # Place root on the exact 45-deg radial line so arm & servo bay are 100% collinear
    sx = 58.0 if target_pos.x > 0 else -58.0
    sy = 58.0 if target_pos.y > 0 else -58.0
    root = App.Vector(sx, sy, target_pos.z)

    vec = tip.sub(root)
    length = vec.Length
    arm = Part.makeBox(length, 26.0, 14.0)
    arm.translate(App.Vector(0, -13.0, -7.0))

    # Weight-reduction lightning pockets along inner arm span
    for frac in (0.26, 0.48):
        pocket = Part.makeCylinder(5.5, 18.0, App.Vector(length * frac, 0, -9.0), App.Vector(0, 0, 1))
        arm = arm.cut(pocket)

    yaw = math.degrees(math.atan2(vec.y, vec.x))
    horiz = math.hypot(vec.x, vec.y)
    pitch = -math.degrees(math.atan2(vec.z, horiz))
    arm.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), pitch)
    arm.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), yaw)
    arm.translate(root)

    # Compact R=8mm bearing pad at 58mm inward -> 50mm inner edge (5mm clear gap from 45mm outer ring)
    pad = Part.makeCylinder(8.0, 14.0, App.Vector(tip.x, tip.y, tip.z - 7.0), App.Vector(0, 0, 1))
    root_flange = Part.makeCylinder(15.0, 14.0, App.Vector(root.x, root.y, root.z - 7.0), App.Vector(0, 0, 1))

    # Integrated flat servo+gear housing centered symmetrically on the arm axis (tangent = 0)
    tangent = App.Vector(-inward.y, inward.x, 0)
    side = 1.0 if target_pos.y >= 0 else -1.0
    bay_center = App.Vector(
        target_pos.x + inward.x * 65.5,
        target_pos.y + inward.y * 65.5,
        target_pos.z,
    )
    bay_outer = Part.makeBox(44.0, 50.0, 15.0)
    bay_outer.translate(App.Vector(-22.0, -25.0, -7.5))
    bay_outer.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    bay_outer.translate(bay_center)

    servo_bay_center = App.Vector(
        target_pos.x + inward.x * 70.25 + tangent.x * (5.8 * side),
        target_pos.y + inward.y * 70.25 + tangent.y * (5.8 * side),
        target_pos.z,
    )
    servo_hole = Part.makeBox(24.2, 23.8, 22.0)
    servo_hole.translate(App.Vector(-9.95, -11.9, -11.0))
    servo_hole.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    servo_hole.translate(servo_bay_center)

    tab_recess = Part.makeBox(4.0, 33.4, 14.0)
    tab_recess.translate(App.Vector(-6.35, -16.7, -7.0))
    tab_recess.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    tab_recess.translate(servo_bay_center)

    wire_slot = Part.makeBox(8.0, 8.0, 14.0)
    wire_slot.translate(App.Vector(5.0, -15.0 if side > 0 else 7.0, -7.0))
    wire_slot.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    wire_slot.translate(servo_bay_center)

    gear_center = App.Vector(
        target_pos.x + inward.x * 55.0 + tangent.x * (5.5 * side),
        target_pos.y + inward.y * 55.0 + tangent.y * (5.5 * side),
        target_pos.z,
    )
    gear_slot = Part.makeBox(13.8, 26.5, 22.0)
    gear_slot.translate(App.Vector(-6.9, -13.25, -11.0))
    gear_slot.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    gear_slot.translate(gear_center)

    # Direct structural link from arm tip to Outer_Gimbal_Ring (41.5..58mm inward) with shaft bore
    ring_link_center = App.Vector(
        target_pos.x + inward.x * 49.5,
        target_pos.y + inward.y * 49.5,
        target_pos.z,
    )
    ring_link = Part.makeBox(18.0, 26.0, 12.0)
    ring_link.translate(App.Vector(-9.0, -13.0, -6.0))
    ring_link.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    ring_link.translate(ring_link_center)

    shaft_bore = Part.makeCylinder(3.0, 28.0, App.Vector(36.0, 0, 0), App.Vector(1, 0, 0))
    shaft_bore.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    shaft_bore.translate(target_pos)

    # Rear pillow-block bearing support + bottom bridge web so it stays fused to ring_link
    rear_pillow = Part.makeBox(3.3, 10.0, 11.5)
    rear_pillow.translate(App.Vector(57.5, -5.0, -7.5))
    pillow_bridge = Part.makeBox(20.0, 10.0, 3.0)
    pillow_bridge.translate(App.Vector(41.0, -5.0, -9.0))
    rear_pillow = rear_pillow.fuse(pillow_bridge)
    rear_pillow_bore = Part.makeCylinder(2.75, 5.0, App.Vector(56.8, 0, 0), App.Vector(1, 0, 0))
    rear_pillow = rear_pillow.cut(rear_pillow_bore)
    rear_pillow.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    rear_pillow.translate(target_pos)

    # Permanently fuse Outer_Gimbal_Ring into the arm as one rigid structural frame piece
    outer_ring_local = build_outer_gimbal_ring()
    outer_ring_local.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
    outer_ring_local.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), inward_deg)
    outer_ring_local.translate(target_pos)

    return arm.fuse(pad).fuse(root_flange).fuse(bay_outer).fuse(ring_link).fuse(outer_ring_local).cut(servo_hole).cut(tab_recess).cut(wire_slot).cut(gear_slot).cut(shaft_bore).fuse(rear_pillow)


def build_landing_skids():
    """Twin carbon landing skids with rubberized foot caps and angled struts."""
    skids = None
    for side in (-1.0, 1.0):
        y_skid = side * 90.0
        rail = Part.makeCylinder(4.5, 220.0, App.Vector(-110.0, y_skid, -92.0), App.Vector(1, 0, 0))
        cap_f = Part.makeSphere(6.2, App.Vector(110.0, y_skid, -92.0))
        cap_r = Part.makeSphere(6.2, App.Vector(-110.0, y_skid, -92.0))
        strut_f = Part.makeCylinder(4.0, 85.0, App.Vector(56.0, side * 44.0, -20.0), App.Vector(0, side * 0.55, -0.85))
        strut_r = Part.makeCylinder(4.0, 85.0, App.Vector(-56.0, side * 44.0, -20.0), App.Vector(0, side * 0.55, -0.85))
        part = rail.fuse(cap_f).fuse(cap_r).fuse(strut_f).fuse(strut_r)
        skids = part if skids is None else skids.fuse(part)
    return skids


def add_gimbal_pod(doc, tag, offset):
    """Instantiate one detailed vertical-lift (VTOL) 50mm 2-axis gimbaled EDF unit at `offset`."""
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
    add_part(doc, f"{tag}_EDF_Mount_Bolts", place(build_edf_mount_bolts()), (0.82, 0.84, 0.86))

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
    add_part(doc, f"{tag}_Servo_Yaw_SG90", place(yaw_servo), (0.10, 0.35, 0.85))


def main():
    doc = App.newDocument("BlackPantherQuadEDF")

    add_part(doc, "Panther_X_Chassis", build_panther_x_chassis(), (0.10, 0.11, 0.13))
    add_part(doc, "Panther_Battery_Bay", build_panther_battery_bay(), (0.09, 0.10, 0.12))
    add_part(doc, "Panther_Electronics_Bay", build_panther_electronics_bay(), (0.11, 0.12, 0.15))
    add_part(doc, "Panther_Top_Cover", build_panther_top_frame(), (0.13, 0.14, 0.17))
    add_part(doc, "Panther_Top_Edge_Lights", build_top_edge_lights(), (0.15, 0.88, 1.00))
    add_part(doc, "Panther_Landing_Skids", build_landing_skids(), (0.12, 0.12, 0.14))

    for tag, pos in POD_POSITIONS.items():
        add_part(doc, f"Panther_Arm_{tag}", build_swept_arm(pos), (0.10, 0.10, 0.12))
        add_gimbal_pod(doc, tag, pos)

    doc.recompute()

    fcstd_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd")
    step_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.step")
    doc.saveAs(fcstd_path)
    Part.export(doc.Objects, step_path)

    stl_dir = os.path.join(SCRIPT_DIR, "print_stl")
    os.makedirs(stl_dir, exist_ok=True)
    printable = [
        "Panther_Battery_Bay",
        "Panther_X_Chassis",
        "Panther_Electronics_Bay",
        "Panther_Top_Cover",
        "Panther_Top_Edge_Lights",
        "Panther_Landing_Skids",
        "Panther_Arm_FL",
        "Panther_Arm_FR",
        "FL_Inner_Gimbal_Ring",
        "FL_Innermost_Gimbal_Ring",
        "FL_EDF_Shroud",
        "FL_EDF_Exhaust_Nozzle",
        "FL_EDF_Rotor_11Blade",
        "FL_Spur_Gear_Upper",
        "FL_Spur_Gear_Lower",
        "FL_Gimbal_Drive_Shaft",
    ]
    for pname in printable:
        obj = doc.getObject(pname)
        if obj is not None:
            Mesh.export([obj], os.path.join(stl_dir, f"{pname}.stl"))
    print(f"Saved: {fcstd_path}")
    print(f"Exported: {step_path}")
    print(f"Exported {len(printable)} printable STLs to: {stl_dir}")


if __name__ in ("__main__", "build_black_panther_quad"):
    main()
