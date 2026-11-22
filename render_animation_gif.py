"""Render a looping 960x540 animated GIF of the 4-pod VTOL gimbal & rotor animation."""

import math
import os
import sys
import FreeCAD as App
from PIL import Image
import vtk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from render_full_model_pic import get_part_color, shape_to_vtk_actor

POD_CENTERS = {
    "FL_": App.Vector(195.0, 195.0, 15.0),
    "FR_": App.Vector(195.0, -195.0, 15.0),
    "RL_": App.Vector(-195.0, 195.0, 15.0),
    "RR_": App.Vector(-195.0, -195.0, 15.0),
}

OUTER_PARTS = {
    "Outer_Gimbal_Ring",
    "Outer_Pivot_Pins",
    "Gimbal_Drive_Shaft",
    "Spur_Gear_Upper",
    "Spur_Gear_Lower",
    "Servo_Pitch_SG90",
}

INNER_PARTS = {
    "Inner_Gimbal_Ring",
    "Inner_Pivot_Pins",
    "EDF_Shroud",
    "EDF_Exhaust_Nozzle",
}


def _vtk_transform(base_pos, base_rot, c, r_delta):
    new_pos = c.add(r_delta.multVec(base_pos.sub(c)))
    new_rot = r_delta.multiply(base_rot)
    pl = App.Placement(new_pos, new_rot)
    m = pl.toMatrix()
    vm = vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4):
            vm.SetElement(row, col, m.A[row * 4 + col])
    tf = vtk.vtkTransform()
    tf.SetMatrix(vm)
    return tf


def main():
    src_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd")
    out_gif = os.path.join(SCRIPT_DIR, "black_panther_animation.gif")

    doc = App.openDocument(src_path)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.10, 0.11, 0.14)
    renderer.SetBackground2(0.18, 0.20, 0.25)
    renderer.GradientBackgroundOn()

    animated_actors = {}
    base_placements = {}

    for obj in doc.Objects:
        if not (hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull()):
            continue
        # Build actor from local unplaced shape so UserTransform sets exact world placement
        local_shape = obj.Shape.copy()
        local_shape.Placement = App.Placement()
        actor = shape_to_vtk_actor(local_shape, get_part_color(obj.Name))

        base_p = obj.Placement.copy()
        m = base_p.toMatrix()
        vm = vtk.vtkMatrix4x4()
        for row in range(4):
            for col in range(4):
                vm.SetElement(row, col, m.A[row * 4 + col])
        tf = vtk.vtkTransform()
        tf.SetMatrix(vm)
        actor.SetUserTransform(tf)

        renderer.AddActor(actor)
        animated_actors[obj.Name] = actor
        base_placements[obj.Name] = base_p

    App.closeDocument(doc.Name)

    for pos, intensity in [((450, 350, 600), 0.95), ((-400, -350, 300), 0.55), ((0, 0, -500), 0.35)]:
        light = vtk.vtkLight()
        light.SetPosition(*pos)
        light.SetIntensity(intensity)
        renderer.AddLight(light)

    cam = renderer.GetActiveCamera()
    cam.SetPosition(460, -420, 310)
    cam.SetFocalPoint(0, 0, 0)
    cam.SetViewUp(0, 0, 1)
    renderer.ResetCamera()
    cam.Zoom(1.18)

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(renderer)
    rw.SetSize(960, 540)

    w2if = vtk.vtkWindowToImageFilter()
    w2if.SetInput(rw)

    frames = []
    num_frames = 36
    for i in range(num_frames):
        t = 2.0 * math.pi * i / num_frames
        rotor_deg = (360.0 * 3.0 * i) / num_frames

        for idx, (prefix, c) in enumerate(POD_CENTERS.items()):
            phase = idx * 0.45
            yaw_deg = 18.0 * math.sin(t + phase)
            pitch_deg = 14.0 * math.sin(t * 2.0 + phase)

            inward = App.Vector(-c.x, -c.y, 0)
            inward.normalize()
            tangent = App.Vector(-inward.y, inward.x, 0)

            r_outer = App.Rotation(inward, yaw_deg)
            r_inner = r_outer.multiply(App.Rotation(tangent, pitch_deg))
            r_rotor = r_inner.multiply(App.Rotation(App.Vector(0, 0, 1), rotor_deg))

            for part_name in OUTER_PARTS:
                full = f"{prefix}{part_name}"
                if full in animated_actors:
                    bp = base_placements[full]
                    animated_actors[full].SetUserTransform(_vtk_transform(bp.Base, bp.Rotation, c, r_outer))

            for part_name in INNER_PARTS:
                full = f"{prefix}{part_name}"
                if full in animated_actors:
                    bp = base_placements[full]
                    animated_actors[full].SetUserTransform(_vtk_transform(bp.Base, bp.Rotation, c, r_inner))

            full_rotor = f"{prefix}EDF_Rotor_11Blade"
            if full_rotor in animated_actors:
                bp = base_placements[full_rotor]
                animated_actors[full_rotor].SetUserTransform(_vtk_transform(bp.Base, bp.Rotation, c, r_rotor))

        rw.Render()
        w2if.Modified()
        w2if.Update()

        vtk_img = w2if.GetOutput()
        w, h, _ = vtk_img.GetDimensions()
        raw = vtk_img.GetPointData().GetScalars()
        from vtk.util.numpy_support import vtk_to_numpy
        arr = vtk_to_numpy(raw).reshape(h, w, 3)[::-1]  # flip Y
        frames.append(Image.fromarray(arr).convert("P", palette=Image.ADAPTIVE, colors=128))

    frames[0].save(
        out_gif,
        save_all=True,
        append_images=frames[1:],
        duration=60,
        loop=0,
        optimize=True,
    )
    print(f"Saved animation GIF: {out_gif}")


if __name__ in ("__main__", "render_animation_gif"):
    main()
