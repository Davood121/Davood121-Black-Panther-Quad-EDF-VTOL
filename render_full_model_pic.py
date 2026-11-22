"""Render a studio-lit 3D CAD image of black_panther_quad_edf.FCStd with all 59 parts colored."""

import os
import FreeCAD as App
import MeshPart
import vtk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

POD_COLORS = {
    "EDF_Shroud": (0.42, 0.44, 0.48),
    "EDF_Exhaust_Nozzle": (0.34, 0.30, 0.28),
    "EDF_Rotor_11Blade": (0.28, 0.30, 0.34),
    "Inner_Gimbal_Ring": (0.95, 0.44, 0.12),
    "Inner_Pivot_Pins": (0.78, 0.80, 0.84),
    "Outer_Gimbal_Ring": (0.14, 0.14, 0.16),
    "Outer_Pivot_Pins": (0.78, 0.80, 0.84),
    "Gimbal_Drive_Shaft": (0.80, 0.82, 0.85),
    "Spur_Gear_Upper": (0.65, 0.67, 0.70),
    "Spur_Gear_Lower": (0.65, 0.67, 0.70),
    "CF_Servo_Bracket": (0.18, 0.19, 0.22),
    "Bracket_Screws": (0.85, 0.87, 0.90),
    "Servo_Yaw_SG90": (0.12, 0.42, 0.92),
    "Servo_Pitch_SG90": (0.12, 0.42, 0.92),
}

FRAME_COLORS = {
    "Panther_Fuselage": (0.12, 0.13, 0.15),
    "Panther_Accent_Trim": (0.95, 0.44, 0.12),
    "Panther_Landing_Skids": (0.16, 0.17, 0.20),
    "Panther_Arm_FL": (0.15, 0.16, 0.19),
    "Panther_Arm_FR": (0.15, 0.16, 0.19),
    "Panther_Arm_RL": (0.15, 0.16, 0.19),
    "Panther_Arm_RR": (0.15, 0.16, 0.19),
}


def get_part_color(name):
    if name in FRAME_COLORS:
        return FRAME_COLORS[name]
    for prefix in ("FL_", "FR_", "RL_", "RR_", ""):
        if name.startswith(prefix):
            base = name[len(prefix):]
            if base in POD_COLORS:
                return POD_COLORS[base]
    return (0.5, 0.5, 0.5)


def shape_to_vtk_actor(shape, color):
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.15, AngularDeflection=0.3)
    pts = vtk.vtkPoints()
    for p in mesh.Points:
        pts.InsertNextPoint(p.x, p.y, p.z)

    polys = vtk.vtkCellArray()
    for f in mesh.Facets:
        polys.InsertNextCell(3)
        for idx in f.PointIndices:
            polys.InsertCellPoint(idx)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(pts)
    polydata.SetPolys(polys)

    normals = vtk.vtkPolyDataNormals()
    normals.SetInputData(polydata)
    normals.ComputePointNormalsOn()
    normals.SplittingOn()
    normals.SetFeatureAngle(35.0)
    normals.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    prop = actor.GetProperty()
    prop.SetColor(*color)
    prop.SetAmbient(0.25)
    prop.SetDiffuse(0.70)
    prop.SetSpecular(0.35)
    prop.SetSpecularPower(28.0)
    return actor


def main():
    src_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd")
    out_png = os.path.join(SCRIPT_DIR, "black_panther_full_parts.png")

    doc = App.openDocument(src_path)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.11, 0.12, 0.15)
    renderer.SetBackground2(0.20, 0.22, 0.27)
    renderer.GradientBackgroundOn()

    for obj in doc.Objects:
        if hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull():
            color = get_part_color(obj.Name)
            renderer.AddActor(shape_to_vtk_actor(obj.Shape, color))

    App.closeDocument(doc.Name)

    # Studio 3-point lighting
    light1 = vtk.vtkLight()
    light1.SetPosition(450, 350, 600)
    light1.SetIntensity(0.95)
    renderer.AddLight(light1)

    light2 = vtk.vtkLight()
    light2.SetPosition(-400, -350, 300)
    light2.SetIntensity(0.55)
    renderer.AddLight(light2)

    light3 = vtk.vtkLight()
    light3.SetPosition(0, 0, -500)
    light3.SetIntensity(0.35)
    renderer.AddLight(light3)

    cam = renderer.GetActiveCamera()
    cam.SetPosition(460, -420, 310)
    cam.SetFocalPoint(0, 0, 0)
    cam.SetViewUp(0, 0, 1)
    renderer.ResetCamera()
    cam.Zoom(1.18)

    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(renderer)
    rw.SetSize(1920, 1080)
    rw.Render()

    w2if = vtk.vtkWindowToImageFilter()
    w2if.SetInput(rw)
    w2if.Update()

    writer = vtk.vtkPNGWriter()
    writer.SetFileName(out_png)
    writer.SetInputConnection(w2if.GetOutputPort())
    writer.Write()
    print(f"Saved full-parts render: {out_png}")


if __name__ in ("__main__", "render_full_model_pic"):
    main()
