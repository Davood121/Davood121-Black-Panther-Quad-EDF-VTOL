"""Pack all 3D-printable Black Panther A2212 + 8045 parts flat onto Z=0 in a single combined STL."""

import os
import FreeCAD as App
import Mesh
import Part

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, "print_stl")


def pack_parts_to_stl(shapes_with_names, out_stl):
    """Lay each shape flat on Z=0 and pack into a clean non-overlapping XY grid."""
    placed = []
    x_cursor = 0.0
    y_cursor = 0.0
    row_depth = 0.0
    max_row_w = 520.0
    gap = 8.0

    for name, sh in shapes_with_names:
        s = sh.copy()
        bb = s.BoundBox
        w = bb.XLength
        d = bb.YLength
        if x_cursor > 0.0 and x_cursor + w > max_row_w:
            x_cursor = 0.0
            y_cursor += row_depth + gap
            row_depth = 0.0
        s.translate(App.Vector(x_cursor - bb.XMin, y_cursor - bb.YMin, -bb.ZMin))
        placed.append(s)
        x_cursor += w + gap
        row_depth = max(row_depth, d)

    comp = Part.makeCompound(placed)
    doc = App.newDocument("PrintPlate")
    obj = doc.addObject("Part::Feature", "AllPrintableParts")
    obj.Shape = comp
    doc.recompute()
    Mesh.export([obj], out_stl)
    bb_all = comp.BoundBox
    App.closeDocument(doc.Name)
    return len(placed), (bb_all.XLength, bb_all.YLength, bb_all.ZLength)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    src = App.openDocument(os.path.join(SCRIPT_DIR, "black_panther_a2212_quad.FCStd"))

    # All 3D-printable structural/gimbal parts (excludes metal motors, servos, bolts, pins, shafts, and bought 8045 props)
    printable_names = [
        "Panther_X_Chassis",
        "Panther_Battery_Bay",
        "Panther_Electronics_Bay",
        "Panther_Top_Cover",
        "Panther_Arm_FL",
        "Panther_Arm_FR",
        "Panther_Arm_RL",
        "Panther_Arm_RR",
    ]
    for tag in ("FL", "FR", "RL", "RR"):
        printable_names.extend([
            f"{tag}_A2212_Cradle",
            f"{tag}_Innermost_Gimbal_Ring",
            f"{tag}_Inner_Gimbal_Ring",
            f"{tag}_Spur_Gear_Upper",
            f"{tag}_Spur_Gear_Lower",
            f"{tag}_Pitch_Servo_Mount",
        ])

    items = []
    for name in printable_names:
        obj = src.getObject(name)
        if obj is not None:
            items.append((name, obj.Shape))

    # Split Panther_Landing_Skids into 2 separate skids and lay flat on their side (Z-height ~12.4mm)
    skids_obj = src.getObject("Panther_Landing_Skids")
    if skids_obj is not None:
        for idx, solid in enumerate(skids_obj.Shape.Solids):
            s = solid.copy()
            s.rotate(App.Vector(0, 0, 0), App.Vector(1, 0, 0), 90.0)
            items.append((f"Panther_Landing_Skid_{idx + 1}", s))

    out_all = os.path.join(OUT_DIR, "black_panther_a2212_all_printable.stl")
    n_all, dims_all = pack_parts_to_stl(items, out_all)
    print(f"EXPORTED_ALL_STL: {out_all} | parts={n_all} | bbox={dims_all[0]:.1f}x{dims_all[1]:.1f}x{dims_all[2]:.1f} mm")

    # Also export a 1-Pod Bench Test Plate (1 arm + 1 cradle + 2 gimbal rings + 2 gears)
    pod_items = [(n, s) for n, s in items if n.startswith(("Panther_Arm_FL", "FL_"))]
    out_pod = os.path.join(OUT_DIR, "black_panther_a2212_1pod_test.stl")
    n_pod, dims_pod = pack_parts_to_stl(pod_items, out_pod)
    print(f"EXPORTED_1POD_STL: {out_pod} | parts={n_pod} | bbox={dims_pod[0]:.1f}x{dims_pod[1]:.1f}x{dims_pod[2]:.1f} mm")

    App.closeDocument(src.Name)


if __name__ in ("__main__", "export_all_printable_stl"):
    main()
