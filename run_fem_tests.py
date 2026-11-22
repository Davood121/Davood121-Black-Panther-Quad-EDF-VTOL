"""Automated per-part FreeCAD FEM (Gmsh + CalculiX) structural analysis."""

import os
import sys
import FreeCAD as App
import ObjectsFem
import Part
from femmesh.gmshtools import GmshTools
from femtools import ccxtools

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

OUT_DIR = os.path.join(SCRIPT_DIR, "fem_results")
os.makedirs(OUT_DIR, exist_ok=True)


def _find_faces_by_bbox(shape, axis, target_val, tol=2.0):
    """Return face names ('Face1', ...) whose BoundBox center is near target_val along axis."""
    idx_map = {"x": 0, "y": 1, "z": 2}
    ax = idx_map[axis]
    matched = []
    for i, face in enumerate(shape.Faces, start=1):
        c = face.CenterOfMass
        val = (c.x, c.y, c.z)[ax]
        if abs(val - target_val) <= tol:
            matched.append(f"Face{i}")
    if not matched:
        # Fallback: pick the face closest to target_val
        best_i = min(
            range(1, len(shape.Faces) + 1),
            key=lambda i: abs((shape.Faces[i - 1].CenterOfMass.x,
                               shape.Faces[i - 1].CenterOfMass.y,
                               shape.Faces[i - 1].CenterOfMass.z)[ax] - target_val),
        )
        matched.append(f"Face{best_i}")
    return matched


def run_part_fem(name, shape, youngs_mpa, poisson, yield_mpa, fixed_spec, load_spec, mesh_max=5.0):
    """Create a FreeCAD FEM analysis for a single part, mesh with Gmsh, and solve with CalculiX."""
    doc = App.newDocument(f"FEM_{name}")
    part_obj = doc.addObject("Part::Feature", name)
    part_obj.Shape = shape
    doc.recompute()

    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiX")
    solver.GeometricalNonlinearity = "linear"
    solver.ThermoMechSteadyState = True
    solver.MatrixSolverType = "default"
    solver.IterationsControlParameterTimeUse = False
    analysis.addObject(solver)

    mat = ObjectsFem.makeMaterialSolid(doc, "Material")
    m = mat.Material
    m["Name"] = f"{name}_Mat"
    m["YoungsModulus"] = f"{youngs_mpa} MPa"
    m["PoissonRatio"] = f"{poisson}"
    m["Density"] = "1600 kg/m^3"
    mat.Material = m
    analysis.addObject(mat)

    fixed_faces = _find_faces_by_bbox(shape, *fixed_spec)
    fixed = ObjectsFem.makeConstraintFixed(doc, "Fixed")
    fixed.References = [(part_obj, fixed_faces)]
    analysis.addObject(fixed)

    load_axis, load_target, load_tol, force_n, dir_vec = load_spec
    load_faces = _find_faces_by_bbox(shape, load_axis, load_target, load_tol)
    force = ObjectsFem.makeConstraintForce(doc, "Force")
    force.References = [(part_obj, load_faces)]
    force.Force = f"{force_n} N"
    force.Reversed = False
    # Set direction via a tiny edge/normal or default; use DirectionVector if supported
    if hasattr(force, "DirectionVector"):
        force.DirectionVector = dir_vec
    analysis.addObject(force)

    mesh_obj = ObjectsFem.makeMeshGmsh(doc, "FEMMeshGmsh")
    mesh_obj.Shape = part_obj
    mesh_obj.CharacteristicLengthMax = mesh_max
    mesh_obj.CharacteristicLengthMin = max(0.8, mesh_max / 4.0)
    mesh_obj.ElementOrder = "2nd"
    analysis.addObject(mesh_obj)
    doc.recompute()

    gmsh = GmshTools(mesh_obj)
    err = gmsh.create_mesh()
    if err:
        raise RuntimeError(f"Gmsh failed for {name}: {err}")

    fea = ccxtools.FemToolsCcx(analysis, solver)
    fea.update_objects()
    fea.setup_working_dir()
    fea.setup_ccx()
    message = fea.check_prerequisites()
    if message:
        raise RuntimeError(f"FEM prerequisites failed for {name}: {message}")

    fea.write_inp_file()
    fea.run()
    fea.load_results()
    doc.recompute()

    res = None
    for obj in doc.Objects:
        if obj.isDerivedFrom("Fem::FemResultObject"):
            res = obj
            break

    max_disp = max(res.DisplacementLengths) if res and res.DisplacementLengths else 0.0
    max_vm = max(res.vonMises) if res and res.vonMises else 0.0
    sf = (yield_mpa / max_vm) if max_vm > 1e-6 else float("inf")

    nodes = mesh_obj.FemMesh.NodeCount
    elems = mesh_obj.FemMesh.VolumeCount
    fcstd_out = os.path.join(OUT_DIR, f"{name}_fem.FCStd")
    doc.saveAs(fcstd_out)
    App.closeDocument(doc.Name)

    return {
        "part": name,
        "nodes": nodes,
        "elems": elems,
        "max_disp_mm": max_disp,
        "max_vm_mpa": max_vm,
        "yield_mpa": yield_mpa,
        "safety_factor": sf,
        "file": fcstd_out,
    }


def main():
    src_path = os.path.join(SCRIPT_DIR, "black_panther_quad_edf.FCStd")
    src_doc = App.openDocument(src_path)

    # Extract clean shapes (reset placement to local where needed)
    arm_shape = src_doc.getObject("Panther_Arm_FL").Shape.copy()
    skid_shape = src_doc.getObject("Panther_Landing_Skids").Shape.copy()
    bracket_shape = src_doc.getObject("FL_CF_Servo_Bracket").Shape.copy()
    inner_shape = src_doc.getObject("FL_Inner_Gimbal_Ring").Shape.copy()
    outer_shape = src_doc.getObject("FL_Outer_Gimbal_Ring").Shape.copy()
    shaft_shape = src_doc.getObject("FL_Gimbal_Drive_Shaft").Shape.copy()
    App.closeDocument(src_doc.Name)

    # Compute bounding-box targets dynamically for each part
    cases = [
        (
            "Panther_Arm_FL",
            arm_shape,
            70000.0,
            0.30,
            300.0,  # CF laminate
            ("x", arm_shape.BoundBox.XMin, 8.0),
            ("x", arm_shape.BoundBox.XMax, 12.0, 20.0, App.Vector(0, 0, 1)),
            6.0,
        ),
        (
            "CF_Servo_Bracket",
            bracket_shape,
            70000.0,
            0.30,
            300.0,
            ("z", bracket_shape.BoundBox.ZMin, 3.0),
            ("z", bracket_shape.BoundBox.ZMax, 3.0, 20.0, App.Vector(0, 0, 1)),
            3.0,
        ),
        (
            "Inner_Gimbal_Ring",
            inner_shape,
            2500.0,
            0.35,
            50.0,  # PETG / CF-Nylon
            ("z", inner_shape.BoundBox.ZMin, 2.0),
            ("z", inner_shape.BoundBox.ZMax, 2.0, 20.0, App.Vector(0, 0, 1)),
            3.5,
        ),
        (
            "Outer_Gimbal_Ring",
            outer_shape,
            2500.0,
            0.35,
            50.0,
            ("z", outer_shape.BoundBox.ZMin, 4.0),
            ("z", outer_shape.BoundBox.ZMax, 4.0, 20.0, App.Vector(0, 0, 1)),
            3.5,
        ),
        (
            "Gimbal_Drive_Shaft",
            shaft_shape,
            193000.0,
            0.29,
            215.0,  # 304 Stainless Steel
            ("x", shaft_shape.BoundBox.XMin, 3.0),
            ("x", shaft_shape.BoundBox.XMax, 3.0, 20.0, App.Vector(0, 0, 1)),
            2.0,
        ),
        (
            "Panther_Landing_Skids",
            skid_shape,
            70000.0,
            0.30,
            300.0,
            ("z", skid_shape.BoundBox.ZMax, 12.0),
            ("z", skid_shape.BoundBox.ZMin, 8.0, 40.0, App.Vector(0, 0, 1)),
            6.0,
        ),
    ]

    results = []
    for c in cases:
        print(f"=== Running FEM on {c[0]} ===")
        try:
            r = run_part_fem(*c)
            results.append(r)
            print(
                f"  OK: {r['nodes']} nodes, {r['elems']} elems | "
                f"MaxDisp={r['max_disp_mm']:.4f} mm | "
                f"MaxVM={r['max_vm_mpa']:.3f} MPa | "
                f"SF={r['safety_factor']:.1f}"
            )
        except Exception as exc:
            print(f"  FAILED {c[0]}: {exc}")

    print("\n=== FEM SUMMARY ===")
    for r in results:
        print(
            f"{r['part']:24s} | Disp: {r['max_disp_mm']:8.4f} mm | "
            f"VonMises: {r['max_vm_mpa']:8.3f} MPa | Yield: {r['yield_mpa']:6.1f} MPa | "
            f"SF: {r['safety_factor']:6.1f}"
        )


if __name__ in ("__main__", "run_fem_tests"):
    main()
