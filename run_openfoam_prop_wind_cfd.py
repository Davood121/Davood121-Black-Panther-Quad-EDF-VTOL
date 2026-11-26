"""OpenFOAM-12 3D CFD: (A) Blade-resolved 8045 Propeller MRF & (B) 15 m/s (54 km/h) Wind-Tunnel Test."""

import os
import shutil
import subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.tri as mtri

try:
    import FreeCAD as App
    import Mesh
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CASE_HOVER = os.path.join(BASE_DIR, "cfd_case_prop_hover")
CASE_WIND = os.path.join(BASE_DIR, "cfd_case_prop_wind")
SETVARS = r"C:\Program Files\blueCFD-Core-2024\setvars_OF12.bat"


def write_file(case_dir, rel, txt):
    p = os.path.join(case_dir, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)


def run_of(case_dir, cmd_str):
    bat_path = os.path.join(case_dir, "_run.bat")
    with open(bat_path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write(
            '@echo off\n'
            'set "PATH=C:\\Windows\\System32;C:\\Windows;C:\\Windows\\System32\\Wbem"\n'
            f'call "{SETVARS}"\n'
            f'cd /d "{case_dir}"\n'
            f'{cmd_str}\n'
        )
    res = subprocess.run(["cmd", "/c", bat_path], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"RC={res.returncode}")
        print(res.stdout[-1500:])
        print(res.stderr[-1500:])
        raise RuntimeError(f"OpenFOAM command failed in {case_dir}: {cmd_str}")
    return res.stdout


def read_of_field(path, is_vec=False):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    idx = next(i for i, ln in enumerate(lines) if ln.strip().isdigit() and i > 10 and lines[i + 1].strip() == "(")
    n = int(lines[idx].strip())
    data = []
    for ln in lines[idx + 2 : idx + 2 + n]:
        s = ln.strip()
        data.append([float(x) for x in s[1:-1].split()] if is_vec else float(s))
    return np.array(data)


def export_stl_freecad():
    for c in (CASE_HOVER, CASE_WIND):
        os.makedirs(os.path.join(c, "constant", "triSurface"), exist_ok=True)

    doc = App.openDocument(os.path.join(BASE_DIR, "gimbal_a2212_8045.FCStd"))
    parts = []
    for name in ("Prop_8045", "A2212_1400KV", "A2212_Cradle",
                 "Innermost_Gimbal_Ring", "Inner_Gimbal_Ring", "Outer_Gimbal_Ring"):
        obj = doc.getObject(name)
        if obj is not None:
            parts.append(obj.Shape.copy())
    comp = Part.makeCompound(parts)
    comp.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
    comp.scale(0.001)  # mm -> m

    tmp = App.newDocument("PropWindSTL")
    pf = tmp.addObject("Part::Feature", "PropWindPod")
    pf.Shape = comp
    tmp.recompute()
    stl_h = os.path.join(CASE_HOVER, "constant", "triSurface", "prop_wind_pod.stl")
    stl_w = os.path.join(CASE_WIND, "constant", "triSurface", "prop_wind_pod.stl")
    Mesh.export([pf], stl_h)
    shutil.copy2(stl_h, stl_w)
    App.closeDocument(tmp.Name)
    App.closeDocument(doc.Name)
    print("EXPORTED_PROP_WIND_STL:", stl_h)


def setup_common(case_dir, end_time=100, with_forces=False):
    forces_block = ""
    if with_forces:
        forces_block = """
functions
{
    forces
    {
        type            forces;
        libs            ("libforces.so");
        writeControl    timeStep;
        writeInterval   100;
        patches         (prop_wind_pod);
        rho             rhoInf;
        rhoInf          1.225;
        CofR            (0 0 0);
    }
}
"""
    write_file(case_dir, "system/controlDict", f"""FoamFile
{{
    format      ascii;
    class       dictionary;
    object      controlDict;
}}
application     foamRun;
solver          incompressibleFluid;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         {end_time};
deltaT          1;
writeControl    timeStep;
writeInterval   {end_time};
purgeWrite      1;
writeFormat     ascii;
writePrecision  6;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;
{forces_block}
""")

    write_file(case_dir, "system/fvSchemes", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      fvSchemes;
}
ddtSchemes { default steadyState; }
gradSchemes { default Gauss linear; }
divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss upwind;
    div(phi,k)      bounded Gauss upwind;
    div(phi,omega)  bounded Gauss upwind;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear corrected; }
interpolationSchemes { default linear; }
snGradSchemes { default corrected; }
wallDist { method meshWave; }
""")

    write_file(case_dir, "system/fvSolution", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      fvSolution;
}
solvers
{
    p { solver GAMG; smoother GaussSeidel; tolerance 1e-6; relTol 0.1; }
    "(U|k|omega)" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-6; relTol 0.1; }
}
SIMPLE
{
    nNonOrthogonalCorrectors 0;
    consistent yes;
    residualControl { p 1e-4; U 1e-4; "(k|omega)" 1e-4; }
}
relaxationFactors { equations { U 0.6; k 0.6; omega 0.6; } }
""")

    write_file(case_dir, "constant/physicalProperties", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      physicalProperties;
}
viscosityModel  constant;
nu              1.5e-05;
""")

    write_file(case_dir, "constant/momentumTransport", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      momentumTransport;
}
simulationType  RAS;
RAS
{
    model           kOmegaSST;
    turbulence      on;
    printCoeffs     on;
}
""")


def run_prop_hover():
    setup_common(CASE_HOVER, end_time=100, with_forces=True)
    write_file(CASE_HOVER, "system/blockMeshDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}
scale 1;
vertices
(
    (-0.24 -0.24 -0.26)
    ( 0.24 -0.24 -0.26)
    ( 0.24  0.24 -0.26)
    (-0.24  0.24 -0.26)
    (-0.24 -0.24  0.20)
    ( 0.24 -0.24  0.20)
    ( 0.24  0.24  0.20)
    (-0.24  0.24  0.20)
);
blocks ( hex (0 1 2 3 4 5 6 7) (26 26 26) simpleGrading (1 1 1) );
patches
(
    patch inlet ( (4 7 6 5) )
    patch outlet ( (0 1 2 3) )
    patch sides ( (0 4 5 1) (1 5 6 2) (2 6 7 3) (3 7 4 0) )
);
""")

    write_file(CASE_HOVER, "system/snappyHexMeshDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      snappyHexMeshDict;
}
castellatedMesh true;
snap            true;
addLayers       false;
geometry
{
    prop_wind_pod { type triSurfaceMesh; file "prop_wind_pod.stl"; }
}
castellatedMeshControls
{
    maxLocalCells 250000;
    maxGlobalCells 500000;
    minRefinementCells 0;
    maxLoadUnbalance 0.1;
    nCellsBetweenLevels 2;
    features ();
    refinementSurfaces
    {
        prop_wind_pod { level (2 3); patchInfo { type wall; } }
    }
    resolveFeatureAngle 30;
    refinementRegions {}
    locationInMesh (0.20 0.20 0.16);
    allowFreeStandingZoneFaces true;
}
snapControls { nSmoothPatch 3; tolerance 2.0; nSolveIter 30; nRelaxIter 5; }
addLayersControls { relativeSizes true; layers {} expansionRatio 1.2; finalLayerThickness 0.3; minThickness 0.1; nGrow 0; featureAngle 60; nRelaxIter 3; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10; maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0; nLayerIter 50; }
meshQualityControls { maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13; minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.02; minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75; }
mergeTolerance 1e-6;
""")

    write_file(CASE_HOVER, "system/topoSetDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      topoSetDict;
}
actions
(
    {
        name    propMRF;
        type    cellSet;
        action  new;
        source  cylinderToCell;
        point1  (0 0 0.056);
        point2  (0 0 0.080);
        radius  0.104;
    }
    {
        name    propMRF;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     propMRF;
    }
);
""")

    write_file(CASE_HOVER, "constant/MRFProperties", """FoamFile
{
    format      ascii;
    class       dictionary;
    location    "constant";
    object      MRFProperties;
}
MRF
{
    cellZone    propMRF;
    origin      (0 0 0.068);
    axis        (0 0 1);
    omega       12000 [rpm];
}
""")

    write_file(CASE_HOVER, "0/U", """FoamFile
{
    format      ascii;
    class       volVectorField;
    object      U;
}
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform (0 0 -2);
boundaryField
{
    inlet   { type fixedValue; value uniform (0 0 -2); }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    prop_wind_pod { type noSlip; }
}
""")
    write_file(CASE_HOVER, "0/p", "FoamFile { format ascii; class volScalarField; object p; }\ndimensions [0 2 -2 0 0 0 0];\ninternalField uniform 0;\nboundaryField { inlet { type zeroGradient; } outlet { type fixedValue; value uniform 0; } sides { type zeroGradient; } prop_wind_pod { type zeroGradient; } }\n")
    write_file(CASE_HOVER, "0/k", "FoamFile { format ascii; class volScalarField; object k; }\ndimensions [0 2 -2 0 0 0 0];\ninternalField uniform 0.5;\nboundaryField { inlet { type fixedValue; value uniform 0.5; } outlet { type zeroGradient; } sides { type slip; } prop_wind_pod { type kqRWallFunction; value uniform 0.5; } }\n")
    write_file(CASE_HOVER, "0/omega", "FoamFile { format ascii; class volScalarField; object omega; }\ndimensions [0 0 -1 0 0 0 0];\ninternalField uniform 100;\nboundaryField { inlet { type fixedValue; value uniform 100; } outlet { type zeroGradient; } sides { type slip; } prop_wind_pod { type omegaWallFunction; value uniform 100; } }\n")
    write_file(CASE_HOVER, "0/nut", "FoamFile { format ascii; class volScalarField; object nut; }\ndimensions [0 2 -1 0 0 0 0];\ninternalField uniform 0;\nboundaryField { inlet { type calculated; value uniform 0; } outlet { type calculated; value uniform 0; } sides { type calculated; value uniform 0; } prop_wind_pod { type nutkWallFunction; value uniform 0; } }\n")

    print("[Case A: 3D Prop Blade MRF] blockMesh -> snappyHexMesh -> topoSet -> foamRun...")
    run_of(CASE_HOVER, "blockMesh")
    run_of(CASE_HOVER, "snappyHexMesh -overwrite")
    run_of(CASE_HOVER, "topoSet")
    run_of(CASE_HOVER, "foamPostProcess -func writeCellCentres -time 0")
    run_of(CASE_HOVER, "foamRun")

    t_dirs = [d for d in os.listdir(CASE_HOVER) if d.isdigit() and int(d) > 0]
    lt = max(t_dirs, key=int)
    cc = read_of_field(os.path.join(CASE_HOVER, "0", "C"), is_vec=True) * 1000.0
    u_vec = read_of_field(os.path.join(CASE_HOVER, lt, "U"), is_vec=True)
    p_pa = read_of_field(os.path.join(CASE_HOVER, lt, "p"), is_vec=False) * 1.225
    u_mag = np.linalg.norm(u_vec, axis=1)
    print(f"PROP_MRF_OK | cells={len(u_mag)} | max_|U|={u_mag.max():.2f} m/s | p_range=[{p_pa.min():.0f}, {p_pa.max():.0f}] Pa")

    # Plot XY blade-plane slice (Z=68mm) and XZ centerplane
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.8), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 Blade-Resolved 3D Propeller MRF (8045 @ 12,000 RPM, {len(u_mag):,} cells)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    m_xy = np.abs(cc[:, 2] - 68.0) < 9.0
    ax1.set_facecolor("#16181d")
    tc1 = ax1.tricontourf(cc[m_xy, 0], cc[m_xy, 1], u_mag[m_xy], levels=28, cmap="turbo")
    ax1.set_title(f"8045 Blade Plane (Z=+68mm) Velocity |U| (Max {u_mag[m_xy].max():.1f} m/s)", color="#e5e7eb", fontsize=10)
    ax1.set_xlabel("X (mm)", color="#e5e7eb")
    ax1.set_ylabel("Y (mm)", color="#e5e7eb")
    ax1.set_aspect("equal")
    ax1.tick_params(colors="#9ca3af")
    cb1 = fig.colorbar(tc1, ax=ax1, shrink=0.84)
    cb1.set_label("m/s", color="#e5e7eb")
    cb1.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    m_xz = np.abs(cc[:, 1]) < 12.0
    ax2.set_facecolor("#16181d")
    tc2 = ax2.tricontourf(cc[m_xz, 0], cc[m_xz, 2], p_pa[m_xz], levels=28, cmap="coolwarm")
    ax2.set_title(f"Static Pressure Around 8045 Blades & Pod (Pa)", color="#e5e7eb", fontsize=10)
    ax2.set_xlabel("X (mm)", color="#e5e7eb")
    ax2.set_ylabel("Z (mm)", color="#e5e7eb")
    ax2.set_aspect("equal")
    ax2.tick_params(colors="#9ca3af")
    cb2 = fig.colorbar(tc2, ax=ax2, shrink=0.84)
    cb2.set_label("Pa", color="#e5e7eb")
    cb2.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out_png = os.path.join(BASE_DIR, "fem_results", "openfoam_prop_blade_cfd.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED_PROP:", out_png)


def run_wind_tunnel():
    setup_common(CASE_WIND, end_time=100, with_forces=True)
    # Horizontal wind tunnel: flow in +X at 15 m/s (54 km/h) across the 8045 prop + gimbal pod
    write_file(CASE_WIND, "system/blockMeshDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}
scale 1;
vertices
(
    (-0.22 -0.22 -0.22)
    ( 0.36 -0.22 -0.22)
    ( 0.36  0.22 -0.22)
    (-0.22  0.22 -0.22)
    (-0.22 -0.22  0.20)
    ( 0.36 -0.22  0.20)
    ( 0.36  0.22  0.20)
    (-0.22  0.22  0.20)
);
blocks ( hex (0 1 2 3 4 5 6 7) (30 24 24) simpleGrading (1 1 1) );
patches
(
    patch inlet ( (0 4 7 3) )
    patch outlet ( (1 2 6 5) )
    patch sides ( (0 1 5 4) (3 7 6 2) (0 3 2 1) (4 5 6 7) )
);
""")

    write_file(CASE_WIND, "system/snappyHexMeshDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      snappyHexMeshDict;
}
castellatedMesh true;
snap            true;
addLayers       false;
geometry
{
    prop_wind_pod { type triSurfaceMesh; file "prop_wind_pod.stl"; }
}
castellatedMeshControls
{
    maxLocalCells 250000;
    maxGlobalCells 500000;
    minRefinementCells 0;
    maxLoadUnbalance 0.1;
    nCellsBetweenLevels 2;
    features ();
    refinementSurfaces
    {
        prop_wind_pod { level (2 2); patchInfo { type wall; } }
    }
    resolveFeatureAngle 30;
    refinementRegions {}
    locationInMesh (0.28 0.16 0.16);
    allowFreeStandingZoneFaces true;
}
snapControls { nSmoothPatch 3; tolerance 2.0; nSolveIter 30; nRelaxIter 5; }
addLayersControls { relativeSizes true; layers {} expansionRatio 1.2; finalLayerThickness 0.3; minThickness 0.1; nGrow 0; featureAngle 60; nRelaxIter 3; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10; maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0; nLayerIter 50; }
meshQualityControls { maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13; minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.02; minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75; }
mergeTolerance 1e-6;
""")

    write_file(CASE_WIND, "system/topoSetDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      topoSetDict;
}
actions
(
    {
        name    propDisk;
        type    cellSet;
        action  new;
        source  cylinderToCell;
        point1  (0 0 0.060);
        point2  (0 0 0.076);
        radius  0.1016;
    }
    {
        name    propDisk;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     propDisk;
    }
);
""")

    write_file(CASE_WIND, "constant/fvModels", """FoamFile
{
    format      ascii;
    class       dictionary;
    location    "constant";
    object      fvModels;
}
propThrust
{
    type            semiImplicitSource;
    select          cellZone;
    cellZone        propDisk;
    volumeMode      specific;
    sources
    {
        U
        {
            explicit    (0 0 -13500);
            implicit    0;
        }
    }
}
""")

    write_file(CASE_WIND, "0/U", """FoamFile
{
    format      ascii;
    class       volVectorField;
    object      U;
}
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform (15 0 0);
boundaryField
{
    inlet   { type fixedValue; value uniform (15 0 0); }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    prop_wind_pod { type noSlip; }
}
""")
    write_file(CASE_WIND, "0/p", "FoamFile { format ascii; class volScalarField; object p; }\ndimensions [0 2 -2 0 0 0 0];\ninternalField uniform 0;\nboundaryField { inlet { type zeroGradient; } outlet { type fixedValue; value uniform 0; } sides { type zeroGradient; } prop_wind_pod { type zeroGradient; } }\n")
    write_file(CASE_WIND, "0/k", "FoamFile { format ascii; class volScalarField; object k; }\ndimensions [0 2 -2 0 0 0 0];\ninternalField uniform 1.0;\nboundaryField { inlet { type fixedValue; value uniform 1.0; } outlet { type zeroGradient; } sides { type slip; } prop_wind_pod { type kqRWallFunction; value uniform 1.0; } }\n")
    write_file(CASE_WIND, "0/omega", "FoamFile { format ascii; class volScalarField; object omega; }\ndimensions [0 0 -1 0 0 0 0];\ninternalField uniform 150;\nboundaryField { inlet { type fixedValue; value uniform 150; } outlet { type zeroGradient; } sides { type slip; } prop_wind_pod { type omegaWallFunction; value uniform 150; } }\n")
    write_file(CASE_WIND, "0/nut", "FoamFile { format ascii; class volScalarField; object nut; }\ndimensions [0 2 -1 0 0 0 0];\ninternalField uniform 0;\nboundaryField { inlet { type calculated; value uniform 0; } outlet { type calculated; value uniform 0; } sides { type calculated; value uniform 0; } prop_wind_pod { type nutkWallFunction; value uniform 0; } }\n")

    print("[Case B: 15 m/s Wind Tunnel] blockMesh -> snappyHexMesh -> topoSet -> foamRun...")
    run_of(CASE_WIND, "blockMesh")
    run_of(CASE_WIND, "snappyHexMesh -overwrite")
    run_of(CASE_WIND, "topoSet")
    run_of(CASE_WIND, "foamPostProcess -func writeCellCentres -time 0")
    run_of(CASE_WIND, "foamRun")

    # Parse aerodynamic forces (drag Fx, lift Fz)
    f_path = os.path.join(CASE_WIND, "postProcessing", "forces", "0", "force.dat")
    drag_n, lift_n = 0.0, 0.0
    if os.path.exists(f_path):
        with open(f_path, "r", encoding="utf-8", errors="ignore") as f:
            last = [ln for ln in f.readlines() if ln.strip() and not ln.strip().startswith("#")][-1]
        nums = [float(x) for x in last.replace("(", " ").replace(")", " ").split()]
        # OpenFOAM force.dat: time, (total_x total_y total_z) or (fp_x fp_y fp_z) (fv_x fv_y fv_z)
        if len(nums) >= 7:
            drag_n = nums[1] + nums[4]
            lift_n = nums[3] + nums[6]

    t_dirs = [d for d in os.listdir(CASE_WIND) if d.isdigit() and int(d) > 0]
    lt = max(t_dirs, key=int)
    cc = read_of_field(os.path.join(CASE_WIND, "0", "C"), is_vec=True) * 1000.0
    u_vec = read_of_field(os.path.join(CASE_WIND, lt, "U"), is_vec=True)
    p_pa = read_of_field(os.path.join(CASE_WIND, lt, "p"), is_vec=False) * 1.225
    u_mag = np.linalg.norm(u_vec, axis=1)

    print(f"WIND_TUNNEL_OK | cells={len(u_mag)} | max_|U|={u_mag.max():.2f} m/s | p_range=[{p_pa.min():.0f}, {p_pa.max():.0f}] Pa | pod_drag={drag_n:.3f} N | pod_lift={lift_n:.3f} N")

    m_xz = np.abs(cc[:, 1]) < 12.0
    xs, zs = cc[m_xz, 0], cc[m_xz, 2]
    um, pm = u_mag[m_xz], p_pa[m_xz]
    ux, uz = u_vec[m_xz, 0], u_vec[m_xz, 2]

    tri = mtri.Triangulation(xs, zs)
    gx = np.linspace(-210, 350, 100)
    gz = np.linspace(-210, 190, 90)
    GX, GZ = np.meshgrid(gx, gz)
    UX_g = np.ma.filled(mtri.LinearTriInterpolator(tri, ux)(GX, GZ), 0.0)
    UZ_g = np.ma.filled(mtri.LinearTriInterpolator(tri, uz)(GX, GZ), 0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14.0, 5.8), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 Wind-Tunnel Test: 15 m/s (54 km/h) Crosswind + 8045 Pod ({len(u_mag):,} cells, Drag={drag_n:.2f} N)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    ax1.set_facecolor("#16181d")
    tc1 = ax1.tricontourf(xs, zs, um, levels=28, cmap="turbo")
    ax1.streamplot(gx, gz, UX_g, UZ_g, color="white", density=1.25, linewidth=0.75, arrowsize=0.9)
    ax1.set_title(f"Crosswind Velocity |U| & Wake Deflection (Max {um.max():.1f} m/s)", color="#e5e7eb", fontsize=10)
    ax1.set_xlabel("Streamwise X (mm) [15 m/s ->]", color="#e5e7eb")
    ax1.set_ylabel("Vertical Z (mm)", color="#e5e7eb")
    ax1.set_aspect("equal")
    ax1.tick_params(colors="#9ca3af")
    cb1 = fig.colorbar(tc1, ax=ax1, shrink=0.84)
    cb1.set_label("m/s", color="#e5e7eb")
    cb1.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    ax2.set_facecolor("#16181d")
    tc2 = ax2.tricontourf(xs, zs, pm, levels=28, cmap="coolwarm")
    ax2.set_title("Wind-Tunnel Static Pressure (Pa)", color="#e5e7eb", fontsize=10)
    ax2.set_xlabel("Streamwise X (mm) [15 m/s ->]", color="#e5e7eb")
    ax2.set_ylabel("Vertical Z (mm)", color="#e5e7eb")
    ax2.set_aspect("equal")
    ax2.tick_params(colors="#9ca3af")
    cb2 = fig.colorbar(tc2, ax=ax2, shrink=0.84)
    cb2.set_label("Pa", color="#e5e7eb")
    cb2.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out_png = os.path.join(BASE_DIR, "fem_results", "openfoam_wind_tunnel_cfd.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED_WIND:", out_png)


def main():
    if HAS_FREECAD:
        export_stl_freecad()
        return
    run_prop_hover()
    run_wind_tunnel()


if __name__ in ("__main__", "run_openfoam_prop_wind_cfd"):
    main()
