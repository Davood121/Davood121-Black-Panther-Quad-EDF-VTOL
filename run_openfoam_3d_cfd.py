"""Automated 3D OpenFOAM-12 CFD simulation on the 50mm EDF Shroud + Exhaust Nozzle."""

import os
import shutil
import subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
try:
    import FreeCAD as App
    import Mesh
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CASE_DIR = os.path.join(BASE_DIR, "cfd_case")
SETVARS = r"C:\Program Files\blueCFD-Core-2024\setvars_OF12.bat"


def write_file(rel, txt):
    p = os.path.join(CASE_DIR, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)


def run_of(cmd_str):
    bat_path = os.path.join(CASE_DIR, "_run.bat")
    with open(bat_path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write(
            '@echo off\n'
            'set "PATH=C:\\Windows\\System32;C:\\Windows;C:\\Windows\\System32\\Wbem"\n'
            f'call "{SETVARS}"\n'
            f'cd /d "{CASE_DIR}"\n'
            f'{cmd_str}\n'
        )
    res = subprocess.run(["cmd", "/c", bat_path], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"RC={res.returncode}")
        print(res.stdout[-1500:])
        print(res.stderr[-1500:])
        raise RuntimeError(f"OpenFOAM command failed: {cmd_str}")
    return res.stdout


def main():
    stl_path = os.path.join(CASE_DIR, "constant", "triSurface", "edf_duct.stl")
    if HAS_FREECAD:
        if os.path.exists(CASE_DIR):
            shutil.rmtree(CASE_DIR)
        os.makedirs(os.path.join(CASE_DIR, "constant", "triSurface"), exist_ok=True)

        # Export vertical (Z-axis) EDF shroud + nozzle STL in meters
        doc = App.openDocument(os.path.join(BASE_DIR, "gimbal_edf_50mm.FCStd"))
        shroud = doc.getObject("EDF_Shroud").Shape.copy()
        nozzle = doc.getObject("EDF_Exhaust_Nozzle").Shape.copy()
        duct = shroud.fuse(nozzle)
        duct.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
        duct.scale(0.001)  # mm -> m
        App.closeDocument(doc.Name)

        mesh_doc = App.newDocument("CFD_STL")
        pf = mesh_doc.addObject("Part::Feature", "EDF_Duct")
        pf.Shape = duct
        mesh_doc.recompute()
        Mesh.export([pf], stl_path)
        App.closeDocument(mesh_doc.Name)
        print("EXPORTED_STL:", stl_path)
        return
    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"Missing {stl_path}; run with freecadcmd.exe first.")

    write_file("system/controlDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      controlDict;
}
application     foamRun;
solver          incompressibleFluid;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         120;
deltaT          1;
writeControl    timeStep;
writeInterval   120;
purgeWrite      0;
writeFormat     ascii;
writePrecision  6;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;
""")

    write_file("system/fvSchemes", """FoamFile
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

    write_file("system/fvSolution", """FoamFile
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
relaxationFactors { equations { U 0.7; k 0.7; omega 0.7; } }
""")

    write_file("system/blockMeshDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}
scale 1;
vertices
(
    (-0.075 -0.075 -0.120)
    ( 0.075 -0.075 -0.120)
    ( 0.075  0.075 -0.120)
    (-0.075  0.075 -0.120)
    (-0.075 -0.075  0.090)
    ( 0.075 -0.075  0.090)
    ( 0.075  0.075  0.090)
    (-0.075  0.075  0.090)
);
blocks ( hex (0 1 2 3 4 5 6 7) (26 26 36) simpleGrading (1 1 1) );
patches
(
    patch inlet ( (4 7 6 5) )
    patch outlet ( (0 1 2 3) )
    patch sides ( (0 4 5 1) (1 5 6 2) (2 6 7 3) (3 7 4 0) )
);
""")

    write_file("system/snappyHexMeshDict", """FoamFile
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
    edf_duct { type triSurfaceMesh; file "edf_duct.stl"; }
}
castellatedMeshControls
{
    maxLocalCells 200000;
    maxGlobalCells 400000;
    minRefinementCells 0;
    nCellsBetweenLevels 2;
    features ();
    refinementSurfaces { edf_duct { level (1 2); patchInfo { type wall; } } }
    resolveFeatureAngle 30;
    refinementRegions {}
    locationInMesh (0 0 0.07);
    allowFreeStandingZoneFaces true;
}
snapControls
{
    nSmoothPatch 3;
    tolerance 2.0;
    nSolveIter 30;
    nRelaxIter 5;
}
addLayersControls { relativeSizes true; layers {} expansionRatio 1.2; finalLayerThickness 0.3; minThickness 0.1; }
meshQualityControls { maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13; minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.05; minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75; }
mergeTolerance 1e-6;
""")

    write_file("constant/physicalProperties", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      physicalProperties;
}
viscosityModel  constant;
nu              1.5e-05;
""")

    write_file("constant/momentumTransport", """FoamFile
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

    write_file("constant/fvModels", """FoamFile
{
    format      ascii;
    class       dictionary;
    location    "constant";
    object      fvModels;
}
fanThrust
{
    type            semiImplicitSource;
    select          cellZone;
    cellZone        fanDisk;
    volumeMode      specific;
    sources
    {
        U
        {
            explicit    (0 0 -220000);
            implicit    0;
        }
    }
}
""")

    write_file("system/topoSetDict", """FoamFile
{
    format      ascii;
    class       dictionary;
    object      topoSetDict;
}
actions
(
    {
        name    fanDisk;
        type    cellSet;
        action  new;
        source  cylinderToCell;
        point1  (0 0 0.006);
        point2  (0 0 0.020);
        radius  0.0245;
    }
    {
        name    fanDisk;
        type    cellSet;
        action  delete;
        source  cylinderToCell;
        point1  (0 0 0.005);
        point2  (0 0 0.021);
        radius  0.009;
    }
    {
        name    fanDisk;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     fanDisk;
    }
);
""")

    write_file("0/U", """FoamFile
{
    format      ascii;
    class       volVectorField;
    object      U;
}
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform (0 0 -10);
boundaryField
{
    inlet   { type fixedValue; value uniform (0 0 -10); }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    edf_duct { type noSlip; }
}
""")

    write_file("0/p", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      p;
}
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform 0;
boundaryField
{
    inlet   { type zeroGradient; }
    outlet  { type fixedValue; value uniform 0; }
    sides   { type zeroGradient; }
    edf_duct { type zeroGradient; }
}
""")

    write_file("0/k", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      k;
}
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform 1.5;
boundaryField
{
    inlet   { type fixedValue; value uniform 1.5; }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    edf_duct { type kqRWallFunction; value uniform 1.5; }
}
""")

    write_file("0/omega", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      omega;
}
dimensions      [0 0 -1 0 0 0 0];
internalField   uniform 200;
boundaryField
{
    inlet   { type fixedValue; value uniform 200; }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    edf_duct { type omegaWallFunction; value uniform 200; }
}
""")

    write_file("0/nut", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      nut;
}
dimensions      [0 2 -1 0 0 0 0];
internalField   uniform 0;
boundaryField
{
    inlet   { type calculated; value uniform 0; }
    outlet  { type calculated; value uniform 0; }
    sides   { type calculated; value uniform 0; }
    edf_duct { type nutkWallFunction; value uniform 0; }
}
""")

    print("1. Running blockMesh...")
    run_of("blockMesh")
    print("2. Running snappyHexMesh -overwrite...")
    run_of("snappyHexMesh -overwrite")
    print("2b. Creating powered fanDisk cellZone (topoSet)...")
    run_of("topoSet")
    print("3. Writing cell centres (foamPostProcess -func writeCellCentres)...")
    run_of("foamPostProcess -func writeCellCentres -time 0")
    print("4. Running OpenFOAM-12 solver (foamRun)...")
    run_of("foamRun")
    print("5. Reading OpenFOAM 3D fields & rendering PNG...")

    def read_of_field(path, is_vec=False):
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
        idx = next(i for i, ln in enumerate(lines) if ln.strip().isdigit() and i > 10 and lines[i + 1].strip() == "(")
        n = int(lines[idx].strip())
        data = []
        for ln in lines[idx + 2 : idx + 2 + n]:
            s = ln.strip()
            if is_vec:
                vals = [float(x) for x in s[1:-1].split()]
                data.append(vals)
            else:
                data.append(float(s))
        return np.array(data)

    time_dirs = [d for d in os.listdir(CASE_DIR) if d.isdigit() and int(d) > 0]
    latest_t = max(time_dirs, key=int)
    cc = read_of_field(os.path.join(CASE_DIR, "0", "C"), is_vec=True)
    u_vec = read_of_field(os.path.join(CASE_DIR, latest_t, "U"), is_vec=True)
    p_kin = read_of_field(os.path.join(CASE_DIR, latest_t, "p"), is_vec=False)
    k_val = read_of_field(os.path.join(CASE_DIR, latest_t, "k"), is_vec=False)
    u_mag = np.linalg.norm(u_vec, axis=1)
    p_pa = p_kin * 1.225

    r_xy = np.hypot(cc[:, 0], cc[:, 1])
    exit_mask = (np.abs(cc[:, 2] + 0.040) < 0.004) & (r_xy < 0.0185)
    v_exit = float(np.mean(-u_vec[exit_mask, 2])) if np.any(exit_mask) else 0.0
    a_exit = np.pi * (0.0185 ** 2)
    mdot = 1.225 * a_exit * v_exit
    thrust_n = mdot * max(0.0, v_exit - 10.0)

    mask = np.abs(cc[:, 1]) < 0.0045
    xs = cc[mask, 0] * 1000.0
    zs = cc[mask, 2] * 1000.0
    um = u_mag[mask]
    pm = p_pa[mask]
    km = k_val[mask]

    print(f"OPENFOAM_3D_OK | cells={len(u_mag)} | max_|U|={um.max():.2f} m/s | p_range=[{pm.min():.0f}, {pm.max():.0f}] Pa | max_k={km.max():.2f} m2/s2")
    print(f"THRUST_OK | v_exit={v_exit:.2f} m/s | mdot={mdot:.4f} kg/s | net_thrust={thrust_n:.2f} N ({thrust_n * 101.97:.0f} gf)")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.6), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 3D RANS CFD (k-omega SST, {len(u_mag):,} cells) — 50mm EDF Shroud + Nozzle",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    for ax, vals, title, cmap, unit in [
        (ax1, um, f"Velocity Magnitude |U| (Max {um.max():.1f} m/s)", "turbo", "m/s"),
        (ax2, pm, "Static Pressure Gauge (Pa)", "coolwarm", "Pa"),
    ]:
        ax.set_facecolor("#16181d")
        tc = ax.tricontourf(xs, zs, vals, levels=28, cmap=cmap)
        ax.set_title(title, color="#e5e7eb", fontsize=10)
        ax.set_xlabel("Radial X (mm)", color="#e5e7eb")
        ax.set_ylabel("Axial Z (mm)", color="#e5e7eb")
        ax.set_aspect("equal")
        ax.tick_params(colors="#9ca3af")
        cb = fig.colorbar(tc, ax=ax, shrink=0.82)
        cb.set_label(unit, color="#e5e7eb")
        cb.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out_png = os.path.join(BASE_DIR, "fem_results", "openfoam_3d_cfd.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED_OPENFOAM_3D:", out_png)


if __name__ in ("__main__", "run_openfoam_3d_cfd"):
    main()
