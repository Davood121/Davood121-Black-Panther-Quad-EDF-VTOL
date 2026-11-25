"""3D OpenFOAM-12 RANS CFD for A2212/10T 1400KV + 8045 (8x4.5) Propeller Tilt-Gimbal Pod."""

import os
import struct
import subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CASE_DIR = os.path.join(BASE_DIR, "cfd_case_a2212")
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


def read_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n_tri = struct.unpack("<I", f.read(4))[0]
        f.seek(0, os.SEEK_END)
        if f.tell() == 84 + n_tri * 50:
            f.seek(84)
            tris = np.empty((n_tri, 3, 3), dtype=np.float32)
            for i in range(n_tri):
                vals = struct.unpack("<12fH", f.read(50))
                tris[i, 0], tris[i, 1], tris[i, 2] = vals[3:6], vals[6:9], vals[9:12]
            return tris * 1000.0
    tris, cur = [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            s = ln.strip()
            if s.startswith("vertex "):
                cur.append([float(x) * 1000.0 for x in s.split()[1:4]])
                if len(cur) == 3:
                    tris.append(cur)
                    cur = []
    return np.array(tris, dtype=np.float32)


def main():
    stl_path = os.path.join(CASE_DIR, "constant", "triSurface", "a2212_pod.stl")
    if not os.path.exists(stl_path):
        raise FileNotFoundError(f"Missing {stl_path}; run build_a2212_prop_drone.py first.")

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
endTime         100;
deltaT          1;
writeControl    timeStep;
writeInterval   100;
purgeWrite      1;
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
    (-0.30 -0.30 -0.32)
    ( 0.30 -0.30 -0.32)
    ( 0.30  0.30 -0.32)
    (-0.30  0.30 -0.32)
    (-0.30 -0.30  0.22)
    ( 0.30 -0.30  0.22)
    ( 0.30  0.30  0.22)
    (-0.30  0.30  0.22)
);
blocks ( hex (0 1 2 3 4 5 6 7) (26 26 28) simpleGrading (1 1 1) );
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
    a2212_pod { type triSurfaceMesh; file "a2212_pod.stl"; }
}
castellatedMeshControls
{
    maxLocalCells 200000;
    maxGlobalCells 400000;
    minRefinementCells 0;
    maxLoadUnbalance 0.1;
    nCellsBetweenLevels 2;
    features ();
    refinementSurfaces
    {
        a2212_pod { level (2 2); patchInfo { type wall; } }
    }
    resolveFeatureAngle 30;
    refinementRegions {}
    locationInMesh (0.24 0.24 0.18);
    allowFreeStandingZoneFaces true;
}
snapControls
{
    nSmoothPatch 3;
    tolerance 2.0;
    nSolveIter 30;
    nRelaxIter 5;
}
addLayersControls { relativeSizes true; layers {} expansionRatio 1.2; finalLayerThickness 0.3; minThickness 0.1; nGrow 0; featureAngle 60; nRelaxIter 3; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10; maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0; nLayerIter 50; }
meshQualityControls { maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13; minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.02; minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75; }
mergeTolerance 1e-6;
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
        name    prop8045Disk;
        type    cellSet;
        action  new;
        source  cylinderToCell;
        point1  (0 0 0.062);
        point2  (0 0 0.074);
        radius  0.1016;
    }
    {
        name    prop8045Disk;
        type    cellSet;
        action  delete;
        source  cylinderToCell;
        point1  (0 0 0.060);
        point2  (0 0 0.076);
        radius  0.008;
    }
    {
        name    prop8045Disk;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     prop8045Disk;
    }
);
""")

    write_file("constant/fvModels", """FoamFile
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
    cellZone        prop8045Disk;
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

    write_file("0/U", """FoamFile
{
    format      ascii;
    class       volVectorField;
    object      U;
}
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform (0 0 -3);
boundaryField
{
    inlet   { type fixedValue; value uniform (0 0 -3); }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    a2212_pod { type noSlip; }
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
    a2212_pod { type zeroGradient; }
}
""")

    write_file("0/k", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      k;
}
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform 0.5;
boundaryField
{
    inlet   { type fixedValue; value uniform 0.5; }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    a2212_pod { type kqRWallFunction; value uniform 0.5; }
}
""")

    write_file("0/omega", """FoamFile
{
    format      ascii;
    class       volScalarField;
    object      omega;
}
dimensions      [0 0 -1 0 0 0 0];
internalField   uniform 100;
boundaryField
{
    inlet   { type fixedValue; value uniform 100; }
    outlet  { type zeroGradient; }
    sides   { type slip; }
    a2212_pod { type omegaWallFunction; value uniform 100; }
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
    a2212_pod { type nutkWallFunction; value uniform 0; }
}
""")

    print("1. blockMesh...")
    run_of("blockMesh")
    print("2. snappyHexMesh -overwrite...")
    run_of("snappyHexMesh -overwrite")
    print("3. topoSet (8045 prop disk)...")
    run_of("topoSet")
    print("4. writeCellCentres...")
    run_of("foamPostProcess -func writeCellCentres -time 0")
    print("5. foamRun...")
    run_of("foamRun")

    time_dirs = [d for d in os.listdir(CASE_DIR) if d.isdigit() and int(d) > 0]
    latest_t = max(time_dirs, key=int)
    cc = read_of_field(os.path.join(CASE_DIR, "0", "C"), is_vec=True) * 1000.0
    u_vec = read_of_field(os.path.join(CASE_DIR, latest_t, "U"), is_vec=True)
    p_pa = read_of_field(os.path.join(CASE_DIR, latest_t, "p"), is_vec=False) * 1.225
    u_mag = np.linalg.norm(u_vec, axis=1)

    r_xy = np.hypot(cc[:, 0], cc[:, 1])
    wake_m = (np.abs(cc[:, 2] + 80.0) < 12.0) & (r_xy < 95.0)
    v_wake = float(np.mean(-u_vec[wake_m, 2])) if np.any(wake_m) else 0.0
    a_wake = np.pi * (0.095 ** 2)
    mdot = 1.225 * a_wake * v_wake
    thrust_n = mdot * max(0.0, v_wake - 3.0)

    print(f"A2212_CFD_OK | cells={len(u_mag)} | max_|U|={u_mag.max():.2f} m/s | p_range=[{p_pa.min():.0f}, {p_pa.max():.0f}] Pa")
    print(f"A2212_THRUST_OK | v_wake={v_wake:.2f} m/s | mdot={mdot:.3f} kg/s | thrust={thrust_n:.2f} N ({thrust_n * 101.97:.0f} gf)")

    mask = np.abs(cc[:, 1]) < 12.0
    xs, zs = cc[mask, 0], cc[mask, 2]
    um, pm = u_mag[mask], p_pa[mask]
    ux, uz = u_vec[mask, 0], u_vec[mask, 2]

    tri = mtri.Triangulation(xs, zs)
    gx = np.linspace(-270, 270, 100)
    gz = np.linspace(-300, 200, 100)
    GX, GZ = np.meshgrid(gx, gz)
    UX_g = np.ma.filled(mtri.LinearTriInterpolator(tri, ux)(GX, GZ), 0.0)
    UZ_g = np.ma.filled(mtri.LinearTriInterpolator(tri, uz)(GX, GZ), 0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 6.0), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 3D CFD — A2212/10T 1400KV + 8045 Prop ({len(u_mag):,} cells, {thrust_n * 101.97:.0f} gf Thrust)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )

    ax1.set_facecolor("#16181d")
    tc1 = ax1.tricontourf(xs, zs, um, levels=28, cmap="turbo")
    ax1.streamplot(gx, gz, UX_g, UZ_g, color="white", density=1.25, linewidth=0.75, arrowsize=0.9)
    ax1.set_title(f"Downwash Velocity |U| & Streamlines (Max {um.max():.1f} m/s)", color="#e5e7eb", fontsize=10)
    ax1.set_xlabel("Radial X (mm)", color="#e5e7eb")
    ax1.set_ylabel("Axial Z (mm)", color="#e5e7eb")
    ax1.set_aspect("equal")
    ax1.tick_params(colors="#9ca3af")
    cb1 = fig.colorbar(tc1, ax=ax1, shrink=0.84)
    cb1.set_label("m/s", color="#e5e7eb")
    cb1.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    ax2.set_facecolor("#16181d")
    tc2 = ax2.tricontourf(xs, zs, pm, levels=28, cmap="coolwarm")
    ax2.set_title("Static Pressure Jump Across 8045 Rotor Disk (Pa)", color="#e5e7eb", fontsize=10)
    ax2.set_xlabel("Radial X (mm)", color="#e5e7eb")
    ax2.set_ylabel("Axial Z (mm)", color="#e5e7eb")
    ax2.set_aspect("equal")
    ax2.tick_params(colors="#9ca3af")
    cb2 = fig.colorbar(tc2, ax=ax2, shrink=0.84)
    cb2.set_label("Pa", color="#e5e7eb")
    cb2.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out_png = os.path.join(BASE_DIR, "fem_results", "openfoam_a2212_8045_cfd.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out_png)

    # 3D Isometric Pod + Wake Cloud + XY Cross-Section at Z=-40mm
    tris = read_stl(stl_path)
    step = max(1, len(tris) // 3000)
    fig3d = plt.figure(figsize=(13.5, 6.0), dpi=150, facecolor="#16181d")
    fig3d.suptitle(
        f"3D Analysis — Raised (+30mm) A2212/10T + 8045 Pod & Wake ({len(u_mag):,} cells)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    ax3d = fig3d.add_subplot(1, 2, 1, projection="3d", facecolor="#16181d")
    ax3d.add_collection3d(Poly3DCollection(tris[::step], alpha=0.65, facecolor="#f59e0b", edgecolor="#0f172a", linewidths=0.15))
    hi = (u_mag > 12.0) & (np.abs(cc[:, 1]) < 18.0)
    ax3d.scatter(cc[hi, 0], cc[hi, 1], cc[hi, 2], c=u_mag[hi], cmap="turbo", s=5, alpha=0.6)
    ax3d.set_xlim(-120, 120)
    ax3d.set_ylim(-120, 120)
    ax3d.set_zlim(-180, 110)
    ax3d.set_xlabel("X (mm)", color="#e5e7eb")
    ax3d.set_ylabel("Y (mm)", color="#e5e7eb")
    ax3d.set_zlabel("Z (mm)", color="#e5e7eb")
    ax3d.set_title("3D Pod STL & High-Speed Downwash Cloud", color="#e5e7eb", fontsize=10)
    ax3d.tick_params(colors="#9ca3af")
    ax3d.view_init(elev=20, azim=38)

    ax_xy = fig3d.add_subplot(1, 2, 2, facecolor="#16181d")
    m_xy = np.abs(cc[:, 2] + 40.0) < 12.0
    tc_xy = ax_xy.tricontourf(cc[m_xy, 0], cc[m_xy, 1], u_mag[m_xy], levels=26, cmap="turbo")
    ax_xy.set_title(f"XY Wake Slice at Z = -40 mm (Max {u_mag[m_xy].max():.1f} m/s)", color="#e5e7eb", fontsize=10)
    ax_xy.set_xlabel("X (mm)", color="#e5e7eb")
    ax_xy.set_ylabel("Y (mm)", color="#e5e7eb")
    ax_xy.set_aspect("equal")
    ax_xy.tick_params(colors="#9ca3af")
    cb_xy = fig3d.colorbar(tc_xy, ax=ax_xy, shrink=0.84)
    cb_xy.set_label("m/s", color="#e5e7eb")
    cb_xy.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out_3d = os.path.join(BASE_DIR, "fem_results", "openfoam_a2212_3d_analysis.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_3d, facecolor=fig3d.get_facecolor())
    plt.close(fig3d)
    print("RENDERED_3D:", out_3d)


if __name__ == "__main__":
    main()
