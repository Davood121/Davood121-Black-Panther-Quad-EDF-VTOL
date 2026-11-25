"""Generate additional 3D OpenFOAM-12 CFD visualizations from converged cfd_case/."""

import os
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CASE_DIR = os.path.join(BASE_DIR, "cfd_case")
OUT_DIR = os.path.join(BASE_DIR, "fem_results")


def read_of_field(path, is_vec=False):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    idx = next(
        i for i, ln in enumerate(lines)
        if ln.strip().isdigit() and i > 10 and lines[i + 1].strip() == "("
    )
    n = int(lines[idx].strip())
    data = []
    for ln in lines[idx + 2 : idx + 2 + n]:
        s = ln.strip()
        if is_vec:
            data.append([float(x) for x in s[1:-1].split()])
        else:
            data.append(float(s))
    return np.array(data)


def read_stl(path):
    """Read binary or ASCII STL triangles as (N, 3, 3) array in mm."""
    with open(path, "rb") as f:
        header = f.read(80)
        count_bytes = f.read(4)
        n_tri = struct.unpack("<I", count_bytes)[0]
        expected = 84 + n_tri * 50
        f.seek(0, os.SEEK_END)
        size = f.tell()
        if size == expected:
            f.seek(84)
            tris = np.empty((n_tri, 3, 3), dtype=np.float32)
            for i in range(n_tri):
                rec = f.read(50)
                vals = struct.unpack("<12fH", rec)
                tris[i, 0] = vals[3:6]
                tris[i, 1] = vals[6:9]
                tris[i, 2] = vals[9:12]
            return tris * 1000.0
    # Fallback ASCII STL
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
    time_dirs = [d for d in os.listdir(CASE_DIR) if d.isdigit() and int(d) > 0]
    latest_t = max(time_dirs, key=int)
    t_dir = os.path.join(CASE_DIR, latest_t)

    cc = read_of_field(os.path.join(CASE_DIR, "0", "C"), is_vec=True) * 1000.0  # mm
    u_vec = read_of_field(os.path.join(t_dir, "U"), is_vec=True)
    p_pa = read_of_field(os.path.join(t_dir, "p"), is_vec=False) * 1.225
    k_val = read_of_field(os.path.join(t_dir, "k"), is_vec=False)
    nut_val = read_of_field(os.path.join(t_dir, "nut"), is_vec=False)
    u_mag = np.linalg.norm(u_vec, axis=1)

    # 1. Streamlines + Turbulence (XZ centerplane |Y| < 4.5 mm)
    mask_xz = np.abs(cc[:, 1]) < 4.5
    xs, zs = cc[mask_xz, 0], cc[mask_xz, 2]
    ux, uz = u_vec[mask_xz, 0], u_vec[mask_xz, 2]
    um, km = u_mag[mask_xz], k_val[mask_xz]
    nut_ratio = nut_val[mask_xz] / 1.5e-5

    tri = mtri.Triangulation(xs, zs)
    gx = np.linspace(-70, 70, 90)
    gz = np.linspace(-115, 85, 110)
    GX, GZ = np.meshgrid(gx, gz)
    interp_ux = mtri.LinearTriInterpolator(tri, ux)
    interp_uz = mtri.LinearTriInterpolator(tri, uz)
    UX_g = np.ma.filled(interp_ux(GX, GZ), 0.0)
    UZ_g = np.ma.filled(interp_uz(GX, GZ), 0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 6.0), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 3D CFD — Streamlines & Turbulence (t={latest_t}, {len(u_mag):,} cells)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )

    ax1.set_facecolor("#16181d")
    tc1 = ax1.tricontourf(xs, zs, um, levels=28, cmap="turbo")
    ax1.streamplot(gx, gz, UX_g, UZ_g, color="white", density=1.35, linewidth=0.75, arrowsize=0.9)
    ax1.set_title("Velocity |U| & Airflow Streamlines (Y=0 Plane)", color="#e5e7eb", fontsize=10)
    ax1.set_xlabel("Radial X (mm)", color="#e5e7eb")
    ax1.set_ylabel("Axial Z (mm)", color="#e5e7eb")
    ax1.set_aspect("equal")
    ax1.tick_params(colors="#9ca3af")
    cb1 = fig.colorbar(tc1, ax=ax1, shrink=0.84)
    cb1.set_label("m/s", color="#e5e7eb")
    cb1.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    ax2.set_facecolor("#16181d")
    tc2 = ax2.tricontourf(xs, zs, km, levels=28, cmap="inferno")
    ax2.tricontour(xs, zs, nut_ratio, levels=6, colors="cyan", linewidths=0.6, alpha=0.65)
    ax2.set_title(f"Turbulent Kinetic Energy k (Max {km.max():.1f} m2/s2) + nu_t/nu", color="#e5e7eb", fontsize=10)
    ax2.set_xlabel("Radial X (mm)", color="#e5e7eb")
    ax2.set_ylabel("Axial Z (mm)", color="#e5e7eb")
    ax2.set_aspect("equal")
    ax2.tick_params(colors="#9ca3af")
    cb2 = fig.colorbar(tc2, ax=ax2, shrink=0.84)
    cb2.set_label("k (m2/s2)", color="#e5e7eb")
    cb2.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out1 = os.path.join(OUT_DIR, "openfoam_streamlines_turbulence.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out1, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out1)

    # 2. 4 Axial XY Cross-Sections
    stations = [
        (35.0, "Z = +35 mm (Intake Lip Entry)"),
        (10.0, "Z = +10 mm (Rotor / Hub Plane)"),
        (-25.0, "Z = -25 mm (Nozzle Contraction)"),
        (-60.0, "Z = -60 mm (Exhaust Jet Plume)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 10.0), dpi=150, facecolor="#16181d")
    fig.suptitle(
        "OpenFOAM-12 3D CFD — Axial Cross-Sections (Velocity Magnitude |U|)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    vmin, vmax = 0.0, float(np.percentile(u_mag, 99.5))
    for ax, (z0, title) in zip(axes.flat, stations):
        m = np.abs(cc[:, 2] - z0) < 4.5
        ax.set_facecolor("#16181d")
        tc = ax.tricontourf(cc[m, 0], cc[m, 1], u_mag[m], levels=np.linspace(vmin, vmax, 26), cmap="turbo", extend="max")
        ax.set_title(f"{title} | Max {u_mag[m].max():.1f} m/s", color="#e5e7eb", fontsize=10)
        ax.set_xlabel("X (mm)", color="#e5e7eb")
        ax.set_ylabel("Y (mm)", color="#e5e7eb")
        ax.set_aspect("equal")
        ax.tick_params(colors="#9ca3af")
        cb = fig.colorbar(tc, ax=ax, shrink=0.82)
        cb.set_label("m/s", color="#e5e7eb")
        cb.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out2 = os.path.join(OUT_DIR, "openfoam_axial_slices.png")
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(out2, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out2)

    # 3. 3D STL Geometry + Axial Profiles
    stl_path = os.path.join(CASE_DIR, "constant", "triSurface", "edf_duct.stl")
    tris = read_stl(stl_path)
    step = max(1, len(tris) // 3500)
    tris_sub = tris[::step]

    fig = plt.figure(figsize=(13.5, 6.0), dpi=150, facecolor="#16181d")
    fig.suptitle(
        "Black Panther 50mm EDF — 3D Shroud/Nozzle Mesh & Axial Flow Profiles",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )

    ax3d = fig.add_subplot(1, 2, 1, projection="3d", facecolor="#16181d")
    poly = Poly3DCollection(tris_sub, alpha=0.55, facecolor="#38bdf8", edgecolor="#0f172a", linewidths=0.15)
    ax3d.add_collection3d(poly)
    # Overlay high-velocity core scatter points (|U| > 36 m/s)
    hi = (u_mag > 36.0) & (np.abs(cc[:, 1]) < 6.0)
    sc = ax3d.scatter(cc[hi, 0], cc[hi, 1], cc[hi, 2], c=u_mag[hi], cmap="turbo", s=4, alpha=0.65)
    ax3d.set_xlim(-45, 45)
    ax3d.set_ylim(-45, 45)
    ax3d.set_zlim(-75, 55)
    ax3d.set_xlabel("X (mm)", color="#e5e7eb")
    ax3d.set_ylabel("Y (mm)", color="#e5e7eb")
    ax3d.set_zlabel("Z (mm)", color="#e5e7eb")
    ax3d.set_title("3D EDF Shroud + Nozzle STL & High-Speed Core", color="#e5e7eb", fontsize=10)
    ax3d.tick_params(colors="#9ca3af")
    ax3d.view_init(elev=22, azim=38)

    ax_prof = fig.add_subplot(1, 2, 2, facecolor="#1e222a")
    r_xy = np.hypot(cc[:, 0], cc[:, 1])
    z_bins = np.linspace(-110, 80, 38)
    z_mid = 0.5 * (z_bins[:-1] + z_bins[1:])
    u_core, p_core, u_lip = [], [], []
    for z0, z1 in zip(z_bins[:-1], z_bins[1:]):
        m_c = (cc[:, 2] >= z0) & (cc[:, 2] < z1) & (r_xy < 8.0)
        m_l = (cc[:, 2] >= z0) & (cc[:, 2] < z1) & (r_xy >= 20.0) & (r_xy <= 30.0)
        u_core.append(np.mean(u_mag[m_c]) if np.any(m_c) else np.nan)
        p_core.append(np.mean(p_pa[m_c]) if np.any(m_c) else np.nan)
        u_lip.append(np.mean(u_mag[m_l]) if np.any(m_l) else np.nan)

    l1, = ax_prof.plot(z_mid, u_core, "-o", color="#38bdf8", ms=3.5, lw=2, label="Centerline |U| (r < 8 mm)")
    l2, = ax_prof.plot(z_mid, u_lip, "--s", color="#f59e0b", ms=3.0, lw=1.8, label="Annulus / Lip |U| (r = 20–30 mm)")
    ax_prof.set_xlabel("Axial Coordinate Z (mm)  [+Z Intake -> -Z Exhaust]", color="#e5e7eb")
    ax_prof.set_ylabel("Velocity |U| (m/s)", color="#38bdf8")
    ax_prof.tick_params(colors="#9ca3af")
    ax_prof.grid(True, color="#334155", alpha=0.5)
    ax_prof.set_title("Axial Velocity & Static Pressure Profiles", color="#e5e7eb", fontsize=10)

    ax_p2 = ax_prof.twinx()
    l3, = ax_p2.plot(z_mid, p_core, "-^", color="#f43f5e", ms=3.0, lw=1.8, label="Centerline Pressure (Pa)")
    ax_p2.set_ylabel("Static Pressure Gauge (Pa)", color="#f43f5e")
    ax_p2.tick_params(axis="y", colors="#f43f5e")

    lines = [l1, l2, l3]
    ax_prof.legend(lines, [l.get_label() for l in lines], loc="lower left", facecolor="#16181d", edgecolor="#334155", labelcolor="#e5e7eb", fontsize=8.5)

    out3 = os.path.join(OUT_DIR, "openfoam_3d_duct_and_profiles.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out3, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out3)


if __name__ == "__main__":
    main()
