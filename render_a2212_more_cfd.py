"""Generate turbulence + axial XY slice OpenFOAM-12 FVM plots for A2212/10T + 8045."""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CASE_DIR = os.path.join(BASE_DIR, "cfd_case_a2212")
OUT_DIR = os.path.join(BASE_DIR, "fem_results")


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


def main():
    t_dirs = [d for d in os.listdir(CASE_DIR) if d.isdigit() and int(d) > 0]
    latest_t = max(t_dirs, key=int)
    t_dir = os.path.join(CASE_DIR, latest_t)

    cc = read_of_field(os.path.join(CASE_DIR, "0", "C"), is_vec=True) * 1000.0
    u_vec = read_of_field(os.path.join(t_dir, "U"), is_vec=True)
    p_pa = read_of_field(os.path.join(t_dir, "p"), is_vec=False) * 1.225
    k_val = read_of_field(os.path.join(t_dir, "k"), is_vec=False)
    nut_val = read_of_field(os.path.join(t_dir, "nut"), is_vec=False)
    u_mag = np.linalg.norm(u_vec, axis=1)

    # 1. Turbulence Kinetic Energy k + Eddy Viscosity Ratio nu_t/nu
    mask_xz = np.abs(cc[:, 1]) < 14.0
    xs, zs = cc[mask_xz, 0], cc[mask_xz, 2]
    km = k_val[mask_xz]
    nut_r = nut_val[mask_xz] / 1.5e-5

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.8), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"OpenFOAM-12 FVM — A2212/10T + 8045 Turbulence & Eddy Viscosity (t={latest_t}, {len(u_mag):,} cells)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    for ax, vals, title, cmap, unit in [
        (ax1, km, f"Turbulent Kinetic Energy k (Max {km.max():.2f} m2/s2)", "inferno", "m2/s2"),
        (ax2, nut_r, f"Turbulent Viscosity Ratio nu_t / nu (Max {nut_r.max():.0f})", "viridis", "nu_t/nu"),
    ]:
        ax.set_facecolor("#16181d")
        tc = ax.tricontourf(xs, zs, vals, levels=26, cmap=cmap)
        ax.set_title(title, color="#e5e7eb", fontsize=10)
        ax.set_xlabel("Radial X (mm)", color="#e5e7eb")
        ax.set_ylabel("Axial Z (mm)", color="#e5e7eb")
        ax.set_aspect("equal")
        ax.tick_params(colors="#9ca3af")
        cb = fig.colorbar(tc, ax=ax, shrink=0.84)
        cb.set_label(unit, color="#e5e7eb")
        cb.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out1 = os.path.join(OUT_DIR, "openfoam_a2212_turbulence.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out1, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out1)

    # 2. 4 Axial XY Cross-Sections
    stations = [
        (90.0, "Z = +90 mm (Inflow Above 8045)"),
        (65.0, "Z = +65 mm (Rotor Disk Plane)"),
        (10.0, "Z = +10 mm (Gimbal Ring Plane)"),
        (-80.0, "Z = -80 mm (Downstream Wake)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 10.0), dpi=150, facecolor="#16181d")
    fig.suptitle(
        "OpenFOAM-12 FVM — A2212/10T + 8045 Axial Cross-Sections (|U|)",
        color="white", fontsize=12, fontweight="bold", y=0.97,
    )
    vmax = float(np.percentile(u_mag, 99.5))
    for ax, (z0, title) in zip(axes.flat, stations):
        m = np.abs(cc[:, 2] - z0) < 14.0
        ax.set_facecolor("#16181d")
        tc = ax.tricontourf(cc[m, 0], cc[m, 1], u_mag[m], levels=np.linspace(0.0, vmax, 26), cmap="turbo", extend="max")
        ax.set_title(f"{title} | Max {u_mag[m].max():.1f} m/s", color="#e5e7eb", fontsize=10)
        ax.set_xlabel("X (mm)", color="#e5e7eb")
        ax.set_ylabel("Y (mm)", color="#e5e7eb")
        ax.set_aspect("equal")
        ax.tick_params(colors="#9ca3af")
        cb = fig.colorbar(tc, ax=ax, shrink=0.82)
        cb.set_label("m/s", color="#e5e7eb")
        cb.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb")

    out2 = os.path.join(OUT_DIR, "openfoam_a2212_axial_slices.png")
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(out2, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("RENDERED:", out2)


if __name__ == "__main__":
    main()
