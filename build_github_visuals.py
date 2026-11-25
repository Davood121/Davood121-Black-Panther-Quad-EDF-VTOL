"""Generate GitHub hero banner, 3D tilt-rotor animation GIF, and 3D OpenFOAM flow GIF."""

import math
import os
import shutil
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageDraw, ImageFont

try:
    import FreeCAD as App
    import Mesh
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(BASE_DIR, "assets")
TMP_DIR = os.path.join(ASSETS_DIR, "_tmp")


def export_stls_freecad():
    os.makedirs(TMP_DIR, exist_ok=True)
    doc = App.openDocument(os.path.join(BASE_DIR, "black_panther_a2212_quad.FCStd"))

    frame_objs = [
        o for o in doc.Objects
        if o.Name.startswith(("Panther_X_Chassis", "Panther_Battery_Bay", "Panther_Electronics_Bay",
                              "Panther_Top_Cover", "Panther_Landing_Skids", "Panther_Arm_"))
    ]
    Mesh.export(frame_objs, os.path.join(TMP_DIR, "frame.stl"))

    # Export one centered pod (motor + cradle + inner rings) and one centered 8045 prop in local Z-up frame
    pod_doc = App.openDocument(os.path.join(BASE_DIR, "gimbal_a2212_8045.FCStd"))
    pod_parts = []
    for n in ("A2212_1400KV", "A2212_Cradle", "Innermost_Gimbal_Ring", "Inner_Gimbal_Ring"):
        sh = pod_doc.getObject(n).Shape.copy()
        sh.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)
        pod_parts.append(sh)
    prop_sh = pod_doc.getObject("Prop_8045").Shape.copy()
    prop_sh.rotate(App.Vector(0, 0, 0), App.Vector(0, 1, 0), -90.0)

    tmp_doc = App.newDocument("TmpSTL")
    f_pod = tmp_doc.addObject("Part::Feature", "Pod")
    f_pod.Shape = Part.makeCompound(pod_parts)
    f_prop = tmp_doc.addObject("Part::Feature", "Prop")
    f_prop.Shape = prop_sh
    tmp_doc.recompute()
    Mesh.export([f_pod], os.path.join(TMP_DIR, "pod.stl"))
    Mesh.export([f_prop], os.path.join(TMP_DIR, "prop.stl"))
    App.closeDocument(tmp_doc.Name)
    App.closeDocument(pod_doc.Name)
    App.closeDocument(doc.Name)
    print("EXPORTED_ANIM_STLS")


def read_stl(path, max_tris=2200):
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
        else:
            tris_l, cur = [], []
            f.seek(0)
            for ln in f.read().decode("utf-8", errors="ignore").splitlines():
                s = ln.strip()
                if s.startswith("vertex "):
                    cur.append([float(x) for x in s.split()[1:4]])
                    if len(cur) == 3:
                        tris_l.append(cur)
                        cur = []
            tris = np.array(tris_l, dtype=np.float32)
    step = max(1, len(tris) // max_tris)
    return tris[::step]


def rot_x(pts, deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    m = np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)
    return pts @ m.T


def rot_z(pts, deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    m = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    return pts @ m.T


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


def build_tilt_gif(frame_tris, pod_tris, prop_tris):
    pod_pos = [(195.0, 195.0, 15.0), (195.0, -195.0, 15.0), (-195.0, 195.0, 15.0), (-195.0, -195.0, 15.0)]
    frames = []
    n_frames = 36
    for i in range(n_frames):
        ang = 2.0 * math.pi * i / n_frames
        tilt_deg = 22.0 * math.sin(ang * 2.0)
        spin_deg = i * 47.0
        azim = 32.0 + i * (360.0 / n_frames)

        fig = plt.figure(figsize=(8.0, 5.2), dpi=110, facecolor="#0d1117")
        ax = fig.add_subplot(111, projection="3d", facecolor="#0d1117")
        ax.add_collection3d(Poly3DCollection(frame_tris, facecolor="#1f2937", edgecolor="#38bdf8", linewidths=0.12, alpha=0.85))

        for idx, (px, py, pz) in enumerate(pod_pos):
            inward_deg = math.degrees(math.atan2(-py, -px))
            sign = 1.0 if idx % 2 == 0 else -1.0
            p_rot = rot_z(rot_x(pod_tris, tilt_deg * sign), inward_deg) + np.array([px, py, pz], dtype=np.float32)
            pr_rot = rot_z(rot_x(rot_z(prop_tris, spin_deg * sign), tilt_deg * sign), inward_deg) + np.array([px, py, pz], dtype=np.float32)
            ax.add_collection3d(Poly3DCollection(p_rot, facecolor="#f59e0b", edgecolor="#111827", linewidths=0.10, alpha=0.90))
            ax.add_collection3d(Poly3DCollection(pr_rot, facecolor="#38bdf8", edgecolor="#0284c7", linewidths=0.15, alpha=0.85))

            # Thrust vector arrow
            vec = rot_z(rot_x(np.array([[0.0, 0.0, 95.0]], dtype=np.float32), tilt_deg * sign), inward_deg)[0]
            base = np.array([px, py, pz + 45.0])
            ax.quiver(base[0], base[1], base[2], vec[0], vec[1], vec[2], color="#22d3ee", linewidth=2.0, arrow_length_ratio=0.25)

        ax.set_xlim(-285, 285)
        ax.set_ylim(-285, 285)
        ax.set_zlim(-120, 190)
        ax.set_axis_off()
        ax.view_init(elev=24, azim=azim)
        fig.suptitle(
            f"Black Panther 6-DOF Tilt-Rotor  |  A2212/10T + 8045  |  Vectoring: {tilt_deg:+.1f}°",
            color="#f8fafc", fontsize=11.5, fontweight="bold", y=0.95,
        )
        fig.text(0.5, 0.03, "4x 2-Axis MG90S Gimbals  |  186.8mm Prop Tip Clearance  |  OpenFOAM-12 + CalculiX 2.22",
                 ha="center", color="#94a3b8", fontsize=8.5)
        plt.tight_layout(rect=[0, 0.05, 1, 0.92])
        fig.canvas.draw()
        img = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB")
        frames.append(img)
        plt.close(fig)

    out_gif = os.path.join(ASSETS_DIR, "black_panther_tilt_animation.gif")
    frames[0].save(out_gif, save_all=True, append_images=frames[1:], duration=85, loop=0, optimize=True)
    print("RENDERED_GIF:", out_gif)


def build_cfd_gif():
    case_dir = os.path.join(BASE_DIR, "cfd_case_a2212")
    stl_path = os.path.join(case_dir, "constant", "triSurface", "a2212_pod.stl")
    pod_m = read_stl(stl_path, max_tris=1400) * 1000.0
    cc = read_of_field(os.path.join(case_dir, "0", "C"), is_vec=True) * 1000.0
    u_vec = read_of_field(os.path.join(case_dir, "100", "U"), is_vec=True)
    u_mag = np.linalg.norm(u_vec, axis=1)

    hi = (u_mag > 9.0) & (np.abs(cc[:, 1]) < 24.0)
    pts = cc[hi]
    vel = u_mag[hi]

    frames = []
    n_frames = 28
    for i in range(n_frames):
        azim = 20.0 + i * (360.0 / n_frames)
        phase = (i / n_frames) * 40.0
        fig = plt.figure(figsize=(7.6, 5.2), dpi=110, facecolor="#0d1117")
        ax = fig.add_subplot(111, projection="3d", facecolor="#0d1117")
        ax.add_collection3d(Poly3DCollection(pod_m, facecolor="#f59e0b", edgecolor="#0f172a", linewidths=0.12, alpha=0.80))

        # Animated downward-flowing tracer shift inside wake bounds
        z_anim = pts[:, 2] - phase
        z_anim = np.where(z_anim < -220.0, z_anim + 280.0, z_anim)
        sc = ax.scatter(pts[:, 0], pts[:, 1], z_anim, c=vel, cmap="turbo", s=8, alpha=0.72, vmin=6.0, vmax=20.0)
        ax.set_xlim(-140, 140)
        ax.set_ylim(-140, 140)
        ax.set_zlim(-210, 120)
        ax.set_xlabel("X (mm)", color="#94a3b8", fontsize=8)
        ax.set_ylabel("Y (mm)", color="#94a3b8", fontsize=8)
        ax.set_zlabel("Z (mm)", color="#94a3b8", fontsize=8)
        ax.tick_params(colors="#64748b", labelsize=7)
        ax.view_init(elev=18, azim=azim)
        cb = fig.colorbar(sc, ax=ax, shrink=0.72, pad=0.06)
        cb.set_label("|U| (m/s)", color="#e2e8f0", fontsize=8)
        cb.ax.yaxis.set_tick_params(color="#94a3b8", labelcolor="#e2e8f0", labelsize=7)
        fig.suptitle("OpenFOAM-12 3D RANS Downwash — Raised (+30mm) A2212/10T + 8045 Pod",
                     color="#f8fafc", fontsize=11, fontweight="bold", y=0.95)
        plt.tight_layout(rect=[0, 0.02, 1, 0.92])
        fig.canvas.draw()
        img = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB")
        frames.append(img)
        plt.close(fig)

    out_gif = os.path.join(ASSETS_DIR, "openfoam_3d_flow_animation.gif")
    frames[0].save(out_gif, save_all=True, append_images=frames[1:], duration=90, loop=0, optimize=True)
    print("RENDERED_CFD_GIF:", out_gif)


def build_hero_banner(frame_tris, pod_tris, prop_tris):
    fig = plt.figure(figsize=(14.0, 4.4), dpi=130, facecolor="#0d1117")
    ax = fig.add_axes([0.01, 0.04, 0.44, 0.92], projection="3d", facecolor="#0d1117")
    ax.add_collection3d(Poly3DCollection(frame_tris, facecolor="#1f2937", edgecolor="#38bdf8", linewidths=0.12, alpha=0.88))
    for idx, (px, py, pz) in enumerate([(195, 195, 15), (195, -195, 15), (-195, 195, 15), (-195, -195, 15)]):
        inward = math.degrees(math.atan2(-py, -px))
        p_rot = rot_z(pod_tris, inward) + np.array([px, py, pz], dtype=np.float32)
        pr_rot = rot_z(prop_tris, inward) + np.array([px, py, pz], dtype=np.float32)
        ax.add_collection3d(Poly3DCollection(p_rot, facecolor="#f59e0b", edgecolor="#111827", linewidths=0.10, alpha=0.92))
        ax.add_collection3d(Poly3DCollection(pr_rot, facecolor="#38bdf8", edgecolor="#0284c7", linewidths=0.14, alpha=0.86))
    ax.set_xlim(-275, 275)
    ax.set_ylim(-275, 275)
    ax.set_zlim(-110, 175)
    ax.set_axis_off()
    ax.view_init(elev=26, azim=38)

    ax_txt = fig.add_axes([0.46, 0.05, 0.52, 0.90], facecolor="#0d1117")
    ax_txt.axis("off")
    ax_txt.text(0.0, 0.80, "BLACK PANTHER", color="#38bdf8", fontsize=28, fontweight="bold")
    ax_txt.text(0.0, 0.63, "6-DOF Dual-Axis Tilt-Rotor VTOL Quadcopter", color="#f8fafc", fontsize=15, fontweight="bold")
    ax_txt.text(0.0, 0.48, "50mm EDF  ->  A2212/10T 1400KV + 8045 Open-Propeller Trade Study", color="#94a3b8", fontsize=11.5)
    cards = [
        ("18.9x", "Swept Area", "#38bdf8", 0.00),
        ("25-40x", "Lower Disk Load", "#22c55e", 0.25),
        ("±25°", "Vector Clearance", "#f59e0b", 0.50),
        ("12-Act", "4 Motors + 8 Servos", "#a855f7", 0.75),
    ]
    for val, lbl, col, x0 in cards:
        rect = matplotlib.patches.FancyBboxPatch((x0, 0.10), 0.22, 0.26, boxstyle="round,pad=0.02",
                                                 facecolor="#161b22", edgecolor=col, linewidth=1.6)
        ax_txt.add_patch(rect)
        ax_txt.text(x0 + 0.11, 0.26, val, color=col, fontsize=14, fontweight="bold", ha="center", va="center")
        ax_txt.text(x0 + 0.11, 0.16, lbl, color="#cbd5e1", fontsize=8.5, ha="center", va="center")

    out_png = os.path.join(ASSETS_DIR, "hero_banner.png")
    plt.savefig(out_png, facecolor="#0d1117")
    plt.close(fig)
    print("RENDERED_BANNER:", out_png)


def main():
    if HAS_FREECAD:
        export_stls_freecad()
        return

    os.makedirs(ASSETS_DIR, exist_ok=True)
    frame_tris = read_stl(os.path.join(TMP_DIR, "frame.stl"), max_tris=2400)
    pod_tris = read_stl(os.path.join(TMP_DIR, "pod.stl"), max_tris=700)
    prop_tris = read_stl(os.path.join(TMP_DIR, "prop.stl"), max_tris=500)

    build_hero_banner(frame_tris, pod_tris, prop_tris)
    build_tilt_gif(frame_tris, pod_tris, prop_tris)
    build_cfd_gif()
    shutil.rmtree(TMP_DIR, ignore_errors=True)


if __name__ in ("__main__", "build_github_visuals"):
    main()
