"""Render high-resolution 3D FEM Von Mises Stress & Displacement PNGs from solved *_fem.FCStd files."""

import glob
import os
import FreeCAD as App
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FEM_DIR = os.path.join(SCRIPT_DIR, "fem_results")


def _set_equal_3d(ax, xs, ys, zs):
    max_range = max(xs.ptp(), ys.ptp(), zs.ptp(), 1e-3) / 2.0
    mid_x = (xs.max() + xs.min()) / 2.0
    mid_y = (ys.max() + ys.min()) / 2.0
    mid_z = (zs.max() + zs.min()) / 2.0
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def render_part_png(fcstd_path):
    doc = App.openDocument(fcstd_path)
    part_name = os.path.basename(fcstd_path).replace("_fem.FCStd", "")
    res = None
    mesh_obj = None
    for obj in doc.Objects:
        if obj.isDerivedFrom("Fem::FemResultObject"):
            res = obj
        elif obj.isDerivedFrom("Fem::FemMeshObject"):
            mesh_obj = obj

    if res is None or mesh_obj is None:
        App.closeDocument(doc.Name)
        return None

    nodes = mesh_obj.FemMesh.Nodes
    node_ids = res.NodeNumbers if res.NodeNumbers else sorted(nodes.keys())
    pts = np.array([[nodes[nid].x, nodes[nid].y, nodes[nid].z] for nid in node_ids])
    vm = np.array(res.vonMises)
    disp = np.array(res.DisplacementLengths)

    # Subsample slightly if huge mesh (>12k nodes) so scatter renders crisply and fast
    step = max(1, len(pts) // 8000)
    pts_s = pts[::step]
    vm_s = vm[::step]
    disp_s = disp[::step]

    xs, ys, zs = pts_s[:, 0], pts_s[:, 1], pts_s[:, 2]

    fig = plt.figure(figsize=(13, 5.5), dpi=150, facecolor="#16181d")
    fig.suptitle(
        f"{part_name} — CalculiX FEM Analysis  |  "
        f"Max Von Mises: {vm.max():.3f} MPa  |  Max Disp: {disp.max():.4f} mm",
        color="white",
        fontsize=12,
        fontweight="bold",
        y=0.97,
    )

    for idx, (vals, title, cmap_name, unit) in enumerate([
        (vm_s, "Von Mises Stress (MPa)", "turbo", "MPa"),
        (disp_s, "Total Displacement (mm)", "viridis", "mm"),
    ], start=1):
        ax = fig.add_subplot(1, 2, idx, projection="3d", facecolor="#16181d")
        sc = ax.scatter(xs, ys, zs, c=vals, cmap=cmap_name, s=8, alpha=0.92, depthshade=True)
        _set_equal_3d(ax, xs, ys, zs)
        ax.set_title(title, color="#e5e7eb", fontsize=10, pad=8)
        ax.tick_params(colors="#9ca3af", labelsize=7)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor("#2d313b")
        cb = fig.colorbar(sc, ax=ax, shrink=0.72, pad=0.08)
        cb.ax.yaxis.set_tick_params(color="#9ca3af", labelcolor="#e5e7eb", labelsize=8)
        cb.set_label(unit, color="#e5e7eb", fontsize=9)

    out_png = os.path.join(FEM_DIR, f"{part_name}_fem.png")
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    App.closeDocument(doc.Name)
    print(f"Rendered: {out_png}")
    return out_png


def main():
    files = sorted(glob.glob(os.path.join(FEM_DIR, "*_fem.FCStd")))
    for f in files:
        render_part_png(f)


if __name__ in ("__main__", "render_fem_images"):
    main()
