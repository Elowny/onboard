#!/usr/bin/env python3
"""Regenerate patent figures with docx-correct filenames and aspect ratios.

Docx embedding map (do NOT swap filenames):
  image1.png -> Figure 1  (6.50 x 3.76 in)
  image2.png -> Figure 5  (5.50 x 7.95 in, portrait)
  image3.png -> Figure 2  (6.50 x 1.96 in, wide banner)
  image4.png -> Figure 3  (4.50 x 3.62 in, bicycle - keep original)
  image5.png -> Figure 4  (6.50 x 2.89 in, wide)
  image6.png -> Figure 6  (6.50 x 2.55 in, wide - keep original)
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

ROOT = Path("/workspace")
OUT = ROOT / "patent_media"
OUT.mkdir(parents=True, exist_ok=True)
ORIG = ROOT / "patent_figures"

# Target pixel sizes chosen to match Word embed aspect ratios.
SIZE = {
    "image1": (1690, 975),    # 1.73
    "image2": (1100, 1590),   # 0.69 portrait - Figure 5
    "image3": (2145, 645),      # 3.33 wide banner - Figure 2
    "image5": (1950, 867),      # 2.25 wide - Figure 4
}

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.titlesize": 13,
    "figure.dpi": 150,
})


def box(ax, xy, w, h, text, fc="#E8F4FD", ec="#2B6CB0", fontsize=11, weight=None):
    x, y = xy
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.06",
        linewidth=1.8, edgecolor=ec, facecolor=fc,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, weight=weight)
    return patch


def arrow(ax, start, end, color="#4A5568", lw=2.0):
    arr = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=14,
                          linewidth=lw, color=color, shrinkA=3, shrinkB=3)
    ax.add_patch(arr)


def diamond(ax, center, w, h, text, fc="#FEF3C7", ec="#D97706", fontsize=10):
    x, y = center
    pts = [(x, y + h / 2), (x + w / 2, y), (x, y - h / 2), (x - w / 2, y)]
    patch = Polygon(pts, closed=True, facecolor=fc, edgecolor=ec, linewidth=1.8)
    ax.add_patch(patch)
    ax.text(x, y, text, ha="center", va="center", fontsize=fontsize)
    return patch


def save(fig, name):
    w, h = SIZE[name.replace(".png", "")]
    fig.set_size_inches(w / 150, h / 150)
    path = OUT / f"{name}.png"
    fig.savefig(path, bbox_inches="tight", pad_inches=0.15, facecolor="white")
    plt.close(fig)
    print("wrote", path, SIZE[name.replace(".png", "")])


def fig1_system():
    w, h = SIZE["image1"]
    fig, ax = plt.subplots(figsize=(w / 150, h / 150))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7.2)
    ax.axis("off")
    ax.text(6, 6.85, "Trajectory Generation System Architecture", ha="center",
            fontsize=15, weight="bold")

    inputs = [
        ("Perception", "Position, velocity,\nheading, bbox length"),
        ("Map", "Drive Passage"),
        ("Lane Selection", "Target L_target"),
        ("Motion History", "Past 1.0 s\n(L_target & accel trend)"),
    ]
    for i, (title, body) in enumerate(inputs):
        y = 5.2 - i * 1.2
        box(ax, (0.2, y), 2.8, 0.95, f"{title}\n{body}", fc="#FEF3C7", ec="#D97706", fontsize=9)

    stages = [
        "Stage 1: Reference Path",
        "Stage 2: Forward Simulation\n(Pole Placement)",
        "Stage 3: Independent\nSpeed Planning",
        "Stage 4: Spatio-Temporal Fusion",
    ]
    for i, text in enumerate(stages):
        y = 5.0 - i * 1.28
        box(ax, (4.3, y), 3.0, 1.0, text, fc="#DBEAFE", ec="#2563EB", fontsize=10, weight="bold")
        if i < 3:
            arrow(ax, (5.8, y), (5.8, y - 0.22), color="#2563EB")
        for j in range(4):
            iy = 5.65 - j * 1.2
            arrow(ax, (3.05, iy + 0.45), (4.25, y + 0.5), color="#D97706", lw=1.2)

    box(ax, (8.5, 1.6), 3.0, 1.4,
        "Predicted Trajectory\n{t,x,y,θ,κ,v,a,s}\n× 80 pts / 8 s",
        fc="#D1FAE5", ec="#059669", fontsize=10)
    arrow(ax, (7.35, 1.0), (8.45, 2.0), color="#059669", lw=2.2)
    save(fig, "image1")


def fig2_reference_banner():
    """Figure 2 -> image3.png (very wide, short)."""
    w, h = SIZE["image3"]
    fig, axes = plt.subplots(1, 3, figsize=(w / 150, h / 150))

    ax = axes[0]
    s = [0, 2, 4, 6, 8, 10]
    l_target = [0.3, 0.55, 0.95, 1.25, 1.05, 0.75]
    ax.fill_between([0, 10], -2.2, 2.2, color="#E5E7EB", alpha=0.5)
    ax.plot([0, 10], [-2.2, -2.2], "k-", lw=2)
    ax.plot([0, 10], [2.2, 2.2], "k-", lw=2)
    ax.plot([0, 10], [0.3, 0.3], "k--", lw=1.2)
    ax.plot(s, l_target, "b--", lw=1.5)
    ax.scatter(s, l_target, c="red", s=45, zorder=5)
    ax.set_title("(a) Sampling", fontsize=11)
    ax.set_xlabel("s")
    ax.set_ylabel("L")
    ax.tick_params(labelsize=9)

    ax = axes[1]
    wx = [0, 2.2, 4.5, 6.8, 9.0, 10.0]
    wy = [0.75, 0.82, 0.95, 1.05, 0.98, 0.88]
    dense_x = [wx[0] + (wx[-1] - wx[0]) * i / 60 for i in range(61)]
    dense_y = []
    for xq in dense_x:
        for j in range(len(wx) - 1):
            if wx[j] <= xq <= wx[j + 1]:
                t = (xq - wx[j]) / (wx[j + 1] - wx[j])
                dense_y.append(wy[j] * (1 - t) + wy[j + 1] * t)
                break
        else:
            dense_y.append(wy[-1])
    ax.scatter(wx, wy, c="red", s=40)
    ax.plot(dense_x, dense_y, color="#2563EB", lw=2)
    ax.set_title("(b) Parametric Fit x(s),y(s)", fontsize=11)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.tick_params(labelsize=9)

    ax = axes[2]
    ax.plot(dense_x, dense_y, color="#2563EB", lw=1.8)
    for i in range(0, len(dense_x), 10):
        j = min(i + 1, len(dense_x) - 1)
        ang = math.atan2(dense_y[j] - dense_y[i], dense_x[j] - dense_x[i])
        ax.arrow(dense_x[i], dense_y[i], 0.25 * math.cos(ang), 0.25 * math.sin(ang),
                 head_width=0.03, head_length=0.04, fc="#EA580C", ec="#EA580C")
    ax.set_title("(c) Dense (x,y,θ,κ)", fontsize=11)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.tick_params(labelsize=9)

    fig.suptitle("Reference Path Construction", fontsize=13, weight="bold", y=1.02)
    fig.subplots_adjust(wspace=0.35, top=0.78, bottom=0.22)
    save(fig, "image3")


def fig4_control_wide():
    """Figure 4 -> image5.png (wide landscape)."""
    w, h = SIZE["image5"]
    fig, ax = plt.subplots(figsize=(w / 150, h / 150))
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 5)
    ax.axis("off")
    ax.text(10, 4.7, "Feedforward + Feedback Pole Placement Control Loop",
            ha="center", fontsize=14, weight="bold")

    box(ax, (0.3, 2.0), 2.6, 1.3, "Reference Path\n(x,y), θ_ref, κ_ref",
        fc="#FED7AA", ec="#EA580C", fontsize=10)
    box(ax, (3.4, 3.1), 2.4, 0.9, "Feedforward\nδ_ff = arctan(κ_ref·L)",
        fc="#BBF7D0", ec="#059669", fontsize=10)
    box(ax, (3.4, 1.0), 2.4, 0.9, "Error Comp.\ne_lat, e_head",
        fc="#FBCFE8", ec="#DB2777", fontsize=10)
    box(ax, (6.3, 1.0), 3.0, 0.9,
        "Feedback\nδ_fb = k_lat·e_lat + k_head·e_head",
        fc="#BBF7D0", ec="#059669", fontsize=10)
    box(ax, (6.3, 0.1), 3.0, 0.75,
        "Pole Map: v→p(v)→z→k_lat,k_head",
        fc="#ECFDF5", ec="#059669", fontsize=9)
    box(ax, (9.8, 1.8), 1.0, 0.8, "Σ\nδ_total", fc="#E0E7FF", ec="#4338CA",
        fontsize=12, weight="bold")
    box(ax, (11.2, 1.5), 2.5, 1.4, "Bicycle\nKinematic Model",
        fc="#DBEAFE", ec="#2563EB", fontsize=11, weight="bold")
    box(ax, (14.2, 1.5), 2.6, 1.4, "Trajectory Pt (k+1)\n{x,y,θ,κ,v,s}",
        fc="#E9D5FF", ec="#7C3AED", fontsize=10)
    box(ax, (17.2, 1.5), 2.3, 1.4, "State\nfeedback",
        fc="#F3E8FF", ec="#7C3AED", fontsize=10)

    arrow(ax, (2.95, 2.7), (3.35, 3.35), color="#EA580C")
    arrow(ax, (2.95, 2.4), (3.35, 1.45), color="#EA580C")
    arrow(ax, (5.85, 3.35), (9.75, 2.35), color="#059669")
    arrow(ax, (5.85, 1.45), (6.25, 1.45), color="#DB2777")
    arrow(ax, (9.35, 1.45), (9.75, 2.05), color="#059669")
    arrow(ax, (10.85, 2.2), (11.15, 2.2), color="#4338CA", lw=2.5)
    arrow(ax, (13.75, 2.2), (14.15, 2.2), color="#7C3AED", lw=2.5)
    arrow(ax, (16.85, 2.2), (17.15, 2.2), color="#7C3AED", lw=2.5)
    arrow(ax, (18.35, 1.45), (4.0, 1.0), color="#7C3AED", lw=1.5)

    save(fig, "image5")


def fig5_forward_portrait():
    """Figure 5 -> image2.png (portrait, tall)."""
    w, h = SIZE["image2"]
    fig, ax = plt.subplots(figsize=(w / 150, h / 150))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 16)
    ax.axis("off")
    ax.text(5, 15.4, "Forward Simulation Loop", ha="center", fontsize=15, weight="bold")
    ax.text(5, 14.9, "at each time step k = 1, 2, …, N", ha="center", fontsize=10)

    y = 13.8
    box(ax, (1.0, y), 8.0, 0.75, "Initialize rear-axle pose",
        fc="#E9D5FF", ec="#7C3AED", fontsize=11)
    arrow(ax, (5, y), (5, y - 0.35))

    y -= 1.2
    diamond(ax, (5, y), 4.2, 0.95, "End of path\nor dev > 10 m?", fontsize=10)
    ax.text(8.0, y, "BREAK", color="red", fontsize=11, weight="bold")
    arrow(ax, (7.15, y), (8.3, y), color="red")

    steps = [
        ("1", "Reference search\n(sliding window)"),
        ("2", "Error: e_lat, e_head"),
        ("3", "Feedforward: δ_ff = arctan(κ_ref·L)"),
        ("4", "Feedback: δ_fb = k_lat·e_lat + k_head·e_head"),
        ("5", "Total: δ_total = δ_ff + δ_fb"),
        ("6", "Bicycle update +\nrecord trajectory point"),
    ]
    colors = ["#DBEAFE", "#FBCFE8", "#BBF7D0", "#BBF7D0", "#DBEAFE", "#E9D5FF"]
    edges = ["#2563EB", "#DB2777", "#059669", "#059669", "#2563EB", "#7C3AED"]
    y -= 1.1
    for i, ((num, text), fc, ec) in enumerate(zip(steps, colors, edges)):
        hbox = 0.95 if i != 3 else 1.05
        box(ax, (1.0, y - hbox), 8.0, hbox, f"Step {num}: {text}",
            fc=fc, ec=ec, fontsize=11)
        if i < len(steps) - 1:
            arrow(ax, (5, y - hbox), (5, y - hbox - 0.28))
        y -= hbox + 0.32

    box(ax, (1.0, 0.35), 8.0, 0.7,
        "OUTPUT → spatial path → speed planning & fusion",
        fc="#D1FAE5", ec="#059669", fontsize=11, weight="bold")
    arrow(ax, (5, y), (5, 1.1))

    ax.annotate("", xy=(0.4, 13.5), xytext=(0.4, 0.8),
                arrowprops=dict(arrowstyle="-|>", color="#7C3AED", lw=2))
    ax.text(0.05, 7.2, "k←k+1", rotation=90, va="center", fontsize=10, color="#7C3AED")

    save(fig, "image2")


def patch_docx_sizes():
  """Increase embed sizes for readability in Word."""
  from docx import Document
  from docx.shared import Inches

  path = ROOT / "IR_Trajectory_Generation_Pole_Placement_revised.docx"
  doc = Document(path)
  # drawing order: 0=img1, 1=img2(Fig5), 2=img3(Fig2), 3=img4(Fig3), 4=img5(Fig4), 5=img6
  sizes = [
      (6.5, 3.8),   # Fig 1
      (6.2, 9.0),   # Fig 5 portrait - larger
      (6.5, 2.0),   # Fig 2 banner
      (4.8, 3.9),   # Fig 3 bicycle
      (6.5, 3.6),   # Fig 4 control - taller than old 2.89
      (6.5, 2.6),   # Fig 6
  ]
  for shape, (w, h) in zip(doc.inline_shapes, sizes):
      shape.width = Inches(w)
      shape.height = Inches(h)
  doc.save(path)
  print("patched docx image sizes")


def main():
    fig1_system()
    fig5_forward_portrait()   # -> image2.png
    fig2_reference_banner()   # -> image3.png
    fig4_control_wide()       # -> image5.png

    # Restore untouched originals
    shutil.copy2(ORIG / "image4.png", OUT / "image4.png")
    shutil.copy2(ORIG / "image6.png", OUT / "image6.png")
    print("kept originals image4.png (Fig 3), image6.png (Fig 6)")

    # Pack into docx
    import zipfile
    docx_path = ROOT / "IR_Trajectory_Generation_Pole_Placement_revised.docx"
    work = Path("/tmp/patent_repack")
    if work.exists():
        shutil.rmtree(work)
    work.mkdir()
    with zipfile.ZipFile(docx_path, "r") as z:
        z.extractall(work)
    media = work / "word" / "media"
    for name in ["image1", "image2", "image3", "image4", "image5", "image6"]:
        shutil.copy2(OUT / f"{name}.png", media / f"{name}.png")
    with zipfile.ZipFile(docx_path, "w", zipfile.ZIP_DEFLATED) as zout:
        for p in sorted(work.rglob("*")):
            if p.is_file():
                zout.write(p, p.relative_to(work))

    patch_docx_sizes()
    shutil.copy2(docx_path, "/opt/cursor/artifacts/IR_Trajectory_Generation_Pole_Placement_revised.docx")
    print("done")


if __name__ == "__main__":
    main()
