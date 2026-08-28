#!/usr/bin/env python3
"""Regenerate patent figures to match revised document text."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

OUT = Path("/workspace/patent_figures_new")
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.titlesize": 12,
    "figure.dpi": 150,
})


def box(ax, xy, w, h, text, fc="#E8F4FD", ec="#2B6CB0", fontsize=9, weight=None):
    x, y = xy
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        linewidth=1.5, edgecolor=ec, facecolor=fc,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, weight=weight, wrap=True)
    return patch


def arrow(ax, start, end, color="#4A5568", style="-|>", lw=1.6):
    arr = FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=12,
                          linewidth=lw, color=color, shrinkA=2, shrinkB=2)
    ax.add_patch(arr)
    return arr


def diamond(ax, center, w, h, text, fc="#FEF3C7", ec="#D97706"):
    x, y = center
    pts = [(x, y + h / 2), (x + w / 2, y), (x, y - h / 2), (x - w / 2, y)]
    patch = Polygon(pts, closed=True, facecolor=fc, edgecolor=ec, linewidth=1.5)
    ax.add_patch(patch)
    ax.text(x, y, text, ha="center", va="center", fontsize=8)
    return patch


def save(fig, name):
    path = OUT / name
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


def fig1_system_architecture():
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 8)
    ax.axis("off")
    ax.text(6, 7.6, "Trajectory Generation System Architecture", ha="center",
            fontsize=14, weight="bold")

    inputs = [
        ("Perception System", "Position (x, y)\nVelocity (vx, vy)\nHeading θ\nBBox length"),
        ("Map System", "Drive Passage\n(lane geometry,\nboundaries, type)"),
        ("Lane Selection\n(APF)", "Target lateral offset\nL_target"),
        ("Motion History", "Past 1.0 s trajectory\n(for L_target &\naccel trend)"),
    ]
    for i, (title, body) in enumerate(inputs):
        y = 5.8 - i * 1.35
        box(ax, (0.3, y), 3.0, 1.05, f"{title}\n{body}", fc="#FEF3C7", ec="#D97706", fontsize=8)

    stages = [
        "Stage 1:\nReference Path\nConstruction",
        "Stage 2:\nForward Simulation\n(Pole Placement Control)",
        "Stage 3:\nIndependent\nSpeed Planning",
        "Stage 4:\nSpatio-Temporal\nFusion",
    ]
    for i, text in enumerate(stages):
        y = 5.5 - i * 1.45
        box(ax, (4.6, y), 3.2, 1.15, text, fc="#DBEAFE", ec="#2563EB", fontsize=9, weight="bold")
        if i < 3:
            arrow(ax, (6.2, y), (6.2, y - 0.28), color="#2563EB")
        for j in range(4):
            iy = 6.25 - j * 1.35
            arrow(ax, (3.35, iy + 0.5), (4.55, y + 0.55), color="#D97706", style="-|>", lw=1.0)

    box(ax, (8.8, 2.0), 2.8, 1.5,
        "Predicted Trajectory\n\nPer point (0.1 s):\n{t, x, y, θ, κ, v, a, s}\n× 80 points (8 s)",
        fc="#D1FAE5", ec="#059669", fontsize=8)
    arrow(ax, (7.85, 1.05), (8.75, 2.75), color="#059669", lw=2)

    ax.text(1.8, 7.1, "INPUTS", fontsize=11, weight="bold", color="#D97706")
    ax.text(6.2, 7.1, "PROCESSING STAGES", fontsize=11, weight="bold", color="#2563EB")
    ax.text(10.2, 7.1, "OUTPUT", fontsize=11, weight="bold", color="#059669")
    save(fig, "image1.png")


def fig2_reference_path():
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))

    # (a) Frenet sampling
    ax = axes[0]
    s = [0, 2, 4, 6, 8, 10]
    l_target = [0.3, 0.55, 0.95, 1.25, 1.05, 0.75]
    ax.fill_between([0, 10], -2.5, 2.5, color="#E5E7EB", alpha=0.5)
    ax.plot([0, 10], [-2.5, -2.5], "k-", lw=2)
    ax.plot([0, 10], [2.5, 2.5], "k-", lw=2)
    ax.plot([0, 10], [0.3, 0.3], "k--", lw=1.2, label="lane center")
    ax.plot(s, l_target, "b--", lw=1.5, label="target L offset")
    ax.scatter(s, l_target, c="red", s=60, zorder=5, label="sampled waypoints")
    for si, li, ti in zip(s, l_target, [0.3] * len(s)):
        ax.plot([si, si], [ti, li], "r--", lw=0.8, alpha=0.7)
    ax.annotate("L_target", xy=(6, 1.25), xytext=(6.3, 2.0), color="red", fontsize=8,
                arrowprops=dict(arrowstyle="->", color="red", lw=0.8))
    ax.set_title("(a) Drive Passage Sampling")
    ax.set_xlabel("s (longitudinal)")
    ax.set_ylabel("L (lateral)")
    ax.set_xlim(-0.2, 10.5)
    ax.set_ylim(-3, 3)
    ax.legend(fontsize=7, loc="lower right")

    # (b) single parametric fit
    ax = axes[1]
    wx = [0, 2.2, 4.5, 6.8, 9.0, 10.0]
    wy = [0.75, 0.82, 0.95, 1.05, 0.98, 0.88]
    dense_x = [wx[0] + (wx[-1] - wx[0]) * i / 80 for i in range(81)]
    dense_y = []
    for xq in dense_x:
        # piecewise linear interpolation through waypoints
        for j in range(len(wx) - 1):
            if wx[j] <= xq <= wx[j + 1]:
                t = (xq - wx[j]) / (wx[j + 1] - wx[j])
                dense_y.append(wy[j] * (1 - t) + wy[j + 1] * t)
                break
        else:
            dense_y.append(wy[-1])
    ax.scatter(wx, wy, c="red", s=55, zorder=5, label="waypoints")
    ax.plot(dense_x, dense_y, color="#2563EB", lw=2.2, label="fitted curve x(s), y(s)")
    ax.set_title("(b) Parametric Curve Fitting")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_xlim(-0.2, 10.5)
    ax.legend(fontsize=7, loc="upper left")

    # (c) dense reference
    ax = axes[2]
    ax.plot(dense_x, dense_y, color="#2563EB", lw=1.5)
    step = 8
    for i in range(0, len(dense_x), step):
        j = min(i + 1, len(dense_x) - 1)
        dx = dense_x[j] - dense_x[i]
        dy = dense_y[j] - dense_y[i]
        ang = math.atan2(dy, dx)
        ax.arrow(dense_x[i], dense_y[i], 0.35 * math.cos(ang), 0.35 * math.sin(ang),
                 head_width=0.04, head_length=0.05, fc="#EA580C", ec="#EA580C", lw=0.8)
    ax.scatter(dense_x, dense_y, c="#2563EB", s=8, alpha=0.6)
    ax.text(7.0, 0.72, "Each point:\n(x, y, θ, κ)", fontsize=8,
            bbox=dict(boxstyle="round", fc="#FEF3C7", ec="#D97706"))
    ax.set_title("(c) Dense Reference Points")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    fig.suptitle("Reference Path Construction", fontsize=13, weight="bold", y=1.02)
    fig.tight_layout()
    save(fig, "image2.png")


def fig4_control_loop():
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.text(5.5, 5.7, "Feedforward + Feedback Pole Placement Control Loop",
            ha="center", fontsize=13, weight="bold")

    box(ax, (0.4, 3.8), 2.3, 1.2,
        "Reference Path\npos (x,y), θ_ref, κ_ref",
        fc="#FED7AA", ec="#EA580C", fontsize=8)

    box(ax, (3.3, 4.7), 2.0, 0.9, "Feedforward\nδ_ff = arctan(κ_ref·L)",
        fc="#BBF7D0", ec="#059669", fontsize=8)
    box(ax, (3.3, 2.3), 2.0, 1.0, "Error Computation\ne_lat, e_head",
        fc="#FBCFE8", ec="#DB2777", fontsize=8)
    box(ax, (6.0, 2.3), 2.2, 1.0,
        "Feedback (Pole Placement)\nδ_fb = k_lat·e_lat + k_head·e_head",
        fc="#BBF7D0", ec="#059669", fontsize=8)

    box(ax, (6.0, 0.5), 2.2, 1.0,
        "Pole Mapping\nv → p(v) → z = exp(p·Ts)\n→ k_lat, k_head",
        fc="#ECFDF5", ec="#059669", fontsize=7)

    box(ax, (3.3, 0.7), 1.2, 0.7, "Σ", fc="#E0E7FF", ec="#4338CA", fontsize=14, weight="bold")
    ax.text(3.55, 0.72, "δ_total", fontsize=7, ha="left")

    box(ax, (5.0, 0.55), 0.8, 0.55, "δ_ff", fc="white", ec="#94A3B8", fontsize=7)
    box(ax, (5.0, 1.25), 0.8, 0.55, "δ_fb", fc="white", ec="#94A3B8", fontsize=7)

    box(ax, (8.7, 1.8), 2.0, 1.3,
        "Bicycle Kinematic\nModel",
        fc="#DBEAFE", ec="#2563EB", fontsize=9, weight="bold")
    box(ax, (8.7, 0.3), 2.0, 1.0,
        "Trajectory Point (k+1)\nstore {t,x,y,θ,κ,v,s}",
        fc="#E9D5FF", ec="#7C3AED", fontsize=8)

    arrow(ax, (2.75, 4.5), (3.25, 4.95), color="#EA580C")
    arrow(ax, (2.75, 4.2), (3.25, 2.85), color="#EA580C")
    arrow(ax, (5.35, 4.95), (5.35, 1.35), color="#059669")
    arrow(ax, (5.35, 2.75), (5.95, 2.75), color="#DB2777")
    arrow(ax, (7.0, 1.5), (7.0, 1.25), color="#059669")
    arrow(ax, (7.0, 0.5), (7.0, 0.45), color="#059669")
    arrow(ax, (4.55, 1.05), (4.95, 1.05), color="#4338CA")
    arrow(ax, (5.85, 1.05), (8.65, 2.2), color="#2563EB", lw=2)
    arrow(ax, (9.7, 1.75), (9.7, 1.35), color="#7C3AED")
    arrow(ax, (9.7, 0.25), (4.2, 2.2), color="#7C3AED", style="-|>", lw=1.2)
    ax.text(6.8, 0.15, "vehicle state feedback", fontsize=7, color="#7C3AED")

    save(fig, "image4.png")


def fig5_forward_loop():
    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.set_xlim(0, 8.5)
    ax.set_ylim(0, 13)
    ax.axis("off")
    ax.text(4.25, 12.5, "Forward Simulation Loop", ha="center", fontsize=14, weight="bold")
    ax.text(4.25, 12.0, "(executed at each time step k = 1, 2, …, N)", ha="center", fontsize=9)

    box(ax, (1.0, 10.7), 6.5, 0.9,
        "Initialize: rear-axle pose from geometric center",
        fc="#E9D5FF", ec="#7C3AED", fontsize=8)

    steps = [
        ("Step 1", "Reference point search\n(sliding window, 20 pts ahead)"),
        ("Step 2", "Error computation\ne_lat, e_head"),
        ("Step 3", "Feedforward steering\nδ_ff = arctan(κ_ref · L)"),
        ("Step 4", "Feedback steering\nδ_fb = k_lat·e_lat + k_head·e_head\n(pole: p = −PLF(v) → z = exp(p·Ts))"),
        ("Step 5", "Total steering\nδ_total = δ_ff + δ_fb"),
        ("Step 6", "Bicycle model update\n+ record trajectory point\n{x,y,θ,κ,v,s} at geometric center"),
    ]

    y = 9.8
    arrow(ax, (4.25, 10.65), (4.25, y + 0.95))
    diamond(ax, (4.25, y - 0.2), 3.6, 1.0, "End of path\nOR dev > 10 m?", fc="#FEF3C7", ec="#D97706")
    ax.text(7.5, y - 0.2, "BREAK", color="red", fontsize=9, weight="bold")
    arrow(ax, (6.05, y - 0.2), (7.2, y - 0.2), color="red")

    y -= 1.5
    colors = ["#DBEAFE", "#FBCFE8", "#BBF7D0", "#BBF7D0", "#DBEAFE", "#E9D5FF"]
    edges = ["#2563EB", "#DB2777", "#059669", "#059669", "#2563EB", "#7C3AED"]
    heights = [0.95, 0.85, 0.85, 1.15, 0.75, 1.05]
    for i, ((label, text), h, fc, ec) in enumerate(zip(steps, heights, colors, edges)):
        box(ax, (1.0, y - h), 6.5, h, f"{label}: {text}", fc=fc, ec=ec, fontsize=8.5)
        if i < len(steps) - 1:
            arrow(ax, (4.25, y - h), (4.25, y - h - 0.25))
        y -= h + 0.35

    box(ax, (1.0, 0.35), 6.5, 0.75,
        "OUTPUT: spatial path  →  independent speed planning & fusion",
        fc="#D1FAE5", ec="#059669", fontsize=8.5, weight="bold")
    arrow(ax, (4.25, y), (4.25, 1.15))

    # loop-back arrow on left
    ax.annotate("", xy=(0.55, 9.5), xytext=(0.55, 1.0),
                arrowprops=dict(arrowstyle="-|>", color="#7C3AED", lw=1.5))
    ax.text(0.15, 5.2, "k = k + 1", rotation=90, va="center", fontsize=8, color="#7C3AED")

    save(fig, "image5.png")


def main():
    fig1_system_architecture()
    fig2_reference_path()
    fig4_control_loop()
    fig5_forward_loop()
    print("Done.")


if __name__ == "__main__":
    main()
