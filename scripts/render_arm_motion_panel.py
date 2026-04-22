import argparse
import csv
import gc
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
from PIL import Image


LINK_LENGTH_CELLS = 10.0
TWO_PI = 2.0 * math.pi
LINK_COLORS = [
    "#2457C5",
    "#13A2A6",
    "#20A260",
    "#E4A11B",
    "#D54A77",
    "#7D55C7",
    "#555555",
]
OBSTACLE_COLOR = "#111417"
OBSTACLE_EDGE = "#3B424A"
FREE_COLOR = "#FDFEFE"
GRID_COLOR = "#D8DEE6"


def load_solution(solution_path: Path) -> tuple[Path, np.ndarray]:
    with solution_path.open() as f:
        map_path = Path(f.readline().strip())
        rows = []
        for line in f:
            vals = [float(v) for v in line.strip().split(",") if v]
            if vals:
                rows.append(vals)
    if not rows:
        raise RuntimeError(f"No path states found in {solution_path}")
    return map_path, np.asarray(rows, dtype=np.float64)


def load_map(map_path: Path) -> np.ndarray:
    tokens = map_path.read_text().split()
    if len(tokens) < 4 or tokens[0] != "height" or tokens[2] != "width":
        raise RuntimeError(f"Invalid map header in {map_path}")
    height = int(tokens[1])
    width = int(tokens[3])
    values = np.asarray([int(v) for v in tokens[4:]], dtype=np.uint8)
    if values.size != height * width:
        raise RuntimeError(f"Invalid map cell count in {map_path}")
    raw = values.reshape(height, width)
    return raw.T


def read_stats(stats_path: Path | None) -> dict:
    if stats_path is None or not stats_path.exists():
        return {}
    with stats_path.open() as f:
        rows = list(csv.DictReader(f))
    return rows[0] if rows else {}


def wrapped_delta(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    diff = np.abs(a - b)
    return np.minimum(diff, np.abs(TWO_PI - diff))


def path_metrics(path: np.ndarray) -> dict:
    if path.shape[0] <= 1:
        edge_cost = np.zeros(1, dtype=np.float64)
    else:
        edge_cost = np.sum(wrapped_delta(path[1:], path[:-1]), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(edge_cost)])
    return {
        "edge_cost": edge_cost,
        "cumulative_cost": cumulative,
        "total_cost": float(cumulative[-1]),
        "max_step_cost": float(np.max(edge_cost)) if edge_cost.size else 0.0,
        "mean_step_cost": float(np.mean(edge_cost)) if edge_cost.size else 0.0,
    }


def forward_kinematics(q: np.ndarray, width: int) -> np.ndarray:
    joints = np.zeros((q.shape[0] + 1, 2), dtype=np.float64)
    joints[0] = [width / 2.0, 0.0]
    for i, angle in enumerate(q):
        joints[i + 1, 0] = joints[i, 0] + LINK_LENGTH_CELLS * math.cos(TWO_PI - angle)
        joints[i + 1, 1] = joints[i, 1] - LINK_LENGTH_CELLS * math.sin(TWO_PI - angle)
    return joints


def all_joint_positions(path: np.ndarray, width: int) -> np.ndarray:
    return np.stack([forward_kinematics(q, width) for q in path], axis=0)


def sample_frame_indices(count: int, max_frames: int) -> np.ndarray:
    if count <= 1:
        return np.asarray([0], dtype=int)
    return np.unique(np.linspace(0, count - 1, min(count, max_frames), dtype=int))


def setup_axes(fig: plt.Figure):
    gs = fig.add_gridspec(
        3,
        2,
        width_ratios=[2.28, 1.22],
        height_ratios=[1.08, 1.02, 0.88],
        left=0.035,
        right=0.975,
        top=0.855,
        bottom=0.075,
        wspace=0.15,
        hspace=0.34,
    )
    ax_scene = fig.add_subplot(gs[:, 0])
    ax_focus = fig.add_subplot(gs[0:2, 1])
    ax_cost = fig.add_subplot(gs[2, 1])
    return ax_scene, ax_focus, ax_cost


def style_2d_axis(ax, title: str):
    ax.set_title(title, fontsize=12, loc="left", color="#111827", pad=7, weight="bold")
    ax.set_facecolor("#F7F9FB")
    for spine in ax.spines.values():
        spine.set_color("#C8D0D8")
    ax.grid(True, color="#D8DEE6", linewidth=0.8, alpha=0.75)
    ax.tick_params(colors="#4B5563", labelsize=8.4)


def draw_exact_obstacles(ax, occupancy: np.ndarray, alpha: float = 1.0, depth_cue: bool = True):
    width, height = occupancy.shape
    if depth_cue:
        for x0, x1, y in occupied_runs(occupancy):
            ax.add_patch(
                Rectangle(
                    (x0 + 0.22, y + 0.22),
                    x1 - x0,
                    1.0,
                    facecolor="#8B98A7",
                    edgecolor="none",
                    alpha=0.35 * alpha,
                    zorder=1,
                )
            )

    for x0, x1, y in occupied_runs(occupancy):
        ax.add_patch(
            Rectangle(
                (x0, y),
                x1 - x0,
                1.0,
                facecolor=OBSTACLE_COLOR,
                edgecolor=OBSTACLE_EDGE,
                linewidth=0.25,
                alpha=alpha,
                zorder=3,
            )
        )

    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)


def draw_map_grid(ax, occupancy: np.ndarray, major_step: int = 5):
    width, height = occupancy.shape
    ax.set_facecolor(FREE_COLOR)
    draw_exact_obstacles(ax, occupancy, alpha=1.0, depth_cue=True)
    for x in range(0, width + 1, major_step):
        ax.axvline(x, color=GRID_COLOR, linewidth=0.6, alpha=0.65, zorder=0)
    for y in range(0, height + 1, major_step):
        ax.axhline(y, color=GRID_COLOR, linewidth=0.6, alpha=0.65, zorder=0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x cells", fontsize=9, color="#4B5563")
    ax.set_ylabel("y cells", fontsize=9, color="#4B5563")
    ax.tick_params(colors="#566170", labelsize=8)
    for spine in ax.spines.values():
        spine.set_color("#AEB8C4")
        spine.set_linewidth(1.0)


def draw_arm_topdown(
    ax,
    joints: np.ndarray,
    colors: list[str] | None = None,
    alpha: float = 1.0,
    linewidth: float = 3.0,
    joint_size: float = 38.0,
    label_joints: bool = False,
    zorder: int = 10,
):
    if colors is None:
        colors = LINK_COLORS
    for i in range(joints.shape[0] - 1):
        ax.plot(
            joints[i : i + 2, 0],
            joints[i : i + 2, 1],
            color="#111827",
            linewidth=linewidth + 1.8,
            alpha=0.20 * alpha,
            solid_capstyle="round",
            zorder=zorder - 1,
        )
        ax.plot(
            joints[i : i + 2, 0],
            joints[i : i + 2, 1],
            color=colors[i % len(colors)],
            linewidth=linewidth,
            alpha=alpha,
            solid_capstyle="round",
            zorder=zorder + i,
        )

    ax.scatter(
        joints[1:, 0],
        joints[1:, 1],
        s=joint_size,
        color="#FFFFFF",
        edgecolor="#111827",
        linewidth=0.9,
        alpha=alpha,
        zorder=zorder + 20,
    )
    ax.scatter(
        [joints[-1, 0]],
        [joints[-1, 1]],
        s=joint_size * 1.35,
        color="#E63946",
        edgecolor="#FFFFFF",
        linewidth=0.9,
        alpha=alpha,
        zorder=zorder + 22,
    )

    if label_joints:
        for i in range(1, joints.shape[0]):
            ax.text(
                joints[i, 0] + 0.7,
                joints[i, 1] - 0.7,
                f"q{i}",
                fontsize=7.5,
                color="#111827",
                bbox=dict(boxstyle="round,pad=0.16", facecolor="white", edgecolor="#CBD5E1", alpha=0.88),
                zorder=zorder + 30,
            )


def signed_angle_delta(from_angle: float, to_angle: float) -> float:
    return (to_angle - from_angle + math.pi) % TWO_PI - math.pi


def draw_mini_arm(ax, joints: np.ndarray, label: str, endpoint_color: str, alpha: float = 1.0):
    rel = joints - joints[0]
    min_xy = np.min(rel, axis=0)
    max_xy = np.max(rel, axis=0)
    center = 0.5 * (min_xy + max_xy)
    span = max(float(np.max(max_xy - min_xy)), 8.0)
    pad = max(2.0, 0.22 * span)
    half = 0.5 * span + pad
    ax.set_xlim(float(center[0] - half), float(center[0] + half))
    ax.set_ylim(float(center[1] + half), float(center[1] - half))
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    ax.plot([center[0] - half, center[0] + half], [0, 0], color="#E2E8F0", linewidth=0.8, zorder=0)
    ax.plot([0, 0], [center[1] - half, center[1] + half], color="#E2E8F0", linewidth=0.8, zorder=0)
    ax.scatter([0.0], [0.0], s=28, color="#176B45", edgecolor="white", linewidth=0.6, zorder=10)
    for j in range(rel.shape[0] - 1):
        ax.plot(
            rel[j : j + 2, 0],
            rel[j : j + 2, 1],
            color="#111827",
            linewidth=4.4,
            alpha=0.18 * alpha,
            solid_capstyle="round",
            zorder=4,
        )
        ax.plot(
            rel[j : j + 2, 0],
            rel[j : j + 2, 1],
            color=LINK_COLORS[j % len(LINK_COLORS)],
            linewidth=3.0,
            alpha=alpha,
            solid_capstyle="round",
            zorder=5 + j,
        )
    ax.scatter(rel[1:, 0], rel[1:, 1], s=25, color="#FFFFFF", edgecolor="#111827", linewidth=0.7, alpha=alpha, zorder=20)
    ax.scatter([rel[-1, 0]], [rel[-1, 1]], s=44, color=endpoint_color, edgecolor="#FFFFFF", linewidth=0.7, alpha=alpha, zorder=25)
    ax.text(0.5, -0.05, label, transform=ax.transAxes, ha="center", va="top", fontsize=8.0, color="#111827", weight="bold")


def angle_to_axis_x(angle: float, left: float, width: float) -> float:
    return left + width * (angle % TWO_PI) / TWO_PI


def draw_wrapped_motion_segment(ax, y: float, start: float, current: float, left: float, width: float, color: str):
    delta = signed_angle_delta(start, current)
    samples = (start + np.linspace(0.0, delta, 80)) % TWO_PI
    xs = np.asarray([angle_to_axis_x(a, left, width) for a in samples])
    breaks = np.where(np.abs(np.diff(xs)) > width * 0.55)[0]
    begin = 0
    for end in list(breaks + 1) + [len(xs)]:
        if end - begin >= 2:
            ax.plot(xs[begin:end], np.full(end - begin, y), color=color, linewidth=3.0, solid_capstyle="round", zorder=6)
        begin = end


def draw_joint_progress_rows(ax, path: np.ndarray, idx: int):
    dofs = path.shape[1]
    left = 0.15
    width = 0.70
    top = 0.365
    if dofs <= 3:
        row_gap = 0.054
    elif dofs <= 5:
        row_gap = 0.044
    else:
        row_gap = 0.036
    for j in range(dofs):
        y = top - j * row_gap
        color = LINK_COLORS[j % len(LINK_COLORS)]
        ax.text(0.045, y, f"q{j + 1}", ha="left", va="center", fontsize=9.0, weight="bold", color=color)
        ax.plot([left, left + width], [y, y], color="#D8DEE6", linewidth=2.0, solid_capstyle="round", zorder=1)
        for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
            x = left + width * frac
            ax.plot([x, x], [y - 0.008, y + 0.008], color="#C8D0D8", linewidth=0.7, zorder=2)
        start = float(path[0, j])
        current = float(path[idx, j])
        goal = float(path[-1, j])
        draw_wrapped_motion_segment(ax, y, start, current, left, width, color)
        sx = angle_to_axis_x(start, left, width)
        cx = angle_to_axis_x(current, left, width)
        gx = angle_to_axis_x(goal, left, width)
        ax.scatter([sx], [y], s=18, marker="s", color="#2A9D55", edgecolor="white", linewidth=0.5, zorder=10)
        ax.scatter([cx], [y], s=24, marker="o", color="#F5B301", edgecolor="#111827", linewidth=0.5, zorder=11)
        ax.scatter([gx], [y], s=31, marker="*", color="#8B5CF6", edgecolor="white", linewidth=0.5, zorder=10)
        err = abs(signed_angle_delta(current, goal))
        ax.text(0.945, y, f"{err:.2f}", ha="right", va="center", fontsize=8.1, color="#374151")

    header_y = top + row_gap * 0.78
    ax.text(0.045, header_y, "joint", ha="left", va="center", fontsize=8.0, color="#374151", weight="bold")
    ax.text(left, header_y, "0", ha="center", va="center", fontsize=8.0, color="#374151")
    ax.text(left + width / 2.0, header_y, "pi", ha="center", va="center", fontsize=8.0, color="#374151")
    ax.text(left + width, header_y, "2pi", ha="center", va="center", fontsize=8.0, color="#374151")
    ax.text(0.945, header_y, "err", ha="right", va="center", fontsize=8.0, color="#374151", weight="bold")


def draw_joint_motion_summary(ax, joints_all: np.ndarray, path: np.ndarray, idx: int):
    dofs = path.shape[1]
    ax.set_title("Joint-Space Motion Summary", fontsize=12, loc="left", color="#111827", pad=7, weight="bold")
    ax.set_facecolor("#F7F9FB")
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    ax.add_patch(Rectangle((0.02, 0.035), 0.96, 0.905, facecolor="#FDFEFE", edgecolor="#CBD5E1", linewidth=0.9, zorder=0))
    ax.text(
        0.045,
        0.910,
        "start / current / goal arm geometry",
        ha="left",
        va="center",
        fontsize=9.0,
        color="#111827",
        weight="bold",
    )

    panels = [
        ("start", joints_all[0], "#2A9D55", 0.95),
        ("current", joints_all[idx], "#F5B301", 1.0),
        ("goal", joints_all[-1], "#8B5CF6", 0.95),
    ]
    for panel_idx, (label, joints, marker_color, alpha) in enumerate(panels):
        inset = ax.inset_axes([0.035 + panel_idx * 0.315, 0.515, 0.300, 0.345])
        draw_mini_arm(inset, joints, label, marker_color, alpha=alpha)

    draw_joint_progress_rows(ax, path, idx)

    progress = 0.0 if path.shape[0] <= 1 else idx / float(path.shape[0] - 1)
    ax.add_patch(Rectangle((0.09, 0.060), 0.82, 0.034, facecolor="#E5EAF0", edgecolor="none", zorder=1))
    ax.add_patch(Rectangle((0.09, 0.060), 0.82 * progress, 0.034, facecolor="#00A9E0", edgecolor="none", zorder=2))
    ax.text(0.09, 0.123, f"{dofs} DOF verified path", ha="left", va="center", fontsize=9.4, weight="bold", color="#111827")
    ax.text(0.91, 0.123, f"waypoint {idx + 1}/{path.shape[0]}", ha="right", va="center", fontsize=8.8, color="#374151")


def annotate_marker(ax, label: str, xy: np.ndarray, offset: tuple[float, float], color: str):
    ax.annotate(
        label,
        xy=(float(xy[0]), float(xy[1])),
        xytext=(float(xy[0] + offset[0]), float(xy[1] + offset[1])),
        fontsize=8,
        color=color,
        arrowprops=dict(arrowstyle="->", color=color, lw=1.0, shrinkA=2, shrinkB=3),
        bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="#CBD5E1", alpha=0.88),
        zorder=80,
    )


def occupied_runs(occupancy: np.ndarray) -> list[tuple[int, int, int]]:
    width, height = occupancy.shape
    runs = []
    for y in range(height):
        x = 0
        while x < width:
            if occupancy[x, y] == 0:
                x += 1
                continue
            x0 = x
            while x < width and occupancy[x, y] == 1:
                x += 1
            runs.append((x0, x, y))
    return runs


def draw_scene(ax, occupancy: np.ndarray, joints_all: np.ndarray, idx: int, title: str, metrics: dict, stats: dict):
    width, height = occupancy.shape
    end_xy = joints_all[:, -1, :]
    current_joints = joints_all[idx]

    draw_map_grid(ax, occupancy, major_step=5)
    ax.set_title("Planner Motion on Verified Map Geometry", loc="left", fontsize=13, color="#111827", pad=8, weight="bold")

    ghost_ids = np.unique(np.linspace(0, idx, min(18, idx + 1), dtype=int))
    for k in ghost_ids:
        draw_arm_topdown(
            ax,
            joints_all[k],
            colors=["#5E8C61"] * max(1, joints_all.shape[2]),
            alpha=0.18,
            linewidth=2.1,
            joint_size=15,
            label_joints=False,
            zorder=7,
        )

    draw_arm_topdown(
        ax,
        joints_all[0],
        colors=["#9CA3AF"] * max(1, joints_all.shape[2]),
        alpha=0.36,
        linewidth=2.2,
        joint_size=20,
        label_joints=False,
        zorder=8,
    )
    draw_arm_topdown(
        ax,
        joints_all[-1],
        colors=["#94A3B8"] * max(1, joints_all.shape[2]),
        alpha=0.42,
        linewidth=2.2,
        joint_size=20,
        label_joints=False,
        zorder=9,
    )

    ax.plot(end_xy[:, 0], end_xy[:, 1], color="#94A3B8", linewidth=1.4, alpha=0.70, zorder=20)
    ax.plot(end_xy[: idx + 1, 0], end_xy[: idx + 1, 1], color="#00A9E0", linewidth=2.8, alpha=0.98, zorder=21)
    draw_arm_topdown(
        ax,
        current_joints,
        alpha=0.98,
        linewidth=4.8,
        joint_size=54,
        label_joints=True,
        zorder=30,
    )

    ax.scatter([width / 2.0], [0.0], color="#176B45", s=90, edgecolor="white", linewidth=1.0, zorder=60)
    ax.text(width / 2.0 + 0.8, 1.8, "base", color="#176B45", fontsize=9, zorder=61)
    ax.scatter(end_xy[0, 0], end_xy[0, 1], s=58, marker="s", color="#2A9D55", edgecolor="white", linewidth=0.9, zorder=50)
    ax.scatter(end_xy[idx, 0], end_xy[idx, 1], s=70, marker="o", color="#F5B301", edgecolor="#111827", linewidth=0.8, zorder=52)
    ax.scatter(end_xy[-1, 0], end_xy[-1, 1], s=96, marker="*", color="#8B5CF6", edgecolor="white", linewidth=0.9, zorder=50)
    annotate_marker(ax, "start", end_xy[0], (1.4, -2.2), "#176B45")
    final_frame = np.linalg.norm(end_xy[idx] - end_xy[-1]) < 1e-8
    if final_frame:
        annotate_marker(ax, "current = goal", end_xy[-1], (1.4, 3.2), "#5B21B6")
    else:
        annotate_marker(ax, "current", end_xy[idx], (1.4, -2.2), "#8A5A00")
        annotate_marker(ax, "goal", end_xy[-1], (1.4, 2.6), "#5B21B6")

    vertices = stats.get("vertices_generated", "n/a")
    first_solution = stats.get("first_solution_time", "n/a")
    try:
        first_solution = f"{float(first_solution):.4f}s"
    except (TypeError, ValueError):
        pass
    detail = (
        f"Waypoint  {idx + 1}/{joints_all.shape[0]}\n"
        f"Arm DOF   {current_joints.shape[0] - 1}\n"
        f"Cost      {metrics['cumulative_cost'][idx]:.3f}/{metrics['total_cost']:.3f}\n"
        f"Vertices  {vertices}\n"
        f"First sol {first_solution}"
    )
    ax.text(
        0.018,
        0.94,
        detail,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9.4,
        color="#111827",
        linespacing=1.22,
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#FFFFFF", edgecolor="#CBD5E1", alpha=0.98),
    )

    handles = [
        Line2D([0], [0], color="#00A9E0", lw=2.5, label="end-effector trace"),
        Line2D([0], [0], color="#2457C5", lw=3.5, label="arm links"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#2A9D55", markeredgecolor="white", markersize=7, label="start"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#F5B301", markeredgecolor="#111827", markersize=7, label="current"),
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#8B5CF6", markeredgecolor="white", markersize=10, label="goal"),
        Patch(facecolor=OBSTACLE_COLOR, edgecolor=OBSTACLE_EDGE, label="obstacle cells"),
    ]
    ax.legend(
        handles=handles,
        loc="lower left",
        bbox_to_anchor=(0.015, 0.025),
        fontsize=8.3,
        frameon=True,
        handlelength=1.8,
        handletextpad=0.55,
    )


def draw_focus(ax, joints_all: np.ndarray, path: np.ndarray, idx: int):
    draw_joint_motion_summary(ax, joints_all, path, idx)


def draw_joint_panel(ax, path: np.ndarray, idx: int):
    style_2d_axis(ax, f"Joint Angles Over Path ({path.shape[1]} DOF)")
    t = np.arange(path.shape[0])
    for j in range(path.shape[1]):
        ax.plot(t, path[:, j], linewidth=1.55, color=LINK_COLORS[j % len(LINK_COLORS)], label=f"q{j + 1}")
        ax.scatter([idx], [path[idx, j]], s=18, color=LINK_COLORS[j % len(LINK_COLORS)], zorder=3)
    ax.axvline(idx, color="#111827", linestyle="--", linewidth=1.0, alpha=0.72)
    ax.set_xlim(0, max(1, path.shape[0] - 1))
    ax.set_ylim(-0.08, TWO_PI + 0.08)
    ax.set_ylabel("radians", fontsize=8)
    ax.legend(loc="upper right", fontsize=7, ncol=2, frameon=True)


def draw_cost_panel(ax, metrics: dict, idx: int):
    style_2d_axis(ax, "Path Cost")
    t = np.arange(metrics["cumulative_cost"].shape[0])
    ax.plot(t, metrics["cumulative_cost"], color="#2F80ED", linewidth=2.0, label="cumulative")
    if metrics["edge_cost"].size:
        ax.fill_between(
            np.arange(1, metrics["edge_cost"].shape[0] + 1),
            0.0,
            metrics["edge_cost"],
            color="#F2994A",
            alpha=0.28,
            label="per-step cost",
        )
    ax.axvline(idx, color="#111827", linestyle="--", linewidth=1.0, alpha=0.72)
    ax.scatter([idx], [metrics["cumulative_cost"][idx]], s=34, color="#F5B301", edgecolor="#1F2937", linewidth=0.7)
    ax.set_xlim(0, max(1, metrics["cumulative_cost"].shape[0] - 1))
    ax.set_ylim(0, max(0.1, float(metrics["cumulative_cost"][-1]) * 1.08))
    ax.set_xlabel("path waypoint", fontsize=8)
    ax.set_ylabel("wrapped joint cost", fontsize=8)
    ax.legend(loc="upper left", fontsize=7, frameon=True)


def render_frame(
    occupancy: np.ndarray,
    path: np.ndarray,
    joints_all: np.ndarray,
    metrics: dict,
    stats: dict,
    idx: int,
    title: str,
    subtitle: str,
    dpi: int,
) -> Image.Image:
    fig = plt.figure(figsize=(14.8, 8.4), dpi=dpi, facecolor="#EEF2F6")
    fig.text(0.035, 0.972, title, ha="left", va="top", fontsize=17, weight="bold", color="#111827")
    fig.text(0.035, 0.925, subtitle, ha="left", va="top", fontsize=11, color="#374151")
    ax_scene, ax_focus, ax_cost = setup_axes(fig)
    draw_scene(ax_scene, occupancy, joints_all, idx, title, metrics, stats)
    draw_focus(ax_focus, joints_all, path, idx)
    draw_cost_panel(ax_cost, metrics, idx)
    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8)
    image = Image.fromarray(frame[:, :, :3]).copy()
    plt.close(fig)
    plt.close("all")
    gc.collect()
    return image


def write_metrics(output_dir: Path, solution_path: Path, map_path: Path, path: np.ndarray, metrics: dict, stats: dict):
    payload = {
        "solution": str(solution_path),
        "map": str(map_path),
        "waypoints": int(path.shape[0]),
        "dofs": int(path.shape[1]),
        "total_cost": metrics["total_cost"],
        "max_step_cost": metrics["max_step_cost"],
        "mean_step_cost": metrics["mean_step_cost"],
        "stats": stats,
    }
    (output_dir / "render_metrics.json").write_text(json.dumps(payload, indent=2))


def main():
    parser = argparse.ArgumentParser(description="Render a polished high-DOF arm planner motion panel.")
    parser.add_argument("solution", type=Path, help="Path file produced by the planner")
    parser.add_argument("--stats", type=Path, default=None, help="Optional planner stats CSV")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for animated_panel.gif and final_panel.png")
    parser.add_argument("--title", type=str, default=None)
    parser.add_argument("--subtitle", type=str, default=None)
    parser.add_argument("--max-frames", type=int, default=10)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--dpi", type=int, default=95, help="Final PNG DPI. GIF DPI defaults lower for stable rendering.")
    parser.add_argument("--gif-dpi", type=int, default=None, help="Optional GIF frame DPI")
    args = parser.parse_args()

    map_path, path = load_solution(args.solution)
    occupancy = load_map(map_path)
    metrics = path_metrics(path)
    stats = read_stats(args.stats)
    joints_all = all_joint_positions(path, occupancy.shape[0])

    title = args.title or f"{map_path.stem} | High-DOF Arm Planner"
    subtitle = args.subtitle or (
        f"Planner motion on verified map geometry | {path.shape[1]} DOF | "
        f"{path.shape[0]} waypoints | cost {metrics['total_cost']:.3f}"
    )
    gif_dpi = args.gif_dpi if args.gif_dpi is not None else min(args.dpi, 60)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_metrics(args.output_dir, args.solution, map_path, path, metrics, stats)

    final = render_frame(occupancy, path, joints_all, metrics, stats, path.shape[0] - 1, title, subtitle, args.dpi)
    final.save(args.output_dir / "final_panel.png")

    frame_ids = sample_frame_indices(path.shape[0], args.max_frames)
    frames = [
        render_frame(occupancy, path, joints_all, metrics, stats, int(idx), title, subtitle, gif_dpi).convert(
            "P",
            palette=Image.ADAPTIVE,
            colors=192,
        )
        for idx in frame_ids
    ]
    duration_ms = max(20, int(round(1000.0 / max(1, args.fps))))
    frames[0].save(
        args.output_dir / "animated_panel.gif",
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
    )
    print(args.output_dir / "animated_panel.gif")
    print(args.output_dir / "final_panel.png")


if __name__ == "__main__":
    main()
