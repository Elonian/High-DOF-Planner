import argparse
import gc
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from render_arm_motion_panel import (
    all_joint_positions,
    draw_scene,
    load_map,
    load_solution,
    path_metrics,
    read_stats,
    sample_frame_indices,
)


def render_map_frame(
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
    fig = plt.figure(figsize=(9.6, 9.1), dpi=dpi, facecolor="#EEF2F6")
    fig.text(0.055, 0.975, title, ha="left", va="top", fontsize=17, weight="bold", color="#111827")
    fig.text(0.055, 0.935, subtitle, ha="left", va="top", fontsize=10.5, color="#374151")
    ax = fig.add_axes([0.080, 0.075, 0.865, 0.805])
    draw_scene(ax, occupancy, joints_all, idx, title, metrics, stats)
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
        "outputs": {
            "map_motion_gif": "map_motion.gif",
            "map_final_png": "map_final.png",
        },
    }
    (output_dir / "map_render_metrics.json").write_text(json.dumps(payload, indent=2))


def main():
    parser = argparse.ArgumentParser(description="Render the main-map-only high-DOF arm motion GIF.")
    parser.add_argument("solution", type=Path, help="Path file produced by the planner")
    parser.add_argument("--stats", type=Path, default=None, help="Optional planner stats CSV")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for map_motion.gif and map_final.png")
    parser.add_argument("--title", type=str, default=None)
    parser.add_argument("--subtitle", type=str, default=None)
    parser.add_argument("--max-frames", type=int, default=14)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--dpi", type=int, default=100, help="Final PNG DPI")
    parser.add_argument("--gif-dpi", type=int, default=None, help="Optional GIF frame DPI")
    args = parser.parse_args()

    map_path, path = load_solution(args.solution)
    occupancy = load_map(map_path)
    metrics = path_metrics(path)
    stats = read_stats(args.stats)
    joints_all = all_joint_positions(path, occupancy.shape[0])

    title = args.title or f"{map_path.stem} | main map motion"
    subtitle = args.subtitle or (
        f"Verified map geometry | {path.shape[1]} DOF | "
        f"{path.shape[0]} waypoints | cost {metrics['total_cost']:.3f}"
    )
    gif_dpi = args.gif_dpi if args.gif_dpi is not None else min(args.dpi, 65)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_metrics(args.output_dir, args.solution, map_path, path, metrics, stats)

    final = render_map_frame(occupancy, path, joints_all, metrics, stats, path.shape[0] - 1, title, subtitle, args.dpi)
    final.save(args.output_dir / "map_final.png")

    frame_ids = sample_frame_indices(path.shape[0], args.max_frames)
    frames = [
        render_map_frame(occupancy, path, joints_all, metrics, stats, int(idx), title, subtitle, gif_dpi).convert(
            "P",
            palette=Image.ADAPTIVE,
            colors=192,
        )
        for idx in frame_ids
    ]
    duration_ms = max(20, int(round(1000.0 / max(1, args.fps))))
    frames[0].save(
        args.output_dir / "map_motion.gif",
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
    )
    print(args.output_dir / "map_motion.gif")
    print(args.output_dir / "map_final.png")


if __name__ == "__main__":
    main()
