import argparse
import csv
import gc
from pathlib import Path

from PIL import Image

from render_arm_map_motion import render_map_frame, write_metrics as write_map_metrics
from render_arm_motion_panel import (
    all_joint_positions,
    load_map,
    load_solution,
    path_metrics,
    read_stats,
    render_frame,
    sample_frame_indices,
    write_metrics as write_panel_metrics,
)


def read_rows(benchmark_root: Path) -> dict[tuple[str, str, int], dict]:
    rows = {}
    for all_runs in benchmark_root.glob("*/all_runs.csv"):
        map_name = all_runs.parent.name
        with all_runs.open() as f:
            for row in csv.DictReader(f):
                key = (map_name, row["planner"], int(row["pair_id"]))
                rows[key] = row
    return rows


def path_dofs_and_waypoints(path_file: Path) -> tuple[int, int]:
    waypoints = 0
    dofs = 0
    with path_file.open() as f:
        f.readline()
        for line in f:
            vals = [v for v in line.strip().split(",") if v]
            if vals:
                waypoints += 1
                dofs = len(vals)
    return dofs, waypoints


def should_render(success_value: str) -> bool:
    return success_value.lower() in {"true", "1", "yes"}


def save_gif(frames: list[Image.Image], output_path: Path, fps: int):
    duration_ms = max(20, int(round(1000.0 / max(1, fps))))
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
    )


def render_panel_inline(
    output_dir: Path,
    path_file: Path,
    map_path: Path,
    occupancy,
    path,
    joints_all,
    metrics: dict,
    stats: dict,
    title: str,
    subtitle: str,
    max_frames: int,
    fps: int,
    dpi: int,
    gif_dpi: int,
):
    write_panel_metrics(output_dir, path_file, map_path, path, metrics, stats)
    final = render_frame(occupancy, path, joints_all, metrics, stats, path.shape[0] - 1, title, subtitle, dpi)
    final.save(output_dir / "final_panel.png")
    frame_ids = sample_frame_indices(path.shape[0], max_frames)
    frames = [
        render_frame(occupancy, path, joints_all, metrics, stats, int(idx), title, subtitle, gif_dpi).convert(
            "P",
            palette=Image.ADAPTIVE,
            colors=192,
        )
        for idx in frame_ids
    ]
    save_gif(frames, output_dir / "animated_panel.gif", fps)
    del frames
    gc.collect()


def render_map_inline(
    output_dir: Path,
    path_file: Path,
    map_path: Path,
    occupancy,
    path,
    joints_all,
    metrics: dict,
    stats: dict,
    title: str,
    subtitle: str,
    max_frames: int,
    fps: int,
    dpi: int,
    gif_dpi: int,
):
    write_map_metrics(output_dir, path_file, map_path, path, metrics, stats)
    final = render_map_frame(occupancy, path, joints_all, metrics, stats, path.shape[0] - 1, title, subtitle, dpi)
    final.save(output_dir / "map_final.png")
    frame_ids = sample_frame_indices(path.shape[0], max_frames)
    frames = [
        render_map_frame(occupancy, path, joints_all, metrics, stats, int(idx), title, subtitle, gif_dpi).convert(
            "P",
            palette=Image.ADAPTIVE,
            colors=192,
        )
        for idx in frame_ids
    ]
    save_gif(frames, output_dir / "map_motion.gif", fps)
    del frames
    gc.collect()


def main():
    parser = argparse.ArgumentParser(description="Render panel and map-only GIFs for benchmark result paths.")
    parser.add_argument("--benchmark-root", type=Path, required=True)
    parser.add_argument("--visual-root", type=Path, default=Path("output/visualizations"))
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--render-panels", action="store_true")
    parser.add_argument("--render-map-gifs", action="store_true")
    parser.add_argument("--map-filter", action="append", default=[])
    parser.add_argument("--planner-filter", action="append", default=[])
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--map-max-frames", type=int, default=12)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--dpi", type=int, default=90)
    parser.add_argument("--gif-dpi", type=int, default=55)
    parser.add_argument("--map-dpi", type=int, default=90)
    parser.add_argument("--map-gif-dpi", type=int, default=55)
    args = parser.parse_args()

    if not args.render_panels and not args.render_map_gifs:
        args.render_panels = True
        args.render_map_gifs = True

    project_root = Path(__file__).resolve().parents[1]
    benchmark_root = args.benchmark_root
    if not benchmark_root.is_absolute():
        benchmark_root = project_root / benchmark_root
    visual_root = args.visual_root
    if not visual_root.is_absolute():
        visual_root = project_root / visual_root

    run_label = benchmark_root.name
    rows = read_rows(benchmark_root)
    path_files = sorted(benchmark_root.glob("*/*/pair_*_path.txt"))
    rendered = 0
    skipped = 0

    for path_file in path_files:
        rel = path_file.relative_to(benchmark_root)
        map_name = rel.parts[0]
        planner = rel.parts[1]
        if args.map_filter and map_name not in args.map_filter:
            continue
        if args.planner_filter and planner not in args.planner_filter:
            continue
        pair_id = int(path_file.stem.split("_")[1])
        row = rows.get((map_name, planner, pair_id), {})
        if row and not should_render(str(row.get("success", ""))):
            skipped += 1
            continue

        dofs, waypoints = path_dofs_and_waypoints(path_file)
        instance_seed = row.get("instance_seed", "n/a")
        cost = row.get("cost", "n/a")
        output_dir = visual_root / run_label / map_name / planner / f"pair_{pair_id:02d}"
        stats_path = path_file.with_name(f"pair_{pair_id:02d}_stats.csv")
        title = f"{map_name} | {planner} | pair {pair_id:02d}"
        subtitle = f"Planner motion on verified map geometry | {dofs} DOF | seed {instance_seed}"

        need_panel = args.render_panels and (
            args.force or not (output_dir / "animated_panel.gif").exists() or not (output_dir / "final_panel.png").exists()
        )
        need_map = args.render_map_gifs and (
            args.force or not (output_dir / "map_motion.gif").exists() or not (output_dir / "map_final.png").exists()
        )
        if need_panel or need_map:
            output_dir.mkdir(parents=True, exist_ok=True)
            map_path, path = load_solution(path_file)
            occupancy = load_map(map_path)
            stats = read_stats(stats_path)
            metrics = path_metrics(path)
            joints_all = all_joint_positions(path, occupancy.shape[0])

            if need_panel:
                print(f"panel {map_name}/{planner}/pair_{pair_id:02d}", flush=True)
                render_panel_inline(
                    output_dir,
                    path_file,
                    map_path,
                    occupancy,
                    path,
                    joints_all,
                    metrics,
                    stats,
                    title,
                    subtitle,
                    args.max_frames,
                    args.fps,
                    args.dpi,
                    args.gif_dpi,
                )

            if need_map:
                print(f"map-only {map_name}/{planner}/pair_{pair_id:02d}", flush=True)
                map_subtitle = (
                    f"Main map animation | {dofs} DOF | seed {instance_seed} | "
                    f"waypoints {waypoints} | cost {cost}"
                )
                render_map_inline(
                    output_dir,
                    path_file,
                    map_path,
                    occupancy,
                    path,
                    joints_all,
                    metrics,
                    stats,
                    title,
                    map_subtitle,
                    args.map_max_frames,
                    args.fps,
                    args.map_dpi,
                    args.map_gif_dpi,
                )

            del occupancy, path, joints_all, metrics, stats
            gc.collect()

        rendered += 1

    print(f"rendered_success_paths={rendered}")
    print(f"skipped_failed_paths={skipped}")
    print(visual_root / run_label)


if __name__ == "__main__":
    main()
