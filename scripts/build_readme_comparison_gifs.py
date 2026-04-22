import argparse
import csv
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageSequence


METHODS = [
    ("rrt", "Rapidly exploring Random Tree (RRT)", "rapidly_exploring_random_tree"),
    (
        "rrt_connect",
        "Rapidly exploring Random Tree Connect (RRT Connect)",
        "rapidly_exploring_random_tree_connect",
    ),
    (
        "rrt_star",
        "Rapidly exploring Random Tree Star (RRT*)",
        "rapidly_exploring_random_tree_star",
    ),
    ("prm", "Probabilistic Roadmap (PRM)", "probabilistic_roadmap"),
]

MAPS = [
    ("map1", "Map 1"),
    ("map2", "Map 2"),
]

PAIR_IDS = range(20)

SELECTED_CASES = [
    ("map1", 1, "Map 1 | hard pair 01"),
    ("map1", 4, "Map 1 | hard pair 04"),
    ("map2", 0, "Map 2 | hard pair 00"),
    ("map2", 11, "Map 2 | hard pair 11"),
]


def load_font(size: int, bold: bool = False):
    candidates = (
        ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"] if bold else []
    ) + [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            pass
    return ImageFont.load_default()


def load_gif_frames(path: Path) -> tuple[list[Image.Image], list[int]]:
    image = Image.open(path)
    frames = []
    durations = []
    for frame in ImageSequence.Iterator(image):
        frames.append(frame.convert("RGB"))
        durations.append(int(frame.info.get("duration", 500)))
    if not frames:
        raise RuntimeError(f"no frames in {path}")
    return frames, durations


def read_pair_metrics(benchmark_root: Path, map_name: str, pair_id: int) -> dict:
    with (benchmark_root / map_name / "random_pairs.csv").open() as f:
        for row in csv.DictReader(f):
            if int(row["pair_id"]) == pair_id:
                return row
    raise RuntimeError(f"pair {pair_id:02d} not found for {map_name}")


def frame_at(frames: list[Image.Image], idx: int, frame_count: int) -> Image.Image:
    if len(frames) == frame_count:
        return frames[idx]
    mapped = round(idx * (len(frames) - 1) / max(1, frame_count - 1))
    return frames[mapped]


def draw_label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    font: ImageFont.ImageFont,
):
    x, y = xy
    padding_x = 7
    padding_y = 4
    bbox = draw.textbbox((0, 0), text, font=font)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    draw.rounded_rectangle(
        (x, y, x + width + 2 * padding_x, y + height + 2 * padding_y),
        radius=5,
        fill=(17, 24, 39),
        outline=(255, 255, 255),
        width=1,
    )
    draw.text((x + padding_x, y + padding_y - 1), text, fill=(255, 255, 255), font=font)


def build_map_grid_panel(
    visual_root: Path,
    output_dir: Path,
    method_key: str,
    method_title: str,
    output_stem: str,
    map_key: str,
    map_title: str,
    tile_width: int,
) -> Path:
    loaded = []
    for pair_id in PAIR_IDS:
        gif_path = visual_root / map_key / method_key / f"pair_{pair_id:02d}" / "map_motion.gif"
        frames, durations = load_gif_frames(gif_path)
        loaded.append((pair_id, frames, durations))

    source_w, source_h = loaded[0][1][0].size
    tile_h = round(tile_width * source_h / source_w)
    cols = 5
    rows = 4
    gutter = 10
    header_h = 78
    footer_h = 34
    canvas_w = tile_width * cols + gutter * (cols + 1)
    canvas_h = header_h + tile_h * rows + gutter * (rows + 1) + footer_h
    frame_count = max(len(frames) for _, frames, _ in loaded)
    duration = max(80, round(sum(loaded[0][2]) / len(loaded[0][2])))

    title_font = load_font(24, bold=True)
    text_font = load_font(15)
    label_font = load_font(15, bold=True)

    title = f"{method_title} | {map_title}"
    subtitle = "All 20 corrected benchmark motions, arranged as 5 columns x 4 rows."

    out_frames = []
    for idx in range(frame_count):
        canvas = Image.new("RGB", (canvas_w, canvas_h), (238, 242, 246))
        draw = ImageDraw.Draw(canvas)
        draw.text((gutter, 12), title, fill=(17, 24, 39), font=title_font)
        draw.text((gutter, 46), subtitle, fill=(55, 65, 81), font=text_font)

        for tile_idx, (pair_id, frames, _) in enumerate(loaded):
            row = tile_idx // cols
            col = tile_idx % cols
            x = gutter + col * (tile_width + gutter)
            y = header_h + gutter + row * (tile_h + gutter)
            frame = frame_at(frames, idx, frame_count).resize(
                (tile_width, tile_h),
                Image.Resampling.LANCZOS,
            )
            canvas.paste(frame, (x, y))
            draw.rectangle((x, y, x + tile_width - 1, y + tile_h - 1), outline=(148, 163, 184), width=1)
            draw_label(draw, (x + 7, y + 7), f"pair {pair_id:02d}", label_font)

        footer = "Each tile shows the same planner solving one start goal pair on this map."
        draw.text((gutter, canvas_h - footer_h + 8), footer, fill=(55, 65, 81), font=text_font)
        out_frames.append(canvas.convert("P", palette=Image.ADAPTIVE, colors=192))

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{output_stem}_{map_key}_all_cases.gif"
    out_frames[0].save(
        output_path,
        save_all=True,
        append_images=out_frames[1:],
        duration=duration,
        loop=0,
        optimize=False,
    )
    return output_path


def build_hard_case_panel(
    visual_root: Path,
    benchmark_root: Path,
    output_dir: Path,
    method_key: str,
    method_title: str,
    output_stem: str,
) -> Path:
    loaded = []
    for map_name, pair_id, case_label in SELECTED_CASES:
        gif_path = visual_root / map_name / method_key / f"pair_{pair_id:02d}" / "animated_panel.gif"
        metrics = read_pair_metrics(benchmark_root, map_name, pair_id)
        frames, durations = load_gif_frames(gif_path)
        loaded.append((case_label, metrics, frames, durations))

    tile_w, tile_h = loaded[0][2][0].size
    gutter = 16
    header_h = 96
    footer_h = 42
    canvas_w = tile_w * 2 + gutter * 3
    canvas_h = header_h + tile_h * 2 + gutter * 3 + footer_h
    frame_count = max(len(frames) for _, _, frames, _ in loaded)
    duration = max(80, round(sum(loaded[0][3]) / len(loaded[0][3])))

    title_font = load_font(27, bold=True)
    text_font = load_font(17)
    label_font = load_font(18, bold=True)

    subtitle = "Top row: two hard Map 1 cases. Bottom row: two hard Map 2 cases."

    out_frames = []
    for idx in range(frame_count):
        canvas = Image.new("RGB", (canvas_w, canvas_h), (238, 242, 246))
        draw = ImageDraw.Draw(canvas)
        draw.text((gutter, 14), method_title, fill=(17, 24, 39), font=title_font)
        draw.text((gutter, 52), subtitle, fill=(55, 65, 81), font=text_font)

        for tile_idx, (case_label, metrics, frames, _) in enumerate(loaded):
            row = tile_idx // 2
            col = tile_idx % 2
            x = gutter + col * (tile_w + gutter)
            y = header_h + gutter + row * (tile_h + gutter)
            canvas.paste(frame_at(frames, idx, frame_count), (x, y))
            draw.rectangle((x, y, x + tile_w - 1, y + tile_h - 1), outline=(148, 163, 184), width=2)
            score = float(metrics["score"])
            joint = float(metrics["joint_distance"])
            label = f"{case_label} | score {score:.2f} | joint {joint:.2f}"
            draw_label(draw, (x + 12, y + 12), label, label_font)

        footer = "Each quadrant is the same planner on a different hard start goal query."
        draw.text((gutter, canvas_h - footer_h + 8), footer, fill=(55, 65, 81), font=text_font)
        out_frames.append(canvas.convert("P", palette=Image.ADAPTIVE, colors=192))

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{output_stem}_hard_cases.gif"
    out_frames[0].save(
        output_path,
        save_all=True,
        append_images=out_frames[1:],
        duration=duration,
        loop=0,
        optimize=False,
    )
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Build README hard case panels and 5x4 map motion GIF grids."
    )
    parser.add_argument("--visual-root", type=Path, default=Path("output/visualizations/seedset_16782"))
    parser.add_argument("--benchmark-root", type=Path, default=Path("output/report_benchmark/seedset_16782"))
    parser.add_argument("--output-dir", type=Path, default=Path("output/visualizations/readme_panels"))
    parser.add_argument("--tile-width", type=int, default=235)
    args = parser.parse_args()

    for method_key, method_title, output_stem in METHODS:
        path = build_hard_case_panel(
            args.visual_root,
            args.benchmark_root,
            args.output_dir,
            method_key,
            method_title,
            output_stem,
        )
        print(path)
        for map_key, map_title in MAPS:
            path = build_map_grid_panel(
                args.visual_root,
                args.output_dir,
                method_key,
                method_title,
                output_stem,
                map_key,
                map_title,
                args.tile_width,
            )
            print(path)


if __name__ == "__main__":
    main()
