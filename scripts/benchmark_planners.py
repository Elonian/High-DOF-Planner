import argparse
import csv
import math
import os
import random
import subprocess
import time
from pathlib import Path


PLANNERS = {
    0: "rrt",
    1: "rrt_connect",
    2: "rrt_star",
    3: "prm",
}

LINK_LENGTH_CELLS = 10.0
TWO_PI = 2.0 * math.pi


def load_map(root, map_name):
    map_path = root / "maps" / map_name
    tokens = map_path.read_text().split()
    if len(tokens) < 4 or tokens[0] != "height" or tokens[2] != "width":
        raise RuntimeError(f"Invalid map header in {map_path}")

    height = int(tokens[1])
    width = int(tokens[3])
    values = tokens[4:]
    if len(values) != width * height:
        raise RuntimeError(f"Invalid map cell count in {map_path}")

    grid = [0] * (width * height)
    idx = 0
    for y in range(height):
        for x in range(width):
            grid[y * width + x] = 0 if values[idx] == "0" else 1
            idx += 1
    return grid, width, height


def cell_from_continuous(x, y, width, height):
    cx = int(x)
    cy = int(y)
    if x < 0:
        cx = 0
    if y < 0:
        cy = 0
    if cx >= width:
        cx = width - 1
    if cy >= height:
        cy = height - 1
    return cx, cy


def line_is_valid(x0, y0, x1, y1, grid, width, height):
    if (
        x0 < 0 or x0 >= width or x1 < 0 or x1 >= width or
        y0 < 0 or y0 >= height or y1 < 0 or y1 >= height
    ):
        return False

    x0c, y0c = cell_from_continuous(x0, y0, width, height)
    x1c, y1c = cell_from_continuous(x1, y1, width, height)
    dx = abs(x1c - x0c)
    dy = abs(y1c - y0c)
    sx = 1 if x0c < x1c else -1
    sy = 1 if y0c < y1c else -1
    err = dx - dy
    x = x0c
    y = y0c

    while True:
        if grid[y * width + x] == 1:
            return False
        if x == x1c and y == y1c:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def config_is_valid(q, grid, width, height):
    x1 = width / 2.0
    y1 = 0.0
    for angle in q:
        x0 = x1
        y0 = y1
        x1 = x0 + LINK_LENGTH_CELLS * math.cos(TWO_PI - angle)
        y1 = y0 - LINK_LENGTH_CELLS * math.sin(TWO_PI - angle)
        if not line_is_valid(x0, y0, x1, y1, grid, width, height):
            return False
    return True


def run(cmd, cwd, env=None, timeout=None):
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    start = time.perf_counter()
    next_heartbeat = start + 2.0
    while proc.poll() is None:
        now = time.perf_counter()
        if timeout is not None and now - start > timeout:
            proc.kill()
            stdout, stderr = proc.communicate()
            return subprocess.CompletedProcess(
                cmd,
                124,
                stdout=stdout or "",
                stderr=(stderr or "") + f"\nTimed out after {timeout} seconds.",
            )
        if timeout is not None and timeout >= 3.0 and now >= next_heartbeat:
            print("  planner subprocess still running...", flush=True)
            next_heartbeat = now + 2.0
        time.sleep(0.05)

    stdout, stderr = proc.communicate()
    return subprocess.CompletedProcess(cmd, proc.returncode, stdout=stdout, stderr=stderr)


def write_solution(path, map_path, configs):
    with path.open("w") as f:
        f.write(str(map_path) + "\n")
        for q in configs:
            f.write(",".join(f"{v:.12f}" for v in q) + ",\n")


def verifier_accepts_config(root, map_name, q):
    tmp = root / "output" / "report_benchmark" / "_tmp_config_check.txt"
    map_path = root / "maps" / map_name
    q_str = ",".join(f"{v:.12f}" for v in q)
    write_solution(tmp, map_path, [q])
    proc = run(
        [str(root / "build" / "verifier"), str(map_path), str(len(q)), q_str, q_str, str(tmp)],
        cwd=root,
        timeout=5,
    )
    return proc.returncode == 0


def wrapped_joint_cost(path):
    cost = 0.0
    max_step = 0.0
    for a, b in zip(path, path[1:]):
        for x, y in zip(a, b):
            d = abs(x - y)
            d = min(d, abs(2.0 * math.pi - d))
            cost += d
            max_step = max(max_step, d)
    return cost, max_step


def end_effector(q, width):
    x = width / 2.0
    y = 0.0
    for angle in q:
        x += LINK_LENGTH_CELLS * math.cos(TWO_PI - angle)
        y -= LINK_LENGTH_CELLS * math.sin(TWO_PI - angle)
    return x, y


def pair_score(start, goal, width):
    joint_distance, _ = wrapped_joint_cost([start, goal])
    sx, sy = end_effector(start, width)
    gx, gy = end_effector(goal, width)
    ee_distance = math.hypot(gx - sx, gy - sy)
    return joint_distance + 0.035 * ee_distance, joint_distance, ee_distance


def read_solution(path):
    configs = []
    with path.open() as f:
        f.readline()
        for line in f:
            vals = [float(x) for x in line.strip().split(",") if x]
            if vals:
                configs.append(vals)
    return configs


def read_stats(path):
    if not path.exists():
        return {
            "vertices_generated": -1,
            "first_solution_time": -1.0,
            "used_fallback": -1,
            "plan_waypoints": -1,
        }
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {
            "vertices_generated": -1,
            "first_solution_time": -1.0,
            "used_fallback": -1,
            "plan_waypoints": -1,
        }
    row = rows[0]
    return {
        "vertices_generated": int(row["vertices_generated"]),
        "first_solution_time": float(row["first_solution_time"]),
        "used_fallback": int(row["used_fallback"]),
        "plan_waypoints": int(row["plan_waypoints"]),
    }


def generate_pairs(root, map_name, dofs, pair_count, seed, min_distance, candidate_pool):
    grid, width, height = load_map(root, map_name)
    pairs = []
    for pair_id in range(pair_count):
        pair_seed = seed + pair_id
        rng = random.Random(pair_seed)
        best = None
        attempts = 0
        target_valid_candidates = max(1, candidate_pool)
        valid_candidates = 0
        max_attempts = max(2000, target_valid_candidates * 300)
        while attempts < max_attempts and valid_candidates < target_valid_candidates:
            attempts += 1
            start = [rng.random() * 2.0 * math.pi for _ in range(dofs)]
            goal = [rng.random() * 2.0 * math.pi for _ in range(dofs)]
            if not config_is_valid(start, grid, width, height):
                continue
            if not config_is_valid(goal, grid, width, height):
                continue
            score, joint_distance, ee_distance = pair_score(start, goal, width)
            if joint_distance < min_distance:
                continue
            valid_candidates += 1
            if best is None or score > best["score"]:
                best = {
                    "pair_id": pair_id,
                    "instance_seed": pair_seed,
                    "start": start,
                    "goal": goal,
                    "score": score,
                    "joint_distance": joint_distance,
                    "end_effector_distance": ee_distance,
                    "sampling_attempts": attempts,
                    "valid_candidates": valid_candidates,
                }
        if best is None:
            raise RuntimeError(f"Could not sample a valid pair for seed {pair_seed}")
        pairs.append(best)
        print(
            f"sampled pair {pair_id:02d}/{pair_count - 1:02d} "
            f"seed={pair_seed} joint_distance={best['joint_distance']:.3f} "
            f"ee_distance={best['end_effector_distance']:.3f}",
            flush=True,
        )
    return pairs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="map2.txt")
    parser.add_argument("--dofs", type=int, default=3)
    parser.add_argument("--pairs", type=int, default=20)
    parser.add_argument("--seed", type=int, default=16782)
    parser.add_argument("--min-distance", type=float, default=1.0)
    parser.add_argument("--candidate-pool", type=int, default=80)
    parser.add_argument("--output-label", default=None)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    run_label = args.output_label or f"seed_{args.seed}"
    map_stem = args.map.replace(".txt", "")
    run_root = root / "output" / "report_benchmark" / run_label / map_stem
    run_root.mkdir(parents=True, exist_ok=True)

    pairs = generate_pairs(
        root,
        args.map,
        args.dofs,
        args.pairs,
        args.seed,
        args.min_distance,
        args.candidate_pool,
    )
    pairs_csv = run_root / "random_pairs.csv"
    with pairs_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "pair_id",
            "instance_seed",
            "dofs",
            "start",
            "goal",
            "joint_distance",
            "end_effector_distance",
            "score",
            "sampling_attempts",
            "valid_candidates",
        ])
        for pair in pairs:
            writer.writerow([
                pair["pair_id"],
                pair["instance_seed"],
                args.dofs,
                ",".join(f"{v:.12f}" for v in pair["start"]),
                ",".join(f"{v:.12f}" for v in pair["goal"]),
                f"{pair['joint_distance']:.12f}",
                f"{pair['end_effector_distance']:.12f}",
                f"{pair['score']:.12f}",
                pair["sampling_attempts"],
                pair["valid_candidates"],
            ])

    all_rows = []
    for planner_id, planner_name in PLANNERS.items():
        planner_dir = run_root / planner_name
        planner_dir.mkdir(parents=True, exist_ok=True)
        per_planner_rows = []
        for pair in pairs:
            pair_id = pair["pair_id"]
            start = pair["start"]
            goal = pair["goal"]
            start_str = ",".join(f"{v:.12f}" for v in start)
            goal_str = ",".join(f"{v:.12f}" for v in goal)
            rel_solution = (
                Path("report_benchmark")
                / run_label
                / map_stem
                / planner_name
                / f"pair_{pair_id:02d}_path.txt"
            )
            solution_path = root / "output" / rel_solution
            stats_path = planner_dir / f"pair_{pair_id:02d}_stats.csv"
            env = os.environ.copy()
            env["HDOF_PLANNER_STATS"] = str(stats_path)
            env["HDOF_PLANNER_SEED"] = str(pair["instance_seed"])

            print(
                f"running {planner_name} pair {pair_id:02d} "
                f"seed={pair['instance_seed']}",
                flush=True,
            )
            start_time = time.perf_counter()
            proc = run(
                [
                    str(root / "build" / "planner"),
                    args.map,
                    str(args.dofs),
                    start_str,
                    goal_str,
                    str(planner_id),
                    str(rel_solution),
                ],
                cwd=root,
                env=env,
                timeout=args.timeout,
            )
            wall_time = time.perf_counter() - start_time

            verified = False
            waypoints = -1
            cost = -1.0
            max_step = -1.0
            if proc.returncode == 0 and solution_path.exists():
                verify = run(
                    [
                        str(root / "build" / "verifier"),
                        str(root / "maps" / args.map),
                        str(args.dofs),
                        start_str,
                        goal_str,
                        str(solution_path),
                    ],
                    cwd=root,
                    timeout=5,
                )
                verified = verify.returncode == 0
                path = read_solution(solution_path)
                waypoints = len(path)
                cost, max_step = wrapped_joint_cost(path)

            stats = read_stats(stats_path)
            row = {
                "map": map_stem,
                "dofs": args.dofs,
                "planner": planner_name,
                "planner_id": planner_id,
                "pair_id": pair_id,
                "instance_seed": pair["instance_seed"],
                "target_joint_distance": pair["joint_distance"],
                "target_end_effector_distance": pair["end_effector_distance"],
                "success": verified,
                "under_5s": verified and wall_time <= 5.0,
                "wall_time_s": wall_time,
                "cost": cost,
                "waypoints": waypoints,
                "vertices_generated": stats["vertices_generated"],
                "first_solution_time_s": stats["first_solution_time"],
                "used_fallback": stats["used_fallback"],
                "max_joint_step": max_step,
                "planner_returncode": proc.returncode,
            }
            all_rows.append(row)
            per_planner_rows.append(row)

        with (planner_dir / "results.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(per_planner_rows[0].keys()))
            writer.writeheader()
            writer.writerows(per_planner_rows)

    all_csv = run_root / "all_runs.csv"
    with all_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)

    summary_rows = []
    for planner_id, planner_name in PLANNERS.items():
        rows = [r for r in all_rows if r["planner_id"] == planner_id]
        successes = [r for r in rows if r["success"]]
        def avg(key):
            vals = [r[key] for r in successes if isinstance(r[key], (int, float)) and r[key] >= 0]
            return sum(vals) / len(vals) if vals else -1.0
        summary_rows.append({
            "planner": planner_name,
            "planner_id": planner_id,
            "success_rate": len(successes) / len(rows),
            "under_5s_rate": sum(1 for r in rows if r["under_5s"]) / len(rows),
            "avg_wall_time_s": avg("wall_time_s"),
            "avg_vertices_generated": avg("vertices_generated"),
            "avg_path_cost": avg("cost"),
            "avg_waypoints": avg("waypoints"),
            "fallback_count": sum(1 for r in rows if r["used_fallback"] == 1),
        })

    summary_csv = run_root / "summary.csv"
    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    md = [
        "# High-DOF Arm Planner Benchmark",
        "",
        f"- Map: `{args.map}`",
        f"- DOFs: `{args.dofs}`",
        f"- Seed set: `{args.seed}` through `{args.seed + args.pairs - 1}`",
        f"- Shared start/goal pairs: `{args.pairs}`",
        f"- Minimum wrapped joint distance: `{args.min_distance}`",
        f"- Candidate pool per pair seed: `{args.candidate_pool}`",
        "- Visualization: not generated in this run",
        "",
        "| Planner | Success Rate | Under 5s Rate | Avg Time (s) | Avg Vertices | Avg Cost | Avg Waypoints | Fallback Count |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        md.append(
            f"| {row['planner']} | {row['success_rate']:.2f} | {row['under_5s_rate']:.2f} | "
            f"{row['avg_wall_time_s']:.4f} | {row['avg_vertices_generated']:.1f} | "
            f"{row['avg_path_cost']:.4f} | {row['avg_waypoints']:.1f} | {row['fallback_count']} |"
        )
    md.extend([
        "",
        "## Benchmark Details",
        "",
        f"- The run uses `{args.map}`, {args.pairs} random start/goal pairs, and the same pair set for every planner.",
        "- Each pair has a recorded `instance_seed`; that same seed is also passed to the stochastic planner internals.",
        "- `cost` is the sum of wrapped absolute joint-angle changes along the emitted path.",
        "- `vertices_generated` is recorded through `HDOF_PLANNER_STATS`; the normal solution file remains unchanged.",
        "- `success` means the compiled verifier accepted all emitted waypoints plus the exact start and goal.",
    ])
    (run_root / "summary.md").write_text("\n".join(md))

    print(summary_csv.read_text())


if __name__ == "__main__":
    main()
