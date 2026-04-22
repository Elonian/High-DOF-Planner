# High-DOF Arm Planner Benchmark

- Map: `map2.txt`
- DOFs: `3`
- Seed set: `16782` through `16801`
- Shared start/goal pairs: `20`
- Minimum wrapped joint distance: `2.4`
- Candidate pool per pair seed: `80`
- Visualization: not generated in this run

| Planner | Success Rate | Under 5s Rate | Avg Time (s) | Avg Vertices | Avg Cost | Avg Waypoints | Fallback Count |
|---|---:|---:|---:|---:|---:|---:|---:|
| rrt | 1.00 | 1.00 | 0.0584 | 14.8 | 7.5417 | 64.5 | 0 |
| rrt_connect | 1.00 | 1.00 | 0.0519 | 22.4 | 7.3451 | 64.8 | 0 |
| rrt_star | 1.00 | 1.00 | 3.8125 | 7667.4 | 6.8666 | 64.5 | 0 |
| prm | 1.00 | 1.00 | 0.1948 | 1328.0 | 6.9028 | 67.1 | 0 |

## Benchmark Details

- The run uses `map2.txt`, 20 random start/goal pairs, and the same pair set for every planner.
- Each pair has a recorded `instance_seed`; that same seed is also passed to the stochastic planner internals.
- `cost` is the sum of wrapped absolute joint-angle changes along the emitted path.
- `vertices_generated` is recorded through `HDOF_PLANNER_STATS`; the normal solution file remains unchanged.
- `success` means the compiled verifier accepted all emitted waypoints plus the exact start and goal.