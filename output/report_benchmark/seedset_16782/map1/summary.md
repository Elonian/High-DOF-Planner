# High-DOF Arm Planner Benchmark

- Map: `map1.txt`
- DOFs: `5`
- Seed set: `16782` through `16801`
- Shared start/goal pairs: `20`
- Minimum wrapped joint distance: `3.2`
- Candidate pool per pair seed: `80`
- Visualization: not generated in this run

| Planner | Success Rate | Under 5s Rate | Avg Time (s) | Avg Vertices | Avg Cost | Avg Waypoints | Fallback Count |
|---|---:|---:|---:|---:|---:|---:|---:|
| rrt | 1.00 | 1.00 | 0.0415 | 4.3 | 11.1262 | 58.4 | 0 |
| rrt_connect | 1.00 | 1.00 | 0.0351 | 7.8 | 10.8612 | 56.9 | 0 |
| rrt_star | 1.00 | 1.00 | 1.3753 | 2180.2 | 10.7385 | 57.1 | 0 |
| prm | 1.00 | 1.00 | 0.2218 | 602.0 | 10.8359 | 60.8 | 0 |

## Benchmark Details

- The run uses `map1.txt`, 20 random start/goal pairs, and the same pair set for every planner.
- Each pair has a recorded `instance_seed`; that same seed is also passed to the stochastic planner internals.
- `cost` is the sum of wrapped absolute joint-angle changes along the emitted path.
- `vertices_generated` is recorded through `HDOF_PLANNER_STATS`; the normal solution file remains unchanged.
- `success` means the compiled verifier accepted all emitted waypoints plus the exact start and goal.