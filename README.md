<img src="assets/icon.svg" width="64" height="64" alt="">

# Lantern

A small RViz stand-in in Rust on [Rune](https://github.com/anomalyco/rune) (pinned at `631e8a6`).
A simulated 128-beam LiDAR scans a street (road with lane paint, a building and annex, poles
every 6 m, trees, crates). The resulting ~1M-point cloud is shown in a 3D viewport with a grid,
TF frames, and a panel for colour mode, point size and density.

```sh
cargo run --release                                    # window
target/release/lantern --snap out/default.png [--scene top|intensity|street]
target/release/lantern --script scripts/interact.json --snap out/interact.png --at 1.8
scripts/bench.sh [frames] [trials]                     # timing table → out/bench.tsv
scripts/audit-controls.sh                              # every control must change the view
```

Viewport: drag to orbit, Shift-drag or middle-drag to pan, scroll to zoom.

## Layout

| File | Owns |
| --- | --- |
| `src/camera.rs` | Orbit camera (REP 103, z up), reversed infinite depth |
| `src/lidar.rs` | Scene, ray casts, sensor sweeps until ≥ 1M returns |
| `src/viewport.rs` | Rune `Compositor`: offscreen 3D pass, blit into `El::effect` slot, GPU timer |
| `src/viewport.wgsl` | sky + grid, points (vertex-pulled quads), TF lines, blit |
| `src/main.rs` | State, panel UI, input, scenes |

## Performance (RTX PRO 5000 Blackwell, driver 595.99, 1948×1560 px viewport, 1,028,089 points)

From `scripts/bench.sh 600 3`: 3 trials × 600 frames per row. Raw logs are in `out/bench/`.
Spread across trials is ≤ 0.001 ms on GPU p50.

| Config | Rune CPU/frame p50 | GPU 3D pass p50 (p95) |
| --- | --- | --- |
| 1.03M points, 2 pt | 0.10 ms | 0.342 (0.355) ms |
| ½ density | 0.03–0.10 ms | 0.188 ms |
| ¼ density | 0.03–0.10 ms | 0.113 ms |
| points off (sky + grid + TF only) | 0.03–0.10 ms | 0.030 ms |
| 1 pt points | 0.10 ms | 0.280 ms |
| 4 pt points | 0.05–0.13 ms | 0.662 ms |
| full + PNG readback every frame | 0.16 ms | 0.349 ms |

- **CPU** is Rune's own `frame cpu` line: update, view, layout, our encode, and submit. The
  compositor's share is about 0.02 ms. CPU cost doesn't depend on point count, because the
  cloud uploads once and the per-frame CPU work is a single draw call.
- **GPU** comes from begin/end-of-pass timestamp queries on the 3D pass alone. The UI pass and
  any readback copy are recorded after that pass ends, so the timestamps can't include them.
  The readback row confirms this: the GPU time stays the same (0.349 vs 0.342 ms), while wall
  throughput drops from ~1,260 to 26 frames/s because of PNG encoding.
- **It measures the point draw.** GPU time scales with the number of points drawn: 0 → 0.030,
  257k → 0.113, 514k → 0.188, 1.03M → 0.342 ms. That is ≈ 0.30 ms per million points plus a
  0.03 ms floor for the sky and grid.
- **60 fps:** with `--realtime` (paced to 60 Hz), 600 frames took 9.98 s wall (60.1 frames/s).
  The per-frame budget used is 0.17 ms CPU + 0.35 ms GPU out of 16.7 ms. That is measured
  headless, not in an on-screen window; window presentation is unmeasured.

### What bounds the point draw

The first version drew each point as a 4-vertex instance and took 0.666 ms. That time did not
change between 1 pt and 4 pt points (0.666 vs 0.672 ms), even though 4 pt points cover 16× the
pixels, so pixel fill was not the limit. Drawing six vertices per point in one non-instanced
draw halved it to 0.342 ms. The image is unchanged as far as I checked: `out/default.png` was
1,731,550 bytes before and after, and looks the same, but I didn't diff the pixels. That points
to per-instance overhead in the vertex front-end.
At 1 pt the draw now takes 0.280 ms, about 270 ns per thousand points. At 4 pt it takes
0.662 ms, so fill matters again at large point sizes. The GPU ran at 1.6 GHz of its 3.09 GHz
maximum (P1) during the runs. The headless loop waits for the GPU every frame, so these are
not peak-clock numbers.
