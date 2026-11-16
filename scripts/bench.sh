#!/usr/bin/env bash
# Times Lantern headless: Rune's CPU frame time and the viewport's GPU timestamps, per scene,
# over several trials. Usage: scripts/bench.sh [frames] [trials]  (defaults 600, 3)
# `--frames N` alone reads no pixels back; the `+readback` row adds a PNG readback every frame.
# Writes out/bench.tsv; raw logs go to out/bench/.
set -euo pipefail
cd "$(dirname "$0")/.."
frames=${1:-600}
trials=${2:-3}
bin=target/release/lantern
mkdir -p out/bench
printf "config\ttrial\tdrawn\tcpu_mean\tcpu_p50\tcpu_p95\tgpu_mean\tgpu_p50\tgpu_p95\tgpu_n\tuntimed\tframes_per_s_wall\n" > out/bench.tsv
run() { # name, frames, extra args...
  local name=$1 n_frames=$2; shift 2
  for t in $(seq 1 "$trials"); do
    local log=out/bench/$name-$t.log
    rm -rf out/bench/frames
    "$bin" --frames "$n_frames" "$@" > "$log" 2>&1
    local cpu gpu drawn wall
    cpu=$(grep -oP 'frame cpu mean \K[0-9.]+ ms p50 [0-9.]+ p95 [0-9.]+' "$log" | sed -E 's/ ms p50 | p95 /\t/g')
    gpu=$(grep -oP 'gpu 3D pass \(timestamps\) mean \K[0-9.]+ ms p50 [0-9.]+ p95 [0-9.]+' "$log" | sed -E 's/ ms p50 | p95 /\t/g')
    local n untimed
    n=$(grep -oP '\K[0-9]+(?= frames timed)' "$log")
    untimed=$(grep -oP '\K[0-9]+(?= untimed)' "$log")
    drawn=$(grep -oP '\K[0-9]+(?= points drawn)' "$log")
    wall=$(grep -oP 'wall \(\K[0-9.]+(?= frames/s)' "$log" || echo "-")
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$name" "$t" "$drawn" "$cpu" "$gpu" "$n" "$untimed" "$wall" >> out/bench.tsv
  done
}
run full "$frames"
run no-points "$frames" --scene no-points
run half "$frames" --scene half
run quarter "$frames" --scene quarter
run size-1 "$frames" --scene size-1
run size-4 "$frames" --scene size-4
# 120 frames: each writes a ~2 MB PNG.
run full+readback 120 --out out/bench/frames
rm -rf out/bench/frames
column -t -s $'\t' out/bench.tsv

# The runs above are ~1 s each, too short to read clocks during. One long run, sampled mid-load:
"$bin" --frames 8000 > out/bench/clocks.log 2>&1 &
pid=$!
sleep 4
nvidia-smi --query-gpu=clocks.gr,clocks.max.gr,clocks.mem,pstate,utilization.gpu --format=csv | tee -a out/bench/clocks.log
wait $pid
grep -E "frame cpu mean|gpu 3D pass|wall" out/bench/clocks.log
