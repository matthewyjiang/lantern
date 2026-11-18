#!/usr/bin/env bash
# Times Lantern headless against live data: the recorded 1.37 M-point terrain scan
# (scripts/serve.py over the Foxglove SDK), Rune's CPU frame time and the viewport's GPU
# timestamps, per scene, over several trials. Usage: scripts/bench.sh [trials]  (default 3)
#
# - Draw rows: `--realtime --frames 600` (10 s paced at 60 Hz) with the cloud sent once, on subscribe,
#   so each run ingests one cloud and then draws it; no pixels are read back.
# - `+readback` adds a PNG readback every frame (`--out`), to show the GPU number excludes it.
# - `ingest-10hz` sends the cloud at 10 Hz for the whole run: what a live sensor costs.
# - `camera-10hz` adds a 640x400 camera image and its annotations at 10 Hz: decoded on the source
#   thread, annotations drawn into a copy and uploaded on the UI thread (in Rune's frame time).
# - `unpaced` drops `--realtime`: how many frames/s the loop can do.
# - `+robot` rows load the KUKA Med 7 (fixtures/med7, scripts/robot_fixture.py; 0.5 M vertices),
#   posed by serve.py's joint states: what the arm adds to the pass, with and without the points.
# Writes out/bench.tsv; raw logs go to out/bench/. PY as in snap-live.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
trials=${1:-3}
bin=target/release/lantern
py=${PY:-/tmp/fxenv/bin/python}
mkdir -p out/bench
cfg=$(mktemp -d)
server=
serve() { # serve.py args
  [[ -n $server ]] && { kill "$server"; wait "$server" 2>/dev/null || true; }
  "$py" -u scripts/serve.py "$@" > out/bench/serve.log 2>&1 &
  server=$!
  for _ in $(seq 100); do grep -q "^serving" out/bench/serve.log && return; sleep 0.1; done
  echo "serve.py didn't start"; cat out/bench/serve.log; exit 1
}
trap '[[ -n $server ]] && kill $server 2>/dev/null; wait 2>/dev/null || true; rm -rf "$cfg"' EXIT
printf "config\ttrial\tdrawn\tcpu_mean\tcpu_p50\tcpu_p95\tcpu_max\tgpu_p50\tgpu_p95\tgpu_n\tuploads\tupload_mean_ms\tgap_p95\tgap_max\tgaps_over_25ms\twall_s\n" > out/bench.tsv
run() { # name, extra args...
  local name=$1; shift
  for t in $(seq 1 "$trials"); do
    local log=out/bench/$name-$t.log
    rm -rf out/bench/frames "$cfg/lantern"
    [[ -n ${SETTINGS:-} ]] && mkdir -p "$cfg/lantern" && printf '%s\n' "$SETTINGS" > "$cfg/lantern/settings.json"
    local start; start=$(date +%s.%N)
    XDG_CONFIG_HOME=$cfg "$bin" "$@" > "$log" 2>&1
    local wall; wall=$(echo "$(date +%s.%N) - $start" | bc)
    local cpu gpu drawn uploads
    cpu=$(grep -oP 'frame cpu mean \K[0-9.]+ ms p50 [0-9.]+ p95 [0-9.]+ max [0-9.]+' "$log" | sed -E 's/ ms p50 | p95 | max /\t/g')
    drawn=$(grep -oP '\K[0-9]+(?= points and)' "$log")
    gpu=$(grep -oP 'frames drawing [0-9]+ points: mean [0-9.]+ ms p50 \K[0-9.]+ p95 [0-9.]+ max [0-9.]+ \(n=[0-9]+' "$log" | sed -E 's/ p95 /\t/; s/ max [0-9.]+ \(n=/\t/')
    uploads=$(grep -oP 'uploaded to the GPU: \K[0-9]+ \(mean [0-9.]+' "$log" | sed -E 's/ \(mean /\t/')
    gaps=$(grep -oP 'start-to-start mean [0-9.]+ ms p50 [0-9.]+ p95 \K[0-9.]+ max [0-9.]+ \(n=[0-9]+\), [0-9]+' "$log" | sed -E 's/ max /\t/; s/ \(n=[0-9]+\), /\t/')
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%.2f\n" "$name" "$t" "$drawn" "$cpu" "$gpu" "$uploads" "$gaps" "$wall" >> out/bench.tsv
  done
}
paced=(--realtime --frames 600)
serve --cloud terrain --hz 0
run full "${paced[@]}"
run no-points "${paced[@]}" --scene no-points
run half "${paced[@]}" --scene half
run quarter "${paced[@]}" --scene quarter
run size-1 "${paced[@]}" --scene size-1
run size-4 "${paced[@]}" --scene size-4
# 120 frames: each writes a ~2 MB PNG.
run full+readback --realtime --frames 120 --out out/bench/frames
rm -rf out/bench/frames
run unpaced --frames 6000
serve --cloud terrain --hz 0 --robot-pose home
robot="{\"layout\": {\"panel\": \"3d\", \"robot\": {\"urdf\": \"$PWD/fixtures/med7/med7.urdf\"}}}"
SETTINGS=$robot run full+robot "${paced[@]}"
SETTINGS=$robot run robot-only "${paced[@]}" --scene no-points
serve --cloud terrain --hz 10
run ingest-10hz "${paced[@]}"
serve --cloud terrain --hz 10 --camera
run camera-10hz "${paced[@]}"
column -t -s $'\t' out/bench.tsv


# GPU clocks every 100 ms through one paced and one unpaced run (the draw rows' clock state).
serve --cloud terrain --hz 0
for mode in paced unpaced; do
  args=("${paced[@]}"); [[ $mode == unpaced ]] && args=(--frames 6000)
  rm -rf "$cfg/lantern"
  nvidia-smi --query-gpu=clocks.gr,clocks.max.gr,pstate,utilization.gpu --format=csv,noheader -lms 100 > "out/bench/clocks-$mode.csv" &
  smi=$!
  XDG_CONFIG_HOME=$cfg "$bin" "${args[@]}" > "out/bench/clocks-$mode.log" 2>&1
  kill $smi; wait $smi 2>/dev/null || true
  echo "$mode: $(grep -o 'frames drawing.*points: mean [0-9.]* ms p50 [0-9.]*' "out/bench/clocks-$mode.log")"
  sort "out/bench/clocks-$mode.csv" | uniq -c | sort -rn | head -4
done
