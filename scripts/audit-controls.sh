#!/usr/bin/env bash
# Does every control change the view? Drives each one headlessly through a Rune script (taps by
# element key, drags, wheel), snaps, and counts viewport pixels that differ from a no-input
# baseline. A control that changes 0 pixels does nothing. `expect` says what should happen:
# "changes" (differs from baseline) or "same" (Reset view after an orbit must land back on it).
# Usage: scripts/audit-controls.sh   (needs a release build; ImageMagick `compare`)
set -euo pipefail
cd "$(dirname "$0")/.."
bin=target/release/lantern
dir=out/audit
rm -rf "$dir" && mkdir -p "$dir"
at=1.6
# Viewport in device px at --scale 2: 10 pt padding, sidebar 296 pt, window 1280×800 pt.
crop=1948x1560+20+20

snap() { # name, events JSON array
  printf '%s\n' "$2" > "$dir/$1.json"
  "$bin" --script "$dir/$1.json" --snap "$dir/$1.png" --at "$at" > "$dir/$1.log" 2>&1 \
    || { echo "$1: run failed, see $dir/$1.log"; exit 1; }
}
diff_px() { compare -metric AE -fuzz 1% -extract "$crop" "$dir/base.png" "$dir/$1.png" null: 2>&1 | cut -d' ' -f1 || true; }

snap base '[]'
fail=0
check() { # name, expect, events
  snap "$1" "$3"
  local px; px=$(diff_px "$1")
  local ok=PASS
  if [[ $2 == changes && $px == 0 ]] || [[ $2 == same && $px != 0 ]]; then ok=FAIL; fail=1; fi
  printf "%-22s %-8s %10s px  %s\n" "$1" "$2" "$px" "$ok"
}
tap() { echo "[{\"t\": 0.1, \"type\": \"tap\", \"key\": \"$1\"}]"; }
drag='{"t": 0.1, "type": "drag", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}'

printf "%-22s %-8s %13s  %s\n" control expect "viewport diff" result
# Each toggle twice: its row (tapped at the centre) and the switch track people aim for.
for k in grid points frames; do
  check "$k"       changes "$(tap $k)"
  check "$k-track" changes "$(tap $k-track)"
done
check color-intensity changes "$(tap color-1)"
check size-1         changes "$(tap size-0)"
check size-4         changes "$(tap size-3)"
check density-half   changes "$(tap decimate-1)"
check density-eighth changes "$(tap decimate-3)"
check top-down       changes "$(tap top)"
check reset-after-orbit same "[$drag, {\"t\": 0.8, \"type\": \"tap\", \"key\": \"home\"}]"
check orbit-drag     changes "[$drag]"
check shift-drag-pan changes "[{\"t\": 0.05, \"hold\": \"shift\"}, $drag, {\"t\": 0.7, \"hold\": \"\"}]"
check middle-drag-pan changes "[{\"t\": 0.1, \"type\": \"drag\", \"button\": \"middle\", \"x\": 400, \"y\": 330, \"dx\": 160, \"dy\": 40, \"dur\": 0.4}]"
check wheel-zoom     changes '[{"t": 0.1, "type": "wheel", "x": 400, "y": 330, "dx": 0, "dy": 120}]'
exit $fail
