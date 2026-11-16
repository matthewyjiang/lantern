#!/usr/bin/env bash
# Does every control change the view? Replays the recorded table scan (scripts/serve.py), drives
# each control headlessly through a Rune script (taps by element key, drags, wheel, typing),
# snaps, and counts viewport pixels that differ from a no-input baseline. A control that
# changes 0 pixels does nothing. `expect` is "changes" or "same" (Reset view after an orbit,
# and Done after opening Settings, must land back on the baseline).
# Usage: scripts/audit-controls.sh   (release build, ImageMagick `compare`, PY as snap-live.sh)
set -euo pipefail
cd "$(dirname "$0")/.."
bin=target/release/lantern
py=${PY:-/tmp/fxenv/bin/python}
dir=out/audit
rm -rf "$dir" && mkdir -p "$dir"
cfg=$(mktemp -d)
"$py" -u scripts/serve.py --cloud table > "$dir/serve.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null; wait $server 2>/dev/null || true; rm -rf "$cfg"' EXIT
for _ in $(seq 100); do grep -q "^serving" "$dir/serve.log" && break; sleep 0.1; done
# Data lands within ~0.3 s of start; inputs start at 1.5 s and the snap is at 3 s.
at=3
# Viewport in device px at --scale 2 (window 1280×800 pt, 10 pt padding): beside the 296 pt
# side panel, and beside the 440 pt Settings page.
crop_side=1948x1560+20+20
crop_settings=1640x1560+20+20

snap() { # name, events JSON array
  printf '%s\n' "$2" > "$dir/$1.json"
  rm -rf "$cfg/lantern"
  XDG_CONFIG_HOME=$cfg "$bin" --realtime --script "$dir/$1.json" --snap "$dir/$1.png" --at "$at" > "$dir/$1.log" 2>&1 \
    || { echo "$1: run failed, see $dir/$1.log"; exit 1; }
}
fail=0
check() { # baseline, crop, name, expect, events
  snap "$3" "$5"
  local px; px=$(compare -metric AE -fuzz 1% -extract "$2" "$dir/$1.png" "$dir/$3.png" null: 2>&1 | cut -d' ' -f1 || true)
  local ok=PASS
  if [[ $4 == changes && $px == 0 ]] || [[ $4 == same && $px != 0 ]]; then ok=FAIL; fail=1; fi
  printf "%-22s %-8s %10s px  %s\n" "$3" "$4" "$px" "$ok"
}
ev() { echo "{\"t\": $1, \"type\": \"tap\", \"key\": \"$2\"}"; }
tap() { echo "[$(ev 1.5 "$1")]"; }
open='{"t": 0.5, "type": "tap", "key": "open-settings"}'
stap() { echo "[$open, $(ev 1.5 "$1")]"; }
# The same, after scrolling the page to its end (the Display section is below the fold).
send() { echo "[$open, {\"t\": 1.0, \"type\": \"wheel\", \"x\": 1060, \"y\": 400, \"dx\": 0, \"dy\": ${SCROLL_DY:--2000}}, $(ev 1.5 "$1")]"; }
drag='{"t": 1.5, "type": "drag", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}'

snap base '[]'
snap settings "[$open]"
printf "%-22s %-8s %13s  %s\n" control expect "viewport diff" result
side() { check base "$crop_side" "$@"; }
sett() { check settings "$crop_settings" "$@"; }
# The live data must give the same frame twice, or a "changes" proves nothing.
side no-input-again   same    '[]'
# Side panel. Each toggle twice: its row (tapped at the centre) and the switch track.
for k in grid points frames poses; do
  side "$k"       changes "$(tap $k)"
  side "$k-track" changes "$(tap $k-track)"
done
side color-height     changes "$(tap color-1)"
side size-1           changes "$(tap size-0)"
side size-4           changes "$(tap size-3)"
side density-half     changes "$(tap decimate-1)"
side density-eighth   changes "$(tap decimate-3)"
side top-down         changes "$(tap top)"
side reset-after-orbit same   "[$drag, $(ev 2.2 home)]"
side orbit-drag       changes "[$drag]"
side shift-drag-pan   changes "[{\"t\": 1.45, \"hold\": \"shift\"}, $drag, {\"t\": 2.1, \"hold\": \"\"}]"
side middle-drag-pan  changes '[{"t": 1.5, "type": "drag", "button": "middle", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}]'
side wheel-zoom       changes '[{"t": 1.5, "type": "wheel", "x": 400, "y": 330, "dx": 0, "dy": 120}]'
side settings-done    same    "[$open, $(ev 1.5 close-settings)]"
# Settings page. Topics sort by name: /markers, /objects, /points, /tf.
sett topic-markers    changes "$(stap topic-0-show)"
sett topic-objects    changes "$(stap topic-1-show)"
sett fixed-frame      changes "$(stap frame-1)"
sett labels           changes "$(stap labels)"
sett axis-5cm         changes "$(stap axis-0)"
sett color-field      changes "$(send color-full-1)"
sett size-full        changes "$(send size-full-3)"
sett density-full     changes "$(send decimate-full-2)"
sett grid-full        changes "$(send grid-full)"
sett url-enter        changes "[$open, $(ev 1.0 url), {\"t\": 1.1, \"type\": \"key\", \"ctrl\": true, \"name\": \"a\"}, {\"t\": 1.2, \"type\": \"type\", \"text\": \"ws://127.0.0.1:9\"}, {\"t\": 1.5, \"type\": \"key\", \"name\": \"Enter\"}]"
exit $fail
