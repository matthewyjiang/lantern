#!/usr/bin/env bash
# Does every control change the view? Replays the recorded table scan (scripts/serve.py), drives
# each control headlessly through a Rune script (taps by element key, drags, wheel, typing),
# snaps, and counts viewport pixels that differ from a no-input baseline. A control that
# changes 0 pixels does nothing. `expect` is "changes" or "same" (Reset view after an orbit,
# and closing Settings or a panel's settings, must land back on the baseline).
# Usage: scripts/audit-controls.sh   (release build, ImageMagick `compare`, PY as snap-live.sh)
set -euo pipefail
cd "$(dirname "$0")/.."
bin=target/release/lantern
py=${PY:-/tmp/fxenv/bin/python}
dir=out/audit
rm -rf "$dir" && mkdir -p "$dir"
cfg=$(mktemp -d)
"$py" -u scripts/serve.py --cloud table --robot-pose reach --camera --status > "$dir/serve.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null; wait $server 2>/dev/null || true; rm -rf "$cfg"' EXIT
for _ in $(seq 100); do grep -q "^serving" "$dir/serve.log" && break; sleep 0.1; done
# Data lands within ~0.3 s of start; inputs start at 1.5 s and the snap is at 3 s.
at=3
# Viewport in device px at --scale 2 (window 1280×800 pt, 10 pt padding): beside the 296 pt
# side panel, and beside the 440 pt Settings page.
crop_side=1948x1560+20+20
crop_settings=1640x1560+20+20
# The Settings page itself, for checks that the page scrolls.
crop_page=880x1560+1660+20

snap() { # name, events JSON array, [settings JSON to start from]
  printf '%s\n' "$2" > "$dir/$1.json"
  rm -rf "$cfg/lantern"
  [[ -n ${3:-} ]] && mkdir -p "$cfg/lantern" && printf '%s\n' "$3" > "$cfg/lantern/settings.json"
  XDG_CONFIG_HOME=$cfg "$bin" --realtime --script "$dir/$1.json" --snap "$dir/$1.png" --at "$at" > "$dir/$1.log" 2>&1 \
    || { echo "$1: run failed, see $dir/$1.log"; exit 1; }
}
fail=0
check() { # baseline, crop, name, expect, events, [settings JSON]
  snap "$3" "$5" "${6:-}"
  local px; px=$(compare -metric AE -fuzz 1% -extract "$2" "$dir/$1.png" "$dir/$3.png" null: 2>&1 | cut -d' ' -f1 || true)
  local ok=PASS
  if [[ $4 == changes && $px == 0 ]] || [[ $4 == same && $px != 0 ]]; then ok=FAIL; fail=1; fi
  printf "%-22s %-8s %10s px  %s\n" "$3" "$4" "$px" "$ok"
}
ev() { echo "{\"t\": $1, \"type\": \"tap\", \"key\": \"$2\"}"; }
tap() { echo "[$(ev 1.5 "$1")]"; }
open='{"t": 0.5, "type": "tap", "key": "open-settings"}'
# The Settings page, scrolled to its end (the Layout section): taps land only on what's on screen.
send() { echo "[$open, {\"t\": 1.0, \"type\": \"wheel\", \"x\": 1060, \"y\": 400, \"dx\": 0, \"dy\": -2000}, $(ev 1.5 "$1")]"; }
# The 3D view's settings, open over the viewport's right part (380 pt wide, from its settings
# button): checks compare the viewport left of them. `pwtap DY KEY` scrolls them by DY pt first
# (the tap waits for the scroll to settle), after the events in $PRE if set; `pwclose DY KEY`
# then closes them with Escape, for controls of what they cover.
popen='{"t": 0.5, "type": "tap", "key": "panel-settings"}'
crop_pop=1160x1560+20+20
pwtap() { echo "[${PRE:+$PRE, }$popen, {\"t\": 1.0, \"type\": \"wheel\", \"x\": 780, \"y\": 400, \"dx\": 0, \"dy\": $1}, $(ev 2.0 "$2")]"; }
pwclose() { local e; e=$(pwtap "$@"); echo "${e%]}, {\"t\": 2.4, \"type\": \"key\", \"name\": \"Escape\"}]"; }
drag='{"t": 1.5, "type": "drag", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}'

snap base '[]'
snap settings "[$open]"
snap panel "[$popen]"
printf "%-22s %-8s %13s  %s\n" control expect "viewport diff" result
side() { check base "$crop_side" "$@"; }
sett() { check settings "$crop_settings" "$@"; }
pset() { check panel "$crop_pop" "$@"; }
# The live data must give the same frame twice, or a "changes" proves nothing.
side no-input-again   same    '[]'
# Side panel. Each toggle twice: its row (tapped at the centre) and the switch track.
for k in grid points frames poses image text stats; do
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
# The 3D view's settings, top to bottom. Topics sort by name: /markers, /objects are the poses.
pset fixed-frame      changes "$(pwtap 0 frame-1)"
for k in grid-full frames-full labels axis-0 points-full color-full-1; do
  pset "$k"           changes "$(pwtap 0 $k)"
done
pset size-full        changes "$(pwtap -600 size-full-3)"
pset density-full     changes "$(pwtap -600 decimate-full-2)"
pset poses-full       changes "$(pwtap -600 poses-full)"
pset pose-markers     changes "$(pwtap -600 pose-0)"
pset pose-objects     changes "$(pwtap -600 pose-1)"
# The cards (no Image or Data panel in the layout) are under the settings: each is checked once
# they're closed.
for k in show-image-card card-annotation-0 card-text card-text-topic-0 card-stats card-stats-topic-0; do
  side "$k"           changes "$(pwclose -4000 $k)"
done
# Done, Escape and a press outside close them, landing back on the baseline.
side panel-done       same    "[$popen, $(ev 1.5 close-panel-settings)]"
# (After Escape the pointer is still over the settings button: moved off, it isn't lit.)
side panel-escape     same    "[$popen, {\"t\": 1.5, \"type\": \"key\", \"name\": \"Escape\"}, {\"t\": 1.6, \"type\": \"move\", \"x\": 400, \"y\": 600}]"
side panel-outside    same    "[$popen, {\"t\": 1.5, \"type\": \"tap\", \"x\": 1100, \"y\": 700}]"
# The wheel over the page's side margins (x 850 and 1270 pt) scrolls the page, once it has
# settled from opening (by about 2.2 s; the snap is at 3 s), and never zooms the viewport.
for x in 850 1270; do
  wheel="[$open, {\"t\": 2.3, \"type\": \"wheel\", \"x\": $x, \"y\": 400, \"dx\": 0, \"dy\": -900}]"
  check settings "$crop_page" "wheel-$x-page" changes "$wheel"
  sett "wheel-$x-view"  same    "$wheel"
done
# Adding a server that refuses connections: the status chip says so.
sett url-enter        changes "[$open, $(ev 0.6 url), {\"t\": 0.7, \"type\": \"key\", \"cmd\": true, \"name\": \"a\"}, {\"t\": 0.8, \"type\": \"type\", \"text\": \"ws://127.0.0.1:9\"}, {\"t\": 2.2, \"type\": \"key\", \"name\": \"Enter\"}]"
# Start on an unreachable address (its connect hangs until the 3 s timeout), add the live server
# at 2.2 s and remove the dead one at 2.4 s: the dead connection's late failure must not blank
# the new one.
dead='{"connections": ["ws://10.255.255.1:8765"]}'
switch="[$open, $(ev 0.6 url), {\"t\": 0.8, \"type\": \"type\", \"text\": \"ws://127.0.0.1:8765\"}, {\"t\": 2.2, \"type\": \"key\", \"name\": \"Enter\"}, $(ev 2.4 remove-0)]"
at=6 sett switch-from-dead  same    "$switch" "$dead"

# Layout. Edit layout draws each panel's controls; with it switched off again, a split leaves a
# second panel, and closing that panel lands back on the baseline. A preset replaces the layout.
edit="$(ev 0.6 edit-layout)"
side edit-layout      changes "[$edit]"
side split-right      changes "[$edit, $(ev 1.0 panel-split-right), $(ev 1.5 edit-layout)]"
side split-then-close same    "[$edit, $(ev 1.0 panel-split-right), $(ev 1.5 panel-1-close), $(ev 2.0 edit-layout)]"
sett edit-layout-full changes "$(send edit-layout-full)"
sett layout-preset    changes "$(send layout-preset-dashboard)"
# Dragging a divider resizes the panels beside it; the 3D view is the one that moves the camera.
dash='{"layout": {"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}}'
snap base-dash '[]' "$dash"
check base-dash "$crop_side" divider-drag changes '[{"t": 1.5, "type": "drag", "key": "panel-divider-0", "dx": -150, "dy": 0, "dur": 0.4}]' "$dash"
# Weights are shares: 0.5 and 0.25 fill the window as 2 and 1 do.
snap base-thirds '[]' '{"layout": {"row": [{"panel": "3d", "weight": 2}, {"panel": "image"}]}}'
check base-thirds "$crop_side" small-weights same '[]' '{"layout": {"row": [{"panel": "3d", "weight": 0.5}, {"panel": "image", "weight": 0.25}]}}'
# Showing the 3D view in the Data panel swaps it with the 3D panel.
check base-dash "$crop_side" show-3d-swaps changes "[$edit, $(ev 1.0 panel-1-1-show-3d), $(ev 1.5 edit-layout)]" "$dash"

# Robot (the Med 7 from scripts/robot_fixture.py, posed by serve.py's joint states). Its side
# panel section only exists once it has loaded, so these start from settings that load it.
robot="{\"layout\": {\"panel\": \"3d\", \"robot\": {\"urdf\": \"$PWD/fixtures/med7/med7.urdf\"}}}"
[[ -f fixtures/med7/med7.urdf ]] || { echo "no fixtures/med7: run scripts/robot_fixture.py"; exit 1; }
snap base-robot '[]' "$robot"
rside() { check base-robot "$crop_side" "$@" "$robot"; }
rside robot           changes "$(tap robot)"
rside robot-track     changes "$(tap robot-track)"
rside robot-opacity   changes "$(tap robot-opacity-2)"
# In the 3D view's settings the robot would be under them: a middle-button pan first moves the
# scene left of them.
pan='{"t": 0.2, "type": "drag", "button": "middle", "x": 400, "y": 330, "dx": -300, "dy": 0, "dur": 0.2}'
snap panel-robot "[$pan, $popen]" "$robot"
rpset() { check panel-robot "$crop_pop" "$@" "$robot"; }
rpset robot-full         changes "$(PRE=$pan pwtap -1200 robot-full)"
rpset robot-opacity-full changes "$(PRE=$pan pwtap -1200 robot-opacity-full-3)"
# Picking the topic Auto already uses keeps the pose: switching must not drop received joints.
rpset joints-explicit same    "$(PRE=$pan pwtap -1200 joints-1)"
# Typing the path and pressing Enter loads the robot (from no robot, so the arm appears).
# Typing runs at about 30 characters a second: Enter waits for the whole path.
snap panel-pan "[$pan, $popen]"
at=5 check panel-pan "$crop_pop" urdf-enter changes "[$pan, $popen, {\"t\": 0.6, \"type\": \"wheel\", \"x\": 780, \"y\": 400, \"dx\": 0, \"dy\": -600}, $(ev 1.0 urdf), {\"t\": 1.1, \"type\": \"type\", \"text\": \"$PWD/fixtures/med7/med7.urdf\"}, {\"t\": 3.5, \"type\": \"key\", \"name\": \"Enter\"}]"
exit $fail
