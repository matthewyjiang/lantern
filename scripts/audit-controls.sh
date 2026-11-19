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
# Data lands within ~0.3 s of start; inputs start at 0.5 s or later and the snap is at 3 s.
at=3
# Viewport in device px at --scale 2 (window 1280×800 pt, under the 52 pt top bar, 10 pt
# padding): the whole width, and beside the 440 pt Settings page.
crop_view=2520x1476+20+104
crop_settings=1640x1476+20+104
# The Settings page itself, for checks that the page scrolls.
crop_page=880x1476+1680+104

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
# Taps at the given times, in turn: `taps 1.0 a 1.4 b`.
taps() { local e=() ; while (($#)); do e+=("$(ev "$1" "$2")"); shift 2; done; local IFS=,; echo "[${e[*]}]"; }
# The pointer off the panels (on the top bar), so no panel header shows: for checks that must
# land back on a baseline taken without a pointer.
away='{"t": 2.6, "type": "move", "x": 640, "y": 26}'
with_away() { echo "${1%]}, $away]"; }
open='{"t": 0.5, "type": "tap", "key": "open-settings"}'
# The Settings page, scrolled to its end (the Layout section): taps land only on what's on screen.
send() { echo "[$open, {\"t\": 1.0, \"type\": \"wheel\", \"x\": 1060, \"y\": 400, \"dx\": 0, \"dy\": -2000}, $(ev 1.5 "$1")]"; }
# The 3D view's settings, open over the viewport's right part (from x 1775 px, under its
# settings button): checks compare the viewport left of them. `ptab TAB KEY [KEY]` opens them
# (after the events in $PRE if set), picks tab TAB (0 Scene, 1 Points, 2 Poses, 3 Robot,
# 4 Cards) and taps the keys in turn (a picker's field, then its option); `pclose` then closes
# them with Escape, for controls of what they cover.
popen='{"t": 0.5, "type": "tap", "key": "panel-settings"}'
crop_pop=1740x1476+20+104
ptab() { echo "[${PRE:+$PRE, }$popen, $(ev 0.9 "tab-$1")${2:+, $(ev 1.3 "$2")}${3:+, $(ev 1.7 "$3")}]"; }
pclose() { local e; e=$(ptab "$@"); echo "${e%]}, {\"t\": 2.1, \"type\": \"key\", \"name\": \"Escape\"}, $away]"; }
drag='{"t": 1.5, "type": "drag", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}'
# The 3D panel's menu (panel-menu): Reset view, Top down, then (after a separator) Split right,
# Split down, Show, each with a submenu of the panel kinds (3D view, Image, Data, Topics).
menu=panel-menu

snap base '[]'
snap settings "[$open]"
snap panel "[$popen]"
printf "%-22s %-8s %13s  %s\n" control expect "viewport diff" result
view() { check base "$crop_view" "$@"; }
sett() { check settings "$crop_settings" "$@"; }
pset() { check panel "$crop_pop" "$@"; }
# The live data must give the same frame twice, or a "changes" proves nothing.
view no-input-again   same    '[]'
# A panel's header shows while the pointer is over the panel, and goes with it.
view header-on-hover  changes '[{"t": 1.5, "type": "move", "x": 400, "y": 330}]'
view header-hides     same    "[{\"t\": 1.5, \"type\": \"move\", \"x\": 400, \"y\": 330}, $away]"
view top-down         changes "$(taps 1.0 $menu 1.5 $menu-item1)"
view reset-after-orbit same   "$(with_away "[$drag, $(ev 2.0 $menu), $(ev 2.3 $menu-item0)]")"
view menu-escape      same    "$(with_away "[$(ev 1.0 $menu), {\"t\": 1.5, \"type\": \"key\", \"name\": \"Escape\"}]")"
view orbit-drag       changes "[$drag]"
view shift-drag-pan   changes "[{\"t\": 1.45, \"hold\": \"shift\"}, $drag, {\"t\": 2.1, \"hold\": \"\"}]"
view middle-drag-pan  changes '[{"t": 1.5, "type": "drag", "button": "middle", "x": 400, "y": 330, "dx": 160, "dy": 40, "dur": 0.4}]'
view wheel-zoom       changes '[{"t": 1.5, "type": "wheel", "x": 400, "y": 330, "dx": 0, "dy": 120}]'
# The top bar's button opens the Settings page and closes it again.
view settings-toggle  same    "$(with_away "[$open, $(ev 1.5 open-settings)]")"
# The 3D view's settings, tab by tab. Topics sort by name: /markers, /objects are the poses.
pset fixed-frame      changes "$(ptab 0 fixed-frame-trigger option1)"
for k in grid grid-track frames labels axis-0; do
  pset "$k"           changes "$(ptab 0 $k)"
done
for k in points color-1 size-3 decimate-2; do
  pset "$k"           changes "$(ptab 1 $k)"
done
for k in poses pose-0 pose-1; do
  pset "$k"           changes "$(ptab 2 $k)"
done
# The cards (no Image or Data panel in the layout) are under the settings: each is checked once
# they're closed.
for k in show-image-card card-annotation-0 card-text card-text-topic-0 card-stats card-stats-topic-0; do
  view "$k"           changes "$(pclose 4 $k)"
done
# Its button again, Escape and a press outside close them, landing back on the baseline.
view panel-toggle     same    "$(with_away "[$popen, $(ev 1.5 panel-settings)]")"
view panel-escape     same    "$(with_away "[$popen, {\"t\": 1.5, \"type\": \"key\", \"name\": \"Escape\"}]")"
view panel-outside    same    "$(with_away "[$popen, {\"t\": 1.5, \"type\": \"tap\", \"x\": 300, \"y\": 700}]")"
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

# Layout, from panels' menus: splitting adds a panel, and closing it lands back on the
# baseline once the viewport has settled from resizing twice (by 4.5 s). A preset replaces the
# layout.
view split-right      changes "$(taps 1.0 $menu 1.3 $menu-item3 1.6 $menu-item3-sub-item2)"
at=5 view split-then-close same "$(with_away "$(taps 1.0 $menu 1.3 $menu-item3 1.6 $menu-item3-sub-item2 2.0 panel-1-menu 2.3 panel-1-menu-item4)")"
sett layout-preset    changes "$(send layout-preset-dashboard)"
# Dragging a divider resizes the panels beside it; the 3D view is the one that moves the camera.
dash='{"layout": {"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}}'
snap base-dash '[]' "$dash"
dview() { check base-dash "$crop_view" "$@" "$dash"; }
dview divider-drag    changes '[{"t": 1.5, "type": "drag", "key": "panel-divider-0", "dx": -150, "dy": 0, "dur": 0.4}]'
# Weights are shares: 0.5 and 0.25 fill the window as 2 and 1 do.
snap base-thirds '[]' '{"layout": {"row": [{"panel": "3d", "weight": 2}, {"panel": "image"}]}}'
check base-thirds "$crop_view" small-weights same '[]' '{"layout": {"row": [{"panel": "3d", "weight": 0.5}, {"panel": "image", "weight": 0.25}]}}'
# Showing the 3D view in the Data panel (its menu's Show) swaps it with the 3D panel.
dview show-3d-swaps   changes "$(with_away "$(taps 1.0 panel-1-1-menu 1.3 panel-1-1-menu-item2 1.6 panel-1-1-menu-item2-sub-item0)")"
# Dragging a panel by its header's name: over another panel the two swap places, live; dragged
# back, they swap back; Escape puts the layout back; over itself nothing moves. The 3D panel's
# handle is at about (60, 76) pt, the Data panel's middle at (1060, 600). `hpath X Y ...` holds
# the handle and moves through the points, 0.3 s apart, then lets go (after the events in $MID).
hpath() {
  local evs=("{\"t\": 1.0, \"type\": \"move\", \"x\": 60, \"y\": 76}" "{\"t\": 1.1, \"type\": \"down\", \"x\": 60, \"y\": 76}") t=11 x y
  while (($#)); do x=$1 y=$2; t=$((t + 3)); evs+=("{\"t\": $((t / 10)).$((t % 10)), \"type\": \"move\", \"x\": $x, \"y\": $y}"); shift 2; done
  t=$((t + 3))
  evs+=(${MID:+"$MID"} "{\"t\": $((t / 10)).$((t % 10)), \"type\": \"up\", \"x\": $x, \"y\": $y}" "$away")
  local IFS=,; echo "[${evs[*]}]"
}
dview drag-swap       changes "$(hpath 200 200 1060 600)"
dview drag-and-back   same    "$(hpath 200 200 1060 600 300 400)"
dview drag-escape     same    "$(MID='{"t": 1.8, "type": "key", "name": "Escape"}' hpath 200 200 1060 600)"
dview drag-onto-self  same    "$(hpath 200 200 300 300)"

# Robot (the Med 7 from scripts/robot_fixture.py, posed by serve.py's joint states), on the
# Robot tab. The robot would be under the settings: a middle-button pan first moves the scene
# left of them.
robot="{\"layout\": {\"panel\": \"3d\", \"robot\": {\"urdf\": \"$PWD/fixtures/med7/med7.urdf\"}}}"
[[ -f fixtures/med7/med7.urdf ]] || { echo "no fixtures/med7: run scripts/robot_fixture.py"; exit 1; }
pan='{"t": 0.2, "type": "drag", "button": "middle", "x": 400, "y": 330, "dx": -300, "dy": 0, "dur": 0.2}'
snap panel-robot "$(PRE=$pan ptab 3)" "$robot"
rpset() { check panel-robot "$crop_pop" "$@" "$robot"; }
rpset robot              changes "$(PRE=$pan ptab 3 robot)"
rpset robot-track        changes "$(PRE=$pan ptab 3 robot-track)"
rpset robot-opacity      changes "$(PRE=$pan ptab 3 robot-opacity-3)"
# Picking the topic Auto already uses keeps the pose: switching must not drop received joints.
rpset joints-explicit    same    "$(PRE=$pan ptab 3 joint-topic-trigger option1)"
# Typing the path and pressing Enter loads the robot (from no robot, so the arm appears).
# Typing runs at about 30 characters a second: Enter waits for the whole path.
snap panel-pan "$(PRE=$pan ptab 3)"
at=5 check panel-pan "$crop_pop" urdf-enter changes "$(e=$(PRE=$pan ptab 3 urdf); echo "${e%]}, {\"t\": 1.4, \"type\": \"type\", \"text\": \"$PWD/fixtures/med7/med7.urdf\"}, {\"t\": 3.8, \"type\": \"key\", \"name\": \"Enter\"}]")"
exit $fail
