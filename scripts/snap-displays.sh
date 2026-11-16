#!/usr/bin/env bash
# Headless proofs for the displays beyond the cloud: joint states from either encoding, two
# servers at once, the robot from --urdf and from a robot_description topic (and a file that
# isn't the described robot), pose trails and expiry, the camera image with annotations, text
# and stats. Starts scripts/serve.py servers, snaps Lantern against them into out/displays/, and
# compares the frames a claim says must match.
# Usage: scripts/snap-displays.sh   (release build, ImageMagick `compare`, fixtures from
#        scripts/robot_fixture.py; PY as snap-live.sh)
set -euo pipefail
cd "$(dirname "$0")/.."
bin=target/release/lantern
py=${PY:-/tmp/fxenv/bin/python}
dir=out/displays
med7=$PWD/fixtures/med7/med7.urdf med14=$PWD/fixtures/med14/med14.urdf
[[ -f $med7 && -f $med14 ]] || { echo "no fixtures/med7 or med14: run scripts/robot_fixture.py"; exit 1; }
rm -rf "$dir" && mkdir -p "$dir"
cfg=$(mktemp -d)
servers=()
trap 'for p in "${servers[@]}"; do kill "$p" 2>/dev/null; wait "$p" 2>/dev/null || true; done; rm -rf "$cfg"' EXIT

serve() { # log name, port, serve.py args
  # shellcheck disable=SC2086
  "$py" -u scripts/serve.py --port "$2" $3 > "$dir/$1.serve.log" 2>&1 &
  servers+=($!)
  for _ in $(seq 100); do grep -q "^serving" "$dir/$1.serve.log" && return; sleep 0.1; done
  echo "serve.py $3 didn't start:"; cat "$dir/$1.serve.log"; exit 1
}
stop() {
  for p in "${servers[@]}"; do kill "$p" 2>/dev/null; wait "$p" 2>/dev/null || true; done
  servers=()
}
shot() { # name, settings JSON, seconds, [Lantern flags...]
  local name=$1 settings=$2 at=$3
  shift 3
  rm -rf "$cfg/lantern" && mkdir -p "$cfg/lantern" && printf '%s\n' "$settings" > "$cfg/lantern/settings.json"
  XDG_CONFIG_HOME=$cfg "$bin" --realtime "$@" --snap "$dir/$name.png" --at "$at" > "$dir/$name.log" 2>&1 \
    || { echo "$name: run failed, see $dir/$name.log"; exit 1; }
  echo "$dir/$name.png: $(grep -o '[0-9]* points and [0-9]* robot vertices drawn' "$dir/$name.log")"
}
fail=0
same() { # a, b: the viewport beside the side panel must match
  local px; px=$(compare -metric AE -fuzz 1% -extract 1948x1560+20+20 "$dir/$1.png" "$dir/$2.png" null: 2>&1 | cut -d' ' -f1 || true)
  local ok=PASS; [[ $px == 0 ]] || { ok=FAIL; fail=1; }
  printf "%-44s %8s px  %s\n" "$1 = $2" "$px" "$ok"
}
open_settings='--script '$dir/open-settings.json
printf '[{"t": 0.5, "type": "tap", "key": "open-settings"}]\n' > "$dir/open-settings.json"
local8765='"connections": ["ws://127.0.0.1:8765"]'

# Joint states: foxglove.JointStates (protobuf) poses the arm exactly as JSON JointState does.
serve joints-json 8765 "--cloud none --robot-pose reach"
shot joints-json "{$local8765, \"robot_urdf\": \"$med7\"}" 3
stop
serve joints-proto 8765 "--cloud none --robot-pose reach --joints proto"
shot joints-proto "{$local8765, \"robot_urdf\": \"$med7\"}" 3
stop
same joints-proto joints-json

# Two servers: the table scan from one, the arm, a moving pose and stats from the other; the
# robot from --urdf, the servers from --connect (the settings file names neither).
serve two-a 8765 "--cloud table"
serve two-b 8766 "--cloud none --robot-pose reach --status --motion"
two="--connect ws://127.0.0.1:8765 --connect ws://127.0.0.1:8766 --urdf $med7"
# shellcheck disable=SC2086
shot two-servers '{"trail": 15}' 6 $two
# shellcheck disable=SC2086
shot two-servers-settings '{"trail": 15}' 3 $two $open_settings
stop

# The robot a server describes: drawn with no file at all; with a file of the same robot, the
# same; with a different robot's file (same joint names), the described one wins and the status
# says so.
serve description 8765 "--cloud none --robot-pose reach --robot-description $med14"
shot description-only "{$local8765}" 3
shot description-same "{$local8765, \"robot_urdf\": \"$med14\"}" 3
shot description-mismatch "{$local8765, \"robot_urdf\": \"$med7\"}" 3
shot description-mismatch-settings "{$local8765, \"robot_urdf\": \"$med7\"}" 3 $open_settings
stop
same description-same description-only

# Poses over the table: /once is sent once, /tracked keeps moving. Kept forever, /once is there
# at 4 s; with a 2 s timeout it is gone.
serve poses 8765 "--cloud table --motion"
shot poses-never "{$local8765, \"pose_timeout\": 0, \"show_labels\": true}" 4 --scene top
shot poses-expire "{$local8765, \"pose_timeout\": 2, \"show_labels\": true}" 4 --scene top
stop

# Everything over the table scan: camera image and annotations, text and stats, a trail.
serve all 8765 "--cloud table --camera --status --motion"
shot all "{$local8765, \"trail\": 15}" 6
stop
exit $fail
