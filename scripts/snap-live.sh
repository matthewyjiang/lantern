#!/usr/bin/env bash
# Headless snapshots of Lantern against live data: starts scripts/serve.py with the given
# arguments, waits for it, then renders each scene `--realtime` (so the WebSocket client runs
# in real time) and snaps it once data has arrived. Settings go to a throwaway config dir.
# Usage: scripts/snap-live.sh OUT_DIR "SERVE ARGS" SCENE...   (SCENE "-" is the default state;
#        Rune flags may follow a colon, as "settings:--script x.json")
# Env: PY (python with foxglove-sdk, default /tmp/fxenv/bin/python), AT (seconds, default 3),
#      URL (Lantern's server, default the SDK server serve.py starts), SETTINGS (a whole
#      settings.json to start from, instead of URL).
set -euo pipefail
cd "$(dirname "$0")/.."
out=$1 serve_args=$2
shift 2
py=${PY:-/tmp/fxenv/bin/python}
at=${AT:-3}
mkdir -p "$out"
cfg=$(mktemp -d)
if [[ -n ${SETTINGS:-} ]]; then
  mkdir -p "$cfg/lantern" && printf '%s\n' "$SETTINGS" > "$cfg/lantern/settings.json"
elif [[ -n ${URL:-} ]]; then
  mkdir -p "$cfg/lantern" && printf '{"url": "%s"}\n' "$URL" > "$cfg/lantern/settings.json"
fi
# shellcheck disable=SC2086
"$py" -u scripts/serve.py $serve_args > "$out/serve.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null; wait $server 2>/dev/null || true; rm -rf "$cfg"' EXIT
for _ in $(seq 100); do grep -q "^serving\|^publishing" "$out/serve.log" && break; sleep 0.1; done
grep -q "^serving\|^publishing" "$out/serve.log" || { echo "serve.py didn't start:"; cat "$out/serve.log"; exit 1; }
for spec in "$@"; do
  name=${spec%%:*}
  extra=()
  [[ $spec == *:* ]] && read -r -a extra <<< "${spec#*:}"
  flags=() file=default
  [[ $name != - ]] && flags=(--scene "$name") file=$name
  XDG_CONFIG_HOME=$cfg target/release/lantern --realtime "${flags[@]}" "${extra[@]}" --snap "$out/$file.png" --at "$at" > "$out/$file.log" 2>&1 \
    || { echo "$file: run failed, see $out/$file.log"; exit 1; }
  echo "$out/$file.png: $(grep -o '[0-9]* points and [0-9]* robot vertices drawn' "$out/$file.log")"
done
