#!/bin/sh
# Usage: curl -fsSL https://matthewyjiang.github.io/lantern/install.sh | sh
# Env: LANTERN_VERSION (default: latest release), LANTERN_INSTALL_DIR (default: ~/.local/bin)
set -eu

fail() {
  echo "lantern install: $1" >&2
  exit 1
}

# Called on the last line, so a truncated download runs nothing.
main() {
  case $(uname -sm) in
    "Linux x86_64") arch=x86_64 ;;
    "Linux aarch64") arch=aarch64 ;;
    *) fail "no release for $(uname -sm). Build from source: https://matthewyjiang.github.io/lantern/getting-started/" ;;
  esac

  releases=https://github.com/matthewyjiang/lantern/releases
  url=$releases/latest/download
  [ -z "${LANTERN_VERSION:-}" ] || url=$releases/download/$LANTERN_VERSION
  archive=lantern-$arch-unknown-linux-gnu.tar.gz
  dir=${LANTERN_INSTALL_DIR:-$HOME/.local/bin}
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT

  echo "Downloading $url/$archive"
  for f in "$archive" "$archive.sha256"; do
    curl -fsSL "$url/$f" -o "$tmp/$f" || fail "can't download $url/$f"
  done
  (cd "$tmp" && sha256sum -c "$archive.sha256" > /dev/null) || fail "checksum mismatch, nothing installed"
  mkdir -p "$dir"
  tar -xzf "$tmp/$archive" -C "$dir" lantern
  echo "Installed $dir/lantern"
}

main
