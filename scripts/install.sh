#!/bin/sh
# Installs a Lantern release binary for this machine: downloads the archive for its
# architecture from the GitHub release, checks its SHA-256, and puts `lantern` in ~/.local/bin.
# Usage: curl -fsSL https://raw.githubusercontent.com/matthewyjiang/lantern/main/scripts/install.sh | sh
# Env: LANTERN_VERSION (a release tag such as v0.1.0; default: the latest release),
#      LANTERN_INSTALL_DIR (default: ~/.local/bin)
set -eu

repo=https://github.com/matthewyjiang/lantern
docs=https://matthewyjiang.github.io/lantern/getting-started/

fail() {
  echo "lantern install: $1" >&2
  exit 1
}

# In a function, called on the last line: a download cut short runs nothing.
main() {
  os=$(uname -s)
  [ "$os" = Linux ] || fail "there are release binaries for Linux only, and this is $os. Build from source: $docs"
  machine=$(uname -m)
  case $machine in
    x86_64 | amd64) arch=x86_64 ;;
    aarch64 | arm64) arch=aarch64 ;;
    *) fail "there are release binaries for x86_64 and aarch64 only, and this is $machine. Build from source: $docs" ;;
  esac
  for tool in curl tar sha256sum; do
    command -v "$tool" > /dev/null || fail "needs $tool, which isn't installed"
  done

  archive=lantern-$arch-unknown-linux-gnu.tar.gz
  if [ -n "${LANTERN_VERSION:-}" ]; then
    url=$repo/releases/download/$LANTERN_VERSION
  else
    url=$repo/releases/latest/download
  fi
  dir=${LANTERN_INSTALL_DIR:-$HOME/.local/bin}
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT

  echo "Downloading $url/$archive"
  for file in "$archive" "$archive.sha256"; do
    curl -fsSL "$url/$file" -o "$tmp/$file" || fail "can't download $url/$file"
  done
  (cd "$tmp" && sha256sum -c "$archive.sha256" > /dev/null 2>&1) || fail "$archive doesn't match its SHA-256; nothing was installed"
  tar -xzf "$tmp/$archive" -C "$tmp"
  chmod 755 "$tmp/lantern"
  mkdir -p "$dir" || fail "can't create $dir"
  # Next to where it goes, so it's checked where it will run and the last step is one rename:
  # a lantern already there stays until the new one works, and one that's running keeps its file.
  new=$dir/.lantern.new
  mv "$tmp/lantern" "$new" || fail "can't write to $dir. Set LANTERN_INSTALL_DIR to a directory you can write to"
  # The release is built on Ubuntu 22.04, so it needs that glibc (2.35) or a newer one.
  version=$("$new" --version) || {
    rm -f "$new"
    fail "the release binary doesn't run here; nothing was installed. It needs glibc 2.35 or newer (Ubuntu 22.04, Debian 12). Build from source: $docs"
  }
  mv "$new" "$dir/lantern"
  echo "Installed $version to $dir/lantern"
  case ":$PATH:" in
    *":$dir:"*) ;;
    *) echo "$dir isn't on your PATH. Add it, or run $dir/lantern" ;;
  esac
}

main
