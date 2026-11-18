#!/usr/bin/env bash
# Release notes for a version tag, as Markdown on stdout: the commits on the tag's first-parent
# line since the version tag before it (all of them for the first tag), grouped by their
# Conventional Commits type. feat, fix, perf and docs each get a section, and a `!` after the
# type puts the commit under Breaking changes instead. chore, ci, build, test, style and
# refactor are left out. Any other subject is listed under Other changes, so a commit that
# doesn't follow the convention still shows.
# Usage: scripts/release-notes.sh TAG   (needs the history and tags: `fetch-depth: 0` in CI)
# Env: GITHUB_REPOSITORY (owner/name for the links, default matthewyjiang/lantern)
set -euo pipefail
cd "$(dirname "$0")/.."
tag=${1:?usage: scripts/release-notes.sh TAG}
repo=${GITHUB_REPOSITORY:-matthewyjiang/lantern}
git rev-parse -q --verify "$tag^{commit}" > /dev/null || { echo "release-notes: no tag or commit called $tag" >&2; exit 1; }
prev=$(git describe --tags --abbrev=0 --match 'v[0-9]*' "$tag^" 2> /dev/null || true)

sections=(breaking feat fix perf docs other)
declare -A title=([breaking]="Breaking changes" [feat]="Features" [fix]="Fixes" [perf]="Performance" [docs]="Documentation" [other]="Other changes")
declare -A lines
subject_re='^([a-z]+)(\(([^)]+)\))?(!)?: (.+)$'
while IFS= read -r subject; do
  section=other text=$subject
  if [[ $subject =~ $subject_re ]]; then
    type=${BASH_REMATCH[1]} scope=${BASH_REMATCH[3]} breaking=${BASH_REMATCH[4]}
    case $type in
      feat | fix | perf | docs) section=$type ;;
      chore | ci | build | test | style | refactor) section= ;;
    esac
    # An unknown type keeps its whole subject, under Other changes.
    if [[ $section != other ]]; then
      text=${BASH_REMATCH[5]}
      [[ -n $scope ]] && text="**$scope:** $text"
    fi
    [[ -n $breaking ]] && section=breaking
  fi
  if [[ -n $section ]]; then
    lines[$section]+="- $text"$'\n'
  fi
done < <(git log --first-parent --format=%s "${prev:+$prev..}$tag")

for section in "${sections[@]}"; do
  if [[ -n ${lines[$section]:-} ]]; then
    printf '## %s\n\n%s\n' "${title[$section]}" "${lines[$section]}"
  fi
done
fence='```'
printf '## Install\n\n%ssh\ncurl -fsSL https://raw.githubusercontent.com/%s/main/scripts/install.sh | LANTERN_VERSION=%s sh\n%s\n' "$fence" "$repo" "$tag" "$fence"
if [[ -n $prev ]]; then
  printf '\n**Full changelog:** https://github.com/%s/compare/%s...%s\n' "$repo" "$prev" "$tag"
fi
