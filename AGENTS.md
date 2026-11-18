# AGENTS.md

Notes for coding agents working on Lantern. The user docs are the Starlight site in `docs/`.

## Commits

Write commit subjects and PR titles as [Conventional Commits](https://www.conventionalcommits.org/):
`type(scope): summary`. The scope is optional. A `!` after the type marks a breaking change.

| Type | For |
| --- | --- |
| `feat` | Something a user can now do |
| `fix` | A bug fix |
| `perf` | A change that only makes something faster |
| `docs` | Docs only |
| `refactor`, `test`, `style` | Code changes with no change in behaviour |
| `ci`, `build`, `chore` | Workflows, dependencies, housekeeping |

Squash-merge PRs. The PR's title becomes its one commit on `main`, so the title follows the same
format. Release notes are generated from the subjects on `main` (`scripts/release-notes.sh`):
`feat`, `fix`, `perf`, `docs` and breaking changes are listed, and the other types are left out.

## Compatibility

Until 1.0, don't keep backward compatibility: no migrations for old `settings.json` or layout
files, no aliases for renamed CLI flags, no shims for old APIs. Change the format and move on.
Mark a breaking change in its commit subject (`feat!:`) or with a `BREAKING CHANGE:` footer, so
the release notes list it.

## Checks

Run what CI runs before pushing:

```sh
cargo clippy --release --all-targets --locked -- -D warnings
cargo test --release --locked
shellcheck scripts/*.sh
```

## Releases

The maintainer tags releases by hand. Don't create or push a version tag, and don't add tooling
that bumps versions or tags on its own.

To prepare a release, set `version` in `Cargo.toml`, run `cargo build` so `Cargo.lock` follows,
and merge that. Pushing the tag `v<version>` then runs `.github/workflows/release.yml`, which
builds the Linux binaries and publishes the GitHub release with its notes.
