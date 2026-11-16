"""The KUKA LBR Med 7 and Med 14 as plain URDF with their visual meshes, for Lantern's dev server
and headless proofs. Lantern reads URDF, not xacro, so this expands the xacro once.

Source: a directory holding med{7,14}.urdf.xacro and meshes/med{7,14}/visual/*.stl, such as
pyfri's examples/robots (Apache-2.0). Writes, for each ARM in med7 and med14 (gitignored):

    fixtures/ARM/ARM.urdf
    fixtures/ARM/meshes/ARM/visual/link_{0..7}.stl   (the URDF's relative mesh paths)

and the URDFs alone (no meshes) to src/robot/testdata/med{7,14}.urdf, for the kinematics tests.
The two arms share joint names, which is why Lantern compares a model's kinematics, not names.

Usage: python scripts/robot_fixture.py ROBOTS_DIR [XACRO]
       (XACRO: the xacro executable, default `xacro` on PATH)
"""

import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
TESTDATA = ROOT / "src/robot/testdata"
ARMS = [("med7", "Med 7"), ("med14", "Med 14")]


def git(robots: pathlib.Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(robots), *args], capture_output=True, text=True).stdout.strip()


def main(robots: pathlib.Path, xacro: str) -> None:
    missing = [f"{arm}.urdf.xacro" for arm, _ in ARMS if not (robots / f"{arm}.urdf.xacro").exists()]
    if missing:
        raise SystemExit(f"{robots}: no {', '.join(missing)} here")
    if shutil.which(xacro) is None:
        raise SystemExit(f"{xacro}: not found (pip install xacro, or pass its path)")
    # Where the descriptions came from, for the test copies' headers: owner/repo@commit when
    # it's a git checkout, else the directory's name.
    remote = git(robots, "remote", "get-url", "origin").removesuffix(".git").replace(":", "/")
    commit = git(robots, "rev-parse", "HEAD")
    origin = f"{'/'.join(remote.split('/')[-2:])}@{commit}" if remote and commit else robots.name
    prefix = git(robots, "rev-parse", "--show-prefix")
    TESTDATA.mkdir(parents=True, exist_ok=True)
    for arm, name in ARMS:
        text = subprocess.run([xacro, str(robots / f"{arm}.urdf.xacro")], capture_output=True, text=True, check=True).stdout
        out = ROOT / "fixtures" / arm
        meshes = out / f"meshes/{arm}/visual"
        meshes.mkdir(parents=True, exist_ok=True)
        for stl in sorted((robots / f"meshes/{arm}/visual").glob("*.stl")):
            shutil.copy2(stl, meshes / stl.name)
        # Only visuals: Lantern doesn't draw collision geometry, so those paths are never opened.
        (out / f"{arm}.urdf").write_text(text)
        print(f"wrote {out / f'{arm}.urdf'} ({len(text)} B) and {len(list(meshes.glob('*.stl')))} meshes, from {origin}")
        # xacro's banner (which names this machine's path) swapped for where it came from.
        note = (
            '<?xml version="1.0" ?>\n'
            f"<!-- KUKA LBR {name}, expanded from {origin}\n"
            f"     {prefix}{arm}.urdf.xacro (Apache-2.0) by scripts/robot_fixture.py. -->\n"
        )
        (TESTDATA / f"{arm}.urdf").write_text(note + text[text.index("<robot"):])
        print(f"wrote {TESTDATA / f'{arm}.urdf'}")


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    main(pathlib.Path(sys.argv[1]).expanduser(), sys.argv[2] if len(sys.argv) == 3 else "xacro")
