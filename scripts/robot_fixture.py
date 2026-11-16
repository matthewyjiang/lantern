"""The KUKA LBR Med 7 as a plain URDF plus its visual meshes, for Lantern's dev server and
headless proofs. Lantern reads URDF, not xacro, so this expands the xacro once.

Source: a directory holding med7.urdf.xacro and meshes/med7/visual/*.stl, such as pyfri's
examples/robots (Apache-2.0). Writes fixtures/med7/ (gitignored):

    fixtures/med7/med7.urdf
    fixtures/med7/meshes/med7/visual/link_{0..7}.stl   (the URDF's relative mesh paths)

and the URDF alone (no meshes) to src/robot/testdata/med7.urdf, for the kinematics test.

Usage: python scripts/robot_fixture.py ROBOTS_DIR [XACRO]
       (XACRO: the xacro executable, default `xacro` on PATH)
"""

import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "fixtures" / "med7"
TEST = ROOT / "src/robot/testdata/med7.urdf"


def git(robots: pathlib.Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(robots), *args], capture_output=True, text=True).stdout.strip()


def main(robots: pathlib.Path, xacro: str) -> None:
    if not (robots / "med7.urdf.xacro").exists():
        raise SystemExit(f"{robots}: no med7.urdf.xacro here")
    if shutil.which(xacro) is None:
        raise SystemExit(f"{xacro}: not found (pip install xacro, or pass its path)")
    # Where the description came from, for the test copy's header: owner/repo@commit when it's a
    # git checkout, else the directory's name.
    remote = git(robots, "remote", "get-url", "origin").removesuffix(".git").replace(":", "/")
    commit = git(robots, "rev-parse", "HEAD")
    origin = f"{'/'.join(remote.split('/')[-2:])}@{commit}" if remote and commit else robots.name
    xacro_file = git(robots, "rev-parse", "--show-prefix") + "med7.urdf.xacro"
    urdf = subprocess.run([xacro, str(robots / "med7.urdf.xacro")], capture_output=True, text=True, check=True).stdout
    meshes = OUT / "meshes/med7/visual"
    meshes.mkdir(parents=True, exist_ok=True)
    TEST.parent.mkdir(parents=True, exist_ok=True)
    for stl in sorted((robots / "meshes/med7/visual").glob("*.stl")):
        shutil.copy2(stl, meshes / stl.name)
    # Only visuals: Lantern doesn't draw collision geometry, so those paths are never opened.
    (OUT / "med7.urdf").write_text(urdf)
    print(f"wrote {OUT / 'med7.urdf'} ({len(urdf)} B) and {len(list(meshes.glob('*.stl')))} meshes, from {origin}")
    # The forward-kinematics test's copy: the same URDF, xacro's banner (which names this
    # machine's path) swapped for where it came from.
    body = urdf[urdf.index("<robot"):]
    note = (
        '<?xml version="1.0" ?>\n'
        f"<!-- KUKA LBR Med 7, expanded from {origin}\n"
        f"     {xacro_file} (Apache-2.0) by scripts/robot_fixture.py. -->\n"
    )
    TEST.write_text(note + body)
    print(f"wrote {TEST}")


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    main(pathlib.Path(sys.argv[1]).expanduser(), sys.argv[2] if len(sys.argv) == 3 else "xacro")
