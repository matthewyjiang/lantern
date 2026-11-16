"""The KUKA LBR Med 7 that kpd drives, as a plain URDF plus its visual meshes, for Lantern's dev
server and headless proofs. Lantern reads URDF, not xacro, so this expands kpd's xacro once.

Source: kuka-policy-deployment's pyfri submodule (lunarlab-gatech/pyfri, Apache-2.0),
examples/robots/med7.urdf.xacro and meshes/med7/visual/*.stl. Writes fixtures/med7/ (gitignored):

    fixtures/med7/med7.urdf
    fixtures/med7/meshes/med7/visual/link_{0..7}.stl   (the URDF's relative mesh paths)

and the URDF alone (no meshes) to src/robot/testdata/med7.urdf, for the kinematics test.

Usage: python scripts/robot_fixture.py [KPD_CHECKOUT]
       (default ~/.t3/projects/kuka-policy-deployment; xacro from its .venv, as kpd runs it)
"""

import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "fixtures" / "med7"
TEST = ROOT / "src/robot/testdata/med7.urdf"


def main(kpd: pathlib.Path) -> None:
    robots = kpd / "external/pyfri/examples/robots"
    xacro = kpd / ".venv/bin/xacro"
    if not (robots / "med7.urdf.xacro").exists() or not xacro.exists():
        raise SystemExit(f"{kpd}: needs external/pyfri (git submodule update --init) and .venv (uv sync)")
    commit = subprocess.run(["git", "-C", str(kpd / "external/pyfri"), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    urdf = subprocess.run([str(xacro), str(robots / "med7.urdf.xacro")], capture_output=True, text=True, check=True).stdout
    meshes = OUT / "meshes/med7/visual"
    meshes.mkdir(parents=True, exist_ok=True)
    TEST.parent.mkdir(parents=True, exist_ok=True)
    for stl in sorted((robots / "meshes/med7/visual").glob("*.stl")):
        shutil.copy2(stl, meshes / stl.name)
    # Only visuals: Lantern doesn't draw collision geometry, so those paths are never opened.
    (OUT / "med7.urdf").write_text(urdf)
    print(f"wrote {OUT / 'med7.urdf'} ({len(urdf)} B) and {len(list(meshes.glob('*.stl')))} meshes, from pyfri {commit}")
    # The forward-kinematics test's copy: the same URDF, xacro's banner (which names this
    # machine's path) swapped for where it came from.
    body = urdf[urdf.index("<robot"):]
    note = (
        '<?xml version="1.0" ?>\n'
        f"<!-- KUKA LBR Med 7, expanded from lunarlab-gatech/pyfri@{commit}\n"
        "     examples/robots/med7.urdf.xacro (Apache-2.0) by scripts/robot_fixture.py. -->\n"
    )
    TEST.write_text(note + body)
    print(f"wrote {TEST}")


if __name__ == "__main__":
    main(pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path.home() / ".t3/projects/kuka-policy-deployment")
