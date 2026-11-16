"""The KUKA LBR Med 7 and Med 14 as plain URDF with their visual meshes, for Lantern's dev server
and headless proofs. Lantern reads URDF, not xacro, so this expands pyfri's xacro once.

Source: lunarlab-gatech/pyfri (Apache-2.0), examples/robots/med{7,14}.urdf.xacro and
meshes/med{7,14}/visual/*.stl. Writes, for each ARM in med7 and med14 (gitignored):

    fixtures/ARM/ARM.urdf
    fixtures/ARM/meshes/ARM/visual/link_{0..7}.stl   (the URDF's relative mesh paths)

and the URDFs alone (no meshes) to src/robot/testdata/med{7,14}.urdf, for the kinematics tests.
The two arms share joint names, which is why Lantern compares a model's kinematics, not names.

Usage: python scripts/robot_fixture.py PYFRI_CHECKOUT [--xacro PATH]   (xacro: pip install xacro)
"""

import argparse
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parent.parent
TESTDATA = ROOT / "src/robot/testdata"


def expand(xacro: str, robots: pathlib.Path, arm: str) -> str:
    return subprocess.run([xacro, str(robots / f"{arm}.urdf.xacro")], capture_output=True, text=True, check=True).stdout


def main(pyfri: pathlib.Path, xacro: str) -> None:
    robots = pyfri / "examples/robots"
    if not (robots / "med7.urdf.xacro").exists():
        raise SystemExit(f"{pyfri}: not a pyfri checkout (no examples/robots/med7.urdf.xacro)")
    commit = subprocess.run(["git", "-C", str(pyfri), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    TESTDATA.mkdir(parents=True, exist_ok=True)
    for arm, name in [("med7", "Med 7"), ("med14", "Med 14")]:
        text = expand(xacro, robots, arm)
        out = ROOT / "fixtures" / arm
        meshes = out / f"meshes/{arm}/visual"
        meshes.mkdir(parents=True, exist_ok=True)
        for stl in sorted((robots / f"meshes/{arm}/visual").glob("*.stl")):
            shutil.copy2(stl, meshes / stl.name)
        # Only visuals: Lantern doesn't draw collision geometry, so those paths are never opened.
        (out / f"{arm}.urdf").write_text(text)
        print(f"wrote {out / f'{arm}.urdf'} ({len(text)} B) and {len(list(meshes.glob('*.stl')))} meshes, from pyfri {commit}")
        # xacro's banner (which names this machine's path) swapped for where it came from.
        note = (
            '<?xml version="1.0" ?>\n'
            f"<!-- KUKA LBR {name}, expanded from lunarlab-gatech/pyfri@{commit}\n"
            f"     examples/robots/{arm}.urdf.xacro (Apache-2.0) by scripts/robot_fixture.py. -->\n"
        )
        (TESTDATA / f"{arm}.urdf").write_text(note + text[text.index("<robot"):])
        print(f"wrote {TESTDATA / f'{arm}.urdf'}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pyfri", type=pathlib.Path)
    p.add_argument("--xacro", default="xacro")
    a = p.parse_args()
    main(a.pyfri, a.xacro)
