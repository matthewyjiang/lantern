"""Recorded point clouds for Lantern's dev server and headless proofs.

Each fixture is a real scan from the Point Cloud Library's data repository (BSD-3-Clause,
https://github.com/PointCloudLibrary/data), fetched at a pinned commit and checked by SHA-256,
then cached in fixtures/ (gitignored) as a decoded .npy. Nothing here generates points.
"""

import hashlib
import pathlib
import struct
import urllib.request

import numpy as np

COMMIT = "5c26bdd0591ba150b91858b5c9fe5e91cb39ae86"
CACHE = pathlib.Path(__file__).resolve().parent.parent / "fixtures"

# name: (path in the repo, sha256 of the .pcd, what it is)
FIXTURES = {
    # 1,366,408 returns, airborne LiDAR over terrain (PCL's progressive morphological filter data).
    "terrain": ("terrain/CSite1_orig-utm.pcd", "8947298a57ea2e86f9d8c414f82c5be21794c56555c7851c7ad032f7f8c7fabe", "airborne LiDAR, 1.37 M points"),
    # 460,400 returns from a SICK LMS400 sweeping a table top, with intensity.
    "table": ("tutorials/table_scene_lms400.pcd", "e285d415641e0d9de695b611db874cc8fe995e8089b77a50d6056d24d8cbcc58", "SICK LMS400 table scan, 460 k points, intensity"),
    # 640x480 RGB-D frame of a scene with objects, packed rgb.
    "rgbd": ("tutorials/region_growing_rgb_tutorial.pcd", "4403819d67223b02b9e0d3051f45b2d54eb4d6d401927597ff42ea7078ebc30b", "RGB-D frame, 307 k points, rgb"),
}


def lzf_decompress(data: bytes, out_len: int) -> bytes:
    """liblzf's format, which PCD's binary_compressed uses."""
    out = bytearray(out_len)
    i = o = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:  # literal run of ctrl + 1 bytes
            k = ctrl + 1
            out[o:o + k] = data[i:i + k]
            i += k
            o += k
        else:  # back reference
            length = ctrl >> 5
            ref = o - ((ctrl & 0x1F) << 8) - 1
            if length == 7:
                length += data[i]
                i += 1
            ref -= data[i]
            i += 1
            length += 2
            if ref + length <= o:
                out[o:o + length] = out[ref:ref + length]
            else:  # overlapping copy repeats the pattern
                for j in range(length):
                    out[o + j] = out[ref + j]
            o += length
    if o != out_len:
        raise ValueError(f"LZF: expected {out_len} bytes, got {o}")
    return bytes(out)


def read_pcd(raw: bytes) -> np.ndarray:
    """A structured array with one field per PCD field (count 1 fields only)."""
    header = {}
    at = 0
    while True:
        end = raw.index(b"\n", at)
        line = raw[at:end].decode().strip()
        at = end + 1
        if not line or line.startswith("#"):
            continue
        key, *vals = line.split()
        header[key] = vals
        if key == "DATA":
            break
    kinds = {("F", "4"): "<f4", ("F", "8"): "<f8", ("U", "1"): "u1", ("U", "2"): "<u2", ("U", "4"): "<u4", ("I", "1"): "i1", ("I", "2"): "<i2", ("I", "4"): "<i4"}
    names = header["FIELDS"]
    dtypes = [kinds[(t, s)] for t, s in zip(header["TYPE"], header["SIZE"])]
    count = int(header["POINTS"][0])
    mode = header["DATA"][0]
    if mode == "binary":
        return np.frombuffer(raw[at:], dtype=list(zip(names, dtypes)), count=count).copy()
    if mode != "binary_compressed":
        raise ValueError(f"PCD DATA {mode} not supported")
    packed_len, raw_len = struct.unpack("<II", raw[at:at + 8])
    body = lzf_decompress(raw[at + 8:at + 8 + packed_len], raw_len)
    # binary_compressed stores each field as one contiguous column.
    out = np.empty(count, dtype=list(zip(names, dtypes)))
    off = 0
    for name, dt in zip(names, dtypes):
        size = np.dtype(dt).itemsize * count
        out[name] = np.frombuffer(body[off:off + size], dtype=dt)
        off += size
    return out


def load(name: str) -> np.ndarray:
    path, sha, _ = FIXTURES[name]
    CACHE.mkdir(exist_ok=True)
    npy = CACHE / f"{name}.npy"
    if npy.exists():
        return np.load(npy)
    pcd = CACHE / pathlib.Path(path).name
    if not pcd.exists():
        url = f"https://raw.githubusercontent.com/PointCloudLibrary/data/{COMMIT}/{path}"
        print(f"fetching {url}")
        pcd.write_bytes(urllib.request.urlopen(url).read())
    raw = pcd.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != sha:
        raise SystemExit(f"{pcd}: sha256 {digest}, expected {sha}")
    cloud = read_pcd(raw)
    np.save(npy, cloud)
    print(f"{name}: {len(cloud)} points, fields {cloud.dtype.names}, sha256 {digest}")
    return cloud


def dominant_plane(xyz: np.ndarray, tol: float, tries: int = 300) -> tuple[np.ndarray, float, float]:
    """RANSAC over every 20th point: the plane (unit normal n, offset d, n.p + d = 0) most
    points lie within `tol` metres of, with n flipped to face the sensor at the origin (d > 0),
    and the fraction of points on it. d is then the sensor's height above the plane."""
    sample = xyz[::20]
    rng = np.random.default_rng(0)
    best = (-1, None, 0.0)
    for _ in range(tries):
        p = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-12:
            continue
        n /= np.linalg.norm(n)
        d = -float(n @ p[0])
        inliers = int((np.abs(sample @ n + d) < tol).sum())
        if inliers > best[0]:
            best = (inliers, n, d)
    inliers, n, d = best
    if d < 0:
        n, d = -n, -d
    return n, d, inliers / len(sample)


if __name__ == "__main__":
    for n in FIXTURES:
        c = load(n)
        xyz = np.stack([c["x"], c["y"], c["z"]], 1).astype(np.float64)
        ok = np.isfinite(xyz).all(1)
        lo, hi = xyz[ok].min(0), xyz[ok].max(0)
        print(f"{n}: {len(c)} points ({ok.sum()} finite), min {lo.round(2)}, max {hi.round(2)}")
        if n != "terrain":
            normal, height, frac = dominant_plane(xyz[ok], tol=0.01)
            print(f"  dominant plane: normal {normal.round(3)} (toward the sensor), sensor {height:.3f} m above it, {frac:.0%} of points")
