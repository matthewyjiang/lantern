"""Minimal glTF 2.0 writer: triangle meshes with one base colour each, as GLB or as .gltf with
the buffer in a data: URI. `serve.py --objects` sends a GLB made here. Run as a script, it
writes the test models in src/mesh/testdata/:

    python3 scripts/glb.py
"""

import base64
import json
import struct
from pathlib import Path


def document(parts):
    """The glTF JSON and its binary buffer for `parts`: (positions, indices, rgba, translation),
    one mesh and node each. Positions are Y-up, as glTF has them; rgba is linear."""
    blob = b""
    accessors, views, meshes, materials, nodes = [], [], [], [], []
    for positions, indices, rgba, translation in parts:
        pos = b"".join(struct.pack("<3f", *p) for p in positions)
        idx = b"".join(struct.pack("<H", i) for i in indices)
        idx += b"\0" * (-len(idx) % 4)
        lo = [min(p[k] for p in positions) for k in range(3)]
        hi = [max(p[k] for p in positions) for k in range(3)]
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(pos)})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5126, "count": len(positions), "type": "VEC3", "min": lo, "max": hi})
        blob += pos
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": 2 * len(indices)})
        accessors.append({"bufferView": len(views) - 1, "componentType": 5123, "count": len(indices), "type": "SCALAR"})
        blob += idx
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": list(rgba)}})
        meshes.append({"primitives": [{"attributes": {"POSITION": len(accessors) - 2}, "indices": len(accessors) - 1, "material": len(materials) - 1}]})
        node = {"mesh": len(meshes) - 1}
        if translation != (0, 0, 0):
            node["translation"] = list(translation)
        nodes.append(node)
    doc = {
        "asset": {"version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(blob)}],
    }
    return doc, blob


def glb(parts):
    doc, blob = document(parts)
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    chunks = struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(blob), 0x004E4942) + blob
    return struct.pack("<III", 0x46546C67, 2, 12 + len(chunks)) + chunks


def gltf(parts):
    doc, blob = document(parts)
    doc["buffers"][0]["uri"] = "data:application/octet-stream;base64," + base64.b64encode(blob).decode()
    return json.dumps(doc, indent=1).encode()


def box(size, rgba, at=(0, 0, 0)):
    """A part: a box `size` across (x, y, z, Y-up), its node at `at`."""
    h = [s / 2 for s in size]
    positions = [(sx * h[0], sy * h[1], sz * h[2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    indices = [i for a, b, c, d in faces for i in (a, b, c, a, c, d)]
    return positions, indices, rgba, at


# Two parts: a red triangle at the origin, and a half-grey one moved 1 m up (glTF's +y) by its
# node. Linear 0.2140 is sRGB 0.5.
TEST_PARTS = [
    ([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [0, 1, 2], (1.0, 0.0, 0.0, 1.0), (0, 0, 0)),
    ([(0, 0, 0), (1, 0, 0), (0, 0, 1)], [0, 1, 2], (0.2140, 0.2140, 0.2140, 0.5), (0, 1, 0)),
]

if __name__ == "__main__":
    out = Path(__file__).resolve().parent.parent / "src/mesh/testdata"
    out.mkdir(exist_ok=True)
    for name, data in [("two_parts.glb", glb(TEST_PARTS)), ("two_parts.gltf", gltf(TEST_PARTS))]:
        (out / name).write_bytes(data)
        print(f"{name}: {len(data)} bytes")
