<img src="assets/icon.svg" width="64" height="64" alt="">

# Lantern

A small RViz stand-in in Rust on [Rune](https://github.com/anomalyco/rune) (pinned at `631e8a6`).
Lantern connects to a [Foxglove WebSocket](https://github.com/foxglove/foxglove-sdk/blob/main/docs/spec/websocket/README.md)
server and draws what it sends: a point cloud, the TF tree, poses with their names and
velocities, and a robot model (URDF and meshes) posed by joint states, in a 3D viewport with a
grid.

The protocol works with and without ROS:

- **ROS 2:** run [`foxglove_bridge`](https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge),
  and Lantern reads ROS messages (CDR) from it.
- **No ROS:** any program using a Foxglove SDK (Python, C++, Rust) publishes `foxglove.*`
  protobuf messages directly. This is the path kpd will use.

Lantern is pure Rust. Building or running it needs no ROS installation.

```sh
cargo run --release                          # window; connects to ws://localhost:8765
```

In the viewport, drag to orbit, Shift-drag or middle-drag to pan, and scroll to zoom. The side
panel holds the view controls. The settings button opens the Settings page: server URL, topics,
fixed frame, labels, axis length, the robot's URDF and joint-state topic, and colour by any
numeric field. Settings are saved to
`$XDG_CONFIG_HOME/lantern/settings.json` (`~/.config/lantern/settings.json` by default).

## What it reads

| Kind | ROS 2 via foxglove_bridge (`cdr`) | Foxglove SDK (`protobuf`, or `json` where noted) |
| --- | --- | --- |
| Point cloud | `sensor_msgs/msg/PointCloud2` | `foxglove.PointCloud` |
| Transforms | `tf2_msgs/msg/TFMessage`, `geometry_msgs/msg/TransformStamped` | `foxglove.FrameTransform`, `foxglove.FrameTransforms` |
| Poses | `geometry_msgs/msg/PoseStamped`, `PoseArray`, `nav_msgs/msg/Odometry` | `foxglove.PoseInFrame`, `PosesInFrame`, `Odometry` |
| Joint states | `sensor_msgs/msg/JointState` | `json`, schema named `sensor_msgs/msg/JointState` (Foxglove has no joint-state schema) |

- Other topics are listed in Settings as "No decoder".
- One point cloud topic is drawn at a time; all transform topics are read; pose topics can be
  switched on and off individually.
- Lantern draws the newest message on each topic. Timestamps are ignored.
- Points are coloured by packed `rgb`/`rgba`, by `red`/`green`/`blue` fields, by `intensity`,
  or by height (Auto picks the first that exists). Settings can also colour by any numeric
  field, using its 1st–99th percentile range.

Limits, each shown when hit:

- A cloud can have up to 8,388,608 points, the 128 MiB storage binding Rune's GPU device allows
  at 16 bytes per point. A bigger cloud isn't drawn, and its topic reports the count and the cap.
- A message can be up to 256 MiB. A bigger one fails the connection, with tungstenite's error as
  the status.
- Poses stay drawn until their topic is unadvertised or hidden, or the connection changes. There
  is no expiry yet, so an object a publisher stops sending stays at its last pose.
- A robot can have up to 5,592,405 mesh vertices (1,864,135 triangles), the same 128 MiB binding
  at 24 bytes per vertex. A bigger model fails to load (checked before each mesh is processed),
  and the status chip and Settings name the link, the count and the cap.
  The KUKA Med 7 has 501,186.

To add a message type, add one row to `DECODERS` in [src/decode/mod.rs](src/decode/mod.rs) and
write a function that turns the message into a `scene` type. To add another transport (a file,
a different protocol), implement `Source` in [src/source.rs](src/source.rs).

### Publishing from kpd (no ROS)

With the [Foxglove Python SDK](https://pypi.org/project/foxglove-sdk/):

```python
import foxglove
from foxglove import messages as fm
from foxglove.channels import FrameTransformsChannel, OdometryChannel, PointCloudChannel, PosesInFrameChannel

foxglove.start_server(name="kpd", port=8765)  # Lantern's default URL is ws://localhost:8765
tf, objects = FrameTransformsChannel("/tf"), OdometryChannel("/objects")
points, markers = PointCloudChannel("/points"), PosesInFrameChannel("/markers")

tf.log(fm.FrameTransforms(transforms=[fm.FrameTransform(parent_frame_id="world", child_frame_id="camera", translation=..., rotation=...)]))
# One message per tracked object. body_frame_id is the label Lantern draws; the velocity is in
# the body frame, as foxglove.Odometry defines it, and is drawn as a 1 s arrow.
objects.log(fm.Odometry(frame_id="world", body_frame_id=obj.name, pose=..., linear_velocity=...))
markers.log(fm.PosesInFrame(frame_id="camera", poses=[...]))  # e.g. ArUco detections
points.log(fm.PointCloud(frame_id="camera", point_stride=16, fields=[...], data=...))
```

Joint states, for the robot model. Foxglove has no joint-state schema, so log the ROS message's
shape as JSON under its name; Lantern matches the schema name (the SDK sends the `title`) and
reads `name` and `position`:

```python
joints = foxglove.Channel("/joint_states", schema={
    "title": "sensor_msgs/msg/JointState",
    "type": "object",
    "properties": {
        "name": {"type": "array", "items": {"type": "string"}},
        "position": {"type": "array", "items": {"type": "number"}},
    },
})
joints.log({"name": [f"lbr_joint_{k}" for k in range(7)], "position": list(q)})  # radians, every control tick
```

The names must match the URDF's joints (`lbr_joint_0`..`lbr_joint_6` for the Med 7). Through
foxglove_bridge, a ROS `sensor_msgs/msg/JointState` on any topic works the same way.

[scripts/serve.py](scripts/serve.py) is a complete working example (`serve_sdk`).

## Robot model

Like RViz's RobotModel display. In Settings → Robot, enter a URDF path and press Load (or
Enter). The side panel then gets a Robot section with show/hide and opacity, both also on the
Settings page.

- **URDF only.** Expand xacro once: `xacro robot.urdf.xacro > robot.urdf`. A xacro file is
  refused with that instruction.
- **Meshes** are STL (binary or ASCII), plus the `box`, `cylinder` and `sphere` primitives.
  Filenames resolve as RViz's do: relative to the URDF, absolute, `file://`, or
  `package://pkg/...`, with pkg looked up in `ROS_PACKAGE_PATH`, `AMENT_PREFIX_PATH`
  (`<prefix>/share/pkg`), or as a directory named pkg at or above the URDF (a source checkout).
  A mesh that can't be drawn (a DAE, a missing file) is listed under the load status, and the
  rest of the robot still draws. An unknown geometry element or a `floating`/`planar` joint
  fails the whole load, with the reason. A load that fails shows in the viewport's status chip.
- **Joints:** revolute, continuous, prismatic, fixed, and `mimic`. Positions come from the
  joint-state topic chosen in Settings (Auto picks the first one advertised). Messages that carry
  only some joints merge. A joint with no position yet sits at zero, and Settings shows "n of m
  joints received". Positions outside the URDF's joint limits are drawn as sent, not clamped.
- **Placement:** the URDF's root link is placed in the fixed frame through TF when TF connects
  them (for example `world` from `robot_state_publisher`). Otherwise it's drawn at the fixed
  frame's origin, and Settings says so. With no cloud or poses, Auto picks the robot's root as
  the fixed frame if TF knows it, else the TF tree's root, else (no TF at all) the robot's root.
- **Drawing:** colours come from URDF materials, with light grey where there are none.
  Normals are recomputed with a 40° crease angle, because STL normals are often unusable. Below
  100% opacity (or with a translucent material), the robot shows the points, grid, TF axes and
  poses behind and inside it, but not its own hidden faces (depth prepass). Lines in front of a
  translucent robot take a faint tint from it.
- **Loading** takes 40–48 ms for the Med 7 (8 STL files, 167 k triangles; the Settings status
  shows the time). It runs on the UI thread when the path changes, so that one frame is late.

Not yet: posing links from TF alone (without joint states), the URDF sent over a topic
(`/robot_description`), collision geometry, and DAE/OBJ meshes.

## Layout

| File | Owns |
| --- | --- |
| `src/source.rs` | `Source` trait; `FoxgloveWs`: WebSocket client thread (tungstenite), subscribes to what the app wants, reconnects |
| `src/decode/` | `(encoding, schema)` → decoder table; ROS CDR and Foxglove protobuf decoders; point field extraction |
| `src/scene.rs` | What gets drawn, independent of message types: clouds, transforms, poses, the TF tree |
| `src/app.rs` | App state: applies source events, syncs settings with the source, loads the robot, derives the display (fixed frame, lines, labels, bounds, link poses) |
| `src/robot/` | URDF parsing and forward kinematics (`urdf.rs`), STL and primitive meshes with smoothed normals (`mesh.rs`), loading and mesh path resolution (`mod.rs`) |
| `src/settings.rs` | Persisted settings |
| `src/ui.rs` | Side panel, Settings page, viewport overlays |
| `src/camera.rs` | Orbit camera (REP 103, z up), reversed infinite depth |
| `src/viewport.rs` | Rune `Compositor`: offscreen 3D pass blitted into an `El::effect` slot, GPU timer |
| `src/viewport.wgsl` | Sky and grid, points (vertex-pulled quads), robot (lit meshes), lines, blit |
| `src/main.rs` | Wiring, the source-to-app event inbox, headless scenes |

## Dev data and headless proofs

There is no simulator. [scripts/fixtures.py](scripts/fixtures.py) fetches three real scans from
the [PCL data repository](https://github.com/PointCloudLibrary/data) (BSD-3-Clause) at a pinned
commit, checks their SHA-256, and caches them in `fixtures/` (gitignored):

| Fixture | Points | What it is |
| --- | --- | --- |
| `terrain` | 1,366,408 | airborne LiDAR |
| `table` | 460,400 | SICK LMS400 table scan, intensity |
| `rgbd` | 307,200 (259,847 finite) | RGB-D frame, packed rgb |

[scripts/serve.py](scripts/serve.py) replays one of them over the Foxglove protocol, together
with the transform that places the sensor and, for `table`, two objects and two markers. It
has two modes, matching the two paths above:

```sh
# Setup, once: a Python with the Foxglove SDK.
uv venv --python /usr/bin/python3 --system-site-packages /tmp/fxenv
uv pip install --python /tmp/fxenv/bin/python foxglove-sdk==0.31.0

/tmp/fxenv/bin/python scripts/serve.py --cloud table              # Foxglove SDK, no ROS
/tmp/fxenv/bin/python scripts/serve.py --cloud table --via ros    # rclpy; run foxglove_bridge too
```

For the robot model, [scripts/robot_fixture.py](scripts/robot_fixture.py) expands the KUKA LBR
Med 7 URDF from kuka-policy-deployment's pyfri submodule (lunarlab-gatech/pyfri, Apache-2.0) and
copies its visual meshes into `fixtures/med7/`. `serve.py --robot-pose home|reach` adds fixed
joint states on `/joint_states`: SDK JSON, or the ROS message with `--via ros`. `home` is kpd's
`HOME_Q`.

```sh
python scripts/robot_fixture.py [path/to/kuka-policy-deployment]   # needs its .venv (xacro)
SETTINGS='{"robot_urdf": "'$PWD'/fixtures/med7/med7.urdf"}' \
  scripts/snap-live.sh out/robot "--cloud table --robot-pose reach" - settings
```

The kinematics test checks link poses in both poses against pinocchio 4.1.0 on the same URDF
(committed as `src/robot/testdata/med7.urdf`).

Scripts:

```sh
scripts/snap-live.sh out/live "--cloud table" - settings top      # serve, then headless snaps
scripts/snap-live.sh out/live "--cloud table" "-:--script scripts/interact.json"
# snap-live.sh env: URL (server), or SETTINGS (a whole settings.json to start from)
scripts/audit-controls.sh      # every control must change the viewport (pixel diff vs baseline)
scripts/bench.sh 3             # timing table → out/bench.tsv, raw logs in out/bench/
```

`--scene` names (for `--snap`) are `top`, `height`, `intensity`, `settings`, plus the timing
scenes `no-points`, `half`, `quarter`, `size-1` and `size-4`. Scenes don't save settings.

## Performance

Measured on an RTX PRO 5000 Blackwell (driver 595.99), headless, with a 1948×1560 px viewport
(`--scale 2`). Data is the 1,366,408-point terrain scan served live by `scripts/serve.py`.
Numbers come from `scripts/bench.sh 3`: 3 trials per row, each 600 frames paced at 60 Hz
(`--realtime`). The cloud is sent once, when Lantern subscribes, except in the ingest row.
Spread across trials is ≤ 0.002 ms on GPU p50.

| Config | Points drawn | Rune CPU/frame p50 | GPU 3D pass p50 (p95) |
| --- | --- | --- | --- |
| All points, 2 pt | 1,366,408 | 0.10–0.18 ms | 0.495 (0.521) ms |
| ½ density | 683,204 | 0.17–0.18 ms | 0.262 (0.279) ms |
| ¼ density | 341,602 | 0.18 ms | 0.149 (0.159) ms |
| Points off (sky, grid, TF) | 0 | 0.18 ms | 0.033 (0.036) ms |
| 1 pt points | 1,366,408 | 0.17–0.18 ms | 0.394 (0.416) ms |
| 4 pt points | 1,366,408 | 0.17–0.18 ms | 1.087 (1.096) ms |
| All points + PNG readback every frame | 1,366,408 | 0.16 ms | 0.492 (0.515) ms |
| All points, new cloud at 10 Hz | 1,366,408 | 0.18 ms (p95 9.4–9.5 ms) | 0.492 (0.516) ms |
| All points + robot (Med 7, 501,186 vertices) | 1,366,408 | 0.18 ms | 0.511–0.515 (0.535–0.538) ms |
| Robot only, points off | 0 | 0.18 ms | 0.062 (0.065–0.066) ms |

How to read these:

- **CPU** is Rune's own `frame cpu` line: update, view, layout, Lantern's encode, and submit.
  On frames without a new cloud, the compositor's share is about 0.04 ms. A frame doesn't
  re-upload the cloud; it records one draw call.
- **GPU** comes from begin/end timestamp queries around the 3D pass only, counted only on frames
  that drew the final point count. The UI pass and any readback copy are recorded after that
  pass ends, so they're outside the timestamps. The readback row confirms it: GPU time is
  unchanged (0.492 vs 0.494 ms), while wall time per frame rises to a 59–75 ms p95 because of
  PNG encoding.
- **It measures the point draw.** GPU time is linear in points drawn: 0.033 ms with none, plus
  0.338 ms per million points. That line predicts the ½ and ¼ rows within 0.002 ms.
- **60 fps:** every paced run delivered 600 frames in 9.98 s (60.1 frames/s). The largest gap
  between frame starts was 22.9 ms, and no gap exceeded 25 ms, which would mean a missed slot.
  That includes the ingest row: a new 1.37 M-point cloud arriving 10 times a second (97 uploads
  per 10 s run). Unpaced, the loop reaches 1,200–1,560 frames/s; the spread is from run to run (old and new
  builds alternated land in both modes), with GPU time the same 0.48 ms. This is headless; on-screen
  presentation isn't measured.
- **Ingesting a cloud** costs its frame 5.5–6.0 ms on average to copy 21.9 MB into the GPU
  buffer (`queue.write_buffer`), plus about 1 ms (0.9–1.0 ms mean) to rebuild the display,
  whose bounds come from a 16 Ki-point sample. Decoding, including the colour range, happens
  on the WebSocket thread. Before the
  points buffer was reused across clouds, each cloud created a new buffer, the upload averaged
  10.6–11.0 ms, and cloud frames reached a p95 of 17.2 ms, over the 16.7 ms budget.
- **Clocks:** unpaced, nvidia-smi read 1.61–1.62 GHz (P1, 81–88% busy). The maximum is 3.09 GHz.
  Paced, the samples alternate between idle clocks (P8, ~340 MHz) and 1.63 GHz. The paced GPU time
  is within 2.5% of the unpaced one (0.494 vs 0.482 ms), so the draws themselves run at about
  1.6 GHz. These are not peak-clock numbers.

- **The robot** adds 0.029 ms on its own (0.062 vs 0.033 ms with points off) and 0.017–0.020 ms
  on top of the full cloud. Both runs log 501,186 robot vertices drawn on every frame. Each frame
  draws them twice (depth prepass, then colour), about 1.0 M vertex invocations, plus the parts'
  matrices written per frame (8 × 80 bytes). CPU per frame is unchanged at 0.18 ms. Loading the
  URDF is a one-time cost. It runs synchronously (41 ms in these runs), so that first frame takes
  66–67 ms instead of the usual 23 ms.

### What bounds the point draw

Each point is drawn as six vertices (two triangles) pulled from a storage buffer in one draw
call. An earlier version drew each point as a 4-vertex instance, which took twice as long at the
same image, so the per-instance cost of the vertex front-end was the limit then. The front end
still dominates at small point sizes. Going from 1 pt to 2 pt quadruples the pixels per point but
adds only 0.10 ms (0.394 → 0.495 ms), and 1 pt is still 0.36 ms over the empty-scene floor. At
2 pt, 8.2 M vertex invocations take 0.46 ms, about 18 G/s or roughly 11 per clock at 1.6 GHz. At
4 pt, fill takes over: four times the pixels again costs 0.59 ms more (1.087 ms).
