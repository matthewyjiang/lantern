"""Dev data for Lantern: replays a recorded point cloud (scripts/fixtures.py) over the Foxglove
WebSocket protocol, plus the transforms that place it and a few fixed object poses.

Two paths, the two Lantern supports:

  --via sdk  The Foxglove Python SDK serves foxglove.* protobuf on ws://127.0.0.1:8765, as any
             SDK app does. Colour as separate red/green/blue bytes (Foxglove's convention).
  --via ros  rclpy publishes ROS 2 messages (PointCloud2, TFMessage on /tf_static, Odometry,
             PoseArray); run foxglove_bridge next to it and Lantern connects to the bridge.
             Colour as PCL's packed float `rgb`.

--robot-pose home|reach adds the KUKA Med 7's joint states on /joint_states, fixed: as JSON under
the sensor_msgs/msg/JointState schema name through the SDK (README, "Publishing without ROS"; or
foxglove.JointStates protobuf with --joints proto), or as the ROS message through the bridge.
Load fixtures/med7/med7.urdf (scripts/robot_fixture.py) in Lantern to see it.

SDK only, for Lantern's other displays:
  --robot-description PATH  the URDF at PATH on /robot_description (std_msgs/msg/String, JSON)
  --camera                  a test-pattern camera image and annotations over it
  --status                  text on /status and a DiagnosticArray on /diagnostics (JSON)
  --motion                  /tracked circles the table (for trails); /once is sent only on the
                            first subscribe (for pose expiry)
  --cloud none              no cloud and no transforms

The cloud is the same recorded scan every message. Setup:
    uv venv --python /usr/bin/python3 --system-site-packages /tmp/fxenv
    uv pip install --python /tmp/fxenv/bin/python foxglove-sdk==0.31.0
    /tmp/fxenv/bin/python scripts/serve.py --cloud table
    # ROS: source /opt/ros/humble/setup.bash, then --via ros, and
    #      ros2 run foxglove_bridge foxglove_bridge --ros-args -p address:=127.0.0.1
"""

import argparse
import math
import os
import re
import threading
import time

import numpy as np

import fixtures

# Where each recording's sensor sat, from the dominant plane in the scan (the floor in both), as
# `python scripts/fixtures.py` prints it (`fixtures.dominant_plane`). The normal is in the sensor
# frame, pointing at the sensor; `height` is the sensor's distance to the plane.
# terrain is already z-up; it's recentred on its mean because its UTM coordinates (5.4e6 m
# north) were stored as float32, which keeps only 0.5 m steps in y.
MOUNTS = {
    "table": {"frame": "lms400", "normal": (0.011, 0.876, 0.482), "height": 1.174},
    "rgbd": {"frame": "camera_optical", "normal": (0.065, -0.845, -0.531), "height": 0.798},
    "terrain": {"frame": "lidar", "normal": (0.0, 0.0, 1.0), "height": 0.0},
}
# Fixed test poses (world frame), named like tracked objects, on the table top: in the
# table scan the dominant plane is the floor, and the table top sits 0.70 m above it with 90% of
# its points in x -0.67..0.39, y 0.95..1.61 (the 5th-95th percentiles of points within 2 cm of
# z = 0.70). The cup has a velocity so its arrow shows; nothing moves.
OBJECTS = {"cup": ((0.05, 1.10, 0.75), 0.4, (0.1, 0.0, 0.0)), "box": ((-0.35, 1.35, 0.74), -0.9, None)}
MARKERS = [((0.20, 1.45, 0.70), 0.0), ((-0.50, 1.00, 0.70), 1.2)]
# Med 7 joint positions, degrees: a home pose, and a reach that turns every joint (the same two
# poses src/robot/urdf.rs checks against pinocchio).
ROBOT_POSES = {"home": [0, 30, 0, -60, 0, 90, 0], "reach": [50, 55, -20, -70, 30, 60, -40]}
JOINTS = [f"lbr_joint_{k}" for k in range(7)]
# The JSON schema an SDK app declares for joint states: Lantern matches on the title (the SDK
# sends it as the schema name) and reads `name` and `position`.
JOINT_STATE_SCHEMA = {
    "title": "sensor_msgs/msg/JointState",
    "type": "object",
    "properties": {
        "name": {"type": "array", "items": {"type": "string"}},
        "position": {"type": "array", "items": {"type": "number"}, "description": "rad (revolute) or m (prismatic)"},
    },
}
STRING_SCHEMA = {"title": "std_msgs/msg/String", "type": "object", "properties": {"data": {"type": "string"}}}
DIAGNOSTICS_SCHEMA = {"title": "diagnostic_msgs/msg/DiagnosticArray", "type": "object"}
# /tracked: a circle over the table (centre, radius m, seconds per turn), at the table top's height.
CIRCLE = ((-0.15, 1.28, 0.78), 0.25, 8.0)


def joint_positions(args):
    return [math.radians(d) for d in ROBOT_POSES[args.robot_pose]] if args.robot_pose else None


def test_pattern(w=640, h=400):
    """A PNG camera frame: a gradient with a checkerboard 'target' and two coloured discs, which
    the annotations outline. Returns (png bytes, annotation shapes in pixels)."""
    import io

    from PIL import Image, ImageDraw

    x = np.linspace(0, 1, w)[None, :]
    y = np.linspace(0, 1, h)[:, None]
    rgb = np.stack([40 + 60 * x + 0 * y, 50 + 40 * y + 0 * x, 70 + 90 * (1 - x) * y], -1).astype(np.uint8)
    img = Image.fromarray(rgb, "RGB")
    d = ImageDraw.Draw(img)
    board = (380, 80, 580, 280)
    cell = (board[2] - board[0]) // 8
    for i in range(8):
        for j in range(8):
            if (i + j) % 2 == 0:
                d.rectangle((board[0] + i * cell, board[1] + j * cell, board[0] + (i + 1) * cell - 1, board[1] + (j + 1) * cell - 1), fill=(235, 235, 235))
    d.ellipse((90, 120, 190, 220), fill=(200, 60, 50))
    d.ellipse((200, 250, 260, 310), fill=(60, 150, 220))
    png = io.BytesIO()
    img.save(png, format="PNG")
    return png.getvalue(), {"board": board, "discs": [((140, 170), 100), ((230, 280), 60)]}


def quat_between(a, b):
    """Unit quaternion (x, y, z, w) rotating unit vector a onto b."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    c = np.cross(a, b)
    w = 1.0 + float(a @ b)
    q = np.array([*c, w])
    return tuple(q / np.linalg.norm(q))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def scene(name):
    cloud = fixtures.load(name)
    m = MOUNTS[name]
    xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], 1).astype(np.float64)
    if name == "terrain":
        xyz -= np.nanmean(xyz, 0)
    mount = ((0.0, 0.0, m["height"]), quat_between(m["normal"], (0.0, 0.0, 1.0)))
    return cloud, xyz.astype(np.float32), m["frame"], mount


def serve_sdk(args, cloud, xyz, frame, mount):
    import foxglove
    from foxglove import messages as fm
    from foxglove.channels import FrameTransformsChannel, OdometryChannel, PointCloudChannel, PosesInFrameChannel

    # The SDK doesn't latch: a client that subscribes between publishes waits for the next one.
    # Publishing again on every subscribe means a new client always gets data straight away,
    # and lets --hz 0 publish only then (scripts/bench.sh: one cloud per run).
    subscribed = threading.Event()

    class Listener(foxglove.ServerListener):
        def on_subscribe(self, client, channel):
            subscribed.set()

    server = foxglove.start_server(name="lantern-fixture", host="127.0.0.1", port=args.port, server_listener=Listener())

    def vec(v):
        return fm.Vector3(x=v[0], y=v[1], z=v[2])

    def quat(q):
        return fm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    extras = sdk_extras(args, fm, vec, quat)
    if cloud is None:
        print(f"serving on ws://127.0.0.1:{args.port} (no cloud)")
        try:
            while True:
                fresh = subscribed.wait(1.0 / args.hz if args.hz > 0 else None)
                if fresh:
                    time.sleep(0.05)
                    subscribed.clear()
                extras(fresh)
        except KeyboardInterrupt:
            server.stop()
        return
    names = ["x", "y", "z"]
    F32, U8 = fm.PackedElementFieldNumericType.Float32, fm.PackedElementFieldNumericType.Uint8
    fields = [fm.PackedElementField(name=n, offset=4 * i, type=F32) for i, n in enumerate(names)]
    cols = [xyz]
    if "intensity" in cloud.dtype.names:
        fields.append(fm.PackedElementField(name="intensity", offset=12, type=F32))
        cols.append(cloud["intensity"].astype(np.float32)[:, None])
    stride = 4 * sum(c.shape[1] for c in cols)
    data = np.ascontiguousarray(np.hstack(cols)).tobytes()
    if "rgb" in cloud.dtype.names:
        packed = cloud["rgb"].view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], 1).astype(np.uint8)
        rec = np.zeros(len(xyz), dtype=[("xyz", "<f4", 3), ("rgb", "u1", 3), ("pad", "u1")])
        rec["xyz"], rec["rgb"] = xyz, rgb
        fields += [fm.PackedElementField(name=n, offset=12 + i, type=U8) for i, n in enumerate(["red", "green", "blue"])]
        stride, data = 16, rec.tobytes()

    points = PointCloudChannel(args.topic)
    tf = FrameTransformsChannel("/tf")
    objects = OdometryChannel("/objects")
    markers = PosesInFrameChannel("/markers")
    rate = f"at {args.hz} Hz and on subscribe" if args.hz > 0 else "on subscribe only"
    print(f"serving {len(xyz)} points ({len(data)} B) on ws://127.0.0.1:{args.port} {args.topic} {rate}")
    try:
        while True:
            # A client subscribes to its channels in one message; let the rest land, then send once.
            fresh = subscribed.wait(1.0 / args.hz if args.hz > 0 else None)
            if fresh:
                time.sleep(0.05)
                subscribed.clear()
            tf.log(fm.FrameTransforms(transforms=[fm.FrameTransform(parent_frame_id="world", child_frame_id=frame, translation=vec(mount[0]), rotation=quat(mount[1]))]))
            points.log(fm.PointCloud(frame_id=frame, point_stride=stride, fields=fields, data=data))
            if args.cloud == "table":
                for name, (p, yaw, v) in OBJECTS.items():
                    objects.log(fm.Odometry(frame_id="world", body_frame_id=name, pose=fm.Pose(position=vec(p), orientation=quat(yaw_quat(yaw))), linear_velocity=vec(v) if v else None))
                markers.log(fm.PosesInFrame(frame_id="world", poses=[fm.Pose(position=vec(p), orientation=quat(yaw_quat(y))) for p, y in MARKERS]))
            extras(fresh)
    except KeyboardInterrupt:
        server.stop()


def sdk_extras(args, fm, vec, quat):
    """Channels for the flags beyond the cloud; returns the function that publishes them, which
    takes whether a client just subscribed."""
    import foxglove
    from foxglove.channels import CompressedImageChannel, ImageAnnotationsChannel, JointStatesChannel, PoseInFrameChannel

    pubs = []
    q = joint_positions(args)
    if q and args.joints == "proto":
        ch = JointStatesChannel("/joint_states")
        msg = fm.JointStates(joints=[fm.JointState(name=n, position=v) for n, v in zip(JOINTS, q)])
        pubs.append(lambda _: ch.log(msg))
    elif q:
        ch_json = foxglove.Channel("/joint_states", schema=JOINT_STATE_SCHEMA)
        pubs.append(lambda _: ch_json.log({"name": JOINTS, "position": q}))
    if args.robot_description:
        # Relative mesh paths name files next to the URDF, which the receiver can't know: send
        # them as file:// URLs (a robot_state_publisher URDF uses package:// instead).
        base = os.path.dirname(os.path.abspath(args.robot_description))
        urdf = re.sub(r'filename="(?![a-z]+://|/)([^"]+)"', lambda m: f'filename="file://{base}/{m[1]}"', open(args.robot_description).read())
        desc = foxglove.Channel("/robot_description", schema=STRING_SCHEMA)
        pubs.append(lambda _: desc.log({"data": urdf}))
    if args.camera:
        png, shapes = test_pattern()
        image = CompressedImageChannel("/camera/image")
        notes = ImageAnnotationsChannel("/camera/annotations")
        x0, y0, x1, y1 = shapes["board"]

        def color(r, g, b, a=1.0):
            return fm.Color(r=r, g=g, b=b, a=a)

        def pt(x, y):
            return fm.Point2(x=float(x), y=float(y))

        annotations = fm.ImageAnnotations(
            circles=[fm.CircleAnnotation(position=pt(*c), diameter=float(dia) + 16, thickness=3.0, outline_color=color(1.0, 0.85, 0.1)) for c, dia in shapes["discs"]],
            points=[
                fm.PointsAnnotation(type=fm.PointsAnnotationType.LineLoop, points=[pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)], outline_color=color(0.2, 1.0, 0.4), fill_color=color(0.2, 1.0, 0.4, 0.18), thickness=3.0),
                fm.PointsAnnotation(type=fm.PointsAnnotationType.LineStrip, points=[pt(40, 360), pt(160, 330), pt(300, 350), pt(440, 320), pt(600, 340)], outline_color=color(1.0, 0.3, 0.8), thickness=2.0),
                fm.PointsAnnotation(type=fm.PointsAnnotationType.Points, points=[pt(x, y) for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]], outline_color=color(1.0, 1.0, 1.0), thickness=8.0),
            ],
            texts=[
                fm.TextAnnotation(position=pt(x0, y0 - 8), text="target 0.92", font_size=18.0, text_color=color(0.2, 1.0, 0.4), background_color=color(0.0, 0.0, 0.0, 0.6)),
                fm.TextAnnotation(position=pt(96, 112), text="cup", font_size=18.0, text_color=color(1.0, 0.85, 0.1), background_color=color(0.0, 0.0, 0.0, 0.6)),
            ],
        )
        img = fm.CompressedImage(frame_id="camera", format="png", data=png)
        pubs.append(lambda _: (image.log(img), notes.log(annotations)))
    if args.status:
        text = foxglove.Channel("/status", schema=STRING_SCHEMA)
        diag = foxglove.Channel("/diagnostics", schema=DIAGNOSTICS_SCHEMA)
        # Fixed, so headless runs (scripts/audit-controls.sh) give the same frame every time.
        def status(_):
            text.log({"data": "policy: pick_cup\nphase: grasp · 3 of 5"})
            diag.log({"status": [
                {"level": 0, "name": "arm", "message": "ok", "values": [{"key": "loop rate", "value": "1000 Hz"}, {"key": "max torque", "value": "41 %"}]},
                {"level": 1, "name": "camera", "message": "frame drops", "values": [{"key": "fps", "value": "27.4"}]},
                {"level": 2, "name": "gripper", "message": "not homed", "values": []},
            ]})
        pubs.append(status)
    if args.motion:
        tracked = PoseInFrameChannel("/tracked")
        once = PoseInFrameChannel("/once")
        t0 = time.monotonic()
        (cx, cy, cz), r, period = CIRCLE

        def motion(fresh):
            a = 2 * math.pi * (time.monotonic() - t0) / period
            p = (cx + r * math.cos(a), cy + r * math.sin(a), cz)
            tracked.log(fm.PoseInFrame(frame_id="world", pose=fm.Pose(position=vec(p), orientation=quat(yaw_quat(a + math.pi / 2)))))
            # Only for a new subscriber: it then never repeats, and expires in Lantern.
            if fresh:
                once.log(fm.PoseInFrame(frame_id="world", pose=fm.Pose(position=vec((0.25, 1.0, 0.75)), orientation=quat(yaw_quat(0.0)))))
        pubs.append(motion)
    return lambda fresh: [p(fresh) for p in pubs]


def serve_ros(args, cloud, xyz, frame, mount):
    import rclpy
    from geometry_msgs.msg import Pose, PoseArray, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("lantern_fixture")
    names = ["x", "y", "z"]
    cols = [xyz]
    if "intensity" in cloud.dtype.names:
        names.append("intensity")
        cols.append(cloud["intensity"].astype(np.float32)[:, None])
    if "rgb" in cloud.dtype.names:
        names.append("rgb")
        cols.append(cloud["rgb"].astype(np.float32)[:, None])
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(names)]
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * len(xyz)
    msg.data = np.ascontiguousarray(np.hstack(cols)).tobytes()
    msg.is_dense = False

    def pose(p, q):
        out = Pose()
        out.position.x, out.position.y, out.position.z = p
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q
        return out

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "world", frame
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = mount[0]
    r = tf.transform.rotation
    r.x, r.y, r.z, r.w = mount[1]
    node.create_publisher(TFMessage, "/tf_static", latched).publish(TFMessage(transforms=[tf]))
    points = node.create_publisher(PointCloud2, args.topic, 1)
    objects = node.create_publisher(Odometry, "/objects", 10)
    markers = node.create_publisher(PoseArray, "/markers", 1)
    q = joint_positions(args)
    joints = node.create_publisher(JointState, "/joint_states", 10) if q else None
    print(f"publishing {len(xyz)} points ({len(msg.data)} B) on ROS {args.topic} at {args.hz} Hz")
    try:
        while rclpy.ok():
            t0 = time.monotonic()
            msg.header.stamp = node.get_clock().now().to_msg()
            points.publish(msg)
            if args.cloud == "table":
                for name, (p, yaw, v) in OBJECTS.items():
                    o = Odometry()
                    o.header.frame_id, o.child_frame_id = "world", name
                    o.pose.pose = pose(p, yaw_quat(yaw))
                    if v:
                        # nav_msgs/Odometry's twist is in the child frame: rotate world v into it.
                        c, s = math.cos(-yaw), math.sin(-yaw)
                        o.twist.twist.linear.x, o.twist.twist.linear.y = c * v[0] - s * v[1], s * v[0] + c * v[1]
                    objects.publish(o)
                pa = PoseArray()
                pa.header.frame_id = "world"
                pa.poses = [pose(p, yaw_quat(y)) for p, y in MARKERS]
                markers.publish(pa)
            if joints:
                js = JointState(name=JOINTS, position=q)
                js.header.stamp = msg.header.stamp
                joints.publish(js)
            time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cloud", choices=[*fixtures.FIXTURES, "none"], default="table")
    p.add_argument("--via", choices=["sdk", "ros"], default="sdk")
    p.add_argument("--topic", default="/points")
    p.add_argument("--hz", type=float, default=10.0, help="--via sdk: 0 publishes only when a client subscribes")
    p.add_argument("--port", type=int, default=8765, help="--via sdk only")
    p.add_argument("--robot-pose", choices=list(ROBOT_POSES), help="also publish the Med 7's joint states, fixed at this pose")
    p.add_argument("--joints", choices=["json", "proto"], default="json", help="--via sdk: joint states as JSON JointState or foxglove.JointStates")
    p.add_argument("--robot-description", metavar="PATH", help="--via sdk: publish this URDF on /robot_description")
    p.add_argument("--camera", action="store_true", help="--via sdk: camera image and annotations")
    p.add_argument("--status", action="store_true", help="--via sdk: text and diagnostics")
    p.add_argument("--motion", action="store_true", help="--via sdk: a moving pose, and one sent once")
    a = p.parse_args()
    if a.via == "ros" and (a.cloud == "none" or a.joints != "json" or a.robot_description or a.camera or a.status or a.motion):
        p.error("--cloud none, --joints proto, --robot-description, --camera, --status and --motion need --via sdk")
    (serve_sdk if a.via == "sdk" else serve_ros)(a, *(scene(a.cloud) if a.cloud != "none" else (None, None, None, None)))
