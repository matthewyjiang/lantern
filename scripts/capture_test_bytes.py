"""Writes the decoder test fixtures in src/decode/testdata/ from the reference serializers:
rclpy's CDR (what foxglove_bridge forwards) and the Foxglove Python SDK's protobuf (what an
SDK app publishes). The Rust tests in src/decode/mod.rs assert on exactly these values.

Needs ROS 2 (for rclpy) with foxglove_msgs, and foxglove-sdk; none is needed to build or run
Lantern:
    sudo apt install ros-humble-foxglove-msgs
    source /opt/ros/humble/setup.bash
    uv venv --python /usr/bin/python3 --system-site-packages /tmp/fxenv
    uv pip install --python /tmp/fxenv/bin/python foxglove-sdk==0.31.0
    /tmp/fxenv/bin/python scripts/capture_test_bytes.py
"""

import math
import pathlib
import struct

from foxglove import messages as fm
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

OUT = pathlib.Path(__file__).resolve().parent.parent / "src/decode/testdata"
OUT.mkdir(exist_ok=True)

# The same three points in both clouds; the NaN one must be dropped.
POINTS = [(1.0, 2.0, 3.0, 10.0), (4.0, 5.0, 6.0, 20.0), (math.nan, 0.0, 0.0, 30.0)]
YAW90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
TFS = [("map", "base_link", (1.0, 2.0, 0.0), YAW90), ("base_link", "lidar", (0.2, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))]
POSES = [((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), ((0.5, -0.25, 0.1), YAW90)]


def write(name, data):
    (OUT / name).write_bytes(data)
    print(f"{name}: {len(data)} bytes")


# sensor_msgs/PointCloud2 with x y z intensity (float32) and PCL-style packed rgb (float32 bits).
pc2 = PointCloud2()
pc2.header.stamp.sec, pc2.header.stamp.nanosec = 12, 34
pc2.header.frame_id = "lidar"
pc2.height, pc2.width = 1, len(POINTS)
pc2.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(["x", "y", "z", "intensity", "rgb"])]
pc2.is_bigendian = False
pc2.point_step = 20
pc2.row_step = 20 * len(POINTS)
rgb = struct.unpack("<f", struct.pack("<I", 0x00FF8000))[0]
pc2.data = b"".join(struct.pack("<5f", *p, rgb) for p in POINTS)
pc2.is_dense = False
write("pc2.cdr", serialize_message(pc2))

tf = TFMessage()
for parent, child, t, q in TFS:
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
    tf.transforms.append(m)
write("tf.cdr", serialize_message(tf))

pa = PoseArray()
pa.header.frame_id = "map"
for t, q in POSES:
    p = Pose()
    p.position.x, p.position.y, p.position.z = t
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    pa.poses.append(p)
write("posearray.cdr", serialize_message(pa))


def fpose(t, q):
    return fm.Pose(position=fm.Vector3(x=t[0], y=t[1], z=t[2]), orientation=fm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


F32 = fm.PackedElementFieldNumericType.Float32
cloud = fm.PointCloud(
    timestamp=fm.Timestamp(sec=12, nsec=34),
    frame_id="lidar",
    pose=fpose((0.0, 0.0, 1.5), (0.0, 0.0, 0.0, 1.0)),
    point_stride=16,
    fields=[fm.PackedElementField(name=n, offset=4 * i, type=F32) for i, n in enumerate(["x", "y", "z", "intensity"])],
    data=b"".join(struct.pack("<4f", *p) for p in POINTS),
)
write("pointcloud.pb", cloud.encode())

fts = fm.FrameTransforms(transforms=[
    fm.FrameTransform(parent_frame_id=p, child_frame_id=c, translation=fm.Vector3(x=t[0], y=t[1], z=t[2]), rotation=fm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
    for p, c, t, q in TFS
])
write("frametransforms.pb", fts.encode())

write("posesinframe.pb", fm.PosesInFrame(frame_id="map", poses=[fpose(t, q) for t, q in POSES]).encode())

# One tracked object: pose in "world", 0.2 m/s along its own x (body-frame twist, as both
# nav_msgs/Odometry and foxglove.Odometry define it).
from nav_msgs.msg import Odometry

ODOM_T, ODOM_Q = POSES[1]
odom = Odometry()
odom.header.frame_id, odom.child_frame_id = "world", "cup"
odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = ODOM_T
o = odom.pose.pose.orientation
o.x, o.y, o.z, o.w = ODOM_Q
odom.twist.twist.linear.x = 0.2
odom.twist.twist.angular.z = 0.5
write("odometry.cdr", serialize_message(odom))
# Three joints with velocities (unused by Lantern); the JSON twin is in the Rust test.
from sensor_msgs.msg import JointState

js = JointState()
js.header.frame_id = ""
js.name = ["lbr_joint_0", "lbr_joint_1", "lbr_joint_2"]
js.position = [0.5, -1.25, 2.0]
js.velocity = [0.0, 0.0, 0.0]
write("jointstate.cdr", serialize_message(js))
write("odometry.pb", fm.Odometry(frame_id="world", body_frame_id="cup", pose=fpose(ODOM_T, ODOM_Q), linear_velocity=fm.Vector3(x=0.2, y=0.0, z=0.0), angular_velocity=fm.Vector3(x=0.0, y=0.0, z=0.5)).encode())

# The same three joints as foxglove.JointStates (protobuf), plus a fourth with no position
# (effort only), which Lantern skips.
write("jointstates.pb", fm.JointStates(joints=[
    *(fm.JointState(name=n, position=q, velocity=0.0) for n, q in zip(js.name, js.position)),
    fm.JointState(name="gripper", effort=1.5),
]).encode())

# Images: a 3x2 picture, red green blue / white black grey, as PNG (lossless, so the test checks
# exact pixels) and as raw rgb8 / bgr8. ROS Image rows are padded (step 12 for 9 bytes).
import io

from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage, Image

PIXELS = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255), (0, 0, 0), (128, 128, 128)]
png = io.BytesIO()
pic = PILImage.new("RGB", (3, 2))
pic.putdata(PIXELS)
pic.save(png, format="PNG")
write("compressedimage.pb", fm.CompressedImage(frame_id="camera", format="png", data=png.getvalue()).encode())
ci = CompressedImage()
ci.header.frame_id, ci.format, ci.data = "camera", "png", png.getvalue()
write("compressedimage.cdr", serialize_message(ci))
rgb = bytes(c for p in PIXELS for c in p)
write("rawimage.pb", fm.RawImage(frame_id="camera", width=3, height=2, encoding="rgb8", step=9, data=rgb).encode())
im = Image()
im.header.frame_id, im.height, im.width, im.encoding, im.is_bigendian, im.step = "camera", 2, 3, "bgr8", 0, 12
im.data = b"".join(bytes(c for p in PIXELS[r * 3:r * 3 + 3] for c in p[::-1]) + b"\0\0\0" for r in range(2))
write("image.cdr", serialize_message(im))

# Annotations: a circle, a closed square (LINE_LOOP), and a label.
def color(r, g, b, a=1.0):
    return fm.Color(r=r, g=g, b=b, a=a)


write("imageannotations.pb", fm.ImageAnnotations(
    circles=[fm.CircleAnnotation(position=fm.Point2(x=10.0, y=20.0), diameter=8.0, thickness=2.0, outline_color=color(1.0, 0.0, 0.0))],
    points=[fm.PointsAnnotation(type=fm.PointsAnnotationType.LineLoop, points=[fm.Point2(x=x, y=y) for x, y in [(1, 1), (5, 1), (5, 5), (1, 5)]], outline_color=color(0.0, 1.0, 0.0), thickness=1.0)],
    texts=[fm.TextAnnotation(position=fm.Point2(x=3.0, y=4.0), text="tag 7", font_size=12.0, text_color=color(1.0, 1.0, 1.0))],
).encode())

# Text and stats: a std_msgs/String, and a DiagnosticArray with one status and two values.
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

write("string.cdr", serialize_message(String(data="tracking ok\nlate ticks: 0")))
da = DiagnosticArray()
da.status = [DiagnosticStatus(level=DiagnosticStatus.WARN, name="controller", message="velocity limit binding", hardware_id="arm", values=[KeyValue(key="tracking error", value="1.8 mm"), KeyValue(key="late ticks", value="3")])]
write("diagnostics.cdr", serialize_message(da))

# One scene in both encodings: two deletions, a grasped object (a model sent as bytes, one by
# URL with its scale unset) that expires after 1.5 s, and one of every other shape. The line's
# indices close it back to its first point; the triangle list's per-vertex colours draw as the
# first.
from foxglove_msgs import msg as fx
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Duration as RosDuration

MODEL_BYTES = b"glTF fake model bytes"
LINE = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
LINE_COLORS = [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
TRI = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0)]
AT = ((0.5, -0.25, 0.1), YAW90)


def fcol(c):
    return fm.Color(r=c[0], g=c[1], b=c[2], a=c[3])


def fv(v):
    return fm.Vector3(x=v[0], y=v[1], z=v[2])


def fp(v):
    return fm.Point3(x=v[0], y=v[1], z=v[2])


write("sceneupdate.pb", fm.SceneUpdate(
    deletions=[fm.SceneEntityDeletion(type=fm.SceneEntityDeletionType.MatchingId, id="old"), fm.SceneEntityDeletion(type=fm.SceneEntityDeletionType.All)],
    entities=[
        fm.SceneEntity(frame_id="gripper", id="grasped", lifetime=fm.Duration(sec=1, nsec=500_000_000), models=[
            fm.ModelPrimitive(pose=fpose(*AT), scale=fv((2.0, 2.0, 2.0)), color=fcol((1.0, 0.0, 0.0, 1.0)), override_color=True, media_type="model/gltf-binary", data=MODEL_BYTES),
            fm.ModelPrimitive(pose=fpose(*AT), url="package://objects/meshes/cup.stl"),
        ]),
        fm.SceneEntity(frame_id="world", id="shapes",
            arrows=[fm.ArrowPrimitive(pose=fpose(*AT), shaft_length=0.3, shaft_diameter=0.02, head_length=0.1, head_diameter=0.05, color=fcol((1.0, 1.0, 0.0, 1.0)))],
            cubes=[fm.CubePrimitive(pose=fpose(*AT), size=fv((0.1, 0.2, 0.3)), color=fcol((0.0, 1.0, 0.0, 0.5)))],
            spheres=[fm.SpherePrimitive(pose=fpose(*AT), size=fv((0.2, 0.2, 0.2)), color=fcol((0.0, 0.0, 1.0, 1.0)))],
            cylinders=[fm.CylinderPrimitive(pose=fpose(*AT), size=fv((0.1, 0.1, 0.4)), bottom_scale=1.0, top_scale=0.0, color=fcol((1.0, 0.5, 0.0, 1.0)))],
            lines=[fm.LinePrimitive(type=fm.LinePrimitiveLineType.LineStrip, pose=fpose(*AT), thickness=0.01, scale_invariant=False, points=[fp(p) for p in LINE], color=fcol((1.0, 1.0, 1.0, 1.0)), colors=[fcol(c) for c in LINE_COLORS], indices=[0, 1, 2, 0])],
            triangles=[fm.TriangleListPrimitive(pose=fpose(*AT), points=[fp(p) for p in TRI], colors=[fcol(c) for c in LINE_COLORS])],
            texts=[fm.TextPrimitive(pose=fpose(*AT), billboard=True, font_size=14.0, scale_invariant=True, color=fcol((1.0, 1.0, 1.0, 1.0)), text="cup")],
        ),
    ],
).encode())


def rpose(t, q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = t
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def rcol(c):
    return fx.Color(r=c[0], g=c[1], b=c[2], a=c[3])


def rv(v):
    return Vector3(x=v[0], y=v[1], z=v[2])


def rp(v):
    return Point(x=v[0], y=v[1], z=v[2])


su = fx.SceneUpdate()
su.deletions = [fx.SceneEntityDeletion(type=fx.SceneEntityDeletion.MATCHING_ID, id="old"), fx.SceneEntityDeletion(type=fx.SceneEntityDeletion.ALL)]
su.entities = [
    fx.SceneEntity(frame_id="gripper", id="grasped", lifetime=RosDuration(sec=1, nanosec=500_000_000), models=[
        fx.ModelPrimitive(pose=rpose(*AT), scale=rv((2.0, 2.0, 2.0)), color=rcol((1.0, 0.0, 0.0, 1.0)), override_color=True, media_type="model/gltf-binary", data=MODEL_BYTES),
        fx.ModelPrimitive(pose=rpose(*AT), url="package://objects/meshes/cup.stl"),
    ]),
    fx.SceneEntity(frame_id="world", id="shapes",
        arrows=[fx.ArrowPrimitive(pose=rpose(*AT), shaft_length=0.3, shaft_diameter=0.02, head_length=0.1, head_diameter=0.05, color=rcol((1.0, 1.0, 0.0, 1.0)))],
        cubes=[fx.CubePrimitive(pose=rpose(*AT), size=rv((0.1, 0.2, 0.3)), color=rcol((0.0, 1.0, 0.0, 0.5)))],
        spheres=[fx.SpherePrimitive(pose=rpose(*AT), size=rv((0.2, 0.2, 0.2)), color=rcol((0.0, 0.0, 1.0, 1.0)))],
        cylinders=[fx.CylinderPrimitive(pose=rpose(*AT), size=rv((0.1, 0.1, 0.4)), bottom_scale=1.0, top_scale=0.0, color=rcol((1.0, 0.5, 0.0, 1.0)))],
        lines=[fx.LinePrimitive(type=fx.LinePrimitive.LINE_STRIP, pose=rpose(*AT), thickness=0.01, scale_invariant=False, points=[rp(p) for p in LINE], color=rcol((1.0, 1.0, 1.0, 1.0)), colors=[rcol(c) for c in LINE_COLORS], indices=[0, 1, 2, 0])],
        triangles=[fx.TriangleListPrimitive(pose=rpose(*AT), points=[rp(p) for p in TRI], colors=[rcol(c) for c in LINE_COLORS])],
        texts=[fx.TextPrimitive(pose=rpose(*AT), billboard=True, font_size=14.0, scale_invariant=True, color=rcol((1.0, 1.0, 1.0, 1.0)), text="cup")],
    ),
]
write("sceneupdate.cdr", serialize_message(su))
