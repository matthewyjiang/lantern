"""Writes the decoder test fixtures in src/decode/testdata/ from the reference serializers:
rclpy's CDR (what foxglove_bridge forwards) and the Foxglove Python SDK's protobuf (what kpd
publishes with). The Rust tests in src/decode/mod.rs assert on exactly these values.

Needs ROS 2 (for rclpy) and foxglove-sdk; neither is needed to build or run Lantern:
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
