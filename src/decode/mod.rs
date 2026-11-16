//! Wire messages to scene types. A channel is decodable when its (message encoding, schema
//! name) pair has a row in `DECODERS`; supporting a new message type means adding one row and
//! its function. Nothing outside this module sees a message type.

mod cdr;
mod foxglove;
mod json;
mod points;
mod ros;

use std::fmt;

use crate::scene::{PointCloud, Poses, Transform};

pub use points::ColorChoice;

/// What a channel feeds, which decides how the UI offers it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    PointCloud,
    Transforms,
    Poses,
    JointStates,
}

impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Kind::PointCloud => "Point cloud",
            Kind::Transforms => "Transforms",
            Kind::Poses => "Poses",
            Kind::JointStates => "Joints",
        }
    }
}

pub enum Decoded {
    Cloud(PointCloud),
    Transforms(Vec<Transform>),
    Poses(Poses),
    /// Joint positions by name (radians, or metres for prismatic joints).
    Joints(Vec<(String, f64)>),
}

pub struct Decoder {
    encoding: &'static str,
    schema: &'static str,
    pub kind: Kind,
    decode: fn(&[u8], &ColorChoice) -> Result<Decoded, DecodeError>,
}

impl Decoder {
    /// Decodes one message. `color` only matters for point clouds.
    pub fn decode(&self, payload: &[u8], color: &ColorChoice) -> Result<Decoded, DecodeError> {
        (self.decode)(payload, color)
    }
}

/// ROS 2 messages arrive as `cdr` through foxglove_bridge; Foxglove's own schemas as
/// `protobuf` from the Foxglove SDKs (what kpd publishes with).
const DECODERS: &[Decoder] = &[
    Decoder { encoding: "cdr", schema: "sensor_msgs/msg/PointCloud2", kind: Kind::PointCloud, decode: ros::point_cloud2 },
    Decoder { encoding: "cdr", schema: "tf2_msgs/msg/TFMessage", kind: Kind::Transforms, decode: ros::tf_message },
    Decoder { encoding: "cdr", schema: "geometry_msgs/msg/TransformStamped", kind: Kind::Transforms, decode: ros::transform_stamped },
    Decoder { encoding: "cdr", schema: "geometry_msgs/msg/PoseStamped", kind: Kind::Poses, decode: ros::pose_stamped },
    Decoder { encoding: "cdr", schema: "geometry_msgs/msg/PoseArray", kind: Kind::Poses, decode: ros::pose_array },
    Decoder { encoding: "cdr", schema: "nav_msgs/msg/Odometry", kind: Kind::Poses, decode: ros::odometry },
    Decoder { encoding: "protobuf", schema: "foxglove.PointCloud", kind: Kind::PointCloud, decode: foxglove::point_cloud },
    Decoder { encoding: "protobuf", schema: "foxglove.FrameTransform", kind: Kind::Transforms, decode: foxglove::frame_transform },
    Decoder { encoding: "protobuf", schema: "foxglove.FrameTransforms", kind: Kind::Transforms, decode: foxglove::frame_transforms },
    Decoder { encoding: "protobuf", schema: "foxglove.PoseInFrame", kind: Kind::Poses, decode: foxglove::pose_in_frame },
    Decoder { encoding: "protobuf", schema: "foxglove.PosesInFrame", kind: Kind::Poses, decode: foxglove::poses_in_frame },
    Decoder { encoding: "protobuf", schema: "foxglove.Odometry", kind: Kind::Poses, decode: foxglove::odometry },
    Decoder { encoding: "cdr", schema: "sensor_msgs/msg/JointState", kind: Kind::JointStates, decode: ros::joint_state },
    // Foxglove has no joint-state schema; an SDK app (kpd) logs the ROS message's shape as JSON
    // under its name: `{"name": [...], "position": [...]}` (README, "Publishing from kpd").
    Decoder { encoding: "json", schema: "sensor_msgs/msg/JointState", kind: Kind::JointStates, decode: json::joint_state },
];

/// The decoder for a channel's encoding and schema, or `None` if Lantern can't read it. The
/// source looks it up once, when the channel is advertised.
pub fn decoder_for(encoding: &str, schema: &str) -> Option<&'static Decoder> {
    DECODERS.iter().find(|d| d.encoding == encoding && d.schema == schema)
}

#[derive(Clone, Debug, PartialEq)]
pub struct DecodeError(String);

impl DecodeError {
    pub fn new(msg: impl Into<String>) -> DecodeError {
        DecodeError(msg.into())
    }
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for DecodeError {}

/// Message bytes produced by the reference implementations (rclpy's CDR serializer, the
/// Foxglove Python SDK's protobuf), captured by `scripts/capture_test_bytes.py`.
#[cfg(test)]
mod tests {
    use super::*;

    fn decode(encoding: &str, schema: &str, payload: &[u8], color: &ColorChoice) -> Result<Decoded, DecodeError> {
        decoder_for(encoding, schema).unwrap_or_else(|| panic!("no decoder for {schema}")).decode(payload, color)
    }
    use crate::scene::PointColor;
    use glam::DVec3;

    fn cloud(encoding: &str, schema: &str, bytes: &[u8], color: ColorChoice) -> PointCloud {
        match decode(encoding, schema, bytes, &color).unwrap() {
            Decoded::Cloud(c) => c,
            _ => panic!("not a cloud"),
        }
    }

    // Both reference clouds hold the same three points: (1,2,3) i=10, (4,5,6) i=20, and
    // (NaN,0,0) i=30, which must be dropped. The ROS one also packs rgb 0x00ff8000 per point.
    #[test]
    fn ros_point_cloud2_from_rclpy() {
        let c = cloud("cdr", "sensor_msgs/msg/PointCloud2", include_bytes!("testdata/pc2.cdr"), ColorChoice::Auto);
        assert_eq!(c.frame_id, "lidar");
        assert_eq!(c.fields, ["x", "y", "z", "intensity", "rgb"]);
        // Auto picks the packed colour over intensity.
        assert_eq!(c.color, PointColor::Rgb);
        assert_eq!(c.points.len(), 2);
        assert_eq!(&c.points[1][..3], &[4.0, 5.0, 6.0]);
        assert_eq!(c.points[0][3].to_bits(), 0xffff_8000, "rgb bits, alpha forced opaque");

        let c = cloud("cdr", "sensor_msgs/msg/PointCloud2", include_bytes!("testdata/pc2.cdr"), ColorChoice::Field("intensity".into()));
        assert_eq!(c.points.iter().map(|p| p[3]).collect::<Vec<_>>(), [10.0, 20.0]);
        assert!(matches!(c.color, PointColor::Field { ref name, min, max } if name == "intensity" && min == 10.0 && max == 20.0));
    }

    #[test]
    fn foxglove_point_cloud_from_python_sdk() {
        let c = cloud("protobuf", "foxglove.PointCloud", include_bytes!("testdata/pointcloud.pb"), ColorChoice::Auto);
        assert_eq!(c.frame_id, "lidar");
        assert_eq!(c.origin.translation, DVec3::new(0.0, 0.0, 1.5));
        assert_eq!(c.fields, ["x", "y", "z", "intensity"]);
        assert!(matches!(c.color, PointColor::Field { ref name, .. } if name == "intensity"));
        assert_eq!(c.points, [[1.0, 2.0, 3.0, 10.0], [4.0, 5.0, 6.0, 20.0]]);

        let c = cloud("protobuf", "foxglove.PointCloud", include_bytes!("testdata/pointcloud.pb"), ColorChoice::Height);
        assert_eq!(c.color, PointColor::Height);
    }

    #[test]
    fn transforms_from_both_encodings() {
        for (enc, schema, bytes) in [
            ("cdr", "tf2_msgs/msg/TFMessage", &include_bytes!("testdata/tf.cdr")[..]),
            ("protobuf", "foxglove.FrameTransforms", &include_bytes!("testdata/frametransforms.pb")[..]),
        ] {
            let Ok(Decoded::Transforms(ts)) = decode(enc, schema, bytes, &ColorChoice::Auto) else { panic!("{schema}") };
            assert_eq!(ts.len(), 2, "{schema}");
            assert_eq!((ts[0].parent.as_str(), ts[0].child.as_str()), ("map", "base_link"));
            assert_eq!(ts[0].pose.translation, DVec3::new(1.0, 2.0, 0.0));
            assert!((ts[0].pose.rotation.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12, "{schema}");
            assert_eq!((ts[1].parent.as_str(), ts[1].child.as_str()), ("base_link", "lidar"));
        }
    }

    #[test]
    fn poses_from_both_encodings() {
        for (enc, schema, bytes) in [
            ("cdr", "geometry_msgs/msg/PoseArray", &include_bytes!("testdata/posearray.cdr")[..]),
            ("protobuf", "foxglove.PosesInFrame", &include_bytes!("testdata/posesinframe.pb")[..]),
        ] {
            let Ok(Decoded::Poses(p)) = decode(enc, schema, bytes, &ColorChoice::Auto) else { panic!("{schema}") };
            assert_eq!(p.frame_id, "map", "{schema}");
            assert_eq!(p.items.len(), 2, "{schema}");
            assert_eq!(p.items[1].pose.translation, DVec3::new(0.5, -0.25, 0.1), "{schema}");
        }
    }

    // A body at (0.5, -0.25, 0.1) turned 90° about z, moving 0.2 m/s along its own x: that's
    // +y in the parent frame, which is how Lantern keeps velocities.
    #[test]
    fn odometry_velocity_lands_in_the_parent_frame() {
        for (enc, schema, bytes) in [
            ("cdr", "nav_msgs/msg/Odometry", &include_bytes!("testdata/odometry.cdr")[..]),
            ("protobuf", "foxglove.Odometry", &include_bytes!("testdata/odometry.pb")[..]),
        ] {
            let Ok(Decoded::Poses(p)) = decode(enc, schema, bytes, &ColorChoice::Auto) else { panic!("{schema}") };
            assert_eq!(p.frame_id, "world", "{schema}");
            let [item] = &p.items[..] else { panic!("{schema}: one item") };
            assert_eq!(item.label.as_deref(), Some("cup"), "{schema}");
            assert_eq!(item.pose.translation, DVec3::new(0.5, -0.25, 0.1), "{schema}");
            let v = item.velocity.unwrap();
            assert!((v - DVec3::new(0.0, 0.2, 0.0)).length() < 1e-12, "{schema}: {v:?}");
        }
    }

    // The same state in both encodings: three joints, positions for all, plus velocity and
    // effort that Lantern ignores.
    #[test]
    fn joint_states_from_both_encodings() {
        let json = br#"{"name": ["lbr_joint_0", "lbr_joint_1", "lbr_joint_2"], "position": [0.5, -1.25, 2.0], "velocity": [0.0, 0.0, 0.0]}"#;
        for (enc, bytes) in [("cdr", &include_bytes!("testdata/jointstate.cdr")[..]), ("json", &json[..])] {
            let Ok(Decoded::Joints(j)) = decode(enc, "sensor_msgs/msg/JointState", bytes, &ColorChoice::Auto) else { panic!("{enc}") };
            assert_eq!(j, [("lbr_joint_0".to_string(), 0.5), ("lbr_joint_1".to_string(), -1.25), ("lbr_joint_2".to_string(), 2.0)], "{enc}");
        }
    }

    // ROS allows a JointState with names but no positions (an effort-only driver): that's no
    // positions, not an error. Positions for only some names is malformed.
    #[test]
    fn joint_states_without_positions_carry_none() {
        let Ok(Decoded::Joints(j)) = decode("json", "sensor_msgs/msg/JointState", br#"{"name": ["a", "b"], "effort": [1, 2]}"#, &ColorChoice::Auto) else { panic!() };
        assert!(j.is_empty());
        assert!(decode("json", "sensor_msgs/msg/JointState", br#"{"name": ["a", "b"], "position": [1]}"#, &ColorChoice::Auto).is_err());
    }

    #[test]
    fn truncated_messages_are_errors_not_panics() {
        let full = include_bytes!("testdata/pc2.cdr");
        for cut in [3, 20, full.len() / 2, full.len() - 1] {
            assert!(decode("cdr", "sensor_msgs/msg/PointCloud2", &full[..cut], &ColorChoice::Auto).is_err(), "cut at {cut}");
        }
        let full = include_bytes!("testdata/pointcloud.pb");
        assert!(decode("protobuf", "foxglove.PointCloud", &full[..full.len() - 3], &ColorChoice::Auto).is_err());
    }
}
