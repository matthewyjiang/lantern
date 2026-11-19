//! Wire messages to scene types. A channel is decodable when its (message encoding, schema
//! name) pair has a row in `DECODERS`; supporting a new message type means adding one row and
//! its function. Nothing outside this module sees a message type.

mod cdr;
mod entities;
mod foxglove;
mod image;
mod json;
mod points;
mod ros;

use std::fmt;

use crate::scene::{Annotation, Diagnostic, Image, PointCloud, Poses, SceneUpdate, Transform};

pub use points::ColorChoice;

/// What a channel feeds, which decides how the UI offers it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    PointCloud,
    Transforms,
    Poses,
    /// Scene entities: shapes, text and meshes placed in TF frames.
    Objects,
    JointStates,
    Image,
    Annotations,
    Text,
    Stats,
    RobotDescription,
}

impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Kind::PointCloud => "Point cloud",
            Kind::Transforms => "Transforms",
            Kind::Poses => "Poses",
            Kind::Objects => "Objects",
            Kind::JointStates => "Joints",
            Kind::Image => "Image",
            Kind::Annotations => "Annotations",
            Kind::Text => "Text",
            Kind::Stats => "Stats",
            Kind::RobotDescription => "Robot",
        }
    }
}

pub enum Decoded {
    Cloud(PointCloud),
    Transforms(Vec<Transform>),
    Poses(Poses),
    Objects(SceneUpdate),
    /// Joint positions by name (radians, or metres for prismatic joints).
    Joints(Vec<(String, f64)>),
    Image(Image),
    Annotations(Vec<Annotation>),
    Text(String),
    Diagnostics(Vec<Diagnostic>),
    /// A URDF, as text.
    RobotDescription(String),
}

/// A JointState's names with their positions. No positions at all is allowed (ROS: a driver
/// that reports only effort); some but not all is a malformed message.
fn joints(names: Vec<String>, positions: Vec<f64>) -> Result<Decoded, DecodeError> {
    if positions.is_empty() {
        return Ok(Decoded::Joints(Vec::new()));
    }
    if positions.len() != names.len() {
        return Err(DecodeError::new(format!("JointState has {} names but {} positions", names.len(), positions.len())));
    }
    Ok(Decoded::Joints(names.into_iter().zip(positions).collect()))
}

pub struct Decoder {
    encoding: &'static str,
    schema: &'static str,
    /// Only for topics named this (or ending in `/` and this): a URDF travels as a plain
    /// string, which only its topic's name tells apart from text.
    topic: Option<&'static str>,
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
/// `protobuf` from the Foxglove SDKs (any app that serves the protocol without ROS); ROS types an
/// SDK app logs as JSON (Foxglove has no schema of its own for them) as `json` under the ROS
/// type's name.
const DECODERS: &[Decoder] = &[
    // Topic-specific rows first: `decoder_for` takes the first match.
    row("cdr", "std_msgs/msg/String", Some("robot_description"), Kind::RobotDescription, ros::robot_description),
    row("json", "std_msgs/msg/String", Some("robot_description"), Kind::RobotDescription, json::robot_description),
    row("cdr", "sensor_msgs/msg/PointCloud2", None, Kind::PointCloud, ros::point_cloud2),
    row("cdr", "tf2_msgs/msg/TFMessage", None, Kind::Transforms, ros::tf_message),
    row("cdr", "geometry_msgs/msg/TransformStamped", None, Kind::Transforms, ros::transform_stamped),
    row("cdr", "geometry_msgs/msg/PoseStamped", None, Kind::Poses, ros::pose_stamped),
    row("cdr", "geometry_msgs/msg/PoseArray", None, Kind::Poses, ros::pose_array),
    row("cdr", "nav_msgs/msg/Odometry", None, Kind::Poses, ros::odometry),
    row("cdr", "foxglove_msgs/msg/SceneUpdate", None, Kind::Objects, ros::scene_update),
    row("cdr", "sensor_msgs/msg/JointState", None, Kind::JointStates, ros::joint_state),
    row("cdr", "sensor_msgs/msg/CompressedImage", None, Kind::Image, ros::compressed_image),
    row("cdr", "sensor_msgs/msg/Image", None, Kind::Image, ros::image),
    row("cdr", "std_msgs/msg/String", None, Kind::Text, ros::string),
    row("cdr", "diagnostic_msgs/msg/DiagnosticArray", None, Kind::Stats, ros::diagnostic_array),
    row("protobuf", "foxglove.PointCloud", None, Kind::PointCloud, foxglove::point_cloud),
    row("protobuf", "foxglove.FrameTransform", None, Kind::Transforms, foxglove::frame_transform),
    row("protobuf", "foxglove.FrameTransforms", None, Kind::Transforms, foxglove::frame_transforms),
    row("protobuf", "foxglove.PoseInFrame", None, Kind::Poses, foxglove::pose_in_frame),
    row("protobuf", "foxglove.PosesInFrame", None, Kind::Poses, foxglove::poses_in_frame),
    row("protobuf", "foxglove.Odometry", None, Kind::Poses, foxglove::odometry),
    row("protobuf", "foxglove.SceneUpdate", None, Kind::Objects, foxglove::scene_update),
    row("protobuf", "foxglove.JointStates", None, Kind::JointStates, foxglove::joint_states),
    row("protobuf", "foxglove.CompressedImage", None, Kind::Image, foxglove::compressed_image),
    row("protobuf", "foxglove.RawImage", None, Kind::Image, foxglove::raw_image),
    row("protobuf", "foxglove.ImageAnnotations", None, Kind::Annotations, foxglove::image_annotations),
    // `{"name": [...], "position": [...]}`, as the ROS message.
    row("json", "sensor_msgs/msg/JointState", None, Kind::JointStates, json::joint_state),
    row("json", "std_msgs/msg/String", None, Kind::Text, json::string),
    row("json", "diagnostic_msgs/msg/DiagnosticArray", None, Kind::Stats, json::diagnostic_array),
];

const fn row(
    encoding: &'static str,
    schema: &'static str,
    topic: Option<&'static str>,
    kind: Kind,
    decode: fn(&[u8], &ColorChoice) -> Result<Decoded, DecodeError>,
) -> Decoder {
    Decoder { encoding, schema, topic, kind, decode }
}

/// The decoder for a channel's encoding, schema and topic, or `None` if Lantern can't read it.
/// The source looks it up once, when the channel is advertised.
pub fn decoder_for(encoding: &str, schema: &str, topic: &str) -> Option<&'static Decoder> {
    let named = |t: &str| topic == t || topic.strip_suffix(t).is_some_and(|rest| rest.ends_with('/'));
    DECODERS.iter().find(|d| d.encoding == encoding && d.schema == schema && d.topic.is_none_or(named))
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
        decoder_for(encoding, schema, "/topic").unwrap_or_else(|| panic!("no decoder for {schema}")).decode(payload, color)
    }
    use crate::scene::{Level, PointColor, PointsKind};
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

    // foxglove.JointStates carries the same three positions; its fourth joint has only an
    // effort, and a joint without a position is no position, not zero.
    #[test]
    fn foxglove_joint_states_skip_joints_without_a_position() {
        let Ok(Decoded::Joints(j)) = decode("protobuf", "foxglove.JointStates", include_bytes!("testdata/jointstates.pb"), &ColorChoice::Auto) else { panic!() };
        assert_eq!(j, [("lbr_joint_0".to_string(), 0.5), ("lbr_joint_1".to_string(), -1.25), ("lbr_joint_2".to_string(), 2.0)]);
    }

    // One 3×2 picture (red green blue / white black grey) in four forms: PNG in both compressed
    // messages, raw rgb8 (Foxglove) and row-padded bgr8 (ROS).
    #[test]
    fn images_decode_to_the_same_rgba() {
        let want: Vec<u8> = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 255], [0, 0, 0], [128, 128, 128]]
            .iter().flat_map(|&[r, g, b]| [r, g, b, 255]).collect();
        for (enc, schema, bytes) in [
            ("protobuf", "foxglove.CompressedImage", &include_bytes!("testdata/compressedimage.pb")[..]),
            ("cdr", "sensor_msgs/msg/CompressedImage", &include_bytes!("testdata/compressedimage.cdr")[..]),
            ("protobuf", "foxglove.RawImage", &include_bytes!("testdata/rawimage.pb")[..]),
            ("cdr", "sensor_msgs/msg/Image", &include_bytes!("testdata/image.cdr")[..]),
        ] {
            let Ok(Decoded::Image(i)) = decode(enc, schema, bytes, &ColorChoice::Auto) else { panic!("{schema}") };
            assert_eq!((i.width, i.height), (3, 2), "{schema}");
            assert_eq!(i.rgba, want, "{schema}");
        }
    }

    #[test]
    fn image_annotations_keep_shape_position_and_colour() {
        let Ok(Decoded::Annotations(a)) = decode("protobuf", "foxglove.ImageAnnotations", include_bytes!("testdata/imageannotations.pb"), &ColorChoice::Auto) else { panic!() };
        assert_eq!(a, [
            Annotation::Circle { center: [10.0, 20.0], diameter: 8.0, thickness: 2.0, outline: [1.0, 0.0, 0.0, 1.0], fill: [0.0; 4] },
            Annotation::Points { kind: PointsKind::Loop, points: vec![[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]], color: [0.0, 1.0, 0.0, 1.0], fill: [0.0; 4], thickness: 1.0 },
            Annotation::Text { at: [3.0, 4.0], text: "tag 7".into(), size: 12.0, color: [1.0; 4], background: [0.0; 4] },
        ]);
    }

    #[test]
    fn text_and_stats_from_both_encodings() {
        let text = br#"{"data": "tracking ok\nlate ticks: 0"}"#;
        for (enc, bytes) in [("cdr", &include_bytes!("testdata/string.cdr")[..]), ("json", &text[..])] {
            let Ok(Decoded::Text(t)) = decode(enc, "std_msgs/msg/String", bytes, &ColorChoice::Auto) else { panic!("{enc}") };
            assert_eq!(t, "tracking ok\nlate ticks: 0", "{enc}");
        }
        let json = br#"{"status": [{"level": 1, "name": "controller", "message": "velocity limit binding", "hardware_id": "arm",
            "values": [{"key": "tracking error", "value": "1.8 mm"}, {"key": "late ticks", "value": "3"}]}]}"#;
        for (enc, bytes) in [("cdr", &include_bytes!("testdata/diagnostics.cdr")[..]), ("json", &json[..])] {
            let Ok(Decoded::Diagnostics(d)) = decode(enc, "diagnostic_msgs/msg/DiagnosticArray", bytes, &ColorChoice::Auto) else { panic!("{enc}") };
            assert_eq!(d, [Diagnostic {
                level: Level::Warn,
                name: "controller".into(),
                message: "velocity limit binding".into(),
                values: vec![("tracking error".into(), "1.8 mm".into()), ("late ticks".into(), "3".into())],
            }], "{enc}");
        }
    }

    // The same scene from the Foxglove SDK and from rclpy's foxglove_msgs: both deletions, the
    // grasped object's lifetime and two models, and one of every other shape, indices resolved.
    #[test]
    fn scene_updates_from_both_encodings() {
        use crate::scene::{Deletion, Geometry, LineKind, ModelSource, Thickness};
        let decoded = |enc, schema, bytes| match decode(enc, schema, bytes, &ColorChoice::Auto) {
            Ok(Decoded::Objects(u)) => u,
            _ => panic!("{schema}"),
        };
        let pb = decoded("protobuf", "foxglove.SceneUpdate", &include_bytes!("testdata/sceneupdate.pb")[..]);
        let cdr = decoded("cdr", "foxglove_msgs/msg/SceneUpdate", &include_bytes!("testdata/sceneupdate.cdr")[..]);
        assert_eq!(pb, cdr);

        assert_eq!(pb.deletions, [Deletion::Id("old".into()), Deletion::All]);
        let [grasped, shapes] = &pb.entities[..] else { panic!("two entities") };
        assert_eq!((grasped.id.as_str(), grasped.frame_id.as_str(), grasped.lifetime), ("grasped", "gripper", Some(1.5)));
        assert_eq!(shapes.lifetime, None, "no lifetime: kept until replaced");
        let at = &grasped.shapes[0].pose;
        assert_eq!(at.translation, DVec3::new(0.5, -0.25, 0.1));
        assert!((at.rotation.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        let Geometry::Model { source: ModelSource::Data { media_type, bytes, .. }, scale, override_color: true } = &grasped.shapes[0].geometry else { panic!() };
        assert_eq!((media_type.as_str(), &bytes[..], *scale), ("model/gltf-binary", &b"glTF fake model bytes"[..], DVec3::splat(2.0)));
        assert_eq!(grasped.shapes[1].geometry, Geometry::Model { source: ModelSource::Url("package://objects/meshes/cup.stl".into()), scale: DVec3::ONE, override_color: false }, "an unset scale is 1");

        let geometry: Vec<&Geometry> = shapes.shapes.iter().map(|s| &s.geometry).collect();
        assert_eq!(geometry[..4], [
            &Geometry::Arrow { shaft_length: 0.3, shaft_diameter: 0.02, head_length: 0.1, head_diameter: 0.05 },
            &Geometry::Cube { size: DVec3::new(0.1, 0.2, 0.3) },
            &Geometry::Sphere { size: DVec3::splat(0.2) },
            &Geometry::Cylinder { size: DVec3::new(0.1, 0.1, 0.4), bottom: 1.0, top: 0.0 },
        ]);
        assert_eq!(shapes.shapes[1].color, [0.0, 1.0, 0.0, 0.5]);
        let (red, green, blue) = ([1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0]);
        let Geometry::Line { kind: LineKind::Strip, points, colors, thickness: Thickness::Metres(t) } = geometry[4] else { panic!("{:?}", geometry[4]) };
        assert_eq!((points.len(), points[3], *t), (4, DVec3::ZERO, 0.01), "indices 0 1 2 0 close the strip");
        assert_eq!(colors, &[red, green, blue, red]);
        assert_eq!(geometry[5], &Geometry::Triangles { points: vec![DVec3::ZERO, DVec3::new(0.1, 0.0, 0.0), DVec3::new(0.0, 0.1, 0.0)] });
        assert_eq!(shapes.shapes[5].color, red, "per-vertex colours draw as the first");
        assert_eq!(geometry[6], &Geometry::Text { text: "cup".into(), size: Thickness::Points(14.0) });
    }

    // A std_msgs/String is a URDF only on a robot_description topic.
    #[test]
    fn a_string_is_a_robot_description_only_by_its_topic() {
        let kind = |topic| decoder_for("cdr", "std_msgs/msg/String", topic).map(|d| d.kind);
        assert_eq!(kind("/robot_description"), Some(Kind::RobotDescription));
        assert_eq!(kind("/arm/robot_description"), Some(Kind::RobotDescription));
        assert_eq!(kind("/my_robot_description"), Some(Kind::Text));
        assert_eq!(kind("/status"), Some(Kind::Text));
    }

    #[test]
    fn truncated_messages_are_errors_not_panics() {
        let full = include_bytes!("testdata/pc2.cdr");
        for cut in [3, 20, full.len() / 2, full.len() - 1] {
            assert!(decode("cdr", "sensor_msgs/msg/PointCloud2", &full[..cut], &ColorChoice::Auto).is_err(), "cut at {cut}");
        }
        let full = include_bytes!("testdata/sceneupdate.cdr");
        for cut in [3, 40, full.len() / 2, full.len() - 1] {
            assert!(decode("cdr", "foxglove_msgs/msg/SceneUpdate", &full[..cut], &ColorChoice::Auto).is_err(), "cut at {cut}");
        }
        let full = include_bytes!("testdata/pointcloud.pb");
        assert!(decode("protobuf", "foxglove.PointCloud", &full[..full.len() - 3], &ColorChoice::Auto).is_err());
    }
}
