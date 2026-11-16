//! Foxglove's well-known schemas in protobuf, as the Foxglove SDKs log them
//! (`foxglove.schemas.PointCloud` and friends). The structs mirror the `.proto` files in
//! foxglove-sdk's `schemas/proto/foxglove`, field numbers included; fields Lantern doesn't use
//! (timestamps, covariances, metadata) are left out, which protobuf skips over.

use glam::{DQuat, DVec3};
use prost::Message;

use super::image;
use super::points::{ColorChoice, Field, Num, Packed};
use super::ros::unit;
use super::{DecodeError, Decoded};
use crate::scene::{Annotation, PointsKind, Pose as ScenePose, Poses, Tracked, Transform};

#[derive(Clone, PartialEq, Message)]
struct Vector3 {
    #[prost(double, tag = "1")]
    x: f64,
    #[prost(double, tag = "2")]
    y: f64,
    #[prost(double, tag = "3")]
    z: f64,
}

#[derive(Clone, PartialEq, Message)]
struct Quaternion {
    #[prost(double, tag = "1")]
    x: f64,
    #[prost(double, tag = "2")]
    y: f64,
    #[prost(double, tag = "3")]
    z: f64,
    #[prost(double, tag = "4")]
    w: f64,
}

#[derive(Clone, PartialEq, Message)]
struct Pose {
    #[prost(message, optional, tag = "1")]
    position: Option<Vector3>,
    #[prost(message, optional, tag = "2")]
    orientation: Option<Quaternion>,
}

#[derive(Clone, PartialEq, Message)]
struct PackedElementField {
    #[prost(string, tag = "1")]
    name: String,
    #[prost(fixed32, tag = "2")]
    offset: u32,
    #[prost(int32, tag = "3")]
    r#type: i32,
}

#[derive(Clone, PartialEq, Message)]
struct PointCloud {
    #[prost(string, tag = "2")]
    frame_id: String,
    #[prost(message, optional, tag = "3")]
    pose: Option<Pose>,
    #[prost(fixed32, tag = "4")]
    point_stride: u32,
    #[prost(message, repeated, tag = "5")]
    fields: Vec<PackedElementField>,
    #[prost(bytes = "vec", tag = "6")]
    data: Vec<u8>,
}

#[derive(Clone, PartialEq, Message)]
struct FrameTransform {
    #[prost(string, tag = "2")]
    parent_frame_id: String,
    #[prost(string, tag = "3")]
    child_frame_id: String,
    #[prost(message, optional, tag = "4")]
    translation: Option<Vector3>,
    #[prost(message, optional, tag = "5")]
    rotation: Option<Quaternion>,
}

#[derive(Clone, PartialEq, Message)]
struct FrameTransforms {
    #[prost(message, repeated, tag = "1")]
    transforms: Vec<FrameTransform>,
}

#[derive(Clone, PartialEq, Message)]
struct PoseInFrame {
    #[prost(string, tag = "2")]
    frame_id: String,
    #[prost(message, optional, tag = "3")]
    pose: Option<Pose>,
}

#[derive(Clone, PartialEq, Message)]
struct PosesInFrame {
    #[prost(string, tag = "2")]
    frame_id: String,
    #[prost(message, repeated, tag = "3")]
    poses: Vec<Pose>,
}

#[derive(Clone, PartialEq, Message)]
struct Odometry {
    #[prost(string, tag = "2")]
    frame_id: String,
    #[prost(string, tag = "3")]
    body_frame_id: String,
    #[prost(message, optional, tag = "4")]
    pose: Option<Pose>,
    #[prost(message, optional, tag = "5")]
    linear_velocity: Option<Vector3>,
}

fn parse<M: Message + Default>(bytes: &[u8]) -> Result<M, DecodeError> {
    M::decode(bytes).map_err(|e| DecodeError::new(format!("protobuf: {e}")))
}

fn vec3(v: Option<Vector3>) -> DVec3 {
    v.map_or(DVec3::ZERO, |v| DVec3::new(v.x, v.y, v.z))
}

/// A missing quaternion is identity (protobuf drops the all-default message: w would be 0).
fn quat(q: Option<Quaternion>) -> DQuat {
    q.map_or(DQuat::IDENTITY, |q| unit(DQuat::from_xyzw(q.x, q.y, q.z, q.w)))
}

fn pose(p: Option<Pose>) -> ScenePose {
    p.map_or(ScenePose::IDENTITY, |p| ScenePose { translation: vec3(p.position), rotation: quat(p.orientation) })
}

fn transform(t: FrameTransform) -> Transform {
    Transform {
        parent: t.parent_frame_id,
        child: t.child_frame_id,
        pose: ScenePose { translation: vec3(t.translation), rotation: quat(t.rotation) },
    }
}

pub fn point_cloud(bytes: &[u8], color: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: PointCloud = parse(bytes)?;
    let mut fields = Vec::with_capacity(m.fields.len());
    for f in m.fields {
        // PackedElementField.NumericType: note the order differs from ROS's PointField.
        let num = match f.r#type {
            1 => Num::U8,
            2 => Num::I8,
            3 => Num::U16,
            4 => Num::I16,
            5 => Num::U32,
            6 => Num::I32,
            7 => Num::F32,
            8 => Num::F64,
            other => return Err(DecodeError::new(format!("field {} has unknown numeric type {other}", f.name))),
        };
        fields.push(Field { name: f.name, offset: f.offset as usize, num });
    }
    let stride = m.point_stride as usize;
    if stride == 0 {
        return Err(DecodeError::new("point_stride is 0"));
    }
    let packed = Packed {
        frame_id: m.frame_id,
        origin: pose(m.pose),
        fields,
        stride,
        count: m.data.len() / stride,
        big_endian: false,
        data: &m.data,
    };
    Ok(Decoded::Cloud(packed.extract(color)?))
}

pub fn frame_transform(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::Transforms(vec![transform(parse(bytes)?)]))
}

pub fn frame_transforms(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: FrameTransforms = parse(bytes)?;
    Ok(Decoded::Transforms(m.transforms.into_iter().map(transform).collect()))
}

pub fn pose_in_frame(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: PoseInFrame = parse(bytes)?;
    Ok(Decoded::Poses(Poses { frame_id: m.frame_id, items: vec![Tracked::at(pose(m.pose))] }))
}

pub fn poses_in_frame(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: PosesInFrame = parse(bytes)?;
    let items = m.poses.into_iter().map(|p| Tracked::at(pose(Some(p)))).collect();
    Ok(Decoded::Poses(Poses { frame_id: m.frame_id, items }))
}

/// One tracked body: its pose in `frame_id` and its velocity, which Foxglove gives in the
/// body frame and Lantern keeps in `frame_id`.
pub fn odometry(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: Odometry = parse(bytes)?;
    let pose = pose(m.pose);
    let velocity = m.linear_velocity.map(|v| pose.rotation * vec3(Some(v)));
    let item = Tracked { pose, label: Some(m.body_frame_id).filter(|l| !l.is_empty()), velocity };
    Ok(Decoded::Poses(Poses { frame_id: m.frame_id, items: vec![item] }))
}

#[derive(Clone, PartialEq, Message)]
struct JointState {
    #[prost(string, tag = "1")]
    name: String,
    #[prost(double, optional, tag = "2")]
    position: Option<f64>,
}

#[derive(Clone, PartialEq, Message)]
struct JointStates {
    #[prost(message, repeated, tag = "2")]
    joints: Vec<JointState>,
}

/// Every joint that has a position (a joint with only velocity or effort is skipped).
pub fn joint_states(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: JointStates = parse(bytes)?;
    Ok(Decoded::Joints(m.joints.into_iter().filter_map(|j| Some((j.name, j.position?))).collect()))
}

#[derive(Clone, PartialEq, Message)]
struct CompressedImage {
    #[prost(bytes = "vec", tag = "2")]
    data: Vec<u8>,
    #[prost(string, tag = "3")]
    format: String,
}

#[derive(Clone, PartialEq, Message)]
struct RawImage {
    #[prost(fixed32, tag = "2")]
    width: u32,
    #[prost(fixed32, tag = "3")]
    height: u32,
    #[prost(string, tag = "4")]
    encoding: String,
    #[prost(fixed32, tag = "5")]
    step: u32,
    #[prost(bytes = "vec", tag = "6")]
    data: Vec<u8>,
}

pub fn compressed_image(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: CompressedImage = parse(bytes)?;
    Ok(Decoded::Image(image::compressed(&m.format, &m.data)?))
}

pub fn raw_image(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: RawImage = parse(bytes)?;
    Ok(Decoded::Image(image::raw(m.width, m.height, &m.encoding, m.step, false, &m.data)?))
}

#[derive(Clone, PartialEq, Message)]
struct Color {
    #[prost(double, tag = "1")]
    r: f64,
    #[prost(double, tag = "2")]
    g: f64,
    #[prost(double, tag = "3")]
    b: f64,
    #[prost(double, tag = "4")]
    a: f64,
}

#[derive(Clone, PartialEq, Message)]
struct Point2 {
    #[prost(double, tag = "1")]
    x: f64,
    #[prost(double, tag = "2")]
    y: f64,
}

#[derive(Clone, PartialEq, Message)]
struct CircleAnnotation {
    #[prost(message, optional, tag = "2")]
    position: Option<Point2>,
    #[prost(double, tag = "3")]
    diameter: f64,
    #[prost(double, tag = "4")]
    thickness: f64,
    #[prost(message, optional, tag = "5")]
    fill_color: Option<Color>,
    #[prost(message, optional, tag = "6")]
    outline_color: Option<Color>,
}

#[derive(Clone, PartialEq, Message)]
struct PointsAnnotation {
    #[prost(int32, tag = "2")]
    r#type: i32,
    #[prost(message, repeated, tag = "3")]
    points: Vec<Point2>,
    #[prost(message, optional, tag = "4")]
    outline_color: Option<Color>,
    /// Per point (POINTS) or segment colours; Lantern draws a shape in one colour, the first.
    #[prost(message, repeated, tag = "5")]
    outline_colors: Vec<Color>,
    #[prost(message, optional, tag = "6")]
    fill_color: Option<Color>,
    #[prost(double, tag = "7")]
    thickness: f64,
}

#[derive(Clone, PartialEq, Message)]
struct TextAnnotation {
    #[prost(message, optional, tag = "2")]
    position: Option<Point2>,
    #[prost(string, tag = "3")]
    text: String,
    #[prost(double, tag = "4")]
    font_size: f64,
    #[prost(message, optional, tag = "5")]
    text_color: Option<Color>,
    #[prost(message, optional, tag = "6")]
    background_color: Option<Color>,
}

#[derive(Clone, PartialEq, Message)]
struct ImageAnnotations {
    #[prost(message, repeated, tag = "1")]
    circles: Vec<CircleAnnotation>,
    #[prost(message, repeated, tag = "2")]
    points: Vec<PointsAnnotation>,
    #[prost(message, repeated, tag = "3")]
    texts: Vec<TextAnnotation>,
}

/// A missing colour is transparent: an absent fill draws no fill.
fn rgba(c: Option<Color>) -> [f32; 4] {
    c.map_or([0.0; 4], |c| [c.r as f32, c.g as f32, c.b as f32, c.a as f32])
}

fn point2(p: Option<Point2>) -> [f32; 2] {
    p.map_or([0.0; 2], |p| [p.x as f32, p.y as f32])
}

pub fn image_annotations(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: ImageAnnotations = parse(bytes)?;
    let mut out = Vec::new();
    for c in m.circles {
        out.push(Annotation::Circle { center: point2(c.position), diameter: c.diameter as f32, thickness: c.thickness as f32, outline: rgba(c.outline_color), fill: rgba(c.fill_color) });
    }
    for p in m.points {
        let kind = match p.r#type {
            1 => PointsKind::Points,
            2 => PointsKind::Loop,
            3 => PointsKind::Strip,
            4 => PointsKind::Lines,
            other => return Err(DecodeError::new(format!("points annotation has unknown type {other}"))),
        };
        let points = p.points.into_iter().map(|q| point2(Some(q))).collect();
        let color = p.outline_color.or_else(|| p.outline_colors.into_iter().next());
        out.push(Annotation::Points { kind, points, color: rgba(color), fill: rgba(p.fill_color), thickness: p.thickness as f32 });
    }
    for t in m.texts {
        out.push(Annotation::Text { at: point2(t.position), text: t.text, size: t.font_size as f32, color: rgba(t.text_color), background: rgba(t.background_color) });
    }
    Ok(Decoded::Annotations(out))
}
