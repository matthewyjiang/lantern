//! Foxglove's well-known schemas in protobuf, as the Foxglove SDKs log them
//! (`foxglove.schemas.PointCloud` and friends). The structs mirror the `.proto` files in
//! foxglove-sdk's `schemas/proto/foxglove`, field numbers included; fields Lantern doesn't use
//! (timestamps, covariances, metadata) are left out, which protobuf skips over.

use glam::{DQuat, DVec3};
use prost::Message;

use super::points::{ColorChoice, Field, Num, Packed};
use super::ros::unit;
use super::{DecodeError, Decoded};
use crate::scene::{Pose as ScenePose, Poses, Tracked, Transform};

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
