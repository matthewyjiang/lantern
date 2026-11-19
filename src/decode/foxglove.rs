//! Foxglove's well-known schemas in protobuf, as the Foxglove SDKs log them
//! (`foxglove.schemas.PointCloud` and friends). The structs mirror the `.proto` files in
//! foxglove-sdk's `schemas/proto/foxglove`, field numbers included; fields Lantern doesn't use
//! (timestamps, covariances, metadata) are left out, which protobuf skips over.

use glam::{DQuat, DVec3};
use prost::Message;

use super::points::{ColorChoice, Field, Num, Packed};
use super::ros::unit;
use super::{entities, image};
use super::{DecodeError, Decoded};
use crate::scene::{nanos, Annotation, Entity, Geometry, PointsKind, Pose as ScenePose, Poses, SceneUpdate as Scene, Shape, Tracked, Transform};

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
    #[prost(message, optional, tag = "1")]
    timestamp: Option<Timestamp>,
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
        stamp: stamp(t.timestamp),
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
            // UNKNOWN (0) or newer: nothing to draw, but the message's other shapes still are.
            _ => continue,
        };
        let points = p.points.into_iter().map(|q| point2(Some(q))).collect();
        let color = p.outline_color.or_else(|| p.outline_colors.into_iter().next());
        out.push(Annotation::Points { kind, points, color: rgba(color), fill: rgba(p.fill_color), thickness: p.thickness as f32 });
    }
    for t in m.texts {
        // An unset text colour would be transparent: white, as Foxglove draws it.
        let color = t.text_color.map_or([1.0; 4], |c| rgba(Some(c)));
        out.push(Annotation::Text { at: point2(t.position), text: t.text, size: t.font_size as f32, color, background: rgba(t.background_color) });
    }
    Ok(Decoded::Annotations(out))
}

#[derive(Clone, PartialEq, Message)]
struct Timestamp {
    #[prost(int64, tag = "1")]
    seconds: i64,
    #[prost(int32, tag = "2")]
    nanos: i32,
}

/// Nanoseconds since the epoch; zero when unset.
fn stamp(t: Option<Timestamp>) -> i64 {
    t.map_or(0, |t| nanos(t.seconds, t.nanos.into()))
}

#[derive(Clone, PartialEq, Message)]
struct Duration {
    #[prost(int64, tag = "1")]
    seconds: i64,
    #[prost(int32, tag = "2")]
    nanos: i32,
}

#[derive(Clone, PartialEq, Message)]
struct ArrowPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(double, tag = "2")]
    shaft_length: f64,
    #[prost(double, tag = "3")]
    shaft_diameter: f64,
    #[prost(double, tag = "4")]
    head_length: f64,
    #[prost(double, tag = "5")]
    head_diameter: f64,
    #[prost(message, optional, tag = "6")]
    color: Option<Color>,
}

/// `CubePrimitive` and `SpherePrimitive`: the same fields.
#[derive(Clone, PartialEq, Message)]
struct SizedPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(message, optional, tag = "2")]
    size: Option<Vector3>,
    #[prost(message, optional, tag = "3")]
    color: Option<Color>,
}

#[derive(Clone, PartialEq, Message)]
struct CylinderPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(message, optional, tag = "2")]
    size: Option<Vector3>,
    #[prost(double, tag = "3")]
    bottom_scale: f64,
    #[prost(double, tag = "4")]
    top_scale: f64,
    #[prost(message, optional, tag = "5")]
    color: Option<Color>,
}

#[derive(Clone, PartialEq, Message)]
struct LinePrimitive {
    #[prost(int32, tag = "1")]
    r#type: i32,
    #[prost(message, optional, tag = "2")]
    pose: Option<Pose>,
    #[prost(double, tag = "3")]
    thickness: f64,
    #[prost(bool, tag = "4")]
    scale_invariant: bool,
    #[prost(message, repeated, tag = "5")]
    points: Vec<Vector3>,
    #[prost(message, optional, tag = "6")]
    color: Option<Color>,
    #[prost(message, repeated, tag = "7")]
    colors: Vec<Color>,
    #[prost(fixed32, repeated, tag = "8")]
    indices: Vec<u32>,
}

#[derive(Clone, PartialEq, Message)]
struct TriangleListPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(message, repeated, tag = "2")]
    points: Vec<Vector3>,
    #[prost(message, optional, tag = "3")]
    color: Option<Color>,
    #[prost(message, repeated, tag = "4")]
    colors: Vec<Color>,
    #[prost(fixed32, repeated, tag = "5")]
    indices: Vec<u32>,
}

#[derive(Clone, PartialEq, Message)]
struct TextPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(double, tag = "3")]
    font_size: f64,
    #[prost(bool, tag = "4")]
    scale_invariant: bool,
    #[prost(message, optional, tag = "5")]
    color: Option<Color>,
    #[prost(string, tag = "6")]
    text: String,
}

#[derive(Clone, PartialEq, Message)]
struct ModelPrimitive {
    #[prost(message, optional, tag = "1")]
    pose: Option<Pose>,
    #[prost(message, optional, tag = "2")]
    scale: Option<Vector3>,
    #[prost(message, optional, tag = "3")]
    color: Option<Color>,
    #[prost(bool, tag = "4")]
    override_color: bool,
    #[prost(string, tag = "5")]
    url: String,
    #[prost(string, tag = "6")]
    media_type: String,
    #[prost(bytes = "vec", tag = "7")]
    data: Vec<u8>,
}

#[derive(Clone, PartialEq, Message)]
struct SceneEntity {
    #[prost(message, optional, tag = "1")]
    timestamp: Option<Timestamp>,
    #[prost(string, tag = "2")]
    frame_id: String,
    #[prost(string, tag = "3")]
    id: String,
    #[prost(message, optional, tag = "4")]
    lifetime: Option<Duration>,
    #[prost(bool, tag = "5")]
    frame_locked: bool,
    #[prost(message, repeated, tag = "7")]
    arrows: Vec<ArrowPrimitive>,
    #[prost(message, repeated, tag = "8")]
    cubes: Vec<SizedPrimitive>,
    #[prost(message, repeated, tag = "9")]
    spheres: Vec<SizedPrimitive>,
    #[prost(message, repeated, tag = "10")]
    cylinders: Vec<CylinderPrimitive>,
    #[prost(message, repeated, tag = "11")]
    lines: Vec<LinePrimitive>,
    #[prost(message, repeated, tag = "12")]
    triangles: Vec<TriangleListPrimitive>,
    #[prost(message, repeated, tag = "13")]
    texts: Vec<TextPrimitive>,
    #[prost(message, repeated, tag = "14")]
    models: Vec<ModelPrimitive>,
}

#[derive(Clone, PartialEq, Message)]
struct SceneEntityDeletion {
    #[prost(int32, tag = "2")]
    r#type: i32,
    #[prost(string, tag = "3")]
    id: String,
}

#[derive(Clone, PartialEq, Message)]
struct SceneUpdate {
    #[prost(message, repeated, tag = "1")]
    deletions: Vec<SceneEntityDeletion>,
    #[prost(message, repeated, tag = "2")]
    entities: Vec<SceneEntity>,
}

fn points(v: Vec<Vector3>) -> Vec<DVec3> {
    v.into_iter().map(|p| vec3(Some(p))).collect()
}

fn colors(v: Vec<Color>) -> Vec<[f32; 4]> {
    v.into_iter().map(|c| rgba(Some(c))).collect()
}

fn entity(e: SceneEntity) -> Result<Entity, DecodeError> {
    let mut shapes = Vec::new();
    for a in e.arrows {
        let geometry = Geometry::Arrow { shaft_length: a.shaft_length, shaft_diameter: a.shaft_diameter, head_length: a.head_length, head_diameter: a.head_diameter };
        shapes.push(Shape { pose: pose(a.pose), color: rgba(a.color), geometry });
    }
    for c in e.cubes {
        shapes.push(Shape { pose: pose(c.pose), color: rgba(c.color), geometry: Geometry::Cube { size: vec3(c.size) } });
    }
    for s in e.spheres {
        shapes.push(Shape { pose: pose(s.pose), color: rgba(s.color), geometry: Geometry::Sphere { size: vec3(s.size) } });
    }
    for c in e.cylinders {
        let geometry = Geometry::Cylinder { size: vec3(c.size), bottom: c.bottom_scale, top: c.top_scale };
        shapes.push(Shape { pose: pose(c.pose), color: rgba(c.color), geometry });
    }
    for l in e.lines {
        let shape = entities::line(l.r#type.into(), pose(l.pose), l.thickness, l.scale_invariant, points(l.points), rgba(l.color), colors(l.colors), &l.indices)?;
        shapes.extend(shape);
    }
    for t in e.triangles {
        shapes.push(entities::triangles(pose(t.pose), points(t.points), rgba(t.color), colors(t.colors), &t.indices)?);
    }
    for t in e.texts {
        shapes.push(entities::text(pose(t.pose), t.text, t.font_size, t.scale_invariant, rgba(t.color)));
    }
    for m in e.models {
        shapes.push(entities::model(pose(m.pose), vec3(m.scale), rgba(m.color), m.override_color, m.url, m.media_type, m.data)?);
    }
    let lifetime = e.lifetime.and_then(|d| entities::lifetime(d.seconds, d.nanos.into()));
    Ok(Entity { id: e.id, timestamp: stamp(e.timestamp), frame_id: e.frame_id, frame_locked: e.frame_locked, lifetime, shapes })
}

pub fn scene_update(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: SceneUpdate = parse(bytes)?;
    let deletions = m.deletions.into_iter().map(|d| entities::deletion(d.r#type.into(), d.id)).collect::<Result<_, _>>()?;
    let entities = m.entities.into_iter().map(entity).collect::<Result<_, _>>()?;
    Ok(Decoded::Objects(Scene { deletions, entities }))
}
