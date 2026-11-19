//! ROS 2 messages in CDR, field order as in the `.msg` definitions (sensor_msgs, tf2_msgs,
//! geometry_msgs, std_msgs, diagnostic_msgs, foxglove_msgs; identical in Humble and Jazzy).

use glam::{DQuat, DVec3};

use super::cdr::Cdr;
use super::{entities, image};
use super::points::{ColorChoice, Field, Num, Packed};
use super::{DecodeError, Decoded};
use crate::scene::{nanos, Diagnostic, Entity, Geometry, Level, Pose, Poses, SceneUpdate, Shape, Tracked, Transform};

/// `std_msgs/Header`: stamp (int32 sec, uint32 nanosec), then the frame_id it returns. Lantern
/// draws the newest message of each topic, so it doesn't use stamps.
fn header(r: &mut Cdr) -> Result<String, DecodeError> {
    Ok(stamped_header(r)?.1)
}

/// A `std_msgs/Header`: its stamp in nanoseconds, and its frame.
fn stamped_header(r: &mut Cdr) -> Result<(i64, String), DecodeError> {
    let stamp = time(r)?;
    Ok((stamp, r.string()?))
}

/// A `builtin_interfaces/Time`, in nanoseconds.
fn time(r: &mut Cdr) -> Result<i64, DecodeError> {
    Ok(nanos(r.i32()?.into(), r.u32()?.into()))
}

fn vector3(r: &mut Cdr) -> Result<DVec3, DecodeError> {
    Ok(DVec3::new(r.f64()?, r.f64()?, r.f64()?))
}

fn quaternion(r: &mut Cdr) -> Result<DQuat, DecodeError> {
    let (x, y, z, w) = (r.f64()?, r.f64()?, r.f64()?, r.f64()?);
    Ok(unit(DQuat::from_xyzw(x, y, z, w)))
}

/// Normalises, treating an all-zero quaternion (a common "unset" in hand-written publishers) as
/// identity, as RViz does.
pub fn unit(q: DQuat) -> DQuat {
    if q.length_squared() < 1e-12 { DQuat::IDENTITY } else { q.normalize() }
}

fn pose(r: &mut Cdr) -> Result<Pose, DecodeError> {
    Ok(Pose { translation: vector3(r)?, rotation: quaternion(r)? })
}

/// `geometry_msgs/TransformStamped` without the encapsulation header.
fn transform(r: &mut Cdr) -> Result<Transform, DecodeError> {
    let (stamp, parent) = stamped_header(r)?;
    let child = r.string()?;
    Ok(Transform { parent, child, stamp, pose: Pose { translation: vector3(r)?, rotation: quaternion(r)? } })
}

pub fn point_cloud2(bytes: &[u8], color: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    let frame_id = header(&mut r)?;
    let height = r.u32()? as usize;
    let width = r.u32()? as usize;
    // PointField: name, offset (u32), datatype (u8), count (u32). Smallest is 4+1+3pad+4+4.
    let n = r.seq_len(13)?;
    let mut fields = Vec::with_capacity(n);
    for _ in 0..n {
        let name = r.string()?;
        let offset = r.u32()? as usize;
        let datatype = r.u8()?;
        let _count = r.u32()?;
        let num = match datatype {
            1 => Num::I8,
            2 => Num::U8,
            3 => Num::I16,
            4 => Num::U16,
            5 => Num::I32,
            6 => Num::U32,
            7 => Num::F32,
            8 => Num::F64,
            other => return Err(DecodeError::new(format!("PointField {name} has unknown datatype {other}"))),
        };
        fields.push(Field { name, offset, num });
    }
    let big_endian = r.bool()?;
    let point_step = r.u32()? as usize;
    let row_step = r.u32()? as usize;
    let data = r.bytes()?;
    let _is_dense = r.bool()?;
    if height > 1 && row_step != width * point_step {
        return Err(DecodeError::new(format!("row_step {row_step} has padding past width × point_step ({width} × {point_step}); unsupported")));
    }
    let count = width.checked_mul(height).ok_or_else(|| DecodeError::new("width × height overflows"))?;
    let packed = Packed { frame_id, origin: Pose::IDENTITY, fields, stride: point_step, count, big_endian, data };
    Ok(Decoded::Cloud(packed.extract(color)?))
}

pub fn tf_message(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    // A TransformStamped is at least 4+4+4+4 (header) + 4 (child) + 56 + pad.
    let n = r.seq_len(76)?;
    (0..n).map(|_| transform(&mut r)).collect::<Result<_, _>>().map(Decoded::Transforms)
}

pub fn transform_stamped(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::Transforms(vec![transform(&mut Cdr::new(bytes)?)?]))
}

pub fn pose_stamped(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    let frame_id = header(&mut r)?;
    Ok(Decoded::Poses(Poses { frame_id, items: vec![Tracked::at(pose(&mut r)?)] }))
}

pub fn pose_array(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    let frame_id = header(&mut r)?;
    let n = r.seq_len(56)?;
    let items = (0..n).map(|_| pose(&mut r).map(Tracked::at)).collect::<Result<_, _>>()?;
    Ok(Decoded::Poses(Poses { frame_id, items }))
}

/// `sensor_msgs/JointState`: header, name, position, velocity, effort (velocity and effort
/// unused).
pub fn joint_state(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    header(&mut r)?;
    let n = r.seq_len(4)?;
    let names = (0..n).map(|_| r.string()).collect::<Result<Vec<_>, _>>()?;
    let m = r.seq_len(8)?;
    let positions = (0..m).map(|_| r.f64()).collect::<Result<Vec<_>, _>>()?;
    super::joints(names, positions)
}

/// `nav_msgs/Odometry`: header, child_frame_id, PoseWithCovariance, TwistWithCovariance
/// (covariances are fixed `float64[36]`, no length prefix). The twist is in the child frame.
pub fn odometry(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    let frame_id = header(&mut r)?;
    let child = r.string()?;
    let pose = pose(&mut r)?;
    for _ in 0..36 {
        r.f64()?;
    }
    let linear = vector3(&mut r)?;
    let _angular = vector3(&mut r)?;
    for _ in 0..36 {
        r.f64()?;
    }
    let item = Tracked { pose, label: Some(child), velocity: Some(pose.rotation * linear) };
    Ok(Decoded::Poses(Poses { frame_id, items: vec![item] }))
}

/// `sensor_msgs/CompressedImage`: header, format, data.
pub fn compressed_image(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    header(&mut r)?;
    let format = r.string()?;
    Ok(Decoded::Image(image::compressed(&format, r.bytes()?)?))
}

/// `sensor_msgs/Image`: header, height, width, encoding, is_bigendian, step, data.
pub fn image(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    header(&mut r)?;
    let height = r.u32()?;
    let width = r.u32()?;
    let encoding = r.string()?;
    let big_endian = r.bool()?;
    let step = r.u32()?;
    Ok(Decoded::Image(image::raw(width, height, &encoding, step, big_endian, r.bytes()?)?))
}

/// `std_msgs/String`.
pub fn string(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::Text(Cdr::new(bytes)?.string()?))
}

/// `std_msgs/String` on a `robot_description` topic: a URDF.
pub fn robot_description(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::RobotDescription(Cdr::new(bytes)?.string()?))
}

/// `diagnostic_msgs/DiagnosticArray`: header, then each status's level (byte), name, message,
/// hardware_id and key/value pairs.
pub fn diagnostic_array(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    header(&mut r)?;
    // A status is at least 32 bytes (level and padding, three empty strings, an empty
    // sequence's count); 20 is a safe lower bound for refusing an impossible count.
    let n = r.seq_len(20)?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let level = r.u8()?;
        let name = r.string()?;
        let message = r.string()?;
        let _hardware_id = r.string()?;
        let m = r.seq_len(8)?;
        let values = (0..m).map(|_| Ok((r.string()?, r.string()?))).collect::<Result<_, DecodeError>>()?;
        out.push(Diagnostic { level: Level::from_byte(level), name, message, values });
    }
    Ok(Decoded::Diagnostics(out))
}

/// `foxglove_msgs/Color`: r, g, b, a as float64.
fn color(r: &mut Cdr) -> Result<[f32; 4], DecodeError> {
    Ok([r.f64()? as f32, r.f64()? as f32, r.f64()? as f32, r.f64()? as f32])
}

/// A sequence of `item`s, each at least `min` bytes.
fn seq<T>(r: &mut Cdr, min: usize, mut item: impl FnMut(&mut Cdr) -> Result<T, DecodeError>) -> Result<Vec<T>, DecodeError> {
    let n = r.seq_len(min)?;
    (0..n).map(|_| item(r)).collect()
}

/// One `foxglove_msgs/SceneEntity`, fields in the `.msg`'s order.
fn scene_entity(r: &mut Cdr) -> Result<Entity, DecodeError> {
    let timestamp = time(r)?;
    let frame_id = r.string()?;
    let id = r.string()?;
    let lifetime = entities::lifetime(r.i32()?.into(), r.u32()?.into());
    let frame_locked = r.bool()?;
    seq(r, 8, |r| Ok((r.string()?, r.string()?)))?; // metadata
    let mut shapes = Vec::new();
    // Every primitive starts with a 56-byte pose.
    for a in seq(r, 56, |r| Ok((pose(r)?, [r.f64()?, r.f64()?, r.f64()?, r.f64()?], color(r)?)))? {
        let (pose, [shaft_length, shaft_diameter, head_length, head_diameter], color) = a;
        shapes.push(Shape { pose, color, geometry: Geometry::Arrow { shaft_length, shaft_diameter, head_length, head_diameter } });
    }
    for (pose, size, color) in seq(r, 56, |r| Ok((pose(r)?, vector3(r)?, color(r)?)))? {
        shapes.push(Shape { pose, color, geometry: Geometry::Cube { size } });
    }
    for (pose, size, color) in seq(r, 56, |r| Ok((pose(r)?, vector3(r)?, color(r)?)))? {
        shapes.push(Shape { pose, color, geometry: Geometry::Sphere { size } });
    }
    for (pose, size, bottom, top, color) in seq(r, 56, |r| Ok((pose(r)?, vector3(r)?, r.f64()?, r.f64()?, color(r)?)))? {
        shapes.push(Shape { pose, color, geometry: Geometry::Cylinder { size, bottom, top } });
    }
    let n = r.seq_len(56)?;
    for _ in 0..n {
        let kind = r.u8()?;
        let (pose, thickness, scale_invariant) = (pose(r)?, r.f64()?, r.bool()?);
        let points = seq(r, 24, vector3)?;
        let base = color(r)?;
        let colors = seq(r, 32, color)?;
        let indices = seq(r, 4, |r| r.u32())?;
        shapes.extend(entities::line(kind.into(), pose, thickness, scale_invariant, points, base, colors, &indices)?);
    }
    let n = r.seq_len(56)?;
    for _ in 0..n {
        let pose = pose(r)?;
        let points = seq(r, 24, vector3)?;
        let base = color(r)?;
        let colors = seq(r, 32, color)?;
        let indices = seq(r, 4, |r| r.u32())?;
        shapes.push(entities::triangles(pose, points, base, colors, &indices)?);
    }
    let n = r.seq_len(56)?;
    for _ in 0..n {
        let (pose, _billboard, font_size, scale_invariant, color) = (pose(r)?, r.bool()?, r.f64()?, r.bool()?, color(r)?);
        shapes.push(entities::text(pose, r.string()?, font_size, scale_invariant, color));
    }
    let n = r.seq_len(56)?;
    for _ in 0..n {
        let (pose, scale, color, override_color) = (pose(r)?, vector3(r)?, color(r)?, r.bool()?);
        let (url, media_type, data) = (r.string()?, r.string()?, r.bytes()?.to_vec());
        shapes.push(entities::model(pose, scale, color, override_color, url, media_type, data)?);
    }
    Ok(Entity { id, timestamp, frame_id, frame_locked, lifetime, shapes })
}

/// `foxglove_msgs/SceneUpdate`, as a ROS node publishes it through foxglove_bridge.
pub fn scene_update(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let mut r = Cdr::new(bytes)?;
    // A deletion: time (8), type (1, padded to 4), id (4 or more).
    let deletions = seq(&mut r, 16, |r| {
        let (_sec, _nanosec, kind, id) = (r.i32()?, r.u32()?, r.u8()?, r.string()?);
        entities::deletion(kind.into(), id)
    })?;
    // An entity: two times, two strings, a bool, nine sequences: well over 40 bytes.
    let entities = seq(&mut r, 40, scene_entity)?;
    Ok(Decoded::Objects(SceneUpdate { deletions, entities }))
}
