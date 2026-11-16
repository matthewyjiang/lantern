//! ROS 2 messages in CDR, field order as in the `.msg` definitions (sensor_msgs, tf2_msgs,
//! geometry_msgs, std_msgs, diagnostic_msgs; identical in Humble and Jazzy).

use glam::{DQuat, DVec3};

use super::cdr::Cdr;
use super::image;
use super::points::{ColorChoice, Field, Num, Packed};
use super::{DecodeError, Decoded};
use crate::scene::{Diagnostic, Level, Pose, Poses, Tracked, Transform};

/// `std_msgs/Header`: stamp (int32 sec, uint32 nanosec), then the frame_id it returns. Lantern
/// draws the newest message of each topic, so it doesn't use stamps.
fn header(r: &mut Cdr) -> Result<String, DecodeError> {
    r.i32()?;
    r.u32()?;
    r.string()
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
    let parent = header(r)?;
    let child = r.string()?;
    Ok(Transform { parent, child, pose: Pose { translation: vector3(r)?, rotation: quaternion(r)? } })
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
