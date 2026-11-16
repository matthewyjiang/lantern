//! JSON messages, as Foxglove SDK apps log them (`Channel(topic, schema={...})`).

use serde::Deserialize;

use super::{ColorChoice, DecodeError, Decoded};

/// `sensor_msgs/JointState`'s fields, by name. Extra fields (header, velocity, effort) are
/// ignored.
#[derive(Deserialize)]
struct JointState {
    name: Vec<String>,
    #[serde(default)]
    position: Vec<f64>,
}

pub fn joint_state(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: JointState = serde_json::from_slice(bytes).map_err(|e| DecodeError::new(format!("JointState JSON: {e}")))?;
    pair(m.name, m.position)
}

/// Names with their positions. No positions at all is allowed (ROS: a driver that reports only
/// effort); some but not all is a malformed message.
pub fn pair(names: Vec<String>, positions: Vec<f64>) -> Result<Decoded, DecodeError> {
    if positions.is_empty() {
        return Ok(Decoded::Joints(Vec::new()));
    }
    if positions.len() != names.len() {
        return Err(DecodeError::new(format!("JointState has {} names but {} positions", names.len(), positions.len())));
    }
    Ok(Decoded::Joints(names.into_iter().zip(positions).collect()))
}
