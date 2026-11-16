//! JSON messages, as Foxglove SDK apps log them (`Channel(topic, schema={...})`).

use serde::Deserialize;

use super::{ColorChoice, DecodeError, Decoded, joints};

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
    joints(m.name, m.position)
}
