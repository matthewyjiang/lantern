//! JSON messages, as Foxglove SDK apps log them (`Channel(topic, schema={...})`): the ROS
//! message's fields by name, under the ROS type's name as the schema title.

use serde::Deserialize;
use serde::de::DeserializeOwned;

use super::{ColorChoice, DecodeError, Decoded, joints};
use crate::scene::{Diagnostic, Level};

fn parse<T: DeserializeOwned>(what: &str, bytes: &[u8]) -> Result<T, DecodeError> {
    serde_json::from_slice(bytes).map_err(|e| DecodeError::new(format!("{what} JSON: {e}")))
}

/// `sensor_msgs/JointState`'s fields, by name. Extra fields (header, velocity, effort) are
/// ignored.
#[derive(Deserialize)]
struct JointState {
    name: Vec<String>,
    #[serde(default)]
    position: Vec<f64>,
}

pub fn joint_state(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: JointState = parse("JointState", bytes)?;
    joints(m.name, m.position)
}

/// `std_msgs/String`.
#[derive(Deserialize)]
struct Text {
    data: String,
}

pub fn string(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::Text(parse::<Text>("String", bytes)?.data))
}

pub fn robot_description(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    Ok(Decoded::RobotDescription(parse::<Text>("String", bytes)?.data))
}

/// `diagnostic_msgs/DiagnosticArray`: `{"status": [{"level", "name", "message", "values":
/// [{"key", "value"}]}]}`; the header and `hardware_id` are ignored.
#[derive(Deserialize)]
struct DiagnosticArray {
    status: Vec<Status>,
}

#[derive(Deserialize)]
struct Status {
    #[serde(default)]
    level: u8,
    #[serde(default)]
    name: String,
    #[serde(default)]
    message: String,
    #[serde(default)]
    values: Vec<KeyValue>,
}

#[derive(Deserialize)]
struct KeyValue {
    key: String,
    value: String,
}

pub fn diagnostic_array(bytes: &[u8], _: &ColorChoice) -> Result<Decoded, DecodeError> {
    let m: DiagnosticArray = parse("DiagnosticArray", bytes)?;
    Ok(Decoded::Diagnostics(m.status.into_iter().map(|s| Diagnostic {
        level: Level::from_byte(s.level),
        name: s.name,
        message: s.message,
        values: s.values.into_iter().map(|v| (v.key, v.value)).collect(),
    }).collect()))
}
