//! What the user chose, kept across runs in `$XDG_CONFIG_HOME/lantern/settings.json` (or
//! `~/.config/...`). Saved whenever it changes; a file that doesn't parse is reported and
//! replaced by defaults rather than stopping the app.

use std::collections::BTreeSet;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use crate::decode::{ColorChoice, Kind};
use crate::layout::Layout;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// The Foxglove WebSocket servers to read, all at once: foxglove_bridge or SDK apps (both
    /// default to port 8765). Topics of the same name on two servers feed one display.
    pub connections: Vec<String>,
    /// The single server of settings files from before `connections`: read, never written.
    #[serde(skip_serializing)]
    url: Option<String>,
    /// The point cloud topic to draw; `None` draws the first one advertised.
    pub cloud_topic: Option<String>,
    /// Pose, annotation, text and stats topics the user switched off (new topics show by default).
    pub hidden_topics: BTreeSet<String>,
    /// The frame the scene is drawn in; `None` uses the root of the TF tree the data is in.
    pub fixed_frame: Option<String>,
    pub color: ColorChoice,
    /// Point size, in points.
    pub point_size: f32,
    /// Draw every n-th point.
    pub decimate: u32,
    /// Length of TF and pose axes, metres.
    pub axis_length: f32,
    /// A pose not refreshed for this long is dropped, seconds; 0 keeps poses until their topic
    /// goes away.
    pub pose_timeout: f32,
    /// How far back each pose's trail reaches, seconds; 0 draws no trails.
    pub trail: f32,
    pub show_grid: bool,
    pub show_points: bool,
    pub show_frames: bool,
    pub show_labels: bool,
    pub show_poses: bool,
    /// The robot's URDF file; empty for none.
    pub robot_urdf: String,
    /// The joint-state topic that poses the robot; `None` uses the first one advertised.
    pub joint_topic: Option<String>,
    pub show_robot: bool,
    /// 0–1.
    pub robot_opacity: f32,
    /// The `robot_description` topic whose URDF is drawn; `None` uses the first advertised.
    pub description_topic: Option<String>,
    /// The image topic to show; `None` shows the first advertised.
    pub image_topic: Option<String>,
    pub show_image: bool,
    pub show_text: bool,
    pub show_stats: bool,
    /// The panels and how they tile the window.
    #[serde(deserialize_with = "layout_or_default")]
    pub layout: Layout,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            connections: vec!["ws://localhost:8765".into()],
            url: None,
            cloud_topic: None,
            hidden_topics: BTreeSet::new(),
            fixed_frame: None,
            color: ColorChoice::Auto,
            point_size: 2.0,
            decimate: 1,
            // Table-top scale: a 25 cm axis hides a 10 cm object.
            axis_length: 0.1,
            pose_timeout: 2.0,
            trail: 0.0,
            show_grid: true,
            show_points: true,
            show_frames: true,
            show_labels: true,
            show_poses: true,
            robot_urdf: String::new(),
            joint_topic: None,
            show_robot: true,
            robot_opacity: 1.0,
            description_topic: None,
            image_topic: None,
            show_image: true,
            show_text: true,
            show_stats: true,
            layout: Layout::default(),
        }
    }
}

/// The file's layout, or the default with the reason reported when it isn't a valid one (a
/// hand edit, or a panel type from a newer Lantern): the other settings in the file still load.
fn layout_or_default<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Layout, D::Error> {
    let value = serde_json::Value::deserialize(d)?;
    Ok(serde_json::from_value(value).unwrap_or_else(|e| {
        eprintln!("lantern: the layout in the settings file isn't valid ({e}); using the default layout");
        Layout::default()
    }))
}

fn path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("lantern/settings.json"))
}

/// The longest pose trail, seconds (the UI's longest option): every trail is redrawn whole on
/// each display rebuild, so a hand-edited hour of a fast topic would be millions of segments.
const MAX_TRAIL: f32 = 60.0;

impl Settings {
    /// The setting that picks the one topic drawn, for kinds drawn one topic at a time; `None`
    /// for kinds where every topic is drawn (each can be hidden instead).
    pub fn picked(&self, kind: Kind) -> Option<&Option<String>> {
        match kind {
            Kind::PointCloud => Some(&self.cloud_topic),
            Kind::JointStates => Some(&self.joint_topic),
            Kind::Image => Some(&self.image_topic),
            Kind::RobotDescription => Some(&self.description_topic),
            Kind::Transforms | Kind::Poses | Kind::Annotations | Kind::Text | Kind::Stats => None,
        }
    }

    pub fn picked_mut(&mut self, kind: Kind) -> Option<&mut Option<String>> {
        match kind {
            Kind::PointCloud => Some(&mut self.cloud_topic),
            Kind::JointStates => Some(&mut self.joint_topic),
            Kind::Image => Some(&mut self.image_topic),
            Kind::RobotDescription => Some(&mut self.description_topic),
            Kind::Transforms | Kind::Poses | Kind::Annotations | Kind::Text | Kind::Stats => None,
        }
    }

    /// Values a hand-edited file can get wrong, put right; an old single `url` becomes the
    /// connection list.
    fn checked(mut self) -> Settings {
        if self.trail > MAX_TRAIL {
            eprintln!("lantern: trail {} s in the settings file is over the {MAX_TRAIL} s limit; using {MAX_TRAIL} s", self.trail);
        }
        if let Some(url) = self.url.take()
            && !self.connections.contains(&url)
        {
            self.connections = vec![url];
        }
        Settings {
            // Every n-th point, so 0 would draw nothing (and divide by zero).
            decimate: self.decimate.max(1),
            robot_opacity: self.robot_opacity.clamp(0.0, 1.0),
            pose_timeout: self.pose_timeout.max(0.0),
            trail: self.trail.clamp(0.0, MAX_TRAIL),
            ..self
        }
    }

    pub fn load() -> Settings {
        let Some(path) = path() else { return Settings::default() };
        match std::fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<Settings>(&text) {
                Ok(s) => s.checked(),
                Err(e) => {
                    eprintln!("lantern: {} doesn't parse ({e}); using defaults", path.display());
                    Settings::default()
                }
            },
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Settings::default(),
            Err(e) => {
                eprintln!("lantern: can't read {} ({e}); using defaults", path.display());
                Settings::default()
            }
        }
    }

    pub fn save(&self) {
        let Some(path) = path() else { return };
        let write = || -> std::io::Result<()> {
            std::fs::create_dir_all(path.parent().unwrap())?;
            let tmp = path.with_extension("json.tmp");
            std::fs::write(&tmp, serde_json::to_string_pretty(self).unwrap())?;
            std::fs::rename(&tmp, &path)
        };
        if let Err(e) = write() {
            eprintln!("lantern: can't save settings to {} ({e})", path.display());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Files written before several connections name one `url`; that's the list now, and a file
    // with the list keeps it.
    #[test]
    fn an_old_single_url_becomes_the_connection_list() {
        let old: Settings = serde_json::from_str(r#"{"url": "ws://robot:9090"}"#).unwrap();
        assert_eq!(old.checked().connections, ["ws://robot:9090"]);
        let new: Settings = serde_json::from_str(r#"{"connections": ["ws://a:1", "ws://b:2"]}"#).unwrap();
        assert_eq!(new.checked().connections, ["ws://a:1", "ws://b:2"]);
    }

    // A layout this Lantern can't read costs only the layout: the rest of the file still loads.
    #[test]
    fn a_bad_layout_keeps_the_other_settings() {
        let s: Settings = serde_json::from_str(r#"{"connections": ["ws://robot:9090"], "layout": {"panel": "plot"}}"#).unwrap();
        assert_eq!((s.connections, s.layout), (vec!["ws://robot:9090".to_string()], Layout::default()));
    }
}
