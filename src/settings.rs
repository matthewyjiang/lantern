//! What the user chose, kept across runs in `$XDG_CONFIG_HOME/lantern/settings.json` (or
//! `~/.config/...`). Saved whenever it changes; a file that doesn't parse is reported and
//! replaced by defaults rather than stopping the app.

use std::collections::BTreeSet;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use crate::decode::ColorChoice;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// The Foxglove WebSocket server: foxglove_bridge or an SDK app (both default to 8765).
    pub url: String,
    /// The point cloud topic to draw; `None` draws the first one advertised.
    pub cloud_topic: Option<String>,
    /// Pose topics the user switched off (new topics show by default).
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
    pub show_grid: bool,
    pub show_points: bool,
    pub show_frames: bool,
    pub show_labels: bool,
    pub show_poses: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            url: "ws://localhost:8765".into(),
            cloud_topic: None,
            hidden_topics: BTreeSet::new(),
            fixed_frame: None,
            color: ColorChoice::Auto,
            point_size: 2.0,
            decimate: 1,
            axis_length: 0.25,
            show_grid: true,
            show_points: true,
            show_frames: true,
            show_labels: true,
            show_poses: true,
        }
    }
}

fn path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("lantern/settings.json"))
}

impl Settings {
    pub fn load() -> Settings {
        let Some(path) = path() else { return Settings::default() };
        match std::fs::read_to_string(&path) {
            Ok(text) => match serde_json::from_str::<Settings>(&text) {
                // Every n-th point, so 0 would draw nothing (and divide by zero).
                Ok(s) => Settings { decimate: s.decimate.max(1), ..s },
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
