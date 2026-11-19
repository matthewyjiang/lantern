//! What the user chose, kept across runs in `$XDG_CONFIG_HOME/lantern/settings.json` (or
//! `~/.config/...`). Saved whenever it changes.
//!
//! The file holds only what's app-wide: the servers and the layout. Each panel's own settings
//! (what the 3D view draws, its robot, which topic an Image panel shows, ...) live on that panel
//! in the layout, as the types here. A file that doesn't parse (a hand edit, or one an older or
//! newer Lantern wrote) is reported, copied to `settings.json.bak`, and replaced by defaults.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

use serde::{Deserialize, Serialize};

use crate::decode::ColorChoice;
use crate::layout::Layout;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Settings {
    /// The Foxglove WebSocket servers to read, all at once: foxglove_bridge or SDK apps (both
    /// default to port 8765). Topics of the same name on two servers feed one display.
    pub connections: Vec<String>,
    /// The panels, how they tile the window, and each one's settings.
    pub layout: Layout,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { connections: vec!["ws://localhost:8765".into()], layout: Layout::default() }
    }
}

/// The 3D view's settings: what it draws, and the cards over it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct View3dSettings {
    /// The frame the scene is drawn in; `None` uses the root of the TF tree the data is in.
    pub fixed_frame: Option<String>,
    pub grid: bool,
    /// TF frames' axes and the lines between them.
    pub frames: bool,
    /// Frame and pose names.
    pub labels: bool,
    /// Length of TF and pose axes, metres.
    pub axis_length: f32,
    pub points: PointsSettings,
    pub poses: PosesSettings,
    pub robot: RobotSettings,
    pub objects: ObjectsSettings,
    /// The camera image as a card in the view's corner, when the layout has no Image panel.
    pub image_card: bool,
    /// What that card shows.
    pub image: ImageSettings,
    /// Text and stats as cards over the view, when the layout has no Data panel.
    pub data: DataSettings,
}

impl Default for View3dSettings {
    fn default() -> Self {
        View3dSettings {
            fixed_frame: None,
            grid: true,
            frames: true,
            labels: true,
            // Table-top scale: a 25 cm axis hides a 10 cm object.
            axis_length: 0.1,
            points: PointsSettings::default(),
            poses: PosesSettings::default(),
            robot: RobotSettings::default(),
            objects: ObjectsSettings::default(),
            image_card: true,
            image: ImageSettings::default(),
            data: DataSettings::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PointsSettings {
    pub show: bool,
    /// The point cloud topic to draw; `None` draws the first one advertised.
    pub topic: Option<String>,
    pub color: ColorChoice,
    /// Point size, in points.
    pub size: f32,
    /// Draw every n-th point.
    pub decimate: u32,
}

impl Default for PointsSettings {
    fn default() -> Self {
        PointsSettings { show: true, topic: None, color: ColorChoice::Auto, size: 2.0, decimate: 1 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PosesSettings {
    pub show: bool,
    /// Pose topics switched off (new topics show by default).
    pub hidden: BTreeSet<String>,
    /// A pose not refreshed for this long is dropped, seconds; 0 keeps poses until their topic
    /// goes away.
    pub timeout: f32,
    /// How far back each pose's trail reaches, seconds; 0 draws no trails.
    pub trail: f32,
}

impl Default for PosesSettings {
    fn default() -> Self {
        PosesSettings { show: true, hidden: BTreeSet::new(), timeout: 2.0, trail: 0.0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RobotSettings {
    pub show: bool,
    /// The robot's URDF file; empty for none.
    pub urdf: String,
    /// The `robot_description` topic whose URDF is drawn; `None` uses the first advertised.
    pub description_topic: Option<String>,
    /// The joint-state topic that poses the robot; `None` uses the first one advertised.
    pub joint_topic: Option<String>,
    /// 0–1.
    pub opacity: f32,
}

impl Default for RobotSettings {
    fn default() -> Self {
        RobotSettings { show: true, urdf: String::new(), description_topic: None, joint_topic: None, opacity: 1.0 }
    }
}

/// Objects: the shapes and meshes `SceneUpdate` topics send, and meshes fixed to TF frames.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ObjectsSettings {
    pub show: bool,
    /// Scene topics switched off (new topics show by default).
    pub hidden: BTreeSet<String>,
    /// Meshes drawn at TF frames, for fixtures no topic sends (a table, a bin).
    pub meshes: Vec<FrameMesh>,
}

impl Default for ObjectsSettings {
    fn default() -> Self {
        ObjectsSettings { show: true, hidden: BTreeSet::new(), meshes: Vec::new() }
    }
}

/// A mesh file drawn at a TF frame, offset by `xyz` and `rpy` as a URDF origin is.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameMesh {
    pub frame: String,
    /// A GLB, glTF or STL file (taken as Y-up, as Foxglove does): a path, `file://` or
    /// `package://` URL; a relative path is from the working directory.
    pub file: String,
    /// Offset from the frame, metres.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub xyz: [f64; 3],
    /// Roll, pitch, yaw about the frame's fixed x, y, z axes, radians.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub rpy: [f64; 3],
    #[serde(default = "unit_scale", skip_serializing_if = "is_unit")]
    pub scale: [f64; 3],
    /// sRGB and alpha, 0–1, replacing the file's colours; `None` keeps them.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<[f32; 4]>,
}

fn is_zero(v: &[f64; 3]) -> bool {
    *v == [0.0; 3]
}

fn unit_scale() -> [f64; 3] {
    [1.0; 3]
}

fn is_unit(v: &[f64; 3]) -> bool {
    *v == unit_scale()
}

/// An Image panel's settings (and the 3D view's image card's).
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImageSettings {
    /// The image topic to show; `None` shows the first advertised.
    pub topic: Option<String>,
    /// Annotation topics not drawn over it (new topics are drawn).
    pub hidden: BTreeSet<String>,
}

/// A Data panel's settings (and the 3D view's text and stats cards').
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DataSettings {
    pub text: bool,
    pub stats: bool,
    /// Text and stats topics not shown (new topics are shown).
    pub hidden: BTreeSet<String>,
}

impl Default for DataSettings {
    fn default() -> Self {
        DataSettings { text: true, stats: true, hidden: BTreeSet::new() }
    }
}

/// The longest pose trail, seconds (the UI's longest option): every trail is redrawn whole on
/// each display rebuild, so a hand-edited hour of a fast topic would be millions of segments.
pub const MAX_TRAIL: f32 = 60.0;

impl View3dSettings {
    /// Everything off: what a layout with no 3D panel draws (so it subscribes to nothing 3D).
    pub fn off() -> View3dSettings {
        let d = View3dSettings::default();
        View3dSettings {
            grid: false,
            frames: false,
            labels: false,
            points: PointsSettings { show: false, ..d.points },
            poses: PosesSettings { show: false, ..d.poses },
            robot: RobotSettings { show: false, ..d.robot },
            objects: ObjectsSettings { show: false, ..d.objects },
            ..d
        }
    }

    /// Values a hand-edited file can get wrong, put right.
    pub fn checked(mut self) -> View3dSettings {
        if self.poses.trail > MAX_TRAIL {
            eprintln!("lantern: trail {} s is over the {MAX_TRAIL} s limit; using {MAX_TRAIL} s", self.poses.trail);
        }
        // Every n-th point, so 0 would draw nothing (and divide by zero).
        self.points.decimate = self.points.decimate.max(1);
        self.robot.opacity = self.robot.opacity.clamp(0.0, 1.0);
        self.poses.timeout = self.poses.timeout.max(0.0);
        self.poses.trail = self.poses.trail.clamp(0.0, MAX_TRAIL);
        self
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
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Settings::default(),
            Err(e) => {
                eprintln!("lantern: can't read {} ({e}); using defaults", path.display());
                return Settings::default();
            }
        };
        serde_json::from_str(&text).unwrap_or_else(|e| {
            eprintln!("lantern: {} doesn't parse ({e}); using defaults", path.display());
            if !back_up(&path) {
                eprintln!("lantern: so as not to lose it, settings won't be saved this run");
                KEEP_FILE.store(true, Ordering::Relaxed);
            }
            Settings::default()
        })
    }

    pub fn save(&self) {
        let Some(path) = path() else { return };
        if KEEP_FILE.load(Ordering::Relaxed) {
            return;
        }
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

/// Set when a settings file that didn't parse couldn't be backed up: saving would lose it.
static KEEP_FILE: AtomicBool = AtomicBool::new(false);

/// Copies the settings file to `settings.json.bak` (or `.bak.1`, `.bak.2`, ... next to an
/// earlier backup, which stays) before Lantern's next save replaces it: what didn't parse stays
/// on disk to fix and put back. Whether it was copied.
fn back_up(path: &Path) -> bool {
    let copy = |bak: &Path| -> std::io::Result<()> {
        let mut to = std::fs::OpenOptions::new().write(true).create_new(true).open(bak)?;
        std::io::copy(&mut std::fs::File::open(path)?, &mut to)?;
        Ok(())
    };
    let first = path.with_extension("json.bak");
    let mut bak = first.clone();
    for n in 1.. {
        match copy(&bak) {
            Ok(()) => {
                eprintln!("lantern: the file as it was is in {}", bak.display());
                return true;
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => bak = first.with_extension(format!("bak.{n}")),
            Err(e) => {
                eprintln!("lantern: can't back up {} to {} ({e})", path.display(), bak.display());
                return false;
            }
        }
    }
    unreachable!("there's always a free name")
}

#[cfg(test)]
mod tests {
    use super::*;

    // What's saved is only what's app-wide, and reads back as itself.
    #[test]
    fn a_saved_file_holds_the_servers_and_the_layout_and_reads_back() {
        let mut s = Settings::default();
        s.layout.view3d_mut().unwrap().robot.urdf = "arm.urdf".into();
        let saved = serde_json::to_value(&s).unwrap();
        assert_eq!(saved.as_object().unwrap().keys().collect::<Vec<_>>(), ["connections", "layout"]);
        assert_eq!(serde_json::from_value::<Settings>(saved).unwrap(), s);
    }

    // A second file that doesn't parse is backed up beside the first, which stays.
    #[test]
    fn backups_dont_replace_earlier_ones() {
        let dir = std::env::temp_dir().join(format!("lantern-bak-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("settings.json");
        for text in ["old", "typo"] {
            std::fs::write(&path, text).unwrap();
            assert!(back_up(&path));
        }
        let read = |name: &str| std::fs::read_to_string(dir.join(name)).unwrap();
        assert_eq!((read("settings.json.bak"), read("settings.json.bak.1")), ("old".into(), "typo".into()));
        std::fs::remove_dir_all(&dir).unwrap();
    }

    // A key that isn't app-wide (a panel setting at the top level, as files from before
    // per-panel settings have them) is a mistake to report, not something to drop quietly.
    #[test]
    fn a_panel_setting_at_the_top_level_doesnt_parse() {
        let err = serde_json::from_str::<Settings>(r#"{"connections": [], "robot_urdf": "arm.urdf"}"#).unwrap_err();
        assert!(err.to_string().contains("unknown field `robot_urdf`"), "{err}");
    }
}
