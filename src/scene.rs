//! Lantern's own data model: what the renderer and UI understand. Decoders turn wire messages
//! (ROS 2 CDR, Foxglove protobuf) into these types; nothing past `decode` knows a message type.

use std::collections::{HashMap, VecDeque};

use glam::{DQuat, DVec3, Mat4};

/// A rigid transform, metres and a unit quaternion (ROS and Foxglove conventions agree).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: DVec3,
    pub rotation: DQuat,
}

impl Pose {
    pub const IDENTITY: Pose = Pose { translation: DVec3::ZERO, rotation: DQuat::IDENTITY };

    /// A URDF-style offset: `xyz`, then roll, pitch and yaw about the fixed x, y and z axes.
    pub fn from_xyz_rpy(xyz: DVec3, rpy: DVec3) -> Pose {
        let rotation = DQuat::from_rotation_z(rpy.z) * DQuat::from_rotation_y(rpy.y) * DQuat::from_rotation_x(rpy.x);
        Pose { translation: xyz, rotation }
    }

    pub fn matrix(&self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation.as_quat(), self.translation.as_vec3())
    }

    /// `self ∘ other`: apply `other`, then `self`.
    pub fn then(&self, other: &Pose) -> Pose {
        Pose { translation: self.translation + self.rotation * other.translation, rotation: self.rotation * other.rotation }
    }

    pub fn inverse(&self) -> Pose {
        let r = self.rotation.inverse();
        Pose { translation: -(r * self.translation), rotation: r }
    }
}

/// Where the colour of each point comes from; also what the fourth float of every point holds.
#[derive(Clone, Debug, PartialEq)]
pub enum PointColor {
    /// No colour data: shade by height in the display frame. `w` is unused.
    Height,
    /// A numeric field, colour-mapped over the cloud's own range. `w` is the field's value.
    Field { name: String, min: f32, max: f32 },
    /// Per-point colour: `w` holds the bits of a packed `0xAARRGGBB`.
    Rgb,
}

/// One point cloud message, decoded.
#[derive(Clone, Debug)]
pub struct PointCloud {
    pub frame_id: String,
    /// The cloud's origin in `frame_id` (Foxglove clouds carry one; ROS clouds are identity).
    pub origin: Pose,
    /// x, y, z in the cloud's own frame, then the colour value `color` describes. Non-finite
    /// points are dropped while decoding.
    pub points: Vec<[f32; 4]>,
    pub color: PointColor,
    /// Every field the message had, so the UI can offer them as colour sources.
    pub fields: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    /// When it holds, nanoseconds on the publisher's clock (the header stamp).
    pub stamp: i64,
    /// Child frame expressed in the parent: `parent_T_child`.
    pub pose: Pose,
}

/// Tracked things (objects, markers, a robot base) in one frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Poses {
    pub frame_id: String,
    pub items: Vec<Tracked>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tracked {
    pub pose: Pose,
    /// The thing's name, when the message gives one (an odometry's body frame).
    pub label: Option<String>,
    /// Linear velocity in `frame_id`, m/s, when the message gives one.
    pub velocity: Option<DVec3>,
}

impl Tracked {
    pub fn at(pose: Pose) -> Tracked {
        Tracked { pose, label: None, velocity: None }
    }
}

/// A camera image, decoded: `width × height` straight-alpha sRGB pixels, rows top first.
#[derive(Clone, Debug)]
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

/// A shape drawn over an image, in its pixel coordinates (origin top-left, y down). Colours are
/// sRGB 0–1 RGBA as Foxglove gives them; alpha 0 draws nothing (an unset fill).
#[derive(Clone, Debug, PartialEq)]
pub enum Annotation {
    Circle { center: [f32; 2], diameter: f32, thickness: f32, outline: [f32; 4], fill: [f32; 4] },
    Points { kind: PointsKind, points: Vec<[f32; 2]>, color: [f32; 4], fill: [f32; 4], thickness: f32 },
    Text { at: [f32; 2], text: String, size: f32, color: [f32; 4], background: [f32; 4] },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointsKind {
    /// A dot at each point.
    Points,
    /// A closed outline.
    Loop,
    /// An open polyline.
    Strip,
    /// Separate segments, two points each.
    Lines,
}

/// One `SceneUpdate` message: entities to remove, then entities to add or replace. Entities are
/// per topic, by id.
#[derive(Clone, Debug, PartialEq)]
pub struct SceneUpdate {
    pub deletions: Vec<Deletion>,
    pub entities: Vec<Entity>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Deletion {
    /// The topic's entity with this id.
    Id(String),
    /// Every entity of the topic.
    All,
}

/// Shapes placed together in one frame, replaced as a whole when an entity with its id arrives.
#[derive(Clone, Debug, PartialEq)]
pub struct Entity {
    pub id: String,
    pub frame_id: String,
    /// `None` follows the frame as TF moves it (`frame_locked`). `Some(t)` stays where the frame
    /// was at `t`, the entity's `timestamp` in nanoseconds on the publisher's clock.
    pub placed_at: Option<i64>,
    /// Dropped this many seconds after it arrives; `None` keeps it until replaced or deleted.
    pub lifetime: Option<f64>,
    pub shapes: Vec<Shape>,
}

/// One primitive of an entity, at `pose` in the entity's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    pub pose: Pose,
    /// sRGB and alpha, 0–1. A model uses it only with `override_color`.
    pub color: [f32; 4],
    pub geometry: Geometry,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    /// A box `size` across, centred on the pose.
    Cube { size: DVec3 },
    /// An ellipsoid `size` across.
    Sphere { size: DVec3 },
    /// Along z, `size` its bounding box; `bottom` and `top` scale the radius at each end (a cone
    /// has one at 0).
    Cylinder { size: DVec3, bottom: f64, top: f64 },
    /// From the pose along +x: a shaft, then a cone.
    Arrow { shaft_length: f64, shaft_diameter: f64, head_length: f64, head_diameter: f64 },
    /// `points` as Foxglove's indices give them, each with its colour when `colors` isn't empty.
    Line { kind: LineKind, points: Vec<DVec3>, colors: Vec<[f32; 4]>, thickness: Thickness },
    /// Three points per triangle, in one colour.
    Triangles { points: Vec<DVec3> },
    Text { text: String, size: Thickness },
    /// A GLB, glTF or STL mesh, scaled along its axes after it's turned Z-up.
    Model { source: ModelSource, scale: DVec3, override_color: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineKind {
    /// An open polyline.
    Strip,
    /// A closed one.
    Loop,
    /// Separate segments, two points each.
    List,
}

/// A line's width or a text's height: on screen, or in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Thickness {
    Points(f64),
    Metres(f64),
}

/// Where a model's mesh comes from.
#[derive(Clone, Debug, PartialEq)]
pub enum ModelSource {
    /// An absolute path, `file://` or `package://` URL.
    Url(String),
    /// Mesh bytes sent in the message, with their media type (`model/gltf-binary`, `model/stl`,
    /// or empty) and a hash of both that names them.
    Data { media_type: String, bytes: std::sync::Arc<[u8]>, hash: u64 },
}

/// One status of a `diagnostic_msgs/DiagnosticArray`: a named component's health and readings.
#[derive(Clone, Debug, PartialEq)]
pub struct Diagnostic {
    pub level: Level,
    pub name: String,
    pub message: String,
    pub values: Vec<(String, String)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Ok,
    Warn,
    Error,
    Stale,
}

impl Level {
    /// `DiagnosticStatus.level`: 0 OK, 1 WARN, 2 ERROR, 3 STALE; anything else reads as an error.
    pub fn from_byte(b: u8) -> Level {
        match b {
            0 => Level::Ok,
            1 => Level::Warn,
            3 => Level::Stale,
            _ => Level::Error,
        }
    }
}

/// The `lo` and `hi` percentiles (0–1) of `values`, nearest rank; `None` when empty.
pub fn percentiles(mut values: Vec<f32>, lo: f32, hi: f32) -> Option<(f32, f32)> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    let at = |p: f32| values[((values.len() - 1) as f32 * p).round() as usize];
    Some((at(lo), at(hi)))
}

/// The most transforms kept per child frame, as Foxglove's 3D panel keeps
/// (`DEFAULT_MAX_CAPACITY_PER_FRAME`): 100 s of history at 100 Hz. Older ones are dropped.
pub const TF_HISTORY: usize = 10_000;

/// One connection's transforms: for every child frame its parent, the transform that arrived
/// last, and up to `TF_HISTORY` past ones by stamp, for placing data at a past time.
#[derive(Clone, Debug, Default)]
pub struct TfTree {
    parents: HashMap<String, Edge>,
}

#[derive(Clone, Debug)]
struct Edge {
    parent: String,
    latest: Pose,
    /// Sorted by stamp, one transform per stamp; never empty.
    history: VecDeque<(i64, Pose)>,
}

impl Edge {
    /// `parent_T_child` at `stamp`, as Foxglove looks it up: the transform at that stamp,
    /// interpolated between the two around it, or the nearest when it's outside the history.
    fn at(&self, stamp: i64) -> Pose {
        let h = &self.history;
        let i = h.partition_point(|(t, _)| *t < stamp);
        match (i.checked_sub(1).map(|k| &h[k]), h.get(i)) {
            (Some((t0, a)), Some((t1, b))) if *t1 != stamp => {
                // In i128: stamps far apart (bad data) mustn't overflow.
                let f = (i128::from(stamp) - i128::from(*t0)) as f64 / (i128::from(*t1) - i128::from(*t0)) as f64;
                Pose { translation: a.translation.lerp(b.translation, f), rotation: a.rotation.slerp(b.rotation, f) }
            }
            (_, Some((_, p))) | (Some((_, p)), None) => *p,
            (None, None) => unreachable!("an edge's history is never empty"),
        }
    }
}

impl TfTree {
    /// Adds a transform. A child given a new parent starts a new history.
    pub fn insert(&mut self, t: Transform) {
        if t.parent == t.child {
            return;
        }
        let edge = match self.parents.get_mut(&t.child) {
            Some(edge) if edge.parent == t.parent => edge,
            _ => {
                self.parents.insert(t.child, Edge { parent: t.parent, latest: t.pose, history: VecDeque::from([(t.stamp, t.pose)]) });
                return;
            }
        };
        edge.latest = t.pose;
        let h = &mut edge.history;
        let i = h.partition_point(|(s, _)| *s < t.stamp);
        match h.get_mut(i) {
            Some((s, p)) if *s == t.stamp => *p = t.pose,
            _ => h.insert(i, (t.stamp, t.pose)),
        }
        if h.len() > TF_HISTORY {
            h.pop_front();
        }
    }
}

/// Several connections' transforms read as one tree: where two have the same child frame, the
/// later one's wins.
pub struct Tf<'a>(pub Vec<&'a TfTree>);

impl Tf<'_> {
    fn edge(&self, child: &str) -> Option<&Edge> {
        self.0.iter().rev().find_map(|t| t.parents.get(child))
    }

    fn edges_by_child(&self) -> HashMap<&str, &Edge> {
        self.0.iter().flat_map(|t| &t.parents).map(|(c, e)| (c.as_str(), e)).collect()
    }

    /// Every frame named so far, sorted.
    pub fn frames(&self) -> Vec<String> {
        let mut all: Vec<String> = self.edges_by_child().into_iter().flat_map(|(c, e)| [c.to_string(), e.parent.clone()]).collect();
        all.sort();
        all.dedup();
        all
    }

    /// Parent/child pairs, for drawing the tree.
    pub fn edges(&self) -> Vec<(&str, &str)> {
        self.edges_by_child().into_iter().map(|(c, e)| (e.parent.as_str(), c)).collect()
    }

    /// `root_T_frame` and the root's name, walking up the parents with the latest transforms,
    /// or those at `stamp`. Cycles (bad data) stop at the frame where they close.
    fn to_root(&self, frame: &str, stamp: Option<i64>) -> (String, Pose) {
        let mut pose = Pose::IDENTITY;
        let mut at = frame;
        for _ in 0..=self.0.iter().map(|t| t.parents.len()).sum::<usize>() {
            let Some(e) = self.edge(at) else { break };
            pose = stamp.map_or(e.latest, |s| e.at(s)).then(&pose);
            at = &e.parent;
        }
        (at.to_string(), pose)
    }

    /// The root of the tree `frame` belongs to (itself, if it has no parent).
    pub fn root_of(&self, frame: &str) -> String {
        self.to_root(frame, None).0
    }

    /// `target_T_source`, if both are in one tree: with the latest transforms, or for something
    /// that stays where `source` was at `stamp`, as Foxglove places it: from `source` into the
    /// tree's root with the transforms at `stamp`, then into `target` with the latest.
    pub fn lookup(&self, target: &str, source: &str, stamp: Option<i64>) -> Option<Pose> {
        if target == source && stamp.is_none() {
            return Some(Pose::IDENTITY);
        }
        let (root_t, root_from_target) = self.to_root(target, None);
        let (root_s, root_from_source) = self.to_root(source, stamp);
        (root_t == root_s).then(|| root_from_target.inverse().then(&root_from_source))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tf(parent: &str, child: &str, t: [f64; 3], yaw_deg: f64) -> Transform {
        Transform {
            parent: parent.into(),
            child: child.into(),
            stamp: 0,
            pose: Pose { translation: t.into(), rotation: DQuat::from_rotation_z(yaw_deg.to_radians()) },
        }
    }

    #[test]
    fn lookup_chains_through_the_common_root_and_misses_across_trees() {
        let mut tree = TfTree::default();
        tree.insert(tf("map", "base", [2.0, 0.0, 0.0], 90.0));
        tree.insert(tf("base", "lidar", [1.0, 0.0, 0.5], 0.0));
        tree.insert(tf("map", "camera", [0.0, 3.0, 1.0], 0.0));
        tree.insert(tf("odom", "wheel", [0.0, 0.0, 0.0], 0.0));
        let tree = Tf(vec![&tree]);

        // The lidar sits 1 m ahead of a base turned 90°, so 1 m along map +y from the base.
        let p = tree.lookup("map", "lidar", None).unwrap();
        assert!((p.translation - DVec3::new(2.0, 1.0, 0.5)).length() < 1e-9, "{:?}", p.translation);
        // A point at the lidar's origin seen from the camera: map (2,1,0.5) minus camera (0,3,1).
        let c = tree.lookup("camera", "lidar", None).unwrap();
        assert!((c.translation - DVec3::new(2.0, -2.0, -0.5)).length() < 1e-9, "{:?}", c.translation);
        assert_eq!(tree.root_of("lidar"), "map");
        assert!(tree.lookup("map", "wheel", None).is_none(), "odom tree is not connected to map");
    }

    // A gripper moving along map x at 1 m/s from t = 1 s, its transforms sent late and out of
    // order. Looked up at a stamp, it's where it was then: between two transforms, part way
    // from one to the other; before or after them all, at the nearest. The latest lookup takes
    // the one that arrived last.
    #[test]
    fn lookups_at_a_stamp_interpolate_the_history_and_clamp_outside_it() {
        let s = 1_000_000_000;
        let at = |stamp: i64, x: f64| Transform { stamp, ..tf("map", "gripper", [x, 0.0, 0.0], 0.0) };
        let mut tree = TfTree::default();
        for t in [at(3 * s, 2.0), at(s, 0.0), at(2 * s, 1.0)] {
            tree.insert(t);
        }
        let view = Tf(vec![&tree]);
        let x = |stamp| view.lookup("map", "gripper", Some(stamp)).unwrap().translation.x;
        assert_eq!([x(s), x(5 * s / 2), x(0), x(9 * s)], [0.0, 1.5, 0.0, 2.0]);
        assert_eq!(view.lookup("map", "gripper", None).unwrap().translation.x, 1.0, "arrived last");

        // Seen from the gripper, something left at 2 s stays at map x 1 as the gripper moves on.
        tree.insert(at(4 * s, 3.0));
        assert_eq!(Tf(vec![&tree]).lookup("gripper", "gripper", Some(2 * s)).unwrap().translation.x, -2.0);

        // A new parent starts over: the old history was relative to another frame.
        tree.insert(Transform { stamp: 5 * s, ..tf("odom", "gripper", [7.0, 0.0, 0.0], 0.0) });
        assert_eq!(Tf(vec![&tree]).lookup("odom", "gripper", Some(s)).unwrap().translation.x, 7.0);
    }
}
