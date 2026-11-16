//! Lantern's own data model: what the renderer and UI understand. Decoders turn wire messages
//! (ROS 2 CDR, Foxglove protobuf) into these types; nothing past `decode` knows a message type.

use std::collections::HashMap;

use glam::{DQuat, DVec3, Mat4};

/// A rigid transform, metres and a unit quaternion (ROS and Foxglove conventions agree).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: DVec3,
    pub rotation: DQuat,
}

impl Pose {
    pub const IDENTITY: Pose = Pose { translation: DVec3::ZERO, rotation: DQuat::IDENTITY };

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

/// The `lo` and `hi` percentiles (0–1) of `values`, nearest rank; `None` when empty.
pub fn percentiles(mut values: Vec<f32>, lo: f32, hi: f32) -> Option<(f32, f32)> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    let at = |p: f32| values[((values.len() - 1) as f32 * p).round() as usize];
    Some((at(lo), at(hi)))
}

/// The latest transform for every child frame: enough to place data from any connected frame
/// in a display frame. No history: Lantern draws the newest message of each topic with the
/// newest transforms, not a time-synchronised past.
#[derive(Clone, Debug, Default)]
pub struct TfTree {
    parents: HashMap<String, (String, Pose)>,
}

impl TfTree {
    pub fn insert(&mut self, t: Transform) {
        if t.parent != t.child {
            self.parents.insert(t.child, (t.parent, t.pose));
        }
    }

    /// Every frame named so far, sorted.
    pub fn frames(&self) -> Vec<String> {
        let mut all: Vec<String> = self.parents.iter().flat_map(|(c, (p, _))| [c.clone(), p.clone()]).collect();
        all.sort();
        all.dedup();
        all
    }

    /// Parent/child pairs, for drawing the tree.
    pub fn edges(&self) -> impl Iterator<Item = (&str, &str)> {
        self.parents.iter().map(|(c, (p, _))| (p.as_str(), c.as_str()))
    }

    /// `root_T_frame` and the root's name, walking up the parents. Cycles (bad data) stop at the
    /// frame where they close.
    fn to_root(&self, frame: &str) -> (String, Pose) {
        let mut pose = Pose::IDENTITY;
        let mut at = frame.to_string();
        for _ in 0..=self.parents.len() {
            let Some((parent, p)) = self.parents.get(&at) else { break };
            pose = p.then(&pose);
            at = parent.clone();
        }
        (at, pose)
    }

    /// The root of the tree `frame` belongs to (itself, if it has no parent).
    pub fn root_of(&self, frame: &str) -> String {
        self.to_root(frame).0
    }

    /// `target_T_source`: maps points in `source` into `target`, if both are in one tree.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Pose> {
        if target == source {
            return Some(Pose::IDENTITY);
        }
        let (root_t, root_from_target) = self.to_root(target);
        let (root_s, root_from_source) = self.to_root(source);
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

        // The lidar sits 1 m ahead of a base turned 90°, so 1 m along map +y from the base.
        let p = tree.lookup("map", "lidar").unwrap();
        assert!((p.translation - DVec3::new(2.0, 1.0, 0.5)).length() < 1e-9, "{:?}", p.translation);
        // A point at the lidar's origin seen from the camera: map (2,1,0.5) minus camera (0,3,1).
        let c = tree.lookup("camera", "lidar").unwrap();
        assert!((c.translation - DVec3::new(2.0, -2.0, -0.5)).length() < 1e-9, "{:?}", c.translation);
        assert_eq!(tree.root_of("lidar"), "map");
        assert!(tree.lookup("map", "wheel").is_none(), "odom tree is not connected to map");
    }
}
