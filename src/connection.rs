//! One open source and what it delivered. A connection's `Received` changes with its source's
//! events (`Received::apply`) and ages with the UI clock (`Received::expire`); `State` merges
//! every connection's for display.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::Arc;

use glam::DVec3;

use crate::decode::Decoded;
use crate::scene::{Annotation, Diagnostic, Image, PointCloud, Poses, TfTree};
use crate::source::{self, Channel, Event, Source, Status, Wanted};
use crate::ui::thousands;
use crate::viewport;

/// Where every source's events go, tagged with the connection that sent them (`main.rs` queues
/// them for the UI thread, which hands them to `State::apply`).
pub type Post = Arc<dyn Fn(u64, Event) + Send + Sync>;

/// One open source, what it was last asked for, and what it delivered.
pub struct Connection {
    pub url: String,
    source: Box<dyn Source>,
    /// What `source` was last asked for.
    sent: Wanted,
    /// Tags this connection's events: a removed source's thread can still send a few (a connect
    /// timing out, a cloud mid-decode), and `State::apply` drops them.
    pub(crate) id: u64,
    pub received: Received,
}

impl Connection {
    pub fn open(url: &str, id: u64, post: &Post) -> Connection {
        let post = Arc::clone(post);
        let source = source::open(url, Arc::new(move |e| post(id, e)));
        Connection { url: url.to_string(), source, sent: Wanted::default(), id, received: Received::default() }
    }

    /// Asks the source for `wanted`, unless that's what it was last asked for.
    pub fn want(&mut self, wanted: &Wanted) {
        if *wanted != self.sent {
            self.source.want(wanted.clone());
            self.sent = wanted.clone();
        }
    }
}

/// Everything one connection delivered; a new connection starts from `default()`.
#[derive(Default)]
pub struct Received {
    pub status: Status,
    pub channels: Vec<Channel>,
    /// The drawn cloud and its topic.
    pub cloud: Option<(String, Arc<PointCloud>)>,
    pub tf: TfTree,
    /// The joint-state topic's positions so far, by joint name: messages may carry a subset
    /// (one per driver), so they merge.
    pub joints: Option<(String, HashMap<String, f64>)>,
    /// The newest poses per (topic, label), and when they arrived: one odometry topic can carry
    /// several bodies. Kept until `pose_timeout` passes without a new one, or the topic is
    /// unadvertised or hidden.
    pub poses: BTreeMap<(String, String), (f64, Poses)>,
    /// Where each single-pose (topic, label) has been, newest last.
    pub trails: BTreeMap<(String, String), Trail>,
    /// The shown image topic's newest image.
    pub image: Option<(String, Arc<Image>)>,
    pub annotations: BTreeMap<String, Vec<Annotation>>,
    pub texts: BTreeMap<String, String>,
    pub stats: BTreeMap<String, Vec<Diagnostic>>,
    /// URDF text per `robot_description` topic.
    pub descriptions: BTreeMap<String, String>,
    /// The last error per topic (a decode failure, a cloud over the GPU cap), cleared by its
    /// next good message.
    pub errors: BTreeMap<String, String>,
    /// The server's latest warning or error.
    pub server_warning: Option<String>,
}

/// A pose's positions in `frame_id`, with the UI clock when each arrived.
pub struct Trail {
    pub frame_id: String,
    pub points: VecDeque<(f64, DVec3)>,
}

/// The topics the user picked (or Auto chose) for the kinds where only one is kept.
pub struct Picked {
    pub cloud: Option<String>,
    pub joints: Option<String>,
    pub image: Option<String>,
    /// Trails are on: single poses record where they've been.
    pub trail: bool,
}

/// What an event changed, beyond `Received` itself.
#[derive(Default)]
pub struct Changed {
    /// The 3D display must be derived again.
    pub display: bool,
    /// Joint positions changed: only the robot re-poses.
    pub robot: bool,
    /// The shown image or its annotations changed.
    pub image: bool,
}

impl Received {
    /// Takes one event from this connection's source; `clock` (UI seconds) stamps poses.
    pub fn apply(&mut self, e: Event, clock: f64, picked: &Picked) -> Changed {
        let mut changed = Changed::default();
        let display = match e {
            Event::Status(s) => {
                self.status = s;
                true
            }
            Event::Channels(c) => {
                self.channels = c;
                let advertised: HashSet<String> = self.channels.iter().map(|c| c.topic.clone()).collect();
                let gone = |topic: &String| !advertised.contains(topic);
                self.poses.retain(|(topic, _), _| !gone(topic));
                self.trails.retain(|(topic, _), _| !gone(topic));
                self.errors.retain(|topic, _| !gone(topic));
                self.annotations.retain(|topic, _| !gone(topic));
                self.texts.retain(|topic, _| !gone(topic));
                self.stats.retain(|topic, _| !gone(topic));
                self.descriptions.retain(|topic, _| !gone(topic));
                if self.image.as_ref().is_some_and(|(t, _)| gone(t)) {
                    self.image = None;
                }
                changed.image = true;
                true
            }
            Event::Data { topic, data: Decoded::Cloud(c) } if c.points.len() > viewport::MAX_POINTS => {
                let error = format!("{} points, over the {} the GPU can draw", thousands(c.points.len()), thousands(viewport::MAX_POINTS));
                self.errors.insert(topic, error);
                true
            }
            Event::Data { topic, data } => {
                // Clearing an error changes what the display says, whatever the message.
                let cleared = self.errors.remove(&topic).is_some();
                match data {
                    Decoded::Cloud(c) => {
                        if picked.cloud.as_deref() == Some(topic.as_str()) {
                            self.cloud = Some((topic, Arc::new(c)));
                        }
                        true
                    }
                    Decoded::Transforms(ts) => {
                        ts.into_iter().for_each(|t| self.tf.insert(t));
                        true
                    }
                    Decoded::Poses(mut p) => {
                        // A lone unnamed pose (PoseStamped, PoseInFrame) is named by its topic.
                        if let [item] = &mut p.items[..] {
                            item.label.get_or_insert_with(|| topic.clone());
                        }
                        let label = p.items.first().and_then(|i| i.label.clone()).unwrap_or_default();
                        let key = (topic, label);
                        if let ([item], true) = (&p.items[..], picked.trail) {
                            let t = self.trails.entry(key.clone()).or_insert_with(|| Trail { frame_id: p.frame_id.clone(), points: VecDeque::new() });
                            if t.frame_id != p.frame_id {
                                *t = Trail { frame_id: p.frame_id.clone(), points: VecDeque::new() };
                            }
                            t.points.push_back((clock, item.pose.translation));
                        }
                        self.poses.insert(key, (clock, p));
                        true
                    }
                    // These arrive at the robot's control rate (often 100 Hz or more).
                    Decoded::Joints(js) => {
                        if picked.joints.as_deref() == Some(topic.as_str()) {
                            match &mut self.joints {
                                Some((t, positions)) if *t == topic => positions.extend(js),
                                other => *other = Some((topic, js.into_iter().collect())),
                            }
                            changed.robot = true;
                        }
                        cleared
                    }
                    Decoded::Image(i) => {
                        if picked.image.as_deref() == Some(topic.as_str()) {
                            self.image = Some((topic, Arc::new(i)));
                            changed.image = true;
                        }
                        cleared
                    }
                    Decoded::Annotations(a) => {
                        self.annotations.insert(topic, a);
                        changed.image = true;
                        cleared
                    }
                    Decoded::Text(t) => {
                        self.texts.insert(topic, t);
                        cleared
                    }
                    Decoded::Diagnostics(d) => {
                        self.stats.insert(topic, d);
                        cleared
                    }
                    Decoded::RobotDescription(urdf) => {
                        self.descriptions.insert(topic, urdf);
                        cleared
                    }
                }
            }
            Event::DecodeFailed { topic, error } => {
                self.errors.insert(topic, error);
                true
            }
            Event::ServerWarning(w) => {
                self.server_warning = Some(w);
                true
            }
        };
        changed.display = display;
        changed
    }

    /// Drops poses older than `timeout` seconds (0: never) and trail points older than `trail`
    /// (0: no trails). Returns whether anything went, and when the next is due.
    pub fn expire(&mut self, now: f64, timeout: f64, trail: f64) -> (bool, Option<f64>) {
        let mut due: Option<f64> = None;
        let mut soonest = |t: f64| due = Some(due.map_or(t, |d| d.min(t)));
        let mut dropped = false;
        if timeout > 0.0 {
            let before = self.poses.len();
            self.poses.retain(|_, (at, _)| now - *at < timeout);
            dropped |= self.poses.len() != before;
            self.poses.values().for_each(|(at, _)| soonest(at + timeout));
        }
        let poses = &self.poses;
        self.trails.retain(|key, t| {
            while t.points.front().is_some_and(|(at, _)| trail <= 0.0 || now - at >= trail) {
                t.points.pop_front();
                dropped = true;
            }
            poses.contains_key(key) && !t.points.is_empty()
        });
        self.trails.values().filter_map(|t| t.points.front()).for_each(|(at, _)| soonest(at + trail));
        (dropped, due)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{Pose, Tracked};

    fn pose_at(x: f64) -> Event {
        let item = Tracked { pose: Pose { translation: DVec3::new(x, 0.0, 0.0), ..Pose::IDENTITY }, label: None, velocity: None };
        Event::Data { topic: "/tracked".into(), data: Decoded::Poses(Poses { frame_id: "world".into(), items: vec![item] }) }
    }

    // A pose sent at 0, 1 and 2 s, with a 2 s timeout and a 1.5 s trail: the trail
    // at 2.4 s keeps the points from 1 and 2 s and wakes when the 1 s one is due, at 2.5; once the
    // topic goes quiet, the pose and its trail go at 2 + 2.
    #[test]
    fn poses_expire_and_trails_keep_only_their_window() {
        let picked = Picked { cloud: None, joints: None, image: None, trail: true };
        let mut r = Received::default();
        for t in [0.0, 1.0, 2.0] {
            r.apply(pose_at(t), t, &picked);
        }
        let key = ("/tracked".to_string(), "/tracked".to_string());
        assert_eq!(r.expire(2.4, 2.0, 1.5), (true, Some(2.5)));
        assert_eq!(r.trails[&key].points.iter().map(|(at, _)| *at).collect::<Vec<_>>(), [1.0, 2.0]);
        assert!(r.poses.contains_key(&key), "labelled by its topic, and still fresh");
        let (dropped, _) = r.expire(4.0, 2.0, 1.5);
        assert!(dropped && r.poses.is_empty() && r.trails.is_empty());
    }
}
