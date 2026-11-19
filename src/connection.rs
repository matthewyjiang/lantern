//! One open source and what it delivered. A connection's `Received` changes with its source's
//! events (`Received::apply`) and ages with the UI clock (`Received::expire`); `State` merges
//! every connection's for display.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::Arc;

use glam::DVec3;

use crate::decode::Decoded;
use crate::scene::{Annotation, Deletion, Diagnostic, Entity, Image, PointCloud, Poses, TfTree};
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
    /// Scene entities per (topic, id), with when each expires (UI clock), if it does. An
    /// entity stays until its topic sends one of the same id or deletes it, its lifetime
    /// passes, or the topic is unsubscribed (`State::sync`).
    pub objects: BTreeMap<(String, String), (Option<f64>, Entity)>,
    /// Where each single-pose (topic, label) has been, newest last.
    pub trails: BTreeMap<(String, String), Trail>,
    /// The newest image of each image topic some view shows.
    pub images: BTreeMap<String, Arc<Image>>,
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

/// The topics the user picked (or Auto chose) for the kinds where only the shown ones are kept:
/// the one cloud and joint-state topic the 3D view draws, every image topic a view shows.
#[derive(Default)]
pub struct Picked {
    pub cloud: Option<String>,
    pub joints: Option<String>,
    pub images: HashSet<String>,
}

/// What an event changed, beyond `Received` itself.
#[derive(Default)]
pub struct Changed {
    /// The 3D display must be derived again.
    pub display: bool,
    /// Joint positions changed: only the robot re-poses.
    pub robot: bool,
    /// A shown image or annotations changed.
    pub image: bool,
}

impl Received {
    /// Takes one event from this connection's source; `clock` (UI seconds) stamps poses, and
    /// with `trail` on, single poses record where they've been.
    pub fn apply(&mut self, e: Event, clock: f64, picked: &Picked, trail: bool) -> Changed {
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
                self.objects.retain(|(topic, _), _| !gone(topic));
                self.trails.retain(|(topic, _), _| !gone(topic));
                self.errors.retain(|topic, _| !gone(topic));
                self.annotations.retain(|topic, _| !gone(topic));
                self.texts.retain(|topic, _| !gone(topic));
                self.stats.retain(|topic, _| !gone(topic));
                self.descriptions.retain(|topic, _| !gone(topic));
                self.images.retain(|topic, _| !gone(topic));
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
                        // Keyed by the message's own label, before naming: an unlabelled array
                        // keeps one key whatever its length, so each message replaces the last.
                        let label = p.items.first().and_then(|i| i.label.clone()).unwrap_or_default();
                        // A lone unnamed pose (PoseStamped, PoseInFrame) is drawn named by its topic.
                        if let [item] = &mut p.items[..] {
                            item.label.get_or_insert_with(|| topic.clone());
                        }
                        let key = (topic, label);
                        if let ([item], true) = (&p.items[..], trail) {
                            let t = self.trails.entry(key.clone()).or_insert_with(|| Trail { frame_id: p.frame_id.clone(), points: VecDeque::new() });
                            if t.frame_id != p.frame_id {
                                *t = Trail { frame_id: p.frame_id.clone(), points: VecDeque::new() };
                            }
                            t.points.push_back((clock, item.pose.translation));
                        }
                        self.poses.insert(key, (clock, p));
                        true
                    }
                    // Deletions first: an update can clear its topic and send the new scene.
                    Decoded::Objects(u) => {
                        for d in u.deletions {
                            match d {
                                Deletion::All => self.objects.retain(|(t, _), _| *t != topic),
                                Deletion::Id(id) => _ = self.objects.remove(&(topic.clone(), id)),
                            }
                        }
                        for e in u.entities {
                            let until = e.lifetime.map(|l| clock + l);
                            self.objects.insert((topic.clone(), e.id.clone()), (until, e));
                        }
                        true
                    }
                    // These arrive at the robot's control rate (often 100 Hz or more).
                    Decoded::Joints(js) => {
                        // Only a joint name not seen before changes the display (it may be one
                        // the robot lacks); new positions just re-pose the robot.
                        let mut new_name = false;
                        if picked.joints.as_deref() == Some(topic.as_str()) {
                            match &mut self.joints {
                                Some((t, positions)) if *t == topic => {
                                    new_name = js.iter().any(|(n, _)| !positions.contains_key(n));
                                    positions.extend(js);
                                }
                                other => {
                                    new_name = true;
                                    *other = Some((topic, js.into_iter().collect()));
                                }
                            }
                            changed.robot = true;
                        }
                        cleared || new_name
                    }
                    Decoded::Image(i) => {
                        if picked.images.contains(&topic) {
                            self.images.insert(topic, Arc::new(i));
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

    /// Drops poses older than `timeout` seconds (0: never), trail points older than `trail`
    /// (0: no trails), and objects past their lifetime. Returns whether anything went, and when
    /// the next is due.
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
        let before = self.objects.len();
        self.objects.retain(|_, (until, _)| until.is_none_or(|t| now < t));
        dropped |= self.objects.len() != before;
        self.objects.values().filter_map(|(until, _)| *until).for_each(&mut soonest);
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

    /// An unlabelled pose message on /tracked with a pose at each x.
    fn poses_at(xs: &[f64]) -> Event {
        let item = |x| Tracked { pose: Pose { translation: DVec3::new(x, 0.0, 0.0), ..Pose::IDENTITY }, label: None, velocity: None };
        Event::Data { topic: "/tracked".into(), data: Decoded::Poses(Poses { frame_id: "world".into(), items: xs.iter().map(|&x| item(x)).collect() }) }
    }

    fn pose_at(x: f64) -> Event {
        poses_at(&[x])
    }

    // An array (PoseArray, PosesInFrame) whose length goes 2, 1, 0 replaces itself each time:
    // the two poses don't stay drawn next to the one.
    #[test]
    fn an_unlabelled_array_replaces_itself_whatever_its_length() {
        let picked = Picked::default();
        let mut r = Received::default();
        let drawn = |r: &Received| r.poses.values().map(|(_, p)| p.items.len()).collect::<Vec<_>>();
        r.apply(poses_at(&[0.0, 1.0]), 0.0, &picked, false);
        r.apply(poses_at(&[2.0]), 0.1, &picked, false);
        assert_eq!(drawn(&r), [1]);
        r.apply(poses_at(&[]), 0.2, &picked, false);
        assert_eq!(drawn(&r), [0]);
    }

    // A pose sent at 0, 1 and 2 s, with a 2 s timeout and a 1.5 s trail: the trail
    // at 2.4 s keeps the points from 1 and 2 s and wakes when the 1 s one is due, at 2.5; once the
    // topic goes quiet, the pose and its trail go at 2 + 2.
    #[test]
    fn poses_expire_and_trails_keep_only_their_window() {
        let picked = Picked::default();
        let mut r = Received::default();
        for t in [0.0, 1.0, 2.0] {
            r.apply(pose_at(t), t, &picked, true);
        }
        let key = ("/tracked".to_string(), String::new());
        assert_eq!(r.expire(2.4, 2.0, 1.5), (true, Some(2.5)));
        assert_eq!(r.trails[&key].points.iter().map(|(at, _)| *at).collect::<Vec<_>>(), [1.0, 2.0]);
        assert!(r.poses.contains_key(&key), "labelled by its topic, and still fresh");
        let (dropped, _) = r.expire(4.0, 2.0, 1.5);
        assert!(dropped && r.poses.is_empty() && r.trails.is_empty());
    }

    fn scene(deletions: Vec<Deletion>, entities: &[(&str, Option<f64>)]) -> Event {
        let entity = |&(id, lifetime): &(&str, Option<f64>)| Entity { id: id.into(), timestamp: 0, frame_id: "world".into(), frame_locked: true, lifetime, shapes: vec![] };
        Event::Data { topic: "/scene".into(), data: Decoded::Objects(crate::scene::SceneUpdate { deletions, entities: entities.iter().map(entity).collect() }) }
    }

    // An update deletes before it adds, so "delete all, then this scene" replaces the scene; an
    // id sent again replaces its entity; a lifetime counts from arrival on the UI clock.
    #[test]
    fn scene_updates_delete_replace_and_expire() {
        let picked = Picked::default();
        let mut r = Received::default();
        let ids = |r: &Received| r.objects.keys().map(|(_, id)| id.clone()).collect::<Vec<_>>();
        r.apply(scene(vec![], &[("table", None), ("cup", Some(1.0)), ("old", None)]), 0.0, &picked, false);
        r.apply(scene(vec![Deletion::Id("old".into())], &[("cup", Some(1.0))]), 0.5, &picked, false);
        assert_eq!(ids(&r), ["cup", "table"]);
        assert_eq!(r.expire(1.2, 0.0, 0.0), (false, Some(1.5)), "the resent cup lives to 0.5 + 1");
        assert_eq!(r.expire(1.5, 0.0, 0.0), (true, None));
        assert_eq!(ids(&r), ["table"]);
        r.apply(scene(vec![Deletion::All], &[("bin", None)]), 2.0, &picked, false);
        assert_eq!(ids(&r), ["bin"]);
    }
}
