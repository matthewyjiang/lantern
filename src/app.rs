//! App state: the user's settings, what the source has delivered, the robot model, and the
//! display derived from them (fixed frame, the cloud's placement, lines, labels, link poses).
//! Handlers only edit `settings` and the camera; `sync` runs at the start of every frame and
//! does the rest: reconnects when the URL changes, loads the URDF when its path changes, tells
//! the source what to deliver, saves settings, rebuilds the display.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;
use std::sync::Arc;

use glam::{Mat4, Vec3};
use rune::TextEdit;

use crate::camera::OrbitCamera;
use crate::decode::{Decoded, Kind};
use crate::robot::{self, Model};
use crate::scene::{percentiles, PointCloud, PointColor, Pose, Poses, TfTree};
use crate::settings::Settings;
use crate::source::{self, Channel, Event, Source, Status, Wanted};
use crate::ui::thousands;
use crate::viewport::{self, Lines, RobotView, Shade, View};

/// Where every source's events go, tagged with the connection that sent them (`main.rs` queues
/// them for the UI thread, which hands them to `State::apply`).
pub type Post = Arc<dyn Fn(u64, Event) + Send + Sync>;

pub struct State {
    pub settings: Settings,
    /// Settings as last saved and applied.
    applied: Settings,
    conn: Connection,
    post: Post,
    pub received: Received,
    /// Something the display depends on changed; `sync` rebuilds it.
    dirty: bool,
    pub display: Display,
    /// The URDF last loaded: its path, and the model or why it didn't load. `sync` loads again
    /// when the path setting changes, or when this is cleared (the Load button).
    pub robot: Option<(String, Result<Model, String>)>,
    /// Joint positions changed: `sync` re-poses the robot without rebuilding the display.
    robot_dirty: bool,

    pub camera: OrbitCamera,
    framed: Framed,
    /// How the first data frames the camera (headless scenes choose a view before data lands).
    pub frame_with: fn(&State) -> OrbitCamera,
    pub settings_open: bool,
    pub url_edit: TextEdit,
    pub urdf_edit: TextEdit,
    /// Pointer travel already applied by the drag in progress.
    pub drag_seen: (f32, f32),
}

/// What the camera was last framed on, since the last connect. A robot alone frames the view;
/// a cloud arriving later frames it again, unless the user has moved the camera since.
#[derive(Clone, Copy, PartialEq)]
enum Framed {
    Nothing,
    Robot(OrbitCamera),
    Cloud,
}

/// The open source, and what it was last asked for.
struct Connection {
    url: String,
    source: Box<dyn Source>,
    sent: Wanted,
    /// Tags this connection's events: a replaced source's thread can still send a few (a connect
    /// timing out, a cloud mid-decode), and `apply` drops them.
    generation: u64,
}

impl Connection {
    fn open(url: &str, generation: u64, post: &Post) -> Connection {
        let post = Arc::clone(post);
        let source = source::open(url, Arc::new(move |e| post(generation, e)));
        Connection { url: url.to_string(), source, sent: Wanted::default(), generation }
    }
}

/// Everything the current connection delivered; a new connection starts from `default()`.
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
    /// The newest poses per (topic, label): one odometry topic can carry several bodies. Kept
    /// until the topic is unadvertised, hidden, or the connection changes.
    pub poses: BTreeMap<(String, String), Poses>,
    /// The last error per topic (a decode failure, a cloud over the GPU cap), cleared by its
    /// next good message.
    pub errors: BTreeMap<String, String>,
    /// The server's latest warning or error.
    pub server_warning: Option<String>,
}

/// What the viewport and overlays draw, derived from data and settings in `sync`.
pub struct Display {
    /// The frame everything is drawn in, once there's data to name one.
    pub fixed: Option<String>,
    /// `fixed_T_cloud`, when the cloud's frame connects to the fixed frame.
    pub cloud_model: Option<Mat4>,
    pub shade: Shade,
    pub lines: Arc<Lines>,
    /// Names to draw at display-frame positions (TF frames, tracked objects).
    pub labels: Vec<(String, Vec3)>,
    /// Centre and radius of the data, metres.
    pub bounds: Option<(Vec3, f32)>,
    pub grid_cell: f32,
    /// Why something isn't drawn, for the status overlay.
    pub problem: Option<String>,
    /// `fixed_T_root` for the robot, and `fixed_T_link` for each of its links.
    pub robot_root: Pose,
    pub robot_links: Vec<Mat4>,
    /// Why the robot sits at the fixed frame's origin, when TF doesn't place its root.
    pub robot_note: Option<String>,
}

impl Default for Display {
    fn default() -> Self {
        Display {
            fixed: None,
            cloud_model: None,
            shade: Shade::Height { lo: 0.0, hi: 1.0 },
            lines: Arc::default(),
            labels: Vec::new(),
            bounds: None,
            grid_cell: 1.0,
            problem: None,
            robot_root: Pose::IDENTITY,
            robot_links: Vec::new(),
            robot_note: None,
        }
    }
}

const VELOCITY_COLOR: [f32; 4] = [0.98, 0.78, 0.30, 1.0];
const TREE_COLOR: [f32; 4] = [0.75, 0.78, 0.85, 0.45];
/// Velocity arrows show where a body will be in this many seconds.
const VELOCITY_LOOKAHEAD_S: f32 = 1.0;

impl State {
    pub fn new(settings: Settings, post: Post) -> State {
        State {
            conn: Connection::open(&settings.url, 0, &post),
            post,
            url_edit: TextEdit::new(settings.url.clone()),
            urdf_edit: TextEdit::new(settings.robot_urdf.clone()),
            applied: settings.clone(),
            settings,
            received: Received::default(),
            dirty: true,
            display: Display::default(),
            robot: None,
            robot_dirty: false,
            camera: OrbitCamera::default(),
            framed: Framed::Nothing,
            frame_with: State::home_view,
            settings_open: false,
            drag_seen: (0.0, 0.0),
        }
    }

    /// Changes settings for this run without saving them: headless scenes (`--scene`) use it, so a
    /// test run doesn't rewrite the user's config (a later change in the UI saves them along).
    pub fn set_unsaved(&mut self, change: fn(&mut Settings)) {
        change(&mut self.settings);
        self.applied = self.settings.clone();
    }

    /// Applies what the current source sent since the last frame.
    pub fn apply(&mut self, events: Vec<(u64, Event)>) {
        for (generation, e) in events {
            if generation != self.conn.generation {
                continue;
            }
            let cloud_topic = self.cloud_topic();
            let joint_topic = self.joint_topic();
            let r = &mut self.received;
            match e {
                Event::Status(s) => r.status = s,
                Event::Channels(c) => {
                    r.channels = c;
                    let advertised: HashSet<&str> = r.channels.iter().map(|c| c.topic.as_str()).collect();
                    r.poses.retain(|(topic, _), _| advertised.contains(topic.as_str()));
                    r.errors.retain(|topic, _| advertised.contains(topic.as_str()));
                }
                Event::Data { topic, data: Decoded::Cloud(c) } if c.points.len() > viewport::MAX_POINTS => {
                    let error = format!("{} points, over the {} the GPU can draw", thousands(c.points.len()), thousands(viewport::MAX_POINTS));
                    r.errors.insert(topic, error);
                }
                Event::Data { topic, data: Decoded::Joints(js) } => {
                    // Joint states arrive at the robot's control rate (kpd: 100 Hz): re-pose,
                    // don't rebuild the display, unless clearing an error changes what it says.
                    self.dirty |= r.errors.remove(&topic).is_some();
                    if joint_topic.as_deref() == Some(topic.as_str()) {
                        let (_, positions) = r.joints.get_or_insert_with(|| (topic, HashMap::new()));
                        positions.extend(js);
                        self.robot_dirty = true;
                    }
                    continue;
                }
                Event::Data { topic, data } => {
                    r.errors.remove(&topic);
                    match data {
                        Decoded::Cloud(c) => {
                            if cloud_topic.as_deref() == Some(topic.as_str()) {
                                r.cloud = Some((topic, Arc::new(c)));
                            }
                        }
                        Decoded::Transforms(ts) => ts.into_iter().for_each(|t| r.tf.insert(t)),
                        Decoded::Poses(p) => {
                            let label = p.items.first().and_then(|i| i.label.clone()).unwrap_or_default();
                            r.poses.insert((topic, label), p);
                        }
                        Decoded::Joints(_) => unreachable!("handled above"),
                    }
                }
                Event::DecodeFailed { topic, error } => {
                    r.errors.insert(topic, error);
                }
                Event::ServerWarning(w) => r.server_warning = Some(w),
            }
            self.dirty = true;
        }
    }

    /// The cloud topic to draw: the chosen one, else the first advertised.
    pub fn cloud_topic(&self) -> Option<String> {
        self.settings.cloud_topic.clone().or_else(|| self.topics_of(Kind::PointCloud).next().map(str::to_string))
    }

    /// The joint-state topic that poses the robot: the chosen one, else the first advertised.
    pub fn joint_topic(&self) -> Option<String> {
        self.settings.joint_topic.clone().or_else(|| self.topics_of(Kind::JointStates).next().map(str::to_string))
    }

    /// The loaded robot, if its URDF loaded.
    pub fn robot_model(&self) -> Option<&Model> {
        self.robot.as_ref().and_then(|(_, m)| m.as_ref().ok())
    }

    pub fn topics_of(&self, kind: Kind) -> impl Iterator<Item = &str> {
        self.received.channels.iter().filter(move |c| c.kind == Some(kind)).map(|c| c.topic.as_str())
    }

    fn wanted(&self) -> Wanted {
        let mut topics: HashSet<String> = self.topics_of(Kind::Transforms).map(str::to_string).collect();
        // Hidden points aren't fetched (a 1.4 M-point cloud at 10 Hz is ~160 MB/s); the last one
        // stays, so turning them back on shows it until the next arrives.
        if self.settings.show_points {
            topics.extend(self.cloud_topic());
        }
        if self.settings.show_poses {
            topics.extend(self.topics_of(Kind::Poses).filter(|t| !self.settings.hidden_topics.contains(*t)).map(str::to_string));
        }
        if self.settings.show_robot && self.robot_model().is_some() {
            topics.extend(self.joint_topic());
        }
        Wanted { topics, color: self.settings.color.clone() }
    }

    /// Runs at the start of every frame (`AppSpec::poll`); true when it changed anything.
    pub fn sync(&mut self) -> bool {
        let mut changed = false;
        if self.settings.url != self.conn.url {
            // A new server: drop the old connection and everything it delivered.
            self.conn = Connection::open(&self.settings.url, self.conn.generation + 1, &self.post);
            self.received = Received::default();
            self.framed = Framed::Nothing;
            self.dirty = true;
        }
        let urdf = self.settings.robot_urdf.trim();
        let want = (!urdf.is_empty()).then(|| urdf.to_string());
        if self.robot.as_ref().map(|(p, _)| p) != want.as_ref() {
            self.robot = want.map(|path| {
                let model = robot::load(Path::new(&path), viewport::MAX_ROBOT_VERTICES);
                match &model {
                    Ok(m) => eprintln!(
                        "lantern: loaded {} from {path} in {:.1} ms: {} links, {} movable joints, {} parts, {} vertices, {} problems",
                        m.robot.name, m.load_ms, m.robot.links.len(), m.robot.movable().count(), m.mesh.parts.len(), m.mesh.vertices.len(), m.problems.len(),
                    ),
                    Err(e) => eprintln!("lantern: can't load {path}: {e}"),
                }
                (path, model)
            });
            self.dirty = true;
        }
        // A switched joint topic: the old topic's positions don't apply.
        if self.received.joints.as_ref().is_some_and(|(t, _)| Some(t) != self.joint_topic().as_ref()) {
            self.received.joints = None;
            self.robot_dirty = true;
        }
        // A switched cloud topic: stop drawing the old one rather than mislabel it.
        if self.received.cloud.as_ref().is_some_and(|(t, _)| Some(t) != self.cloud_topic().as_ref()) {
            self.received.cloud = None;
            self.dirty = true;
        }
        let hidden = &self.settings.hidden_topics;
        self.received.poses.retain(|(topic, _), _| !hidden.contains(topic));
        let wanted = self.wanted();
        if wanted != self.conn.sent {
            self.conn.source.want(wanted.clone());
            self.conn.sent = wanted;
        }
        if self.settings != self.applied {
            self.settings.save();
            self.applied = self.settings.clone();
            self.dirty = true;
        }
        if self.dirty {
            self.dirty = false;
            self.robot_dirty = false;
            self.display = self.derive();
            changed = true;
            self.frame();
        } else if self.robot_dirty {
            self.robot_dirty = false;
            self.display.robot_links = self.robot_links(self.display.robot_root);
            changed = true;
        }
        changed
    }

    /// Frames the camera on the data the first time there is some (see `Framed`).
    fn frame(&mut self) {
        if self.display.bounds.is_none() {
            return;
        }
        let cloud = self.received.cloud.is_some();
        let robot = self.settings.show_robot && self.robot_model().is_some();
        match self.framed {
            Framed::Nothing | Framed::Robot(_) if cloud => {
                if self.framed == Framed::Nothing || self.framed == Framed::Robot(self.camera) {
                    self.camera = (self.frame_with)(self);
                }
                self.framed = Framed::Cloud;
            }
            Framed::Nothing if robot => {
                self.camera = (self.frame_with)(self);
                self.framed = Framed::Robot(self.camera);
            }
            _ => {}
        }
    }

    /// `fixed_T_link` for every robot link, at the received joint positions.
    fn robot_links(&self, root: Pose) -> Vec<Mat4> {
        let Some(m) = self.robot_model() else { return Vec::new() };
        let none = HashMap::new();
        let joints = self.received.joints.as_ref().map_or(&none, |(_, j)| j);
        m.robot.link_poses(joints).iter().map(|p| root.then(p).matrix()).collect()
    }

    fn fixed_frame(&self) -> Option<String> {
        if let Some(f) = &self.settings.fixed_frame {
            return Some(f.clone());
        }
        let r = &self.received;
        let data_frame = r.cloud.as_ref().map(|(_, c)| c.frame_id.clone())
            .or_else(|| r.poses.values().next().map(|p| p.frame_id.clone()))
            .or_else(|| self.robot_model().filter(|_| self.settings.show_robot).map(|m| m.robot.links[0].clone()))
            .or_else(|| r.tf.frames().into_iter().next())?;
        Some(r.tf.root_of(&data_frame))
    }

    fn derive(&self) -> Display {
        let (s, r) = (&self.settings, &self.received);
        let mut d = Display { fixed: self.fixed_frame(), ..Display::default() };
        let Some(fixed) = d.fixed.clone() else { return d };
        // The cloud topic's error first: a cloud refused or undecodable is why the view is empty.
        let mut problems: Vec<String> = self.cloud_topic().iter()
            .filter(|_| s.show_points)
            .filter_map(|t| r.errors.get(t).map(|e| format!("{t}: {e}")))
            .collect();

        // The cloud: placement, a sample of its points in the display frame for bounds and the
        // height ramp (16 Ki points: percentiles to well under 1%).
        let mut sample: Vec<Vec3> = Vec::new();
        if let Some((topic, c)) = &r.cloud {
            match r.tf.lookup(&fixed, &c.frame_id) {
                Some(p) => {
                    let m = p.then(&c.origin).matrix();
                    d.cloud_model = Some(m);
                    let step = c.points.len().div_ceil(1 << 14).max(1);
                    sample = c.points.iter().step_by(step).map(|p| m.transform_point3(Vec3::new(p[0], p[1], p[2]))).collect();
                }
                None => problems.push(format!("{topic}: no transform from {} to {fixed}", c.frame_id)),
            }
        }

        let mut lines = Lines::default();
        let axis = s.axis_length;
        // Frame and pose origins, which the bounds always include.
        let mut marks: Vec<Vec3> = Vec::new();
        if s.show_frames {
            let mut origins = BTreeMap::new();
            for f in r.tf.frames() {
                if let Some(p) = r.tf.lookup(&fixed, &f) {
                    let m = p.matrix();
                    lines.axes(m, axis, 2.5);
                    let o = m.transform_point3(Vec3::ZERO);
                    marks.push(o);
                    if s.show_labels {
                        d.labels.push((f.clone(), o));
                    }
                    origins.insert(f, o);
                }
            }
            for (parent, child) in r.tf.edges() {
                if let (Some(&a), Some(&b)) = (origins.get(parent), origins.get(child)) {
                    lines.segment(a, b, 1.5, TREE_COLOR);
                }
            }
        }
        if s.show_poses {
            for ((topic, _), poses) in &r.poses {
                let Some(frame) = r.tf.lookup(&fixed, &poses.frame_id) else {
                    problems.push(format!("{topic}: no transform from {} to {fixed}", poses.frame_id));
                    continue;
                };
                for item in &poses.items {
                    let m = frame.then(&item.pose).matrix();
                    lines.axes(m, axis * 0.6, 3.0);
                    let o = m.transform_point3(Vec3::ZERO);
                    marks.push(o);
                    if let (true, Some(l)) = (s.show_labels, &item.label) {
                        d.labels.push((l.clone(), o));
                    }
                    if let Some(v) = item.velocity {
                        let v = frame.rotation.as_quat() * v.as_vec3();
                        lines.arrow(o, o + v * VELOCITY_LOOKAHEAD_S, 2.5, VELOCITY_COLOR);
                    }
                }
            }
        }
        if let Some(m) = self.robot_model().filter(|_| s.show_robot) {
            let root = &m.robot.links[0];
            d.robot_root = r.tf.lookup(&fixed, root).unwrap_or_else(|| {
                d.robot_note = Some(format!("No transform from {root} to {fixed}: drawn at {fixed}'s origin"));
                Pose::IDENTITY
            });
            d.robot_links = self.robot_links(d.robot_root);
            marks.extend(d.robot_links.iter().map(|l| l.transform_point3(Vec3::ZERO)));
        }
        d.lines = Arc::new(lines);
        d.bounds = bounds(&sample, &marks, axis);
        // The grid scales with the cloud; frames and poses alone (an axis length across) keep
        // the default 1 m cells rather than shrink them to centimetres.
        if let (Some((_, radius)), false) = (d.bounds, sample.is_empty()) {
            d.grid_cell = 10f32.powf((radius / 12.0).log10().round());
        }
        d.shade = match r.cloud.as_ref().map(|(_, c)| &c.color) {
            Some(PointColor::Rgb) => Shade::Rgb,
            Some(PointColor::Field { min, max, .. }) => Shade::Field { lo: *min, hi: *max },
            _ => match percentiles(sample.iter().map(|p| p.z).collect(), 0.02, 0.98) {
                Some((lo, hi)) if hi > lo => Shade::Height { lo, hi },
                Some((lo, _)) => Shade::Height { lo, hi: lo + 1.0 },
                None => Shade::Height { lo: 0.0, hi: 1.0 },
            },
        };
        d.problem = problems.into_iter().next();
        d
    }

    /// Three-quarter view framing the data.
    pub fn home_view(&self) -> OrbitCamera {
        let (centre, radius) = self.display.bounds.unwrap_or((Vec3::ZERO, 2.0));
        let fov_y = OrbitCamera::default().fov_y;
        OrbitCamera {
            target: centre,
            yaw: (-135.0f32).to_radians(),
            pitch: 35.0f32.to_radians(),
            distance: radius / (fov_y * 0.5).sin() * 1.05,
            fov_y,
        }
    }

    pub fn top_view(&self) -> OrbitCamera {
        OrbitCamera { yaw: (-90.0f32).to_radians(), pitch: 89.0f32.to_radians(), ..self.home_view() }
    }

    pub fn view(&self) -> View<'_> {
        let s = &self.settings;
        View {
            camera: self.camera,
            cloud: match (&self.received.cloud, self.display.cloud_model) {
                (Some((_, c)), Some(m)) if s.show_points => Some((c, m)),
                _ => None,
            },
            shade: self.display.shade,
            point_size: s.point_size,
            decimate: s.decimate,
            grid: s.show_grid.then_some(self.display.grid_cell),
            radius: self.display.bounds.map_or(2.0, |(_, r)| r),
            lines: &self.display.lines,
            robot: match self.robot_model() {
                Some(m) if s.show_robot && !self.display.robot_links.is_empty() => {
                    Some(RobotView { mesh: &m.mesh, links: &self.display.robot_links, opacity: s.robot_opacity })
                }
                _ => None,
            },
        }
    }

    /// Points drawn per frame, for the badge.
    pub fn drawn_points(&self) -> usize {
        match self.view().cloud {
            Some((c, _)) => c.points.len().div_ceil(self.settings.decimate as usize),
            None => 0,
        }
    }
}

/// Centre and radius of the 1st–99th percentile box of the cloud `sample`, grown to include
/// every mark (frame and pose origins) padded by an axis length, so a lone frame still frames.
fn bounds(sample: &[Vec3], marks: &[Vec3], axis: f32) -> Option<(Vec3, f32)> {
    if sample.is_empty() && marks.is_empty() {
        return None;
    }
    let mut lo = Vec3::splat(f32::INFINITY);
    let mut hi = Vec3::splat(f32::NEG_INFINITY);
    for k in 0..3 {
        if let Some((l, h)) = percentiles(sample.iter().map(|p| p[k]).collect(), 0.01, 0.99) {
            (lo[k], hi[k]) = (l, h);
        }
    }
    for m in marks {
        lo = lo.min(*m - axis);
        hi = hi.max(*m + axis);
    }
    Some(((lo + hi) * 0.5, ((hi - lo).length() * 0.5).max(0.05)))
}
