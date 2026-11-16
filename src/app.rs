//! App state: the user's settings, what the source has delivered, and the display derived from
//! both (fixed frame, the cloud's placement, lines, labels). Handlers only edit `settings` and
//! the camera; `sync` runs at the start of every frame and does the rest: reconnects when the
//! URL changes, tells the source what to deliver, saves settings, rebuilds the display.

use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;

use glam::{Mat4, Vec3};
use rune::TextEdit;

use crate::camera::OrbitCamera;
use crate::decode::{Decoded, Kind};
use crate::scene::{PointCloud, PointColor, Poses, TfTree};
use crate::settings::Settings;
use crate::source::{Channel, Event, FoxgloveWs, Sink, Source, Status, Wanted};
use crate::viewport::{Lines, Shade, View};

pub struct State {
    pub settings: Settings,
    /// Settings as last saved and applied.
    applied: Settings,
    source: Box<dyn Source>,
    source_url: String,
    sink: Sink,
    sent: Wanted,

    pub status: Status,
    pub channels: Vec<Channel>,
    /// The drawn cloud and its topic.
    pub cloud: Option<(String, Arc<PointCloud>)>,
    pub tf: TfTree,
    /// The newest poses per (topic, label): one odometry topic can carry several bodies.
    pub poses: BTreeMap<(String, String), Poses>,
    /// The last decode error per topic, cleared by its next good message.
    pub errors: BTreeMap<String, String>,
    /// Bumped by every delivery; the display is rebuilt when it moves.
    revision: u64,
    shown: u64,
    pub display: Display,

    pub camera: OrbitCamera,
    /// Whether the camera has been framed on data since the last connect.
    framed: bool,
    /// How the first data frames the camera (headless scenes choose a view before data lands).
    pub frame_with: fn(&State) -> OrbitCamera,
    pub settings_open: bool,
    pub url_edit: TextEdit,
    /// Pointer travel already applied by the drag in progress.
    pub drag_seen: (f32, f32),
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
        }
    }
}

const VELOCITY_COLOR: [f32; 4] = [0.98, 0.78, 0.30, 1.0];
const TREE_COLOR: [f32; 4] = [0.75, 0.78, 0.85, 0.45];
/// Velocity arrows show where a body will be in this many seconds.
const VELOCITY_LOOKAHEAD_S: f32 = 1.0;

impl State {
    pub fn new(settings: Settings, sink: Sink) -> State {
        let sent = Wanted::default();
        let source = Box::new(FoxgloveWs::spawn(settings.url.clone(), sent.clone(), Arc::clone(&sink)));
        State {
            source_url: settings.url.clone(),
            url_edit: TextEdit::new(settings.url.clone()),
            applied: settings.clone(),
            settings,
            source,
            sink,
            sent,
            status: Status::Connecting,
            channels: Vec::new(),
            cloud: None,
            tf: TfTree::default(),
            poses: BTreeMap::new(),
            errors: BTreeMap::new(),
            revision: 0,
            shown: u64::MAX,
            display: Display::default(),
            camera: OrbitCamera::default(),
            framed: false,
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

    /// Applies what the source sent since the last frame.
    pub fn apply(&mut self, events: Vec<Event>) {
        for e in events {
            match e {
                Event::Status(s) => self.status = s,
                Event::Channels(c) => self.channels = c,
                Event::Data { topic, data } => {
                    self.errors.remove(&topic);
                    match data {
                        Decoded::Cloud(c) => {
                            if self.cloud_topic().as_deref() == Some(topic.as_str()) {
                                self.cloud = Some((topic, Arc::new(c)));
                            }
                        }
                        Decoded::Transforms(ts) => ts.into_iter().for_each(|t| self.tf.insert(t)),
                        Decoded::Poses(p) => {
                            let label = p.items.first().and_then(|i| i.label.clone()).unwrap_or_default();
                            self.poses.insert((topic, label), p);
                        }
                    }
                }
                Event::DecodeFailed { topic, error } => {
                    self.errors.insert(topic, error);
                }
            }
            self.revision += 1;
        }
    }

    /// The cloud topic to draw: the chosen one, else the first advertised.
    pub fn cloud_topic(&self) -> Option<String> {
        self.settings.cloud_topic.clone().or_else(|| self.topics_of(Kind::PointCloud).next().map(str::to_string))
    }

    pub fn topics_of(&self, kind: Kind) -> impl Iterator<Item = &str> {
        self.channels.iter().filter(move |c| c.kind == Some(kind)).map(|c| c.topic.as_str())
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
        Wanted { topics, color: self.settings.color.clone() }
    }

    /// Runs at the start of every frame (`AppSpec::poll`); true when it changed anything.
    pub fn sync(&mut self) -> bool {
        let mut changed = false;
        if self.settings.url != self.source_url {
            // A new server: drop the old connection and everything it delivered.
            self.source_url = self.settings.url.clone();
            self.source = Box::new(FoxgloveWs::spawn(self.source_url.clone(), Wanted::default(), Arc::clone(&self.sink)));
            self.sent = Wanted::default();
            self.status = Status::Connecting;
            self.channels.clear();
            self.cloud = None;
            self.tf = TfTree::default();
            self.poses.clear();
            self.errors.clear();
            self.framed = false;
            self.revision += 1;
            changed = true;
        }
        // A switched cloud topic: stop drawing the old one rather than mislabel it.
        if self.cloud.as_ref().is_some_and(|(t, _)| Some(t) != self.cloud_topic().as_ref()) {
            self.cloud = None;
            self.revision += 1;
        }
        self.poses.retain(|(topic, _), _| !self.settings.hidden_topics.contains(topic));
        let wanted = self.wanted();
        if wanted != self.sent {
            self.source.want(wanted.clone());
            self.sent = wanted;
        }
        if self.settings != self.applied {
            self.settings.save();
            self.applied = self.settings.clone();
            self.shown = u64::MAX;
        }
        if self.shown != self.revision {
            self.shown = self.revision;
            self.display = self.derive();
            changed = true;
            if !self.framed && self.cloud.is_some() && self.display.bounds.is_some() {
                self.camera = (self.frame_with)(self);
                self.framed = true;
            }
        }
        changed
    }

    fn fixed_frame(&self) -> Option<String> {
        if let Some(f) = &self.settings.fixed_frame {
            return Some(f.clone());
        }
        let data_frame = self.cloud.as_ref().map(|(_, c)| c.frame_id.clone())
            .or_else(|| self.poses.values().next().map(|p| p.frame_id.clone()))
            .or_else(|| self.tf.frames().into_iter().next())?;
        Some(self.tf.root_of(&data_frame))
    }

    fn derive(&self) -> Display {
        let s = &self.settings;
        let mut d = Display { fixed: self.fixed_frame(), ..Display::default() };
        let Some(fixed) = d.fixed.clone() else { return d };
        let mut problems = Vec::new();

        // The cloud: placement, a sample of its points in the display frame for bounds and the
        // height ramp (16 Ki points: percentiles to well under 1%).
        let mut sample: Vec<Vec3> = Vec::new();
        if let Some((topic, c)) = &self.cloud {
            match self.tf.lookup(&fixed, &c.frame_id) {
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
        let mut anchors = sample.len();
        if s.show_frames {
            let mut origins = BTreeMap::new();
            for f in self.tf.frames() {
                if let Some(p) = self.tf.lookup(&fixed, &f) {
                    let m = p.matrix();
                    lines.axes(m, axis, 2.5);
                    let o = m.transform_point3(Vec3::ZERO);
                    sample.push(o);
                    if s.show_labels {
                        d.labels.push((f.clone(), o));
                    }
                    origins.insert(f, o);
                }
            }
            for (parent, child) in self.tf.edges() {
                if let (Some(&a), Some(&b)) = (origins.get(parent), origins.get(child)) {
                    lines.segment(a, b, 1.5, TREE_COLOR);
                }
            }
        }
        if s.show_poses {
            for ((topic, _), poses) in &self.poses {
                let Some(frame) = self.tf.lookup(&fixed, &poses.frame_id) else {
                    problems.push(format!("{topic}: no transform from {} to {fixed}", poses.frame_id));
                    continue;
                };
                for item in &poses.items {
                    let m = frame.then(&item.pose).matrix();
                    lines.axes(m, axis * 0.6, 3.0);
                    let o = m.transform_point3(Vec3::ZERO);
                    sample.push(o);
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
        d.lines = Arc::new(lines);
        anchors = sample.len() - anchors;
        d.bounds = bounds(&sample, axis, anchors);
        if let Some((_, r)) = d.bounds {
            d.grid_cell = 10f32.powf((r / 12.0).log10().round());
        }
        d.shade = match self.cloud.as_ref().map(|(_, c)| &c.color) {
            Some(PointColor::Rgb) => Shade::Rgb,
            Some(PointColor::Field { min, max, .. }) => Shade::Field { lo: *min, hi: *max },
            _ => {
                let mut z: Vec<f32> = sample.iter().map(|p| p.z).collect();
                z.sort_by(f32::total_cmp);
                let q = |p: f32| z.get(((z.len().max(1) - 1) as f32 * p).round() as usize).copied();
                match (q(0.02), q(0.98)) {
                    (Some(lo), Some(hi)) if hi > lo => Shade::Height { lo, hi },
                    (Some(lo), _) => Shade::Height { lo, hi: lo + 1.0 },
                    _ => Shade::Height { lo: 0.0, hi: 1.0 },
                }
            }
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
            cloud: match (&self.cloud, self.display.cloud_model) {
                (Some((_, c)), Some(m)) if s.show_points => Some((c, m)),
                _ => None,
            },
            shade: self.display.shade,
            point_size: s.point_size,
            decimate: s.decimate,
            grid: s.show_grid.then_some(self.display.grid_cell),
            radius: self.display.bounds.map_or(2.0, |(_, r)| r),
            lines: &self.display.lines,
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

/// Centre and radius of the 1st–99th percentile box of `points`; the last `anchors` (frame and
/// pose origins) always count, padded by an axis length, so a lone frame still frames.
fn bounds(points: &[Vec3], axis: f32, anchors: usize) -> Option<(Vec3, f32)> {
    if points.is_empty() {
        return None;
    }
    let (cloud, marks) = points.split_at(points.len() - anchors);
    let mut lo = Vec3::splat(f32::INFINITY);
    let mut hi = Vec3::splat(f32::NEG_INFINITY);
    if !cloud.is_empty() {
        for k in 0..3 {
            let mut v: Vec<f32> = cloud.iter().map(|p| p[k]).collect();
            v.sort_by(f32::total_cmp);
            lo[k] = v[((v.len() - 1) as f32 * 0.01).round() as usize];
            hi[k] = v[((v.len() - 1) as f32 * 0.99).round() as usize];
        }
    }
    for m in marks {
        lo = lo.min(*m - axis);
        hi = hi.max(*m + axis);
    }
    Some(((lo + hi) * 0.5, ((hi - lo).length() * 0.5).max(0.05)))
}
