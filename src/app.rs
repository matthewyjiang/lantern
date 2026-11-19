//! App state: the user's settings, what each connection has delivered, the robot model, and the
//! display derived from them (fixed frame, the cloud's placement, lines, labels, link poses).
//! Handlers only edit `settings` and the camera; `sync` runs at the start of every frame and
//! does the rest: opens and closes connections to match the list, loads the URDF when its path
//! or the server's description changes, expires old poses, tells each source what to deliver,
//! saves settings, rebuilds the display.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;
use std::sync::{Arc, LazyLock};

use glam::{DVec3, Mat4, Vec3};
use rune::{Pixels, TextEdit, Tx};

use crate::annotate;
use crate::camera::OrbitCamera;
use crate::connection::{Connection, Picked, Post, Received};
use crate::decode::Kind;
use crate::layout::{Dir, Layout, PaneId, Panel, PanelKind};
use crate::robot::{self, Model};
use crate::scene::{percentiles, Annotation, Diagnostic, Image, PointCloud, PointColor, Pose, TfTree};
use crate::settings::{DataSettings, ImageSettings, Settings, View3dSettings};
use crate::source::{Event, Wanted};
use crate::viewport::{self, Lines, RobotView, Shade, View};

/// The submenus of a panel's menu, each a list of panel kinds: split in that direction, or show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Submenu {
    Split(Dir),
    Show,
}

/// What's open over the panel at `path`; one thing at a time.
pub enum Popup {
    /// Its menu, and the submenu open in it.
    Menu { path: Vec<usize>, sub: Option<Submenu> },
    /// Its settings, and the picker in them whose options are listed, by its key.
    Settings { path: Vec<usize>, picker: Option<String> },
}

impl Popup {
    pub fn path(&self) -> &[usize] {
        match self {
            Popup::Menu { path, .. } | Popup::Settings { path, .. } => path,
        }
    }
}

/// A panel dragged by its header, swapping places with the panels it passes over.
pub struct PanelDrag {
    pub id: PaneId,
    /// The layout when the drag started, which a cancelled drag puts back.
    pub before: Layout,
    /// Where the card was grabbed, from its top left corner, and its size then, points.
    pub grab: [f32; 2],
    pub size: [f32; 2],
    /// The pointer, in the window's points.
    pub pointer: [f32; 2],
}

/// The tabs of the 3D view's settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab3d {
    Scene,
    Points,
    Poses,
    Robot,
    Cards,
}

impl Tab3d {
    pub const ALL: [Tab3d; 5] = [Tab3d::Scene, Tab3d::Points, Tab3d::Poses, Tab3d::Robot, Tab3d::Cards];

    pub fn label(self) -> &'static str {
        match self {
            Tab3d::Scene => "Scene",
            Tab3d::Points => "Points",
            Tab3d::Poses => "Poses",
            Tab3d::Robot => "Robot",
            Tab3d::Cards => "Cards",
        }
    }
}

pub struct State {
    pub settings: Settings,
    /// Settings as last saved and applied.
    applied: Settings,
    /// Command-line values, which last for this run, and the file's values for the same
    /// fields (see `to_save`).
    overrides: Overrides,
    from_file: Settings,
    /// One per `settings.connections` entry, in that order.
    conns: Vec<Connection>,
    next_conn: u64,
    post: Post,
    /// Something the display depends on changed; `sync` rebuilds it.
    dirty: bool,
    pub display: Display,
    /// The URDF file last loaded: its path, and the model or why it didn't load. `sync` loads
    /// again when the path setting changes, or when this is cleared (`load_robot`).
    robot_file: Option<(String, Result<Model, String>)>,
    /// The URDF a `robot_description` topic sent: topic, text, and the model or why not.
    robot_description: Option<(String, String, Result<Model, String>)>,
    /// How the file's robot differs from the described one, when both loaded and they differ.
    robot_mismatch: Option<Mismatch>,
    /// Joint positions changed: `sync` re-poses the robot without rebuilding the display.
    robot_dirty: bool,
    /// Each shown image with its annotations drawn in, by its topic and the annotation topics
    /// its views hide, and whether a new image or annotation arrived since they were drawn.
    images: BTreeMap<(String, BTreeSet<String>), Pixels>,
    image_dirty: bool,

    pub camera: OrbitCamera,
    framed: Framed,
    /// How the first data frames the camera (headless scenes choose a view before data lands).
    pub frame_with: fn(&State) -> OrbitCamera,
    pub settings_open: bool,
    /// The menu or settings open over a panel.
    pub popup: Option<Popup>,
    /// The 3D view's settings tab on show.
    pub tab_3d: Tab3d,
    /// The Settings field for a server to add.
    pub url_edit: TextEdit,
    pub urdf_edit: TextEdit,
    /// Pointer travel already applied by the drag in progress.
    pub drag_seen: (f32, f32),
    /// The panel being dragged by its header, and where it would land.
    pub panel_drag: Option<PanelDrag>,
    /// The Settings field for a layout file to load or save.
    pub layout_file_edit: TextEdit,
    /// How the last layout file load or save went.
    pub layout_file_note: Option<Result<String, String>>,
}

/// The 3D view's settings when the layout has no 3D panel: nothing on, so nothing 3D is
/// subscribed to or loaded.
static NO_3D: LazyLock<View3dSettings> = LazyLock::new(View3dSettings::off);

/// The URDF file and the described robot differ: how, and which is drawn.
struct Mismatch {
    message: String,
    /// The described robot wins, unless none of its meshes load here (a robot's `package://`
    /// paths usually aren't on this machine) and the file's do.
    draw_described: bool,
}

/// What the command line set. Saving writes the file's value back for any value still at the
/// command line's, so `--urdf` for one run doesn't replace the URDF the 3D panel remembers; a
/// value changed in the UI saves as changed.
#[derive(Default)]
pub struct Overrides {
    pub connections: Option<Vec<String>>,
    /// The 3D panel's URDF.
    pub robot_urdf: Option<String>,
    /// The layout, as the command line made it (`State::new` puts `robot_urdf` in it).
    pub layout: Option<Layout>,
    /// The URDF `robot_urdf` replaced (`State::new` sets it).
    replaced_urdf: String,
}

impl Overrides {
    /// What to write to the settings file: `settings`, with command-line values the user hasn't
    /// changed since swapped back for `file`'s. While the tiling is still the command line's,
    /// the file's layout stays, with the panels the user changed; once the tiling is edited, the
    /// layout saves as edited. Either way the 3D panel keeps the URDF `--urdf` replaced, unless
    /// another was loaded.
    fn to_save(&self, settings: &Settings, file: &Settings) -> Settings {
        let mut s = settings.clone();
        if self.connections.as_ref() == Some(&s.connections) {
            s.connections = file.connections.clone();
        }
        if let Some(cli) = &self.layout
            && s.layout.same_tiling(cli)
        {
            let mut layout = file.layout.clone();
            layout.adopt_changes(cli, &s.layout);
            s.layout = layout;
        }
        if let Some(urdf) = &self.robot_urdf
            && let Some(v) = s.layout.view3d_mut()
            && v.robot.urdf == *urdf
        {
            v.robot.urdf = self.replaced_urdf.clone();
        }
        s
    }
}

/// Whether `a` and `b` hold the same panels, in any order.
fn same_panels(a: &[&Panel], b: &[&Panel]) -> bool {
    let count = |of: &[&Panel], p: &Panel| of.iter().filter(|q| **q == p).count();
    a.len() == b.len() && a.iter().all(|p| count(a, p) == count(b, p))
}

/// What the camera was last framed on. A robot alone frames the view; a cloud arriving later
/// frames it again, unless the user has moved the camera since.
#[derive(Clone, Copy, PartialEq)]
enum Framed {
    Nothing,
    Robot(OrbitCamera),
    Cloud,
}

/// What the viewport and overlays draw, derived from data and settings in `sync`.
pub struct Display {
    /// The frame everything is drawn in, once there's data to name one.
    pub fixed: Option<String>,
    /// Every connection's transforms, as one tree.
    pub tf: TfTree,
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
    /// The shown robot, posed.
    pub robot: Option<PosedRobot>,
    /// Why the robot sits at the fixed frame's origin, when TF doesn't place its root.
    pub robot_note: Option<String>,
}

pub struct PosedRobot {
    /// `fixed_T_root`.
    pub root: Pose,
    /// `fixed_T_link`, for every link of the model.
    pub links: Vec<Mat4>,
}

impl Default for Display {
    fn default() -> Self {
        Display {
            fixed: None,
            tf: TfTree::default(),
            cloud_model: None,
            shade: Shade::Height { lo: 0.0, hi: 1.0 },
            lines: Arc::default(),
            labels: Vec::new(),
            bounds: None,
            grid_cell: 1.0,
            problem: None,
            robot: None,
            robot_note: None,
        }
    }
}

const VELOCITY_COLOR: [f32; 4] = [0.98, 0.78, 0.30, 1.0];
const TREE_COLOR: [f32; 4] = [0.75, 0.78, 0.85, 0.45];
const TRAIL_COLOR: [f32; 4] = [1.0, 0.55, 0.15, 0.9];
/// Velocity arrows show where a body will be in this many seconds.
const VELOCITY_LOOKAHEAD_S: f32 = 1.0;

impl State {
    pub fn new(mut settings: Settings, overrides: Overrides, post: Post) -> State {
        let from_file = settings.clone();
        if let Some(c) = &overrides.connections {
            settings.connections = c.clone();
        }
        if let Some(l) = &overrides.layout {
            settings.layout = l.clone();
        }
        let mut replaced_urdf = String::new();
        if let Some(u) = &overrides.robot_urdf {
            match settings.layout.view3d_mut() {
                Some(v) => replaced_urdf = std::mem::replace(&mut v.robot.urdf, u.clone()),
                None => eprintln!("lantern: --urdf: the layout has no 3D panel to draw the robot in"),
            }
        }
        let overrides = Overrides { layout: overrides.layout.as_ref().map(|_| settings.layout.clone()), replaced_urdf, ..overrides };
        State {
            conns: Vec::new(),
            next_conn: 0,
            post,
            url_edit: TextEdit::new(String::new()),
            urdf_edit: TextEdit::new(String::new()),
            applied: settings.clone(),
            settings,
            overrides,
            from_file,
            dirty: true,
            display: Display::default(),
            robot_file: None,
            robot_description: None,
            robot_mismatch: None,
            robot_dirty: false,
            images: BTreeMap::new(),
            image_dirty: false,
            camera: OrbitCamera::default(),
            framed: Framed::Nothing,
            frame_with: State::home_view,
            settings_open: false,
            popup: None,
            tab_3d: Tab3d::Scene,
            drag_seen: (0.0, 0.0),
            panel_drag: None,
            layout_file_edit: TextEdit::new(String::new()),
            layout_file_note: None,
        }
    }

    /// Changes the 3D view's settings for this run without saving them: headless scenes
    /// (`--scene`) use it, so a test run doesn't rewrite the user's config (a later change in
    /// the UI saves them along).
    pub fn set_unsaved(&mut self, change: fn(&mut View3dSettings)) {
        self.edit3d(change);
        self.applied = self.settings.clone();
    }

    /// The 3D panel's settings, or with no 3D panel, settings that draw nothing.
    pub fn v3(&self) -> &View3dSettings {
        self.settings.layout.view3d().unwrap_or(&NO_3D)
    }

    /// Edits the 3D panel's settings, if the layout has one.
    pub fn edit3d(&mut self, edit: impl FnOnce(&mut View3dSettings)) {
        if let Some(v) = self.settings.layout.view3d_mut() {
            edit(v);
        }
    }

    /// Opens the settings of the panel at `path` over it, its URDF field (for a 3D panel)
    /// showing what it has, or closes them if they're open.
    pub fn toggle_panel_settings(&mut self, path: Vec<usize>) {
        if self.settings_at() == Some(&path) {
            self.popup = None;
            return;
        }
        if let Some(Panel::View3d(v)) = self.settings.layout.panel(&path) {
            self.urdf_edit = TextEdit::new(v.robot.urdf.clone());
        }
        self.popup = Some(Popup::Settings { path, picker: None });
    }

    /// Opens the menu of the panel at `path`, or closes it if it's open.
    pub fn toggle_panel_menu(&mut self, path: Vec<usize>) {
        self.popup = match self.menu_at() {
            Some((open, _)) if open == path => None,
            _ => Some(Popup::Menu { path, sub: None }),
        };
    }

    /// Opens `sub` in the open menu, or closes it if it's open.
    pub fn toggle_submenu(&mut self, sub: Submenu) {
        if let Some(Popup::Menu { sub: open, .. }) = &mut self.popup {
            *open = if *open == Some(sub) { None } else { Some(sub) };
        }
    }

    /// Lists the options of the picker keyed `key` in the open settings, or closes them.
    pub fn toggle_picker(&mut self, key: &str) {
        if let Some(Popup::Settings { picker, .. }) = &mut self.popup {
            *picker = if picker.as_deref() == Some(key) { None } else { Some(key.to_string()) };
        }
    }

    /// Closes the open settings' picker, keeping the settings open.
    pub fn close_picker(&mut self) {
        if let Some(Popup::Settings { picker, .. }) = &mut self.popup {
            *picker = None;
        }
    }

    /// The path of the panel whose settings are open.
    pub fn settings_at(&self) -> Option<&[usize]> {
        match &self.popup {
            Some(Popup::Settings { path, .. }) => Some(path),
            _ => None,
        }
    }

    /// The path of the panel whose menu is open, and its open submenu.
    pub fn menu_at(&self) -> Option<(&[usize], Option<Submenu>)> {
        match &self.popup {
            Some(Popup::Menu { path, sub }) => Some((path, *sub)),
            _ => None,
        }
    }

    /// The key of the open settings' picker whose options are listed.
    pub fn open_picker(&self) -> Option<&str> {
        match &self.popup {
            Some(Popup::Settings { picker, .. }) => picker.as_deref(),
            _ => None,
        }
    }

    /// The 3D view's image card's settings, when it's drawn: the layout has a 3D panel and no
    /// Image panel, and the card is on.
    pub fn image_card(&self) -> Option<&ImageSettings> {
        self.card_for(PanelKind::Image).filter(|v| v.image_card).map(|v| &v.image)
    }

    /// The 3D view's text and stats cards' settings, when the layout has a 3D panel and no Data
    /// panel.
    pub fn data_cards(&self) -> Option<&DataSettings> {
        self.card_for(PanelKind::Data).map(|v| &v.data)
    }

    /// The 3D view's settings, when it stands in for a `kind` panel with its cards: the layout
    /// has a 3D panel and no `kind` panel.
    pub fn card_for(&self, kind: PanelKind) -> Option<&View3dSettings> {
        let l = &self.settings.layout;
        l.view3d().filter(|_| !l.has(kind))
    }

    /// Every image shown: each Image panel's, and the 3D view's card.
    fn image_views(&self) -> Vec<&ImageSettings> {
        let panels = self.settings.layout.panels().into_iter().filter_map(|p| match p {
            Panel::Image(i) => Some(i),
            _ => None,
        });
        panels.chain(self.image_card()).collect()
    }

    /// Every text and stats view: each Data panel's, and the 3D view's cards.
    fn data_views(&self) -> Vec<&DataSettings> {
        let panels = self.settings.layout.panels().into_iter().filter_map(|p| match p {
            Panel::Data(d) => Some(d),
            _ => None,
        });
        panels.chain(self.data_cards()).collect()
    }

    pub fn connections(&self) -> &[Connection] {
        &self.conns
    }

    fn received(&self) -> impl Iterator<Item = &Received> {
        self.conns.iter().map(|c| &c.received)
    }

    /// Adds the server in the Settings field, unless it's already in the list.
    pub fn add_connection(&mut self) {
        let url = self.url_edit.text.trim().to_string();
        if !url.is_empty() && !self.settings.connections.contains(&url) {
            self.settings.connections.push(url);
        }
        self.url_edit = TextEdit::new(String::new());
    }

    /// Applies what the sources sent since the last frame; `clock` is the UI clock (seconds).
    pub fn apply(&mut self, events: Vec<(u64, Event)>, clock: f64) {
        for (id, e) in events {
            // Per event: a connection's new channel list can change what Auto picks.
            let picked = Picked {
                cloud: self.cloud_topic(),
                joints: self.joint_topic(),
                images: self.image_topics(),
            };
            let trail = self.v3().poses.trail > 0.0;
            let Some(conn) = self.conns.iter_mut().find(|c| c.id == id) else { continue };
            let changed = conn.received.apply(e, clock, &picked, trail);
            self.dirty |= changed.display;
            self.robot_dirty |= changed.robot;
            self.image_dirty |= changed.image;
        }
    }

    /// Every connection's topics of `kind`, each once, in connection then server order.
    pub fn topics_of(&self, kind: Kind) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for c in self.received().flat_map(|r| &r.channels).filter(|c| c.kind == Some(kind)) {
            if !out.contains(&c.topic) {
                out.push(c.topic.clone());
            }
        }
        out
    }

    fn first_of(&self, kind: Kind) -> Option<String> {
        self.received().flat_map(|r| &r.channels).find(|c| c.kind == Some(kind)).map(|c| c.topic.clone())
    }

    /// The topic of `kind` a setting picks: the chosen one, else (Auto) the first advertised.
    pub fn resolve(&self, kind: Kind, chosen: &Option<String>) -> Option<String> {
        chosen.clone().or_else(|| self.first_of(kind))
    }

    /// The point cloud topic the 3D view draws.
    pub fn cloud_topic(&self) -> Option<String> {
        self.resolve(Kind::PointCloud, &self.v3().points.topic)
    }

    /// The joint-state topic that poses the robot.
    pub fn joint_topic(&self) -> Option<String> {
        self.resolve(Kind::JointStates, &self.v3().robot.joint_topic)
    }

    /// The `robot_description` topic the 3D view's robot comes from, with a 3D panel.
    pub fn description_topic(&self) -> Option<String> {
        self.settings.layout.view3d().and_then(|v| self.resolve(Kind::RobotDescription, &v.robot.description_topic))
    }

    /// The image topic a view shows.
    pub fn image_topic(&self, view: &ImageSettings) -> Option<String> {
        self.resolve(Kind::Image, &view.topic)
    }

    /// Every image topic some view shows.
    fn image_topics(&self) -> HashSet<String> {
        self.image_views().into_iter().filter_map(|v| self.image_topic(v)).collect()
    }

    /// The drawn cloud and its topic, from whichever connection has it.
    pub fn cloud(&self) -> Option<&(String, Arc<PointCloud>)> {
        self.received().find_map(|r| r.cloud.as_ref())
    }

    pub fn joints(&self) -> Option<&(String, HashMap<String, f64>)> {
        self.received().find_map(|r| r.joints.as_ref())
    }

    /// The image a view shows, before annotations, with its topic.
    pub fn image(&self, view: &ImageSettings) -> Option<(&String, &Arc<Image>)> {
        let topic = self.image_topic(view)?;
        self.received().find_map(|r| r.images.get_key_value(&topic))
    }

    /// The image a view shows with its annotations drawn in, once drawn.
    pub fn pixels(&self, view: &ImageSettings) -> Option<&Pixels> {
        self.images.get(&(self.image_topic(view)?, view.hidden.clone()))
    }

    /// The error a topic last reported, on any connection.
    pub fn error(&self, topic: &str) -> Option<&String> {
        self.received().find_map(|r| r.errors.get(topic))
    }

    /// The annotations drawn on an image: every annotation topic's newest, but those `hidden`.
    pub fn annotations<'a>(&'a self, hidden: &'a BTreeSet<String>) -> impl Iterator<Item = &'a Annotation> {
        self.received().flat_map(|r| &r.annotations).filter(move |(t, _)| !hidden.contains(*t)).flat_map(|(_, a)| a)
    }

    /// A view's text topics with their newest message.
    pub fn texts<'a>(&'a self, view: &'a DataSettings) -> impl Iterator<Item = (&'a String, &'a String)> {
        self.received().flat_map(|r| &r.texts).filter(move |(t, _)| view.text && !view.hidden.contains(*t))
    }

    /// A view's stats topics with their statuses.
    pub fn stats<'a>(&'a self, view: &'a DataSettings) -> impl Iterator<Item = (&'a String, &'a Vec<Diagnostic>)> {
        self.received().flat_map(|r| &r.stats).filter(move |(t, _)| view.stats && !view.hidden.contains(*t))
    }

    /// The URDF file's load: the model, or why it didn't load; `None` with no file set.
    pub fn robot_file(&self) -> Option<&Result<Model, String>> {
        self.robot_file.as_ref().map(|(_, m)| m)
    }

    /// The described robot's topic and load, once its topic has sent a URDF.
    pub fn robot_description(&self) -> Option<(&str, &Result<Model, String>)> {
        self.robot_description.as_ref().map(|(t, _, m)| (t.as_str(), m))
    }

    pub fn robot_mismatch(&self) -> Option<&str> {
        self.robot_mismatch.as_ref().map(|m| m.message.as_str())
    }

    /// The robot drawn: the file's, unless a server describes a different robot, which wins (a
    /// file that matches still draws, for its meshes and colours; see `Mismatch`); else whichever
    /// loaded.
    pub fn robot_model(&self) -> Option<&Model> {
        let file = self.robot_file().and_then(|m| m.as_ref().ok());
        let described = self.robot_description().and_then(|(_, m)| m.as_ref().ok());
        match (file, described) {
            (Some(_), Some(d)) if self.robot_mismatch.as_ref().is_some_and(|m| m.draw_described) => Some(d),
            (f, d) => f.or(d),
        }
    }

    /// The robot drawn, when it's also switched on.
    pub fn shown_robot(&self) -> Option<&Model> {
        self.robot_model().filter(|_| self.v3().robot.show)
    }

    /// Takes the URDF path from its text field and loads it, again if it's the same path (after
    /// editing the file).
    pub fn load_robot(&mut self) {
        let urdf = self.urdf_edit.text.trim().to_string();
        self.edit3d(|v| v.robot.urdf = urdf);
        self.robot_file = None;
    }

    /// Replaces the layout with the preset or layout file named in the Settings field (a preset
    /// keeps the current panels' settings; see `Layout::adopt`).
    pub fn load_layout(&mut self) {
        let value = self.layout_file_edit.text.trim().to_string();
        if value.is_empty() {
            return;
        }
        self.layout_file_note = Some(Layout::named_or_file(&value, &self.settings.layout).map(|layout| {
            self.settings.layout = layout;
            format!("Loaded {value}")
        }));
    }

    /// Writes the layout to the file named in the Settings field.
    pub fn save_layout(&mut self) {
        let path = self.layout_file_edit.text.trim().to_string();
        if path.is_empty() {
            return;
        }
        self.layout_file_note = Some(self.settings.layout.save(Path::new(&path)).map(|()| format!("Saved {path}")));
    }

    /// The topics to subscribe to: the union of what every panel shows.
    fn wanted(&self) -> Wanted {
        let v = self.v3();
        let mut topics: HashSet<String> = self.topics_of(Kind::Transforms).into_iter().collect();
        let all_but = |kind, hidden: &BTreeSet<String>| self.topics_of(kind).into_iter().filter(|t| !hidden.contains(t)).collect::<Vec<_>>();
        // Hidden points aren't fetched (a 1.4 M-point cloud at 10 Hz is ~160 MB/s); the last one
        // stays, so turning them back on shows it until the next arrives.
        if v.points.show {
            topics.extend(self.cloud_topic());
        }
        if v.poses.show {
            topics.extend(all_but(Kind::Poses, &v.poses.hidden));
        }
        if self.shown_robot().is_some() {
            topics.extend(self.joint_topic());
        }
        // With a 3D panel, always: the description says which robot the server drives, shown
        // or not.
        topics.extend(self.description_topic());
        for view in self.image_views() {
            topics.extend(self.image_topic(view));
            topics.extend(all_but(Kind::Annotations, &view.hidden));
        }
        for view in self.data_views() {
            if view.text {
                topics.extend(all_but(Kind::Text, &view.hidden));
            }
            if view.stats {
                topics.extend(all_but(Kind::Stats, &view.hidden));
            }
        }
        Wanted { topics, color: v.points.color.clone() }
    }

    /// Runs at the start of every frame (`AppSpec::poll`) with the UI clock; true when it changed
    /// anything.
    pub fn sync(&mut self, clock: f64, tx: &mut Tx) -> bool {
        let mut changed = false;
        self.sync_connections();
        self.sync_robot();
        let (joint_topic, cloud_topic, image_topics) = (self.joint_topic(), self.cloud_topic(), self.image_topics());
        let hidden = self.v3().poses.hidden.clone();
        for r in self.conns.iter_mut().map(|c| &mut c.received) {
            // A switched joint topic: the old topic's positions don't apply.
            if r.joints.as_ref().is_some_and(|(t, _)| Some(t) != joint_topic.as_ref()) {
                r.joints = None;
                self.robot_dirty = true;
            }
            // A switched cloud topic: stop drawing the old one rather than mislabel it.
            if r.cloud.as_ref().is_some_and(|(t, _)| Some(t) != cloud_topic.as_ref()) {
                r.cloud = None;
                self.dirty = true;
            }
            // An image no view shows any more.
            let before = r.images.len();
            r.images.retain(|t, _| image_topics.contains(t));
            self.image_dirty |= r.images.len() != before;
            r.poses.retain(|(topic, _), _| !hidden.contains(topic));
            r.trails.retain(|(topic, _), _| !hidden.contains(topic));
        }
        if let Some(due) = self.expire(clock) {
            tx.wake_at(due);
        }
        let wanted = self.wanted();
        self.conns.iter_mut().for_each(|c| c.want(&wanted));
        if self.settings != self.applied {
            self.overrides.to_save(&self.settings, &self.from_file).save();
            // Only the tiling changed (a divider mid-drag moves it every frame, a dragged panel
            // swaps places): the servers and panels, which the display and the images are
            // derived from, are the same, in whatever order.
            let moved_only = self.settings.connections == self.applied.connections
                && same_panels(&self.settings.layout.panels(), &self.applied.layout.panels());
            self.applied = self.settings.clone();
            if !moved_only {
                self.dirty = true;
                self.image_dirty = true;
            }
        }
        if self.image_dirty {
            self.image_dirty = false;
            self.draw_images();
            changed = true;
        }
        if self.dirty {
            self.dirty = false;
            self.robot_dirty = false;
            self.display = self.derive();
            changed = true;
            self.frame();
        } else if self.robot_dirty {
            self.robot_dirty = false;
            if let Some(root) = self.display.robot.as_ref().map(|r| r.root) {
                self.display.robot = self.pose_robot(root);
                changed = true;
            }
        }
        changed
    }

    /// Opens a connection for every listed server that has none, and closes the ones no longer
    /// listed (dropping a source disconnects it, and drops what it delivered).
    fn sync_connections(&mut self) {
        let want = &self.settings.connections;
        let before = self.conns.len();
        self.conns.retain(|c| want.contains(&c.url));
        let kept = self.conns.len();
        for url in want {
            if !self.conns.iter().any(|c| &c.url == url) {
                self.conns.push(Connection::open(url, self.next_conn, &self.post));
                self.next_conn += 1;
            }
        }
        self.conns.sort_by_key(|c| want.iter().position(|u| *u == c.url));
        if kept != before || self.conns.len() != kept {
            self.dirty = true;
            self.image_dirty = true;
            // Nothing left from before: frame the new data as at startup.
            if kept == 0 {
                self.framed = Framed::Nothing;
            }
        }
    }

    /// Loads the URDF file when its path changes, and the described robot when its topic sends a
    /// different URDF; then compares the two.
    fn sync_robot(&mut self) {
        let mut loaded = false;
        let urdf = self.v3().robot.urdf.trim();
        let want = (!urdf.is_empty()).then(|| urdf.to_string());
        if self.robot_file.as_ref().map(|(p, _)| p) != want.as_ref() {
            self.robot_file = want.map(|path| {
                let model = robot::load(Path::new(&path), viewport::MAX_ROBOT_VERTICES);
                log_load(&path, &model);
                (path, model)
            });
            loaded = true;
        }
        let topic = self.description_topic();
        let text = topic.as_ref().and_then(|t| self.received().find_map(|r| r.descriptions.get(t)));
        let current = self.robot_description.as_ref().map(|(t, x, _)| (t, x));
        if current != topic.as_ref().zip(text) {
            self.robot_description = topic.zip(text.cloned()).map(|(topic, text)| {
                let model = robot::load_text(&text, viewport::MAX_ROBOT_VERTICES);
                log_load(&topic, &model);
                (topic, text, model)
            });
            loaded = true;
        }
        if loaded {
            let file = self.robot_file.as_ref().and_then(|(p, m)| Some((p, m.as_ref().ok()?)));
            let described = self.robot_description.as_ref().and_then(|(t, _, m)| Some((t, m.as_ref().ok()?)));
            self.robot_mismatch = file.zip(described).and_then(|((path, f), (topic, d))| {
                let why = f.robot.kinematic_difference(&d.robot)?;
                let file = Path::new(path).file_name().map_or(path.clone(), |n| n.to_string_lossy().into_owned());
                let draw_described = !d.mesh.parts.is_empty() || f.mesh.parts.is_empty();
                let drawing = if draw_described {
                    format!("Drawing {topic}'s.")
                } else {
                    format!("Drawing {file}: none of {topic}'s meshes load here.")
                };
                let message = format!("{file} ({}) isn't the robot {topic} describes ({}): {why}. {drawing}", f.robot.name, d.robot.name);
                Some(Mismatch { message, draw_described })
            });
            self.dirty = true;
        }
    }

    /// Expires every connection's poses and trails (`Received::expire`); returns when the next
    /// is due, for the frame clock to wake then.
    fn expire(&mut self, now: f64) -> Option<f64> {
        let (timeout, trail) = (self.v3().poses.timeout as f64, self.v3().poses.trail as f64);
        let mut dropped = false;
        let due = self.conns.iter_mut().filter_map(|c| {
            let (gone, next) = c.received.expire(now, timeout, trail);
            dropped |= gone;
            next
        }).reduce(f64::min);
        self.dirty |= dropped;
        due
    }

    /// Draws each view's annotations into a copy of its image. Views of the same topic that
    /// hide the same annotations share one copy; a copy no view shows any more is dropped.
    fn draw_images(&mut self) {
        let wanted: BTreeSet<(String, BTreeSet<String>)> =
            self.image_views().into_iter().filter_map(|view| Some((self.image_topic(view)?, view.hidden.clone()))).collect();
        self.images.retain(|key, _| wanted.contains(key));
        for key in wanted {
            let Some(image) = self.received().find_map(|r| r.images.get(&key.0)).cloned() else { continue };
            let mut rgba = image.rgba.clone();
            annotate::draw(&mut rgba, image.width, image.height, self.annotations(&key.1).collect::<Vec<_>>());
            self.images.entry(key).or_default().set(image.width, image.height, rgba);
        }
    }

    /// Frames the camera on the data the first time there is some (see `Framed`).
    fn frame(&mut self) {
        if self.display.bounds.is_none() {
            return;
        }
        // Whether the camera is still where the last framing put it.
        let untouched = match self.framed {
            Framed::Nothing => true,
            Framed::Robot(c) => c == self.camera,
            Framed::Cloud => return,
        };
        if self.cloud().is_some() {
            if untouched {
                self.camera = (self.frame_with)(self);
            }
            self.framed = Framed::Cloud;
        } else if self.framed == Framed::Nothing && self.display.robot.is_some() {
            self.camera = (self.frame_with)(self);
            self.framed = Framed::Robot(self.camera);
        }
    }

    /// The shown robot at the received joint positions, its root at `root` in the fixed frame.
    fn pose_robot(&self, root: Pose) -> Option<PosedRobot> {
        let m = self.shown_robot()?;
        let none = HashMap::new();
        let joints = self.joints().map_or(&none, |(_, j)| j);
        let links = m.robot.link_poses(joints).iter().map(|p| root.then(p).matrix()).collect();
        Some(PosedRobot { root, links })
    }

    fn fixed_frame(&self, tf: &TfTree) -> Option<String> {
        if let Some(f) = &self.v3().fixed_frame {
            return Some(f.clone());
        }
        let data_frame = self.cloud().map(|(_, c)| c.frame_id.clone())
            .or_else(|| self.received().find_map(|r| r.poses.values().next()).map(|(_, p)| p.frame_id.clone()));
        let robot_root = self.shown_robot().map(|m| m.robot.links[0].clone());
        Some(tf.root_of(&anchor_frame(data_frame, robot_root, &tf.frames())?))
    }

    fn derive(&self) -> Display {
        let s = self.v3();
        let mut tf = TfTree::default();
        self.received().for_each(|r| tf.extend(&r.tf));
        let mut d = Display { fixed: self.fixed_frame(&tf), ..Display::default() };
        let Some(fixed) = d.fixed.clone() else {
            d.tf = tf;
            return d;
        };
        // The shown cloud's and image card's errors first: a cloud or image refused or
        // undecodable is why it isn't drawn. (Joint names the robot lacks go before even those,
        // below.)
        let shown = [self.cloud_topic().filter(|_| s.points.show), self.image_card().and_then(|c| self.image_topic(c))];
        let mut problems: Vec<String> = shown.into_iter()
            .flatten()
            .filter_map(|t| self.error(&t).map(|e| format!("{t}: {e}")))
            .collect();

        // The cloud: placement, a sample of its points in the display frame for bounds and the
        // height ramp (16 Ki points: percentiles to well under 1%).
        let mut sample: Vec<Vec3> = Vec::new();
        if let Some((topic, c)) = self.cloud() {
            match tf.lookup(&fixed, &c.frame_id) {
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
        if s.frames {
            let mut origins = BTreeMap::new();
            for f in tf.frames() {
                if let Some(p) = tf.lookup(&fixed, &f) {
                    let m = p.matrix();
                    lines.axes(m, axis, 2.5);
                    let o = m.transform_point3(Vec3::ZERO);
                    marks.push(o);
                    if s.labels {
                        d.labels.push((f.clone(), o));
                    }
                    origins.insert(f, o);
                }
            }
            for (parent, child) in tf.edges() {
                if let (Some(&a), Some(&b)) = (origins.get(parent), origins.get(child)) {
                    lines.segment(a, b, 1.5, TREE_COLOR);
                }
            }
        }
        if s.poses.show {
            for r in self.received() {
                for ((topic, _), (_, poses)) in &r.poses {
                    let Some(frame) = tf.lookup(&fixed, &poses.frame_id) else {
                        problems.push(format!("{topic}: no transform from {} to {fixed}", poses.frame_id));
                        continue;
                    };
                    for item in &poses.items {
                        let m = frame.then(&item.pose).matrix();
                        lines.axes(m, axis * 0.6, 3.0);
                        let o = m.transform_point3(Vec3::ZERO);
                        marks.push(o);
                        if let (true, Some(l)) = (s.labels, &item.label) {
                            d.labels.push((l.clone(), o));
                        }
                        if let Some(v) = item.velocity {
                            let v = frame.rotation.as_quat() * v.as_vec3();
                            lines.arrow(o, o + v * VELOCITY_LOOKAHEAD_S, 2.5, VELOCITY_COLOR);
                        }
                    }
                }
                for t in r.trails.values() {
                    let Some(frame) = tf.lookup(&fixed, &t.frame_id) else { continue };
                    let at = |p: &DVec3| frame.then(&Pose { translation: *p, rotation: Default::default() }).translation.as_vec3();
                    let pts: Vec<Vec3> = t.points.iter().map(|(_, p)| at(p)).collect();
                    for w in pts.windows(2) {
                        lines.segment(w[0], w[1], 2.0, TRAIL_COLOR);
                    }
                }
            }
        }
        if let Some(m) = self.shown_robot() {
            let root = &m.robot.links[0];
            let placed = tf.lookup(&fixed, root).unwrap_or_else(|| {
                d.robot_note = Some(format!("No transform from {root} to {fixed}: drawn at {fixed}'s origin"));
                Pose::IDENTITY
            });
            // Joint names the model doesn't have mean the server drives some other robot.
            if let Some((topic, joints)) = self.joints() {
                let movable: HashSet<&str> = m.robot.movable().collect();
                let mut unknown: Vec<&str> = joints.keys().map(String::as_str).filter(|n| !movable.contains(n)).collect();
                unknown.sort();
                if !unknown.is_empty() {
                    let shown = unknown.iter().take(3).copied().collect::<Vec<_>>().join(", ");
                    let more = if unknown.len() > 3 { format!(" and {} more", unknown.len() - 3) } else { String::new() };
                    problems.insert(0, format!("{topic} names joints {} doesn't have: {shown}{more}", m.robot.name));
                }
            }
            d.robot = self.pose_robot(placed);
            marks.extend(d.robot.iter().flat_map(|r| &r.links).map(|l| l.transform_point3(Vec3::ZERO)));
        }
        d.lines = Arc::new(lines);
        d.bounds = bounds(&sample, &marks, axis);
        // The grid scales with the cloud; frames and poses alone (an axis length across) keep
        // the default 1 m cells rather than shrink them to centimetres.
        if let (Some((_, radius)), false) = (d.bounds, sample.is_empty()) {
            d.grid_cell = 10f32.powf((radius / 12.0).log10().round());
        }
        d.shade = match self.cloud().map(|(_, c)| &c.color) {
            Some(PointColor::Rgb) => Shade::Rgb,
            Some(PointColor::Field { min, max, .. }) => Shade::Field { lo: *min, hi: *max },
            _ => match percentiles(sample.iter().map(|p| p.z).collect(), 0.02, 0.98) {
                Some((lo, hi)) if hi > lo => Shade::Height { lo, hi },
                Some((lo, _)) => Shade::Height { lo, hi: lo + 1.0 },
                None => Shade::Height { lo: 0.0, hi: 1.0 },
            },
        };
        d.problem = problems.into_iter().next();
        d.tf = tf;
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
        let s = self.v3();
        View {
            camera: self.camera,
            cloud: match (self.cloud(), self.display.cloud_model) {
                (Some((_, c)), Some(m)) if s.points.show => Some((c, m)),
                _ => None,
            },
            shade: self.display.shade,
            point_size: s.points.size,
            decimate: s.points.decimate,
            grid: s.grid.then_some(self.display.grid_cell),
            radius: self.display.bounds.map_or(2.0, |(_, r)| r),
            lines: &self.display.lines,
            // A robot with nothing drawable (every mesh failed) has no parts to draw.
            robot: self.display.robot.as_ref().zip(self.shown_robot())
                .filter(|(_, m)| !m.mesh.parts.is_empty())
                .map(|(p, m)| RobotView { mesh: &m.mesh, links: &p.links, opacity: s.robot.opacity }),
        }
    }

    /// Points drawn per frame, for the badge.
    pub fn drawn_points(&self) -> usize {
        match self.view().cloud {
            Some((c, _)) => c.points.len().div_ceil(self.v3().points.decimate as usize),
            None => 0,
        }
    }
}

fn log_load(from: &str, model: &Result<Model, String>) {
    match model {
        Ok(m) => eprintln!(
            "lantern: loaded {} from {from} in {:.1} ms: {} links, {} movable joints, {} parts, {} vertices, {} problems",
            m.robot.name, m.load_ms, m.robot.links.len(), m.robot.movable().count(), m.mesh.parts.len(), m.mesh.vertices.len(), m.problems.len(),
        ),
        Err(e) => eprintln!("lantern: can't load {from}: {e}"),
    }
}

/// The frame whose TF root the display uses, with no fixed frame chosen: the cloud's or a pose's
/// frame; else the robot's root, if TF knows it; else any TF frame; else the robot's root alone
/// (no TF at all). A robot root TF doesn't know must not become the fixed frame while there is
/// TF: nothing would connect to it, and every frame would vanish.
fn anchor_frame(data: Option<String>, robot_root: Option<String>, tf_frames: &[String]) -> Option<String> {
    data.or_else(|| robot_root.clone().filter(|f| tf_frames.contains(f)))
        .or_else(|| tf_frames.first().cloned())
        .or(robot_root)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn layout(json: &str) -> Layout {
        serde_json::from_str(json).unwrap()
    }

    fn urdf(s: &Settings) -> &str {
        &s.layout.view3d().unwrap().robot.urdf
    }

    // `--connect B --urdf cli.urdf --layout camera` for one run: saving keeps the file's servers
    // and layout, the 3D panel's URDF among its settings, until the user changes the servers or
    // the layout (any panel's settings included) in the UI, which then save as changed, but
    // still with the file's URDF.
    #[test]
    fn command_line_values_last_for_the_run_unless_changed() {
        let file = Settings { connections: vec!["ws://a".into()], layout: layout(r#"{"row": [{"panel": "3d", "robot": {"urdf": "file.urdf"}}, {"panel": "data"}]}"#) };
        let camera = Layout::named_or_file("camera", &file.layout).unwrap();
        let o = Overrides { connections: Some(vec!["ws://b".into()]), robot_urdf: Some("cli.urdf".into()), layout: Some(camera), ..Overrides::default() };
        let post: Post = Arc::new(|_, _| {});
        let mut s = State::new(file.clone(), o, post);
        assert_eq!((s.settings.connections.as_slice(), urdf(&s.settings)), (["ws://b".to_string()].as_slice(), "cli.urdf"));
        assert_eq!(s.overrides.to_save(&s.settings, &file), file, "nothing changed in the UI: the file stays");
        // A panel setting changed: the file's tiling, with the change.
        s.settings.connections.push("ws://c".into());
        s.edit3d(|v| v.points.size = 4.0);
        let saved = s.overrides.to_save(&s.settings, &file);
        assert_eq!(saved.connections, s.settings.connections);
        assert!(saved.layout.same_tiling(&file.layout), "the file's tiling");
        assert_eq!((saved.layout.view3d().unwrap().points.size, urdf(&saved)), (4.0, "file.urdf"));
        // The tiling edited: the layout as edited, still with the URDF --urdf replaced.
        s.settings.layout.close(&[1]);
        let saved = s.overrides.to_save(&s.settings, &file);
        assert_eq!(saved.layout.panels().len(), 1);
        assert_eq!((saved.layout.view3d().unwrap().points.size, urdf(&saved)), (4.0, "file.urdf"));
    }

    // A --layout file's own URDF is what --urdf replaced, and what saves.
    #[test]
    fn urdf_override_saves_back_the_layout_files_urdf() {
        let file = Settings::default();
        let arm = layout(r#"{"panel": "3d", "robot": {"urdf": "arm.urdf"}}"#);
        let o = Overrides { robot_urdf: Some("cli.urdf".into()), layout: Some(arm), ..Overrides::default() };
        let post: Post = Arc::new(|_, _| {});
        let mut s = State::new(file.clone(), o, post);
        s.edit3d(|v| v.grid = false);
        s.settings.layout.insert(&[], Dir::Row, Panel::new(PanelKind::Data));
        assert_eq!(urdf(&s.overrides.to_save(&s.settings, &file)), "arm.urdf");
    }

    // Each server's topics are subscribed to as some panel shows them: two Image panels' two
    // topics and the annotations either draws, the Data panel's stats but not its text (off),
    // the cloud, and not the pose topic the 3D view hides. The 3D view's own image and data
    // cards add nothing, as the layout has panels for them.
    #[test]
    fn subscriptions_are_the_union_of_the_panels_topics() {
        let settings = Settings {
            connections: vec!["ws://127.0.0.1:9".into()],
            layout: layout(r#"{"row": [
                {"panel": "3d", "poses": {"hidden": ["/pose"]}, "image": {"topic": "/card"}},
                {"panel": "image", "topic": "/left", "hidden": ["/marks"]},
                {"panel": "image", "topic": "/right"},
                {"panel": "data", "text": false}]}"#),
        };
        let post: Post = Arc::new(|_, _| {});
        let mut s = State::new(settings, Overrides::default(), post);
        s.sync_connections();
        let channel = |topic: &str, kind| crate::source::Channel { topic: topic.into(), schema: String::new(), encoding: String::new(), kind: Some(kind) };
        let channels = vec![
            channel("/cloud", Kind::PointCloud), channel("/pose", Kind::Poses), channel("/card", Kind::Image),
            channel("/left", Kind::Image), channel("/right", Kind::Image), channel("/marks", Kind::Annotations),
            channel("/status", Kind::Text), channel("/diag", Kind::Stats),
        ];
        s.apply(vec![(s.conns[0].id, Event::Channels(channels))], 0.0);
        let mut wanted: Vec<String> = s.wanted().topics.into_iter().collect();
        wanted.sort();
        assert_eq!(wanted, ["/cloud", "/diag", "/left", "/marks", "/right"]);
    }

    // The Med 7's root is `world`; a stack that publishes TF under `map` must keep its TF tree
    // drawable (the robot lands at map's origin, with a note) rather than switch to `world`.
    #[test]
    fn a_robot_root_tf_doesnt_know_doesnt_take_over_the_fixed_frame() {
        let tf = vec!["base".to_string(), "map".to_string()];
        let world = || Some("world".to_string());
        assert_eq!(anchor_frame(None, world(), &tf).as_deref(), Some("base"), "a TF frame, whose root is then used");
        let with_world = vec!["map".to_string(), "world".to_string()];
        assert_eq!(anchor_frame(None, world(), &with_world).as_deref(), Some("world"), "a root TF knows wins");
        assert_eq!(anchor_frame(None, world(), &[]).as_deref(), Some("world"), "no TF: the robot's own frame");
        assert_eq!(anchor_frame(Some("lidar".into()), world(), &tf).as_deref(), Some("lidar"), "data first");
    }
}
