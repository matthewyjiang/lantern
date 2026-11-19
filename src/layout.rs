//! The panel layout: a tree whose leaves are panels and whose branches tile their children in a
//! row or a column, each child with a weight (its share of the branch's length). Each panel
//! carries its own settings (`settings.rs`). The layout is saved in `settings.json`, and reads
//! and writes the same JSON on its own in a layout file (`--layout`, Settings → Layout):
//!
//! ```json
//! {"row": [{"panel": "3d", "weight": 2, "robot": {"urdf": "arm.urdf"}},
//!          {"column": [{"panel": "image", "topic": "/cam/left"}, {"panel": "data"}]}]}
//! ```
//!
//! A panel's settings sit beside `panel`, and only those that differ from the defaults are
//! written; a setting left out takes its default. The file is checked when it's read
//! (`RawNode` → `Layout`), so every `Layout` in the app is valid: its weights are positive, its branches have children, its panels' settings
//! parse, and it has at most one 3D panel. The edits (`insert`, `close`, `show`, `swap`,
//! `move_divider`) keep it that way. They address a node by its path: the child index to take at each branch,
//! from the root.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::settings::{DataSettings, ImageSettings, View3dSettings};

/// What a panel shows. Adding a panel type starts here: a variant, its row in `ALL`, `id` and
/// `label`, its `Panel` variant with its settings, then its body in `panels::body` and its
/// settings in `ui::panel_settings`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanelKind {
    /// The 3D viewport. One per layout: there is one camera and one offscreen 3D pass.
    View3d,
    /// A camera image with its annotations.
    Image,
    /// Text topics' newest messages and stats topics' statuses and readings.
    Data,
    /// Every advertised topic, as Settings → Topics lists them.
    Topics,
}

impl PanelKind {
    pub const ALL: [PanelKind; 4] = [PanelKind::View3d, PanelKind::Image, PanelKind::Data, PanelKind::Topics];

    /// The name in a layout file.
    pub fn id(self) -> &'static str {
        match self {
            PanelKind::View3d => "3d",
            PanelKind::Image => "image",
            PanelKind::Data => "data",
            PanelKind::Topics => "topics",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            PanelKind::View3d => "3D view",
            PanelKind::Image => "Image",
            PanelKind::Data => "Data",
            PanelKind::Topics => "Topics",
        }
    }
}

/// A panel: what it shows, with that kind's settings.
#[derive(Clone, Debug, PartialEq)]
pub enum Panel {
    View3d(Box<View3dSettings>),
    Image(ImageSettings),
    Data(DataSettings),
    Topics,
}

impl Panel {
    /// A panel of `kind` with the default settings.
    pub fn new(kind: PanelKind) -> Panel {
        match kind {
            PanelKind::View3d => Panel::View3d(Box::default()),
            PanelKind::Image => Panel::Image(ImageSettings::default()),
            PanelKind::Data => Panel::Data(DataSettings::default()),
            PanelKind::Topics => Panel::Topics,
        }
    }

    pub fn kind(&self) -> PanelKind {
        match self {
            Panel::View3d(_) => PanelKind::View3d,
            Panel::Image(_) => PanelKind::Image,
            Panel::Data(_) => PanelKind::Data,
            Panel::Topics => PanelKind::Topics,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    /// Children side by side.
    Row,
    /// Children stacked.
    Column,
}

/// A panel's identity while the app runs, so it can be followed as it moves around the layout
/// (and animate as itself). Not saved: every layout read from a file gets new ones.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PaneId(u64);

impl PaneId {
    fn new() -> PaneId {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        PaneId(NEXT.fetch_add(1, Ordering::Relaxed))
    }
}

impl std::fmt::Display for PaneId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

/// Layouts are equal when they tile the same panels with the same settings, whatever their
/// panes' identities.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "RawNode", into = "RawNode")]
pub enum Layout {
    Panel(PaneId, Panel),
    /// Two or more children, each with its weight.
    Split { dir: Dir, children: Vec<(f32, Layout)> },
}

impl PartialEq for Layout {
    fn eq(&self, other: &Layout) -> bool {
        match (self, other) {
            (Layout::Panel(_, a), Layout::Panel(_, b)) => a == b,
            (Layout::Split { dir: d1, children: c1 }, Layout::Split { dir: d2, children: c2 }) => d1 == d2 && c1 == c2,
            _ => false,
        }
    }
}

impl Default for Layout {
    fn default() -> Self {
        leaf(PanelKind::View3d)
    }
}

/// A built-in layout, by the name `--layout` and Settings → Layout offer it under.
pub struct Preset {
    pub name: &'static str,
    pub layout: fn() -> Layout,
}

pub const PRESETS: [Preset; 3] = [
    Preset { name: "3d", layout: Layout::default },
    Preset { name: "camera", layout: camera },
    Preset { name: "dashboard", layout: dashboard },
];

fn leaf(kind: PanelKind) -> Layout {
    Layout::pane(Panel::new(kind))
}

/// The 3D view, with the image to its right.
fn camera() -> Layout {
    Layout::Split { dir: Dir::Row, children: vec![(2.0, leaf(PanelKind::View3d)), (1.0, leaf(PanelKind::Image))] }
}

/// The 3D view, with the image above a Data panel to its right.
fn dashboard() -> Layout {
    let side = vec![(1.0, leaf(PanelKind::Image)), (1.0, leaf(PanelKind::Data))];
    Layout::Split { dir: Dir::Row, children: vec![(2.0, leaf(PanelKind::View3d)), (1.0, Layout::Split { dir: Dir::Column, children: side })] }
}

impl Layout {
    pub fn preset(name: &str) -> Option<Layout> {
        PRESETS.iter().find(|p| p.name == name).map(|p| (p.layout)())
    }

    /// The preset `name`, its panels taking the identities and settings of `current`'s same-kind
    /// panels (`adopt`).
    pub fn preset_keeping(name: &str, current: &Layout) -> Option<Layout> {
        let mut preset = Layout::preset(name)?;
        preset.adopt(current);
        Some(preset)
    }

    /// A preset by name, its panels taking the settings of `current`'s (`adopt`), else the
    /// layout file at that path as it is, its panels taking only their identities. A panel that
    /// keeps its identity moves to its new place rather than being replaced.
    pub fn named_or_file(value: &str, current: &Layout) -> Result<Layout, String> {
        match Layout::preset_keeping(value, current) {
            Some(preset) => Ok(preset),
            None => Layout::load(Path::new(value)).map(|mut file| {
                file.pair_with(current, |(id, _), (old, _)| *id = old);
                file
            }),
        }
    }

    pub fn load(path: &Path) -> Result<Layout, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("can't read {}: {e}", path.display()))?;
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).expect("a layout serializes");
        std::fs::write(path, text + "\n").map_err(|e| format!("can't write {}: {e}", path.display()))
    }

    pub fn has(&self, kind: PanelKind) -> bool {
        self.find(kind).is_some()
    }

    /// Every panel, depth first (left to right, top to bottom).
    pub fn panels(&self) -> Vec<&Panel> {
        match self {
            Layout::Panel(_, p) => vec![p],
            Layout::Split { children, .. } => children.iter().flat_map(|(_, c)| c.panels()).collect(),
        }
    }

    pub fn panels_mut(&mut self) -> Vec<&mut Panel> {
        self.panes_mut().into_iter().map(|(_, p)| p).collect()
    }

    fn panes(&self) -> Vec<(PaneId, &Panel)> {
        match self {
            Layout::Panel(id, p) => vec![(*id, p)],
            Layout::Split { children, .. } => children.iter().flat_map(|(_, c)| c.panes()).collect(),
        }
    }

    fn panes_mut(&mut self) -> Vec<(&mut PaneId, &mut Panel)> {
        match self {
            Layout::Panel(id, p) => vec![(id, p)],
            Layout::Split { children, .. } => children.iter_mut().flat_map(|(_, c)| c.panes_mut()).collect(),
        }
    }

    /// Calls `f` with each of this layout's panes and `from`'s pane of the same kind, in order.
    /// Panes `from` has no match for are left out.
    fn pair_with(&mut self, from: &Layout, mut f: impl FnMut((&mut PaneId, &mut Panel), (PaneId, &Panel))) {
        for kind in PanelKind::ALL {
            let old = from.panes().into_iter().filter(|(_, p)| p.kind() == kind);
            for (new, old) in self.panes_mut().into_iter().filter(|(_, p)| p.kind() == kind).zip(old) {
                f(new, old);
            }
        }
    }

    /// The 3D panel's settings, if the layout has one.
    pub fn view3d(&self) -> Option<&View3dSettings> {
        self.panels().into_iter().find_map(|p| match p {
            Panel::View3d(v) => Some(&**v),
            _ => None,
        })
    }

    pub fn view3d_mut(&mut self) -> Option<&mut View3dSettings> {
        self.panels_mut().into_iter().find_map(|p| match p {
            Panel::View3d(v) => Some(&mut **v),
            _ => None,
        })
    }

    /// The panel at `path`, if there's one there.
    pub fn panel(&self, path: &[usize]) -> Option<&Panel> {
        match self.at(path)? {
            Layout::Panel(_, p) => Some(p),
            Layout::Split { .. } => None,
        }
    }

    fn at(&self, path: &[usize]) -> Option<&Layout> {
        match (path, self) {
            ([], node) => Some(node),
            ([i, rest @ ..], Layout::Split { children, .. }) => children.get(*i)?.1.at(rest),
            _ => None,
        }
    }

    pub fn panel_mut(&mut self, path: &[usize]) -> Option<&mut Panel> {
        match self.at_mut(path)? {
            Layout::Panel(_, p) => Some(p),
            Layout::Split { .. } => None,
        }
    }

    /// The path of the first panel showing `kind`.
    pub fn find(&self, kind: PanelKind) -> Option<Vec<usize>> {
        match self {
            Layout::Panel(_, p) => (p.kind() == kind).then(Vec::new),
            Layout::Split { children, .. } => children.iter().enumerate().find_map(|(i, (_, c))| {
                let mut path = c.find(kind)?;
                path.insert(0, i);
                Some(path)
            }),
        }
    }

    fn at_mut(&mut self, path: &[usize]) -> Option<&mut Layout> {
        match (path, self) {
            ([], node) => Some(node),
            ([i, rest @ ..], Layout::Split { children, .. }) => children.get_mut(*i)?.1.at_mut(rest),
            _ => None,
        }
    }

    /// Shows `kind` in the panel at `path`, with that kind's default settings. The 3D view is in
    /// one panel at most, so asking for it where another panel has it swaps the two, each with
    /// its settings.
    pub fn show(&mut self, path: &[usize], kind: PanelKind) {
        let Some(here) = self.panel(path).map(Panel::kind) else { return };
        if here == kind {
            return;
        }
        if kind == PanelKind::View3d
            && let Some(there) = self.find(kind)
        {
            self.swap(path, &there);
        } else {
            *self.panel_mut(path).expect("found above") = Panel::new(kind);
        }
    }

    /// Puts `panel` after the panel at `path` in `dir` (to its right, or below it), with half its
    /// space. A second 3D panel isn't added.
    pub fn insert(&mut self, path: &[usize], dir: Dir, panel: Panel) {
        if panel.kind() == PanelKind::View3d && self.has(PanelKind::View3d) {
            return;
        }
        let new = Layout::pane(panel);
        // In a branch of the same direction, the new panel is a sibling.
        if let Some((&i, parent)) = path.split_last()
            && let Some(Layout::Split { dir: d, children }) = self.at_mut(parent)
            && *d == dir
            && i < children.len()
        {
            children[i].0 /= 2.0;
            children.insert(i + 1, (children[i].0, new));
            return;
        }
        if let Some(node @ Layout::Panel(..)) = self.at_mut(path) {
            let old = std::mem::replace(node, Layout::HOLE);
            *node = Layout::Split { dir, children: vec![(1.0, old), (1.0, new)] };
        }
    }

    /// Exchanges the panels at `a` and `b`, each with its settings and identity; the tiling
    /// stays.
    pub fn swap(&mut self, a: &[usize], b: &[usize]) {
        if a == b || self.panel(a).is_none() || self.panel(b).is_none() {
            return;
        }
        let first = std::mem::replace(self.at_mut(a).expect("found above"), Layout::HOLE);
        let second = std::mem::replace(self.at_mut(b).expect("found above"), first);
        *self.at_mut(a).expect("found above") = second;
    }

    /// The path of the pane `id`, if the layout has it.
    pub fn path_of(&self, id: PaneId) -> Option<Vec<usize>> {
        self.paths().into_iter().find(|p| matches!(self.at(p), Some(Layout::Panel(pane, _)) if *pane == id))
    }

    /// What a node holds while an edit moves it out, briefly: no pane, so no identity used up.
    const HOLE: Layout = Layout::Split { dir: Dir::Row, children: Vec::new() };

    fn pane(panel: Panel) -> Layout {
        Layout::Panel(PaneId::new(), panel)
    }

    /// Every panel's path, in the order of `panels`.
    pub fn paths(&self) -> Vec<Vec<usize>> {
        match self {
            Layout::Panel(..) => vec![Vec::new()],
            Layout::Split { children, .. } => children.iter().enumerate().flat_map(|(i, (_, c))| {
                c.paths().into_iter().map(move |mut p| {
                    p.insert(0, i);
                    p
                })
            }).collect(),
        }
    }

    /// Removes the panel at `path`; its siblings share its space. A branch left with one child
    /// becomes that child. The root panel stays: a layout always has a panel.
    pub fn close(&mut self, path: &[usize]) {
        let Some((&i, parent)) = path.split_last() else { return };
        let Some(node) = self.at_mut(parent) else { return };
        let Layout::Split { children, .. } = node else { return };
        if i >= children.len() {
            return;
        }
        children.remove(i);
        if children.len() == 1 {
            *node = children.remove(0).1;
        }
    }

    /// Moves the divider after child `i` of the branch at `path`: child `i` takes `weight` and
    /// child `i + 1` the rest of the two's. Neither is pushed below `min`; one already smaller
    /// can grow but not shrink. Weights round to 0.001 to keep saved files short, and a move of
    /// less than that changes nothing.
    pub fn move_divider(&mut self, path: &[usize], i: usize, weight: f32, min: f32) {
        let Some(Layout::Split { children, .. }) = self.at_mut(path) else { return };
        if i + 1 >= children.len() || !weight.is_finite() {
            return;
        }
        let (a, b) = (children[i].0, children[i + 1].0);
        let round = |w: f32| (w * 1000.0).round() / 1000.0;
        let first = round(weight.clamp(min.min(a), a + b - min.min(b)));
        let second = round(a + b - first);
        if first > 0.0 && second > 0.0 && (first - a).abs() >= 0.0005 {
            (children[i].0, children[i + 1].0) = (first, second);
        }
    }

    /// Gives this layout's panels the identities and settings of `from`'s panels of the same
    /// kind, in order: a preset picked over a layout keeps its robot, its image topics, ... and
    /// the panels it keeps move rather than being replaced. Panels `from` has no match for keep
    /// their own.
    pub fn adopt(&mut self, from: &Layout) {
        self.pair_with(from, |(id, panel), (old_id, old)| {
            *id = old_id;
            *panel = old.clone();
        });
    }

    /// Gives each panel the settings of the same-kind panel in `after`, in order, where those
    /// differ from `before`'s (`after` and `before` have the same tiling): the panels changed
    /// since `before`.
    pub fn adopt_changes(&mut self, before: &Layout, after: &Layout) {
        for kind in PanelKind::ALL {
            let of = |l: &Layout| l.panels().into_iter().filter(|p| p.kind() == kind).cloned().collect::<Vec<_>>();
            let pairs = of(before).into_iter().zip(of(after));
            for (new, (b, a)) in self.panels_mut().into_iter().filter(|p| p.kind() == kind).zip(pairs) {
                if a != b {
                    *new = a;
                }
            }
        }
    }

    /// The same tiling as `other`: the same rows, columns, weights and kinds of panel, whatever
    /// their settings.
    pub fn same_tiling(&self, other: &Layout) -> bool {
        match (self, other) {
            (Layout::Panel(_, a), Layout::Panel(_, b)) => a.kind() == b.kind(),
            (Layout::Split { dir: d1, children: c1 }, Layout::Split { dir: d2, children: c2 }) => {
                d1 == d2 && c1.len() == c2.len() && c1.iter().zip(c2).all(|((w1, a), (w2, b))| w1 == w2 && a.same_tiling(b))
            }
            _ => false,
        }
    }

    fn count(&self, kind: PanelKind) -> usize {
        self.panels().into_iter().filter(|p| p.kind() == kind).count()
    }

    /// The layout, if it has at most one 3D panel.
    fn one_3d(self) -> Result<Layout, String> {
        let views = self.count(PanelKind::View3d);
        if views > 1 {
            return Err(format!("layout: {views} \"3d\" panels, and a layout can have one"));
        }
        Ok(self)
    }
}

/// A layout node as the file has it: exactly one of `panel`, `row` and `column`, its weight in
/// the branch that holds it (1 when left out), and a panel's settings beside them.
#[derive(Default, Serialize, Deserialize)]
struct RawNode {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    panel: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    row: Option<Vec<RawNode>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    column: Option<Vec<RawNode>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    weight: Option<f32>,
    /// Every other key: the panel's settings.
    #[serde(flatten)]
    settings: Map<String, Value>,
}

impl RawNode {
    fn of(layout: Layout, weight: f32) -> RawNode {
        let weight = (weight != 1.0).then_some(weight);
        match layout {
            Layout::Panel(_, p) => {
                let settings = match &p {
                    Panel::View3d(v) => non_default(&**v),
                    Panel::Image(i) => non_default(i),
                    Panel::Data(d) => non_default(d),
                    Panel::Topics => Map::new(),
                };
                RawNode { panel: Some(p.kind().id().into()), weight, settings, ..RawNode::default() }
            }
            Layout::Split { dir, children } => {
                let children = Some(children.into_iter().map(|(w, c)| RawNode::of(c, w)).collect());
                match dir {
                    Dir::Row => RawNode { row: children, weight, ..RawNode::default() },
                    Dir::Column => RawNode { column: children, weight, ..RawNode::default() },
                }
            }
        }
    }

    /// The node and its weight; `at` says where it is in the file, for errors.
    fn checked(self, at: &str) -> Result<(f32, Layout), String> {
        let weight = self.weight.unwrap_or(1.0);
        if !(weight.is_finite() && weight > 0.0) {
            return Err(format!("{at}: weight {weight} must be a number above 0"));
        }
        let layout = match (self.panel, self.row, self.column) {
            (Some(id), None, None) => {
                let kind = PanelKind::ALL.into_iter().find(|k| k.id() == id).ok_or_else(|| {
                    let known = PanelKind::ALL.map(PanelKind::id).join(", ");
                    format!("{at}: no panel called \"{id}\" (the panels are {known})")
                })?;
                Layout::pane(panel(kind, self.settings, at)?)
            }
            (None, Some(children), None) => {
                no_settings(&self.settings, at)?;
                branch(Dir::Row, children, &format!("{at}.row"))?
            }
            (None, None, Some(children)) => {
                no_settings(&self.settings, at)?;
                branch(Dir::Column, children, &format!("{at}.column"))?
            }
            _ => return Err(format!("{at}: needs exactly one of \"panel\", \"row\" and \"column\"")),
        };
        Ok((weight, layout))
    }
}

fn branch(dir: Dir, children: Vec<RawNode>, at: &str) -> Result<Layout, String> {
    let mut children = children.into_iter().enumerate()
        .map(|(i, c)| c.checked(&format!("{at}[{i}]")))
        .collect::<Result<Vec<_>, _>>()?;
    match children.len() {
        0 => Err(format!("{at}: has no panels")),
        // A branch of one is that one.
        1 => Ok(children.remove(0).1),
        _ => Ok(Layout::Split { dir, children }),
    }
}

/// A panel of `kind` with the settings a file gives it.
fn panel(kind: PanelKind, settings: Map<String, Value>, at: &str) -> Result<Panel, String> {
    Ok(match kind {
        PanelKind::View3d => Panel::View3d(Box::new(parse::<View3dSettings>(settings, at)?.checked())),
        PanelKind::Image => Panel::Image(parse(settings, at)?),
        PanelKind::Data => Panel::Data(parse(settings, at)?),
        PanelKind::Topics => {
            no_settings(&settings, at)?;
            Panel::Topics
        }
    })
}

/// A node that takes no settings (a branch, a Topics panel) mustn't be given any.
fn no_settings(settings: &Map<String, Value>, at: &str) -> Result<(), String> {
    match settings.keys().next() {
        Some(key) => Err(format!("{at}: unknown field `{key}`")),
        None => Ok(()),
    }
}

fn parse<T: DeserializeOwned>(settings: Map<String, Value>, at: &str) -> Result<T, String> {
    serde_json::from_value(Value::Object(settings)).map_err(|e| format!("{at}: {e}"))
}

/// `settings` as a file has them: only the keys (nested ones too) that differ from the defaults.
fn non_default<T: Serialize + Default>(settings: &T) -> Map<String, Value> {
    let value = serde_json::to_value(settings).expect("settings serialize");
    let default = serde_json::to_value(T::default()).expect("settings serialize");
    diff(value, &default)
}

fn diff(value: Value, default: &Value) -> Map<String, Value> {
    let Value::Object(map) = value else { return Map::new() };
    map.into_iter().filter_map(|(k, v)| match (v, default.get(&k)) {
        (Value::Object(o), Some(d @ Value::Object(_))) => {
            let sub = diff(Value::Object(o), d);
            (!sub.is_empty()).then_some((k, Value::Object(sub)))
        }
        (v, Some(d)) if v == *d => None,
        (v, _) => Some((k, v)),
    }).collect()
}

impl TryFrom<RawNode> for Layout {
    type Error = String;

    fn try_from(raw: RawNode) -> Result<Layout, String> {
        // The root's own weight has nothing to be a share of.
        let (_, layout) = raw.checked("layout")?;
        layout.one_3d()
    }
}

impl From<Layout> for RawNode {
    fn from(layout: Layout) -> RawNode {
        RawNode::of(layout, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use PanelKind::*;

    fn parse(json: &str) -> Result<Layout, String> {
        serde_json::from_str(json).map_err(|e| e.to_string())
    }

    fn row(children: Vec<(f32, Layout)>) -> Layout {
        Layout::Split { dir: Dir::Row, children }
    }

    fn column(children: Vec<(f32, Layout)>) -> Layout {
        Layout::Split { dir: Dir::Column, children }
    }

    // The same file gives the same layout, and a layout the UI changed writes a file that reads
    // back as it.
    #[test]
    fn a_layout_file_round_trips() {
        let json = r#"{"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data", "weight": 0.5}]}]}"#;
        let layout = parse(json).unwrap();
        let expected = row(vec![(2.0, leaf(View3d)), (1.0, column(vec![(1.0, leaf(Image)), (0.5, leaf(Data))]))]);
        assert_eq!(layout, expected);
        assert_eq!(parse(&serde_json::to_string(&layout).unwrap()).unwrap(), expected);
    }

    // A hand-written file's mistakes are named with where they are.
    #[test]
    fn a_bad_layout_file_says_what_and_where() {
        let err = |json| parse(json).unwrap_err();
        assert!(err(r#"{"row": [{"panel": "3d"}, {"panel": "plot"}]}"#).contains(r#"layout.row[1]: no panel called "plot" (the panels are 3d, image, data, topics)"#));
        assert!(err(r#"{"row": [{"panel": "3d"}, {"column": [{"panel": "3d"}, {"panel": "data"}]}]}"#).contains(r#"2 "3d" panels"#));
        assert!(err(r#"{"column": [{"panel": "3d", "weight": 0}, {"panel": "data"}]}"#).contains("layout.column[0]: weight 0 must be a number above 0"));
        assert!(err(r#"{"panel": "3d", "row": []}"#).contains("exactly one of"));
        assert!(err(r#"{"row": []}"#).contains("layout.row: has no panels"));
        assert!(err(r#"{"panel": "3d", "size": 2}"#).contains("unknown field `size`"));
    }

    #[test]
    fn inserting_adds_a_sibling_or_a_branch() {
        let mut l = Layout::default();
        l.insert(&[], Dir::Row, Panel::new(Image));
        assert_eq!(l, row(vec![(1.0, leaf(View3d)), (1.0, leaf(Image))]));
        // Along the branch: a sibling after it, sharing the panel's space.
        l.insert(&[0], Dir::Row, Panel::new(Data));
        assert_eq!(l, row(vec![(0.5, leaf(View3d)), (0.5, leaf(Data)), (1.0, leaf(Image))]));
        // Across it: a branch in the panel's place.
        l.insert(&[0], Dir::Column, Panel::new(Topics));
        let Layout::Split { children, .. } = &l else { panic!() };
        assert_eq!(children[0], (0.5, column(vec![(1.0, leaf(View3d)), (1.0, leaf(Topics))])));
        // Still one 3D view.
        let before = l.clone();
        l.insert(&[2], Dir::Column, Panel::new(View3d));
        assert_eq!(l, before);
    }

    // Swapping exchanges two panels with their settings and identities; the tiling stays.
    #[test]
    fn swapping_exchanges_panels_in_place() {
        let mut l = parse(r#"{"row": [{"panel": "3d", "grid": false, "weight": 2}, {"column": [{"panel": "image", "topic": "/a"}, {"panel": "data"}]}]}"#).unwrap();
        let ids = |l: &Layout| l.paths().iter().map(|p| match l.at(p) { Some(Layout::Panel(id, _)) => *id, _ => unreachable!() }).collect::<Vec<_>>();
        let before = ids(&l);
        l.swap(&[0], &[1, 1]);
        assert_eq!(l, parse(r#"{"row": [{"panel": "data", "weight": 2}, {"column": [{"panel": "image", "topic": "/a"}, {"panel": "3d", "grid": false}]}]}"#).unwrap());
        assert_eq!(ids(&l), [before[2], before[1], before[0]]);
        assert_eq!(l.path_of(before[0]), Some(vec![1, 1]));
    }

    #[test]
    fn closing_collapses_a_branch_of_one_and_keeps_the_last_panel() {
        let mut l = parse(r#"{"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}"#).unwrap();
        l.close(&[1, 0]);
        assert_eq!(l, row(vec![(2.0, leaf(View3d)), (1.0, leaf(Data))]));
        l.close(&[0]);
        assert_eq!(l, leaf(Data));
        l.close(&[]);
        assert_eq!(l, leaf(Data));
    }

    // One 3D view: showing it in another panel moves it there with its settings, and that
    // panel's kind and settings go back. Any other kind starts from its defaults.
    #[test]
    fn showing_the_3d_view_elsewhere_swaps() {
        let mut l = parse(r#"{"row": [{"panel": "3d", "grid": false}, {"panel": "image"}, {"panel": "data", "text": false}]}"#).unwrap();
        l.show(&[2], View3d);
        assert_eq!(l, parse(r#"{"row": [{"panel": "data", "text": false}, {"panel": "image"}, {"panel": "3d", "grid": false}]}"#).unwrap());
        l.show(&[0], Image);
        assert_eq!(l, parse(r#"{"row": [{"panel": "image"}, {"panel": "image"}, {"panel": "3d", "grid": false}]}"#).unwrap());
    }

    // A panel's settings sit beside it in the file, only those off their defaults are written,
    // and a mistake in them says where it is.
    #[test]
    fn panel_settings_round_trip_and_write_only_what_differs() {
        let json = r#"{"row":[{"panel":"3d","weight":2.0,"robot":{"urdf":"arm.urdf"}},{"panel":"image","topic":"/cam/left"},{"panel":"image","topic":"/cam/right"}]}"#;
        let l = parse(json).unwrap();
        assert_eq!(l.view3d().unwrap().robot.urdf, "arm.urdf");
        assert_eq!(serde_json::to_string(&l).unwrap(), json);
        assert_eq!(serde_json::to_string(&Layout::default()).unwrap(), r#"{"panel":"3d"}"#);
        let err = parse(r#"{"row": [{"panel": "3d"}, {"panel": "image", "topic": 3}]}"#).unwrap_err();
        assert!(err.contains("layout.row[1]: invalid type: integer `3`"), "{err}");
        assert!(parse(r#"{"panel": "topics", "topic": "/x"}"#).unwrap_err().contains("layout: unknown field `topic`"));
    }

    // Picking a preset over a layout keeps each kind of panel's settings, in order.
    #[test]
    fn a_preset_adopts_the_current_panels_settings() {
        let current = parse(r#"{"column": [{"panel": "image", "topic": "/a"}, {"panel": "3d", "robot": {"urdf": "arm.urdf"}}]}"#).unwrap();
        let l = Layout::named_or_file("dashboard", &current).unwrap();
        assert!(l.same_tiling(&Layout::preset("dashboard").unwrap()));
        assert_eq!(l.view3d().unwrap().robot.urdf, "arm.urdf");
        assert_eq!(l.panel(&[1, 0]), current.panel(&[0]));
        // The panels it keeps are the same panes, moved.
        assert_eq!(l.path_of(current.panes()[0].0), Some(vec![1, 0]));
    }

    fn weights(l: &Layout) -> Vec<f32> {
        let Layout::Split { children, .. } = l else { panic!("not a branch") };
        children.iter().map(|(w, _)| *w).collect()
    }

    #[test]
    fn a_divider_moves_between_its_two_panels_and_stops_at_the_minimum() {
        let mut l = row(vec![(2.0, leaf(View3d)), (1.0, leaf(Image)), (1.0, leaf(Data))]);
        l.move_divider(&[], 0, 1.25, 0.2);
        assert_eq!(weights(&l), [1.25, 1.75, 1.0]);
        l.move_divider(&[], 0, 9.0, 0.2);
        assert_eq!(weights(&l), [2.8, 0.2, 1.0]);
    }

    // A panel already under the minimum (after splits, or from a file): its divider stays where
    // it is until it's dragged, and then the panel only grows.
    #[test]
    fn a_divider_beside_a_panel_under_the_minimum_doesnt_jump() {
        let mut l = row(vec![(1.3333, leaf(View3d)), (0.1, leaf(Image))]);
        l.move_divider(&[], 0, 1.3333, 0.5);
        assert_eq!(weights(&l), [1.3333, 0.1], "no travel: untouched, not even rounded");
        l.move_divider(&[], 0, 1.4, 0.5);
        assert_eq!(weights(&l), [1.3333, 0.1], "the small panel doesn't shrink");
        l.move_divider(&[], 0, 0.8, 0.5);
        assert_eq!(weights(&l), [0.8, 0.633]);
    }

    // A preset saved to a file reads back as the preset.
    #[test]
    fn presets_round_trip_as_files() {
        for Preset { name, layout } in PRESETS {
            assert_eq!(parse(&serde_json::to_string(&layout()).unwrap()).as_ref(), Ok(&layout()), "{name}");
        }
    }
}
