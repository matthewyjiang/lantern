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
//! `move_panel`, `move_divider`) keep it that way. They address a node by its path: the child index to take at each branch,
//! from the root.

use std::path::Path;

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

/// Where a panel dropped onto another lands: in its place, the two swapping, or on one side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Landing {
    Swap,
    Beside(Edge),
}

/// A side of a panel, where another goes beside it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Edge {
    Left,
    Right,
    Top,
    Bottom,
}

impl Edge {
    /// The direction of the branch that puts a panel on this side, and whether it goes first.
    fn placement(self) -> (Dir, bool) {
        match self {
            Edge::Left => (Dir::Row, true),
            Edge::Right => (Dir::Row, false),
            Edge::Top => (Dir::Column, true),
            Edge::Bottom => (Dir::Column, false),
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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "RawNode", into = "RawNode")]
pub enum Layout {
    Panel(Panel),
    /// Two or more children, each with its weight.
    Split { dir: Dir, children: Vec<(f32, Layout)> },
}

impl Default for Layout {
    fn default() -> Self {
        Layout::Panel(Panel::new(PanelKind::View3d))
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
    Layout::Panel(Panel::new(kind))
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

    /// The preset `name`, its panels with the settings of `current`'s same-kind panels.
    pub fn preset_keeping(name: &str, current: &Layout) -> Option<Layout> {
        let mut preset = Layout::preset(name)?;
        preset.adopt(current);
        Some(preset)
    }

    /// A preset by name, its panels taking the settings of `current`'s (`adopt`), else the
    /// layout file at that path as it is.
    pub fn named_or_file(value: &str, current: &Layout) -> Result<Layout, String> {
        match Layout::preset_keeping(value, current) {
            Some(preset) => Ok(preset),
            None => Layout::load(Path::new(value)),
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
            Layout::Panel(p) => vec![p],
            Layout::Split { children, .. } => children.iter().flat_map(|(_, c)| c.panels()).collect(),
        }
    }

    pub fn panels_mut(&mut self) -> Vec<&mut Panel> {
        match self {
            Layout::Panel(p) => vec![p],
            Layout::Split { children, .. } => children.iter_mut().flat_map(|(_, c)| c.panels_mut()).collect(),
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
            Layout::Panel(p) => Some(p),
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
            Layout::Panel(p) => Some(p),
            Layout::Split { .. } => None,
        }
    }

    /// The path of the first panel showing `kind`.
    pub fn find(&self, kind: PanelKind) -> Option<Vec<usize>> {
        match self {
            Layout::Panel(p) => (p.kind() == kind).then(Vec::new),
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

    /// Puts `panel` on the `edge` side of the panel at `path`, with half its space. A second 3D
    /// panel isn't added.
    pub fn insert(&mut self, path: &[usize], edge: Edge, panel: Panel) {
        if panel.kind() == PanelKind::View3d && self.has(PanelKind::View3d) {
            return;
        }
        let (dir, first) = edge.placement();
        let new = Layout::Panel(panel);
        // In a branch of the same direction, the new panel is a sibling.
        if let Some((&i, parent)) = path.split_last()
            && let Some(Layout::Split { dir: d, children }) = self.at_mut(parent)
            && *d == dir
            && i < children.len()
        {
            children[i].0 /= 2.0;
            children.insert(if first { i } else { i + 1 }, (children[i].0, new));
            return;
        }
        if let Some(node @ Layout::Panel(_)) = self.at_mut(path) {
            let old = std::mem::take(node);
            let children = if first { vec![(1.0, new), (1.0, old)] } else { vec![(1.0, old), (1.0, new)] };
            *node = Layout::Split { dir, children };
        }
    }

    /// Puts the panel at `from` where `landing` says, relative to the panel at `to`.
    pub fn drop_panel(&mut self, from: &[usize], to: &[usize], landing: Landing) {
        match landing {
            Landing::Swap => self.swap(from, to),
            Landing::Beside(edge) => self.move_panel(from, to, edge),
        }
    }

    /// Exchanges the panels at `a` and `b`, each with its settings; the tiling stays.
    fn swap(&mut self, a: &[usize], b: &[usize]) {
        if a == b || self.panel(a).is_none() || self.panel(b).is_none() {
            return;
        }
        let first = std::mem::take(self.at_mut(a).expect("found above"));
        let second = std::mem::replace(self.at_mut(b).expect("found above"), first);
        *self.at_mut(a).expect("found above") = second;
    }

    /// Moves the panel at `from`, with its settings, to the `edge` side of the panel at `to`. A
    /// panel already there stays as it is.
    fn move_panel(&mut self, from: &[usize], to: &[usize], edge: Edge) {
        let paths = self.paths();
        let (Some(f), Some(t)) = (paths.iter().position(|p| p == from), paths.iter().position(|p| p == to)) else { return };
        if f == t || self.beside(from, to, edge) {
            return;
        }
        let panel = self.panel(from).cloned().expect("found above");
        self.close(from);
        // Closing shifts paths but not the order of the panels: find `to` by its place in it.
        let to = self.paths().swap_remove(if t > f { t - 1 } else { t });
        self.insert(&to, edge, panel);
    }

    /// Whether the node at `a` is already on the `edge` side of its sibling at `b`.
    fn beside(&self, a: &[usize], b: &[usize], edge: Edge) -> bool {
        let (dir, first) = edge.placement();
        let (Some((&i, parent)), Some((&j, other))) = (a.split_last(), b.split_last()) else { return false };
        parent == other
            && matches!(self.at(parent), Some(Layout::Split { dir: d, .. }) if *d == dir)
            && if first { i + 1 == j } else { j + 1 == i }
    }

    /// Every panel's path, in the order of `panels`.
    pub fn paths(&self) -> Vec<Vec<usize>> {
        match self {
            Layout::Panel(_) => vec![Vec::new()],
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

    /// Gives this layout's panels the settings of `from`'s panels of the same kind, in order:
    /// a preset picked over a layout keeps its robot, its image topics, ... Panels `from` has no
    /// match for keep their own.
    pub fn adopt(&mut self, from: &Layout) {
        for kind in PanelKind::ALL {
            let old = from.panels().into_iter().filter(|p| p.kind() == kind);
            for (new, old) in self.panels_mut().into_iter().filter(|p| p.kind() == kind).zip(old) {
                *new = old.clone();
            }
        }
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
            (Layout::Panel(a), Layout::Panel(b)) => a.kind() == b.kind(),
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
            Layout::Panel(p) => {
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
                Layout::Panel(panel(kind, self.settings, at)?)
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
        l.insert(&[], Edge::Right, Panel::new(Image));
        assert_eq!(l, row(vec![(1.0, leaf(View3d)), (1.0, leaf(Image))]));
        // Along the branch: a sibling, sharing the panel's space, on the side asked for.
        l.insert(&[1], Edge::Left, Panel::new(Data));
        assert_eq!(l, row(vec![(1.0, leaf(View3d)), (0.5, leaf(Data)), (0.5, leaf(Image))]));
        // Across it: a branch in the panel's place.
        l.insert(&[0], Edge::Bottom, Panel::new(Topics));
        let Layout::Split { children, .. } = &l else { panic!() };
        assert_eq!(children[0], (1.0, column(vec![(1.0, leaf(View3d)), (1.0, leaf(Topics))])));
        // Still one 3D view.
        let before = l.clone();
        l.insert(&[2], Edge::Top, Panel::new(View3d));
        assert_eq!(l, before);
    }

    // A panel dragged onto another's side moves there with its settings; onto its middle, the
    // two swap. The others keep their places.
    #[test]
    fn panels_move_and_swap() {
        let mut l = parse(r#"{"row": [{"panel": "3d", "grid": false}, {"column": [{"panel": "image", "topic": "/a"}, {"panel": "data"}]}]}"#).unwrap();
        // The row of one left collapses, so the 3D panel joins the column, half of Data's space.
        l.move_panel(&[0], &[1, 1], Edge::Bottom);
        assert_eq!(l, parse(r#"{"column": [{"panel": "image", "topic": "/a"}, {"panel": "data", "weight": 0.5}, {"panel": "3d", "grid": false, "weight": 0.5}]}"#).unwrap());
        l.swap(&[0], &[2]);
        assert_eq!(l, parse(r#"{"column": [{"panel": "3d", "grid": false}, {"panel": "data", "weight": 0.5}, {"panel": "image", "topic": "/a", "weight": 0.5}]}"#).unwrap());
        // Onto the side of a neighbour it's already on: nothing moves, weights included.
        let mut l = parse(r#"{"row": [{"panel": "image", "weight": 2}, {"panel": "data"}]}"#).unwrap();
        let before = l.clone();
        l.drop_panel(&[0], &[1], Landing::Beside(Edge::Left));
        assert_eq!(l, before);
        l.drop_panel(&[0], &[1], Landing::Swap);
        assert_eq!(l, parse(r#"{"row": [{"panel": "data", "weight": 2}, {"panel": "image"}]}"#).unwrap());
        // Onto a later panel in the same branch: closing shifts its path, not the target.
        let mut l = parse(r#"{"row": [{"panel": "image"}, {"panel": "data"}, {"panel": "topics"}]}"#).unwrap();
        l.move_panel(&[0], &[2], Edge::Left);
        assert_eq!(l, parse(r#"{"row": [{"panel": "data"}, {"panel": "image", "weight": 0.5}, {"panel": "topics", "weight": 0.5}]}"#).unwrap());
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
