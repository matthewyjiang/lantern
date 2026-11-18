//! The panel layout: a tree whose leaves are panels and whose branches tile their children in a
//! row or a column, each child with a weight (its share of the branch's length). The layout is a
//! setting, saved in `settings.json`, and reads and writes the same JSON on its own in a layout
//! file (`--layout`, Settings → Layout):
//!
//! ```json
//! {"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}
//! ```
//!
//! The file is checked when it's read (`RawNode` → `Layout`), so every `Layout` in the app is
//! valid: its weights are positive, its branches have children, and it has at most one 3D panel.
//! The edits (`split`, `close`, `show`, `move_divider`) keep it that way. They address a node
//! by its path: the child index to take at each branch, from the root.

use std::path::Path;

use serde::{Deserialize, Serialize};

/// What a panel shows. Adding a panel type starts here: a variant, its row in `ALL`, `id` and
/// `label`, then its body in `panels::body`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanelKind {
    /// The 3D viewport. One per layout: there is one camera and one offscreen 3D pass.
    View3d,
    /// The camera image with its annotations.
    Image,
    /// Text topics' newest messages and stats topics' statuses and readings.
    Data,
    /// Every advertised topic, with the controls of Settings → Topics.
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
    Panel(PanelKind),
    /// Two or more children, each with its weight.
    Split { dir: Dir, children: Vec<(f32, Layout)> },
}

impl Default for Layout {
    fn default() -> Self {
        Layout::Panel(PanelKind::View3d)
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

/// The 3D view, with the image to its right.
fn camera() -> Layout {
    Layout::Split { dir: Dir::Row, children: vec![(2.0, Layout::Panel(PanelKind::View3d)), (1.0, Layout::Panel(PanelKind::Image))] }
}

/// The 3D view, with the image above a Data panel to its right.
fn dashboard() -> Layout {
    let side = vec![(1.0, Layout::Panel(PanelKind::Image)), (1.0, Layout::Panel(PanelKind::Data))];
    Layout::Split { dir: Dir::Row, children: vec![(2.0, Layout::Panel(PanelKind::View3d)), (1.0, Layout::Split { dir: Dir::Column, children: side })] }
}

impl Layout {
    pub fn preset(name: &str) -> Option<Layout> {
        PRESETS.iter().find(|p| p.name == name).map(|p| (p.layout)())
    }

    /// A preset by name, else the layout file at that path.
    pub fn named_or_file(value: &str) -> Result<Layout, String> {
        Layout::preset(value).map_or_else(|| Layout::load(Path::new(value)), Ok)
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

    /// The path of the first panel showing `kind`.
    fn find(&self, kind: PanelKind) -> Option<Vec<usize>> {
        match self {
            Layout::Panel(k) => (*k == kind).then(Vec::new),
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

    /// Shows `kind` in the panel at `path`. The 3D view is in one panel at most, so asking for
    /// it where another panel has it swaps the two.
    pub fn show(&mut self, path: &[usize], kind: PanelKind) {
        let Some(&mut Layout::Panel(old)) = self.at_mut(path) else { return };
        if kind == PanelKind::View3d
            && let Some(Layout::Panel(other)) = self.find(kind).and_then(|p| self.at_mut(&p))
        {
            *other = old;
        }
        if let Some(Layout::Panel(k)) = self.at_mut(path) {
            *k = kind;
        }
    }

    /// Splits the panel at `path` in two along `dir`, the new panel after it with half its
    /// space. The new panel shows the first kind the layout doesn't have yet, or Data.
    pub fn split(&mut self, path: &[usize], dir: Dir) {
        let new = Layout::Panel(PanelKind::ALL.into_iter().find(|k| !self.has(*k)).unwrap_or(PanelKind::Data));
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
        if let Some(node @ Layout::Panel(_)) = self.at_mut(path) {
            let old = std::mem::take(node);
            *node = Layout::Split { dir, children: vec![(1.0, old), (1.0, new)] };
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
}

/// A layout node as the file has it: exactly one of `panel`, `row` and `column`, and its weight
/// in the branch that holds it (1 when left out).
#[derive(Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawNode {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    panel: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    row: Option<Vec<RawNode>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    column: Option<Vec<RawNode>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    weight: Option<f32>,
}

impl RawNode {
    fn of(layout: Layout, weight: f32) -> RawNode {
        let weight = (weight != 1.0).then_some(weight);
        match layout {
            Layout::Panel(k) => RawNode { panel: Some(k.id().into()), weight, ..RawNode::default() },
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
                Layout::Panel(kind)
            }
            (None, Some(children), None) => branch(Dir::Row, children, &format!("{at}.row"))?,
            (None, None, Some(children)) => branch(Dir::Column, children, &format!("{at}.column"))?,
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

impl TryFrom<RawNode> for Layout {
    type Error = String;

    fn try_from(raw: RawNode) -> Result<Layout, String> {
        // The root's own weight has nothing to be a share of.
        let (_, layout) = raw.checked("layout")?;
        let views = layout.count(PanelKind::View3d);
        if views > 1 {
            return Err(format!("layout: {views} \"3d\" panels, and a layout can have one"));
        }
        Ok(layout)
    }
}

impl From<Layout> for RawNode {
    fn from(layout: Layout) -> RawNode {
        RawNode::of(layout, 1.0)
    }
}

impl Layout {
    fn count(&self, kind: PanelKind) -> usize {
        match self {
            Layout::Panel(k) => (*k == kind) as usize,
            Layout::Split { children, .. } => children.iter().map(|(_, c)| c.count(kind)).sum(),
        }
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
        let expected = row(vec![(2.0, Layout::Panel(View3d)), (1.0, column(vec![(1.0, Layout::Panel(Image)), (0.5, Layout::Panel(Data))]))]);
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
    fn splitting_adds_a_sibling_or_a_branch() {
        let mut l = Layout::default();
        l.split(&[], Dir::Row);
        assert_eq!(l, row(vec![(1.0, Layout::Panel(View3d)), (1.0, Layout::Panel(Image))]));
        // Along the branch: a sibling, sharing the split panel's space.
        l.split(&[1], Dir::Row);
        assert_eq!(l, row(vec![(1.0, Layout::Panel(View3d)), (0.5, Layout::Panel(Image)), (0.5, Layout::Panel(Data))]));
        // Across it: a branch in the panel's place.
        l.split(&[0], Dir::Column);
        let Layout::Split { children, .. } = &l else { panic!() };
        assert_eq!(children[0], (1.0, column(vec![(1.0, Layout::Panel(View3d)), (1.0, Layout::Panel(Topics))])));
    }

    #[test]
    fn closing_collapses_a_branch_of_one_and_keeps_the_last_panel() {
        let mut l = parse(r#"{"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}"#).unwrap();
        l.close(&[1, 0]);
        assert_eq!(l, row(vec![(2.0, Layout::Panel(View3d)), (1.0, Layout::Panel(Data))]));
        l.close(&[0]);
        assert_eq!(l, Layout::Panel(Data));
        l.close(&[]);
        assert_eq!(l, Layout::Panel(Data));
    }

    // One 3D view: showing it in another panel moves it there, and that panel's kind goes back.
    #[test]
    fn showing_the_3d_view_elsewhere_swaps() {
        let mut l = row(vec![(1.0, Layout::Panel(View3d)), (1.0, Layout::Panel(Image)), (1.0, Layout::Panel(Data))]);
        l.show(&[2], View3d);
        assert_eq!(l, row(vec![(1.0, Layout::Panel(Data)), (1.0, Layout::Panel(Image)), (1.0, Layout::Panel(View3d))]));
        l.show(&[0], Image);
        assert_eq!(l, row(vec![(1.0, Layout::Panel(Image)), (1.0, Layout::Panel(Image)), (1.0, Layout::Panel(View3d))]));
    }

    fn weights(l: &Layout) -> Vec<f32> {
        let Layout::Split { children, .. } = l else { panic!("not a branch") };
        children.iter().map(|(w, _)| *w).collect()
    }

    #[test]
    fn a_divider_moves_between_its_two_panels_and_stops_at_the_minimum() {
        let mut l = row(vec![(2.0, Layout::Panel(View3d)), (1.0, Layout::Panel(Image)), (1.0, Layout::Panel(Data))]);
        l.move_divider(&[], 0, 1.25, 0.2);
        assert_eq!(weights(&l), [1.25, 1.75, 1.0]);
        l.move_divider(&[], 0, 9.0, 0.2);
        assert_eq!(weights(&l), [2.8, 0.2, 1.0]);
    }

    // A panel already under the minimum (after splits, or from a file): its divider stays where
    // it is until it's dragged, and then the panel only grows.
    #[test]
    fn a_divider_beside_a_panel_under_the_minimum_doesnt_jump() {
        let mut l = row(vec![(1.3333, Layout::Panel(View3d)), (0.1, Layout::Panel(Image))]);
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
