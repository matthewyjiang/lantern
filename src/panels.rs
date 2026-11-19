//! The panels, tiled as `Settings::layout` says: `tile` works out each panel's rectangle and
//! each divider's from the weights, and every panel is a card placed there, keyed by its pane
//! (`PaneId`) so it springs to its new place when the layout changes.
//!
//! Over a panel the pointer is on, its header: its name (a handle: drag it over another panel
//! and the two swap places), its settings button (`ui::panel_settings`) and its menu (split,
//! change what it shows, close). All of them edit `settings.layout`.
//!
//! A new panel type is a `PanelKind` and `Panel` variant (`layout.rs`) and its body here, in
//! `body`.

use rune::tok::{self, type_};
use rune::{Anim, Cursor, Cx, Drag, El, Phase, ScrollSpec, Spring, Tx, col, el, oklcha, row, text};
use rune_kit::{Kit, Menu, MenuEntry, MenuItem, Placement};

use crate::app::{PanelDrag, State, Submenu};
use crate::layout::{Dir, Layout, PaneId, Panel, PanelKind};
use crate::settings::{DataSettings, ImageSettings};
use crate::ui::{self, MONO_SMALL};

/// The gap between two panels, which is also the divider to drag.
const GAP: f32 = 8.0;
/// A divider stops where either child beside it would get shorter or narrower than this, points:
/// a panel that size still fits its header's handle and buttons. Panels nested in that child,
/// and panels made by splitting, can be smaller.
const MIN_PANEL: f32 = 120.0;
/// How far the pointer goes before a press on a header drags its panel, points.
const DRAG_START: f32 = 4.0;
/// A panel's header, over its top: what draws there sits this far down.
pub(crate) const HEADER_H: f32 = 44.0;

/// `[x, y, width, height]`, points.
type Rect = [f32; 4];

/// A panel's place.
struct Tile {
    id: PaneId,
    path: Vec<usize>,
    rect: Rect,
}

/// The gap after child `i` of the branch at `path`.
struct Gap {
    path: Vec<usize>,
    i: usize,
    dir: Dir,
    rect: Rect,
    /// Child `i`'s weight, and points per unit of weight along the branch.
    weight: f32,
    per_weight: f32,
}

/// Every panel's and divider's place when `layout` fills `area`. Each child of a branch gets
/// its weight's share of the branch, less the gaps between them.
fn tile(layout: &Layout, area: Rect) -> (Vec<Tile>, Vec<Gap>) {
    fn walk(layout: &Layout, path: &mut Vec<usize>, [x, y, w, h]: Rect, tiles: &mut Vec<Tile>, gaps: &mut Vec<Gap>) {
        let (dir, children) = match layout {
            Layout::Panel(id, _) => return tiles.push(Tile { id: *id, path: path.clone(), rect: [x, y, w, h] }),
            Layout::Split { dir, children } => (*dir, children),
        };
        let length = if dir == Dir::Row { w } else { h };
        let total: f32 = children.iter().map(|(w, _)| w).sum();
        let per_weight = ((length - GAP * (children.len() - 1) as f32) / total).max(0.0);
        let mut at = if dir == Dir::Row { x } else { y };
        for (i, (weight, child)) in children.iter().enumerate() {
            let size = weight * per_weight;
            let rect = |at, size| if dir == Dir::Row { [at, y, size, h] } else { [x, at, w, size] };
            path.push(i);
            walk(child, path, rect(at, size), tiles, gaps);
            path.pop();
            at += size;
            if i + 1 < children.len() {
                gaps.push(Gap { path: path.clone(), i, dir, rect: rect(at, GAP), weight: *weight, per_weight });
                at += GAP;
            }
        }
    }
    let (mut tiles, mut gaps) = (Vec::new(), Vec::new());
    walk(layout, &mut Vec::new(), area, &mut tiles, &mut gaps);
    (tiles, gaps)
}

/// The layout, filling the window under the top bar. Menus go to `layer`.
pub fn layout(s: &State, cx: &mut Cx<State>, k: &Kit, layer: &mut Vec<El>) -> El {
    let mut panels = el().key("panels").grow(1.0).h_full().min_w(0.0).min_h(0.0);
    // Placed once the area is laid out: its place in the window, and its size.
    if let Some([ox, oy, w, h]) = cx.window_layout("panels") {
        // In their places' order (left to right, top to bottom), which is the Tab order; keyed by
        // pane, a card that changes place moves rather than remounting.
        let (tiles, gaps) = tile(&s.settings.layout, [0.0, 0.0, w, h]);
        for t in &tiles {
            let p = s.settings.layout.panel(&t.path).expect("tiled from the layout");
            panels = panels.child(card(s, cx, k, layer, p, t, [ox, oy, w, h]));
        }
        for g in gaps {
            panels = panels.child(divider(g));
        }
    }
    el().grow(1.0).h_full().min_w(0.0).padding(10.0).pt(0.0).child(panels)
}

/// `panel` for the root, then `-i` for each step of the path: `panel-1-0`. The keys of a
/// panel's header buttons and dividers, which stay with the place; the card and what it shows
/// are keyed by the pane (`pane_key`), which moves.
pub(crate) fn key_of(path: &[usize]) -> String {
    path.iter().fold("panel".to_string(), |key, i| format!("{key}-{i}"))
}

fn pane_key(id: PaneId) -> String {
    format!("pane-{id}")
}

/// A divider: dragging it moves space between the two children it separates.
fn divider(g: Gap) -> El {
    let t = tok::theme(true);
    let [x, y, w, h] = g.rect;
    let (cursor, grip) = match g.dir {
        Dir::Row => (Cursor::ResizeColumn, el().size(2.0, 28.0)),
        Dir::Column => (Cursor::ResizeRow, el().size(28.0, 2.0)),
    };
    let Gap { path, i, dir, weight, per_weight, .. } = g;
    el().key(format!("{}-divider-{i}", key_of(&path))).absolute(x, y).size(w, h).cursor(cursor).center().radius(GAP / 2.0).hover(t.hover)
        .child(grip.radius(1.0).fill(t.separator))
        .on_drag(move |s: &mut State, d: &mut Drag, _: &mut Tx| {
            if per_weight <= 0.0 {
                return;
            }
            // `data` keeps the weight the drag started from.
            if d.phase == Phase::Down {
                d.data = weight;
                return;
            }
            let travel = if dir == Dir::Row { d.dx } else { d.dy };
            let weight = if d.cancelled() { d.data } else { d.data + travel / per_weight };
            s.settings.layout.move_divider(&path, i, weight, MIN_PANEL / per_weight);
        })
}

/// One panel's card, at its tile: its body and, while the pointer is over it, its header.
/// `area` is the panels' place in the window. A dragged card follows the pointer above the
/// others, at the size it was picked up at, while the others spring to their new places.
fn card(s: &State, cx: &mut Cx<State>, k: &Kit, layer: &mut Vec<El>, p: &Panel, t: &Tile, area: Rect) -> El {
    let key = pane_key(t.id);
    let [x, y, w, h] = t.rect;
    let card = el().key(key.clone()).absolute(x, y).hover_group().clip_rounded(tok::radius::LG).border(1.0, oklcha(1.0, 0.0, 0.0, 0.07))
        .child(body(s, cx, p, &key))
        .child(header(s, k, layer, p, t, area));
    match s.panel_drag.as_ref().filter(|d| d.id == t.id) {
        Some(d) => {
            let [gw, gh] = d.size;
            let at = [d.pointer[0] - d.grab[0] - area[0] - x, d.pointer[1] - d.grab[1] - area[1] - y];
            k.elevate(card.size(gw, gh).motion(Spring::NONE).offset(at[0], at[1]).offset_motion(Anim::INSTANT).raise(), &k.elevation.floating)
        }
        // The default spring: one set here would also govern what's inside (the 3D view's labels).
        None => card.size(w, h),
    }
}
/// What panel `p` shows, filling its card. `key` is the panel's, to prefix the keys of what's
/// inside (a kind can be in several panels).
fn body(s: &State, cx: &mut Cx<State>, p: &Panel, key: &str) -> El {
    match p {
        Panel::View3d(_) => ui::view3d(s, cx),
        Panel::Image(view) => image(s, cx, view, key),
        Panel::Data(view) => data(s, view, key),
        Panel::Topics => scrolling(format!("{key}-scroll")).child(el().w_full().padding(12.0).pt(HEADER_H).child(ui::topics(s, &format!("{key}-topic")))),
    }
}

/// The panel's name, which drags it, then its settings button and menu. Shown while the pointer
/// is over the panel, while its menu or settings are open, and (the name alone) while it's dragged.
fn header(s: &State, k: &Kit, layer: &mut Vec<El>, p: &Panel, t: &Tile, area: Rect) -> El {
    let theme = tok::theme(true);
    let title = match p {
        Panel::Image(view) => match s.image(view) {
            Some((topic, image)) => format!("{topic} · {}×{}", image.width, image.height),
            None => "Image".into(),
        },
        _ => p.kind().label().into(),
    };
    let dragged = s.panel_drag.as_ref().is_some_and(|d| d.id == t.id);
    let (id, rect) = (t.id, t.rect);
    let handle = ui::glass().key(format!("{}-handle", pane_key(id))).role("button").label(format!("Move {title}")).h(28.0).px(10.0).gap(8.0).items_center().min_w(0.0)
        .cursor(if dragged { Cursor::Grabbing } else { Cursor::Grab })
        .child(grip_glyph())
        .child(text(title).font(type_::LABEL).color(theme.text).truncate().min_w(0.0))
        .on_drag(move |s: &mut State, d: &mut Drag, tx: &mut Tx| drag_panel(s, d, tx, id, rect, area));
    let bar = row().gap(8.0).items_center().justify_between()
        .pin(Some(8.0), Some(8.0), Some(8.0), None)
        .child(handle.shrink());
    if dragged {
        return bar;
    }
    let key = key_of(&t.path);
    let buttons = row().gap(6.0).items_center()
        .children(has_settings(p).then(|| ui::panel_settings_button(s, &key, &t.path)))
        .child(menu(s, k, layer, p.kind(), &key, &t.path));
    let bar = bar.child(buttons);
    let open = s.popup.as_ref().is_some_and(|p| p.path() == t.path);
    if open { bar } else { bar.group_opacity(0.0, 1.0) }
}

fn has_settings(p: &Panel) -> bool {
    !matches!(p, Panel::Topics)
}

/// A press on the handle of pane `id`, whose card is at `rect` in the panels at `area`: past
/// `DRAG_START` the card follows the pointer, and over another panel the two swap places. Let
/// go, it springs into its place; cancelled (Escape), the layout goes back to how it was.
fn drag_panel(s: &mut State, d: &mut Drag, tx: &mut Tx, id: PaneId, [x, y, w, h]: Rect, [ox, oy, aw, ah]: Rect) {
    let pointer = d.window_point();
    match d.phase {
        Phase::Down => {
            s.popup = None;
            // A drag can end without an `Up` (a modal opening under it): never finish that one.
            s.panel_drag = None;
        }
        Phase::Move => {
            if s.panel_drag.is_none() {
                if d.dx.hypot(d.dy) < DRAG_START {
                    return;
                }
                let pressed = [pointer[0] - d.dx - ox - x, pointer[1] - d.dy - oy - y];
                s.panel_drag = Some(PanelDrag { id, before: s.settings.layout.clone(), grab: pressed, size: [w, h], pointer });
            }
            let Some(drag) = s.panel_drag.as_mut() else { return };
            drag.pointer = pointer;
            let (tiles, _) = tile(&s.settings.layout, [0.0, 0.0, aw, ah]);
            let [px, py] = [pointer[0] - ox, pointer[1] - oy];
            let under = tiles.into_iter().find(|t| {
                let [x, y, w, h] = t.rect;
                (x..x + w).contains(&px) && (y..y + h).contains(&py)
            });
            // Over its own place, or a gap: nothing moves. Once swapped, the pointer is over the
            // dragged panel's new place, so the two don't swap back until it leaves.
            if let Some(under) = under.filter(|t| t.id != id)
                && let Some(from) = s.settings.layout.path_of(id)
            {
                s.settings.layout.swap(&from, &under.path);
                tx.animated();
            }
        }
        Phase::Up => {
            if let Some(drag) = s.panel_drag.take() {
                if d.cancelled() {
                    s.settings.layout = drag.before;
                }
                tx.animated();
            }
        }
    }
}

/// The panel's menu: (the 3D view's camera,) split it, change what it shows, close it. Its
/// item keys are rune-kit's: `{key}-menu-itemN`, N counting separators too, and a submenu's
/// `{key}-menu-itemN-sub-itemM`.
fn menu(s: &State, k: &Kit, layer: &mut Vec<El>, kind: PanelKind, key: &str, path: &[usize]) -> El {
    let t = tok::theme(true);
    let open = s.menu_at().filter(|(p, _)| *p == path);
    let sub = open.and_then(|(_, sub)| sub);
    let at = path.to_vec();
    let trigger = el().key(format!("{key}-menu")).role("button").label("Panel menu").focusable(true).size(28.0, 28.0).center()
        .radius(tok::radius::MD).fill(oklcha(0.16, 0.006, 265.0, 0.78)).border(1.0, t.card_border).hover(t.hover).press(t.press)
        .on_press(move |s: &mut State| s.toggle_panel_menu(at.clone()))
        .child(text("⋯").font(type_::BODY).color(t.text_secondary));
    let has_3d = s.settings.layout.has(PanelKind::View3d);
    let kinds = |which: Submenu| -> Vec<MenuEntry> {
        PanelKind::ALL.into_iter().map(|k| {
            let item = match which {
                Submenu::Split(dir) => edit(k.label(), path, move |l, p| l.insert(p, dir, Panel::new(k))).disabled(k == PanelKind::View3d && has_3d),
                Submenu::Show => edit(k.label(), path, move |l, p| l.show(p, k)).checked(k == kind),
            };
            MenuEntry::Item(Box::new(item))
        }).collect()
    };
    let opener = |label: &str, which: Submenu| {
        MenuItem::new(label).submenu(kinds(which), sub == Some(which)).on_select(move |s: &mut State| s.toggle_submenu(which))
    };
    let mut m = Menu::new(format!("{key}-menu")).trigger(trigger).open(open.is_some()).placement(Placement::BottomEnd).min_width(180.0)
        .on_close(|s: &mut State| s.popup = None);
    if kind == PanelKind::View3d {
        m = m.item(act("Reset view", |s| s.camera = s.home_view()))
            .item(act("Top down", |s| s.camera = s.top_view()))
            .separator();
    }
    m.item(opener("Split right", Submenu::Split(Dir::Row)))
        .item(opener("Split down", Submenu::Split(Dir::Column)))
        .item(opener("Show", Submenu::Show))
        .separator()
        // The last panel stays.
        .item(edit("Close", path, Layout::close).danger(true).disabled(path.is_empty()))
        .build(k, layer)
}

/// A menu item that does `f`, then closes the menu.
fn act(label: &str, f: impl Fn(&mut State) + 'static) -> MenuItem {
    MenuItem::new(label).on_select(move |s: &mut State| {
        f(s);
        s.popup = None;
    })
}

/// A menu item that edits the layout at the panel at `path`, then closes the menu.
fn edit(label: &str, path: &[usize], edit: impl Fn(&mut Layout, &[usize]) + 'static) -> MenuItem {
    let at = path.to_vec();
    act(label, move |s| edit(&mut s.settings.layout, &at))
}

/// Six dots: something to grab.
fn grip_glyph() -> El {
    let t = tok::theme(true);
    let dot = || el().size(2.5, 2.5).radius(1.25).fill(t.text_secondary);
    let column = || col().gap(2.5).child(dot()).child(dot()).child(dot());
    row().gap(2.5).child(column()).child(column())
}

/// A panel body that scrolls when its content is taller than the panel.
fn scrolling(key: String) -> El {
    col().key(key).w_full().h_full().min_h(0.0).fill(tok::theme(true).card).scroll(ScrollSpec { stick_bottom: false }).scroll_edge_fade(16.0)
}

/// A panel with nothing to show.
fn empty(message: String) -> El {
    let t = tok::theme(true);
    el().w_full().h_full().center().padding(16.0).fill(t.card).child(text(message).font(type_::LABEL).color(t.text_secondary).max_lines(4))
}

/// The panel's image with its annotations, as large as fits the panel (its topic and size are in
/// its header).
fn image(s: &State, cx: &mut Cx<State>, view: &ImageSettings, key: &str) -> El {
    let Some((_, image)) = s.image(view) else {
        let message = match s.image_topic(view) {
            Some(topic) => match s.error(&topic) {
                Some(e) => format!("{topic}: {e}"),
                None => format!("Waiting for {topic}"),
            },
            None => "No image topic".into(),
        };
        return empty(message);
    };
    // The picture is sized from the panel, once that's laid out.
    let area = format!("{key}-image");
    let ratio = image.width as f32 / image.height as f32;
    let width = cx.layout_size(&area).map(|[w, h]| w.min(h * ratio)).filter(|w| *w >= 1.0);
    el().w_full().h_full().padding(8.0).fill(tok::theme(true).card)
        .child(el().key(area).w_full().h_full().center().children(width.map(|w| ui::picture(s, view, format!("{key}-picture"), image, w))))
}

/// The panel's stats topics' statuses with their readings, then its text topics' newest
/// messages, whole.
fn data(s: &State, view: &DataSettings, key: &str) -> El {
    let t = tok::theme(true);
    if !view.text && !view.stats {
        return empty("Text and stats off".into());
    }
    let statuses = s.stats(view).flat_map(|(_, d)| d);
    let texts = s.texts(view);
    let mut body = col().w_full().padding(12.0).pt(HEADER_H).gap(14.0);
    let mut any = false;
    for d in statuses {
        body = body.child(ui::status_block(d));
        any = true;
    }
    for (topic, message) in texts {
        body = body.child(col().gap(4.0)
            .child(text(topic.clone()).font(MONO_SMALL).color(t.text_secondary).truncate())
            .child(text(message.clone()).font(MONO_SMALL).color(t.text).wrap()));
        any = true;
    }
    if !any {
        return empty("No text or stats topics".into());
    }
    scrolling(format!("{key}-scroll")).child(body)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Each child gets its weight's share of its branch, less the 8 pt gaps; a gap sits after
    // every child but the last.
    #[test]
    fn tiling_shares_each_branch_by_weight() {
        let l: Layout = serde_json::from_str(r#"{"row": [{"panel": "3d", "weight": 2}, {"column": [{"panel": "image"}, {"panel": "data"}]}]}"#).unwrap();
        let (tiles, gaps) = tile(&l, [0.0, 0.0, 308.0, 208.0]);
        let places: Vec<_> = tiles.iter().map(|t| (t.path.clone(), t.rect)).collect();
        assert_eq!(places, [
            (vec![0], [0.0, 0.0, 200.0, 208.0]),
            (vec![1, 0], [208.0, 0.0, 100.0, 100.0]),
            (vec![1, 1], [208.0, 108.0, 100.0, 100.0]),
        ]);
        let gaps: Vec<_> = gaps.iter().map(|g| (g.path.clone(), g.i, g.rect, g.per_weight)).collect();
        assert_eq!(gaps, [(vec![], 0, [200.0, 0.0, 8.0, 208.0], 100.0), (vec![1], 0, [208.0, 100.0, 100.0, 8.0], 100.0)]);
    }
}
