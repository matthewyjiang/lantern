//! The panels, tiled as `Settings::layout` says: each branch of the layout is a row or column
//! of its children with a draggable divider between neighbours, each leaf a card showing its
//! panel's body, drawn as that panel's settings say.
//!
//! Over a panel the pointer is on, its header: its name (a handle to drag the panel elsewhere,
//! onto another's side to move it there or its middle to swap the two), its settings button
//! (`ui::panel_settings`) and its menu (split, change what it shows, close). All of them edit
//! `settings.layout`.
//!
//! A new panel type is a `PanelKind` and `Panel` variant (`layout.rs`) and its body here, in
//! `body`.

use std::rc::Rc;

use rune::tok::{self, type_};
use rune::{Cursor, Cx, Drag, El, Phase, ScrollSpec, Tx, col, el, oklcha, row, text};
use rune_kit::{Kit, Menu, MenuEntry, MenuItem, Placement};

use crate::app::{PanelDrag, State, Submenu};
use crate::layout::{Dir, Edge, Layout, Panel, PanelKind};
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

/// Every panel's place in the window, by its path, as last laid out: where a dragged panel lands.
type Places = Rc<Vec<(Vec<usize>, [f32; 4])>>;

/// The layout, filling the window under the top bar. Menus go to `layer`.
pub fn layout(s: &State, cx: &mut Cx<State>, k: &Kit, layer: &mut Vec<El>) -> El {
    let places: Places = Rc::new(s.settings.layout.paths().into_iter().filter_map(|p| Some((cx.window_layout(&key_of(&p))?, p))).map(|(r, p)| (p, r)).collect());
    let root = node(s, cx, k, layer, &places, &s.settings.layout, &mut Vec::new());
    // Like every node, the root may be smaller than its content (an image as wide as it was
    // before the Settings page opened).
    el().grow(1.0).h_full().min_w(0.0).padding(10.0).pt(0.0).child(root.grow(1.0).h_full().min_w(0.0).min_h(0.0))
}

/// `panel` for the root, then `-i` for each step of the path: `panel-1-0`.
pub(crate) fn key_of(path: &[usize]) -> String {
    path.iter().fold("panel".to_string(), |key, i| format!("{key}-{i}"))
}

/// The layout node at `path`, unsized: the caller gives it its share of the parent.
fn node(s: &State, cx: &mut Cx<State>, k: &Kit, layer: &mut Vec<El>, places: &Places, layout: &Layout, path: &mut Vec<usize>) -> El {
    let (dir, children) = match layout {
        Layout::Panel(p) => return panel(s, cx, k, layer, places, p, path),
        Layout::Split { dir, children } => (*dir, children),
    };
    let key = key_of(path);
    // Points per unit of weight along the branch, for the dividers: known once it's laid out.
    let length = cx.layout_size(&key).map(|[w, h]| if dir == Dir::Row { w } else { h });
    let total: f32 = children.iter().map(|(w, _)| w).sum();
    let per_weight = length.map(|l| (l - GAP * (children.len() - 1) as f32) / total).filter(|p| *p > 0.0);
    let mut branch = match dir {
        Dir::Row => row(),
        Dir::Column => col(),
    };
    for (i, (weight, child)) in children.iter().enumerate() {
        if i > 0 {
            branch = branch.child(divider(dir, format!("{key}-divider-{}", i - 1), path.clone(), i - 1, children[i - 1].0, per_weight));
        }
        path.push(i);
        // Shares of the whole branch: flex factors that sum to under 1 would leave part of it
        // empty, and weights can (0.5 and 0.25 in a file, or what's left after a close).
        let share = weight / total * children.len() as f32;
        let child = node(s, cx, k, layer, places, child, path).grow(share).min_w(0.0).min_h(0.0);
        path.pop();
        branch = branch.child(if dir == Dir::Row { child.h_full() } else { child.w_full() });
    }
    branch.key(key)
}

/// The gap after child `i` of the branch at `path`, whose weight is `weight`: dragging it moves
/// space between the two children it separates.
fn divider(dir: Dir, key: String, path: Vec<usize>, i: usize, weight: f32, per_weight: Option<f32>) -> El {
    let t = tok::theme(true);
    let (handle, grip) = match dir {
        Dir::Row => (el().w(GAP).h_full().cursor(Cursor::ResizeColumn), el().size(2.0, 28.0)),
        Dir::Column => (el().h(GAP).w_full().cursor(Cursor::ResizeRow), el().size(28.0, 2.0)),
    };
    handle.key(key).center().radius(GAP / 2.0).hover(t.hover)
        .child(grip.radius(1.0).fill(t.separator))
        .on_drag(move |s: &mut State, d: &mut Drag, _: &mut Tx| {
            let Some(per_weight) = per_weight else { return };
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

/// One panel: a card with its body and, while the pointer is over it, its header. While a
/// panel is dragged, the one it would land on shows where.
fn panel(s: &State, cx: &mut Cx<State>, k: &Kit, layer: &mut Vec<El>, places: &Places, p: &Panel, path: &[usize]) -> El {
    let key = key_of(path);
    let dragged = s.panel_drag.as_ref().is_some_and(|d| d.from == path);
    let drop = s.panel_drag.as_ref().and_then(|d| d.over.as_ref()).filter(|(to, _)| to == path).map(|(_, edge)| *edge);
    let size = cx.layout_size(&key);
    el().key(key.clone()).hover_group().clip_rounded(tok::radius::LG).border(1.0, oklcha(1.0, 0.0, 0.0, 0.07))
        .opacity(if dragged { 0.45 } else { 1.0 })
        .child(body(s, cx, p, &key))
        .child(header(s, k, layer, places, p, &key, path))
        .children(drop.map(|edge| drop_zone(edge, size)))
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
/// is over the panel, and while its menu or settings are open.
fn header(s: &State, k: &Kit, layer: &mut Vec<El>, places: &Places, p: &Panel, key: &str, path: &[usize]) -> El {
    let t = tok::theme(true);
    let title = match p {
        Panel::Image(view) => match s.image(view) {
            Some((topic, image)) => format!("{topic} · {}×{}", image.width, image.height),
            None => "Image".into(),
        },
        _ => p.kind().label().into(),
    };
    let from = path.to_vec();
    let places = places.clone();
    let handle = ui::glass().key(format!("{key}-handle")).role("button").label(format!("Move {title}")).h(28.0).px(10.0).gap(8.0).items_center().min_w(0.0)
        .cursor(if s.panel_drag.is_some() { Cursor::Grabbing } else { Cursor::Grab })
        .child(grip_glyph())
        .child(text(title).font(type_::LABEL).color(t.text).truncate().min_w(0.0))
        .on_drag(move |s: &mut State, d: &mut Drag, _: &mut Tx| drag_panel(s, d, &from, &places));
    let open = s.panel_menu.as_ref().is_some_and(|(p, _)| p == path) || s.panel_settings.as_deref() == Some(path);
    let buttons = row().gap(6.0).items_center()
        .children(has_settings(p).then(|| ui::panel_settings_button(s, key, path)))
        .child(menu(s, k, layer, p.kind(), key, path));
    let bar = row().gap(8.0).items_center().justify_between()
        .pin(Some(8.0), Some(8.0), Some(8.0), None)
        .child(handle.shrink())
        .child(buttons);
    // Kept on while something it opened is open, or a panel is being dragged.
    if open || s.panel_drag.is_some() { bar } else { bar.group_opacity(0.0, 1.0) }
}

fn has_settings(p: &Panel) -> bool {
    !matches!(p, Panel::Topics)
}

/// A press on a panel's handle: past `DRAG_START`, the panel follows the pointer to the panel
/// under it (`places`), and drops there, onto its side or, in its middle, in its place.
fn drag_panel(s: &mut State, d: &mut Drag, from: &[usize], places: &Places) {
    match d.phase {
        Phase::Down => {
            s.panel_menu = None;
            s.panel_settings = None;
        }
        Phase::Move => {
            if s.panel_drag.is_none() && d.dx.hypot(d.dy) < DRAG_START {
                return;
            }
            let over = landing(places, from, d.window_point());
            s.panel_drag = Some(PanelDrag { from: from.to_vec(), over });
        }
        Phase::Up => {
            let Some(drag) = s.panel_drag.take() else { return };
            if d.cancelled() {
                return;
            }
            match drag.over {
                Some((to, None)) => s.settings.layout.swap(from, &to),
                Some((to, Some(edge))) => s.settings.layout.move_panel(from, &to, edge),
                None => {}
            }
        }
    }
}

/// Where a panel dragged from `from` lands with the pointer at `at`: the panel there and the side
/// of it the pointer is nearest, or its middle (the inner 40% each way) to swap. Not on itself.
fn landing(places: &Places, from: &[usize], [x, y]: [f32; 2]) -> Option<(Vec<usize>, Option<Edge>)> {
    let (path, [px, py, w, h]) = places.iter().find(|(_, [px, py, w, h])| (*px..px + w).contains(&x) && (*py..py + h).contains(&y))?;
    if path == from {
        return None;
    }
    let (u, v) = ((x - px) / w, (y - py) / h);
    let middle = 0.3..0.7;
    if middle.contains(&u) && middle.contains(&v) {
        return Some((path.clone(), None));
    }
    let sides = [(u, Edge::Left), (1.0 - u, Edge::Right), (v, Edge::Top), (1.0 - v, Edge::Bottom)];
    let edge = sides.into_iter().min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, e)| e);
    Some((path.clone(), edge))
}

/// Where a dropped panel would go in this one (of `size`): the half on `edge`, or all of it to swap.
fn drop_zone(edge: Option<Edge>, size: Option<[f32; 2]>) -> El {
    let accent = tok::accent(0, true);
    let [w, h] = size.unwrap_or([0.0, 0.0]);
    let zone = el().fill(accent.alpha(0.18)).border(2.0, accent.alpha(0.8)).radius(tok::radius::LG);
    match edge {
        None => zone.pin(Some(0.0), Some(0.0), Some(0.0), Some(0.0)),
        Some(Edge::Left) => zone.pin(Some(0.0), Some(0.0), None, Some(0.0)).w(w / 2.0),
        Some(Edge::Right) => zone.pin(None, Some(0.0), Some(0.0), Some(0.0)).w(w / 2.0),
        Some(Edge::Top) => zone.pin(Some(0.0), Some(0.0), Some(0.0), None).h(h / 2.0),
        Some(Edge::Bottom) => zone.pin(Some(0.0), None, Some(0.0), Some(0.0)).h(h / 2.0),
    }
}

/// The panel's menu: (the 3D view's camera,) split it, change what it shows, close it.
fn menu(s: &State, k: &Kit, layer: &mut Vec<El>, kind: PanelKind, key: &str, path: &[usize]) -> El {
    let t = tok::theme(true);
    let open = s.panel_menu.as_ref().filter(|(p, _)| p == path);
    let sub = open.and_then(|(_, sub)| *sub);
    let at = path.to_vec();
    let trigger = el().key(format!("{key}-menu")).role("button").label("Panel menu").focusable(true).size(28.0, 28.0).center()
        .radius(tok::radius::MD).fill(oklcha(0.16, 0.006, 265.0, 0.78)).border(1.0, t.card_border).hover(t.hover).press(t.press)
        .on_press(move |s: &mut State| {
            s.panel_settings = None;
            s.panel_menu = if s.panel_menu.as_ref().is_some_and(|(p, _)| *p == at) { None } else { Some((at.clone(), None)) };
        })
        .child(text("⋯").font(type_::BODY).color(t.text_secondary));
    let has_3d = s.settings.layout.has(PanelKind::View3d);
    let kinds = |which: Submenu| -> Vec<MenuEntry> {
        PanelKind::ALL.into_iter().map(|k| {
            let item = match which {
                Submenu::SplitRight => act(k.label(), path, move |l, p| l.insert(p, Edge::Right, Panel::new(k))).disabled(k == PanelKind::View3d && has_3d),
                Submenu::SplitDown => act(k.label(), path, move |l, p| l.insert(p, Edge::Bottom, Panel::new(k))).disabled(k == PanelKind::View3d && has_3d),
                Submenu::Show => act(k.label(), path, move |l, p| l.show(p, k)).checked(k == kind),
            };
            MenuEntry::Item(Box::new(item))
        }).collect()
    };
    let opener = |label: &str, which: Submenu| {
        let at = path.to_vec();
        MenuItem::new(label).submenu(kinds(which), sub == Some(which)).on_select(move |s: &mut State| {
            let open = s.panel_menu.as_ref().and_then(|(_, sub)| *sub);
            s.panel_menu = Some((at.clone(), if open == Some(which) { None } else { Some(which) }));
        })
    };
    let mut m = Menu::new(format!("{key}-menu")).trigger(trigger).open(open.is_some()).placement(Placement::BottomEnd).min_width(180.0)
        .on_close(|s: &mut State| s.panel_menu = None);
    if kind == PanelKind::View3d {
        m = m.item(MenuItem::new("Reset view").on_select(|s: &mut State| {
            s.camera = s.home_view();
            s.panel_menu = None;
        }))
        .item(MenuItem::new("Top down").on_select(|s: &mut State| {
            s.camera = s.top_view();
            s.panel_menu = None;
        }))
        .separator();
    }
    m.item(opener("Split right", Submenu::SplitRight))
        .item(opener("Split down", Submenu::SplitDown))
        .item(opener("Show", Submenu::Show))
        .separator()
        // The last panel stays.
        .item(act("Close", path, Layout::close).danger(true).disabled(path.is_empty()))
        .build(k, layer)
}

/// A menu item that edits the layout at the panel at `path`, then closes the menu.
fn act(label: &str, path: &[usize], edit: impl Fn(&mut Layout, &[usize]) + 'static) -> MenuItem {
    let at = path.to_vec();
    MenuItem::new(label).on_select(move |s: &mut State| {
        edit(&mut s.settings.layout, &at);
        s.panel_menu = None;
    })
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

/// A panel with nothing to show yet says why.
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
            None => "No image topic yet".into(),
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
        return empty("Text and stats are both off in this panel's settings.".into());
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
        return empty("No text or stats topics yet".into());
    }
    scrolling(format!("{key}-scroll")).child(body)
}
