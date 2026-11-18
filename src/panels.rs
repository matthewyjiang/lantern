//! The panels, tiled as `Settings::layout` says: each branch of the layout is a row or column
//! of its children with a draggable divider between neighbours, each leaf a card showing its
//! panel's body, drawn as that panel's settings say. A panel with settings has a button that
//! opens them over it (`ui::panel_settings`). With `State::layout_edit` on, every panel also
//! shows its layout controls: what it shows, split right, split down, close. All of them edit
//! `settings.layout`.
//!
//! A new panel type is a `PanelKind` and `Panel` variant (`layout.rs`) and its body here, in
//! `body`.

use rune::tok::{self, type_};
use rune::{Cursor, Cx, Drag, El, Phase, ScrollSpec, Tx, col, el, oklcha, row, text};

use crate::app::State;
use crate::layout::{Dir, Layout, Panel, PanelKind};
use crate::settings::{DataSettings, ImageSettings};
use crate::ui::{self, MONO_SMALL};

/// The gap between two panels, which is also the divider to drag.
const GAP: f32 = 8.0;
/// A divider stops where either child beside it would get shorter or narrower than this, points:
/// in a panel that size the layout controls' widest button (Split down, 84 pt) still fits with
/// their card's padding and margin (36 pt). Panels nested in that child, and panels made by
/// splitting, can be smaller.
const MIN_PANEL: f32 = 120.0;

/// The layout, filling the window beside the side panel.
pub fn layout(s: &State, cx: &mut Cx<State>) -> El {
    let root = node(s, cx, &s.settings.layout, &mut Vec::new());
    // Like every node, the root may be smaller than its content (an image as wide as it was
    // before the Settings page opened).
    el().grow(1.0).h_full().min_w(0.0).padding(10.0).pr(0.0).child(root.grow(1.0).h_full().min_w(0.0).min_h(0.0))
}

/// `panel` for the root, then `-i` for each step of the path: `panel-1-0`.
pub(crate) fn key_of(path: &[usize]) -> String {
    path.iter().fold("panel".to_string(), |key, i| format!("{key}-{i}"))
}

/// The layout node at `path`, unsized: the caller gives it its share of the parent.
fn node(s: &State, cx: &mut Cx<State>, layout: &Layout, path: &mut Vec<usize>) -> El {
    let (dir, children) = match layout {
        Layout::Panel(p) => return panel(s, cx, p, path),
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
        let child = node(s, cx, child, path).grow(share).min_w(0.0).min_h(0.0);
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

/// One panel: a card with its body, under the layout controls while editing.
fn panel(s: &State, cx: &mut Cx<State>, p: &Panel, path: &[usize]) -> El {
    let key = key_of(path);
    let card = el().key(key.clone()).clip_rounded(tok::radius::LG).border(1.0, oklcha(1.0, 0.0, 0.0, 0.07)).child(body(s, cx, p, &key, path));
    if !s.layout_edit {
        return card;
    }
    let width = cx.layout_size(&key).map_or(f32::INFINITY, |[w, _]| w);
    card.child(controls(p.kind(), path, &key, width))
}

/// What panel `p` at `path` shows, filling its card. `key` is the panel's, to prefix the keys of
/// what's inside (a kind can be in several panels).
fn body(s: &State, cx: &mut Cx<State>, p: &Panel, key: &str, path: &[usize]) -> El {
    let gear = || ui::panel_settings_button(s, key, path);
    match p {
        Panel::View3d(_) => ui::view3d(s, cx, gear()),
        Panel::Image(view) => image(s, cx, view, key, gear()),
        Panel::Data(view) => data(s, view, key, gear()),
        Panel::Topics => scrolling(format!("{key}-scroll")).child(el().w_full().padding(12.0).child(ui::topics(s, &format!("{key}-topic")))),
    }
}

/// The layout controls over a panel: what it shows, and the edits that start from it.
fn controls(kind: PanelKind, path: &[usize], key: &str, panel_w: f32) -> El {
    // The card and its rows are no wider than the panel, so the rows wrap in a narrow one.
    let (margin, padding) = (8.0, 10.0);
    let fit = |row: El| row.gap(6.0).flex_wrap().justify_center().max_w((panel_w - 2.0 * (margin + padding)).max(0.0));
    let mut kinds = fit(row().role("radiogroup"));
    for k in PanelKind::ALL {
        let path = path.to_vec();
        kinds = kinds.child(ui::chip(format!("{key}-show-{}", k.id()), k.label(), k == kind, move |s| s.settings.layout.show(&path, k)));
    }
    let edit = |name: &str, label: &'static str, edit: fn(&mut Layout, &[usize])| {
        let path = path.to_vec();
        ui::fitted_button(format!("{key}-{name}"), label, move |s| edit(&mut s.settings.layout, &path)).h(28.0).px(12.0)
    };
    let actions = fit(row())
        .child(edit("split-right", "Split right", |l, p| l.split(p, Dir::Row)))
        .child(edit("split-down", "Split down", |l, p| l.split(p, Dir::Column)))
        // The last panel stays.
        .children((!path.is_empty()).then(|| edit("close", "Close", Layout::close)));
    el().pin(Some(0.0), Some(0.0), Some(0.0), Some(0.0)).center().fill(oklcha(0.12, 0.006, 265.0, 0.55))
        .child(ui::glass().padding(padding).child(col().gap(8.0).items_center().child(kinds).child(actions)))
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

/// A panel body with its settings button over its top right corner.
fn cornered(body: El, gear: El) -> El {
    el().w_full().h_full().child(body).child(gear.pin(None, Some(8.0), Some(8.0), None))
}

/// The panel's image with its annotations, as large as fits the panel, under its topic and size.
fn image(s: &State, cx: &mut Cx<State>, view: &ImageSettings, key: &str, gear: El) -> El {
    let Some((topic, image)) = s.image(view) else {
        let message = match s.image_topic(view) {
            Some(topic) => match s.error(&topic) {
                Some(e) => format!("{topic}: {e}"),
                None => format!("Waiting for {topic}"),
            },
            None => "No image topic yet".into(),
        };
        return cornered(empty(message), gear);
    };
    // The picture is sized from the space under the caption, once that's laid out.
    let area = format!("{key}-image");
    let ratio = image.width as f32 / image.height as f32;
    let width = cx.layout_size(&area).map(|[w, h]| w.min(h * ratio)).filter(|w| *w >= 1.0);
    let caption = row().items_center().gap(6.0).child(ui::image_caption(topic, image).grow(1.0).min_w(0.0)).child(gear);
    col().w_full().h_full().padding(8.0).gap(5.0).fill(tok::theme(true).card)
        .child(caption)
        .child(el().key(area).grow(1.0).min_h(0.0).w_full().center().children(width.map(|w| ui::picture(s, view, format!("{key}-picture"), image, w))))
}

/// The panel's stats topics' statuses with their readings, then its text topics' newest
/// messages, whole.
fn data(s: &State, view: &DataSettings, key: &str, gear: El) -> El {
    let t = tok::theme(true);
    if !view.text && !view.stats {
        return cornered(empty("Text and stats are both off in this panel's settings.".into()), gear);
    }
    let statuses = s.stats(view).flat_map(|(_, d)| d);
    let texts = s.texts(view);
    let mut body = col().w_full().padding(12.0).gap(14.0);
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
        return cornered(empty("No text or stats topics yet".into()), gear);
    }
    // The button sits in the scroll area's corner; the body leaves it room on the first line.
    cornered(scrolling(format!("{key}-scroll")).child(body.pr(44.0)), gear)
}
