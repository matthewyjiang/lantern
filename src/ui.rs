//! The window: the panels the layout tiles (`panels.rs`), and on the right either the side panel
//! (the 3D view's quick controls) or the Settings page (what's app-wide: connections, topics,
//! layout). Over them, the open panel's settings (`panel_settings`). The 3D view is here with its
//! overlays; so are the pieces other panels share with them (the annotated image, a status
//! block, the topic list). Every control edits `State::settings` or the camera; `State::sync`
//! applies the edit.

use std::collections::BTreeSet;
use std::rc::Rc;

use rune::tok::{self, TextStyle, motion, type_};
use rune::color::srgb_decode;
use rune::{Align, Color, Cx, Drag, El, Phase, PointerButton, ScrollSpec, Side, Tx, WheelEvent, col, effect, el, icon, oklch, oklcha, row, text, text_input};

use crate::app::State;
use crate::connection::Connection;
use crate::decode::{ColorChoice, Kind};
use crate::layout::{Panel, PanelKind, Preset, PRESETS};
use crate::panels;
use crate::scene::{Annotation, Diagnostic, Image, Level, PointColor};
use crate::settings::{DataSettings, ImageSettings, View3dSettings};
use crate::source::Status;
use crate::viewport::{self, Shade};

const SIDEBAR_W: f32 = 296.0;
const SETTINGS_W: f32 = 440.0;
/// A panel's settings, over the window beside its settings button.
const PANEL_SETTINGS_W: f32 = 380.0;
/// The Settings page's side margin. Inside its scroll area, so the wheel scrolls the page over
/// the margins too.
const SETTINGS_PAD: f32 = 22.0;
/// Text and stats cards over the viewport.
const CARD_W: f32 = 340.0;
/// The camera image card's widest: a 640 px frame at 3/4, so it stays an inset on a large window.
const IMAGE_MAX_W: f32 = 480.0;
/// Lines of a text message shown before "…".
const TEXT_LINES: usize = 10;
/// Text cards and stats statuses shown over the viewport before "and N more": together about the
/// height of the viewport beside Settings in a 1280×800 window. Settings → Topics hides topics.
const TEXT_CARDS: usize = 3;
const STATUSES: usize = 6;

/// Status colours: fine, waiting, failed.
fn good() -> Color {
    oklch(0.78, 0.17, 150.0)
}
fn waiting() -> Color {
    oklch(0.78, 0.15, 70.0)
}
fn failed() -> Color {
    oklch(0.66, 0.2, 25.0)
}
pub(crate) const SMALL: TextStyle = TextStyle::new(11.0, 500.0);
/// Topic, frame and URL text: names people type and match character for character.
pub(crate) const MONO: TextStyle = TextStyle::new(12.0, 500.0).mono();
pub(crate) const MONO_SMALL: TextStyle = TextStyle::new(11.0, 500.0).mono();

pub fn view(s: &State, cx: &mut Cx<State>) -> El {
    let t = tok::theme(true);
    let side = if s.settings_open { settings_page(s) } else { sidebar(s) };
    let window = row().w_full().h_full().fill(t.bg).child(panels::layout(s, cx)).child(side);
    // The open panel's settings, after the window so they draw over it and escape its panel's
    // clipping.
    let open = s.panel_settings.as_ref().and_then(|path| Some((path, s.settings.layout.panel(path)?)));
    el().w_full().h_full().child(window).children(open.map(|(path, p)| panel_settings(s, path, p)))
}

// ---- 3D view ----

/// The 3D panel: the viewport, with frame labels and its overlays, and `gear` (its settings
/// button) at the top right. The camera image, text and stats sit over it as cards unless the
/// layout has a panel for them (`State::image_card`, `State::data_cards`).
pub(crate) fn view3d(s: &State, cx: &mut Cx<State>, gear: El) -> El {
    let stage = effect(viewport::MARK)
        .key("viewport")
        .w_full()
        .h_full()
        .pointer_buttons([PointerButton::Middle])
        .on_drag(drag_camera)
        .on_wheel(|s: &mut State, e: WheelEvent, _: &mut Tx| {
            s.camera.zoom((-e.dy * 0.004).exp());
            true
        });
    let mut card = el().w_full().h_full().child(stage);
    let size = cx.layout_size("viewport");
    if let Some(size) = size {
        for label in frame_labels(s, size) {
            card = card.child(label);
        }
    }
    let mut right = col().key("overlay-right").gap(8.0).items_end().child(row().gap(8.0).items_center().child(hints()).child(gear));
    if let Some(cards) = s.data_cards() {
        right = right.children(text_cards(s, cards)).children(stats_card(s, cards));
    }
    // The status chip wraps short of the right column (hints, text and stats cards).
    let chip_w = match (size, cx.layout_size("overlay-right")) {
        (Some([vw, _]), Some([rw, _])) => (vw - rw - 3.0 * 14.0).clamp(200.0, 520.0),
        _ => 520.0,
    };
    card.child(col().gap(8.0).items_start().child(point_badge(s)).children(status_chip(s, chip_w)).pin(Some(14.0), Some(14.0), None, None))
        .child(right.pin(None, Some(14.0), Some(14.0), None))
        .children(legend(s).map(|l| l.pin(Some(14.0), None, None, Some(14.0))))
        .children(size.zip(s.image_card()).and_then(|(size, view)| image_card(s, view, size)).map(|c| c.pin(None, None, Some(14.0), Some(14.0))))
}

/// Orbit with the primary button; pan with the middle button or Shift.
fn drag_camera(s: &mut State, d: &mut Drag, tx: &mut Tx) {
    match d.phase {
        Phase::Down => s.drag_seen = (0.0, 0.0),
        Phase::Move | Phase::Up => {
            let (dx, dy) = (d.dx - s.drag_seen.0, d.dy - s.drag_seen.1);
            s.drag_seen = (d.dx, d.dy);
            if tx.button() == PointerButton::Middle || tx.mods().shift {
                s.camera.pan(dx, dy, d.h);
            } else {
                s.camera.orbit(dx, dy);
            }
        }
    }
}

/// Frame and object names, placed where their origins project into the viewport.
fn frame_labels(s: &State, [w, h]: [f32; 2]) -> Vec<El> {
    let t = tok::theme(true);
    let vp = s.camera.view_proj(w / h.max(1.0));
    s.display.labels.iter().filter_map(|(name, p)| {
        let clip = vp * p.extend(1.0);
        if clip.w <= 0.0 {
            return None;
        }
        let (x, y) = ((clip.x / clip.w * 0.5 + 0.5) * w, (0.5 - clip.y / clip.w * 0.5) * h);
        if !(0.0..w - 40.0).contains(&x) || !(10.0..h - 10.0).contains(&y) {
            return None;
        }
        Some(
            el().absolute(x + 6.0, y - 18.0).px(5.0).py(1.0).radius(4.0).fill(oklcha(0.16, 0.006, 265.0, 0.7))
                .child(text(name.clone()).font(MONO_SMALL).color(t.text_secondary)),
        )
    }).collect()
}

/// Translucent dark glass for overlays on the viewport.
pub(crate) fn glass() -> El {
    el().fill(oklcha(0.16, 0.006, 265.0, 0.78)).radius(tok::radius::MD).border(1.0, oklcha(1.0, 0.0, 0.0, 0.06))
}

/// How many points are drawn, so Density and the Point cloud toggle read back.
fn point_badge(s: &State) -> El {
    let t = tok::theme(true);
    let label = match s.cloud() {
        None => "No point cloud".to_string(),
        Some((topic, c)) => {
            let (total, drawn) = (c.points.len(), s.drawn_points());
            if drawn == total {
                format!("{topic} · {} points", thousands(total))
            } else {
                format!("{topic} · {} of {} points", thousands(drawn), thousands(total))
            }
        }
    };
    glass().px(10.0).py(7.0).child(text(label).key("point-count").font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary))
}

/// Says why the view is empty or incomplete, at most `max_w` wide; absent when everything draws.
fn status_chip(s: &State, max_w: f32) -> Option<El> {
    // One line each for the connections, the robot (a URDF that didn't load, or one that isn't
    // the robot the server describes) and what the data is missing, so a server that's offline
    // doesn't hide what's wrong with the others' data.
    let conns = s.connections();
    let connection = if conns.is_empty() {
        Some("No servers: add one in Settings".to_string())
    } else {
        conns.iter().find_map(|c| match &c.received.status {
            Status::Failed { error } => Some(format!("Can't reach {}: {error}", c.url)),
            Status::Connecting => Some(format!("Connecting to {}…", c.url)),
            Status::Connected { .. } => None,
        })
    };
    let robot = match (s.robot_file(), s.robot_description(), s.robot_mismatch()) {
        (_, _, Some(m)) => Some(format!("Robot: {m}")),
        (Some(Err(e)), _, _) => Some(format!("Robot: {e}")),
        (_, Some((topic, Err(e))), _) => Some(format!("Robot from {topic}: {e}")),
        _ => None,
    };
    let warning = conns.iter().find_map(|c| c.received.server_warning.as_ref());
    let data = match (&s.display.problem, warning) {
        (Some(p), _) => Some(p.clone()),
        (None, Some(w)) => Some(format!("Server: {w}")),
        (None, None) if conns.iter().all(|c| c.received.channels.is_empty()) => Some("Connected; the server offers no topics yet".into()),
        _ => None,
    };
    let messages: Vec<String> = [connection, robot, data].into_iter().flatten().collect();
    if messages.is_empty() {
        return None;
    }
    let mut lines = col().key("status-chip").gap(4.0);
    for m in messages {
        lines = lines.child(
            row().gap(8.0).items_center()
                .child(el().size(7.0, 7.0).radius(3.5).fill(waiting()))
                // The dot, its gap and the padding take 35 pt.
                .child(text(m).font(type_::LABEL).color(tok::theme(true).text).max_w(max_w - 35.0).max_lines(3)),
        );
    }
    Some(glass().px(10.0).py(7.0).child(lines))
}

fn hints() -> El {
    let t = tok::theme(true);
    let key = |k: &str| el().px(6.0).py(2.0).radius(4.0).fill(oklcha(1.0, 0.0, 0.0, 0.09))
        .child(text(k).font(type_::MONO_CAPTION).color(t.text));
    let pair = |k: &str, what: &str| row().items_center().gap(6.0).child(key(k)).child(text(what).font(type_::LABEL).color(t.text_secondary));
    glass().px(10.0).py(7.0).child(
        row().items_center().gap(14.0)
            .child(pair("Drag", "orbit"))
            .child(pair("Shift-drag", "pan"))
            .child(pair("Scroll", "zoom")),
    )
}

/// The colour ramp the points use, with its ends labelled; absent for per-point colour.
fn legend(s: &State) -> Option<El> {
    let t = tok::theme(true);
    s.view().cloud?;
    // Turbo, sampled at the shader's stops, so the bar matches the points.
    let turbo: [(f32, &str); 11] = [
        (0.0, "#30123b"), (0.1, "#4662d7"), (0.2, "#36aaf9"), (0.3, "#1ae4b6"), (0.4, "#72fe5e"),
        (0.5, "#c8ef34"), (0.6, "#faba39"), (0.7, "#f66b19"), (0.8, "#ca2a04"), (0.9, "#930c01"), (1.0, "#7a0403"),
    ];
    let field = match s.cloud().map(|(_, c)| &c.color) {
        Some(PointColor::Field { name, .. }) => name.clone(),
        _ => String::new(),
    };
    let (label, lo, hi, span) = match s.display.shade {
        Shade::Rgb => return None,
        Shade::Height { lo, hi } => (format!("Height in {}", s.display.fixed.as_deref().unwrap_or("?")), metres(lo), metres(hi), (0.12, 0.95)),
        Shade::Field { lo, hi } => (field, compact(lo), compact(hi), (0.08, 0.92)),
    };
    let stops: Vec<(f32, Color)> = turbo.iter()
        .filter(|(at, _)| *at >= span.0 - 0.1 && *at <= span.1 + 0.1)
        .map(|(at, hex)| (((at - span.0) / (span.1 - span.0)).clamp(0.0, 1.0), rune::color::hex(hex).unwrap()))
        .collect();
    let bar = el().w(168.0).h(8.0).radius(4.0).linear_gradient(std::f32::consts::FRAC_PI_2, stops);
    let ends = row().w(168.0).justify_between()
        .child(text(lo).font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary))
        .child(text(hi).font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary));
    Some(glass().px(12.0).py(9.0).child(
        col().gap(6.0).child(text(label).key("legend-label").font(type_::LABEL).color(t.text)).child(bar).child(ends),
    ))
}

fn metres(v: f32) -> String {
    if v.abs() < 10.0 { format!("{v:.2} m") } else { format!("{v:.0} m") }
}

fn compact(v: f32) -> String {
    if v.abs() >= 100.0 || v == v.round() { format!("{v:.0}") } else { format!("{v:.2}") }
}

/// An annotation colour (sRGB 0–1, as Foxglove gives it) as a UI colour.
fn srgb([r, g, b, a]: [f32; 4]) -> Color {
    Color { r: srgb_decode(r), g: srgb_decode(g), b: srgb_decode(b), a }
}

/// The card's image with its annotations, in the viewport's lower right: up to 40% of the
/// viewport's width, `IMAGE_MAX_W`, and 45% of its height, at the image's aspect ratio.
fn image_card(s: &State, view: &ImageSettings, [vw, vh]: [f32; 2]) -> Option<El> {
    let (topic, image) = s.image(view)?;
    let (iw, ih) = (image.width as f32, image.height as f32);
    let w = (vw * 0.4).min(IMAGE_MAX_W).min(vh * 0.45 * iw / ih);
    Some(glass().padding(6.0).child(col().gap(5.0).child(image_caption(topic, image)).child(picture(s, view, "image-card", image, w))))
}

/// An image's topic and size, on one line.
pub(crate) fn image_caption(topic: &str, image: &Image) -> El {
    let t = tok::theme(true);
    row().px(2.0).gap(8.0).justify_between()
        .child(text(topic.to_string()).font(MONO_SMALL).color(t.text_secondary).truncate())
        .child(text(format!("{}×{}", image.width, image.height)).font(MONO_SMALL.features("tnum")).color(t.text_secondary))
}

/// A view's image `w` points wide, its annotations drawn in, with its text annotations set over
/// it at their image positions (Foxglove's: the text's lower left).
pub(crate) fn picture(s: &State, view: &ImageSettings, key: impl Into<String>, image: &Image, w: f32) -> El {
    let scale = w / image.width as f32;
    let mut picture = el().key(key).size(w, image.height as f32 * scale).radius(tok::radius::SM);
    if let Some(pixels) = s.pixels(view) {
        picture = picture.pixels(pixels);
    }
    for a in s.annotations(&view.hidden) {
        let Annotation::Text { at, text: label, size, color, background } = a else { continue };
        // 9 pt at the least, to stay legible on a small card; the chip's top is a line (Rune's
        // default 1.25 × size) above the text's lower left.
        let px = (size * scale).max(9.0);
        let mut chip = el().absolute(at[0] * scale, at[1] * scale - px * 1.25).px(2.0)
            .child(text(label.clone()).font(TextStyle::new(px, 500.0)).color(srgb(*color)));
        if background[3] > 0.0 {
            chip = chip.fill(srgb(*background)).radius(2.0);
        }
        picture = picture.child(chip);
    }
    picture
}

/// "and N more …", when a card shows only the first of its items.
fn more(n: usize, what: &str) -> Option<El> {
    (n > 0).then(|| text(format!("and {n} more {what}: hide topics in the 3D view's settings")).font(SMALL).color(tok::theme(true).text_secondary))
}

/// The newest message of the cards' first `TEXT_CARDS` text topics.
fn text_cards(s: &State, view: &DataSettings) -> Vec<El> {
    let t = tok::theme(true);
    let texts: Vec<_> = s.texts(view).collect();
    let mut cards: Vec<El> = texts.iter().take(TEXT_CARDS).map(|(topic, body)| {
        glass().key(format!("text-{topic}")).w(CARD_W).px(10.0).py(8.0).child(
            col().gap(4.0)
                .child(text(topic.to_string()).font(MONO_SMALL).color(t.text_secondary).truncate())
                .child(text(body.to_string()).font(MONO_SMALL).color(t.text).max_lines(TEXT_LINES)),
        )
    }).collect();
    cards.extend(more(texts.len().saturating_sub(TEXT_CARDS), "text topics").map(|m| glass().w(CARD_W).px(10.0).py(6.0).child(m)));
    cards
}

/// The first `STATUSES` of the cards' stats topics' statuses: level, name and message, then each
/// reading.
fn stats_card(s: &State, view: &DataSettings) -> Option<El> {
    let statuses: Vec<_> = s.stats(view).flat_map(|(_, d)| d).collect();
    if statuses.is_empty() {
        return None;
    }
    let body = col().gap(8.0)
        .children(statuses.iter().take(STATUSES).map(|d| status_block(d)))
        .children(more(statuses.len().saturating_sub(STATUSES), "statuses"));
    Some(glass().key("stats-card").w(CARD_W).px(10.0).py(8.0).child(body))
}

/// One status: its level, name and message, then each reading.
pub(crate) fn status_block(d: &Diagnostic) -> El {
    let t = tok::theme(true);
    let hue = match d.level {
        Level::Ok => good(),
        Level::Warn => waiting(),
        Level::Error => failed(),
        Level::Stale => t.text_secondary,
    };
    let mut block = col().gap(3.0).child(
        row().gap(7.0).items_center()
            .child(el().size(7.0, 7.0).radius(3.5).fill(hue))
            .child(text(d.name.clone()).font(type_::LABEL).color(t.text))
            .child(text(d.message.clone()).font(type_::LABEL).color(t.text_secondary).truncate().grow(1.0).min_w(0.0)),
    );
    for (k, v) in &d.values {
        block = block.child(row().pl(14.0).gap(8.0).justify_between()
            .child(text(k.clone()).font(SMALL).color(t.text_secondary).truncate().min_w(0.0))
            .child(text(v.clone()).font(MONO_SMALL.features("tnum")).color(t.text)));
    }
    block
}

// ---- side panel ----

fn header(s: &State) -> El {
    let t = tok::theme(true);
    let (dot, line) = connections_line(s);
    row().items_center().gap(10.0)
        .child(lantern_mark())
        .child(col().gap(3.0).grow(1.0).min_w(0.0)
            .child(text("Lantern").font(type_::TITLE.lh(18.0)).color(t.text))
            .child(row().gap(6.0).items_center()
                .child(el().size(7.0, 7.0).radius(3.5).fill(dot))
                .child(text(line).key("connection").font(type_::LABEL).color(t.text_secondary).truncate())))
}

/// One connection's colour and one-line summary.
fn connection_line(c: &Connection) -> (Color, String) {
    match &c.received.status {
        Status::Connected { server } if server.is_empty() => (good(), format!("Connected · {}", host(&c.url))),
        Status::Connected { server } => (good(), format!("{server} · {}", host(&c.url))),
        Status::Connecting => (waiting(), format!("Connecting to {}", host(&c.url))),
        Status::Failed { .. } => (failed(), format!("Offline · {}", host(&c.url))),
    }
}

/// Every connection, summarised: one names its server, several are counted.
fn connections_line(s: &State) -> (Color, String) {
    match s.connections() {
        [] => (tok::theme(true).text_secondary, "No servers".into()),
        [c] => connection_line(c),
        all => {
            let status = |f: fn(&Status) -> bool| all.iter().filter(|c| f(&c.received.status)).count();
            let up = status(|s| matches!(s, Status::Connected { .. }));
            let dot = if status(|s| matches!(s, Status::Failed { .. })) > 0 {
                failed()
            } else if up < all.len() {
                waiting()
            } else {
                good()
            };
            (dot, format!("{up} of {} servers connected", all.len()))
        }
    }
}

fn host(url: &str) -> &str {
    url.trim_start_matches("ws://").trim_end_matches('/')
}

/// The 3D view's quick controls (all of its settings are in its settings, `view3d_settings`).
fn sidebar(s: &State) -> El {
    let t = tok::theme(true);
    let gear = icon_button("open-settings", "Settings", false, sliders_glyph(), |s| s.settings_open = true);
    let tiles = icon_button("edit-layout", "Edit layout", s.layout_edit, tiles_glyph(), |s| s.layout_edit ^= true);
    let top = row().items_center().gap(8.0).child(header(s).grow(1.0).min_w(0.0)).child(tiles).child(gear);
    let side = col().key("sidebar").w(SIDEBAR_W).h_full().px(20.0).py(22.0).gap(26.0).child(top);
    if !s.settings.layout.has(PanelKind::View3d) {
        return side.child(text("No 3D view in this layout. Each panel's settings button opens what it shows.").font(type_::LABEL).color(t.text_secondary).max_lines(3));
    }
    let v = s.v3();
    // The cards, only while they'd draw (no panel of their own) and a server offers their topics.
    let offered = |kind, card: bool| card && !s.topics_of(kind).is_empty();
    let displays = section("Displays")
        .child(toggle_row("grid", "Grid", v.grid, |s| s.edit3d(|v| v.grid ^= true)))
        .child(toggle_row("points", "Point cloud", v.points.show, |s| s.edit3d(|v| v.points.show ^= true)))
        .child(toggle_row("frames", "TF frames", v.frames, |s| s.edit3d(|v| v.frames ^= true)))
        .child(toggle_row("poses", "Poses", v.poses.show, |s| s.edit3d(|v| v.poses.show ^= true)))
        .children(offered(Kind::Image, !s.settings.layout.has(PanelKind::Image)).then(|| toggle_row("image", "Camera image", v.image_card, |s| s.edit3d(|v| v.image_card ^= true))))
        .children(offered(Kind::Text, s.data_cards().is_some()).then(|| toggle_row("text", "Text", v.data.text, |s| s.edit3d(|v| v.data.text ^= true))))
        .children(offered(Kind::Stats, s.data_cards().is_some()).then(|| toggle_row("stats", "Stats", v.data.stats, |s| s.edit3d(|v| v.data.stats ^= true))));

    let cloud = section("Point cloud")
        .child(field("Color", color_choices("color", s, false)))
        .child(point_size_field("size", v))
        .child(density_field("decimate", v));

    // Only once a robot has loaded: there's nothing to show or fade before.
    let robot = s.robot_model().map(|_| {
        section("Robot")
            .child(toggle_row("robot", "Robot model", v.robot.show, |s| s.edit3d(|v| v.robot.show ^= true)))
            .child(opacity_field("robot-opacity", v))
    });

    let camera = section("Camera").child(
        row().gap(8.0)
            .child(button("home", "Reset view", |s| s.camera = s.home_view()))
            .child(button("top", "Top down", |s| s.camera = s.top_view())),
    );

    side.child(displays).child(cloud).children(robot).child(camera)
}

/// Colour sources: Auto and Height always, then (in full) every numeric field of the cloud.
fn color_choices(key: &'static str, s: &State, full: bool) -> El {
    let mut options = vec![(ColorChoice::Auto, "Auto".to_string()), (ColorChoice::Height, "Height".to_string())];
    if full {
        let fields = s.cloud().map(|(_, c)| c.fields.clone()).unwrap_or_default();
        for f in fields.into_iter().filter(|f| !matches!(f.as_str(), "x" | "y" | "z")) {
            options.push((ColorChoice::Field(f.clone()), f));
        }
    }
    let mut wrap = row().key(key).role("radiogroup").gap(6.0).flex_wrap();
    for (i, (choice, label)) in options.into_iter().enumerate() {
        let chosen = choice == s.v3().points.color;
        wrap = wrap.child(chip(format!("{key}-{i}"), &label, chosen, move |s| s.edit3d(|v| v.points.color = choice.clone())));
    }
    wrap
}

// ---- settings page ----

/// What's app-wide: the servers, what they offer, and the layout. Each panel's own settings are
/// on the panel (`panel_settings`).
fn settings_page(s: &State) -> El {
    let t = tok::theme(true);
    let done = el().key("close-settings").role("button").label("Done").focusable(true).h(30.0).px(14.0).center().radius(tok::radius::MD)
        .fill(tok::accent(0, true)).hover(t.hover).press(t.press).press_scale(0.97)
        .on_press(|s: &mut State| s.settings_open = false)
        .child(text("Done").font(type_::LABEL).color(oklch(0.98, 0.0, 0.0)));
    let top = row().items_center().gap(10.0)
        .child(col().gap(3.0).grow(1.0)
            .child(text("Settings").font(TextStyle::new(15.0, 600.0).lh(20.0)).color(t.text))
            .child(text("What each panel shows is in its own settings").font(type_::LABEL).color(t.text_secondary)))
        .child(done);

    let body = col().w_full().gap(28.0).px(SETTINGS_PAD).pb(24.0)
        .child(connection_section(s))
        .child(topics_section(s))
        .child(layout_section(s));

    col().key("settings").w(SETTINGS_W).h_full().pt(22.0).gap(20.0).min_h(0.0)
        .child(top.px(SETTINGS_PAD))
        .child(el().mx(SETTINGS_PAD).h(1.0).fill(t.separator))
        .child(col().key("settings-scroll").grow(1.0).min_h(0.0).w_full().scroll(ScrollSpec { stick_bottom: false }).scroll_edge_fade(16.0).child(body))
}

fn connection_section(s: &State) -> El {
    let t = tok::theme(true);
    let input = el().key("url-field").grow(1.0).min_w(0.0).h(32.0).px(10.0).radius(tok::radius::MD).clip_rounded(tok::radius::MD).fill(t.card).border(1.0, t.card_border).center_y()
        .child(
            text_input(&s.url_edit, MONO)
                .key("url")
                .w_full()
                .single_line()
                .color(t.text)
                .placeholder("ws://localhost:8765", t.text_secondary)
                .on_edit(|s: &mut State| &mut s.url_edit)
                .on_submit(|s: &mut State, tx: &mut Tx| {
                    s.add_connection();
                    tx.blur();
                }),
        );
    let add = button("connect", "Add", State::add_connection).grow(0.0).w(92.0).min_w(92.0);
    let mut sec = section("Connections")
        .child(text("Foxglove WebSocket servers: foxglove_bridge for ROS 2, or any app using a Foxglove SDK. Lantern reads them all at once.").font(type_::LABEL).color(t.text_secondary).max_lines(3));
    if !s.connections().is_empty() {
        let mut list = col().key("connections").radius(tok::radius::MD).fill(t.card).border(1.0, t.card_border);
        for (i, c) in s.connections().iter().enumerate() {
            let (dot, line) = connection_line(c);
            let detail = match &c.received.status {
                Status::Failed { error } => error.clone(),
                _ => line,
            };
            let url = c.url.clone();
            let remove = el().key(format!("remove-{i}")).role("button").label(format!("Remove {}", c.url)).focusable(true)
                .h(26.0).px(10.0).center().radius(tok::radius::SM).hover(t.hover).press(t.press)
                .on_press(move |s: &mut State| s.settings.connections.retain(|u| *u != url))
                .child(text("Remove").font(type_::LABEL).color(t.text_secondary));
            if i > 0 {
                list = list.child(el().w_full().h(1.0).fill(t.separator));
            }
            list = list.child(row().key(format!("connection-{i}")).px(12.0).py(9.0).gap(10.0).items_center()
                .child(el().size(7.0, 7.0).radius(3.5).fill(dot))
                .child(col().gap(2.0).grow(1.0).min_w(0.0)
                    .child(text(c.url.clone()).font(MONO).color(t.text).truncate())
                    .child(text(detail).key(format!("connection-detail-{i}")).font(SMALL).color(t.text_secondary).max_lines(3)))
                .child(remove));
        }
        sec = sec.child(list);
    }
    sec.child(row().gap(8.0).child(input).child(add))
}

fn topics_section(s: &State) -> El {
    section("Topics").child(topics(s, "topic"))
}

/// Every server's topics, each with its kind, schema and last error, under a line counting them.
/// Settings → Topics and the Topics panel show it; `prefix` keys their rows apart (`{prefix}-0`,
/// `{prefix}-1`, … across every server). Which topics a panel shows is in its settings.
pub(crate) fn topics(s: &State, prefix: &str) -> El {
    let t = tok::theme(true);
    let conns = s.connections();
    let total: usize = conns.iter().map(|c| c.received.channels.len()).sum();
    if total == 0 {
        return text("No topics yet. Topics appear here as the servers advertise them.").font(type_::LABEL).color(t.text_secondary).max_lines(3);
    }
    let mut list = col().key(format!("{prefix}s")).radius(tok::radius::MD).fill(t.card).border(1.0, t.card_border);
    let mut i = 0;
    for conn in conns.iter().filter(|c| !c.received.channels.is_empty()) {
        // With several servers, each one's topics sit under its address.
        if conns.len() > 1 {
            if i > 0 {
                list = list.child(el().w_full().h(1.0).fill(t.separator));
            }
            list = list.child(row().px(12.0).pt(8.0).pb(4.0).child(text(host(&conn.url).to_string()).font(MONO_SMALL).color(t.text_secondary)));
        }
        for (row_in_group, c) in conn.received.channels.iter().enumerate() {
            let key = format!("{prefix}-{i}");
            let badge = c.kind.map_or_else(unsupported_badge, kind_badge);
            let mut info = col().gap(2.0).grow(1.0).min_w(0.0)
                .child(text(c.topic.clone()).font(MONO).color(if c.kind.is_some() { t.text } else { t.text_secondary }).truncate())
                .child(text(format!("{} · {}", c.schema, c.encoding)).font(SMALL).color(t.text_secondary).truncate());
            if let Some(err) = conn.received.errors.get(&c.topic) {
                info = info.child(text(err.clone()).key(format!("{key}-error")).font(SMALL).color(oklch(0.72, 0.17, 25.0)).max_lines(2));
            }
            if row_in_group > 0 {
                list = list.child(el().w_full().h(1.0).fill(t.separator));
            }
            list = list.child(row().key(key).px(12.0).py(9.0).gap(10.0).items_center().child(badge).child(info));
            i += 1;
        }
    }
    let supported = conns.iter().flat_map(|c| &c.received.channels).filter(|c| c.kind.is_some()).count();
    col().w_full().gap(10.0).child(text(format!("{total} topics, {supported} Lantern can draw")).font(type_::LABEL).color(t.text_secondary)).child(list)
}

fn toggle_in(set: &mut BTreeSet<String>, topic: &str) {
    if !set.remove(topic) {
        set.insert(topic.to_string());
    }
}

fn kind_badge(kind: Kind) -> El {
    let hue = match kind {
        Kind::PointCloud => 250.0,
        Kind::Transforms => 150.0,
        Kind::Poses => 70.0,
        Kind::JointStates => 310.0,
        Kind::Image => 200.0,
        Kind::Annotations => 230.0,
        Kind::Text => 110.0,
        Kind::Stats => 30.0,
        Kind::RobotDescription => 340.0,
    };
    el().w(84.0).h(20.0).radius(10.0).center().fill(oklcha(0.75, 0.12, hue, 0.16))
        .child(text(kind.label()).font(SMALL).color(oklch(0.84, 0.1, hue)))
}

fn unsupported_badge() -> El {
    let t = tok::theme(true);
    el().w(84.0).h(20.0).radius(10.0).center().border(1.0, t.card_border)
        .child(text("No decoder").font(SMALL).color(t.text_secondary))
}

fn layout_section(s: &State) -> El {
    let t = tok::theme(true);
    let mut presets = row().key("layout-presets").gap(6.0).flex_wrap();
    for Preset { name, layout } in PRESETS {
        let chosen = s.settings.layout.same_tiling(&layout());
        presets = presets.child(chip(format!("layout-preset-{name}"), name, chosen, move |s| {
            // The preset's tiling, keeping the panels' settings.
            let mut l = layout();
            l.adopt(&s.settings.layout);
            s.settings.layout = l;
        }));
    }
    let input = el().key("layout-file-field").grow(1.0).min_w(0.0).h(32.0).px(10.0).radius(tok::radius::MD).clip_rounded(tok::radius::MD).fill(t.card).border(1.0, t.card_border).center_y()
        .child(
            text_input(&s.layout_file_edit, MONO)
                .key("layout-file")
                .w_full()
                .single_line()
                .color(t.text)
                .placeholder("/path/to/layout.json", t.text_secondary)
                .on_edit(|s: &mut State| &mut s.layout_file_edit)
                .on_submit(|s: &mut State, tx: &mut Tx| {
                    s.load_layout();
                    tx.blur();
                }),
        );
    let note = s.layout_file_note.as_ref().map(|note| match note {
        Ok(done) => text(done.clone()).font(type_::LABEL).color(t.text_secondary).max_lines(2),
        Err(e) => text(e.clone()).font(type_::LABEL).color(oklch(0.72, 0.17, 25.0)).max_lines(4),
    });
    section("Layout")
        .child(toggle_row("edit-layout-full", "Edit layout", s.layout_edit, |s| s.layout_edit ^= true))
        .child(text("While editing, each panel has controls to change what it shows, split it or close it. Drag the gap between two panels to resize them, editing or not.").font(type_::LABEL).color(t.text_secondary).max_lines(3))
        .child(field("Presets", presets))
        .child(text("A preset keeps the settings of the panels it has in common with the layout.").font(type_::LABEL).color(t.text_secondary).max_lines(2))
        .child(field_note("Layout file", "The panels with their settings. Save writes the layout to this file. Load reads a file, or a preset by name. To start with one: lantern --layout FILE."))
        .child(row().gap(8.0).child(input)
            .child(button("load-layout", "Load", State::load_layout).grow(0.0).w(64.0).min_w(64.0))
            .child(button("save-layout", "Save", State::save_layout).grow(0.0).w(64.0).min_w(64.0)))
        .children(note.map(|n| n.key("layout-file-note")))
}

// ---- panel settings ----

/// Applies an edit to one view's settings of type `T`, wherever they live: a panel's, by its
/// path in the layout, or the 3D view's image or data cards'.
type Lens<T> = Rc<dyn Fn(&mut State, &mut dyn FnMut(&mut T))>;

/// The settings of type `T` of the panel at `path`, as `pick` finds them in it.
fn panel_lens<T: 'static>(path: &[usize], pick: fn(&mut Panel) -> Option<&mut T>) -> Lens<T> {
    let path = path.to_vec();
    Rc::new(move |s, edit| {
        if let Some(t) = s.settings.layout.panel_mut(&path).and_then(pick) {
            edit(t);
        }
    })
}

/// Part of the 3D view's settings, as `pick` finds it.
fn view3d_lens<T: 'static>(pick: fn(&mut View3dSettings) -> &mut T) -> Lens<T> {
    Rc::new(move |s, edit| s.edit3d(|v| edit(pick(v))))
}

/// A handler that edits the settings `lens` reaches.
fn with<T: 'static>(lens: &Lens<T>, edit: impl Fn(&mut T) + 'static) -> impl Fn(&mut State) + 'static {
    let lens = lens.clone();
    move |s| lens(s, &mut |t| edit(t))
}

/// The button that opens the settings of the panel keyed `key` at `path`, over it.
pub(crate) fn panel_settings_button(key: &str, path: &[usize]) -> El {
    let t = tok::theme(true);
    let path = path.to_vec();
    el().key(format!("{key}-settings")).role("button").label("Panel settings").focusable(true).size(30.0, 30.0).center()
        .radius(tok::radius::MD).fill(oklcha(0.16, 0.006, 265.0, 0.78)).border(1.0, t.card_border).hover(t.hover).press(t.press).press_scale(0.94)
        .on_press(move |s: &mut State| s.open_panel_settings(path.clone()))
        .child(sliders_glyph())
}

/// The settings of panel `p` at `path`, under its settings button, as wide as `PANEL_SETTINGS_W`
/// and as tall as the window leaves (they scroll). A press outside, Escape or Done closes them.
fn panel_settings(s: &State, path: &[usize], p: &Panel) -> El {
    let t = tok::theme(true);
    let body = match p {
        Panel::View3d(v) => view3d_settings(s, v),
        Panel::Image(view) => image_settings(s, view, "image-panel", &panel_lens(path, |p| match p {
            Panel::Image(i) => Some(i),
            _ => None,
        })),
        Panel::Data(view) => data_settings(s, view, "data-panel", &panel_lens(path, |p| match p {
            Panel::Data(d) => Some(d),
            _ => None,
        })),
        Panel::Topics => col(),
    };
    let done = fitted_button("close-panel-settings", "Done", |s| s.panel_settings = None).h(28.0).px(12.0);
    let top = row().items_center().gap(10.0)
        .child(text(format!("{} settings", p.kind().label())).font(TextStyle::new(14.0, 600.0)).color(t.text).grow(1.0).min_w(0.0))
        .child(done);
    let scroll = col().key("panel-settings-scroll").w_full().shrink().min_h(0.0).scroll(ScrollSpec { stick_bottom: false }).scroll_edge_fade(16.0)
        .child(body.w_full().px(16.0).pb(16.0));
    col().key("panel-settings").w(PANEL_SETTINGS_W).min_h(0.0).pt(14.0).gap(12.0)
        .fill(oklcha(0.19, 0.006, 265.0, 0.98)).radius(tok::radius::LG).border(1.0, t.card_border).shadow(8.0, 28.0, oklcha(0.0, 0.0, 0.0, 0.45))
        .child(top.px(16.0))
        .child(scroll)
        .anchored_fit(format!("{}-settings", panels::key_of(path)), Side::Bottom, Align::End, 6.0)
        .modal(true, |s: &mut State| s.panel_settings = None)
}

/// The 3D view's settings: the scene, the point cloud, poses, the robot, and the cards over it.
fn view3d_settings(s: &State, v: &View3dSettings) -> El {
    let t = tok::theme(true);
    let frames = s.display.tf.frames();
    let scene = section("Scene")
        .child(field_note("Fixed frame", "Everything is drawn in this frame. Auto uses the root of the transform tree the point cloud is in."))
        .child(auto_picker("fixed-frame", "frame", &v.fixed_frame, s.display.fixed.as_deref(), &frames, "not seen yet", |s, f| s.edit3d(|v| v.fixed_frame = f)))
        .child(text(format!("{} frames received", frames.len())).font(type_::LABEL).color(t.text_secondary))
        .child(toggle_row("grid-full", "Grid", v.grid, |s| s.edit3d(|v| v.grid ^= true)))
        .child(toggle_row("frames-full", "TF frames", v.frames, |s| s.edit3d(|v| v.frames ^= true)))
        .child(toggle_row("labels", "Frame and pose labels", v.labels, |s| s.edit3d(|v| v.labels ^= true)))
        .child(field("Axis length", segmented("axis", &[(0.05, "5 cm"), (0.1, "10 cm"), (0.25, "25 cm"), (0.5, "50 cm"), (1.0, "1 m")], v.axis_length, |s, a| s.edit3d(|v| v.axis_length = a))));

    let clouds = s.topics_of(Kind::PointCloud);
    let points = section("Point cloud")
        .child(toggle_row("points-full", "Show", v.points.show, |s| s.edit3d(|v| v.points.show ^= true)))
        .child(field_note("Topic", "Hidden points aren't subscribed to."))
        .child(auto_picker("cloud-topic", "cloud", &v.points.topic, clouds.first().map(String::as_str), &clouds, "not advertised", |s, c| s.edit3d(|v| v.points.topic = c)))
        .child(field("Color", color_choices("color-full", s, true)))
        .child(point_size_field("size-full", v))
        .child(density_field("decimate-full", v));

    let poses = section("Poses")
        .child(toggle_row("poses-full", "Show", v.poses.show, |s| s.edit3d(|v| v.poses.show ^= true)))
        .child(topic_switches(s, "pose", Kind::Poses, &v.poses.hidden, &view3d_lens(|v| &mut v.poses.hidden)))
        .child(field("Expire after", segmented("pose-timeout", &[(0.0, "Never"), (0.5, "0.5 s"), (2.0, "2 s"), (10.0, "10 s")], v.poses.timeout, |s, x| s.edit3d(|v| v.poses.timeout = x))))
        .child(text("A pose its topic hasn't sent again in this time is removed.").font(type_::LABEL).color(t.text_secondary).max_lines(2))
        .child(field("Trail", segmented("trail", &[(0.0, "Off"), (5.0, "5 s"), (15.0, "15 s"), (60.0, "60 s")], v.poses.trail, |s, x| s.edit3d(|v| v.poses.trail = x))));

    // The cards stand in for the panels: with an Image or Data panel in the layout, it shows them.
    let layout = &s.settings.layout;
    let mut cards = section("Cards");
    cards = if layout.has(PanelKind::Image) {
        cards.child(text("The layout's Image panel shows the camera image.").font(type_::LABEL).color(t.text_secondary))
    } else {
        cards.child(toggle_row("show-image-card", "Camera image", v.image_card, |s| s.edit3d(|v| v.image_card ^= true)))
            .child(image_settings(s, &v.image, "card", &view3d_lens(|v| &mut v.image)))
    };
    cards = if layout.has(PanelKind::Data) {
        cards.child(text("The layout's Data panel shows text and stats.").font(type_::LABEL).color(t.text_secondary))
    } else {
        cards.child(data_settings(s, &v.data, "card", &view3d_lens(|v| &mut v.data)))
    };

    col().gap(28.0).child(scene).child(points).child(poses).child(robot_section(s, v)).child(cards)
}

/// An image view's settings: its topic and the annotations drawn over it.
fn image_settings(s: &State, view: &ImageSettings, key: &str, lens: &Lens<ImageSettings>) -> El {
    let images = s.topics_of(Kind::Image);
    let pick = lens.clone();
    col().w_full().gap(10.0)
        .child(field_note("Image topic", "Only the topics a panel shows are subscribed to."))
        .child(auto_picker(&format!("{key}-image-topic"), &format!("{key}-image"), &view.topic, images.first().map(String::as_str), &images, "not advertised",
            move |s, topic| pick(s, &mut |i| i.topic = topic.clone())))
        .child(field_note("Annotations", "Drawn over the image."))
        .child(topic_switches(s, &format!("{key}-annotation"), Kind::Annotations, &view.hidden, &lens_field(lens, |i| &mut i.hidden)))
}

/// A text and stats view's settings: each on or off, and each topic.
fn data_settings(s: &State, view: &DataSettings, key: &str, lens: &Lens<DataSettings>) -> El {
    let hidden = lens_field(lens, |d| &mut d.hidden);
    col().w_full().gap(10.0)
        .child(toggle_row(format!("{key}-text"), "Text", view.text, with(lens, |d| d.text ^= true)))
        .children(view.text.then(|| topic_switches(s, &format!("{key}-text-topic"), Kind::Text, &view.hidden, &hidden)))
        .child(toggle_row(format!("{key}-stats"), "Stats", view.stats, with(lens, |d| d.stats ^= true)))
        .children(view.stats.then(|| topic_switches(s, &format!("{key}-stats-topic"), Kind::Stats, &view.hidden, &hidden)))
}

/// Part of the settings `lens` reaches.
fn lens_field<T: 'static, F: 'static>(lens: &Lens<T>, pick: fn(&mut T) -> &mut F) -> Lens<F> {
    let lens = lens.clone();
    Rc::new(move |s, edit| lens(s, &mut |t| edit(pick(t))))
}

/// A switch for every advertised topic of `kind`: on unless it's in `hidden` (so new topics are
/// on), keyed `{key}-0`, `{key}-1`, ...
fn topic_switches(s: &State, key: &str, kind: Kind, hidden: &BTreeSet<String>, lens: &Lens<BTreeSet<String>>) -> El {
    let t = tok::theme(true);
    let topics = s.topics_of(kind);
    if topics.is_empty() {
        return text(format!("No {} topics yet", kind.label().to_lowercase())).font(type_::LABEL).color(t.text_secondary);
    }
    let mut list = col().key(key.to_string()).gap(2.0);
    for (i, topic) in topics.into_iter().enumerate() {
        let on = !hidden.contains(&topic);
        let k = format!("{key}-{i}");
        let flip = with(lens, { let topic = topic.clone(); move |h| toggle_in(h, &topic) });
        list = list.child(row().h(30.0).px(8.0).gap(10.0).items_center()
            .child(text(topic.clone()).font(MONO).color(if on { t.text } else { t.text_secondary }).truncate().grow(1.0).min_w(0.0))
            .child(switch(&k, &topic, on, flip)));
    }
    list
}

/// The 3D view's robot: its URDF file or a server's description, the joints that pose it, and
/// how it's drawn.
fn robot_section(s: &State, v: &View3dSettings) -> El {
    let t = tok::theme(true);
    // Clipped: a long path would otherwise draw over the Load button.
    let input = el().key("urdf-field").grow(1.0).min_w(0.0).h(32.0).px(10.0).radius(tok::radius::MD).clip_rounded(tok::radius::MD).fill(t.card).border(1.0, t.card_border).center_y()
        .child(
            text_input(&s.urdf_edit, MONO)
                .key("urdf")
                .w_full()
                .single_line()
                .color(t.text)
                .placeholder("/path/to/robot.urdf", t.text_secondary)
                .on_edit(|s: &mut State| &mut s.urdf_edit)
                .on_submit(|s: &mut State, tx: &mut Tx| {
                    s.load_robot();
                    tx.blur();
                }),
        );
    let error = oklch(0.72, 0.17, 25.0);
    let mut sec = section("Robot")
        .child(field_note("URDF file", "Plain URDF (run xacro first). STL meshes, by path relative to the file, file:// or package://."))
        .child(row().gap(8.0).child(input).child(button("load-urdf", "Load", State::load_robot).grow(0.0).w(92.0).min_w(92.0)));
    let summary = |m: &crate::robot::Model| format!(
        "{}: {} links, {} joints, {} meshes · loaded in {:.0} ms",
        m.robot.name, m.robot.links.len(), m.robot.movable().count(), m.mesh.parts.len(), m.load_ms,
    );
    let status = match s.robot_file() {
        None => text("No file loaded").font(type_::LABEL).color(t.text_secondary),
        Some(Err(e)) => text(e.clone()).font(type_::LABEL).color(error).max_lines(4),
        Some(Ok(m)) => text(summary(m)).font(type_::LABEL).color(t.text_secondary).max_lines(2),
    };
    sec = sec.child(status.key("robot-status"));

    // A server that sends its URDF on a robot_description topic says which robot it drives.
    let descriptions = s.topics_of(Kind::RobotDescription);
    if !descriptions.is_empty() {
        let list = auto_picker("description-topic", "description", &v.robot.description_topic, descriptions.first().map(String::as_str), &descriptions, "not advertised", |s, d| s.edit3d(|v| v.robot.description_topic = d));
        let status = match s.robot_description() {
            None => text("Nothing received yet").font(type_::LABEL).color(t.text_secondary),
            Some((_, Err(e))) => text(e.clone()).font(type_::LABEL).color(error).max_lines(4),
            Some((_, Ok(m))) => text(summary(m)).font(type_::LABEL).color(t.text_secondary).max_lines(2),
        };
        sec = sec.child(field_note("Robot description", "The URDF a server sends (std_msgs/msg/String). It wins over a file that isn't the same robot."))
            .child(list)
            .child(status.key("description-status"));
    }
    if let Some(m) = s.robot_mismatch() {
        sec = sec.child(text(m.to_string()).key("robot-mismatch").font(type_::LABEL).color(error).max_lines(5));
    }
    let Some(m) = s.robot_model() else { return sec };
    // A model can load with some visuals missing: say which, a few at a time.
    const SHOWN: usize = 3;
    for p in m.problems.iter().take(SHOWN) {
        sec = sec.child(text(p.clone()).font(SMALL).color(error).max_lines(2));
    }
    if m.problems.len() > SHOWN {
        sec = sec.child(text(format!("and {} more visuals not drawn", m.problems.len() - SHOWN)).font(SMALL).color(error));
    }
    if let Some(note) = &s.display.robot_note {
        sec = sec.child(text(note.clone()).font(type_::LABEL).color(t.text_secondary).max_lines(3));
    }

    let topics = s.topics_of(Kind::JointStates);
    let list = auto_picker("joint-topic", "joints", &v.robot.joint_topic, topics.first().map(String::as_str), &topics, "not advertised", |s, j| s.edit3d(|v| v.robot.joint_topic = j));
    let movable: Vec<&str> = m.robot.movable().collect();
    let received = match s.joints() {
        Some((topic, j)) => format!("{} of {} joints received on {topic}", movable.iter().filter(|n| j.contains_key(**n)).count(), movable.len()),
        None => "No joint states yet: drawn at the URDF's zero pose".to_string(),
    };
    sec.child(field_note("Joint states", "sensor_msgs/msg/JointState (CDR from foxglove_bridge, or JSON from an SDK app), or foxglove.JointStates."))
        .child(list)
        .child(text(received).key("joints-received").font(type_::LABEL).color(t.text_secondary).max_lines(2))
        .child(toggle_row("robot-full", "Show robot", v.robot.show, |s| s.edit3d(|v| v.robot.show ^= true)))
        .child(opacity_field("robot-opacity-full", v))
}

/// Radio rows for an optional setting: Auto (naming what it resolves to), every available value,
/// and the chosen value when it isn't available, marked `missing`.
fn auto_picker(
    key: &str,
    row_key: &str,
    chosen: &Option<String>,
    auto: Option<&str>,
    available: &[String],
    missing: &str,
    set: impl Fn(&mut State, Option<String>) + Clone + 'static,
) -> El {
    let auto_label = match (chosen, auto) {
        (None, Some(a)) => format!("Auto ({a})"),
        _ => "Auto".to_string(),
    };
    let mut options: Vec<(Option<String>, String)> = vec![(None, auto_label)];
    options.extend(available.iter().map(|v| (Some(v.clone()), v.clone())));
    if let Some(c) = chosen
        && !available.contains(c)
    {
        options.push((Some(c.clone()), format!("{c} ({missing})")));
    }
    let mut list = col().key(key.to_string()).role("radiogroup").gap(2.0);
    for (i, (value, label)) in options.into_iter().enumerate() {
        let on = value == *chosen;
        let set = set.clone();
        list = list.child(radio_row(&format!("{row_key}-{i}"), &label, on, move |s| set(s, value.clone())));
    }
    list
}

// The side panel and the 3D view's settings both offer these (keyed apart).
fn point_size_field(key: &'static str, v: &View3dSettings) -> El {
    field("Point size (pt)", segmented(key, &[(1.0, "1"), (2.0, "2"), (3.0, "3"), (4.0, "4")], v.points.size, |s, x| s.edit3d(|v| v.points.size = x)))
}

fn opacity_field(key: &'static str, v: &View3dSettings) -> El {
    field("Opacity", segmented(key, &[(1.0, "100%"), (0.75, "75%"), (0.5, "50%"), (0.25, "25%")], v.robot.opacity, |s, x| s.edit3d(|v| v.robot.opacity = x)))
}

fn density_field(key: &'static str, v: &View3dSettings) -> El {
    field("Density", segmented(key, &[(1, "All"), (2, "½"), (4, "¼"), (8, "⅛")], v.points.decimate, |s, x| s.edit3d(|v| v.points.decimate = x)))
}

// ---- controls ----

/// The app icon, `assets/icon.svg`, which the README shows too. Rune wraps the body in its own
/// `<svg>` with the same viewBox; the file names its own colours, so it draws in them.
static LANTERN_ICON: rune::Icon = rune::Icon { name: "lantern", view: 64.0, body: include_str!("../assets/icon.svg") };

/// The header's mark: the app icon at 34 pt.
fn lantern_mark() -> El {
    icon(&LANTERN_ICON, 34.0)
}

fn section(title: &str) -> El {
    let t = tok::theme(true);
    col().w_full().gap(10.0).child(text(title.to_uppercase()).font(type_::SECTION.tracking(0.6)).color(t.text_secondary))
}

fn field(label: &str, control: El) -> El {
    let t = tok::theme(true);
    col().gap(6.0).child(text(label).font(type_::LABEL).color(t.text)).child(control)
}

fn field_note(label: &str, note: &str) -> El {
    let t = tok::theme(true);
    col().gap(3.0).child(text(label).font(type_::LABEL).color(t.text)).child(text(note).font(type_::LABEL).color(t.text_secondary).max_lines(4))
}

/// A switch's track and knob, drawn for `on`.
fn track(key: &str, on: bool) -> El {
    let t = tok::theme(true);
    let knob = el().key(format!("{key}-knob")).size(18.0, 18.0).radius(9.0).fill(t.knob)
        .shadow(1.0, 2.0, t.knob_shadow)
        .absolute(if on { 18.0 } else { 2.0 }, 2.0);
    el().key(format!("{key}-track")).size(38.0, 22.0).radius(11.0)
        .fill(if on { tok::accent(0, true) } else { t.control_off })
        .motion(motion::SNAPPY)
        .child(knob)
}

/// A labelled switch. The whole row is the control (one tap target, one focus stop, one
/// accessible switch); the track and knob only draw its state.
fn toggle_row(key: impl Into<String>, label: &'static str, on: bool, flip: impl Fn(&mut State) + 'static) -> El {
    let t = tok::theme(true);
    let key = key.into();
    row().key(key.clone()).role("switch").label(label).checked(on).focusable(true)
        .h(30.0).items_center().justify_between()
        .on_press(flip)
        .child(text(label).font(type_::BODY).color(t.text))
        .child(track(&key, on))
}

/// A bare switch for a list row (the row's text names it).
fn switch(key: &str, label: &str, on: bool, flip: impl Fn(&mut State) + 'static) -> El {
    el().key(key.to_string()).role("switch").label(label.to_string()).checked(on).focusable(true).on_press(flip).child(track(key, on))
}

fn dot(on: bool) -> El {
    let t = tok::theme(true);
    let ring = el().size(18.0, 18.0).radius(9.0).center().border(1.5, if on { tok::accent(0, true) } else { t.text_secondary.alpha(0.55) });
    if on { ring.child(el().size(8.0, 8.0).radius(4.0).fill(tok::accent(0, true))) } else { ring }
}

fn radio_row(key: &str, label: &str, on: bool, pick: impl Fn(&mut State) + 'static) -> El {
    let t = tok::theme(true);
    row().key(key.to_string()).role("radio").label(label.to_string()).checked(on).focusable(true)
        .h(30.0).px(8.0).gap(10.0).items_center().radius(tok::radius::SM).hover(t.hover).press(t.press)
        .on_press(pick)
        .child(dot(on))
        .child(text(label.to_string()).font(MONO).color(if on { t.text } else { t.text_secondary }).truncate())
}

/// The chosen chip or segment: raised above its track.
fn selected(e: El) -> El {
    e.fill(oklch(0.36, 0.008, 265.0)).shadow(1.0, 2.0, tok::theme(true).edge)
}

/// One option of a wrapping row of them; the chosen one sits raised.
pub(crate) fn chip(key: String, label: &str, chosen: bool, pick: impl Fn(&mut State) + 'static) -> El {
    let t = tok::theme(true);
    let chip = el().key(key).role("radio").label(label.to_string()).checked(chosen).focusable(true)
        .h(28.0).px(12.0).center().radius(tok::radius::MD)
        .on_press(pick)
        .child(text(label.to_string()).font(type_::LABEL).color(if chosen { t.text } else { t.text_secondary }));
    if chosen { selected(chip) } else { chip.fill(t.control_track).hover(t.hover).press(t.press) }
}

/// Equal-width options in a track; the chosen one sits raised.
fn segmented<T: Copy + PartialEq + 'static>(key: &'static str, options: &[(T, &str)], current: T, set: fn(&mut State, T)) -> El {
    let t = tok::theme(true);
    let mut track = row().key(key).role("radiogroup").h(30.0).padding(2.0).gap(2.0).radius(tok::radius::MD).fill(t.control_track);
    for (i, &(value, label)) in options.iter().enumerate() {
        let chosen = value == current;
        let mut seg = el().key(format!("{key}-{i}")).role("radio").label(label).checked(chosen).focusable(true)
            .grow(1.0).h_full().center().radius(tok::radius::SM)
            .on_press(move |s: &mut State| set(s, value))
            .child(text(label).font(type_::LABEL).color(if chosen { t.text } else { t.text_secondary }));
        seg = if chosen {
            selected(seg)
        } else {
            seg.hover(t.hover).press(t.press)
        };
        track = track.child(seg);
    }
    track
}

/// A button that shares its row's width with its neighbours.
fn button(key: &'static str, label: &'static str, act: fn(&mut State)) -> El {
    fitted_button(key, label, act).grow(1.0)
}

/// A button as wide as its label and the padding it's given.
pub(crate) fn fitted_button(key: impl Into<String>, label: &'static str, act: impl Fn(&mut State) + 'static) -> El {
    let t = tok::theme(true);
    el().key(key).role("button").label(label).focusable(true).h(32.0).center().radius(tok::radius::MD)
        .fill(t.control_off).border(1.0, t.card_border).hover(t.hover).press(t.press).press_scale(0.97)
        .on_press(act)
        .child(text(label).font(type_::LABEL).color(t.text))
}

/// Two sliders, drawn from primitives (no icon font needed): the Settings button's glyph.
fn sliders_glyph() -> El {
    let t = tok::theme(true);
    let bar = |w: f32| el().w(w).h(2.0).radius(1.0).fill(t.text_secondary);
    col().gap(3.0).items_end()
        .child(row().gap(2.0).items_center().child(bar(4.0)).child(el().size(5.0, 5.0).radius(2.5).border(1.5, t.text_secondary)).child(bar(5.0)))
        .child(row().gap(2.0).items_center().child(bar(9.0)).child(el().size(5.0, 5.0).radius(2.5).border(1.5, t.text_secondary)).child(bar(0.0)))
}

/// A wide pane beside two stacked ones: the Edit layout button's glyph.
fn tiles_glyph() -> El {
    let t = tok::theme(true);
    let pane = |w: f32, h: f32| el().size(w, h).radius(1.5).border(1.5, t.text_secondary);
    row().gap(2.0).child(pane(8.0, 14.0)).child(col().gap(2.0).child(pane(6.0, 6.0)).child(pane(6.0, 6.0)))
}

/// A square button around `glyph`, filled with the accent while `on`.
fn icon_button(key: &'static str, label: &'static str, on: bool, glyph: El, act: fn(&mut State)) -> El {
    let t = tok::theme(true);
    el().key(key).role("button").label(label).focusable(true).size(32.0, 32.0).center().radius(tok::radius::MD)
        .fill(if on { tok::accent(0, true) } else { t.control_off }).border(1.0, t.card_border).hover(t.hover).press(t.press).press_scale(0.94)
        .on_press(act)
        .child(glyph)
}

pub fn thousands(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thousands_groups_digits() {
        assert_eq!(thousands(0), "0");
        assert_eq!(thousands(999), "999");
        assert_eq!(thousands(1_048_576), "1,048,576");
    }
}
