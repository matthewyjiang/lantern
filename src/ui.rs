//! The window: the viewport card with its overlays (camera image, text, stats), and on the right
//! either the side panel (quick display controls) or the Settings page (connections, topics,
//! frames, poses, robot, display). Every control here edits `State::settings` or the camera;
//! `State::sync` applies the edit.

use std::collections::BTreeSet;

use glam::Vec3;
use rune::tok::{self, TextStyle, motion, type_};
use rune::color::srgb_decode;
use rune::{Color, Cx, Drag, El, Phase, PointerButton, ScrollSpec, Tx, WheelEvent, col, effect, el, oklch, oklcha, row, text, text_input};

use crate::app::State;
use crate::connection::Connection;
use crate::decode::{ColorChoice, Kind};
use crate::scene::{Annotation, Level, PointColor};
use crate::settings::Settings;
use crate::source::Status;
use crate::viewport::{self, Shade};

const SIDEBAR_W: f32 = 296.0;
const SETTINGS_W: f32 = 440.0;
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
const SMALL: TextStyle = TextStyle::new(11.0, 500.0);
/// Topic, frame and URL text: names people type and match character for character.
const MONO: TextStyle = TextStyle::new(12.0, 500.0).mono();
const MONO_SMALL: TextStyle = TextStyle::new(11.0, 500.0).mono();

pub fn view(s: &State, cx: &mut Cx<State>) -> El {
    let t = tok::theme(true);
    let side = if s.settings_open { settings_page(s) } else { sidebar(s) };
    row().w_full().h_full().fill(t.bg).child(viewport_pane(s, cx)).child(side)
}

// ---- viewport ----

fn viewport_pane(s: &State, cx: &mut Cx<State>) -> El {
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
    let mut card = el()
        .grow(1.0)
        .h_full()
        .clip_rounded(tok::radius::LG)
        .border(1.0, oklcha(1.0, 0.0, 0.0, 0.07))
        .child(stage);
    let size = cx.layout_size("viewport");
    if let Some(size) = size {
        for label in frame_labels(s, size) {
            card = card.child(label);
        }
    }
    let right = col().key("overlay-right").gap(8.0).items_end().child(hints()).children(text_cards(s)).children(stats_card(s));
    // The status chip wraps short of the right column (hints, text and stats cards).
    let chip_w = match (size, cx.layout_size("overlay-right")) {
        (Some([vw, _]), Some([rw, _])) => (vw - rw - 3.0 * 14.0).clamp(200.0, 520.0),
        _ => 520.0,
    };
    card = card
        .child(col().gap(8.0).items_start().child(point_badge(s)).children(status_chip(s, chip_w)).pin(Some(14.0), Some(14.0), None, None))
        .child(right.pin(None, Some(14.0), Some(14.0), None))
        .children(legend(s).map(|l| l.pin(Some(14.0), None, None, Some(14.0))))
        .children(size.and_then(|size| image_card(s, size)).map(|c| c.pin(None, None, Some(14.0), Some(14.0))));
    el().grow(1.0).h_full().padding(10.0).pr(0.0).child(card)
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
        let clip = vp * Vec3::from(*p).extend(1.0);
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
fn glass() -> El {
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
    // The first two of: the connections, the robot (a URDF that didn't load, or one that isn't
    // the robot the server describes), what the data is missing. Two, so a server that's
    // offline doesn't hide what's wrong with the others' data.
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
    let messages: Vec<String> = [connection, robot, data].into_iter().flatten().take(2).collect();
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

/// The shown image with its annotations, in the viewport's lower right: up to 40% of the
/// viewport's width, `IMAGE_MAX_W`, and 45% of its height, at the image's aspect ratio. Text
/// annotations are set over it at their image positions (Foxglove's: the text's lower left).
fn image_card(s: &State, [vw, vh]: [f32; 2]) -> Option<El> {
    if !s.settings.show_image {
        return None;
    }
    let (topic, image) = s.shown_image()?;
    let t = tok::theme(true);
    let (iw, ih) = (image.width as f32, image.height as f32);
    let w = (vw * 0.4).min(IMAGE_MAX_W).min(vh * 0.45 * iw / ih);
    let scale = w / iw;
    let mut picture = el().key("image-card").size(w, ih * scale).radius(tok::radius::SM).pixels(&s.image);
    for a in s.annotations() {
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
    Some(glass().padding(6.0).child(
        col().gap(5.0)
            .child(row().px(2.0).gap(8.0).justify_between()
                .child(text(topic.clone()).font(MONO_SMALL).color(t.text_secondary).truncate())
                .child(text(format!("{}×{}", image.width, image.height)).font(MONO_SMALL.features("tnum")).color(t.text_secondary)))
            .child(picture),
    ))
}

/// "and N more …", when a card shows only the first of its items.
fn more(n: usize, what: &str) -> Option<El> {
    (n > 0).then(|| text(format!("and {n} more {what}: hide topics in Settings")).font(SMALL).color(tok::theme(true).text_secondary))
}

/// The newest message of the first `TEXT_CARDS` visible text topics.
fn text_cards(s: &State) -> Vec<El> {
    if !s.settings.show_text {
        return Vec::new();
    }
    let t = tok::theme(true);
    let texts: Vec<_> = s.texts().collect();
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

/// The first `STATUSES` of every visible stats topic's statuses: level, name and message, then
/// each reading.
fn stats_card(s: &State) -> Option<El> {
    if !s.settings.show_stats {
        return None;
    }
    let t = tok::theme(true);
    let mut body = col().gap(8.0);
    let statuses: Vec<_> = s.stats().flat_map(|(_, d)| d).collect();
    if statuses.is_empty() {
        return None;
    }
    for d in statuses.iter().take(STATUSES) {
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
        body = body.child(block);
    }
    body = body.children(more(statuses.len().saturating_sub(STATUSES), "statuses"));
    Some(glass().key("stats-card").w(CARD_W).px(10.0).py(8.0).child(body))
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

fn sidebar(s: &State) -> El {
    let st = &s.settings;
    let gear = icon_button("open-settings", "Settings", |s| s.settings_open = true);

    let displays = section("Displays")
        .child(toggle_row("grid", "Grid", st.show_grid, |s| s.settings.show_grid ^= true))
        .child(toggle_row("points", "Point cloud", st.show_points, |s| s.settings.show_points ^= true))
        .child(toggle_row("frames", "TF frames", st.show_frames, |s| s.settings.show_frames ^= true))
        .child(toggle_row("poses", "Poses", st.show_poses, |s| s.settings.show_poses ^= true))
        // Only when a server offers them: nothing to show or hide before.
        .children((!s.topics_of(Kind::Image).is_empty()).then(|| toggle_row("image", "Camera image", st.show_image, |s| s.settings.show_image ^= true)))
        .children((!s.topics_of(Kind::Text).is_empty()).then(|| toggle_row("text", "Text", st.show_text, |s| s.settings.show_text ^= true)))
        .children((!s.topics_of(Kind::Stats).is_empty()).then(|| toggle_row("stats", "Stats", st.show_stats, |s| s.settings.show_stats ^= true)));

    let cloud = section("Point cloud")
        .child(field("Color", color_choices("color", s, false)))
        .child(point_size_field("size", st))
        .child(density_field("decimate", st));

    // Only once a robot has loaded: there's nothing to show or fade before.
    let robot = s.robot_model().map(|_| {
        section("Robot")
            .child(toggle_row("robot", "Robot model", st.show_robot, |s| s.settings.show_robot ^= true))
            .child(opacity_field("robot-opacity", st))
    });

    let camera = section("Camera").child(
        row().gap(8.0)
            .child(button("home", "Reset view", |s| s.camera = s.home_view()))
            .child(button("top", "Top down", |s| s.camera = s.top_view())),
    );

    col().key("sidebar").w(SIDEBAR_W).h_full().px(20.0).py(22.0).gap(26.0)
        .child(row().items_center().gap(8.0).child(header(s).grow(1.0).min_w(0.0)).child(gear))
        .child(displays)
        .child(cloud)
        .children(robot)
        .child(camera)
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
    let t = tok::theme(true);
    let mut wrap = row().key(key).role("radiogroup").gap(6.0).wrap();
    for (i, (choice, label)) in options.into_iter().enumerate() {
        let chosen = choice == s.settings.color;
        let set = choice.clone();
        let mut chip = el().key(format!("{key}-{i}")).role("radio").label(label.clone()).checked(chosen).focusable(true)
            .h(28.0).px(12.0).center().radius(tok::radius::MD)
            .on_press(move |s: &mut State| s.settings.color = set.clone())
            .child(text(label).font(type_::LABEL).color(if chosen { t.text } else { t.text_secondary }));
        chip = if chosen { selected(chip) } else { chip.fill(t.control_track).hover(t.hover).press(t.press) };
        wrap = wrap.child(chip);
    }
    wrap
}

// ---- settings page ----

fn settings_page(s: &State) -> El {
    let t = tok::theme(true);
    let done = el().key("close-settings").role("button").label("Done").focusable(true).h(30.0).px(14.0).center().radius(tok::radius::MD)
        .fill(tok::accent(0, true)).hover(t.hover).press(t.press).press_scale(0.97)
        .on_press(|s: &mut State| s.settings_open = false)
        .child(text("Done").font(type_::LABEL).color(oklch(0.98, 0.0, 0.0)));
    let top = row().items_center().gap(10.0)
        .child(col().gap(3.0).grow(1.0)
            .child(text("Settings").font(TextStyle::new(15.0, 600.0).lh(20.0)).color(t.text))
            .child(text("Changes apply to the view as you make them").font(type_::LABEL).color(t.text_secondary)))
        .child(done);

    let body = col().w_full().gap(28.0).px(SETTINGS_PAD).pb(24.0)
        .child(connection_section(s))
        .child(topics_section(s))
        .child(frames_section(s))
        .child(poses_section(s))
        .child(robot_section(s))
        .child(display_section(s));

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
    let t = tok::theme(true);
    let sec = section("Topics");
    let conns = s.connections();
    let total: usize = conns.iter().map(|c| c.received.channels.len()).sum();
    if total == 0 {
        return sec.child(text("No topics yet. Topics appear here as the servers advertise them.").font(type_::LABEL).color(t.text_secondary).max_lines(3));
    }
    let mut list = col().key("topics").radius(tok::radius::MD).fill(t.card).border(1.0, t.card_border);
    // Rows are keyed topic-0, topic-1, … across every server.
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
            let key = format!("topic-{i}");
            let topic = c.topic.clone();
            let control: El = match c.kind {
                // One topic of the kind is drawn: pick it.
                Some(kind @ (Kind::PointCloud | Kind::JointStates | Kind::Image | Kind::RobotDescription)) => {
                    let on = s.topic(kind).as_deref() == Some(c.topic.as_str());
                    radio_dot(&format!("{key}-show"), &c.topic, on, move |s| {
                        if let Some(slot) = s.settings.picked_mut(kind) {
                            *slot = Some(topic.clone());
                        }
                    })
                }
                Some(Kind::Poses | Kind::Annotations | Kind::Text | Kind::Stats) => {
                    let on = !s.settings.hidden_topics.contains(&c.topic);
                    switch(&format!("{key}-show"), &c.topic, on, move |s| toggle_in(&mut s.settings.hidden_topics, &topic))
                }
                Some(Kind::Transforms) => text("Always on").font(type_::LABEL).color(t.text_secondary),
                None => el(),
            };
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
            list = list.child(row().key(key).px(12.0).py(9.0).gap(10.0).items_center().child(badge).child(info).child(control));
            i += 1;
        }
    }
    let supported = conns.iter().flat_map(|c| &c.received.channels).filter(|c| c.kind.is_some()).count();
    sec.child(text(format!("{total} topics, {supported} Lantern can draw")).font(type_::LABEL).color(t.text_secondary)).child(list)
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

fn frames_section(s: &State) -> El {
    let t = tok::theme(true);
    let frames = s.display.tf.frames();
    let list = auto_picker("fixed-frame", "frame", &s.settings.fixed_frame, s.display.fixed.as_deref(), &frames, "not seen yet", |s, v| s.settings.fixed_frame = v);
    section("Frames")
        .child(field_note("Fixed frame", "Everything is drawn in this frame. Auto uses the root of the transform tree the point cloud is in."))
        .child(list)
        .child(toggle_row("labels", "Frame labels", s.settings.show_labels, |s| s.settings.show_labels ^= true))
        .child(field("Axis length", segmented("axis", &[(0.05, "5 cm"), (0.1, "10 cm"), (0.25, "25 cm"), (0.5, "50 cm"), (1.0, "1 m")], s.settings.axis_length, |s, v| s.settings.axis_length = v)))
        .child(text(format!("{} frames received", frames.len())).font(type_::LABEL).color(t.text_secondary))
}

fn poses_section(s: &State) -> El {
    let t = tok::theme(true);
    let st = &s.settings;
    section("Poses")
        .child(field("Expire after", segmented("pose-timeout", &[(0.0, "Never"), (0.5, "0.5 s"), (2.0, "2 s"), (10.0, "10 s")], st.pose_timeout, |s, v| s.settings.pose_timeout = v)))
        .child(text("A pose its topic hasn't sent again in this time is removed.").font(type_::LABEL).color(t.text_secondary).max_lines(2))
        .child(field("Trail", segmented("trail", &[(0.0, "Off"), (5.0, "5 s"), (15.0, "15 s"), (60.0, "60 s")], st.trail, |s, v| s.settings.trail = v)))
}

fn robot_section(s: &State) -> El {
    let t = tok::theme(true);
    let st = &s.settings;
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
        let list = auto_picker("description-topic", "description", &st.description_topic, descriptions.first().map(String::as_str), &descriptions, "not advertised", |s, v| s.settings.description_topic = v);
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
    let list = auto_picker("joint-topic", "joints", &st.joint_topic, topics.first().map(String::as_str), &topics, "not advertised", |s, v| s.settings.joint_topic = v);
    let movable: Vec<&str> = m.robot.movable().collect();
    let received = match s.joints() {
        Some((topic, j)) => format!("{} of {} joints received on {topic}", movable.iter().filter(|n| j.contains_key(**n)).count(), movable.len()),
        None => "No joint states yet: drawn at the URDF's zero pose".to_string(),
    };
    sec.child(field_note("Joint states", "sensor_msgs/msg/JointState (CDR from foxglove_bridge, or JSON from an SDK app), or foxglove.JointStates."))
        .child(list)
        .child(text(received).key("joints-received").font(type_::LABEL).color(t.text_secondary).max_lines(2))
        .child(toggle_row("robot-full", "Show robot", st.show_robot, |s| s.settings.show_robot ^= true))
        .child(opacity_field("robot-opacity-full", st))
}

/// Radio rows for an optional setting: Auto (naming what it resolves to), every available value,
/// and the chosen value when it isn't available, marked `missing`.
fn auto_picker(
    key: &'static str,
    row_key: &str,
    chosen: &Option<String>,
    auto: Option<&str>,
    available: &[String],
    missing: &str,
    set: fn(&mut State, Option<String>),
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
    let mut list = col().key(key).role("radiogroup").gap(2.0);
    for (i, (value, label)) in options.into_iter().enumerate() {
        let on = value == *chosen;
        list = list.child(radio_row(&format!("{row_key}-{i}"), &label, on, move |s| set(s, value.clone())));
    }
    list
}

fn display_section(s: &State) -> El {
    let st = &s.settings;
    section("Display")
        .child(field("Point color", color_choices("color-full", s, true)))
        .child(point_size_field("size-full", st))
        .child(density_field("decimate-full", st))
        .child(toggle_row("grid-full", "Grid", st.show_grid, |s| s.settings.show_grid ^= true))
        .child(toggle_row("image-full", "Camera image", st.show_image, |s| s.settings.show_image ^= true))
        .child(toggle_row("text-full", "Text", st.show_text, |s| s.settings.show_text ^= true))
        .child(toggle_row("stats-full", "Stats", st.show_stats, |s| s.settings.show_stats ^= true))
}

// The side panel and the Settings page both offer these (keyed apart).
fn point_size_field(key: &'static str, st: &Settings) -> El {
    field("Point size (pt)", segmented(key, &[(1.0, "1"), (2.0, "2"), (3.0, "3"), (4.0, "4")], st.point_size, |s, v| s.settings.point_size = v))
}

fn opacity_field(key: &'static str, st: &Settings) -> El {
    field("Opacity", segmented(key, &[(1.0, "100%"), (0.75, "75%"), (0.5, "50%"), (0.25, "25%")], st.robot_opacity, |s, v| s.settings.robot_opacity = v))
}

fn density_field(key: &'static str, st: &Settings) -> El {
    field("Density", segmented(key, &[(1, "All"), (2, "½"), (4, "¼"), (8, "⅛")], st.decimate, |s, v| s.settings.decimate = v))
}

// ---- controls ----

/// A small lantern glyph: a warm light in a rounded frame.
fn lantern_mark() -> El {
    let glow = oklch(0.84, 0.15, 80.0);
    el().size(34.0, 34.0).radius(10.0).fill(oklch(0.27, 0.02, 75.0)).border(1.0, oklcha(1.0, 0.9, 80.0, 0.12)).center()
        .child(el().size(12.0, 12.0).radius(6.0).fill(glow).shadow(0.0, 10.0, oklcha(0.84, 0.15, 80.0, 0.7)))
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
fn toggle_row(key: &'static str, label: &'static str, on: bool, flip: fn(&mut State)) -> El {
    let t = tok::theme(true);
    row().key(key).role("switch").label(label).checked(on).focusable(true)
        .h(30.0).items_center().justify_between()
        .on_press(flip)
        .child(text(label).font(type_::BODY).color(t.text))
        .child(track(key, on))
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

fn radio_dot(key: &str, label: &str, on: bool, pick: impl Fn(&mut State) + 'static) -> El {
    el().key(key.to_string()).role("radio").label(label.to_string()).checked(on).focusable(true).padding(4.0).on_press(pick).child(dot(on))
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

fn button(key: &'static str, label: &'static str, act: fn(&mut State)) -> El {
    let t = tok::theme(true);
    el().key(key).role("button").label(label).focusable(true).grow(1.0).h(32.0).center().radius(tok::radius::MD)
        .fill(t.control_off).border(1.0, t.card_border).hover(t.hover).press(t.press).press_scale(0.97)
        .on_press(act)
        .child(text(label).font(type_::LABEL).color(t.text))
}

/// A square button with a gear drawn from primitives (no icon font needed).
fn icon_button(key: &'static str, label: &'static str, act: fn(&mut State)) -> El {
    let t = tok::theme(true);
    let bar = |w: f32| el().w(w).h(2.0).radius(1.0).fill(t.text_secondary);
    let glyph = col().gap(3.0).items_end()
        .child(row().gap(2.0).items_center().child(bar(4.0)).child(el().size(5.0, 5.0).radius(2.5).border(1.5, t.text_secondary)).child(bar(5.0)))
        .child(row().gap(2.0).items_center().child(bar(9.0)).child(el().size(5.0, 5.0).radius(2.5).border(1.5, t.text_secondary)).child(bar(0.0)));
    el().key(key).role("button").label(label).focusable(true).size(32.0, 32.0).center().radius(tok::radius::MD)
        .fill(t.control_off).border(1.0, t.card_border).hover(t.hover).press(t.press).press_scale(0.94)
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
