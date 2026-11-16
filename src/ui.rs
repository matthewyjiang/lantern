//! The window: the viewport card with its overlays, and on the right either the side panel
//! (quick display controls) or the Settings page (connection, topics, frames, display). Every
//! control here edits `State::settings` or the camera; `State::sync` applies the edit.

use std::collections::BTreeSet;

use glam::Vec3;
use rune::tok::{self, TextStyle, motion, type_};
use rune::{Color, Cx, Drag, El, Phase, PointerButton, ScrollSpec, Tx, WheelEvent, col, effect, el, oklch, oklcha, row, text, text_input};

use crate::app::State;
use crate::decode::{ColorChoice, Kind};
use crate::scene::PointColor;
use crate::settings::Settings;
use crate::source::Status;
use crate::viewport::{self, Shade};

const SIDEBAR_W: f32 = 296.0;
const SETTINGS_W: f32 = 440.0;
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
    if let Some(size) = cx.layout_size("viewport") {
        for label in frame_labels(s, size) {
            card = card.child(label);
        }
    }
    card = card
        .child(col().gap(8.0).child(point_badge(s)).children(status_chip(s)).pin(Some(14.0), Some(14.0), None, None))
        .child(hints().pin(None, Some(14.0), Some(14.0), None))
        .children(legend(s).map(|l| l.pin(Some(14.0), None, None, Some(14.0))));
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
    let label = match &s.received.cloud {
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

/// Says why the view is empty or incomplete; absent when everything draws.
fn status_chip(s: &State) -> Option<El> {
    let msg = match (&s.received.status, &s.display.problem) {
        (Status::Failed { error }, _) => format!("Can't reach {}: {error}", s.settings.url),
        (Status::Connecting, _) => format!("Connecting to {}…", s.settings.url),
        (Status::Connected { .. }, Some(p)) => p.clone(),
        (Status::Connected { .. }, None) if s.received.server_warning.is_some() => format!("Server: {}", s.received.server_warning.as_deref().unwrap_or_default()),
        (Status::Connected { .. }, None) if s.received.channels.is_empty() => "Connected; the server offers no topics yet".into(),
        _ => return None,
    };
    Some(glass().px(10.0).py(7.0).max_w(520.0).child(
        row().gap(8.0).items_center()
            .child(el().size(7.0, 7.0).radius(3.5).fill(oklch(0.78, 0.15, 70.0)))
            .child(text(msg).key("status-chip").font(type_::LABEL).color(tok::theme(true).text).max_lines(2)),
    ))
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
    let field = match s.received.cloud.as_ref().map(|(_, c)| &c.color) {
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

// ---- side panel ----

fn header(s: &State) -> El {
    let t = tok::theme(true);
    let (dot, line) = connection_line(s);
    row().items_center().gap(10.0)
        .child(lantern_mark())
        .child(col().gap(3.0).grow(1.0).min_w(0.0)
            .child(text("Lantern").font(type_::TITLE.lh(18.0)).color(t.text))
            .child(row().gap(6.0).items_center()
                .child(el().size(7.0, 7.0).radius(3.5).fill(dot))
                .child(text(line).key("connection").font(type_::LABEL).color(t.text_secondary).truncate())))
}

/// The connection's colour and one-line summary.
fn connection_line(s: &State) -> (Color, String) {
    match &s.received.status {
        Status::Connected { server } if server.is_empty() => (oklch(0.78, 0.17, 150.0), format!("Connected · {}", host(&s.settings.url))),
        Status::Connected { server } => (oklch(0.78, 0.17, 150.0), format!("{server} · {}", host(&s.settings.url))),
        Status::Connecting => (oklch(0.78, 0.15, 70.0), format!("Connecting to {}", host(&s.settings.url))),
        Status::Failed { .. } => (oklch(0.66, 0.2, 25.0), format!("Offline · {}", host(&s.settings.url))),
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
        .child(toggle_row("poses", "Poses", st.show_poses, |s| s.settings.show_poses ^= true));

    let cloud = section("Point cloud")
        .child(field("Color", color_choices("color", s, false)))
        .child(point_size_field("size", st))
        .child(density_field("decimate", st));

    let camera = section("Camera").child(
        row().gap(8.0)
            .child(button("home", "Reset view", |s| s.camera = s.home_view()))
            .child(button("top", "Top down", |s| s.camera = s.top_view())),
    );

    col().key("sidebar").w(SIDEBAR_W).h_full().px(20.0).py(22.0).gap(26.0)
        .child(row().items_center().gap(8.0).child(header(s).grow(1.0).min_w(0.0)).child(gear))
        .child(displays)
        .child(cloud)
        .child(camera)
}

/// Colour sources: Auto and Height always, then (in full) every numeric field of the cloud.
fn color_choices(key: &'static str, s: &State, full: bool) -> El {
    let mut options = vec![(ColorChoice::Auto, "Auto".to_string()), (ColorChoice::Height, "Height".to_string())];
    if full {
        let fields = s.received.cloud.as_ref().map(|(_, c)| c.fields.clone()).unwrap_or_default();
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

    let body = col().w_full().gap(28.0).pb(24.0)
        .child(connection_section(s))
        .child(topics_section(s))
        .child(frames_section(s))
        .child(display_section(s));

    col().key("settings").w(SETTINGS_W).h_full().px(22.0).pt(22.0).gap(20.0).min_h(0.0)
        .child(top)
        .child(el().w_full().h(1.0).fill(t.separator))
        .child(col().key("settings-scroll").grow(1.0).min_h(0.0).w_full().scroll(ScrollSpec { stick_bottom: false }).scroll_edge_fade(16.0).child(body))
}

fn connection_section(s: &State) -> El {
    let t = tok::theme(true);
    let (dot, line) = connection_line(s);
    let input = el().key("url-field").grow(1.0).h(32.0).px(10.0).radius(tok::radius::MD).fill(t.card).border(1.0, t.card_border).center_y()
        .child(
            text_input(&s.url_edit, MONO)
                .key("url")
                .single_line()
                .color(t.text)
                .placeholder("ws://localhost:8765", t.text_secondary)
                .on_edit(|s: &mut State| &mut s.url_edit)
                .on_submit(|s: &mut State, tx: &mut Tx| {
                    s.settings.url = s.url_edit.text.trim().to_string();
                    tx.blur();
                }),
        );
    let connect = button("connect", "Connect", |s| s.settings.url = s.url_edit.text.trim().to_string()).grow(0.0).w(92.0).min_w(92.0);
    let detail = match &s.received.status {
        Status::Failed { error } => error.clone(),
        _ => line,
    };
    section("Connection")
        .child(text("Foxglove WebSocket server: foxglove_bridge for ROS 2, or any app using a Foxglove SDK.").font(type_::LABEL).color(t.text_secondary).max_lines(3))
        .child(row().gap(8.0).child(input).child(connect))
        .child(row().gap(8.0).items_center()
            .child(el().size(7.0, 7.0).radius(3.5).fill(dot))
            .child(text(detail).key("connection-detail").font(type_::LABEL).color(t.text_secondary).grow(1.0).max_lines(3)))
}

fn topics_section(s: &State) -> El {
    let t = tok::theme(true);
    let sec = section("Topics");
    if s.received.channels.is_empty() {
        return sec.child(text("No topics yet. Topics appear here as the server advertises them.").font(type_::LABEL).color(t.text_secondary).max_lines(3));
    }
    let cloud_topic = s.cloud_topic();
    let mut list = col().key("topics").radius(tok::radius::MD).fill(t.card).border(1.0, t.card_border);
    for (i, c) in s.received.channels.iter().enumerate() {
        let key = format!("topic-{i}");
        let (badge, control): (El, El) = match c.kind {
            Some(Kind::PointCloud) => {
                let on = cloud_topic.as_deref() == Some(c.topic.as_str());
                let topic = c.topic.clone();
                (kind_badge(Kind::PointCloud), radio_dot(&format!("{key}-show"), &c.topic, on, move |s| s.settings.cloud_topic = Some(topic.clone())))
            }
            Some(Kind::Poses) => {
                let on = !s.settings.hidden_topics.contains(&c.topic);
                let topic = c.topic.clone();
                (kind_badge(Kind::Poses), switch(&format!("{key}-show"), &c.topic, on, move |s| toggle_in(&mut s.settings.hidden_topics, &topic)))
            }
            Some(Kind::Transforms) => (kind_badge(Kind::Transforms), text("Always on").font(type_::LABEL).color(t.text_secondary)),
            None => (unsupported_badge(), el()),
        };
        let mut info = col().gap(2.0).grow(1.0).min_w(0.0)
            .child(text(c.topic.clone()).font(MONO).color(if c.kind.is_some() { t.text } else { t.text_secondary }).truncate())
            .child(text(format!("{} · {}", c.schema, c.encoding)).font(SMALL).color(t.text_secondary).truncate());
        if let Some(err) = s.received.errors.get(&c.topic) {
            info = info.child(text(err.clone()).key(format!("{key}-error")).font(SMALL).color(oklch(0.72, 0.17, 25.0)).max_lines(2));
        }
        if i > 0 {
            list = list.child(el().w_full().h(1.0).fill(t.separator));
        }
        list = list.child(row().key(key).px(12.0).py(9.0).gap(10.0).items_center().child(badge).child(info).child(control));
    }
    let supported = s.received.channels.iter().filter(|c| c.kind.is_some()).count();
    sec.child(text(format!("{} topics, {supported} Lantern can draw", s.received.channels.len())).font(type_::LABEL).color(t.text_secondary)).child(list)
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
    let frames = s.received.tf.frames();
    let auto_label = match (&s.settings.fixed_frame, &s.display.fixed) {
        (None, Some(f)) => format!("Auto ({f})"),
        _ => "Auto".to_string(),
    };
    let mut options: Vec<(Option<String>, String)> = vec![(None, auto_label)];
    options.extend(frames.iter().map(|f| (Some(f.clone()), f.clone())));
    if let Some(f) = &s.settings.fixed_frame
        && !frames.contains(f)
    {
        options.push((Some(f.clone()), format!("{f} (not seen yet)")));
    }
    let mut list = col().key("fixed-frame").role("radiogroup").gap(2.0);
    for (i, (value, label)) in options.into_iter().enumerate() {
        let on = value == s.settings.fixed_frame;
        list = list.child(radio_row(&format!("frame-{i}"), &label, on, move |s| s.settings.fixed_frame = value.clone()));
    }
    section("Frames")
        .child(field_note("Fixed frame", "Everything is drawn in this frame. Auto uses the root of the transform tree the point cloud is in."))
        .child(list)
        .child(toggle_row("labels", "Frame labels", s.settings.show_labels, |s| s.settings.show_labels ^= true))
        .child(field("Axis length", segmented("axis", &[(0.05, "5 cm"), (0.1, "10 cm"), (0.25, "25 cm"), (0.5, "50 cm"), (1.0, "1 m")], s.settings.axis_length, |s, v| s.settings.axis_length = v)))
        .child(text(format!("{} frames received", frames.len())).font(type_::LABEL).color(t.text_secondary))
}

fn display_section(s: &State) -> El {
    let st = &s.settings;
    section("Display")
        .child(field("Point color", color_choices("color-full", s, true)))
        .child(point_size_field("size-full", st))
        .child(density_field("decimate-full", st))
        .child(toggle_row("grid-full", "Grid", st.show_grid, |s| s.settings.show_grid ^= true))
}

// The side panel and the Settings page both offer these (keyed apart).
fn point_size_field(key: &'static str, st: &Settings) -> El {
    field("Point size (pt)", segmented(key, &[(1.0, "1"), (2.0, "2"), (3.0, "3"), (4.0, "4")], st.point_size, |s, v| s.settings.point_size = v))
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

/// Equal-width options in a track; the chosen one sits raised.
/// The chosen chip or segment: raised above its track.
fn selected(e: El) -> El {
    e.fill(oklch(0.36, 0.008, 265.0)).shadow(1.0, 2.0, tok::theme(true).edge)
}

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
