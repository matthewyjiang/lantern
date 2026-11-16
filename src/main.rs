//! Lantern: a small RViz stand-in. A simulated LiDAR scan of a street, shown in a 3D viewport
//! with RViz's usual displays (grid, point cloud, TF frames) and a side panel to tune them.
//!
//! `cargo run --release` opens the window. Headless (Rune's tooling):
//! `--snap out/lantern.png [--scene top]` renders a frame, `--frames 240` times 240 frames with
//! no readback; the run ends with Rune's CPU timing and Lantern's GPU timestamps.

mod camera;
mod lidar;
mod viewport;

use std::sync::Arc;

use glam::Vec3;
use rune::tok::{self, motion, type_};
use rune::{App, AppSpec, Color, Cx, Drag, El, Phase, PointerButton, Tx, WheelEvent, col, effect, el, oklch, oklcha, row, text};

use camera::OrbitCamera;
use lidar::Cloud;
use viewport::{ColorBy, HEIGHT_RANGE, View, Viewport};

/// Points the scan accumulates before the viewport opens (whole sweeps, so a few more).
const TARGET_POINTS: usize = 1_000_000;
const SIDEBAR_W: f32 = 296.0;

struct State {
    cloud: Arc<Cloud>,
    camera: OrbitCamera,
    color_by: ColorBy,
    point_size: f32,
    decimate: u32,
    show_grid: bool,
    show_points: bool,
    show_frames: bool,
    /// Pointer travel already applied by the drag in progress.
    drag_seen: (f32, f32),
}

impl State {
    fn new() -> State {
        let cloud = Arc::new(lidar::simulate(TARGET_POINTS));
        let camera = home_view(&cloud);
        State {
            cloud,
            camera,
            color_by: ColorBy::Height,
            point_size: 2.0,
            decimate: 1,
            show_grid: true,
            show_points: true,
            show_frames: true,
            drag_seen: (0.0, 0.0),
        }
    }
}

/// Three-quarter view from across the road, framing the whole drive and the building behind it.
fn home_view(cloud: &Cloud) -> OrbitCamera {
    let centre = cloud.poses.iter().copied().sum::<Vec3>() / cloud.poses.len() as f32;
    OrbitCamera {
        target: Vec3::new(centre.x, 4.0, 0.0),
        yaw: (-115.0f32).to_radians(),
        pitch: 30.0f32.to_radians(),
        distance: 62.0,
        ..OrbitCamera::default()
    }
}

fn top_view(cloud: &Cloud) -> OrbitCamera {
    OrbitCamera { yaw: (-90.0f32).to_radians(), pitch: 89.0f32.to_radians(), distance: 72.0, ..home_view(cloud) }
}

fn view_of(s: &State) -> View<'_> {
    View {
        cloud: &s.cloud,
        camera: s.camera,
        color_by: s.color_by,
        point_size: s.point_size,
        decimate: s.decimate,
        show_grid: s.show_grid,
        show_points: s.show_points,
        show_frames: s.show_frames,
    }
}

// ---- view ----

fn view(s: &State, _: &mut Cx<State>) -> El {
    let t = tok::theme(true);
    row().w_full().h_full().fill(t.bg).child(viewport_pane(s)).child(sidebar(s))
}

fn viewport_pane(s: &State) -> El {
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
    let card = el()
        .grow(1.0)
        .h_full()
        .clip_rounded(tok::radius::LG)
        .border(1.0, oklcha(1.0, 0.0, 0.0, 0.07))
        .child(stage)
        .child(scan_badge(s).pin(Some(14.0), Some(14.0), None, None))
        .child(hints().pin(Some(14.0), None, None, Some(14.0)))
        .child(legend(s.color_by).pin(None, None, Some(14.0), Some(14.0)));
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

/// Translucent dark glass for overlays on the viewport.
fn glass() -> El {
    el().fill(oklcha(0.16, 0.006, 265.0, 0.78)).radius(tok::radius::MD).border(1.0, oklcha(1.0, 0.0, 0.0, 0.06))
}

/// How many points the viewport is drawing, so Density and the Point cloud toggle read back.
fn scan_badge(s: &State) -> El {
    let t = tok::theme(true);
    let total = s.cloud.points.len();
    let drawn = drawn_points(s);
    let label = if drawn == total { format!("{} points", thousands(total)) } else { format!("{} of {} points", thousands(drawn), thousands(total)) };
    glass().px(10.0).py(7.0).child(text(label).key("point-count").font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary))
}

fn drawn_points(s: &State) -> usize {
    if s.show_points { s.cloud.points.len().div_ceil(s.decimate as usize) } else { 0 }
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

/// The colour ramp the points use, with its ends labelled.
fn legend(color_by: ColorBy) -> El {
    let t = tok::theme(true);
    // Turbo, sampled at the shader's stops, so the bar matches the points.
    let turbo: [(f32, &str); 11] = [
        (0.0, "#30123b"), (0.1, "#4662d7"), (0.2, "#36aaf9"), (0.3, "#1ae4b6"), (0.4, "#72fe5e"),
        (0.5, "#c8ef34"), (0.6, "#faba39"), (0.7, "#f66b19"), (0.8, "#ca2a04"), (0.9, "#930c01"), (1.0, "#7a0403"),
    ];
    let (label, lo, hi, span) = match color_by {
        ColorBy::Height => ("Height", format!("{:.0} m", HEIGHT_RANGE.0.max(0.0)), format!("{:.0} m", HEIGHT_RANGE.1), (0.12, 0.95)),
        ColorBy::Intensity => ("Intensity", "low".into(), "high".into(), (0.08, 0.92)),
    };
    let stops: Vec<(f32, Color)> = turbo.iter()
        .filter(|(at, _)| *at >= span.0 - 0.1 && *at <= span.1 + 0.1)
        .map(|(at, hex)| (((at - span.0) / (span.1 - span.0)).clamp(0.0, 1.0), rune::color::hex(hex).unwrap()))
        .collect();
    let bar = el().w(168.0).h(8.0).radius(4.0).linear_gradient(std::f32::consts::FRAC_PI_2, stops);
    let ends = row().w(168.0).justify_between()
        .child(text(lo).font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary))
        .child(text(hi).font(type_::MONO_CAPTION.features("tnum")).color(t.text_secondary));
    glass().px(12.0).py(9.0).child(
        col().gap(6.0).child(text(label).font(type_::LABEL).color(t.text)).child(bar).child(ends),
    )
}

fn sidebar(s: &State) -> El {
    let t = tok::theme(true);
    let header = row().items_center().gap(10.0)
        .child(lantern_mark())
        .child(col().gap(2.0)
            .child(text("Lantern").font(type_::TITLE.lh(18.0)).color(t.text))
            .child(text("Simulated LiDAR · street scan").font(type_::LABEL).color(t.text_secondary)));

    let displays = section("Displays")
        .child(toggle_row("grid", "Grid", s.show_grid, |s| s.show_grid = !s.show_grid))
        .child(toggle_row("points", "Point cloud", s.show_points, |s| s.show_points = !s.show_points))
        .child(toggle_row("frames", "Sensor frames", s.show_frames, |s| s.show_frames = !s.show_frames));

    let cloud = section("Point cloud")
        .child(field("Color", segmented("color", &[(ColorBy::Height, "Height"), (ColorBy::Intensity, "Intensity")], s.color_by, |s, v| s.color_by = v)))
        .child(field("Point size (pt)", segmented("size", &[(1.0, "1"), (2.0, "2"), (3.0, "3"), (4.0, "4")], s.point_size, |s, v| s.point_size = v)))
        .child(field("Density", segmented("decimate", &[(1, "All"), (2, "½"), (4, "¼"), (8, "⅛")], s.decimate, |s, v| s.decimate = v)));

    let camera = section("Camera").child(
        row().gap(8.0)
            .child(button("home", "Reset view", |s| s.camera = home_view(&s.cloud)))
            .child(button("top", "Top down", |s| s.camera = top_view(&s.cloud))),
    );

    col().key("sidebar").w(SIDEBAR_W).h_full().px(20.0).py(22.0).gap(26.0)
        .child(header)
        .child(displays)
        .child(cloud)
        .child(camera)
}

/// A small lantern glyph: a warm light in a rounded frame.
fn lantern_mark() -> El {
    let glow = oklch(0.84, 0.15, 80.0);
    el().size(34.0, 34.0).radius(10.0).fill(oklch(0.27, 0.02, 75.0)).border(1.0, oklcha(1.0, 0.9, 80.0, 0.12)).center()
        .child(el().size(12.0, 12.0).radius(6.0).fill(glow).shadow(0.0, 10.0, oklcha(0.84, 0.15, 80.0, 0.7)))
}

fn section(title: &str) -> El {
    let t = tok::theme(true);
    col().gap(10.0).child(text(title.to_uppercase()).font(type_::SECTION.tracking(0.6)).color(t.text_secondary))
}

fn field(label: &str, control: El) -> El {
    let t = tok::theme(true);
    col().gap(6.0).child(text(label).font(type_::LABEL).color(t.text)).child(control)
}

/// A labelled switch. The whole row is the control (one tap target, one focus stop, one
/// accessible switch); the track and knob only draw its state.
fn toggle_row(key: &'static str, label: &'static str, on: bool, flip: fn(&mut State)) -> El {
    let t = tok::theme(true);
    let knob = el().key(format!("{key}-knob")).size(18.0, 18.0).radius(9.0).fill(t.knob)
        .shadow(1.0, 2.0, t.knob_shadow)
        .absolute(if on { 18.0 } else { 2.0 }, 2.0);
    let track = el().key(format!("{key}-track")).size(38.0, 22.0).radius(11.0)
        .fill(if on { tok::accent(0, true) } else { t.control_off })
        .motion(motion::SNAPPY)
        .child(knob);
    row().key(key).role("switch").label(label).checked(on).focusable(true)
        .h(30.0).items_center().justify_between()
        .on_press(flip)
        .child(text(label).font(type_::BODY).color(t.text))
        .child(track)
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
            seg.fill(oklch(0.36, 0.008, 265.0)).shadow(1.0, 2.0, t.edge)
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

fn thousands(n: usize) -> String {
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

fn main() {
    let spec = AppSpec::<State> {
        scenes: &[
            ("top", |s| s.camera = top_view(&s.cloud)),
            ("intensity", |s| s.color_by = ColorBy::Intensity),
            ("street", |s| {
                // Standing on the road, looking down it past the crates and poles.
                s.camera = OrbitCamera { target: Vec3::new(12.0, 0.0, 2.0), yaw: (172.0f32).to_radians(), pitch: 12.0f32.to_radians(), distance: 16.0, ..s.camera };
            }),
            // For timing: the same frame without the points, with half / a quarter of them.
            ("no-points", |s| s.show_points = false),
            ("half", |s| s.decimate = 2),
            ("quarter", |s| s.decimate = 4),
            // Same points, a sixteenth / four times the pixels each: is the draw fill-bound?
            ("size-1", |s| s.point_size = 1.0),
            ("size-4", |s| s.point_size = 4.0),
        ],
        ..AppSpec::new("Lantern", (1280.0, 800.0), State::new, view)
    };
    App::new(spec).compositor(Viewport::new(view_of)).run();
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
