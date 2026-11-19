//! Lantern: a small RViz stand-in. Connects to Foxglove WebSocket servers (foxglove_bridge for
//! ROS 2, or any app on a Foxglove SDK) and draws their point clouds, TF frames, poses and robot
//! in a 3D viewport, with camera images, text and stats over it or in panels of their own, each
//! panel with its own settings, and a Settings page for what's app-wide.
//!
//! `cargo run --release` opens the window. `--connect URL` (repeatable), `--layout NAME|PATH`
//! and `--urdf PATH` set the servers, the panel layout and the 3D panel's robot for this run. Headless
//! (Rune's tooling): `--snap out/lantern.png --realtime --at 3` renders a frame three seconds in,
//! once data has arrived; `--frames 240` times 240 frames with no readback; the run ends with
//! Rune's CPU timing and Lantern's GPU timestamps.

mod annotate;
mod app;
mod connection;
mod camera;
mod decode;
mod layout;
mod mesh;
mod objects;
mod panels;
mod robot;
mod scene;
mod settings;
mod source;
mod ui;
mod viewport;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use rune::{App, AppHandle, AppSpec};

use app::{Overrides, State};
use connection::Post;
use decode::{ColorChoice, Decoded};
use layout::Layout;
use settings::Settings;
use source::Event;
use viewport::Viewport;

/// Events waiting for the UI thread, tagged with their connection. Point clouds and images
/// coalesce to the newest per (connection, topic), so a UI that falls behind a fast publisher
/// drops stale ones instead of queueing them; everything else (small, and order matters for TF)
/// queues in arrival order.
#[derive(Default)]
struct Inbox {
    events: Vec<(u64, Event)>,
    /// The newest cloud or image (or its decode failure) per (connection, topic).
    latest: HashMap<(u64, String), Event>,
    /// An update is queued on the UI thread and will drain the inbox.
    scheduled: bool,
}

fn post_for(handle: AppHandle<State>) -> Post {
    let inbox = Arc::new(Mutex::new(Inbox::default()));
    Arc::new(move |conn, e: Event| {
        let mut q = inbox.lock().unwrap();
        match e {
            Event::Data { ref topic, data: Decoded::Cloud(_) | Decoded::Image(_) } => {
                q.latest.insert((conn, topic.clone()), e);
            }
            // A failure newer than a queued cloud or image replaces it, or that would clear it.
            Event::DecodeFailed { ref topic, .. } if q.latest.contains_key(&(conn, topic.clone())) => {
                q.latest.insert((conn, topic.clone()), e);
            }
            e => q.events.push((conn, e)),
        }
        if !q.scheduled {
            q.scheduled = true;
            let inbox = Arc::clone(&inbox);
            handle.update(move |s: &mut State, tx| {
                let mut q = inbox.lock().unwrap();
                q.scheduled = false;
                let mut events = std::mem::take(&mut q.events);
                events.extend(q.latest.drain().map(|((conn, _), e)| (conn, e)));
                drop(q);
                s.apply(events, tx.clock());
            });
        }
    })
}

/// Lantern's own flags, taken out of the command line: the overrides but the layout, and the
/// `--layout` value (a preset adopts the saved layout's panel settings, so it's resolved once
/// they're loaded). The rest go to Rune.
fn parse_args() -> (Overrides, Option<String>, Vec<String>) {
    let mut overrides = Overrides::default();
    let mut layout = None;
    let mut rest = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| {
            eprintln!("lantern: {a} needs a value");
            std::process::exit(2);
        });
        match a.as_str() {
            "--version" => {
                println!("lantern {}", env!("CARGO_PKG_VERSION"));
                std::process::exit(0);
            }
            "--connect" => {
                let (url, list) = (value(), overrides.connections.get_or_insert_with(Vec::new));
                if !list.contains(&url) {
                    list.push(url);
                }
            }
            "--urdf" => overrides.robot_urdf = Some(value()),
            "--layout" => layout = Some(value()),
            _ => rest.push(a),
        }
    }
    (overrides, layout, rest)
}

fn main() {
    let (mut overrides, layout, rune_args) = parse_args();
    let settings = Settings::load();
    if let Some(value) = layout {
        overrides.layout = Some(Layout::named_or_file(&value, &settings.layout).unwrap_or_else(|e| {
            eprintln!("lantern: --layout: {e}");
            std::process::exit(2);
        }));
    }
    let spec = AppSpec::<State> {
        scenes: &[
            ("top", |s| s.frame_with = State::top_view),
            ("height", |s| s.set_unsaved(|v| v.points.color = ColorChoice::Height)),
            ("intensity", |s| s.set_unsaved(|v| v.points.color = ColorChoice::Field("intensity".into()))),
            ("settings", |s| s.settings_open = true),
            // The 3D panel's settings, over it.
            ("panel-settings", |s| {
                if let Some(path) = s.settings.layout.find(layout::PanelKind::View3d) {
                    s.toggle_panel_settings(path);
                }
            }),
            // For timing: the same frame without the points, with half / a quarter of them.
            ("no-points", |s| s.set_unsaved(|v| v.points.show = false)),
            ("half", |s| s.set_unsaved(|v| v.points.decimate = 2)),
            ("quarter", |s| s.set_unsaved(|v| v.points.decimate = 4)),
            // Same points, a sixteenth / four times the pixels each: is the draw fill-bound?
            ("size-1", |s| s.set_unsaved(|v| v.points.size = 1.0)),
            ("size-4", |s| s.set_unsaved(|v| v.points.size = 4.0)),
        ],
        poll: Some(|s, clock, tx| s.sync(clock, tx)),
        // Replaced by `App::init` below, which needs the app's handle for the sources' events.
        ..AppSpec::new("Lantern", (1280.0, 800.0), || unreachable!(), ui::view)
    };
    let app = App::new(spec).args(rune_args).compositor(Viewport::new(State::view));
    let post = post_for(app.handle());
    app.init(move || State::new(settings, overrides, post)).run();
}
