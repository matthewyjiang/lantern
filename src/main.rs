//! Lantern: a small RViz stand-in. Connects to a Foxglove WebSocket server (foxglove_bridge for
//! ROS 2, or any app on a Foxglove SDK) and draws its point clouds, TF frames and poses in a 3D
//! viewport, with a side panel for quick display controls and a Settings page.
//!
//! `cargo run --release` opens the window. Headless (Rune's tooling): `--snap out/lantern.png
//! --realtime --at 3` renders a frame three seconds in, once data has arrived; `--frames 240`
//! times 240 frames with no readback; the run ends with Rune's CPU timing and Lantern's GPU
//! timestamps.

mod app;
mod camera;
mod decode;
mod scene;
mod settings;
mod source;
mod ui;
mod viewport;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use rune::{App, AppHandle, AppSpec};

use app::State;
use decode::{ColorChoice, Decoded};
use settings::Settings;
use source::{Event, Sink};
use viewport::Viewport;

/// Events waiting for the UI thread. Point clouds coalesce to the newest per topic, so a UI
/// that falls behind a fast publisher drops stale clouds instead of queueing them; everything
/// else (small, and order matters for TF) queues in arrival order.
#[derive(Default)]
struct Inbox {
    events: Vec<Event>,
    clouds: HashMap<String, Event>,
    /// An update is queued on the UI thread and will drain the inbox.
    scheduled: bool,
}

fn sink_for(handle: AppHandle<State>) -> Sink {
    let inbox = Arc::new(Mutex::new(Inbox::default()));
    Arc::new(move |e: Event| {
        let mut q = inbox.lock().unwrap();
        match e {
            Event::Data { ref topic, data: Decoded::Cloud(_) } => {
                q.clouds.insert(topic.clone(), e);
            }
            e => q.events.push(e),
        }
        if !q.scheduled {
            q.scheduled = true;
            let inbox = Arc::clone(&inbox);
            handle.update(move |s: &mut State, _| {
                let mut q = inbox.lock().unwrap();
                q.scheduled = false;
                let mut events = std::mem::take(&mut q.events);
                events.extend(q.clouds.drain().map(|(_, e)| e));
                drop(q);
                s.apply(events);
            });
        }
    })
}

fn main() {
    let spec = AppSpec::<State> {
        scenes: &[
            ("top", |s| s.frame_with = State::top_view),
            ("height", |s| s.set_unsaved(|c| c.color = ColorChoice::Height)),
            ("intensity", |s| s.set_unsaved(|c| c.color = ColorChoice::Field("intensity".into()))),
            ("settings", |s| s.settings_open = true),
            // For timing: the same frame without the points, with half / a quarter of them.
            ("no-points", |s| s.set_unsaved(|c| c.show_points = false)),
            ("half", |s| s.set_unsaved(|c| c.decimate = 2)),
            ("quarter", |s| s.set_unsaved(|c| c.decimate = 4)),
            // Same points, a sixteenth / four times the pixels each: is the draw fill-bound?
            ("size-1", |s| s.set_unsaved(|c| c.point_size = 1.0)),
            ("size-4", |s| s.set_unsaved(|c| c.point_size = 4.0)),
        ],
        poll: Some(|s, _, _| s.sync()),
        // Replaced by `App::init` below, which needs the app's handle for the source's sink.
        ..AppSpec::new("Lantern", (1280.0, 800.0), || unreachable!(), ui::view)
    };
    let app = App::new(spec).compositor(Viewport::new(State::view));
    let sink = sink_for(app.handle());
    app.init(move || State::new(Settings::load(), sink)).run();
}
