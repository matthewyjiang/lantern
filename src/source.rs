//! Where Lantern's data comes from. A `Source` runs on its own thread, decodes what it
//! receives into scene types and hands `Event`s to a `Sink`; the app only says which topics it
//! wants. `FoxgloveWs` is the one source today: a client for the Foxglove WebSocket protocol,
//! which foxglove_bridge speaks for ROS 2 and the Foxglove SDKs speak for everything else.
//! Another transport (an MCAP file, a different server) is another `Source` impl.

use std::collections::{HashMap, HashSet};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::time::Duration;

use serde::Deserialize;
use tungstenite::client::IntoClientRequest;
use tungstenite::http::HeaderValue;
use tungstenite::error::ProtocolError;
use tungstenite::protocol::WebSocketConfig;
use tungstenite::{Bytes, Message, WebSocket};

use crate::decode::{self, ColorChoice, Decoded, Decoder, Kind};

/// What a source tells the app, from the source's thread.
pub enum Event {
    Status(Status),
    /// Every topic the server offers now (replaces the previous list).
    Channels(Vec<Channel>),
    Data { topic: String, data: Decoded },
    /// A message on `topic` couldn't be decoded; the topic's last good data stays.
    DecodeFailed { topic: String, error: String },
    /// A warning or error the server reported (a dropped message, a failed subscription).
    ServerWarning(String),
}

#[derive(Clone, Debug, Default, PartialEq)]
pub enum Status {
    #[default]
    Connecting,
    Connected { server: String },
    /// The connection failed or dropped; the source retries on its own.
    Failed { error: String },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Channel {
    pub topic: String,
    pub schema: String,
    pub encoding: String,
    /// What it would feed, or `None` if Lantern has no decoder for it.
    pub kind: Option<Kind>,
}

/// What the app wants delivered.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Wanted {
    pub topics: HashSet<String>,
    pub color: ColorChoice,
}

pub type Sink = Arc<dyn Fn(Event) + Send + Sync>;

pub trait Source: Send {
    /// Replaces what the source delivers; returns at once.
    fn want(&self, wanted: Wanted);
}

/// Opens the source for `url`; dropping it disconnects. Every source is a Foxglove WebSocket
/// server today; another transport dispatches on the URL's scheme here.
pub fn open(url: &str, sink: Sink) -> Box<dyn Source> {
    Box::new(FoxgloveWs::spawn(url.to_string(), sink))
}

/// Subprotocols offered, newest first: the Foxglove SDKs (0.x) speak `foxglove.sdk.v1`;
/// foxglove_bridge releases before the SDK rewrite speak `foxglove.websocket.v1`. Both have
/// the same messages. (`foxglove.sdk.v1.1` moves schemas into a table; Lantern doesn't need
/// schemas, as it decodes by schema name.)
const SUBPROTOCOLS: &str = "foxglove.sdk.v1, foxglove.websocket.v1";
/// The largest message Lantern accepts: 5.6 M points at the 48-byte point_step Ouster drivers
/// publish, 8.4 M (the GPU's cap, `viewport::MAX_POINTS`) at 32 bytes. A larger one fails the
/// connection with tungstenite's "Message too long: N > 268435456", shown as the status.
const MAX_MESSAGE: usize = 256 << 20;
// Latencies the user sees, chosen rather than measured: none caps throughput (a busy socket
// never waits on POLL; data is read as it arrives).
/// How long an idle read waits before checking for a stop or a new subscription: a toggle
/// reaches the server within three frames.
const POLL: Duration = Duration::from_millis(50);
/// How long an unreachable host (a robot that's off) shows "Connecting" before "Can't reach".
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// The wait between attempts once a connection fails or drops.
const RETRY: Duration = Duration::from_secs(1);

/// A Foxglove WebSocket client on its own thread. Dropping it disconnects.
pub struct FoxgloveWs {
    wanted: Sender<Wanted>,
    stop: Arc<AtomicBool>,
}

impl FoxgloveWs {
    pub fn spawn(url: String, sink: Sink) -> FoxgloveWs {
        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        std::thread::Builder::new()
            .name("foxglove-ws".into())
            .spawn(move || run(&url, Wanted::default(), &rx, &stopped, &sink))
            .expect("spawning the Foxglove WebSocket thread");
        FoxgloveWs { wanted: tx, stop }
    }
}

impl Source for FoxgloveWs {
    fn want(&self, wanted: Wanted) {
        // A send fails only once the thread has stopped, which only Drop does.
        let _ = self.wanted.send(wanted);
    }
}

impl Drop for FoxgloveWs {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Connects, serves, and reconnects after `RETRY` until stopped.
fn run(url: &str, mut wanted: Wanted, rx: &Receiver<Wanted>, stop: &AtomicBool, sink: &Sink) {
    while !stop.load(Ordering::Relaxed) {
        sink(Event::Status(Status::Connecting));
        let result = connect(url).and_then(|(socket, server)| {
            sink(Event::Status(Status::Connected { server }));
            Session::new(socket, sink).serve(&mut wanted, rx, stop)
        });
        let Err(error) = result else { return };
        sink(Event::Channels(Vec::new()));
        sink(Event::Status(Status::Failed { error }));
        for _ in 0..(RETRY.as_millis() / POLL.as_millis()) {
            if stop.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(POLL);
        }
        // Keep the newest subscription for the next connection.
        while let Ok(w) = rx.try_recv() {
            wanted = w;
        }
    }
}

/// Opens the socket and negotiates the protocol. Returns the server's name from `serverInfo`
/// once it arrives (it's the first message).
fn connect(url: &str) -> Result<(WebSocket<TcpStream>, String), String> {
    let mut request = url.into_client_request().map_err(|e| format!("bad URL {url:?}: {e}"))?;
    if request.uri().scheme_str() != Some("ws") {
        return Err(format!("{url}: only ws:// URLs are supported"));
    }
    request.headers_mut().insert("Sec-WebSocket-Protocol", HeaderValue::from_static(SUBPROTOCOLS));
    let host = request.uri().host().unwrap_or_default().trim_matches(['[', ']']).to_string();
    let port = request.uri().port_u16().unwrap_or(80);
    // Every address in turn: `localhost` often resolves to ::1 first, and the Foxglove SDK
    // listens on 127.0.0.1.
    let mut error = format!("{host}: no address");
    let mut stream = None;
    for addr in (host.as_str(), port).to_socket_addrs().map_err(|e| format!("{host}: {e}"))? {
        match TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT) {
            Ok(s) => {
                stream = Some(s);
                break;
            }
            Err(e) => error = format!("{addr}: {e}"),
        }
    }
    let stream = stream.ok_or(error)?;
    stream.set_read_timeout(Some(CONNECT_TIMEOUT)).ok();
    stream.set_nodelay(true).ok();
    let config = WebSocketConfig::default().max_message_size(Some(MAX_MESSAGE)).max_frame_size(Some(MAX_MESSAGE));
    let (mut socket, _) = tungstenite::client::client_with_config(request, stream, Some(config)).map_err(|e| match e {
        tungstenite::HandshakeError::Failure(tungstenite::Error::Protocol(ProtocolError::SecWebSocketSubProtocolError(_))) => {
            format!("it's a WebSocket server, but not a Foxglove one (it accepted neither {SUBPROTOCOLS})")
        }
        e => format!("WebSocket handshake with {url}: {e}"),
    })?;
    let server = match socket.read().map_err(|e| e.to_string())? {
        Message::Text(t) => match serde_json::from_str::<Op>(&t) {
            Ok(Op::ServerInfo { name }) => name,
            _ => return Err(format!("{url}: expected serverInfo first, got {t:.80}")),
        },
        other => return Err(format!("{url}: expected serverInfo first, got {other:?}")),
    };
    socket.get_mut().set_read_timeout(Some(POLL)).ok();
    Ok((socket, server))
}

/// The server ops Lantern reads; the rest (parameters, services, time) are ignored.
#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "camelCase", rename_all_fields = "camelCase")]
enum Op {
    ServerInfo { name: String },
    Advertise { channels: Vec<Advertised> },
    Unadvertise { channel_ids: Vec<u64> },
    Status { level: u8, message: String },
    #[serde(other)]
    Other,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Advertised {
    id: u64,
    topic: String,
    encoding: String,
    #[serde(default)]
    schema_name: String,
    /// Set on advertise; `None` when Lantern has no decoder for the channel.
    #[serde(skip)]
    decoder: Option<&'static Decoder>,
}

struct Session<'a> {
    socket: WebSocket<TcpStream>,
    sink: &'a Sink,
    channels: HashMap<u64, Advertised>,
    /// Subscription id → channel id.
    subs: HashMap<u32, u64>,
    next_sub: u32,
    /// The newest raw message of each cloud topic, so a colour change re-decodes at once
    /// instead of waiting for the next message (`Bytes` shares the socket's buffer).
    last_cloud: HashMap<u64, Bytes>,
}

impl<'a> Session<'a> {
    fn new(socket: WebSocket<TcpStream>, sink: &'a Sink) -> Self {
        Session { socket, sink, channels: HashMap::new(), subs: HashMap::new(), next_sub: 1, last_cloud: HashMap::new() }
    }

    fn serve(mut self, wanted: &mut Wanted, rx: &Receiver<Wanted>, stop: &AtomicBool) -> Result<(), String> {
        loop {
            if stop.load(Ordering::Relaxed) {
                let _ = self.socket.close(None);
                let _ = self.socket.flush();
                return Ok(());
            }
            let mut changed = None;
            while let Ok(w) = rx.try_recv() {
                changed = Some(w);
            }
            if let Some(w) = changed {
                let recolor = w.color != wanted.color;
                *wanted = w;
                self.resubscribe(wanted)?;
                if recolor {
                    for (&ch, raw) in &self.last_cloud.clone() {
                        self.deliver(ch, raw, &wanted.color);
                    }
                }
            }
            match self.socket.read() {
                Ok(Message::Text(t)) => self.on_op(&t, wanted)?,
                Ok(Message::Binary(b)) => self.on_binary(b, &wanted.color),
                Ok(Message::Close(frame)) => return Err(format!("server closed the connection{}", frame.map(|f| format!(": {}", f.reason)).unwrap_or_default())),
                Ok(_) => {}
                Err(tungstenite::Error::Io(e)) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {}
                Err(e) => return Err(e.to_string()),
            }
        }
    }

    fn on_op(&mut self, text: &str, wanted: &Wanted) -> Result<(), String> {
        match serde_json::from_str::<Op>(text) {
            Ok(Op::Advertise { channels }) => {
                for mut c in channels {
                    c.decoder = decode::decoder_for(&c.encoding, &c.schema_name);
                    self.channels.insert(c.id, c);
                }
                self.publish_channels();
                self.resubscribe(wanted)?;
            }
            Ok(Op::Unadvertise { channel_ids }) => {
                for id in channel_ids {
                    self.channels.remove(&id);
                    self.last_cloud.remove(&id);
                    // The server drops subscriptions to a channel it unadvertises.
                    self.subs.retain(|_, ch| *ch != id);
                }
                self.publish_channels();
            }
            Ok(Op::Status { level, message }) if level >= 1 => (self.sink)(Event::ServerWarning(message)),
            Ok(_) => {}
            Err(e) => eprintln!("lantern: ignoring a server message I can't parse ({e}): {text:.120}"),
        }
        Ok(())
    }

    /// MessageData: opcode 1, u32 subscription id, u64 log time, payload (all little-endian).
    fn on_binary(&mut self, b: Bytes, color: &ColorChoice) {
        if b.len() < 13 || b[0] != 1 {
            return;
        }
        let sub = u32::from_le_bytes(b[1..5].try_into().unwrap());
        // A message can still arrive for a subscription just dropped.
        let Some(&ch) = self.subs.get(&sub) else { return };
        let payload = b.slice(13..);
        let Some(c) = self.channels.get(&ch) else { return };
        if c.decoder.is_some_and(|d| d.kind == Kind::PointCloud) {
            self.last_cloud.insert(ch, payload.clone());
        }
        self.deliver(ch, &payload, color);
    }

    fn deliver(&self, ch: u64, payload: &[u8], color: &ColorChoice) {
        // Only channels with a decoder are subscribed (`resubscribe`).
        let Some((c, d)) = self.channels.get(&ch).and_then(|c| Some((c, c.decoder?))) else { return };
        let topic = c.topic.clone();
        (self.sink)(match d.decode(payload, color) {
            Ok(data) => Event::Data { topic, data },
            Err(e) => Event::DecodeFailed { topic, error: e.to_string() },
        });
    }

    fn publish_channels(&self) {
        let mut list: Vec<Channel> = self.channels.values()
            .map(|c| Channel { topic: c.topic.clone(), schema: c.schema_name.clone(), encoding: c.encoding.clone(), kind: c.decoder.map(|d| d.kind) })
            .collect();
        list.sort_by(|a, b| a.topic.cmp(&b.topic));
        (self.sink)(Event::Channels(list));
    }

    /// Subscribes to every decodable channel on a wanted topic and drops the rest.
    fn resubscribe(&mut self, wanted: &Wanted) -> Result<(), String> {
        let want: HashSet<u64> = self.channels.iter()
            .filter(|(_, c)| wanted.topics.contains(&c.topic) && c.decoder.is_some())
            .map(|(&id, _)| id)
            .collect();
        let drop: Vec<u32> = self.subs.iter().filter(|(_, ch)| !want.contains(ch)).map(|(&s, _)| s).collect();
        for s in &drop {
            if let Some(ch) = self.subs.remove(s) {
                self.last_cloud.remove(&ch);
            }
        }
        let have: HashSet<u64> = self.subs.values().copied().collect();
        let mut add = Vec::new();
        for ch in want.difference(&have) {
            let id = self.next_sub;
            self.next_sub += 1;
            self.subs.insert(id, *ch);
            add.push(serde_json::json!({ "id": id, "channelId": ch }));
        }
        if !drop.is_empty() {
            self.send(serde_json::json!({ "op": "unsubscribe", "subscriptionIds": drop }))?;
        }
        if !add.is_empty() {
            self.send(serde_json::json!({ "op": "subscribe", "subscriptions": add }))?;
        }
        Ok(())
    }

    fn send(&mut self, op: serde_json::Value) -> Result<(), String> {
        self.socket.send(Message::text(op.to_string())).map_err(|e| e.to_string())
    }
}
