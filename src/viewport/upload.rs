//! The points buffer, filled off the UI thread. A new cloud's bytes go into mapped staging
//! memory on a helper thread; the first frame after that records a GPU copy into the points
//! buffer, and until then frames draw the cloud the buffer already holds.
//!
//! Staging is host-visible VRAM, so writing it crosses PCIe at the link's current speed. An
//! idle GPU slows the link: on the UI thread, 1.37 M points (21.9 MB) took 3.8 ms at Gen 2 and
//! 7.6 ms at Gen 1, against a 4.2 ms frame at 240 Hz.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::JoinHandle;
use std::time::Instant;

use glam::Mat4;

use crate::scene::PointCloud;

/// The points storage buffer, the cloud it holds, and the cloud on its way into it.
pub struct CloudBuffer {
    buffer: wgpu::Buffer,
    /// The cloud the buffer holds (as of this frame's commands) and its `display_T_cloud`.
    held: Option<(Arc<PointCloud>, Mat4)>,
    filling: Option<Filling>,
    /// Staging buffers mapped for writing that no fill is using.
    spare: Vec<wgpu::Buffer>,
    /// Staging buffers come back here once remapped after their GPU copy.
    returned: (Sender<wgpu::Buffer>, Receiver<wgpu::Buffer>),
    /// Helper-thread time to fill each landed cloud's staging, ms.
    pub fill_ms: Vec<f64>,
}

struct Filling {
    cloud: Arc<PointCloud>,
    /// Its placement when the fill started.
    model: Mat4,
    /// The staging buffer, filled and still mapped, and the fill's time in ms.
    thread: JoinHandle<(wgpu::Buffer, f64)>,
}

impl CloudBuffer {
    pub fn new(device: &wgpu::Device) -> CloudBuffer {
        // An empty binding is invalid: start with three points' worth of zeros.
        CloudBuffer { buffer: points_buffer(device, 48), held: None, filling: None, spare: Vec::new(), returned: channel(), fill_ms: Vec::new() }
    }

    pub fn buffer(&self) -> &wgpu::Buffer {
        &self.buffer
    }

    pub fn held(&self) -> Option<(&Arc<PointCloud>, Mat4)> {
        self.held.as_ref().map(|(c, m)| (c, *m))
    }

    /// True while a cloud is being filled: the window should keep drawing so it lands.
    pub fn busy(&self) -> bool {
        self.filling.is_some()
    }

    /// Moves the buffer toward holding `want` (a cloud and its placement); true when a cloud
    /// landed, its copy recorded into `encoder` ahead of the draws. A finished fill lands while
    /// any cloud is wanted, even if a newer one has arrived since: it's still newer than the one
    /// held, and a stream faster than a fill would otherwise never land. Then, with no fill
    /// running, a wanted cloud the buffer doesn't hold starts filling.
    pub fn update(&mut self, device: &wgpu::Device, encoder: &mut wgpu::CommandEncoder, want: Option<(&Arc<PointCloud>, Mat4)>) -> bool {
        self.spare.extend(self.returned.1.try_iter());
        let mut landed = false;
        if let Some(f) = self.filling.take_if(|f| f.thread.is_finished()) {
            let (staging, ms) = f.thread.join().unwrap_or_else(|e| std::panic::resume_unwind(e));
            if want.is_some() {
                self.land(device, encoder, staging, size_of_val(f.cloud.points.as_slice()) as u64);
                self.hold(f.cloud, f.model);
                self.fill_ms.push(ms);
                landed = true;
            } else {
                self.spare.push(staging);
            }
        }
        let Some((want, model)) = want else { return landed };
        match &mut self.held {
            Some((held, m)) if Arc::ptr_eq(held, want) => *m = model,
            _ if self.filling.is_some() => {}
            // Nothing to copy, and nothing of it drawn.
            _ if want.points.is_empty() => {
                self.hold(Arc::clone(want), model);
                landed = true;
            }
            _ => self.filling = Some(self.fill(device, want, model)),
        }
        landed
    }

    fn hold(&mut self, cloud: Arc<PointCloud>, model: Mat4) {
        // Often the last reference: freeing 21.9 MB of points took up to 2.4 ms.
        if let Some(old) = self.held.replace((cloud, model)) {
            std::thread::spawn(move || drop(old));
        }
    }

    /// Starts a helper thread writing `cloud` into a spare big enough, or a new staging buffer
    /// it makes. A new buffer's mapping is zeroed on the CPU, so that stays off the UI thread
    /// too; it has a quarter more room than the cloud, so a sensor's next clouds, whose counts
    /// jitter as invalid returns are dropped, fit it.
    fn fill(&mut self, device: &wgpu::Device, cloud: &Arc<PointCloud>, model: Mat4) -> Filling {
        let len = size_of_val(cloud.points.as_slice()) as u64;
        // Spares too small for this cloud won't fit the next one either.
        self.spare.retain(|b| b.size() >= len);
        let spare = self.spare.pop();
        let (device, c) = (device.clone(), Arc::clone(cloud));
        let thread = std::thread::Builder::new()
            .name("lantern cloud upload".into())
            .spawn(move || {
                let t = Instant::now();
                let staging = spare.unwrap_or_else(|| staging_buffer(&device, len + len / 4));
                let mut view = staging.slice(..len).get_mapped_range_mut().expect("spare staging is mapped");
                view.copy_from_slice(bytemuck::cast_slice(&c.points));
                drop(view);
                (staging, t.elapsed().as_secs_f64() * 1000.0)
            })
            .expect("spawning the cloud upload thread");
        Filling { cloud: Arc::clone(cloud), model, thread }
    }

    /// Copies `len` filled bytes of `staging` into the points buffer (a new one if they don't
    /// fit), and returns `staging` to the spares once that copy has run and it's mapped again.
    fn land(&mut self, device: &wgpu::Device, encoder: &mut wgpu::CommandEncoder, staging: wgpu::Buffer, len: u64) {
        staging.unmap();
        if self.buffer.size() < len {
            self.buffer = points_buffer(device, len);
        }
        encoder.copy_buffer_to_buffer(&staging, 0, &self.buffer, 0, len);
        let (back, buffer) = (self.returned.0.clone(), staging.clone());
        encoder.map_buffer_on_submit(&staging, wgpu::MapMode::Write, .., move |r| {
            // An error means the device is lost; the viewport makes everything again.
            if r.is_ok() {
                let _ = back.send(buffer);
            }
        });
    }
}

fn staging_buffer(device: &wgpu::Device, size: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("viewport points staging"),
        size,
        usage: wgpu::BufferUsages::MAP_WRITE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: true,
    })
}

fn points_buffer(device: &wgpu::Device, size: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("viewport points"),
        size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}
