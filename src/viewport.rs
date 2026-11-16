//! The 3D viewport: a Rune compositor that renders the cloud offscreen, then copies it into the
//! `El::effect(MARK)` slot while Rune draws the UI, so the UI clips, rounds and overlays it like
//! any other element.
//!
//! Per frame: find the slot's rect in the instance stream, record one timestamped 3D pass
//! (sky + grid, points, robot, TF and pose lines) into a target the slot's size, then draw the UI with a hook
//! that blits that target at the slot. GPU timestamps bracket the 3D pass alone, so they never
//! include the UI pass or a headless readback; `summary` reports them after a headless run.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3};
use rune::compose::{Compositor, DeviceKey, Frame};
use rune::render::{CLIP_WGSL, EffectSite, Instance, KIND_EFFECT};
use wgpu::util::DeviceExt;

use crate::camera::OrbitCamera;
use crate::robot::RobotMesh;
use crate::scene::PointCloud;

/// The effect-slot tag the viewport paints.
pub const MARK: u32 = 1;

/// The most points one cloud can have: all of them bind as one storage buffer of 16-byte
/// points, and Rune asks wgpu for its default (and downlevel) 128 MiB binding size. The app
/// refuses larger clouds with a per-topic error (`State::apply`).
pub const MAX_POINTS: usize = (128 << 20) / 16;

/// The most robot vertices (three per triangle, 24 bytes each), for the same reason: 5.6 M, or
/// 1.86 M triangles. The KUKA Med 7's visual meshes are 0.50 M. A bigger model fails to load
/// with the count (`robot::load`).
pub const MAX_ROBOT_VERTICES: usize = (128 << 20) / 24;

const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// How points get their colour, with the range the colour ramp spans.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shade {
    /// By height in the display frame, metres.
    Height { lo: f32, hi: f32 },
    /// By each point's field value (`w`).
    Field { lo: f32, hi: f32 },
    /// Each point's own colour, packed in `w`.
    Rgb,
}

/// What the viewport draws, read from app state each frame.
pub struct View<'a> {
    pub camera: OrbitCamera,
    /// The cloud and `display_T_cloud`; `None` draws no points.
    pub cloud: Option<(&'a Arc<PointCloud>, Mat4)>,
    pub shade: Shade,
    /// Point size, in points (device pixels follow the slot's scale).
    pub point_size: f32,
    /// Draw every `decimate`-th point.
    pub decimate: u32,
    /// Ground grid: minor cell size, metres (majors are ten cells); `None` hides it.
    pub grid: Option<f32>,
    /// Roughly how far the data reaches from its centre, metres: scales the depth cue.
    pub radius: f32,
    /// TF axes, the TF tree, poses and velocities, already in the display frame.
    pub lines: &'a Arc<Lines>,
    pub robot: Option<RobotView<'a>>,
}

/// A robot to draw: its mesh, and `display_T_link` for each link its parts name.
pub struct RobotView<'a> {
    pub mesh: &'a Arc<RobotMesh>,
    pub links: &'a [Mat4],
    /// 0–1, multiplying each part's own alpha.
    pub opacity: f32,
}

/// Thick screen-space line segments: three `vec4`s each, as `seg_vs` reads them.
#[derive(Default)]
pub struct Lines(Vec<[f32; 4]>);

pub const RED: [f32; 4] = [0.94, 0.33, 0.31, 1.0];
pub const GREEN: [f32; 4] = [0.45, 0.85, 0.40, 1.0];
pub const BLUE: [f32; 4] = [0.35, 0.55, 1.0, 1.0];

impl Lines {
    /// A segment `width` points wide; `color` is sRGB and alpha.
    pub fn segment(&mut self, a: Vec3, b: Vec3, width: f32, color: [f32; 4]) {
        self.0.extend([[a.x, a.y, a.z, width], [b.x, b.y, b.z, 0.0], color]);
    }

    /// RGB axes of the frame `m` places, each `len` metres.
    pub fn axes(&mut self, m: Mat4, len: f32, width: f32) {
        let o = m.transform_point3(Vec3::ZERO);
        for (axis, color) in [(Vec3::X, RED), (Vec3::Y, GREEN), (Vec3::Z, BLUE)] {
            self.segment(o, m.transform_point3(axis * len), width, color);
        }
    }

    /// An arrow from `a` to `b`, with a head a fifth of its length.
    pub fn arrow(&mut self, a: Vec3, b: Vec3, width: f32, color: [f32; 4]) {
        let d = b - a;
        let len = d.length();
        if len < 1e-6 {
            return;
        }
        self.segment(a, b, width, color);
        let back = -d * 0.2;
        let side = d.cross(if d.normalize().z.abs() > 0.9 { Vec3::X } else { Vec3::Z }).normalize() * len * 0.1;
        self.segment(b, b + back + side, width, color);
        self.segment(b, b + back - side, width, color);
    }

    pub fn len(&self) -> usize {
        self.0.len() / 3
    }
}

pub struct Viewport<S> {
    view_of: fn(&S) -> View<'_>,
    gpu: Option<Gpu>,
    /// Compositor `render` CPU time per frame, ms (recording only; Rune times the whole frame),
    /// and whether that frame uploaded a new cloud.
    encode_ms: Vec<(bool, f64)>,
    /// Time to copy each new cloud into a GPU buffer, ms (included in its frame's `encode_ms`).
    upload_ms: Vec<f64>,
    /// When each frame's `render` started: in a paced run the gaps show frames that ran late.
    starts: Vec<Instant>,
    /// Points drawn on the last frame, and the target size.
    last: (u32, (u32, u32)),
    /// Robot vertices drawn on the last frame.
    last_robot: usize,
}

impl<S> Viewport<S> {
    pub fn new(view_of: fn(&S) -> View<'_>) -> Self {
        Viewport { view_of, gpu: None, encode_ms: Vec::new(), upload_ms: Vec::new(), starts: Vec::new(), last: (0, (0, 0)), last_robot: 0 }
    }
}

impl<S: 'static> Compositor<S> for Viewport<S> {
    fn features(&self) -> wgpu::Features {
        wgpu::Features::TIMESTAMP_QUERY
    }

    fn render(&mut self, f: &mut Frame<'_, S>) {
        let started = Instant::now();
        self.starts.push(started);
        let (device, queue, size) = (f.device, f.queue, f.size);
        let key = f.device_key();
        if self.gpu.as_ref().is_none_or(|g| g.key != key) {
            self.gpu = Some(Gpu::new(device, queue, key));
        }
        let gpu = self.gpu.as_mut().unwrap();
        if let Some(t) = gpu.timer.as_mut() {
            t.collect();
        }

        let slot = slot_rect(&f.rt.instances, &f.rt.marks);
        let mut uploaded = false;
        if let Some(inst) = slot {
            let view = (self.view_of)(&f.rt.state);
            let w = inst.rect[2].round().max(1.0) as u32;
            let h = inst.rect[3].round().max(1.0) as u32;
            gpu.ensure_target(device, (w, h));
            let t = Instant::now();
            if gpu.ensure_scene(device, queue, view.cloud.map(|(c, _)| c), view.lines) {
                uploaded = true;
                self.upload_ms.push(t.elapsed().as_secs_f64() * 1000.0);
            }
            let drawn = view.cloud.map_or(0, |(c, _)| c.points.len().div_ceil(view.decimate.max(1) as usize) as u32);
            let uniform = frame_uniform(&view, (w, h), inst.params[3]);
            let robot = view.robot.as_ref().map(|r| (Arc::clone(r.mesh), part_data(r)));
            let translucent = view.robot.as_ref().map(|r| r.opacity < 1.0 || r.mesh.parts.iter().any(|p| p.color[3] < 1.0));
            f.write_buffer(&gpu.frame_buf, 0, bytemuck::bytes_of(&uniform));
            if let Some((mesh, parts)) = &robot {
                let uploaded = gpu.ensure_robot(device, queue, mesh);
                f.write_buffer(&uploaded.parts, 0, bytemuck::cast_slice(parts));
            }
            gpu.encode_3d(f.encoder, drawn, translucent);
            self.last = (drawn, (w, h));
            self.last_robot = robot.map_or(0, |(m, _)| m.vertices.len());
        }

        let clear = f.rt.clear_color();
        f.ui.prepare(device, queue, &mut f.rt.text.atlas, &f.rt.instances, size);
        let gpu = &*gpu;
        f.ui.draw_with(f.encoder, f.target, clear, &mut |pass, site| {
            if site.mark == MARK && slot.is_some() {
                gpu.blit(device, queue, pass, site);
            }
        });
        self.encode_ms.push((uploaded, started.elapsed().as_secs_f64() * 1000.0));
    }

    fn summary(&mut self, device: &wgpu::Device) -> Option<String> {
        let (drawn, (w, h)) = self.last;
        let mut out = format!("lantern: viewport {w}x{h} px, {drawn} points and {} robot vertices drawn on the last frame\n", self.last_robot);
        let steady: Vec<f64> = self.encode_ms.iter().filter(|(u, _)| !u).map(|(_, ms)| *ms).collect();
        out += &format!("lantern: compositor encode cpu, frames without a new cloud: {}\n", Stats::of(&steady).line());
        out += &format!("lantern: new clouds uploaded to the GPU: {} ({})\n", self.upload_ms.len(), Stats::all(&self.upload_ms).line());
        // Gaps between frame starts; at 60 Hz, one over 25 ms means a frame missed its slot.
        let gaps: Vec<f64> = self.starts.windows(2).map(|w| (w[1] - w[0]).as_secs_f64() * 1000.0).collect();
        let late = gaps.iter().skip(1).filter(|&&g| g > 25.0).count();
        out += &format!("lantern: frame start-to-start {}, {late} gaps over 25 ms\n", Stats::of(&gaps).line());
        match self.gpu.as_mut().and_then(|g| g.timer.as_mut()) {
            Some(t) => {
                t.drain(device);
                // Only frames that drew the final point count: frames before the data arrived
                // (or after a toggle) draw something else and would blur the number.
                let at_last: Vec<f64> = t.samples.iter().filter(|(n, _)| *n == drawn).map(|(_, ms)| *ms).collect();
                out += &format!(
                    "lantern: gpu 3D pass (timestamps), frames drawing {drawn} points: {}; {} other frames timed, {} untimed (found all {} query slots in flight)",
                    Stats::of(&at_last).line(), t.samples.len() - at_last.len(), t.skipped, RING,
                );
            }
            None => out += "lantern: gpu 3D pass: TIMESTAMP_QUERY unavailable on this adapter",
        }
        Some(out)
    }
}

/// The viewport slot's instance, if laid out this frame.
fn slot_rect(instances: &[Instance], marks: &[u32]) -> Option<Instance> {
    instances.iter().zip(marks).find(|(i, m)| i.params[1] == KIND_EFFECT && **m == MARK).map(|(i, _)| *i)
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FrameUniform {
    view_proj: [f32; 16],
    inv_view_proj: [f32; 16],
    model: [f32; 16],
    eye: [f32; 4],
    viewport: [f32; 4],
    style: [u32; 4],
    shade: [f32; 4],
    grid: [f32; 4],
}

/// Each robot part's model matrix and colour (linear, alpha times opacity), as `robot_vs` reads
/// them: one entry per part, indexed by instance.
fn part_data(r: &RobotView<'_>) -> Vec<[f32; 20]> {
    let linear = |c: f32| if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) };
    r.mesh.parts.iter().map(|p| {
        let mut d = [0.0; 20];
        d[..16].copy_from_slice(&r.links[p.link].to_cols_array());
        d[16..].copy_from_slice(&[linear(p.color[0]), linear(p.color[1]), linear(p.color[2]), p.color[3] * r.opacity]);
        d
    }).collect()
}

fn frame_uniform(v: &View<'_>, (w, h): (u32, u32), scale: f32) -> FrameUniform {
    let vp = v.camera.view_proj(w as f32 / h as f32);
    let (mode, lo, hi) = match v.shade {
        Shade::Height { lo, hi } => (0, lo, hi),
        Shade::Field { lo, hi } => (1, lo, hi),
        Shade::Rgb => (2, 0.0, 1.0),
    };
    FrameUniform {
        view_proj: vp.to_cols_array(),
        inv_view_proj: vp.inverse().to_cols_array(),
        model: v.cloud.map_or(Mat4::IDENTITY, |(_, m)| m).to_cols_array(),
        eye: v.camera.eye().extend(1.0).into(),
        viewport: [w as f32, h as f32, v.point_size * scale, scale],
        style: [mode, v.decimate.max(1), v.grid.is_some() as u32, 0],
        shade: [lo, hi, 0.0, 0.0],
        grid: [v.grid.unwrap_or(1.0), v.radius, 0.0, 0.0],
    }
}

/// Everything the viewport keeps on one device.
struct Gpu {
    key: DeviceKey,
    frame_buf: wgpu::Buffer,
    scene_bgl: wgpu::BindGroupLayout,
    blit_bgl: wgpu::BindGroupLayout,
    blit_buf: wgpu::Buffer,
    module: wgpu::ShaderModule,
    sky: wgpu::RenderPipeline,
    points: wgpu::RenderPipeline,
    segs: wgpu::RenderPipeline,
    robot_bgl: wgpu::BindGroupLayout,
    /// The robot in two passes over the same vertices: depth only, then colour where the depth
    /// matches. A translucent robot then shows what's behind it, not its own hidden faces.
    robot_depth: wgpu::RenderPipeline,
    robot_color: wgpu::RenderPipeline,
    robot: Option<RobotGpu>,
    /// One blit pipeline per format Rune hands the hook (frame target, or a layer's).
    blits: std::cell::RefCell<Vec<(wgpu::TextureFormat, wgpu::RenderPipeline)>>,
    target: Option<Target>,
    scene: Option<Uploaded>,
    timer: Option<GpuTimer>,
}

struct Target {
    size: (u32, u32),
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    blit_group: wgpu::BindGroup,
}

/// Buffers for one cloud and one set of lines (each compared by pointer, re-uploaded on change).
struct Uploaded {
    cloud: Option<Arc<PointCloud>>,
    points: wgpu::Buffer,
    lines: Arc<Lines>,
    segs: wgpu::Buffer,
    group: wgpu::BindGroup,
}

/// One robot mesh's vertices (re-uploaded when the mesh changes) and its per-part matrices and
/// colours (written every frame it's drawn).
struct RobotGpu {
    mesh: Arc<RobotMesh>,
    parts: wgpu::Buffer,
    group: wgpu::BindGroup,
}

impl Gpu {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, key: DeviceKey) -> Gpu {
        let limit = device.limits().max_storage_buffer_binding_size;
        assert!(limit >= (MAX_POINTS * 16) as u64, "this device binds {limit} B of storage, under MAX_POINTS × 16 B");
        assert!(limit >= (MAX_ROBOT_VERTICES * 24) as u64, "this device binds {limit} B of storage, under MAX_ROBOT_VERTICES × 24 B");
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("viewport"),
            source: wgpu::ShaderSource::Wgsl(format!("{CLIP_WGSL}\n{}", include_str!("viewport.wgsl")).into()),
        });
        let entry = |binding, ty| wgpu::BindGroupLayoutEntry { binding, visibility: wgpu::ShaderStages::VERTEX_FRAGMENT, ty, count: None };
        let uniform = wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None };
        let storage = wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None };
        let scene_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport scene"),
            entries: &[entry(0, uniform), entry(1, storage), entry(2, storage)],
        });
        let texture = wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: false }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false };
        let blit_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport blit"),
            entries: &[entry(3, uniform), entry(4, texture)],
        });
        let buffer = |label, size| device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label), size, usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false,
        });
        let frame_buf = buffer("viewport frame", size_of::<FrameUniform>() as u64);
        let blit_buf = buffer("viewport blit", 80);

        let robot_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport robot"),
            entries: &[entry(0, storage), entry(1, storage)],
        });
        let layout_of = |groups: &[Option<&wgpu::BindGroupLayout>]| device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None, bind_group_layouts: groups, immediate_size: 0,
        });
        let scene_layout = layout_of(&[Some(&scene_bgl)]);
        let robot_layout = layout_of(&[Some(&scene_bgl), Some(&robot_bgl)]);
        // `compare` None draws over everything (no depth test).
        let pipeline = |layout: &wgpu::PipelineLayout, vs: &str, fs: &str, topology, depth_write: bool, compare: Option<wgpu::CompareFunction>, blend, write_mask| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(vs),
                layout: Some(layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers: &[] },
                primitive: wgpu::PrimitiveState { topology, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(compare.unwrap_or(wgpu::CompareFunction::Always)),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: COLOR_FORMAT, blend, write_mask })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        use wgpu::CompareFunction::{Equal, Greater};
        use wgpu::PrimitiveTopology::{TriangleList, TriangleStrip};
        let (all, blend) = (wgpu::ColorWrites::ALL, Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING));
        let sky = pipeline(&scene_layout, "sky_vs", "sky_fs", TriangleList, false, None, None, all);
        let points = pipeline(&scene_layout, "points_vs", "points_fs", TriangleList, true, Some(Greater), None, all);
        let segs = pipeline(&scene_layout, "seg_vs", "seg_fs", TriangleStrip, false, Some(Greater), blend, all);
        let robot_depth = pipeline(&robot_layout, "robot_vs", "robot_fs", TriangleList, true, Some(Greater), blend, wgpu::ColorWrites::empty());
        let robot_color = pipeline(&robot_layout, "robot_vs", "robot_fs", TriangleList, false, Some(Equal), blend, all);

        let timer = device.features().contains(wgpu::Features::TIMESTAMP_QUERY).then(|| GpuTimer::new(device, queue));
        Gpu {
            key, frame_buf, scene_bgl, blit_bgl, blit_buf, module, sky, points, segs,
            robot_bgl, robot_depth, robot_color, robot: None,
            blits: Default::default(), target: None, scene: None, timer,
        }
    }

    fn ensure_target(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        if self.target.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let texture = |label, format, usage| device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: size.0, height: size.1, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        }).create_view(&Default::default());
        let color = texture("viewport color", COLOR_FORMAT, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING);
        let depth = texture("viewport depth", DEPTH_FORMAT, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let blit_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport blit"),
            layout: &self.blit_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 3, resource: self.blit_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: wgpu::BindingResource::TextureView(&color) },
            ],
        });
        self.target = Some(Target { size, color, depth, blit_group });
    }

    /// Uploads a changed cloud or line set; true when it uploaded a new cloud.
    fn ensure_scene(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, cloud: Option<&Arc<PointCloud>>, lines: &Arc<Lines>) -> bool {
        let old = self.scene.take();
        let same_cloud = old.as_ref().is_some_and(|u| match (&u.cloud, cloud) {
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            (None, None) => true,
            _ => false,
        });
        let same_lines = old.as_ref().is_some_and(|u| Arc::ptr_eq(&u.lines, lines));
        if same_cloud && same_lines {
            self.scene = old;
            return false;
        }
        let points = match &old {
            Some(u) if same_cloud => u.points.clone(),
            _ => {
                let bytes: &[u8] = cloud.map_or(&[], |c| bytemuck::cast_slice(&c.points));
                storage(device, queue, "viewport points", old.as_ref().map(|u| &u.points), bytes)
            }
        };
        let segs = match &old {
            Some(u) if same_lines => u.segs.clone(),
            _ => storage(device, queue, "viewport segments", old.as_ref().map(|u| &u.segs), bytemuck::cast_slice(&lines.0)),
        };
        let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport scene"),
            layout: &self.scene_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: self.frame_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: points.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: segs.as_entire_binding() },
            ],
        });
        self.scene = Some(Uploaded { cloud: cloud.cloned(), points, lines: Arc::clone(lines), segs, group });
        !same_cloud && cloud.is_some()
    }

    /// Uploads `mesh` unless it's the one already uploaded. A hidden robot keeps its buffers, so
    /// showing it again costs nothing. `State::view` only passes meshes with parts.
    fn ensure_robot(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, mesh: &Arc<RobotMesh>) -> &RobotGpu {
        if !self.robot.as_ref().is_some_and(|r| Arc::ptr_eq(&r.mesh, mesh)) {
            self.robot = Some(self.upload_robot(device, queue, mesh));
        }
        self.robot.as_ref().unwrap()
    }

    fn upload_robot(&self, device: &wgpu::Device, queue: &wgpu::Queue, mesh: &Arc<RobotMesh>) -> RobotGpu {
        let vertices = storage(device, queue, "viewport robot vertices", None, bytemuck::cast_slice(&mesh.vertices));
        let parts = storage(device, queue, "viewport robot parts", None, &vec![0u8; size_of::<[f32; 20]>() * mesh.parts.len()]);
        let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport robot"),
            layout: &self.robot_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vertices.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: parts.as_entire_binding() },
            ],
        });
        RobotGpu { mesh: Arc::clone(mesh), parts, group }
    }

    /// Records the timestamped 3D pass into the offscreen target. `robot_translucent` is `None`
    /// for no robot, else whether any part shows through: a translucent robot draws after the
    /// lines, so its depth doesn't hide the TF axes inside its links (lines in front of it take a
    /// faint tint instead); an opaque one draws before them, so they stay crisp.
    fn encode_3d(&mut self, encoder: &mut wgpu::CommandEncoder, drawn: u32, robot_translucent: Option<bool>) {
        let (Some(target), Some(scene)) = (&self.target, &self.scene) else { return };
        let slot = self.timer.as_mut().and_then(|t| t.begin());
        let timestamp_writes = slot.map(|k| wgpu::RenderPassTimestampWrites {
            query_set: &self.timer.as_ref().unwrap().set,
            beginning_of_pass_write_index: Some(2 * k as u32),
            end_of_pass_write_index: Some(2 * k as u32 + 1),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport 3D"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(0.0), store: wgpu::StoreOp::Discard }),
                    stencil_ops: None,
                }),
                timestamp_writes,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &scene.group, &[]);
            pass.set_pipeline(&self.sky);
            pass.draw(0..3, 0..1);
            if drawn > 0 {
                pass.set_pipeline(&self.points);
                pass.draw(0..6 * drawn, 0..1);
            }
            let robot = robot_translucent.zip(self.robot.as_ref());
            let draw_robot = |pass: &mut wgpu::RenderPass<'_>| {
                let Some((_, r)) = robot else { return };
                pass.set_bind_group(1, &r.group, &[]);
                for pipeline in [&self.robot_depth, &self.robot_color] {
                    pass.set_pipeline(pipeline);
                    // The instance index picks the part's matrix and colour.
                    for (k, part) in r.mesh.parts.iter().enumerate() {
                        pass.draw(part.range.clone(), k as u32..k as u32 + 1);
                    }
                }
            };
            let translucent = robot.is_some_and(|(t, _)| t);
            if !translucent {
                draw_robot(&mut pass);
            }
            if scene.lines.len() > 0 {
                pass.set_pipeline(&self.segs);
                pass.draw(0..4, 0..scene.lines.len() as u32);
            }
            if translucent {
                draw_robot(&mut pass);
            }
        }
        if let Some(k) = slot {
            self.timer.as_mut().unwrap().end(encoder, k, drawn);
        }
    }

    /// Copies the offscreen target into the slot, inside Rune's UI pass.
    fn blit(&self, device: &wgpu::Device, queue: &wgpu::Queue, pass: &mut wgpu::RenderPass<'_>, s: &EffectSite<'_>) {
        let Some(target) = &self.target else { return };
        let i = s.instance;
        let mut data = [0.0f32; 20];
        data[0..4].copy_from_slice(&i.rect);
        data[4..8].copy_from_slice(&i.clip);
        for k in 0..4 { data[8 + k] = f32::from_bits(i.fade[k]); }
        data[12..14].copy_from_slice(&s.origin);
        data[14] = s.size[0] as f32;
        data[15] = s.size[1] as f32;
        data[16] = if s.srgb_blend { 1.0 } else { 0.0 };
        // The hook runs inside the pass, so this lands through the queue ahead of the submit.
        queue.write_buffer(&self.blit_buf, 0, bytemuck::cast_slice(&data));
        let mut blits = self.blits.borrow_mut();
        if !blits.iter().any(|(f, _)| *f == s.format) {
            blits.push((s.format, self.blit_pipeline(device, s.format)));
        }
        pass.set_pipeline(&blits.iter().find(|(f, _)| *f == s.format).unwrap().1);
        pass.set_bind_group(0, &target.blit_group, &[]);
        pass.draw(0..4, 0..1);
    }

    fn blit_pipeline(&self, device: &wgpu::Device, format: wgpu::TextureFormat) -> wgpu::RenderPipeline {
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None, bind_group_layouts: &[Some(&self.blit_bgl)], immediate_size: 0,
        });
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("viewport blit"),
            layout: Some(&layout),
            vertex: wgpu::VertexState { module: &self.module, entry_point: Some("blit_vs"), compilation_options: Default::default(), buffers: &[] },
            primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleStrip, ..Default::default() },
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &self.module,
                entry_point: Some("blit_fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState { format, blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING), write_mask: wgpu::ColorWrites::ALL })],
            }),
            multiview_mask: None,
            cache: None,
        })
    }
}

/// Timestamp query slots in flight. Headless runs wait for the GPU every frame, so a slot is
/// read two frames after it's written and four never run out; eight leave room for a window
/// that queues frames. Frames that find every slot busy go untimed and `summary` counts them.
const RING: usize = 8;
/// `resolve_query_set` writes at multiples of this.
const RESOLVE_STRIDE: u64 = wgpu::QUERY_RESOLVE_BUFFER_ALIGNMENT;

/// `bytes` in a storage buffer: `old` refilled when they fit, else a new buffer. A sensor's
/// clouds are about the same size, and refilling halves a 1.37 M-point cloud's upload against a
/// new 22 MB buffer each time (docs, Performance: "Ingesting a cloud"). An empty binding is
/// invalid, so empty data binds 48 zero bytes (one line segment, three points).
fn storage(device: &wgpu::Device, queue: &wgpu::Queue, label: &str, old: Option<&wgpu::Buffer>, bytes: &[u8]) -> wgpu::Buffer {
    match old.filter(|b| !bytes.is_empty() && b.size() >= bytes.len() as u64) {
        Some(b) => {
            queue.write_buffer(b, 0, bytes);
            b.clone()
        }
        None => device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: if bytes.is_empty() { &[0; 48] } else { bytes },
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        }),
    }
}

/// GPU duration of the 3D pass, from begin/end-of-pass timestamps read back asynchronously.
struct GpuTimer {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    slots: Vec<TimerSlot>,
    next: usize,
    ns_per_tick: f64,
    /// Points drawn and GPU ms, per timed frame.
    samples: Vec<(u32, f64)>,
    skipped: u32,
}

struct TimerSlot {
    readback: wgpu::Buffer,
    state: SlotState,
    mapped: Arc<AtomicBool>,
    drawn: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SlotState {
    Free,
    /// Resolved and copied in a frame's encoder, which Rune submits after `render`.
    Recorded,
    /// `map_async` requested; `mapped` flips when it lands.
    Mapping,
}

impl GpuTimer {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> GpuTimer {
        let set = device.create_query_set(&wgpu::QuerySetDescriptor { label: Some("viewport timer"), ty: wgpu::QueryType::Timestamp, count: 2 * RING as u32 });
        let resolve = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("viewport timer resolve"),
            size: RESOLVE_STRIDE * RING as u64,
            usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let slots = (0..RING)
            .map(|_| TimerSlot {
                readback: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("viewport timer readback"),
                    size: 16,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                }),
                state: SlotState::Free,
                mapped: Arc::new(AtomicBool::new(false)),
                drawn: 0,
            })
            .collect();
        GpuTimer { set, resolve, slots, next: 0, ns_per_tick: queue.get_timestamp_period() as f64, samples: Vec::new(), skipped: 0 }
    }

    /// Maps slots recorded last frame (now submitted) and reads the ones that have landed.
    fn collect(&mut self) {
        for slot in &mut self.slots {
            match slot.state {
                SlotState::Recorded => {
                    let mapped = Arc::clone(&slot.mapped);
                    slot.readback.slice(..).map_async(wgpu::MapMode::Read, move |r| {
                        r.expect("mapping a timestamp readback");
                        mapped.store(true, Ordering::Release);
                    });
                    slot.state = SlotState::Mapping;
                }
                SlotState::Mapping if slot.mapped.load(Ordering::Acquire) => {
                    let ticks: [u64; 2] = {
                        let view = slot.readback.slice(..).get_mapped_range().expect("timestamp readback is mapped");
                        bytemuck::pod_read_unaligned(&view)
                    };
                    slot.readback.unmap();
                    slot.mapped.store(false, Ordering::Relaxed);
                    slot.state = SlotState::Free;
                    self.samples.push((slot.drawn, ticks[1].wrapping_sub(ticks[0]) as f64 * self.ns_per_tick / 1e6));
                }
                _ => {}
            }
        }
    }

    fn begin(&mut self) -> Option<usize> {
        let k = self.next;
        if self.slots[k].state != SlotState::Free {
            self.skipped += 1;
            return None;
        }
        self.next = (k + 1) % RING;
        Some(k)
    }

    fn end(&mut self, encoder: &mut wgpu::CommandEncoder, k: usize, drawn: u32) {
        let at = RESOLVE_STRIDE * k as u64;
        encoder.resolve_query_set(&self.set, 2 * k as u32..2 * k as u32 + 2, &self.resolve, at);
        encoder.copy_buffer_to_buffer(&self.resolve, at, &self.slots[k].readback, 0, 16);
        self.slots[k].state = SlotState::Recorded;
        self.slots[k].drawn = drawn;
    }

    /// Reads every slot still in flight (after the run's last submit).
    fn drain(&mut self, device: &wgpu::Device) {
        for _ in 0..2 {
            self.collect();
            device.poll(wgpu::PollType::wait_indefinitely()).ok();
        }
        self.collect();
    }
}

/// Mean (first sample dropped as warm-up, as Rune's own line does) and percentiles.
struct Stats { n: usize, mean: f64, p50: f64, p95: f64, max: f64 }

impl Stats {
    fn of(samples: &[f64]) -> Stats {
        Stats::all(if samples.len() > 1 { &samples[1..] } else { samples })
    }

    /// Every sample, warm-up included.
    fn all(samples: &[f64]) -> Stats {
        let mut v = samples.to_vec();
        v.sort_by(f64::total_cmp);
        let q = |p: f64| if v.is_empty() { 0.0 } else { v[((v.len() - 1) as f64 * p).round() as usize] };
        let mean = if v.is_empty() { 0.0 } else { v.iter().sum::<f64>() / v.len() as f64 };
        Stats { n: v.len(), mean, p50: q(0.5), p95: q(0.95), max: q(1.0) }
    }

    fn line(&self) -> String {
        format!("mean {:.3} ms p50 {:.3} p95 {:.3} max {:.3} (n={})", self.mean, self.p50, self.p95, self.max, self.n)
    }
}
