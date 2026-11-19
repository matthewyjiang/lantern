//! Objects in the 3D view: the entities `SceneUpdate` topics send and the meshes the layout
//! attaches to TF frames. Shapes with a surface become the parts of one mesh, each part in its
//! own frame (scale applied) and placed at draw time, so a part that only moves keeps the mesh;
//! lines join the TF lines and text the labels.

use std::collections::HashMap;
use std::f32::consts::FRAC_PI_2;
use std::path::Path;
use std::sync::{Arc, LazyLock};
use std::time::Instant;

use glam::{Mat4, Quat, Vec3};

use crate::mesh::{self, Asset, Mesh, Vertex};
use crate::scene::{Geometry, LineKind, ModelSource, Pose, Shape, Thickness};
use crate::viewport::{Lines, PartDraw};

/// What names a loaded model: its URL or file path, or the hash of the bytes a message sent.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum ModelKey {
    Url(String),
    Data(u64),
}

impl ModelKey {
    fn of(source: &ModelSource) -> ModelKey {
        match source {
            ModelSource::Url(url) => ModelKey::Url(url.clone()),
            ModelSource::Data { hash, .. } => ModelKey::Data(*hash),
        }
    }
}

/// The models that objects and the layout's meshes name, each loaded once, or why it can't be.
#[derive(Default)]
pub struct Models(HashMap<ModelKey, Result<Arc<Asset>, String>>);

impl Models {
    /// Loads the `named` models not loaded yet, and drops the ones no longer named. Loads run
    /// on the calling (UI) thread: a model is read when something first names it.
    pub fn sync(&mut self, named: Vec<ModelSource>) {
        let named: HashMap<ModelKey, ModelSource> = named.into_iter().map(|s| (ModelKey::of(&s), s)).collect();
        self.0.retain(|key, _| named.contains_key(key));
        for (key, source) in named {
            if self.0.contains_key(&key) {
                continue;
            }
            let started = Instant::now();
            let (what, loaded) = match &source {
                ModelSource::Url(url) => (url.clone(), mesh::resolve(url, Path::new(".")).and_then(|p| mesh::model_file(&p))),
                ModelSource::Data { media_type, bytes, .. } => (format!("{} bytes of {media_type}", bytes.len()), mesh::model(bytes, media_type, "", None)),
            };
            match &loaded {
                Ok(a) => eprintln!("lantern: loaded model {what} in {:.1} ms: {} pieces, {} vertices", started.elapsed().as_secs_f64() * 1000.0, a.pieces.len(), a.vertex_count()),
                Err(e) => eprintln!("lantern: can't load model {what}: {e}"),
            }
            self.0.insert(key, loaded.map(Arc::new));
        }
    }

    /// A model's load, once something has named it.
    pub fn get(&self, source: &ModelSource) -> Option<&Result<Arc<Asset>, String>> {
        self.0.get(&ModelKey::of(source))
    }
}

/// A loaded model, equal only to itself: the same load builds the same mesh.
#[derive(Clone, Debug)]
pub struct AssetRef(pub Arc<Asset>);

impl PartialEq for AssetRef {
    fn eq(&self, other: &AssetRef) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

/// One part of the objects' mesh, in its own frame. Equal geometry builds an equal mesh.
#[derive(Clone, Debug, PartialEq)]
pub enum Geom {
    /// The unit shapes scaled along their axes.
    Cube(Vec3),
    Sphere(Vec3),
    Cylinder { size: Vec3, bottom: f32, top: f32 },
    Arrow { shaft_length: f32, shaft_diameter: f32, head_length: f32, head_diameter: f32 },
    Triangles(Vec<Vec3>),
    /// Piece `piece` of a loaded model, scaled.
    Model { asset: AssetRef, piece: usize, scale: Vec3 },
}

/// The unit shapes, shaded once: a 1 m cube, and a sphere, cylinder and cone 1 m across (the
/// cylinder and cone 1 m tall along z, the cone's apex at +z).
static CUBE: LazyLock<Vec<Vertex>> = LazyLock::new(|| mesh::shade(&mesh::cuboid(Vec3::ONE)));
static SPHERE: LazyLock<Vec<Vertex>> = LazyLock::new(|| mesh::shade(&mesh::sphere(0.5)));
static CYLINDER: LazyLock<Vec<Vertex>> = LazyLock::new(|| mesh::shade(&mesh::frustum(0.5, 0.5, 1.0)));
static CONE: LazyLock<Vec<Vertex>> = LazyLock::new(|| mesh::shade(&mesh::frustum(0.5, 0.0, 1.0)));

/// `vertices` turned by `q` and moved by `t`.
fn moved(vertices: impl Iterator<Item = Vertex>, q: Quat, t: Vec3) -> impl Iterator<Item = Vertex> {
    vertices.map(move |v| {
        let p = q * Vec3::new(v[0], v[1], v[2]) + t;
        let n = q * Vec3::new(v[3], v[4], v[5]);
        [p.x, p.y, p.z, n.x, n.y, n.z]
    })
}

impl Geom {
    fn vertices(&self) -> Vec<Vertex> {
        match self {
            Geom::Cube(s) => mesh::scaled(&CUBE, *s).collect(),
            Geom::Sphere(s) => mesh::scaled(&SPHERE, *s).collect(),
            Geom::Cylinder { size, bottom: 1.0, top: 1.0 } => mesh::scaled(&CYLINDER, *size).collect(),
            Geom::Cylinder { size, bottom, top } => mesh::scaled(&mesh::shade(&mesh::frustum(0.5 * bottom, 0.5 * top, 1.0)), *size).collect(),
            Geom::Arrow { shaft_length, shaft_diameter, head_length, head_diameter } => {
                // Built along z, then turned onto +x.
                let x = Quat::from_rotation_y(FRAC_PI_2);
                let shaft = mesh::scaled(&CYLINDER, Vec3::new(*shaft_diameter, *shaft_diameter, *shaft_length));
                let head = mesh::scaled(&CONE, Vec3::new(*head_diameter, *head_diameter, *head_length));
                moved(shaft, x, Vec3::X * shaft_length * 0.5).chain(moved(head, x, Vec3::X * (shaft_length + head_length * 0.5))).collect()
            }
            Geom::Triangles(points) => {
                let tris: Vec<mesh::Triangle> = points.chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect();
                mesh::shade(&tris)
            }
            Geom::Model { asset, piece, scale } => mesh::scaled(&asset.0.pieces[*piece].vertices, *scale).collect(),
        }
    }
}

/// The mesh of `geoms`, one part each, in order. Once `max_vertices` would be passed, the rest
/// are left empty, and the second value names the vertices asked for, the limit, and how many
/// aren't drawn.
pub fn build(geoms: &[Geom], max_vertices: usize) -> (Mesh, Option<String>) {
    let mut out = Mesh::default();
    let mut skipped = 0;
    for g in geoms {
        let start = out.vertices.len() as u32;
        let v = g.vertices();
        if skipped > 0 || out.vertices.len() + v.len() > max_vertices {
            skipped += v.len();
        } else {
            out.vertices.extend(v);
        }
        out.parts.push(start..out.vertices.len() as u32);
    }
    let problem = (skipped > 0).then(|| {
        use crate::ui::thousands;
        let total = out.vertices.len() + skipped;
        format!("Objects need {} mesh vertices, over the {} the GPU can draw; {} aren't drawn", thousands(total), thousands(max_vertices), thousands(skipped))
    });
    (out, problem)
}

/// A text label in the view: its colour (sRGB and alpha) and height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LabelStyle {
    pub color: [f32; 4],
    pub size: Thickness,
}

/// What placing shapes produced: mesh parts with their placement and colour, and the lines and
/// labels they add to the view's.
#[derive(Default)]
pub struct Placed {
    pub geoms: Vec<Geom>,
    pub draws: Vec<PartDraw>,
    pub labels: Vec<(String, Vec3, LabelStyle)>,
}

impl Placed {
    fn part(&mut self, geom: Geom, at: Mat4, color: [f32; 4]) {
        self.geoms.push(geom);
        self.draws.push(PartDraw { model: at, color });
    }

    /// Places a model's pieces at `at`, `scale`d, each in `color` or else its own.
    pub fn model(&mut self, asset: &Arc<Asset>, at: Mat4, scale: Vec3, color: Option<[f32; 4]>) {
        for (k, piece) in asset.pieces.iter().enumerate() {
            let color = color.or(piece.color).unwrap_or(mesh::DEFAULT_COLOR);
            self.part(Geom::Model { asset: AssetRef(Arc::clone(asset)), piece: k, scale }, at, color);
        }
    }

    /// Places `shape` at `at` (`fixed_T_shape`), its model from `models`. A model that isn't
    /// loaded isn't drawn; the error is returned when it failed to load.
    pub fn shape<'m>(&mut self, shape: &Shape, at: Pose, lines: &mut Lines, models: &'m Models) -> Option<&'m str> {
        let m = at.matrix();
        let f = |x: f64| x as f32;
        match &shape.geometry {
            Geometry::Cube { size } => self.part(Geom::Cube(size.as_vec3()), m, shape.color),
            Geometry::Sphere { size } => self.part(Geom::Sphere(size.as_vec3()), m, shape.color),
            Geometry::Cylinder { size, bottom, top } => self.part(Geom::Cylinder { size: size.as_vec3(), bottom: f(*bottom), top: f(*top) }, m, shape.color),
            Geometry::Arrow { shaft_length, shaft_diameter, head_length, head_diameter } => {
                let geom = Geom::Arrow { shaft_length: f(*shaft_length), shaft_diameter: f(*shaft_diameter), head_length: f(*head_length), head_diameter: f(*head_diameter) };
                self.part(geom, m, shape.color);
            }
            Geometry::Triangles { points } => self.part(Geom::Triangles(points.iter().map(|p| p.as_vec3()).collect()), m, shape.color),
            Geometry::Model { source, scale, override_color } => match models.get(source)? {
                Ok(asset) => self.model(asset, m, scale.as_vec3(), override_color.then_some(shape.color)),
                Err(e) => return Some(e),
            },
            Geometry::Line { kind, points, colors, thickness } => {
                let pts: Vec<Vec3> = points.iter().map(|p| m.transform_point3(p.as_vec3())).collect();
                let color = |i: usize| colors.get(i).copied().unwrap_or(shape.color);
                let mut segment = |i: usize, j: usize| match *thickness {
                    Thickness::Points(w) => lines.segment(pts[i], pts[j], w as f32, color(i)),
                    Thickness::Metres(w) => lines.segment_metres(pts[i], pts[j], w as f32, color(i)),
                };
                let n = pts.len();
                match kind {
                    LineKind::Strip | LineKind::Loop => (1..n).for_each(|j| segment(j - 1, j)),
                    LineKind::List => (0..n / 2).for_each(|k| segment(2 * k, 2 * k + 1)),
                }
                if *kind == LineKind::Loop && n > 2 {
                    segment(n - 1, 0);
                }
            }
            Geometry::Text { text, size } => self.labels.push((text.clone(), at.translation.as_vec3(), LabelStyle { color: shape.color, size: *size })),
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;

    fn shape(geometry: Geometry) -> Shape {
        Shape { pose: Pose::IDENTITY, color: [1.0; 4], geometry }
    }

    fn bounds(v: &[Vertex]) -> (Vec3, Vec3) {
        v.iter().fold((Vec3::INFINITY, Vec3::NEG_INFINITY), |(lo, hi), v| (lo.min(Vec3::new(v[0], v[1], v[2])), hi.max(Vec3::new(v[0], v[1], v[2]))))
    }

    // Each shape fills the box Foxglove's fields describe: a cube and sphere `size` across, a
    // cylinder along z, an arrow from its pose along +x, its head as wide as `head_diameter`.
    #[test]
    fn shapes_fill_the_boxes_their_fields_describe() {
        let close = |(lo, hi): (Vec3, Vec3), want: (Vec3, Vec3)| (lo - want.0).abs().max_element() < 1e-5 && (hi - want.1).abs().max_element() < 1e-5;
        let size = Vec3::new(0.1, 0.2, 0.3);
        assert!(close(bounds(&Geom::Cube(size).vertices()), (-size / 2.0, size / 2.0)));
        assert!(close(bounds(&Geom::Sphere(size).vertices()), (-size / 2.0, size / 2.0)));
        assert!(close(bounds(&Geom::Cylinder { size, bottom: 1.0, top: 1.0 }.vertices()), (-size / 2.0, size / 2.0)));
        let arrow = Geom::Arrow { shaft_length: 0.3, shaft_diameter: 0.02, head_length: 0.1, head_diameter: 0.05 };
        assert!(close(bounds(&arrow.vertices()), (Vec3::new(0.0, -0.025, -0.025), Vec3::new(0.4, 0.025, 0.025))), "{:?}", bounds(&arrow.vertices()));
        // A stretched sphere's normals still point out of its surface.
        let egg = Geom::Sphere(Vec3::new(1.0, 1.0, 4.0)).vertices();
        assert!(egg.iter().all(|v| {
            let (p, n) = (Vec3::new(v[0], v[1], v[2]), Vec3::new(v[3], v[4], v[5]));
            // The ellipsoid's gradient at p.
            n.dot((p / Vec3::new(0.25, 0.25, 4.0)).normalize()) > 0.99
        }));
    }

    // A shape lands where its frame's pose and its own pose put it, a line's points too, and a
    // loop closes back to its first point.
    #[test]
    fn placement_composes_the_frame_and_the_shape_pose() {
        let frame = Pose { translation: DVec3::new(1.0, 0.0, 0.0), rotation: glam::DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2) };
        let mut cube = shape(Geometry::Cube { size: DVec3::ONE });
        cube.pose.translation = DVec3::new(0.5, 0.0, 0.0);
        let (mut placed, mut lines) = (Placed::default(), Lines::default());
        placed.shape(&cube, frame.then(&cube.pose), &mut lines, &Models::default());
        let centre = placed.draws[0].model.transform_point3(Vec3::ZERO);
        assert!((centre - Vec3::new(1.0, 0.5, 0.0)).length() < 1e-6, "0.5 m along the frame's x, which is fixed y: {centre}");

        let square = [DVec3::ZERO, DVec3::X, DVec3::new(1.0, 1.0, 0.0), DVec3::Y];
        let line = shape(Geometry::Line { kind: LineKind::Loop, points: square.to_vec(), colors: vec![], thickness: Thickness::Points(2.0) });
        placed.shape(&line, frame, &mut lines, &Models::default());
        assert_eq!(lines.len(), 4, "a loop of four points is four segments");
        let list = shape(Geometry::Line { kind: LineKind::List, points: square.to_vec(), colors: vec![], thickness: Thickness::Metres(0.01) });
        placed.shape(&list, frame, &mut lines, &Models::default());
        assert_eq!(lines.len(), 6, "a list of four points is two");
    }

    // Parts that would pass the vertex limit are left empty, not dropped, so every placement
    // still matches its part; the problem names what was asked, the limit and what's missing.
    #[test]
    fn parts_past_the_vertex_limit_stay_empty() {
        let geoms = vec![Geom::Cube(Vec3::ONE), Geom::Cube(Vec3::ONE), Geom::Cube(Vec3::ONE)];
        let (mesh, problem) = build(&geoms, 80);
        assert_eq!(mesh.parts, [0..36, 36..72, 72..72]);
        assert_eq!(problem.unwrap(), "Objects need 108 mesh vertices, over the 80 the GPU can draw; 36 aren't drawn");
    }
}
