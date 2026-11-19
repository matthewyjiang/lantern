//! Robot models: a URDF with its meshes, loaded into one vertex list the viewport draws per
//! part, and posed by joint positions (`Robot::link_poses`).
//!
//! Mesh filenames resolve like RViz's (`mesh::resolve`), relative to the URDF's directory.
//! Meshes are STL; a visual Lantern can't draw is listed in `Model::problems` and skipped.

mod urdf;

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use glam::Vec3;
use urdf::{Geometry, Shape};
pub use urdf::Robot;

use crate::mesh::{self, Mesh};
use crate::ui::thousands;

pub struct Model {
    pub robot: Robot,
    /// Every visual's triangles in its link's frame (visual origin and scale applied), one mesh
    /// part per visual.
    pub mesh: Arc<Mesh>,
    /// What each of the mesh's parts is: its link and colour.
    pub parts: Vec<Part>,
    /// Visuals not drawn, and why.
    pub problems: Vec<String>,
    /// Read, parse and shade, wall time.
    pub load_ms: f64,
}

/// One visual, drawn with its link's pose.
pub struct Part {
    pub link: usize,
    /// sRGB and alpha.
    pub color: [f32; 4],
}

/// Loads a URDF file and its meshes. Errors are about the URDF itself (unreadable, not URDF, a
/// broken tree, too big to draw); a mesh that fails is a problem, and the rest still draws.
pub fn load(path: &Path, max_vertices: usize) -> Result<Model, String> {
    let started = Instant::now();
    let xml = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    build(&xml, path.parent().unwrap_or(Path::new(".")), max_vertices, started)
}

/// Loads a URDF received as text (a `robot_description` topic). Mesh paths must be absolute,
/// `file://` or `package://`: relative ones resolve against the working directory, which is
/// rarely where the sender meant.
pub fn load_text(xml: &str, max_vertices: usize) -> Result<Model, String> {
    build(xml, Path::new("."), max_vertices, Instant::now())
}

fn build(xml: &str, dir: &Path, max_vertices: usize, started: Instant) -> Result<Model, String> {
    let robot = Robot::parse(xml)?;
    let mut out = Mesh::default();
    let mut parts = Vec::new();
    let mut problems = Vec::new();
    for v in &robot.visuals {
        let tris = match &v.geometry {
            Geometry::Mesh { filename, scale } => match read_mesh(filename, dir) {
                Ok(t) => t.into_iter().map(|tri| tri.map(|p| p * scale.as_vec3())).collect(),
                Err(e) => {
                    problems.push(format!("{}: {e}", robot.links[v.link]));
                    continue;
                }
            },
            Geometry::Primitive(Shape::Box { size }) => mesh::cuboid(size.as_vec3()),
            Geometry::Primitive(Shape::Cylinder { radius, length }) => mesh::frustum(*radius as f32, *radius as f32, *length as f32),
            Geometry::Primitive(Shape::Sphere { radius }) => mesh::sphere(*radius as f32),
        };
        // Checked before shading (which costs memory per vertex): shading only drops triangles,
        // so this is the most the visual can add.
        let at_most = out.vertices.len() + 3 * tris.len();
        if at_most > max_vertices {
            return Err(format!(
                "{} brings the robot to {} mesh vertices, over the {} the GPU can draw",
                robot.links[v.link], thousands(at_most), thousands(max_vertices),
            ));
        }
        let m = v.origin.matrix();
        let tris: Vec<mesh::Triangle> = tris.into_iter().map(|t| t.map(|p: Vec3| m.transform_point3(p))).collect();
        let start = out.vertices.len();
        out.vertices.extend(mesh::shade(&tris));
        // A visual with no area (an empty STL) draws nothing; it gets no part.
        if out.vertices.len() > start {
            out.parts.push(start as u32..out.vertices.len() as u32);
            parts.push(Part { link: v.link, color: v.color.unwrap_or(mesh::DEFAULT_COLOR) });
        }
    }
    Ok(Model { robot, mesh: Arc::new(out), parts, problems, load_ms: started.elapsed().as_secs_f64() * 1000.0 })
}

fn read_mesh(filename: &str, dir: &Path) -> Result<Vec<mesh::Triangle>, String> {
    let path = mesh::resolve(filename, dir)?;
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    if ext != "stl" {
        return Err(format!("{filename}: .{ext} meshes aren't supported (Lantern reads STL)"));
    }
    let bytes = mesh::read_file(&path)?;
    mesh::stl(&bytes).map_err(|e| format!("{}: {e}", path.display()))
}
