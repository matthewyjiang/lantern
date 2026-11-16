//! Robot models: a URDF with its meshes, loaded into one vertex list the viewport draws per
//! part, and posed by joint positions (`Robot::link_poses`).
//!
//! Mesh filenames resolve like RViz's: relative to the URDF's directory, absolute, `file://`, or
//! `package://pkg/...`, where pkg is found in `ROS_PACKAGE_PATH`, `AMENT_PREFIX_PATH`
//! (`<prefix>/share/pkg`), or as a directory named pkg at or above the URDF (a source checkout).
//! Meshes are STL; a visual Lantern can't draw is listed in `Model::problems` and skipped.

mod mesh;
mod urdf;

use std::ops::Range;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use urdf::Geometry;
pub use urdf::Robot;

use crate::ui::thousands;

/// Light grey, for visuals with no material.
const DEFAULT_COLOR: [f32; 4] = [0.8, 0.8, 0.8, 1.0];

pub struct Model {
    pub robot: Robot,
    pub mesh: Arc<RobotMesh>,
    /// Visuals not drawn, and why.
    pub problems: Vec<String>,
    /// Read, parse and shade, wall time.
    pub load_ms: f64,
}

/// Every visual's triangles in its link's frame (visual origin and scale applied), position then
/// normal per corner, three corners per triangle.
pub struct RobotMesh {
    pub vertices: Vec<[f32; 6]>,
    pub parts: Vec<Part>,
}

/// One visual: a run of `vertices` drawn with its link's pose.
pub struct Part {
    pub link: usize,
    pub range: Range<u32>,
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
    let mut vertices: Vec<[f32; 6]> = Vec::new();
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
            Geometry::Primitive(shape) => mesh::primitive(shape),
        };
        // Checked before shading (which costs memory per vertex): shading only drops triangles,
        // so this is the most the visual can add.
        let at_most = vertices.len() + 3 * tris.len();
        if at_most > max_vertices {
            return Err(format!(
                "{} brings the robot to {} mesh vertices, over the {} the GPU can draw",
                robot.links[v.link], thousands(at_most), thousands(max_vertices),
            ));
        }
        let m = v.origin.matrix();
        let tris: Vec<mesh::Triangle> = tris.into_iter().map(|t| t.map(|p| m.transform_point3(p))).collect();
        let start = vertices.len();
        vertices.extend(mesh::shade(&tris));
        // A visual with no area (an empty STL) draws nothing; it gets no part.
        if vertices.len() > start {
            parts.push(Part { link: v.link, range: start as u32..vertices.len() as u32, color: v.color.unwrap_or(DEFAULT_COLOR) });
        }
    }
    Ok(Model { robot, mesh: Arc::new(RobotMesh { vertices, parts }), problems, load_ms: started.elapsed().as_secs_f64() * 1000.0 })
}

fn read_mesh(filename: &str, dir: &Path) -> Result<Vec<mesh::Triangle>, String> {
    let path = resolve(filename, dir)?;
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    if ext != "stl" {
        return Err(format!("{filename}: .{ext} meshes aren't supported (Lantern reads STL)"));
    }
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    mesh::stl(&bytes).map_err(|e| format!("{}: {e}", path.display()))
}

/// Where a URDF mesh filename points, given the URDF's directory.
fn resolve(filename: &str, dir: &Path) -> Result<PathBuf, String> {
    let Some(rest) = filename.strip_prefix("package://") else {
        return Ok(dir.join(filename.strip_prefix("file://").unwrap_or(filename)));
    };
    let (pkg, rel) = rest.split_once('/').ok_or_else(|| format!("{filename}: no path after the package name"))?;
    let env = |var: &str| std::env::var_os(var).map(|v| std::env::split_paths(&v).collect::<Vec<_>>()).unwrap_or_default();
    let mut roots: Vec<PathBuf> = env("ROS_PACKAGE_PATH").into_iter().map(|p| p.join(pkg)).collect();
    roots.extend(env("AMENT_PREFIX_PATH").into_iter().map(|p| p.join("share").join(pkg)));
    for up in dir.ancestors() {
        if up.file_name().is_some_and(|n| n == pkg) {
            roots.push(up.to_path_buf());
        }
        roots.push(up.join(pkg));
    }
    roots.into_iter().map(|r| r.join(rel)).find(|p| p.exists())
        .ok_or_else(|| format!("{filename}: package {pkg} not found in ROS_PACKAGE_PATH, AMENT_PREFIX_PATH, or at or above {}", dir.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    // A description package checked out from source: robot.urdf in pkg/urdf/, meshes under
    // pkg/meshes/, referenced as package://pkg/meshes/...
    #[test]
    fn package_urls_resolve_to_an_enclosing_checkout() {
        let root = std::env::temp_dir().join(format!("lantern-resolve-{}", std::process::id()));
        let pkg = root.join("arm_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        std::fs::write(pkg.join("meshes/base.stl"), b"").unwrap();
        let found = resolve("package://arm_description/meshes/base.stl", &pkg.join("urdf"));
        let missing = resolve("package://other_pkg/meshes/base.stl", &pkg.join("urdf"));
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(found.unwrap(), pkg.join("meshes/base.stl"));
        assert!(missing.unwrap_err().contains("package other_pkg not found"));
    }
}
