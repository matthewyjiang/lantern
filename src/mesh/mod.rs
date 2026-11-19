//! Triangle meshes for the viewport: read from STL and glTF files, made from shapes, and shaded
//! with smoothed normals. The robot's URDF visuals and the scene's objects both come through here.
//!
//! A file named in a URDF or a message resolves like RViz's: relative to a base directory,
//! absolute, `file://`, or `package://pkg/...`, where pkg is found in `ROS_PACKAGE_PATH`,
//! `AMENT_PREFIX_PATH` (`<prefix>/share/pkg`), or as a directory named pkg at or above the base
//! (a source checkout).

mod gltf;

use std::collections::HashMap;
use std::f32::consts::{FRAC_PI_2, TAU};
use std::ops::Range;
use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};

pub type Triangle = [Vec3; 3];

/// What a part with no colour of its own is drawn in: light grey, sRGB and alpha.
pub const DEFAULT_COLOR: [f32; 4] = [0.8, 0.8, 0.8, 1.0];

/// Position and normal of one corner, three corners per triangle.
pub type Vertex = [f32; 6];

/// Shaded triangles drawn in parts, each placed and coloured on its own when drawn.
#[derive(Default)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub parts: Vec<Range<u32>>,
}

/// Edges sharper than this stay sharp; softer ones shade smooth. 40° keeps a box's edges and a
/// cylinder's rim crisp while a 32-sided cylinder (11° per side) and tessellated castings, which
/// is most of a robot arm, read as curved. STL normals are ignored: exporters often zero them.
const CREASE_DEG: f32 = 40.0;

/// An STL file's triangles. Binary when its size is exactly 84 + 50 × the count it declares
/// (exporters write "solid" at the start of binary headers too); ASCII when it starts with
/// "solid" and is text; else binary with bytes past the triangles (some exporters pad), as
/// assimp reads it.
pub fn stl(bytes: &[u8]) -> Result<Vec<Triangle>, String> {
    let declared = (bytes.len() >= 84).then(|| u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize);
    let binary_len = declared.and_then(|n| n.checked_mul(50)?.checked_add(84));
    let ascii = bytes.starts_with(b"solid") && std::str::from_utf8(bytes).is_ok();
    match (declared, binary_len) {
        (Some(n), Some(len)) if len == bytes.len() || (!ascii && len < bytes.len()) => {
            let f = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
            return Ok((0..n).map(|k| {
                let at = 84 + 50 * k + 12; // past the facet normal
                [0, 1, 2].map(|c| Vec3::new(f(at + 12 * c), f(at + 12 * c + 4), f(at + 12 * c + 8)))
            }).collect());
        }
        _ if !ascii => {
            return Err(format!("not an STL file: {} bytes, too short for the {} triangles a binary STL header declares, and not ASCII", bytes.len(), declared.unwrap_or(0)));
        }
        _ => {}
    }
    let text = std::str::from_utf8(bytes).unwrap();
    let mut corners = Vec::new();
    let mut words = text.split_ascii_whitespace();
    while let Some(w) = words.next() {
        if w == "vertex" {
            let mut c = [0.0f32; 3];
            for v in &mut c {
                *v = words.next().and_then(|s| s.parse().ok()).ok_or("ASCII STL vertex without three numbers")?;
            }
            corners.push(Vec3::from(c));
        }
    }
    if corners.len() % 3 != 0 {
        return Err(format!("ASCII STL with {} vertices, not a multiple of three", corners.len()));
    }
    Ok(corners.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect())
}

fn quad(out: &mut Vec<Triangle>, a: Vec3, b: Vec3, c: Vec3, d: Vec3) {
    out.extend([[a, b, c], [a, c, d]]);
}

/// A box `size` across, centred on the origin, wound outward.
pub fn cuboid(size: Vec3) -> Vec<Triangle> {
    let mut out = Vec::new();
    let h = size * 0.5;
    for axis in 0..3 {
        for sign in [-1.0f32, 1.0] {
            let (u, v) = (Vec3::AXES[(axis + 1) % 3] * h, Vec3::AXES[(axis + 2) % 3] * h);
            let n = Vec3::AXES[axis] * h * sign;
            let (u, v) = if sign > 0.0 { (u, v) } else { (v, u) };
            quad(&mut out, n - u - v, n + u - v, n + u + v, n - u + v);
        }
    }
    out
}

/// A cylinder along z, `length` long and centred on the origin, with radius `bottom` at -z and
/// `top` at +z (one of them 0 is a cone: its cap and the side's apex triangles have no area and
/// shade away).
pub fn frustum(bottom: f32, top: f32, length: f32) -> Vec<Triangle> {
    const SIDES: usize = 32;
    let mut out = Vec::new();
    let z = length * 0.5;
    // Wrapped like the sphere's seam, so the last side meets the first exactly.
    let rim = |k: usize, r: f32, z: f32| {
        let a = (k % SIDES) as f32 / SIDES as f32 * TAU;
        Vec3::new(a.cos() * r, a.sin() * r, z)
    };
    for k in 0..SIDES {
        let (a, b) = (rim(k, bottom, -z), rim(k + 1, bottom, -z));
        let (c, d) = (rim(k + 1, top, z), rim(k, top, z));
        quad(&mut out, a, b, c, d);
        out.push([Vec3::Z * z, d, c]);
        out.push([-Vec3::Z * z, b, a]);
    }
    out
}

pub fn sphere(radius: f32) -> Vec<Triangle> {
    const RINGS: usize = 16;
    const SEGMENTS: usize = 32;
    let mut out = Vec::new();
    let at = |i: usize, k: usize| {
        // Exact poles: f32 sin(π) isn't 0, and corners that don't meet bit for bit
        // become slivers with arbitrary normals.
        match i {
            0 => return Vec3::Z * radius,
            RINGS => return -Vec3::Z * radius,
            _ => {}
        }
        // k wraps to 0, not TAU: f32 cos/sin of TAU miss 1 and 0, and the seam wouldn't weld.
        let (lat, lon) = (i as f32 / RINGS as f32 * TAU * 0.5, (k % SEGMENTS) as f32 / SEGMENTS as f32 * TAU);
        Vec3::new(lat.sin() * lon.cos(), lat.sin() * lon.sin(), lat.cos()) * radius
    };
    for i in 0..RINGS {
        for k in 0..SEGMENTS {
            quad(&mut out, at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
        }
    }
    out
}

/// Shaded vertices stretched by `s` along each axis, their normals turned to match (by the
/// inverse transpose, which for a diagonal scale is the cofactors `(sy·sz, sx·sz, sx·sy)`: no
/// division, so a flat axis still has normals).
pub fn scaled(vertices: &[Vertex], s: Vec3) -> impl Iterator<Item = Vertex> + '_ {
    let cof = Vec3::new(s.y * s.z, s.x * s.z, s.x * s.y);
    vertices.iter().map(move |v| {
        let p = Vec3::new(v[0], v[1], v[2]) * s;
        let n = Vec3::new(v[3], v[4], v[5]);
        let n = (n * cof).try_normalize().unwrap_or(n);
        [p.x, p.y, p.z, n.x, n.y, n.z]
    })
}

/// Position and normal per corner, three per triangle. A corner's normal averages the normals
/// of the triangles that share its position and meet its own within the crease angle, each
/// weighted by its angle at that position (Thürmer and Wüthrich): unlike area weights, that
/// doesn't let one long CAD sliver or a coarse neighbour tilt the result. Zero-area triangles
/// (CAD slivers, a cone's apex) cover no pixels and are dropped.
pub fn shade(tris: &[Triangle]) -> Vec<[f32; 6]> {
    // Exact positions: STL repeats a shared corner's floats bit for bit. `+ 0.0` folds -0.0 in.
    let key = |p: Vec3| [p.x + 0.0, p.y + 0.0, p.z + 0.0].map(f32::to_bits);
    let face: Vec<Option<Vec3>> = tris.iter().map(|t| (t[1] - t[0]).cross(t[2] - t[0]).try_normalize()).collect();
    let angle = |t: &Triangle, c: usize| (t[(c + 1) % 3] - t[c]).angle_between(t[(c + 2) % 3] - t[c]);
    // Every (unit normal, corner angle) at each position.
    let mut around: HashMap<[u32; 3], Vec<(Vec3, f32)>> = HashMap::with_capacity(tris.len() / 2);
    for (t, n) in tris.iter().zip(&face) {
        let Some(n) = n else { continue };
        for c in 0..3 {
            around.entry(key(t[c])).or_default().push((*n, angle(t, c)));
        }
    }
    let cos_crease = CREASE_DEG.to_radians().cos();
    let mut out = Vec::with_capacity(tris.len() * 3);
    for (t, own) in tris.iter().zip(&face) {
        let Some(own) = *own else { continue };
        for p in t {
            let n: Vec3 = around[&key(*p)].iter().filter(|(g, _)| g.dot(own) >= cos_crease).map(|(g, w)| *g * *w).sum();
            let n = n.try_normalize().unwrap_or(own);
            out.push([p.x, p.y, p.z, n.x, n.y, n.z]);
        }
    }
    out
}

/// Where a mesh filename points, given the directory a relative one is relative to.
pub fn resolve(filename: &str, dir: &Path) -> Result<PathBuf, String> {
    let Some(rest) = filename.strip_prefix("package://") else {
        if let Some((scheme, _)) = filename.split_once("://").filter(|(s, _)| *s != "file") {
            return Err(format!("{filename}: {scheme}:// isn't read; use a path, file:// or package://"));
        }
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

/// A model file's triangles, shaded, in pieces of one material colour each.
#[derive(Debug)]
pub struct Asset {
    pub pieces: Vec<Piece>,
}

#[derive(Debug)]
pub struct Piece {
    pub vertices: Vec<Vertex>,
    /// sRGB and alpha; `None` when the file gives none (STL).
    pub color: Option<[f32; 4]>,
}

impl Asset {
    pub fn vertex_count(&self) -> usize {
        self.pieces.iter().map(|p| p.vertices.len()).sum()
    }
}

/// Media types that name each format, as Foxglove reads them.
const GLTF_TYPES: [&str; 3] = ["model/gltf-binary", "model/gltf+json", "model/gltf"];
const STL_TYPES: [&str; 3] = ["model/stl", "model/x.stl-ascii", "model/x.stl-binary"];

/// Reads a model as Foxglove does: GLB or glTF when the bytes start with GLB's magic, the media
/// type says so or `name` ends in `.glb` or `.gltf`; STL by media type or `.stl`. Both are taken
/// as Y-up and turned to Lantern's Z-up. `dir` is where a glTF's external buffers are, when it
/// came from a file. A model of more than `max_vertices` is an error, found before shading it.
pub fn model(bytes: &[u8], media_type: &str, name: &str, dir: Option<&Path>, max_vertices: usize) -> Result<Asset, String> {
    let ext = Path::new(name).extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    let pieces = if bytes.starts_with(b"glTF") || GLTF_TYPES.contains(&media_type) || ext == "glb" || ext == "gltf" {
        gltf::read(bytes, dir, max_vertices)?
    } else if STL_TYPES.contains(&media_type) || ext == "stl" {
        vec![(stl(bytes)?, None)]
    } else {
        let what = if media_type.is_empty() { String::new() } else { format!(" ({media_type})") };
        return Err(format!("{} bytes{what}: not a GLB, glTF or STL mesh", bytes.len()));
    };
    let vertices: usize = pieces.iter().map(|(tris, _)| 3 * tris.len()).sum();
    if vertices > max_vertices {
        return Err(format!("{} vertices, more than the {} the GPU can draw", crate::ui::thousands(vertices), crate::ui::thousands(max_vertices)));
    }
    let up = Quat::from_rotation_x(FRAC_PI_2);
    let pieces = pieces.into_iter().map(|(tris, color)| {
        let tris: Vec<Triangle> = tris.into_iter().map(|t| t.map(|p| up * p)).collect();
        Piece { vertices: shade(&tris), color }
    });
    Ok(Asset { pieces: pieces.filter(|p| !p.vertices.is_empty()).collect() })
}

/// Reads a model file (`model`, with the file's name and directory).
pub fn model_file(path: &Path, max_vertices: usize) -> Result<Asset, String> {
    let bytes = read_file(path)?;
    model(&bytes, "", &path.to_string_lossy(), path.parent(), max_vertices).map_err(|e| format!("{}: {e}", path.display()))
}

/// The largest mesh file read: as large as a model a message can carry.
pub const MAX_FILE_BYTES: u64 = crate::source::MAX_MESSAGE as u64;

/// A mesh file's bytes. Messages and URDFs name these paths, so only a regular file is read
/// (not a device or a pipe that would never end), and at most `MAX_FILE_BYTES` of it.
pub fn read_file(path: &Path) -> Result<Vec<u8>, String> {
    use std::io::Read;
    let fail = |e: std::io::Error| format!("{}: {e}", path.display());
    let file = std::fs::File::open(path).map_err(fail)?;
    if !file.metadata().map_err(fail)?.is_file() {
        return Err(format!("{}: not a regular file", path.display()));
    }
    let mut bytes = Vec::new();
    file.take(MAX_FILE_BYTES + 1).read_to_end(&mut bytes).map_err(fail)?;
    if bytes.len() as u64 > MAX_FILE_BYTES {
        return Err(format!("{}: over {} bytes, the most a mesh file can be", path.display(), crate::ui::thousands(MAX_FILE_BYTES as usize)));
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binary_stl(header: &[u8], tris: &[[[f32; 3]; 3]]) -> Vec<u8> {
        let mut b = header.to_vec();
        b.resize(80, b' ');
        b.extend((tris.len() as u32).to_le_bytes());
        for t in tris {
            b.extend([0u8; 12]);
            for c in t {
                for v in c {
                    b.extend(v.to_le_bytes());
                }
            }
            b.extend([0u8; 2]);
        }
        b
    }

    // SolidWorks and others start binary headers with "solid"; the size decides, not the text.
    #[test]
    fn binary_stl_with_a_solid_header_is_read_as_binary() {
        let tri = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]];
        let bytes = binary_stl(b"solid link_3 exported by a CAD tool", &[tri, tri]);
        let t = stl(&bytes).unwrap();
        assert_eq!(t.len(), 2);
        assert_eq!(t[1][2], Vec3::new(0.0, 2.0, 0.0));
        assert!(stl(&bytes[..bytes.len() - 1]).is_err(), "a truncated binary STL is an error, not ASCII");
        let mut padded = bytes.clone();
        padded.extend([0u8; 16]);
        assert_eq!(stl(&padded).unwrap().len(), 2, "bytes past the triangles are ignored");
    }

    #[test]
    fn ascii_stl() {
        let text = b"solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0.5\n  endloop\n endfacet\nendsolid t\n";
        assert_eq!(stl(text).unwrap(), [[Vec3::ZERO, Vec3::X, Vec3::new(0.0, 1.0, 0.5)]]);
    }

    fn corners(p: &Piece) -> Vec<Vec3> {
        p.vertices.iter().map(|v| Vec3::new(v[0], v[1], v[2])).collect()
    }

    // The same two-part model as GLB and as .gltf with its buffer inline: each part in its
    // material's colour (linear turned sRGB), placed by its node, all turned from glTF's Y-up to
    // Z-up.
    #[test]
    fn gltf_parts_are_placed_coloured_and_turned_z_up() {
        let glb = model(include_bytes!("testdata/two_parts.glb"), "model/gltf-binary", "", None, usize::MAX).unwrap();
        let gltf = model(include_bytes!("testdata/two_parts.gltf"), "", "two_parts.gltf", None, usize::MAX).unwrap();
        for asset in [glb, gltf] {
            let [red, grey] = &asset.pieces[..] else { panic!("two pieces") };
            let close = |c: Option<[f32; 4]>, want: [f32; 4], tol: f32| c.is_some_and(|c| c.iter().zip(want).all(|(a, b)| (a - b).abs() < tol));
            assert!(close(red.color, [1.0, 0.0, 0.0, 1.0], 1e-6), "{:?}", red.color);
            assert!(close(grey.color, [0.5, 0.5, 0.5, 0.5], 1e-3), "linear 0.214 is sRGB 0.5, alpha as is: {:?}", grey.color);
            let near = |ps: &[Vec3], q: Vec3| ps.iter().any(|p| (*p - q).length() < 1e-6);
            assert!(near(&corners(red), Vec3::Z), "glTF's +y is up");
            let grey = corners(grey);
            assert!(grey.iter().all(|p| (p.z - 1.0).abs() < 1e-6), "its node lifts it 1 m: {grey:?}");
            assert!(near(&grey, Vec3::new(0.0, -1.0, 1.0)), "glTF's +z is -y");
        }
    }

    // A model's STL is Y-up too (as Foxglove reads it); without a media type, its name says STL,
    // and bytes that are neither are an error naming the media type.
    #[test]
    fn model_stl_is_y_up_and_unknown_bytes_are_errors() {
        let bytes = binary_stl(b"", &[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]]);
        for (media_type, name) in [("model/stl", ""), ("", "cup.STL")] {
            let asset = model(&bytes, media_type, name, None, usize::MAX).unwrap();
            assert_eq!((asset.pieces.len(), asset.pieces[0].color), (1, None));
            assert!(corners(&asset.pieces[0]).iter().any(|p| (*p - Vec3::Z).length() < 1e-6));
        }
        let err = model(b"OBJ v 0 0 0", "model/obj", "", None, usize::MAX).err().unwrap();
        assert!(err.contains("(model/obj): not a GLB, glTF or STL mesh"), "{err}");
    }

    // Models are checked before they cost memory: one over the vertex limit (the two-part GLB
    // has 6) is an error, as is a glTF whose nodes loop, which would otherwise never finish.
    // A path that isn't a regular file (a device that never ends) isn't read.
    #[test]
    fn models_past_the_limits_are_errors() {
        let err = model(include_bytes!("testdata/two_parts.glb"), "model/gltf-binary", "", None, 5).unwrap_err();
        assert!(err.contains("more than 5 vertices"), "{err}");
        let looped = br#"{"asset": {"version": "2.0"}, "scenes": [{"nodes": [0]}], "nodes": [{"children": [1]}, {"children": [0]}]}"#;
        let err = model(looped, "model/gltf+json", "", None, usize::MAX).unwrap_err();
        assert!(err.contains("nodes must form trees"), "{err}");
        let bytes = binary_stl(b"", &[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]]);
        let err = model(&bytes, "model/stl", "", None, 2).unwrap_err();
        assert!(err.contains("3 vertices, more than the 2"), "{err}");
        assert!(read_file(Path::new("/dev/zero")).unwrap_err().contains("not a regular file"));
    }

    // A box's corners keep its faces' normals (90° creases); a sphere's corners point outward
    // from the centre, which only smoothing gives (its facets are flat).
    #[test]
    fn creases_stay_sharp_and_curves_shade_smooth() {
        let cube = shade(&cuboid(Vec3::ONE));
        for v in &cube {
            let n = Vec3::new(v[3], v[4], v[5]);
            assert!(n.abs().max_element() > 0.9999, "box normal {n} isn't a face normal");
            assert!(n.dot(Vec3::new(v[0], v[1], v[2])) > 0.0, "box normal {n} points inward");
        }
        let ball = shade(&sphere(2.0));
        let worst = ball.iter().map(|v| Vec3::new(v[3], v[4], v[5]).dot(Vec3::new(v[0], v[1], v[2]).normalize())).fold(1.0, f32::min);
        assert!(worst > 0.999, "a sphere corner's normal is {}° off radial", worst.acos().to_degrees());
    }

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
        assert!(resolve("https://example.com/cup.glb", &pkg).unwrap_err().contains("https:// isn't read"));
    }
}
