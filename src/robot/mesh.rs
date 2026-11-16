//! Triangle soups from STL files and URDF primitives, and the smoothed normals they're drawn with.

use std::collections::HashMap;
use std::f32::consts::TAU;

use glam::Vec3;

use super::urdf::Shape;

pub type Triangle = [Vec3; 3];

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

/// A URDF primitive's triangles, wound outward.
pub fn primitive(shape: &Shape) -> Vec<Triangle> {
    let mut out = Vec::new();
    match *shape {
        Shape::Box { size } => {
            let h = size.as_vec3() * 0.5;
            for axis in 0..3 {
                for sign in [-1.0f32, 1.0] {
                    let (u, v) = (Vec3::AXES[(axis + 1) % 3] * h, Vec3::AXES[(axis + 2) % 3] * h);
                    let n = Vec3::AXES[axis] * h * sign;
                    let (u, v) = if sign > 0.0 { (u, v) } else { (v, u) };
                    quad(&mut out, n - u - v, n + u - v, n + u + v, n - u + v);
                }
            }
        }
        Shape::Cylinder { radius, length } => {
            const SIDES: usize = 32;
            let (r, z) = (radius as f32, length as f32 * 0.5);
            // Wrapped like the sphere's seam, so the last side meets the first exactly.
            let rim = |k: usize| {
                let a = (k % SIDES) as f32 / SIDES as f32 * TAU;
                Vec3::new(a.cos() * r, a.sin() * r, 0.0)
            };
            for k in 0..SIDES {
                let (a, b) = (rim(k), rim(k + 1));
                quad(&mut out, a - Vec3::Z * z, b - Vec3::Z * z, b + Vec3::Z * z, a + Vec3::Z * z);
                out.push([Vec3::Z * z, a + Vec3::Z * z, b + Vec3::Z * z]);
                out.push([-Vec3::Z * z, b - Vec3::Z * z, a - Vec3::Z * z]);
            }
        }
        Shape::Sphere { radius } => {
            const RINGS: usize = 16;
            const SEGMENTS: usize = 32;
            let at = |i: usize, k: usize| {
                // Exact poles: f32 sin(π) isn't 0, and corners that don't meet bit for bit
                // become slivers with arbitrary normals.
                match i {
                    0 => return Vec3::Z * radius as f32,
                    RINGS => return -Vec3::Z * radius as f32,
                    _ => {}
                }
                // k wraps to 0, not TAU: f32 cos/sin of TAU miss 1 and 0, and the seam wouldn't weld.
                let (lat, lon) = (i as f32 / RINGS as f32 * TAU * 0.5, (k % SEGMENTS) as f32 / SEGMENTS as f32 * TAU);
                Vec3::new(lat.sin() * lon.cos(), lat.sin() * lon.sin(), lat.cos()) * radius as f32
            };
            for i in 0..RINGS {
                for k in 0..SEGMENTS {
                    quad(&mut out, at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
                }
            }
        }
    }
    out
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

    // A box's corners keep its faces' normals (90° creases); a sphere's corners point outward
    // from the centre, which only smoothing gives (its facets are flat).
    #[test]
    fn creases_stay_sharp_and_curves_shade_smooth() {
        let cube = shade(&primitive(&Shape::Box { size: glam::DVec3::ONE }));
        for v in &cube {
            let n = Vec3::new(v[3], v[4], v[5]);
            assert!(n.abs().max_element() > 0.9999, "box normal {n} isn't a face normal");
            assert!(n.dot(Vec3::new(v[0], v[1], v[2])) > 0.0, "box normal {n} points inward");
        }
        let ball = shade(&primitive(&Shape::Sphere { radius: 2.0 }));
        let worst = ball.iter().map(|v| Vec3::new(v[3], v[4], v[5]).dot(Vec3::new(v[0], v[1], v[2]).normalize())).fold(1.0, f32::min);
        assert!(worst > 0.999, "a sphere corner's normal is {}° off radial", worst.acos().to_degrees());
    }
}
