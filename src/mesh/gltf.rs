//! glTF 2.0, as `.glb` or `.gltf`: every triangle primitive of the default scene, placed by its
//! node's transform, in its material's base colour. Textures and vertex colours aren't drawn.

use base64::Engine;
use glam::{Mat4, Vec3};
use gltf::buffer::Source;
use gltf::mesh::Mode;
use std::path::Path;

use super::Triangle;

/// Triangles with their colour (sRGB and alpha), one entry per primitive.
pub type Pieces = Vec<(Vec<Triangle>, Option<[f32; 4]>)>;

/// Each triangle primitive's triangles and base colour, in the file's own (Y-up) axes. Buffers
/// come from a GLB's binary chunk, `data:` URIs, or files beside `dir`. Stops with an error once
/// the triangles pass `max_vertices` (a mesh placed by many nodes counts each time).
pub fn read(bytes: &[u8], dir: Option<&Path>, max_vertices: usize) -> Result<Pieces, String> {
    let doc = gltf::Gltf::from_slice(bytes).map_err(|e| format!("glTF: {e}"))?;
    let mut buffers = Vec::new();
    for b in doc.buffers() {
        let data = match b.source() {
            Source::Bin => doc.blob.clone().ok_or("glTF: a buffer refers to a GLB binary chunk the file doesn't have")?,
            Source::Uri(uri) => match uri.strip_prefix("data:") {
                Some(rest) => {
                    let (_, b64) = rest.split_once(";base64,").ok_or_else(|| format!("glTF: a data URI that isn't base64 ({:.40})", uri))?;
                    base64::engine::general_purpose::STANDARD.decode(b64).map_err(|e| format!("glTF: a data URI's base64: {e}"))?
                }
                None => {
                    let dir = dir.ok_or_else(|| format!("glTF: buffer {uri} is a separate file, and a mesh sent as bytes has no directory to find it in"))?;
                    super::read_file(&dir.join(uri.replace("%20", " "))).map_err(|e| format!("glTF: {e}"))?
                }
            },
        };
        if data.len() < b.length() {
            return Err(format!("glTF: buffer {} has {} bytes of the {} it declares", b.index(), data.len(), b.length()));
        }
        buffers.push(data);
    }
    let scene = doc.default_scene().or_else(|| doc.scenes().next()).ok_or("glTF: no scene")?;
    let mut out = Vec::new();
    let mut vertices = 0;
    // Depth first, in the file's order (the stack holds siblings last to first). glTF's nodes
    // form trees, so a node reached twice is a broken file (a cycle would never end).
    let mut seen = vec![false; doc.nodes().len()];
    let mut stack: Vec<(gltf::Node, Mat4)> = scene.nodes().map(|n| (n, Mat4::IDENTITY)).collect();
    stack.reverse();
    while let Some((node, parent)) = stack.pop() {
        if std::mem::replace(&mut seen[node.index()], true) {
            return Err(format!("glTF: node {} has two parents, or is its own ancestor; nodes must form trees", node.index()));
        }
        let m = parent * Mat4::from_cols_array_2d(&node.transform().matrix());
        let children: Vec<_> = node.children().collect();
        stack.extend(children.into_iter().rev().map(|c| (c, m)));
        let Some(mesh) = node.mesh() else { continue };
        for p in mesh.primitives() {
            let reader = p.reader(|b| buffers.get(b.index()).map(Vec::as_slice));
            let Some(positions) = reader.read_positions() else { continue };
            let positions: Vec<Vec3> = positions.map(|v| m.transform_point3(Vec3::from(v))).collect();
            let order: Vec<u32> = match reader.read_indices() {
                Some(i) => i.into_u32().collect(),
                None => (0..positions.len() as u32).collect(),
            };
            let n = match p.mode() {
                Mode::Triangles => order.len() / 3,
                Mode::TriangleStrip | Mode::TriangleFan => order.len().saturating_sub(2),
                Mode::Points | Mode::Lines | Mode::LineLoop | Mode::LineStrip => 0,
            };
            vertices += 3 * n;
            if vertices > max_vertices {
                return Err(format!("glTF: more than {} vertices, the most the GPU can draw", crate::ui::thousands(max_vertices)));
            }
            let corner = |i: u32| positions.get(i as usize).copied().ok_or_else(|| format!("glTF: index {i} is past the {} positions", positions.len()));
            let mut tris = Vec::new();
            match p.mode() {
                Mode::Triangles => {
                    for t in order.chunks_exact(3) {
                        tris.push([corner(t[0])?, corner(t[1])?, corner(t[2])?]);
                    }
                }
                Mode::TriangleStrip => {
                    for (k, t) in order.windows(3).enumerate() {
                        // Every other triangle of a strip is wound the other way round.
                        let t = if k % 2 == 0 { [t[0], t[1], t[2]] } else { [t[1], t[0], t[2]] };
                        tris.push([corner(t[0])?, corner(t[1])?, corner(t[2])?]);
                    }
                }
                Mode::TriangleFan => {
                    for t in order.windows(2).skip(1) {
                        tris.push([corner(order[0])?, corner(t[0])?, corner(t[1])?]);
                    }
                }
                // Points and lines have no surface to draw.
                Mode::Points | Mode::Lines | Mode::LineLoop | Mode::LineStrip => continue,
            }
            // glTF colours are linear; Lantern keeps sRGB.
            let [r, g, b, a] = p.material().pbr_metallic_roughness().base_color_factor();
            out.push((tris, Some([srgb(r), srgb(g), srgb(b), a])));
        }
    }
    Ok(out)
}

fn srgb(c: f32) -> f32 {
    if c <= 0.003_130_8 { c * 12.92 } else { 1.055 * c.powf(1.0 / 2.4) - 0.055 }
}
