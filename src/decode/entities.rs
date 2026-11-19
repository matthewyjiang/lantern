//! What `foxglove.SceneUpdate` (protobuf) and `foxglove_msgs/msg/SceneUpdate` (CDR) share once
//! their fields are read: indices resolved, colours and lifetimes turned into scene types.

use std::hash::{DefaultHasher, Hash, Hasher};

use glam::DVec3;

use super::DecodeError;
use crate::scene::{Deletion, Geometry, LineKind, ModelSource, Pose, Shape, Thickness};

/// A lifetime of `sec` + `nanos`; zero (or less) keeps the entity until replaced or deleted.
pub fn lifetime(sec: i64, nanos: i64) -> Option<f64> {
    let s = sec as f64 + nanos as f64 * 1e-9;
    (s > 0.0).then_some(s)
}

/// A deletion of `kind` 0 (the entity `id`) or 1 (all of the topic's).
pub fn deletion(kind: i64, id: String) -> Result<Deletion, DecodeError> {
    match kind {
        0 => Ok(Deletion::Id(id)),
        1 => Ok(Deletion::All),
        other => Err(DecodeError::new(format!("deletion has unknown type {other}"))),
    }
}

/// `items` in the order `indices` names them, or as they are with no indices.
fn indexed<T: Clone>(items: Vec<T>, indices: &[u32], what: &str) -> Result<Vec<T>, DecodeError> {
    if indices.is_empty() {
        return Ok(items);
    }
    indices.iter().map(|&i| {
        items.get(i as usize).cloned().ok_or_else(|| DecodeError::new(format!("{what} index {i} is past its {} points", items.len())))
    }).collect()
}

/// A line primitive; `None` for a line type Lantern doesn't know (the entity's other shapes
/// still draw). Per-point colours, when given, must match the points.
#[allow(clippy::too_many_arguments)]
pub fn line(kind: i64, pose: Pose, thickness: f64, scale_invariant: bool, points: Vec<DVec3>, color: [f32; 4], colors: Vec<[f32; 4]>, indices: &[u32]) -> Result<Option<Shape>, DecodeError> {
    let kind = match kind {
        0 => LineKind::Strip,
        1 => LineKind::Loop,
        2 => LineKind::List,
        _ => return Ok(None),
    };
    if !colors.is_empty() && colors.len() != points.len() {
        return Err(DecodeError::new(format!("line has {} points but {} colors", points.len(), colors.len())));
    }
    let colors = if colors.is_empty() { colors } else { indexed(colors, indices, "line")? };
    let points = indexed(points, indices, "line")?;
    let thickness = if scale_invariant { Thickness::Points(thickness) } else { Thickness::Metres(thickness) };
    Ok(Some(Shape { pose, color, geometry: Geometry::Line { kind, points, colors, thickness } }))
}

/// A triangle list, drawn in one colour: the first per-vertex colour when there are some.
pub fn triangles(pose: Pose, points: Vec<DVec3>, color: [f32; 4], colors: Vec<[f32; 4]>, indices: &[u32]) -> Result<Shape, DecodeError> {
    let points = indexed(points, indices, "triangle")?;
    if !points.len().is_multiple_of(3) {
        return Err(DecodeError::new(format!("triangle list has {} points, not a multiple of three", points.len())));
    }
    Ok(Shape { pose, color: colors.first().copied().unwrap_or(color), geometry: Geometry::Triangles { points } })
}

pub fn text(pose: Pose, text: String, font_size: f64, scale_invariant: bool, color: [f32; 4]) -> Shape {
    let size = if scale_invariant { Thickness::Points(font_size) } else { Thickness::Metres(font_size) };
    Shape { pose, color, geometry: Geometry::Text { text, size } }
}

/// A model from `data` when it has bytes, else from `url`. A scale left at zero (unset) is 1.
#[allow(clippy::too_many_arguments)]
pub fn model(pose: Pose, scale: DVec3, color: [f32; 4], override_color: bool, url: String, media_type: String, data: Vec<u8>) -> Result<Shape, DecodeError> {
    let source = if !data.is_empty() {
        let mut h = DefaultHasher::new();
        (&media_type, &data).hash(&mut h);
        ModelSource::Data { media_type, bytes: data.into(), hash: h.finish() }
    } else if !url.is_empty() {
        ModelSource::Url(url)
    } else {
        return Err(DecodeError::new("model has neither data nor a url"));
    };
    let scale = if scale == DVec3::ZERO { DVec3::ONE } else { scale };
    Ok(Shape { pose, color, geometry: Geometry::Model { source, scale, override_color } })
}
