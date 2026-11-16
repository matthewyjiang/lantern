//! Packed point records (ROS PointCloud2 and Foxglove PointCloud share the layout: a stride,
//! named fields at byte offsets) into Lantern's `[x, y, z, colour]` points.

use super::DecodeError;
use crate::scene::{percentiles, PointCloud, PointColor, Pose};

/// The user's colour choice for clouds.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ColorChoice {
    /// Packed `rgb`/`rgba` or `red`/`green`/`blue` if present, else `intensity`, else height.
    #[default]
    Auto,
    Height,
    /// A numeric field by name; height if this cloud lacks it.
    Field(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Num { U8, I8, U16, I16, U32, I32, F32, F64 }

impl Num {
    fn size(self) -> usize {
        match self {
            Num::U8 | Num::I8 => 1,
            Num::U16 | Num::I16 => 2,
            Num::U32 | Num::I32 | Num::F32 => 4,
            Num::F64 => 8,
        }
    }
}

pub struct Field {
    pub name: String,
    pub offset: usize,
    pub num: Num,
}

/// One message's point records, before extraction.
pub struct Packed<'a> {
    pub frame_id: String,
    pub origin: Pose,
    pub fields: Vec<Field>,
    pub stride: usize,
    pub count: usize,
    pub big_endian: bool,
    pub data: &'a [u8],
}

/// Where each point's fourth float comes from.
enum Source<'a> {
    None,
    Scalar(&'a Field),
    /// Four bytes holding B, G, R, A in memory order (PCL's packed `rgb`, read as a LE u32).
    Packed(usize),
    Channels([&'a Field; 3]),
}

impl Packed<'_> {
    fn field(&self, name: &str) -> Option<&Field> {
        self.fields.iter().find(|f| f.name == name)
    }

    fn read(&self, at: usize, f: &Field) -> f32 {
        let b = &self.data[at + f.offset..at + f.offset + f.num.size()];
        macro_rules! num {
            ($t:ty) => {{
                let a = b.try_into().unwrap();
                (if self.big_endian { <$t>::from_be_bytes(a) } else { <$t>::from_le_bytes(a) }) as f32
            }};
        }
        match f.num {
            Num::U8 => b[0] as f32,
            Num::I8 => b[0] as i8 as f32,
            Num::U16 => num!(u16),
            Num::I16 => num!(i16),
            Num::U32 => num!(u32),
            Num::I32 => num!(i32),
            Num::F32 => num!(f32),
            Num::F64 => num!(f64),
        }
    }

    fn color_source(&self, choice: &ColorChoice) -> Source<'_> {
        let packed = ["rgb", "rgba"].iter().find_map(|n| self.field(n)).filter(|f| f.num.size() == 4).map(|f| Source::Packed(f.offset));
        let channels = || match (self.field("red"), self.field("green"), self.field("blue")) {
            (Some(r), Some(g), Some(b)) => Some(Source::Channels([r, g, b])),
            _ => None,
        };
        match choice {
            ColorChoice::Height => Source::None,
            ColorChoice::Field(name) => self.field(name).map_or(Source::None, Source::Scalar),
            ColorChoice::Auto => packed
                .or_else(channels)
                .or_else(|| self.field("intensity").map(Source::Scalar))
                .unwrap_or(Source::None),
        }
    }

    /// Extracts finite points and the chosen colour.
    pub fn extract(self, choice: &ColorChoice) -> Result<PointCloud, DecodeError> {
        let need = self.count.checked_mul(self.stride).ok_or_else(|| DecodeError::new("point cloud size overflows"))?;
        if self.data.len() < need {
            return Err(DecodeError::new(format!("point data is {} bytes, {} points × {} B need {need}", self.data.len(), self.count, self.stride)));
        }
        for f in &self.fields {
            if f.offset + f.num.size() > self.stride {
                return Err(DecodeError::new(format!("field {} at byte {} overruns the {} B point stride", f.name, f.offset, self.stride)));
            }
        }
        let [x, y, z] = ["x", "y", "z"].map(|n| self.field(n));
        let (Some(x), Some(y), Some(z)) = (x, y, z) else {
            return Err(DecodeError::new("point cloud has no x, y and z fields"));
        };
        let source = self.color_source(choice);
        let mut points = Vec::with_capacity(self.count);
        for i in 0..self.count {
            let at = i * self.stride;
            let p = [self.read(at, x), self.read(at, y), self.read(at, z)];
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let w = match &source {
                Source::None => 0.0,
                Source::Scalar(f) => self.read(at, f),
                Source::Packed(off) => {
                    let bgra = u32::from_le_bytes(self.data[at + off..at + off + 4].try_into().unwrap());
                    f32::from_bits(bgra | 0xff00_0000)
                }
                Source::Channels(ch) => {
                    let [r, g, b] = ch.map(|f| unit_byte(self.read(at, f), f.num));
                    f32::from_bits(0xff00_0000 | (r << 16) | (g << 8) | b)
                }
            };
            points.push([p[0], p[1], p[2], w]);
        }
        let color = match source {
            Source::None => PointColor::Height,
            Source::Scalar(f) => {
                let (min, max) = robust_range(&points);
                PointColor::Field { name: f.name.clone(), min, max }
            }
            Source::Packed(_) | Source::Channels(_) => PointColor::Rgb,
        };
        Ok(PointCloud {
            frame_id: self.frame_id,
            origin: self.origin,
            points,
            color,
            fields: self.fields.into_iter().map(|f| f.name).collect(),
        })
    }
}

/// A colour channel as 0–255: floats are 0–1 (Foxglove's convention), integers already bytes.
fn unit_byte(v: f32, num: Num) -> u32 {
    let v = if matches!(num, Num::F32 | Num::F64) { v * 255.0 } else { v };
    v.round().clamp(0.0, 255.0) as u32
}

/// The 1st to 99th percentile of the colour values, from a strided sample of at most 64 Ki
/// points: one hot return shouldn't wash the rest of the cloud into a single colour.
fn robust_range(points: &[[f32; 4]]) -> (f32, f32) {
    let step = points.len().div_ceil(1 << 16).max(1);
    let v: Vec<f32> = points.iter().step_by(step).map(|p| p[3]).filter(|w| w.is_finite()).collect();
    match percentiles(v, 0.01, 0.99) {
        Some((lo, hi)) if hi > lo => (lo, hi),
        Some((lo, _)) => (lo, lo + 1.0),
        None => (0.0, 1.0),
    }
}
