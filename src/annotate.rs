//! Draws image annotations (circles, outlines, polylines, dots) into an image's pixels, on the
//! CPU: a handful of shapes over a camera frame, redrawn when the image or its annotations
//! change. Edges are antialiased by distance. Text annotations are left to the UI, which sets
//! type far better than a rasteriser here could.

use crate::scene::{Annotation, PointsKind};

/// Draws `annotations` over `rgba` (`width × height`, straight-alpha sRGB).
pub fn draw<'a>(rgba: &mut [u8], width: u32, height: u32, annotations: impl IntoIterator<Item = &'a Annotation>) {
    let mut canvas = Canvas { rgba, w: width as i32, h: height as i32 };
    for a in annotations {
        match a {
            Annotation::Circle { center, diameter, thickness, outline, fill } => {
                let r = diameter * 0.5;
                if fill[3] > 0.0 {
                    canvas.cover(*center, r + 1.0, *fill, |d| r + 0.5 - d);
                }
                let half = thickness.max(1.0) * 0.5;
                canvas.cover(*center, r + half + 1.0, *outline, |d| half + 0.5 - (d - r).abs());
            }
            Annotation::Points { kind, points, color, fill, thickness } => {
                let t = thickness.max(1.0);
                if *kind == PointsKind::Loop && fill[3] > 0.0 && points.len() >= 3 {
                    canvas.polygon(points, *fill);
                }
                let segments: Vec<([f32; 2], [f32; 2])> = match kind {
                    PointsKind::Points => {
                        // A dot as wide as the line would be (2 px at the least, to be seen).
                        let r = t.max(2.0) * 0.5;
                        points.iter().for_each(|p| canvas.cover(*p, r + 1.0, *color, |d| r + 0.5 - d));
                        Vec::new()
                    }
                    PointsKind::Strip => points.windows(2).map(|w| (w[0], w[1])).collect(),
                    PointsKind::Loop => points.iter().zip(points.iter().cycle().skip(1)).map(|(a, b)| (*a, *b)).collect(),
                    PointsKind::Lines => points.chunks_exact(2).map(|w| (w[0], w[1])).collect(),
                };
                for (a, b) in segments {
                    canvas.segment(a, b, t, *color);
                }
            }
            Annotation::Text { .. } => {}
        }
    }
}

struct Canvas<'a> {
    rgba: &'a mut [u8],
    w: i32,
    h: i32,
}

impl Canvas<'_> {
    /// Blends `color` at `coverage` (0–1) over the pixel at (x, y), if it's in the image.
    fn blend(&mut self, x: i32, y: i32, color: [f32; 4], coverage: f32) {
        let a = (color[3] * coverage).clamp(0.0, 1.0);
        if a <= 0.0 || x < 0 || y < 0 || x >= self.w || y >= self.h {
            return;
        }
        let i = (y * self.w + x) as usize * 4;
        for k in 0..3 {
            let under = self.rgba[i + k] as f32;
            self.rgba[i + k] = (under + (color[k].clamp(0.0, 1.0) * 255.0 - under) * a).round() as u8;
        }
        self.rgba[i + 3] = self.rgba[i + 3].max((a * 255.0) as u8);
    }

    /// Every pixel within `reach` of `centre`, at the coverage `edge` gives its centre's distance.
    fn cover(&mut self, centre: [f32; 2], reach: f32, color: [f32; 4], edge: impl Fn(f32) -> f32) {
        let (x0, x1) = ((centre[0] - reach).floor() as i32, (centre[0] + reach).ceil() as i32);
        let (y0, y1) = ((centre[1] - reach).floor() as i32, (centre[1] + reach).ceil() as i32);
        for y in y0.max(0)..=y1.min(self.h - 1) {
            for x in x0.max(0)..=x1.min(self.w - 1) {
                let d = ((x as f32 + 0.5 - centre[0]).powi(2) + (y as f32 + 0.5 - centre[1]).powi(2)).sqrt();
                self.blend(x, y, color, edge(d).clamp(0.0, 1.0));
            }
        }
    }

    fn segment(&mut self, a: [f32; 2], b: [f32; 2], thickness: f32, color: [f32; 4]) {
        let half = thickness * 0.5;
        let pad = half + 1.0;
        let (x0, x1) = ((a[0].min(b[0]) - pad).floor() as i32, (a[0].max(b[0]) + pad).ceil() as i32);
        let (y0, y1) = ((a[1].min(b[1]) - pad).floor() as i32, (a[1].max(b[1]) + pad).ceil() as i32);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len2 = (dx * dx + dy * dy).max(1e-12);
        for y in y0.max(0)..=y1.min(self.h - 1) {
            for x in x0.max(0)..=x1.min(self.w - 1) {
                let (px, py) = (x as f32 + 0.5 - a[0], y as f32 + 0.5 - a[1]);
                let t = ((px * dx + py * dy) / len2).clamp(0.0, 1.0);
                let d = ((px - t * dx).powi(2) + (py - t * dy).powi(2)).sqrt();
                self.blend(x, y, color, (half + 0.5 - d).clamp(0.0, 1.0));
            }
        }
    }

    /// Fills the polygon `points` (even-odd), sampling pixel centres.
    fn polygon(&mut self, points: &[[f32; 2]], color: [f32; 4]) {
        let (lo, hi) = points.iter().fold(([f32::MAX; 2], [f32::MIN; 2]), |(lo, hi), p| ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]));
        for y in (lo[1].floor() as i32).max(0)..=(hi[1].ceil() as i32).min(self.h - 1) {
            for x in (lo[0].floor() as i32).max(0)..=(hi[0].ceil() as i32).min(self.w - 1) {
                let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                let mut inside = false;
                for (a, b) in points.iter().zip(points.iter().cycle().skip(1)) {
                    if (a[1] > py) != (b[1] > py) && px < a[0] + (py - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
                        inside = !inside;
                    }
                }
                if inside {
                    self.blend(x, y, color, 1.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn px(rgba: &[u8], w: u32, x: u32, y: u32) -> [u8; 4] {
        let i = ((y * w + x) * 4) as usize;
        rgba[i..i + 4].try_into().unwrap()
    }

    // A red ring of diameter 10 around (16, 16) and a 1 px green square outline through pixel
    // centres: pixels on the shapes
    // take their colour, pixels inside them keep the picture's.
    #[test]
    fn shapes_cover_their_outline_and_leave_the_inside() {
        let (w, h) = (32, 32);
        let mut rgba = vec![0u8; (w * h * 4) as usize];
        let shapes = [
            Annotation::Circle { center: [16.0, 16.0], diameter: 10.0, thickness: 2.0, outline: [1.0, 0.0, 0.0, 1.0], fill: [0.0; 4] },
            Annotation::Points { kind: PointsKind::Loop, points: vec![[2.5, 2.5], [8.5, 2.5], [8.5, 8.5], [2.5, 8.5]], color: [0.0, 1.0, 0.0, 1.0], fill: [0.0; 4], thickness: 1.0 },
        ];
        draw(&mut rgba, w, h, &shapes);
        assert_eq!(px(&rgba, w, 20, 16)[..3], [255, 0, 0], "on the ring (pixel centre 4.5 from the centre)");
        assert_eq!(px(&rgba, w, 16, 16), [0, 0, 0, 0], "inside the ring");
        assert_eq!(px(&rgba, w, 5, 2)[..3], [0, 255, 0], "on the top edge, through pixel centres");
        assert_eq!(px(&rgba, w, 5, 5), [0, 0, 0, 0], "inside the square");
        assert_eq!(px(&rgba, w, 28, 28), [0, 0, 0, 0], "away from both");
    }
}
