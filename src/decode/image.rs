//! Camera images to RGBA8, on the source's thread: a compressed image (JPEG, PNG) through the
//! `image` crate (which Rune already builds), or raw pixels in one of the common encodings.

use super::DecodeError;
use crate::scene::Image;

/// The widest or tallest image Lantern shows: Rune's `Pixels` draws images up to 8192 px a side
/// (the 2D texture size every wgpu backend guarantees). A larger one is a per-topic error.
pub const MAX_SIDE: u32 = 8192;

/// Refuses an empty image or one over `MAX_SIDE`, before anything is decoded or allocated.
fn check_size(width: u32, height: u32) -> Result<(), DecodeError> {
    if width == 0 || height == 0 {
        return Err(DecodeError::new("the image is empty"));
    }
    if width > MAX_SIDE || height > MAX_SIDE {
        return Err(DecodeError::new(format!("{width}×{height} px, over the {MAX_SIDE} px a side Lantern can show")));
    }
    Ok(())
}

/// `format` is what the message says ("jpeg", "png", ROS's "rgb8; jpeg compressed bgr8"); the
/// bytes' own signature decides, so a mislabelled image still decodes.
pub fn compressed(format: &str, data: &[u8]) -> Result<Image, DecodeError> {
    let fail = |e: image::ImageError| DecodeError::new(format!("{format:?} image: {e}"));
    let reader = image::ImageReader::new(std::io::Cursor::new(data)).with_guessed_format().map_err(|e| fail(e.into()))?;
    let (width, height) = reader.into_dimensions().map_err(fail)?;
    check_size(width, height)?;
    let img = image::load_from_memory(data).map_err(fail)?;
    Ok(Image { width, height, rgba: img.into_rgba8().into_raw() })
}

/// Raw pixels: `step` bytes per row (rows may be padded). 16-bit mono keeps its high byte.
pub fn raw(width: u32, height: u32, encoding: &str, step: u32, big_endian: bool, data: &[u8]) -> Result<Image, DecodeError> {
    let bpp = match encoding {
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" => 4,
        "mono8" | "8UC1" => 1,
        "mono16" | "16UC1" => 2,
        other => return Err(DecodeError::new(format!("image encoding {other:?} isn't supported (rgb8, bgr8, rgba8, bgra8, mono8, mono16)"))),
    };
    check_size(width, height)?;
    let (w, h, step) = (width as usize, height as usize, step as usize);
    // The last row needn't carry its padding.
    let needed = step * (h - 1) + w * bpp;
    if step < w * bpp || data.len() < needed {
        return Err(DecodeError::new(format!("{width}×{height} {encoding} needs {needed} bytes at step {step}, has {}", data.len())));
    }
    let mut rgba = Vec::with_capacity(w * h * 4);
    for row in 0..h {
        let line = &data[row * step..row * step + w * bpp];
        for px in line.chunks_exact(bpp) {
            rgba.extend_from_slice(&match (encoding, px) {
                ("rgb8", &[r, g, b]) => [r, g, b, 255],
                ("bgr8", &[b, g, r]) => [r, g, b, 255],
                ("rgba8", &[r, g, b, a]) => [r, g, b, a],
                ("bgra8", &[b, g, r, a]) => [r, g, b, a],
                (_, &[v]) => [v, v, v, 255],
                (_, &[a, b]) => {
                    let v = if big_endian { a } else { b };
                    [v, v, v, 255]
                }
                _ => unreachable!("chunks are bpp long"),
            });
        }
    }
    Ok(Image { width, height, rgba })
}
