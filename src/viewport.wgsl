// Lantern's 3D viewport. Three pipelines draw into the offscreen target in one pass, in order:
// `sky` (backdrop + ground grid, no depth), `points` (the cloud, depth-tested and written), then
// `seg` (TF axes and the sensor path, depth-tested, blended). `blit` then copies the target into
// Rune's effect slot inside the UI pass. Rune's CLIP_WGSL is prepended (clip_alpha).
//
// Colours are linear: the offscreen target is *Srgb, and `blit` re-encodes when the UI pass
// blends in sRGB.

struct Frame {
    view_proj: mat4x4f,
    inv_view_proj: mat4x4f,
    eye: vec4f,
    // Target width and height (device px), point size (device px), unused.
    viewport: vec4f,
    // Colour mode (0 height, 1 intensity), decimation stride, grid on (1/0), unused.
    style: vec4<u32>,
    // Height colour range, metres: low, high.
    height: vec4f,
}

@group(0) @binding(0) var<uniform> F: Frame;
@group(0) @binding(1) var<storage, read> points: array<vec4f>;
// Three vec4s per segment: a (xyz, width px), b (xyz), colour (rgb, alpha).
@group(0) @binding(2) var<storage, read> segs: array<vec4f>;

const BG_LOW: vec3f = vec3f(0.0080, 0.0092, 0.0118);
const BG_HORIZON: vec3f = vec3f(0.0235, 0.0262, 0.0320);

fn to_linear(c: vec3f) -> vec3f {
    return select(pow((c + 0.055) / 1.055, vec3f(2.4)), c / 12.92, c <= vec3f(0.04045));
}

// Google's Turbo colormap, polynomial fit (Mikhailov 2019), sRGB out.
fn turbo(x: f32) -> vec3f {
    let t = clamp(x, 0.0, 1.0);
    let r = vec4f(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    let g = vec4f(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    let b = vec4f(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    let r2 = vec2f(-152.94239396, 59.28637943);
    let g2 = vec2f(4.27729857, 2.82956604);
    let b2 = vec2f(-89.90310912, 27.34824973);
    let v4 = vec4f(1.0, t, t * t, t * t * t);
    let v2 = v4.zw * v4.z;
    return clamp(vec3f(dot(v4, r) + dot(v2, r2), dot(v4, g) + dot(v2, g2), dot(v4, b) + dot(v2, b2)), vec3f(0.0), vec3f(1.0));
}

fn fullscreen(vi: u32) -> vec4f {
    let uv = vec2f(f32((vi << 1u) & 2u), f32(vi & 2u));
    return vec4f(uv * 2.0 - 1.0, 0.0, 1.0);
}

// ---- sky: gradient backdrop and the z = 0 grid, ray-cast per pixel ----

struct SkyOut { @builtin(position) pos: vec4f, @location(0) ndc: vec2f }

@vertex fn sky_vs(@builtin(vertex_index) vi: u32) -> SkyOut {
    let p = fullscreen(vi);
    return SkyOut(p, p.xy);
}

// Antialiased coverage of lines every `cell` metres at `c`.
fn grid_lines(c: vec2f, cell: f32, width_px: f32) -> f32 {
    let q = c / cell;
    let w = fwidth(q);
    let d = abs(fract(q - 0.5) - 0.5) / w;
    let line = 1.0 - min(min(d.x, d.y) / width_px, 1.0);
    // Fade lines whose cells shrink under a few pixels, before they alias.
    return line * (1.0 - smoothstep(0.15, 0.45, max(w.x, w.y)));
}

@fragment fn sky_fs(in: SkyOut) -> @location(0) vec4f {
    let near = F.inv_view_proj * vec4f(in.ndc, 1.0, 1.0);
    let dir = normalize(near.xyz / near.w - F.eye.xyz);
    var col = mix(BG_LOW, BG_HORIZON, exp(-abs(dir.z) * 5.0));
    // Derivatives must run in uniform control flow: compute grid terms everywhere, mask after.
    let t = -F.eye.z / dir.z;
    let hit = F.eye.xy + dir.xy * t;
    let on_ground = dir.z * F.eye.z < 0.0 && F.style.z == 1u;
    let fade = select(0.0, 1.0 - smoothstep(25.0, 140.0, t), on_ground);
    let minor = grid_lines(hit, 1.0, 1.0);
    let major = grid_lines(hit, 10.0, 1.3);
    col = mix(col, vec3f(0.050, 0.055, 0.066), minor * fade * 0.8);
    col = mix(col, vec3f(0.105, 0.115, 0.135), major * fade);
    return vec4f(col, 1.0);
}

// ---- points: one screen-aligned quad per return, pulled from the storage buffer ----

struct PointOut { @builtin(position) pos: vec4f, @location(0) color: vec3f }

// Six vertices per point, not a 4-vertex instance: tiny instances leave the GPU's vertex
// front-end mostly idle (see README, "What bounds the point draw").
const QUAD: array<u32, 6> = array<u32, 6>(0u, 1u, 2u, 2u, 1u, 3u);

@vertex fn points_vs(@builtin(vertex_index) v: u32) -> PointOut {
    let ii = v / 6u;
    let vi = QUAD[v % 6u];
    let p = points[ii * F.style.y];
    var clip = F.view_proj * vec4f(p.xyz, 1.0);
    if (clip.w <= 0.0) {
        return PointOut(vec4f(2.0, 2.0, 2.0, 1.0), vec3f(0.0)); // behind the eye: off-screen
    }
    let corner = vec2f(f32(vi & 1u), f32(vi >> 1u)) * 2.0 - 1.0;
    clip = vec4f(clip.xy + corner * F.viewport.z / F.viewport.xy * clip.w, clip.zw);

    var c: vec3f;
    if (F.style.x == 0u) {
        let h = (p.z - F.height.x) / (F.height.y - F.height.x);
        // Start at Turbo's blue: its near-black low end would swallow the road and its paint.
        // Shade by intensity so dark asphalt recedes and paint and poles stand out.
        c = to_linear(turbo(0.12 + 0.83 * h)) * (0.1 + 0.9 * p.w);
    } else {
        c = to_linear(turbo(0.08 + 0.84 * p.w));
    }
    // Depth cue: far returns sink toward the horizon colour.
    let dist = distance(p.xyz, F.eye.xyz);
    c = mix(c, BG_HORIZON, smoothstep(30.0, 160.0, dist) * 0.7);
    return PointOut(clip, c);
}

@fragment fn points_fs(in: PointOut) -> @location(0) vec4f {
    return vec4f(in.color, 1.0);
}

// ---- seg: thick antialiased lines, expanded in screen space ----

struct SegOut { @builtin(position) pos: vec4f, @location(0) color: vec4f, @location(1) across: f32, @location(2) half_px: f32 }

@vertex fn seg_vs(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> SegOut {
    let a = segs[ii * 3u];
    let b = segs[ii * 3u + 1u];
    let color = segs[ii * 3u + 2u];
    let ca = F.view_proj * vec4f(a.xyz, 1.0);
    let cb = F.view_proj * vec4f(b.xyz, 1.0);
    if (ca.w <= 0.0 || cb.w <= 0.0) {
        return SegOut(vec4f(2.0, 2.0, 2.0, 1.0), vec4f(0.0), 0.0, 1.0);
    }
    let half_px = a.w * 0.5 + 1.0; // one pixel of feather on each side
    let pa = ca.xy / ca.w * F.viewport.xy * 0.5;
    let pb = cb.xy / cb.w * F.viewport.xy * 0.5;
    let along = normalize(pb - pa + vec2f(1e-6, 0.0));
    let normal = vec2f(-along.y, along.x);
    let end = vi >> 1u;
    let side = f32(vi & 1u) * 2.0 - 1.0;
    let c = select(ca, cb, end == 1u);
    let cap = select(-along, along, end == 1u) * half_px;
    let px = normal * side * half_px + cap;
    let clip = vec4f(c.xy + px / (F.viewport.xy * 0.5) * c.w, c.zw);
    return SegOut(clip, vec4f(to_linear(color.rgb), color.a), side * half_px, half_px);
}

@fragment fn seg_fs(in: SegOut) -> @location(0) vec4f {
    let a = in.color.a * clamp(in.half_px - abs(in.across), 0.0, 1.0);
    return vec4f(in.color.rgb * a, a);
}

// ---- blit: the offscreen target into the effect slot, clipped like the UI around it ----

struct Blit { rect: vec4f, clip: vec4f, fade: vec4<u32>, frame: vec4f, flags: vec4f }

@group(0) @binding(3) var<uniform> B: Blit;
@group(0) @binding(4) var src: texture_2d<f32>;

@vertex fn blit_vs(@builtin(vertex_index) vi: u32) -> @builtin(position) vec4f {
    let c = vec2f(f32(vi & 1u), f32(vi >> 1u));
    let l = (B.rect.xy + c * B.rect.zw - B.frame.xy) / B.frame.zw;
    return vec4f(l.x * 2.0 - 1.0, 1.0 - l.y * 2.0, 0.0, 1.0);
}

fn encode(x: vec3f) -> vec3f {
    return select(1.055 * pow(x, vec3f(1.0 / 2.4)) - 0.055, x * 12.92, x <= vec3f(0.0031308));
}

@fragment fn blit_fs(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let win = pos.xy + B.frame.xy;
    let size = vec2i(textureDimensions(src));
    let texel = clamp(vec2i(floor(win - B.rect.xy)), vec2i(0), size - 1);
    var rgb = textureLoad(src, texel, 0).rgb;
    if (B.flags.x > 0.5) { rgb = encode(rgb); }
    let a = clip_alpha(win, B.clip, B.fade);
    return vec4f(rgb * a, a);
}
