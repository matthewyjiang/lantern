// Lantern's 3D viewport. These pipelines draw into the offscreen target in one pass, in order:
// `sky` (backdrop + ground grid, no depth), `points` (the cloud, depth-tested and written),
// `robot` twice (depth only, then colour where depth matches, blended), then `seg` (TF axes and
// tree, poses, velocities; depth-tested, blended). `blit` then copies the target into
// Rune's effect slot inside the UI pass. Rune's CLIP_WGSL is prepended (clip_alpha).
//
// Colours are linear: the offscreen target is *Srgb, and `blit` re-encodes when the UI pass
// blends in sRGB.

struct Frame {
    view_proj: mat4x4f,
    inv_view_proj: mat4x4f,
    // The cloud's frame in the display frame.
    model: mat4x4f,
    eye: vec4f,
    // Target width and height (device px), point size (device px), unused.
    viewport: vec4f,
    // Colour mode (0 height, 1 field value, 2 packed rgb), decimation stride, grid on (1/0), unused.
    style: vec4<u32>,
    // The colour ramp's range (height in metres, or field value): low, high.
    shade: vec4f,
    // Minor grid cell, metres (majors are ten; the grid fades out with it), scene radius
    // (the depth cue scales with it), unused, unused.
    grid: vec4f,
}

@group(0) @binding(0) var<uniform> F: Frame;
// x, y, z as f32 bits, then the colour value: an f32 field value, or rgb packed 0xAARRGGBB.
@group(0) @binding(1) var<storage, read> points: array<vec4<u32>>;
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
    // Unproject a point 10^4 near-plane distances out (depth is near/distance here), not the
    // near plane itself: an eye a kilometre out keeps ~0.1 mm of f32 precision, so a point 1 cm
    // away would quantise the ray's direction into visible blocks.
    let far = F.inv_view_proj * vec4f(in.ndc, 1e-4, 1.0);
    let dir = normalize(far.xyz / far.w - F.eye.xyz);
    var col = mix(BG_LOW, BG_HORIZON, exp(-abs(dir.z) * 5.0));
    // Derivatives must run in uniform control flow: compute grid terms everywhere, mask after.
    let t = -F.eye.z / dir.z;
    let hit = F.eye.xy + dir.xy * t;
    let on_ground = dir.z * F.eye.z < 0.0 && F.style.z == 1u;
    let cell = F.grid.x;
    let fade = select(0.0, 1.0 - smoothstep(25.0 * cell, 140.0 * cell, t), on_ground);
    let minor = grid_lines(hit, cell, 1.0);
    let major = grid_lines(hit, 10.0 * cell, 1.3);
    col = mix(col, vec3f(0.050, 0.055, 0.066), minor * fade * 0.8);
    col = mix(col, vec3f(0.105, 0.115, 0.135), major * fade);
    return vec4f(col, 1.0);
}

// ---- points: one screen-aligned quad per point, pulled from the storage buffer ----

struct PointOut { @builtin(position) pos: vec4f, @location(0) color: vec3f }

// Six vertices per point, not a 4-vertex instance: tiny instances leave the GPU's vertex
// front-end mostly idle (see docs, Performance: "What bounds the point draw").
const QUAD: array<u32, 6> = array<u32, 6>(0u, 1u, 2u, 2u, 1u, 3u);

@vertex fn points_vs(@builtin(vertex_index) v: u32) -> PointOut {
    let ii = v / 6u;
    let vi = QUAD[v % 6u];
    let raw = points[ii * F.style.y];
    let world = F.model * vec4f(bitcast<vec3f>(raw.xyz), 1.0);
    var clip = F.view_proj * world;
    if (clip.w <= 0.0) {
        return PointOut(vec4f(2.0, 2.0, 2.0, 1.0), vec3f(0.0)); // behind the eye: off-screen
    }
    let corner = vec2f(f32(vi & 1u), f32(vi >> 1u)) * 2.0 - 1.0;
    clip = vec4f(clip.xy + corner * F.viewport.z / F.viewport.xy * clip.w, clip.zw);

    var c: vec3f;
    let span = F.shade.y - F.shade.x;
    if (F.style.x == 0u) {
        // Start at Turbo's blue: its near-black low end would vanish into the backdrop.
        c = to_linear(turbo(0.12 + 0.83 * (world.z - F.shade.x) / span));
    } else if (F.style.x == 1u) {
        c = to_linear(turbo(0.08 + 0.84 * (bitcast<f32>(raw.w) - F.shade.x) / span));
    } else {
        c = to_linear(vec3f(f32((raw.w >> 16u) & 255u), f32((raw.w >> 8u) & 255u), f32(raw.w & 255u)) / 255.0);
    }
    // Depth cue: far points sink toward the horizon colour.
    let dist = distance(world.xyz, F.eye.xyz);
    c = mix(c, BG_HORIZON, smoothstep(0.6 * F.grid.y, 3.0 * F.grid.y, dist) * 0.7);
    return PointOut(clip, c);
}

@fragment fn points_fs(in: PointOut) -> @location(0) vec4f {
    return vec4f(in.color, 1.0);
}

// ---- robot: mesh parts, each placed by its link's pose ----

// Position and normal per vertex (in the link's frame), three vertices per triangle.
@group(1) @binding(0) var<storage, read> robot_verts: array<f32>;
struct Part { model: mat4x4f, color: vec4f }
// One per part, picked by the instance index: link pose in the display frame, then linear colour
// and alpha (opacity applied).
@group(1) @binding(1) var<storage, read> parts: array<Part>;

// `@invariant`: the depth and colour passes must compute bit-identical depths for `Equal`.
struct RobotOut {
    @invariant @builtin(position) pos: vec4f,
    @location(0) world: vec3f,
    @location(1) normal: vec3f,
    @location(2) @interpolate(flat) part: u32,
}

@vertex fn robot_vs(@builtin(vertex_index) v: u32, @builtin(instance_index) i: u32) -> RobotOut {
    let b = v * 6u;
    let m = parts[i].model;
    let world = m * vec4f(robot_verts[b], robot_verts[b + 1u], robot_verts[b + 2u], 1.0);
    // Link poses are rigid (scale is baked into the vertices), so the model matrix turns normals.
    let n = (m * vec4f(robot_verts[b + 3u], robot_verts[b + 4u], robot_verts[b + 5u], 0.0)).xyz;
    return RobotOut(F.view_proj * world, world.xyz, n, i);
}

// Two-sided (meshes aren't reliably wound): a headlight from the eye so nothing faces into
// darkness, a key light from above for form, ambient, and a soft highlight.
@fragment fn robot_fs(in: RobotOut) -> @location(0) vec4f {
    let c = parts[in.part].color;
    let to_eye = normalize(F.eye.xyz - in.world);
    var n = normalize(in.normal);
    n = select(n, -n, dot(n, to_eye) < 0.0);
    let key = normalize(vec3f(0.35, 0.25, 1.0));
    let lit = 0.16 + 0.52 * max(dot(n, to_eye), 0.0) + 0.38 * max(dot(n, key), 0.0);
    let spec = 0.12 * pow(max(dot(n, normalize(to_eye + key)), 0.0), 48.0);
    return vec4f((c.rgb * lit + spec) * c.a, c.a);
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
