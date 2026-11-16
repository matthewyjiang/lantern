//! Simulated spinning LiDAR on a street scene, in ROS conventions (REP 103): metres, z up.
//!
//! A 128-beam sensor drives down a straight road and takes full sweeps at evenly spaced poses
//! until the accumulated cloud holds at least the asked number of points. Returns carry
//! intensity from the surface's reflectivity and incidence angle, so lane paint reads bright
//! and asphalt dark, as on a real sensor. Deterministic: the same call gives the same cloud.

use bytemuck::{Pod, Zeroable};
use glam::Vec3;

/// One return, laid out for the GPU as a `vec4f`: position, then intensity in 0..=1.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
pub struct Point {
    pub pos: [f32; 3],
    pub intensity: f32,
}

/// The sensor's beam pattern, loosely an Ouster OS1-128.
#[derive(Clone, Copy, Debug)]
pub struct Sensor {
    pub beams: u32,
    pub azimuth_steps: u32,
    /// Lowest and highest beam elevation, radians.
    pub elevation: (f32, f32),
    pub max_range: f32,
    /// Mount height above the road, metres.
    pub height: f32,
}

impl Default for Sensor {
    fn default() -> Self {
        Sensor {
            beams: 128,
            azimuth_steps: 2048,
            elevation: ((-25.0f32).to_radians(), 15.0f32.to_radians()),
            max_range: 100.0,
            height: 1.8,
        }
    }
}

/// An accumulated scan: every sweep's returns in sweep, beam, azimuth order.
pub struct Cloud {
    pub points: Vec<Point>,
    /// Where the sensor was for each sweep.
    pub poses: Vec<Vec3>,
}

/// Sweeps until the cloud holds at least `target` points (whole sweeps, so usually a few more).
pub fn simulate(target: usize) -> Cloud {
    let scene = Scene::street();
    let sensor = Sensor::default();
    let mut points = Vec::with_capacity(target + 300_000);
    let mut poses = Vec::new();
    while points.len() < target {
        let pose = Vec3::new(-30.0 + 10.0 * poses.len() as f32, -2.0, sensor.height);
        points.extend(sweep(&scene, &sensor, pose, poses.len() as u32));
        poses.push(pose);
    }
    Cloud { points, poses }
}

/// One full revolution from `pose`, beams split across up to eight threads.
fn sweep(scene: &Scene, sensor: &Sensor, pose: Vec3, sweep_id: u32) -> Vec<Point> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(8) as u32;
    let per = sensor.beams.div_ceil(threads);
    std::thread::scope(|s| {
        let parts: Vec<_> = (0..threads)
            .map(|t| s.spawn(move || {
                let mut out = Vec::new();
                for beam in (t * per)..((t + 1) * per).min(sensor.beams) {
                    scan_beam(scene, sensor, pose, sweep_id, beam, &mut out);
                }
                out
            }))
            .collect();
        parts.into_iter().flat_map(|p| p.join().unwrap()).collect()
    })
}

fn scan_beam(scene: &Scene, sensor: &Sensor, pose: Vec3, sweep_id: u32, beam: u32, out: &mut Vec<Point>) {
    let (lo, hi) = sensor.elevation;
    let elevation = lo + (hi - lo) * beam as f32 / (sensor.beams - 1) as f32;
    let (se, ce) = elevation.sin_cos();
    for step in 0..sensor.azimuth_steps {
        let azimuth = std::f32::consts::TAU * step as f32 / sensor.azimuth_steps as f32;
        let (sa, ca) = azimuth.sin_cos();
        let dir = Vec3::new(ce * ca, ce * sa, se);
        let Some(hit) = scene.cast(pose, dir) else { continue };
        if hit.t > sensor.max_range {
            continue;
        }
        // ±1.5 cm range noise, the same for the same ray every run.
        let noise = 0.015 * unit_noise(sweep_id, beam, step);
        let cos_incidence = hit.normal.dot(-dir).abs();
        let intensity = (hit.reflectivity * (0.35 + 0.65 * cos_incidence)).clamp(0.0, 1.0);
        let p = pose + dir * (hit.t + noise);
        out.push(Point { pos: p.into(), intensity });
    }
}

/// Hash of a ray's id to -1..1.
fn unit_noise(a: u32, b: u32, c: u32) -> f32 {
    let mut h = (a as u64) << 42 ^ (b as u64) << 21 ^ c as u64;
    h = (h ^ (h >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    h = (h ^ (h >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    h ^= h >> 31;
    (h >> 40) as f32 / (1u64 << 23) as f32 - 1.0
}

#[derive(Clone, Copy, Debug)]
pub struct Hit {
    /// Distance along the (unit) ray, metres.
    pub t: f32,
    pub normal: Vec3,
    pub reflectivity: f32,
}

#[derive(Clone, Copy, Debug)]
enum Shape {
    /// Axis-aligned box.
    Cuboid { min: Vec3, max: Vec3 },
    /// Vertical cylinder standing on `base`. Side only: every top in the scene sits above the
    /// sensor or under a canopy, so no ray can reach a cap.
    Cylinder { base: Vec3, radius: f32, height: f32 },
    Sphere { centre: Vec3, radius: f32 },
}

#[derive(Clone, Copy, Debug)]
struct Object {
    shape: Shape,
    reflectivity: f32,
}

/// Everything a ray can hit besides the road plane at z = 0.
pub struct Scene {
    objects: Vec<Object>,
}

/// Half the road's width; it runs along x through the origin.
pub const ROAD_HALF_WIDTH: f32 = 4.0;
/// Spacing of the street poles along the road, metres.
pub const POLE_SPACING: f32 = 6.0;
const POLE_Y: f32 = 5.0;
const POLE_RADIUS: f32 = 0.09;

impl Scene {
    /// A straight road with poles every six metres on one side, trees on the other, a large
    /// building block and its annex behind the poles, and stacks of crates on both verges.
    pub fn street() -> Scene {
        let mut objects = Vec::new();
        let mut cuboid = |min: [f32; 3], max: [f32; 3], reflectivity| {
            objects.push(Object { shape: Shape::Cuboid { min: min.into(), max: max.into() }, reflectivity });
        };
        cuboid([-22.0, 9.0, 0.0], [18.0, 26.0, 16.0], 0.5); // building
        cuboid([22.0, 9.0, 0.0], [38.0, 20.0, 7.0], 0.55); // annex
        // Crate stacks: two side by side with one on top, 1.2 m cubes.
        for (x, y) in [(6.0, -6.6), (-15.0, 5.8), (29.0, -6.2)] {
            cuboid([x, y, 0.0], [x + 1.2, y + 1.2, 1.2], 0.6);
            cuboid([x + 1.3, y, 0.0], [x + 2.5, y + 1.2, 1.2], 0.6);
            cuboid([x + 0.6, y + 0.1, 1.2], [x + 1.8, y + 1.3, 2.4], 0.6);
        }
        let mut x = -48.0;
        while x <= 48.0 {
            objects.push(Object { shape: Shape::Cylinder { base: Vec3::new(x, POLE_Y, 0.0), radius: POLE_RADIUS, height: 7.0 }, reflectivity: 0.7 });
            x += POLE_SPACING;
        }
        for k in 0..12 {
            // Irregular spacing and size so the row doesn't read as copy-paste.
            let x = -46.0 + 8.3 * k as f32 + 1.7 * unit_noise(7, k, 0);
            let y = -8.5 + 0.6 * unit_noise(7, k, 1);
            let r = 1.6 + 0.35 * unit_noise(7, k, 2);
            objects.push(Object { shape: Shape::Cylinder { base: Vec3::new(x, y, 0.0), radius: 0.18, height: 3.0 }, reflectivity: 0.3 });
            objects.push(Object { shape: Shape::Sphere { centre: Vec3::new(x, y, 2.6 + r), radius: r }, reflectivity: 0.42 });
        }
        Scene { objects }
    }

    /// The nearest surface along the unit ray from `o` toward `d`, if any.
    pub fn cast(&self, o: Vec3, d: Vec3) -> Option<Hit> {
        let mut best = ground(o, d);
        for obj in &self.objects {
            let found = match obj.shape {
                Shape::Cuboid { min, max } => cuboid(o, d, min, max),
                Shape::Cylinder { base, radius, height } => cylinder(o, d, base, radius, height),
                Shape::Sphere { centre, radius } => sphere(o, d, centre, radius),
            };
            if let Some((t, normal)) = found
                && best.is_none_or(|b| t < b.t)
            {
                best = Some(Hit { t, normal, reflectivity: obj.reflectivity });
            }
        }
        best
    }
}

/// The road plane, painted: dashed centre line, solid edge lines, asphalt, then verges.
fn ground(o: Vec3, d: Vec3) -> Option<Hit> {
    if d.z >= 0.0 || o.z <= 0.0 {
        return None;
    }
    let t = -o.z / d.z;
    let p = o + d * t;
    let y = p.y.abs();
    let reflectivity = if y < 0.08 && p.x.rem_euclid(6.0) < 3.0 {
        0.95
    } else if (ROAD_HALF_WIDTH - 0.2..ROAD_HALF_WIDTH).contains(&y) {
        0.85
    } else if y < ROAD_HALF_WIDTH {
        0.12
    } else {
        0.2 // verge: dry grass and dirt
    };
    Some(Hit { t, normal: Vec3::Z, reflectivity })
}

fn cuboid(o: Vec3, d: Vec3, min: Vec3, max: Vec3) -> Option<(f32, Vec3)> {
    let inv = d.recip();
    let (t0, t1) = ((min - o) * inv, (max - o) * inv);
    let (near, far) = (t0.min(t1), t0.max(t1));
    let t_in = near.max_element();
    if t_in > far.min_element() || t_in <= 0.0 {
        return None;
    }
    // The entry face is on the axis whose slab was entered last.
    let axis = if t_in == near.x { Vec3::X } else if t_in == near.y { Vec3::Y } else { Vec3::Z };
    Some((t_in, -axis * d.dot(axis).signum()))
}

fn cylinder(o: Vec3, d: Vec3, base: Vec3, radius: f32, height: f32) -> Option<(f32, Vec3)> {
    let (ox, oy) = (o.x - base.x, o.y - base.y);
    let a = d.x * d.x + d.y * d.y;
    let b = ox * d.x + oy * d.y;
    let c = ox * ox + oy * oy - radius * radius;
    let disc = b * b - a * c;
    if a == 0.0 || disc < 0.0 {
        return None;
    }
    let t = (-b - disc.sqrt()) / a;
    let z = o.z + d.z * t;
    if t <= 0.0 || z < base.z || z > base.z + height {
        return None;
    }
    Some((t, Vec3::new(ox + d.x * t, oy + d.y * t, 0.0) / radius))
}

fn sphere(o: Vec3, d: Vec3, centre: Vec3, radius: f32) -> Option<(f32, Vec3)> {
    let oc = o - centre;
    let b = oc.dot(d);
    let disc = b * b - (oc.length_squared() - radius * radius);
    if disc < 0.0 {
        return None;
    }
    let t = -b - disc.sqrt();
    (t > 0.0).then(|| (t, (oc + d * t) / radius))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_beam_at_a_pole_returns_its_near_surface_and_paint_outshines_asphalt() {
        let scene = Scene::street();
        // From the lane centre straight across the road at the pole standing at x = 0.
        let hit = scene.cast(Vec3::new(0.0, -2.0, 1.8), Vec3::Y).unwrap();
        assert!((hit.t - (POLE_Y - POLE_RADIUS + 2.0)).abs() < 1e-4, "range {}", hit.t);
        assert!((hit.normal - -Vec3::Y).length() < 1e-4);

        let down = |x: f32, y: f32| scene.cast(Vec3::new(x, y, 1.8), -Vec3::Z).unwrap().reflectivity;
        assert!(down(1.0, 0.0) > 3.0 * down(1.0, -2.0), "centre-line dash vs asphalt");
        assert!(down(4.0, 0.0) < down(1.0, -2.0) + 1e-6, "the gap between dashes is asphalt");
    }

    #[test]
    fn simulate_reaches_the_target_in_whole_sweeps_and_repeats_exactly() {
        let a = simulate(1_000_000);
        assert!(a.points.len() >= 1_000_000);
        let sensor = Sensor::default();
        let per_sweep = sensor.beams as usize * sensor.azimuth_steps as usize;
        assert!(a.points.len() < 1_000_000 + per_sweep, "stops after the sweep that crossed the target");
        for (k, pose) in a.poses.iter().enumerate() {
            assert!(pose.y.abs() < ROAD_HALF_WIDTH, "sweep {k} taken off the road");
        }
        let b = simulate(1_000_000);
        assert!(a.points == b.points, "same scan every run");
    }
}
