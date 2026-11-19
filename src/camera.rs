//! Orbit camera for the 3D viewport, in ROS conventions (REP 103): right-handed, z up.

use glam::{Mat4, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitCamera {
    /// The point the camera orbits and looks at, in metres.
    pub target: Vec3,
    /// Heading of the eye around the target, radians about +z (0 puts the eye on +x).
    pub yaw: f32,
    /// Elevation of the eye above the target's xy plane, radians.
    pub pitch: f32,
    /// Eye-to-target distance, metres.
    pub distance: f32,
    /// Vertical field of view, radians.
    pub fov_y: f32,
}

const RAD_PER_PT: f32 = 0.006;
const PITCH_LIMIT: f32 = 89.0 * std::f32::consts::PI / 180.0;
const MIN_DISTANCE: f32 = 0.05;
const MAX_DISTANCE: f32 = 5000.0;
const NEAR: f32 = 0.01;

impl Default for OrbitCamera {
    fn default() -> Self {
        OrbitCamera {
            target: Vec3::ZERO,
            yaw: (-140.0f32).to_radians(),
            pitch: 32.0f32.to_radians(),
            distance: 38.0,
            fov_y: 50.0f32.to_radians(),
        }
    }
}

impl OrbitCamera {
    /// How many units (points or pixels) one metre spans at unit depth in a viewport
    /// `viewport_h` of those units tall; divide by the depth for a metre there.
    pub fn focal(&self, viewport_h: f32) -> f32 {
        viewport_h * 0.5 / (self.fov_y * 0.5).tan()
    }

    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.target + self.distance * Vec3::new(cp * cy, cp * sy, sp)
    }

    /// Orbits by a pointer move of (`dx`, `dy`) points: right swings the eye left around the
    /// target (the scene turns with the pointer), down raises it.
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * RAD_PER_PT;
        self.pitch = (self.pitch + dy * RAD_PER_PT).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    /// Slides the target in the view plane so the world under the pointer follows a move of
    /// (`dx`, `dy`) points in a viewport `viewport_h` points tall.
    pub fn pan(&mut self, dx: f32, dy: f32, viewport_h: f32) {
        let metres_per_pt = 2.0 * self.distance * (self.fov_y * 0.5).tan() / viewport_h.max(1.0);
        let forward = (self.target - self.eye()).normalize();
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        self.target += (-right * dx + up * dy) * metres_per_pt;
    }

    /// Multiplies the distance by `factor` (below 1 moves closer), within sane bounds.
    pub fn zoom(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(MIN_DISTANCE, MAX_DISTANCE);
    }

    /// World → clip space with reversed, infinite depth (near maps to 1, infinity to 0): clear
    /// depth to 0 and test `Greater`.
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let proj = Mat4::perspective_infinite_reverse_rh(self.fov_y, aspect, NEAR);
        proj * Mat4::look_at_rh(self.eye(), self.target, Vec3::Z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where `p` lands in a `w`×`h` point viewport, from its centre, y down.
    fn screen(cam: &OrbitCamera, p: Vec3, w: f32, h: f32) -> (f32, f32) {
        let c = cam.view_proj(w / h) * p.extend(1.0);
        (c.x / c.w * w * 0.5, -c.y / c.w * h * 0.5)
    }

    #[test]
    fn pan_drags_the_world_under_the_pointer() {
        let mut cam = OrbitCamera::default();
        let (w, h) = (900.0, 600.0);
        let anchor = cam.target;
        cam.pan(40.0, -25.0, h);
        let (x, y) = screen(&cam, anchor, w, h);
        assert!((x - 40.0).abs() < 0.01 && (y + 25.0).abs() < 0.01, "anchor moved to ({x}, {y})");
    }

    #[test]
    fn orbit_keeps_the_target_centred_and_never_flips_over_the_pole() {
        let mut cam = OrbitCamera::default();
        cam.orbit(120.0, 10_000.0);
        assert!(cam.pitch <= PITCH_LIMIT);
        let (x, y) = screen(&cam, cam.target, 900.0, 600.0);
        assert!(x.abs() < 1e-3 && y.abs() < 1e-3);
    }
}
