//! Lidar range updates: a downward lidar measures the distance ALONG ITS BEAM to flat ground, not height.
//! The prediction uses the attitude as well as the position, so the tilt correction (and how uncertain it is) is part of the update.
//!  height.rs is the simpler model for a sensor that reports height directly.
//!
//! Model, with R the attitude matrix, o the sensor's position and b its beam direction (body frame):
//!   sensor position p_s = p + R o, beam d = R b, ground: the plane z = ground_z,
//!   range r = (ground_z - p_s.z) / d.z.
//! Jacobian w.r.t. the error state (attitude error in body axes, R <- R (I + skew(dtheta))):
//!   d r / d(position) = (0, 0, -1/d.z),   d r / d(attitude) = e3^T R skew(o + r b) / d.z.
//! Level, pointing straight down, with o = 0 this is exactly the height model: r = p.z - ground_z, H = e3^T.
use crate::*;

/// Where the lidar is, where it points, where the ground is, and which readings are usable.
#[derive(Clone, Copy, Debug)]
pub struct LidarConfig {
    pub offset: V3,     // sensor position in the body frame (m)
    pub beam: V3,       // beam direction in the body frame (unit vector)
    pub ground_z: f64,  // height of the flat ground in the navigation frame (m)
    pub min_range: f64, // readings below this are not used (m)
    pub max_range: f64, // readings beyond this are not used (m)
    pub max_tilt: f64,  // largest angle of the beam from straight down that is used (rad), below pi/2
}

/// The beam in the navigation frame, when it can be used.
struct Beam {
    r: M3,     // attitude matrix (body -> navigation)
    b: V3,     // unit beam direction, body frame
    ps_z: f64, // z of the sensor, navigation frame
    d_z: f64,  // z of the unit beam direction, navigation frame (negative: pointing down)
}

/// None if the configuration is not usable (non-finite values, a zero beam, max_tilt outside [0, pi/2))
/// or the beam is more than max_tilt from straight down. With Some, |d_z| >= cos(max_tilt) > 0, so
/// dividing by d_z is safe.
fn beam(x: &Nominal, cfg: &LidarConfig) -> Option<Beam> {
    let norm = cfg.beam.norm();
    let cos_max = cfg.max_tilt.cos();
    let usable = cfg.offset.iter().chain(cfg.beam.iter()).all(|v| v.is_finite())
        && norm.is_finite()
        && norm > 0.0
        && cfg.max_tilt >= 0.0
        && cfg.max_tilt < FRAC_PI_2 // compared as an angle: cos(pi/2) is not exactly 0 in floating point
        && cos_max > 0.0;
    if !usable {
        return None;
    }
    let r = quat_to_rot(&x.q);
    let b = cfg.beam / norm;
    let d_z = (r * b).z;
    // cos(tilt) = -d_z: the angle between the beam and straight down (0, 0, -1)
    if -d_z >= cos_max {
        Some(Beam { r, b, ps_z: x.p.z + (r * cfg.offset).z, d_z })
    } else {
        None
    }
}

/// Whether a (finite, positive) range is inside the configured limits.
fn in_limits(range: f64, cfg: &LidarConfig) -> bool {
    cfg.min_range.is_finite()
        && cfg.min_range >= 0.0
        && cfg.max_range.is_finite()
        && range >= cfg.min_range
        && range <= cfg.max_range
}

/// Predicted range along the beam (m). None if the beam can't be used (see `LidarConfig::max_tilt`).
pub fn lidar_prediction(x: &Nominal, cfg: &LidarConfig) -> Option<f64> {
    let g = beam(x, cfg)?;
    if !cfg.ground_z.is_finite() {
        return None;
    }
    Some((cfg.ground_z - g.ps_z) / g.d_z)
}

/// H for a range measurement: 1 x 15, non-zero at the position-error z column and the attitude columns.
pub fn lidar_jacobian(x: &Nominal, cfg: &LidarConfig) -> Option<Mat<1, 15>> {
    let g = beam(x, cfg)?;
    let range = lidar_prediction(x, cfg)?;
    let mut h = Mat::<1, 15>::zeros();
    h[(0, 2)] = -1.0 / g.d_z;
    // e3^T R skew(a) = -(skew(a) R^T e3)^T, because skew(a)^T = -skew(a)
    let att = -(skew(&(cfg.offset + g.b * range)) * (g.r.transpose() * V3::z())) / g.d_z;
    h.fixed_view_mut::<1, 3>(0, 6).copy_from(&att.transpose());
    Some(h)
}

/// The ground height that makes `range` the predicted range at state x: call it once at start-up with a
/// reading taken on the pad and store the result in `LidarConfig::ground_z`, so the pad reads as the ground.
/// (`cfg.ground_z` itself is not used.) None if the reading or the beam can't be used.
pub fn lidar_ground_z(x: &Nominal, range: f64, cfg: &LidarConfig) -> Option<f64> {
    if !range.is_finite() || range <= 0.0 || !in_limits(range, cfg) {
        return None;
    }
    let g = beam(x, cfg)?;
    let z = g.ps_z + range * g.d_z;
    if z.is_finite() { Some(z) } else { None }
}

impl Esekf {
    /// Lidar range along the beam (m) with standard deviation sigma (m).
    /// Returns false, changing nothing, if the range is not a positive finite number (counted in
    /// `rejected`, like a bad magnetometer reading); if sigma is not positive and finite, the range is
    /// outside [min_range, max_range] or the beam is more than max_tilt from straight down (not counted:
    /// these are readings the sensor can't give, not outliers); or if the 1-DOF gate rejects it.
    pub fn update_lidar(&mut self, range: f64, sigma: f64, cfg: &LidarConfig) -> bool {
        if !range.is_finite() || range <= 0.0 {
            self.rejected += 1;
            return false;
        }
        if !sigma.is_finite() || sigma <= 0.0 || !in_limits(range, cfg) {
            return false;
        }
        let (Some(predicted), Some(h)) = (lidar_prediction(&self.x, cfg), lidar_jacobian(&self.x, cfg)) else {
            return false;
        };
        let y = Vector::<1>::new(range - predicted);
        self.update(&y, &h, &Mat::<1, 1>::new(sigma * sigma), GATE_1DOF)
    }
}
