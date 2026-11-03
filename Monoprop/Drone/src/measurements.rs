use core::f64::consts::FRAC_PI_2;

use Estimation::math::{diag, quat_to_rot, set_block3, skew, ComplexField};
use Estimation::{FullStateESEKF, Mat, Nominal, UpdateOutcome, Vector, GATE_1DOF, GATE_3DOF, M3, V3};

/// Where the lidar is, where it points, where the ground is, and which readings are usable.
#[derive(Clone, Copy, Debug)]
pub struct LidarConfig {
    pub offset: V3,     // m, body frame
    pub beam: V3,       // unit vector, body frame
    pub ground_z: f64,  // m, navigation frame
    pub min_range: f64, // m
    pub max_range: f64, // m
    pub max_tilt: f64,  // rad from straight down, below pi/2
}

struct Beam {
    r: M3,
    b: V3,
    ps_z: f64,
    d_z: f64,
}

/// None if the config is unusable or the beam is more than max_tilt from straight down; with Some,
/// |d_z| >= cos(max_tilt) > 0, so dividing by d_z is safe.
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
    if -d_z >= cos_max {
        Some(Beam { r, b, ps_z: x.p.z + (r * cfg.offset).z, d_z })
    } else {
        None
    }
}

fn in_limits(range: f64, cfg: &LidarConfig) -> bool {
    cfg.min_range.is_finite()
        && cfg.min_range >= 0.0
        && cfg.max_range.is_finite()
        && range >= cfg.min_range
        && range <= cfg.max_range
}

/// Predicted range along the beam (m).
pub fn lidar_prediction(x: &Nominal, cfg: &LidarConfig) -> Option<f64> {
    let g = beam(x, cfg)?;
    if !cfg.ground_z.is_finite() {
        return None;
    }
    Some((cfg.ground_z - g.ps_z) / g.d_z)
}

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

/// The ground height that makes `range` the predicted range at x (`cfg.ground_z` is not used).
pub fn lidar_ground_z(x: &Nominal, range: f64, cfg: &LidarConfig) -> Option<f64> {
    if !range.is_finite() || range <= 0.0 || !in_limits(range, cfg) {
        return None;
    }
    let g = beam(x, cfg)?;
    let z = g.ps_z + range * g.d_z;
    if z.is_finite() { Some(z) } else { None }
}

/// None, changing nothing, for a range the sensor can't give (outside the limits, beam too tilted, bad sigma).
/// A non-finite or non-positive range is counted in `rejected`.
pub fn update_lidar(f: &mut FullStateESEKF, range: f64, sigma: f64, cfg: &LidarConfig) -> Option<UpdateOutcome> {
    if !range.is_finite() || range <= 0.0 {
        f.rejected += 1;
        return Some(UpdateOutcome::RejectedNonFinite);
    }
    if !sigma.is_finite() || sigma <= 0.0 || !in_limits(range, cfg) {
        return None;
    }
    let predicted = lidar_prediction(&f.nominal_state, cfg)?;
    let h = lidar_jacobian(&f.nominal_state, cfg)?;
    let y = Vector::<1>::new(range - predicted);
    Some(f.update(&y, &h, &Mat::<1, 1>::new(sigma * sigma), GATE_1DOF))
}

/// Where the GPS antenna is (m, navigation frame).
pub fn gps_offset_prediction(x: &Nominal, antenna_offset: &V3) -> V3 {
    x.p + x.q * antenna_offset
}

pub fn gps_offset_jacobian(x: &Nominal, antenna_offset: &V3) -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 0, &M3::identity());
    set_block3(&mut h, 0, 6, &(-(quat_to_rot(&x.q) * skew(antenna_offset))));
    h
}

/// A fix z (m, east/north/up) taken at `antenna_offset` (m, body), with per-axis sigma (m).
/// None, changing nothing, if a sigma is not positive and finite or the offset is not finite.
pub fn update_gps_offset(f: &mut FullStateESEKF, z: &V3, sigma: &V3, antenna_offset: &V3) -> Option<UpdateOutcome> {
    let sigma_ok = sigma.iter().all(|s| s.is_finite() && *s > 0.0);
    if !sigma_ok || !antenna_offset.iter().all(|v| v.is_finite()) {
        return None;
    }
    let y = z - gps_offset_prediction(&f.nominal_state, antenna_offset);
    let h = gps_offset_jacobian(&f.nominal_state, antenna_offset);
    let r = diag([sigma.x * sigma.x, sigma.y * sigma.y, sigma.z * sigma.z]);
    Some(f.update(&y, &h, &r, GATE_3DOF))
}

#[cfg(test)]
mod tests {
    use super::*;
    use Estimation::full_state::inject;
    use Estimation::math::quat_exp;

    const EPS: f64 = 1e-6;

    fn states() -> [Nominal; 3] {
        let x = |p: V3, r: V3| Nominal { p, v: V3::new(0.3, -0.2, 0.1), q: quat_exp(&r), ab: V3::zeros(), wb: V3::zeros() };
        [
            x(V3::new(0.0, 0.0, 1.5), V3::zeros()),
            x(V3::new(1.0, -2.0, 3.0), V3::new(0.2, -0.1, 0.7)),
            x(V3::new(-0.5, 0.4, 0.8), V3::new(-0.3, 0.25, -1.2)),
        ]
    }

    fn step(j: usize, s: f64) -> Vector<15> {
        let mut d = Vector::<15>::zeros();
        d[j] = s;
        d
    }

    #[test]
    fn lidar_jacobian_matches_finite_difference() {
        let cfg = LidarConfig {
            offset: V3::new(0.05, -0.02, -0.1),
            beam: V3::new(0.1, -0.05, -1.0),
            ground_z: -0.3,
            min_range: 0.0,
            max_range: 50.0,
            max_tilt: 1.2,
        };
        let mut worst: f64 = 0.0;
        for x in states() {
            let h = lidar_jacobian(&x, &cfg).unwrap_or_else(|| Mat::from_element(f64::NAN));
            for j in 0..15 {
                let plus = lidar_prediction(&inject(&x, &step(j, EPS)), &cfg).unwrap_or(f64::NAN);
                let minus = lidar_prediction(&inject(&x, &step(j, -EPS)), &cfg).unwrap_or(f64::NAN);
                worst = worst.max((h[(0, j)] - (plus - minus) / (2.0 * EPS)).abs());
            }
        }
        assert!(worst < 1e-6, "worst H error {worst:e}");
    }

    #[test]
    fn gps_offset_jacobian_matches_finite_difference() {
        let offset = V3::new(0.12, -0.04, 0.3);
        let mut worst: f64 = 0.0;
        for x in states() {
            let h = gps_offset_jacobian(&x, &offset);
            for j in 0..15 {
                let plus = gps_offset_prediction(&inject(&x, &step(j, EPS)), &offset);
                let minus = gps_offset_prediction(&inject(&x, &step(j, -EPS)), &offset);
                worst = worst.max((h.column(j) - (plus - minus) / (2.0 * EPS)).amax());
            }
        }
        assert!(worst < 1e-6, "worst H error {worst:e}");
    }
}
