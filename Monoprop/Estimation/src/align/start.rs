use crate::*;

/// Initial uncertainty for an air start: position from one GPS fix, velocity unknown (a few m/s),
/// attitude from TRIAD (a few degrees), biases unknown up to their datasheet limits.
pub fn initial_covariance(sigma_gps: f64) -> Mat<15, 15> {
    let s2 = sigma_gps * sigma_gps;
    diag([
        s2, s2, s2,
        4.0, 4.0, 4.0,
        0.003, 0.003, 0.003,
        0.04, 0.04, 0.04,
        1e-3, 1e-3, 1e-3,
    ])
}

/// Build the filter from the first IMU sample, magnetometer reading and GPS fix.
/// None if TRIAD can't define an attitude from these readings, or the fix isn't finite: try the next ones.
pub fn align(accel: &V3, mag: &V3, gps: &V3, spec: ImuSpec, sigma_gps: f64) -> Option<Esekf> {
    if !gps.iter().all(|v| v.is_finite()) {
        return None;
    }
    let q = triad(accel, mag, &mag_reference())?;
    let x = Nominal { p: *gps, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() };
    Some(Esekf::new(x, initial_covariance(sigma_gps), spec))
}
