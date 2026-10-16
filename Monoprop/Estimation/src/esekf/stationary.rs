//! Updates for a vehicle known to be standing still (on the pad, before take-off). Use them only then:
//! they are pseudo-measurements of what "still" means, not readings of a sensor.
//!
//! - Zero velocity: the navigation-frame velocity is zero. h(x) = v, H = I in the velocity columns.
//!   Through the correlations `predict` builds up it also corrects tilt and accelerometer bias.
//! - Zero angular rate: the true rate is zero, so the gyro reads its bias plus noise. `predict` uses
//!   w = wm - wb, so h(x) = wb and the measurement is the raw gyro reading wm; H = I in the gyro-bias
//!   columns. The Earth's rotation (7.3e-5 rad/s) is ignored: far below one sample of BMI323 gyro noise.
//!   The same gyro sample also drives `predict`; that correlation is ignored, as is usual for this update
//!   (feeding it an average of several samples reduces it).
use crate::*;

/// H for the zero-velocity update: 3 x 15, identity in the velocity-error columns.
pub fn zero_velocity_jacobian() -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 3, &M3::identity());
    h
}

/// H for the zero angular-rate update: 3 x 15, identity in the gyro-bias-error columns.
pub fn zero_rate_jacobian() -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 12, &M3::identity());
    h
}

impl Esekf {
    /// The vehicle is standing still: velocity zero, with standard deviation sigma (m/s) per axis.
    /// Returns false, changing nothing, if sigma is not positive and finite (not counted in `rejected`),
    /// or if the 3-DOF gate rejects it (counted): e.g. the vehicle is actually moving.
    pub fn update_zero_velocity(&mut self, sigma: f64) -> bool {
        if !sigma.is_finite() || sigma <= 0.0 {
            return false;
        }
        let y = -self.x.v;
        self.update(&y, &zero_velocity_jacobian(), &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }

    /// The vehicle is not rotating: the raw gyro reading `gyro` (rad/s, body frame) is the gyro bias plus
    /// noise of standard deviation sigma (rad/s) per axis. Returns false, changing nothing, if sigma is
    /// not positive and finite (not counted in `rejected`), or if the reading is not finite or the 3-DOF
    /// gate rejects it (counted).
    pub fn update_zero_rate(&mut self, gyro: &V3, sigma: f64) -> bool {
        if !sigma.is_finite() || sigma <= 0.0 {
            return false;
        }
        let y = gyro - self.x.wb;
        self.update(&y, &zero_rate_jacobian(), &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }
}
