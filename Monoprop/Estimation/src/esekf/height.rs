//! Height updates: a barometer (zeroed on the pad) or a downward lidar over flat ground measures the
//! navigation-frame z of the vehicle. Like the template's `baro_prediction` / `baro_jacobian`
//! (Drone/src/esekf/full_state_esekf.rs), the only nonzero entry of H is d(height)/d(position error z).
//! Unlike GPS/UWB it needs no radio, so it keeps bounding vertical drift through a position outage.
//! For a lidar on a tilted vehicle, correct the range
//! by the tilt first (height = range * R[2][2]) or extend h and H with the attitude.
use crate::*;

/// Chi-squared threshold, 1 degree of freedom, 99.9 %.
pub const GATE_1DOF: f64 = 10.83;

/// Predicted height measurement: the nominal z.
pub fn height_prediction(x: &Nominal) -> f64 {
    x.p.z
}

/// H for a height measurement: 1 x 15, a single 1 at the position-error z column.
pub fn height_jacobian() -> Mat<1, 15> {
    let mut h = Mat::<1, 15>::zeros();
    h[(0, 2)] = 1.0;
    h
}

impl Esekf {
    /// Height measurement z (m, navigation-frame z) with standard deviation sigma.
    /// Returns false if it was rejected (NaN, or an outlier by the 1-DOF gate).
    pub fn update_height(&mut self, z: f64, sigma: f64) -> bool {
        let y = Vector::<1>::new(z - height_prediction(&self.x));
        self.update(&y, &height_jacobian(), &Mat::<1, 1>::new(sigma * sigma), GATE_1DOF)
    }
}
